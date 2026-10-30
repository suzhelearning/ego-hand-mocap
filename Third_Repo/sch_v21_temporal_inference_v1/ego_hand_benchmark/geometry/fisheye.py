"""Source-image projection for the camera models supported by benchmark v1."""

from functools import lru_cache

import cv2
import numpy as np


@lru_cache(maxsize=32)
def kb4_theta_limit(coefficients):
    """Use only the radial branch connected to the optical axis, before its first fold."""
    k1, k2, k3, k4 = coefficients
    roots = np.roots([9 * k4, 7 * k3, 5 * k2, 3 * k1, 1])
    turning = [np.sqrt(r.real) for r in roots if abs(r.imag) < 1e-9 and r.real > 0]
    return min([np.pi, *turning]) - 1e-6


def validate_camera(camera):
    k = np.asarray(camera["camera_matrix"]["matrix_3x3"], dtype=np.float64)
    model = camera["model"]
    distortion = np.asarray(camera.get("distortion_coefficients", []), dtype=np.float64)
    counts = {"PINHOLE": (0,), "KB4": (4,), "FISHEYE624": (12,), "RATIONAL_POLYNOMIAL": (4, 5, 8, 12, 14)}
    if model not in counts or distortion.size not in counts[model]:
        raise ValueError(f"Unsupported camera model/distortion: {model}, {distortion.size}")
    if k.shape != (3, 3) or not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError("Invalid camera intrinsics")
    if not np.allclose(k[2], [0, 0, 1]) or not np.allclose([k[0, 1], k[1, 0]], 0):
        raise ValueError("Only zero-skew camera matrices are supported")
    if not np.isfinite(distortion).all() or min(camera["image_width"], camera["image_height"]) <= 0:
        raise ValueError("Invalid camera dimensions or distortion")
    return k, distortion


def project(joints, camera):
    k, distortion = validate_camera(camera)
    joints = np.asarray(joints, dtype=np.float64)
    shape = joints.shape[:-1]
    points = joints.reshape(-1, 3)
    valid = np.isfinite(points).all(axis=1) & (np.linalg.norm(points, axis=1) > 1e-12)
    if camera["model"] not in ("KB4", "FISHEYE624"):
        valid &= points[:, 2] > 0
    pixels = np.full((len(points), 2), np.nan)
    if valid.any():
        xyz = np.ascontiguousarray(points[valid, None, :])
        if camera["model"] == "FISHEYE624":
            from .hot3d_camera import project_fisheye624
            uv = project_fisheye624(points[valid], k, distortion)
        elif camera["model"] == "KB4":
            rays = points[valid]
            radius = np.linalg.norm(rays[:, :2], axis=1)
            theta = np.arctan2(radius, rays[:, 2])
            k1, k2, k3, k4 = distortion
            t2 = theta * theta
            radial = theta * (1 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4))))
            scale = np.divide(radial, radius, out=np.zeros_like(radius), where=radius > 1e-12)
            uv = rays[:, :2] * scale[:, None] * [k[0, 0], k[1, 1]] + k[:2, 2]
            uv[(radius < 1e-12) & (rays[:, 2] < 0)] = np.nan
            uv[theta >= kb4_theta_limit(tuple(distortion))] = np.nan
        else:
            uv, _ = cv2.projectPoints(
                xyz, np.zeros(3), np.zeros(3), k, distortion if distortion.size else None
            )
        pixels[valid] = uv.reshape(-1, 2)
    return pixels.reshape(*shape, 2)


def unproject_kb4(pixels, camera):
    """Invert the monotonic KB4 radial polynomial, including rays beyond 90 degrees."""
    k, distortion = validate_camera(camera)
    if camera["model"] != "KB4":
        raise ValueError("Virtual WiLoR views currently require KB4 source calibration")
    pixels = np.asarray(pixels, dtype=np.float64)
    xy = (pixels - k[:2, 2]) / [k[0, 0], k[1, 1]]
    radius = np.linalg.norm(xy, axis=-1)
    k1, k2, k3, k4 = distortion

    def polynomial(theta):
        t2 = theta * theta
        return theta * (1 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4))))

    limit = kb4_theta_limit(tuple(distortion))
    valid = np.isfinite(radius) & (radius < polynomial(limit))
    low, high = np.zeros_like(radius), np.full_like(radius, limit)
    for _ in range(48):
        mid = (low + high) * 0.5
        small = polynomial(mid) < radius
        low = np.where(small, mid, low)
        high = np.where(small, high, mid)
    theta = (low + high) * 0.5
    scale = np.divide(np.sin(theta), radius, out=np.ones_like(radius), where=radius > 1e-12)
    rays = np.concatenate([xy * scale[..., None], np.cos(theta)[..., None]], axis=-1)
    return np.where(valid[..., None], rays, np.nan)


def unproject(pixels, camera):
    """Return unit optical-camera rays, preserving the input leading dimensions."""
    k, distortion = validate_camera(camera)
    pixels = np.asarray(pixels, dtype=np.float64)
    if camera["model"] == "KB4":
        return unproject_kb4(pixels, camera)
    if camera["model"] == "FISHEYE624":
        from .hot3d_camera import unproject_fisheye624
        return unproject_fisheye624(pixels, k, distortion)
    xy = cv2.undistortPoints(pixels.reshape(-1, 1, 2), k, distortion if distortion.size else None)
    rays = np.concatenate([xy.reshape(*pixels.shape[:-1], 2), np.ones((*pixels.shape[:-1], 1))], axis=-1)
    return rays / np.linalg.norm(rays, axis=-1, keepdims=True)


def pinhole_camera(width, height, focal):
    return {
        "model": "PINHOLE",
        "image_width": width,
        "image_height": height,
        "camera_matrix": {"matrix_3x3": [[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1]]},
        "distortion_coefficients": [],
    }
