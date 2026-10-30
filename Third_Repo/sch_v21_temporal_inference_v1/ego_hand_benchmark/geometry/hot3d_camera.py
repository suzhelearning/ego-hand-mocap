"""NumPy FISHEYE624, audited against Meta hand_tracking_toolkit c992c88493d3a5f4153a56e9001424e63093b0f6.

Equations follow Apache-2.0 camera_distortion.OVR624Distortion: radial
distortion precedes tangential/thin-prism (p1 affects x, p2 affects y).
Inverse is restricted to the radial branch connected to the optical axis.
"""

from functools import lru_cache

import numpy as np


def transform_matrix(pose):
    """HOT3D quaternion_wxyz + translation_xyz, in metres."""
    q = np.asarray(pose["quaternion_wxyz"], dtype=np.float64)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 1e-5:
        raise ValueError("Invalid unit quaternion")
    w, x, y, z = q / np.linalg.norm(q)
    out = np.eye(4)
    out[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                    [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                    [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    out[:3, 3] = pose["translation_xyz"]
    if not np.isfinite(out).all():
        raise ValueError("Invalid rigid transform")
    return out


def camera_from_calibration(calibration):
    if calibration["projection_model_type"] != "CameraModelType.FISHEYE624":
        raise ValueError("Expected official FISHEYE624 calibration")
    params = calibration["projection_params"]
    if len(params) != 15:
        raise ValueError("Expected f,cx,cy and 12 distortion coefficients")
    f, cx, cy = params[:3]
    return {"model": "FISHEYE624", "image_width": calibration["image_width"],
            "image_height": calibration["image_height"],
            "camera_matrix": {"matrix_3x3": [[f, 0, cx], [0, f, cy], [0, 0, 1]]},
            "distortion_coefficients": params[3:], "hot3d_calibration": calibration}


@lru_cache(maxsize=64)
def theta_limit(coefficients):
    roots = np.polynomial.polynomial.polyroots([1] + [(2*i+3)*v for i, v in enumerate(coefficients[:6])])
    return min([np.pi, *[np.sqrt(r.real) for r in roots if abs(r.imag) < 1e-9 and r.real > 0]]) - 1e-7


def radial(theta, coefficients):
    return theta * np.polynomial.polynomial.polyval(theta*theta, [1, *coefficients[:6]])


def tangential(uv, coefficients):
    p1, p2, s1, s2, s3, s4 = coefficients[6:]
    x, y = uv[..., 0], uv[..., 1]
    r2 = x*x + y*y
    return np.stack([x + 2*p2*x*y + p1*(r2+2*x*x) + s1*r2+s2*r2*r2,
                     y + 2*p1*x*y + p2*(r2+2*y*y) + s3*r2+s4*r2*r2], axis=-1)


def project_fisheye624(points, k, coefficients):
    radius = np.linalg.norm(points[..., :2], axis=-1)
    theta = np.arctan2(radius, points[..., 2])
    scale = np.divide(radial(theta, coefficients), radius, out=np.zeros_like(radius), where=radius > 1e-12)
    uv = tangential(points[..., :2] * scale[..., None], coefficients)
    uv = uv * [k[0, 0], k[1, 1]] + k[:2, 2]
    valid = (theta < theta_limit(tuple(coefficients))) & (np.linalg.norm(points, axis=-1) > 1e-12)
    return np.where(valid[..., None], uv, np.nan)


def unproject_fisheye624(pixels, k, coefficients):
    target = (pixels - k[:2, 2]) / [k[0, 0], k[1, 1]]
    uv = target.copy()
    # Analytic 2x2 Newton solve for the tangential and thin-prism component.
    p1, p2, s1, s2, s3, s4 = coefficients[6:]
    for _ in range(15):
        x, y = uv[..., 0], uv[..., 1]
        r2 = x*x+y*y
        a = 1+2*p2*y+6*p1*x+2*x*(s1+2*s2*r2)
        b = 2*p2*x+2*p1*y+2*y*(s1+2*s2*r2)
        c = 2*p1*y+2*p2*x+2*x*(s3+2*s4*r2)
        d = 1+2*p1*x+6*p2*y+2*y*(s3+2*s4*r2)
        residual = tangential(uv, coefficients)-target
        det = a*d-b*c
        uv -= np.stack([(d*residual[..., 0]-b*residual[..., 1])/det,
                        (a*residual[..., 1]-c*residual[..., 0])/det], axis=-1)
    radius = np.linalg.norm(uv, axis=-1)
    limit = theta_limit(tuple(coefficients))
    valid = np.isfinite(target).all(axis=-1) & (radius < radial(limit, coefficients))
    valid &= np.linalg.norm(tangential(uv, coefficients)-target, axis=-1) < 1e-9
    low, high = np.zeros_like(radius), np.full_like(radius, limit)
    for _ in range(52):
        mid = (low+high)/2
        small = radial(mid, coefficients) < radius
        low, high = np.where(small, mid, low), np.where(small, high, mid)
    theta = (low+high)/2
    scale = np.divide(np.sin(theta), radius, out=np.ones_like(radius), where=radius > 1e-12)
    rays = np.concatenate([uv*scale[..., None], np.cos(theta)[..., None]], axis=-1)
    return np.where(valid[..., None], rays, np.nan)
