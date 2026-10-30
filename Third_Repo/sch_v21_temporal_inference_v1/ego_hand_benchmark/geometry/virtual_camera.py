"""Calibrated local pinhole views and weak-perspective camera recovery for WiLoR."""

import cv2
import numpy as np

from .fisheye import project, unproject


class ViewUnavailable(ValueError):
    pass


def rotation_to_ray(ray):
    z = np.asarray(ray, dtype=np.float64)
    if not np.isfinite(z).all() or np.linalg.norm(z) < 1e-12:
        raise ViewUnavailable("center_outside_camera_invertible_domain")
    z /= np.linalg.norm(z)
    x = np.cross([0.0, 1.0, 0.0], z)
    if np.linalg.norm(x) < 1e-6:
        x = np.cross([0.0, 0.0, 1.0], z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.stack([x, y, z])


def make_view(image, box, source_camera, size=512, rescale_factor=2.0, max_fov=160.0):
    """The view encloses WiLoR's padded 192:256 crop; padding is applied by its dataset once."""
    x0, y0, x1, y1 = np.asarray(box, dtype=np.float64)
    if x1 <= x0 or y1 <= y0:
        raise ViewUnavailable("invalid_detection_box")
    rotation = rotation_to_ray(unproject([(x0 + x1) / 2, (y0 + y1) / 2], source_camera))
    x, y = np.linspace(x0, x1, 17), np.linspace(y0, y1, 17)
    boundary = np.concatenate(
        [
            np.stack([x, np.full_like(x, y0)], axis=1),
            np.stack([x, np.full_like(x, y1)], axis=1),
            np.stack([np.full_like(y, x0), y], axis=1),
            np.stack([np.full_like(y, x1), y], axis=1),
        ]
    )
    rays = unproject(boundary, source_camera) @ rotation.T
    if not np.isfinite(rays).all():
        raise ViewUnavailable("box_outside_camera_invertible_domain")
    if np.any(rays[:, 2] <= 1e-4):
        raise ViewUnavailable("detection_exceeds_pinhole_hemisphere")
    xy = rays[:, :2] / rays[:, 2:]
    low, high = xy.min(axis=0), xy.max(axis=0)
    center = (low + high) / 2
    crop_side = rescale_factor * max((high[0] - low[0]) / 0.75, high[1] - low[1])
    half_tangent = max(np.max(abs(center) + crop_side / 2) * 1.02, np.tan(np.deg2rad(4)))
    if half_tangent > np.tan(np.deg2rad(max_fov / 2)):
        raise ViewUnavailable("padded_crop_exceeds_virtual_fov")
    focal = (size - 1) / (2 * half_tangent)
    principal = (size - 1) / 2
    camera = {
        "model": "PINHOLE",
        "image_width": size,
        "image_height": size,
        "camera_matrix": {"matrix_3x3": [[focal, 0, principal], [0, focal, principal], [0, 0, 1]]},
        "distortion_coefficients": [],
    }
    u, v = np.meshgrid(np.arange(size), np.arange(size))
    virtual_rays = np.stack(
        [(u - principal) / focal, (v - principal) / focal, np.ones_like(u)], axis=-1
    )
    source_pixels = project(virtual_rays @ rotation, source_camera)
    inside = np.isfinite(source_pixels).all(axis=-1)
    inside &= (source_pixels >= 0).all(axis=-1)
    inside &= (
        source_pixels < [source_camera["image_width"] - 1, source_camera["image_height"] - 1]
    ).all(axis=-1)
    maps = np.where(inside[..., None], source_pixels, -1).astype(np.float32)
    view = cv2.remap(
        image, maps[..., 0], maps[..., 1], cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT
    )
    view_box = np.r_[focal * low + principal, focal * high + principal]
    return (
        view,
        view_box,
        {
            "K_virtual": np.array(camera["camera_matrix"]["matrix_3x3"]),
            "R_virtual_from_source": rotation,
            "virtual_box_xyxy": view_box,
            "virtual_fov_deg": float(np.rad2deg(2 * np.arctan(half_tangent))),
            "sampling_in_source_fraction": float(inside.mean()),
        },
    )


def calibrated_translation(pred_cam, box_center, box_size, intrinsic, is_right):
    """Recover metric translation from WiLoR's scale using the actual virtual pixel geometry."""
    cam = np.asarray(pred_cam, dtype=np.float64).copy()
    k = np.asarray(intrinsic)
    if not np.isclose(k[0, 0], k[1, 1], rtol=1e-6):
        raise ValueError("WiLoR scale recovery requires square pixels in the virtual view")
    if cam[0] <= 0 or box_size <= 0 or not np.isfinite(cam).all():
        raise ViewUnavailable("invalid_predicted_camera")
    cam[1] *= 2 * int(is_right) - 1
    bs = float(box_size) * cam[0]
    return np.array(
        [
            cam[1] + 2 * (box_center[0] - k[0, 2]) / bs,
            cam[2] + 2 * (box_center[1] - k[1, 2]) / bs,
            2 * k[0, 0] / bs,
        ]
    )
