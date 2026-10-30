"""Native KB4/FISHEYE624 geometry. Transforms act on column vectors."""

import json
from pathlib import Path

import numpy as np

from ego_hand_benchmark.geometry.fisheye import project, unproject, validate_camera
from ego_hand_benchmark.geometry.hot3d_camera import transform_matrix


def validate_rotation(rotation):
    r = np.asarray(rotation, dtype=np.float64)
    if r.shape[-2:] != (3, 3) or not np.isfinite(r).all():
        raise ValueError("Expected finite rotation matrices")
    if not np.allclose(r.swapaxes(-1, -2) @ r, np.eye(3), atol=2e-5) or not np.allclose(np.linalg.det(r), 1, atol=2e-5):
        raise ValueError("Rotation must belong to SO(3)")
    return r


def validate_transform(transform):
    """X_left = R X_right + t; translation MUST already be in metres.

    The 1 mm–2 m baseline guard catches common mm/m mistakes; it cannot
    infer units or detect every geometrically plausible reversed transform.
    """
    if isinstance(transform, dict):
        t = np.eye(4)
        t[:3, :3] = validate_rotation(transform["R"])
        t[:3, 3] = transform["t_m"]
    else:
        t = np.asarray(transform, dtype=np.float64)
    if t.shape != (4, 4) or not np.isfinite(t).all() or not np.allclose(t[3], [0, 0, 0, 1]):
        raise ValueError("Expected finite homogeneous T_left_right")
    validate_rotation(t[:3, :3])
    if not 0.001 <= np.linalg.norm(t[:3, 3]) <= 2:
        raise ValueError("Stereo baseline outside [0.001, 2] metres; check units")
    return t


def valid_pixels(pixels, camera):
    """Valid pixel centres within the calibrated rectangular image domain."""
    validate_camera(camera)
    uv = np.asarray(pixels)
    return (np.isfinite(uv).all(axis=-1) & (uv[..., 0] >= 0)
            & (uv[..., 0] <= camera["image_width"] - 1)
            & (uv[..., 1] >= 0) & (uv[..., 1] <= camera["image_height"] - 1))


def pixel_to_unit_ray(pixels, camera):
    rays = unproject(pixels, camera)
    return np.where(valid_pixels(pixels, camera)[..., None], rays, np.nan)


def load_calibration(path):
    """Load task-book KB4 JSON or native benchmark camera dictionaries.

    Returns (left_camera, right_camera, T_left_right).
    """
    data = json.loads(Path(path).read_text()) if not isinstance(path, dict) else path
    cameras = []
    for side in ("left", "right"):
        c = data[side]
        if "K" in c:
            if data.get("model") not in ("opencv_fisheye", "KB4"):
                raise ValueError("K/D example schema supports only explicitly named KB4")
            c = {"model": "KB4", "camera_matrix": {"matrix_3x3": c["K"]},
                 "distortion_coefficients": c["D"], "image_width": c["width"], "image_height": c["height"]}
        validate_camera(c)
        cameras.append(c)
    return *cameras, validate_transform(data["T_left_right"])


def hot3d_stereo_transform(left_camera, right_camera):
    """Use static device extrinsics, never moving world poses."""
    left = transform_matrix(left_camera["hot3d_calibration"]["T_device_from_camera"])
    right = transform_matrix(right_camera["hot3d_calibration"]["T_device_from_camera"])
    return validate_transform(np.linalg.inv(left) @ right)
