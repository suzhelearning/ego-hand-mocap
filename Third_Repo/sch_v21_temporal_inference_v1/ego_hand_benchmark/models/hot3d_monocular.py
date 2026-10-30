"""HOT3D monocular presets and the benchmark-owned Isaac Capture camera bridge."""

import numpy as np

from ego_hand_benchmark.geometry.fisheye import project, unproject, validate_camera


class BenchmarkCamera:
    """Structural Isaac Capture CameraModel using the benchmark's source geometry."""

    def __init__(self, camera):
        self.K, _ = validate_camera(camera)
        self.camera = camera
        self.width = camera["image_width"]
        self.height = camera["image_height"]
        self.model_name = camera["model"]

    def project(self, rays):
        return project(rays, self.camera)

    def unproject(self, pixels):
        return unproject(pixels, self.camera)


def hot3d_method_config(method):
    """Return independent raw per-frame configs accepted by the existing adapters."""
    common = {"id": method, "device": "cuda", "warmup_frames": 0, "batch_size": 8}
    if method == "egohand_raw":
        return {**common, "adapter": "egohand", "smooth": False, "tracking": False, "bbox_smoothing": False,
                "dtype": "bfloat16", "frame_batch_size": 16, "detector_batch_size": 8}
    if method in ("wilor_calibrated_raw", "anyhand_wilor_calibrated_raw"):
        config = {**common, "adapter": "wilor", "camera_mode": "calibrated_virtual", "detector_confidence": 0.3,
                  "detector_iou": 0.7, "rescale_factor": 2.0, "virtual_size": 512,
                  "max_virtual_fov_deg": 160.0, "preview_frames": []}
        if method.startswith("anyhand_"):
            from ego_hand_benchmark.models.anyhand_adapter import DEFAULT_CHECKPOINT, DEFAULT_CONFIG

            config.update(adapter="anyhand", backend="wilor", detector_iou=0.3,
                          checkpoint=str(DEFAULT_CHECKPOINT), model_config=str(DEFAULT_CONFIG))
        return config
    raise ValueError(f"Unsupported HOT3D monocular method: {method}")


def add_input_view_flags(result, camera):
    """Pixel bounds are observable; ground-truth visibility is not an inference flag."""
    pixels = result["keypoints21_px"]
    result["input_view_joint_valid"] = (
        result["projection_valid"] & np.isfinite(pixels).all(axis=-1)
        & (pixels >= 0).all(axis=-1)
        & (pixels < [camera["image_width"], camera["image_height"]]).all(axis=-1)
    )
