"""Flat frame-hand records; scalar headers apply to every record in an NPZ."""

import numpy as np

from .skeleton import HANDS, JOINT_NAMES, JOINT_SET

HEADERS = (
    "schema_version",
    "sequence_id",
    "camera_id",
    "coordinate_frame",
    "timestamp_clock",
    "joint_set",
    "source_model",
    "confidence_source",
    "keypoints_source",
)


def empty_predictions(sequence, model, confidence_source="unavailable"):
    frames = sequence["frames"]
    n = len(frames) * len(HANDS)
    result = {
        "schema_version": np.array("1.0"),
        "sequence_id": np.array(sequence["sequence_id"]),
        "camera_id": np.array(sequence["camera_id"]),
        "coordinate_frame": np.array(sequence["coordinate_frame"]),
        "timestamp_clock": np.array(sequence["timestamp_clock"]),
        "joint_set": np.array(JOINT_SET),
        "joint_names": np.array(JOINT_NAMES),
        "source_model": np.array(model),
        "confidence_source": np.array(confidence_source),
        "keypoints_source": np.array("reprojected_3d"),
        "frame_id": np.repeat([f["frame_id"] for f in frames], 2).astype(np.int64),
        "timestamp_ns": np.repeat([f["timestamp_ns"] for f in frames], 2).astype(np.int64),
        "hand": np.tile(np.array(HANDS), len(frames)),
        "joints21_m": np.full((n, 21, 3), np.nan, dtype=np.float32),
        "joint_valid": np.zeros((n, 21), dtype=bool),
        "joint_confidence": np.full((n, 21), np.nan, dtype=np.float32),
        "keypoints21_px": np.full((n, 21, 2), np.nan, dtype=np.float32),
        "projection_valid": np.zeros((n, 21), dtype=bool),
        "detection_confidence": np.full(n, np.nan, dtype=np.float32),
        "valid": np.zeros(n, dtype=bool),
        "invalid_reason": np.full(n, "no_detection", dtype="<U64"),
    }
    return result


def set_prediction(
    result, row, joints, pixels, confidence=None, detection_confidence=None, allow_negative_z=False
):
    joints = np.asarray(joints, dtype=np.float32)
    pixels = np.asarray(pixels, dtype=np.float32)
    if joints.shape != (21, 3) or pixels.shape != (21, 2):
        raise ValueError("A prediction must contain exactly 21 joints and projections")
    valid = np.isfinite(joints).all(axis=1) & (np.linalg.norm(joints, axis=1) > 1e-12)
    if not allow_negative_z:
        valid &= joints[:, 2] > 0
    projected = valid & np.isfinite(pixels).all(axis=1)
    result["joints21_m"][row] = np.where(valid[:, None], joints, np.nan)
    result["joint_valid"][row] = valid
    result["keypoints21_px"][row] = np.where(projected[:, None], pixels, np.nan)
    result["projection_valid"][row] = projected
    result["valid"][row] = valid.any()
    result["invalid_reason"][row] = "" if valid.any() else "invalid_geometry"
    if confidence is not None:
        result["joint_confidence"][row] = np.where(valid, confidence, np.nan)
    if detection_confidence is not None:
        result["detection_confidence"][row] = detection_confidence


def validate(result):
    for key in HEADERS:
        if key not in result or np.asarray(result[key]).shape != ():
            raise ValueError(f"Missing scalar header: {key}")
    if str(result["schema_version"]) != "1.0" or str(result["joint_set"]) != JOINT_SET:
        raise ValueError("Unsupported prediction schema or joint set")
    if tuple(result["joint_names"].tolist()) != JOINT_NAMES:
        raise ValueError("Joint names disagree with hand21_v1")
    n = len(result["frame_id"])
    shapes = {
        "frame_id": (n,),
        "timestamp_ns": (n,),
        "hand": (n,),
        "joints21_m": (n, 21, 3),
        "joint_valid": (n, 21),
        "joint_confidence": (n, 21),
        "keypoints21_px": (n, 21, 2),
        "projection_valid": (n, 21),
        "valid": (n,),
        "invalid_reason": (n,),
        "detection_confidence": (n,),
    }
    for key, shape in shapes.items():
        if key not in result or result[key].shape != shape:
            raise ValueError(f"{key} must have shape {shape}")
    for key in ("frame_id", "timestamp_ns"):
        if result[key].dtype != np.int64:
            raise ValueError(f"{key} must be int64")
    for key in ("joint_valid", "projection_valid", "valid"):
        if result[key].dtype != bool:
            raise ValueError(f"{key} must be boolean")
    if n == 0 or n % 2 or not np.array_equal(result["hand"], np.tile(HANDS, n // 2)):
        raise ValueError("Every frame must have one left and one right record, in that order")
    for key in ("frame_id", "timestamp_ns"):
        values = result[key].reshape(-1, 2)
        if not np.array_equal(values[:, 0], values[:, 1]) or np.any(np.diff(values[:, 0]) <= 0):
            raise ValueError(f"{key} must increase strictly and match between hands")
    if not np.array_equal(result["valid"], result["joint_valid"].any(axis=1)):
        raise ValueError("valid must equal any(joint_valid)")
    if np.any(result["projection_valid"] & ~result["joint_valid"]):
        raise ValueError("A projected joint must have a valid 3D prediction")
    for key, mask in (("joints21_m", "joint_valid"), ("keypoints21_px", "projection_valid")):
        if not np.isfinite(result[key][result[mask]]).all():
            raise ValueError(f"Valid {key} must be finite")
        if not np.isnan(result[key][~result[mask]]).all():
            raise ValueError(f"Missing {key} must be NaN, not zero or stale predictions")
    for key in ("joint_confidence", "detection_confidence"):
        values = result[key]
        if np.isinf(values).any() or np.any((values < 0) | (values > 1)):
            raise ValueError(f"{key} must be in [0, 1] or NaN")
    if np.any((result["invalid_reason"] == "") != result["valid"]):
        raise ValueError("Invalid records require a reason; valid records require an empty reason")
