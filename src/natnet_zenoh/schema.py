"""Versioned JSON schema for complete NatNet marker frames."""

from __future__ import annotations

import json
import math
from collections.abc import Mapping, Sequence
from typing import Any

SCHEMA_VERSION = 1
FRAME_KEY = "mocap/hands/frame"
COORDINATE_SYSTEM = "motive_x_forward_z_up_right_handed"
UNIT = "meter"
_MARKER_ID_KINDS = {"active", "asset_member", "point_cloud", "unknown"}

_MARKER_BOOLEAN_FIELDS = (
    "occluded",
    "point_cloud_solved",
    "model_filled",
    "has_model",
    "unlabeled",
    "active",
    "established",
    "measurement",
)


class FrameValidationError(ValueError):
    """Raised when a frame does not match the supported wire schema."""


def _require(mapping: Mapping[str, Any], field: str, path: str) -> Any:
    if field not in mapping:
        raise FrameValidationError(f"{path}.{field} is required")
    return mapping[field]


def _is_integer(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _is_finite_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _validate_vector(value: object, length: int, path: str) -> None:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes, bytearray))
        or len(value) != length
        or not all(_is_finite_number(item) for item in value)
    ):
        raise FrameValidationError(f"{path} must contain {length} finite numbers")


def _validate_marker(marker: object, index: int) -> None:
    path = f"markers[{index}]"
    if not isinstance(marker, Mapping):
        raise FrameValidationError(f"{path} must be an object")

    _validate_vector(_require(marker, "position", path), 3, f"{path}.position")
    for field in ("raw_id", "model_id", "member_id"):
        if not _is_integer(_require(marker, field, path)):
            raise FrameValidationError(f"{path}.{field} must be an integer")
    id_kind = _require(marker, "id_kind", path)
    if not isinstance(id_kind, str) or id_kind not in _MARKER_ID_KINDS:
        raise FrameValidationError(f"{path}.id_kind is not supported")
    for field in ("size", "residual_m_per_ray"):
        if not _is_finite_number(_require(marker, field, path)):
            raise FrameValidationError(f"{path}.{field} must be a finite number")
    for field in _MARKER_BOOLEAN_FIELDS:
        if not isinstance(_require(marker, field, path), bool):
            raise FrameValidationError(f"{path}.{field} must be a boolean")


def _validate_rigid_body(rigid_body: object, index: int) -> None:
    path = f"rigid_bodies[{index}]"
    if not isinstance(rigid_body, Mapping):
        raise FrameValidationError(f"{path} must be an object")

    _validate_vector(_require(rigid_body, "position", path), 3, f"{path}.position")
    _validate_vector(
        _require(rigid_body, "quaternion_xyzw", path),
        4,
        f"{path}.quaternion_xyzw",
    )
    if not _is_integer(_require(rigid_body, "id", path)):
        raise FrameValidationError(f"{path}.id must be an integer")
    if not _is_finite_number(_require(rigid_body, "mean_error", path)):
        raise FrameValidationError(f"{path}.mean_error must be a finite number")
    if not isinstance(_require(rigid_body, "tracking_valid", path), bool):
        raise FrameValidationError(f"{path}.tracking_valid must be a boolean")


def validate_frame(frame: Mapping[str, object]) -> None:
    """Validate a frame and raise :class:`FrameValidationError` on mismatch."""

    if not isinstance(frame, Mapping):
        raise FrameValidationError("frame must be a JSON object")

    schema_version = _require(frame, "schema_version", "frame")
    if not _is_integer(schema_version) or schema_version != SCHEMA_VERSION:
        raise FrameValidationError(f"schema_version must be {SCHEMA_VERSION}")
    if not _is_integer(_require(frame, "frame_number", "frame")):
        raise FrameValidationError("frame_number must be an integer")
    if not _is_finite_number(_require(frame, "motive_timestamp", "frame")):
        raise FrameValidationError("motive_timestamp must be a finite number")
    if not _is_integer(_require(frame, "publisher_received_time_ns", "frame")):
        raise FrameValidationError("publisher_received_time_ns must be an integer")
    if _require(frame, "coordinate_system", "frame") != COORDINATE_SYSTEM:
        raise FrameValidationError(f"coordinate_system must be {COORDINATE_SYSTEM}")
    if _require(frame, "unit", "frame") != UNIT:
        raise FrameValidationError(f"unit must be {UNIT}")

    dropped_frames = _require(frame, "publisher_dropped_frames", "frame")
    if not _is_integer(dropped_frames) or dropped_frames < 0:
        raise FrameValidationError("publisher_dropped_frames must be a non-negative integer")

    markers = _require(frame, "markers", "frame")
    if not isinstance(markers, list):
        raise FrameValidationError("markers must be a list")
    for index, marker in enumerate(markers):
        _validate_marker(marker, index)

    rigid_bodies = _require(frame, "rigid_bodies", "frame")
    if not isinstance(rigid_bodies, list):
        raise FrameValidationError("rigid_bodies must be a list")
    for index, rigid_body in enumerate(rigid_bodies):
        _validate_rigid_body(rigid_body, index)


def encode_frame(frame: Mapping[str, object]) -> str:
    """Validate and encode one frame as compact UTF-8-compatible JSON text."""

    validate_frame(frame)
    try:
        return json.dumps(frame, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise FrameValidationError(f"frame is not JSON serializable: {exc}") from exc


def decode_frame(payload: bytes | str) -> dict[str, object]:
    """Decode and validate one JSON frame payload."""

    try:
        text = payload.decode("utf-8") if isinstance(payload, bytes) else payload
        decoded = json.loads(text)
    except (UnicodeDecodeError, json.JSONDecodeError, TypeError) as exc:
        raise FrameValidationError(f"invalid frame JSON: {exc}") from exc
    if not isinstance(decoded, dict):
        raise FrameValidationError("frame JSON must contain an object")
    validate_frame(decoded)
    return decoded
