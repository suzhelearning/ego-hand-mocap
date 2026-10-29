"""Convert NatNet Python sample objects into the shared frame schema."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from .schema import COORDINATE_SYSTEM, SCHEMA_VERSION, UNIT, validate_frame

_PARAMETER_BITS = {
    "occluded": 0x01,
    "point_cloud_solved": 0x02,
    "model_filled": 0x04,
    "has_model": 0x08,
    "unlabeled": 0x10,
    "active": 0x20,
    "established": 0x40,
    "measurement": 0x80,
}


def decode_marker_id(raw_id: int) -> tuple[int, int]:
    """Decode NatNet's packed Asset/Model ID and member ID."""

    unsigned_id = int(raw_id) & 0xFFFFFFFF
    return (unsigned_id >> 16) & 0xFFFF, unsigned_id & 0xFFFF


def decode_marker_params(params: int) -> dict[str, bool]:
    """Decode every marker status flag defined by NatNet 4.4."""

    value = int(params)
    return {name: bool(value & bit) for name, bit in _PARAMETER_BITS.items()}


def _items(container: object, attribute: str) -> list[Any]:
    if container is None:
        return []
    value = getattr(container, attribute, None)
    return list(value) if value is not None else []


def _convert_marker(marker: object) -> dict[str, object]:
    raw_id = int(getattr(marker, "id_num"))
    model_id, member_id = decode_marker_id(raw_id)
    marker_params = decode_marker_params(int(getattr(marker, "param")))
    if marker_params["active"]:
        id_kind = "active"
    elif marker_params["has_model"]:
        id_kind = "asset_member"
    elif marker_params["unlabeled"]:
        id_kind = "point_cloud"
    else:
        id_kind = "unknown"
    converted: dict[str, object] = {
        "raw_id": raw_id,
        "model_id": model_id,
        "member_id": member_id,
        "id_kind": id_kind,
        "position": [float(value) for value in getattr(marker, "pos")],
        "size": float(getattr(marker, "size")),
        # NatNet's Python depacketizer stores labeled-marker residual in mm/ray.
        "residual_m_per_ray": float(getattr(marker, "residual")) / 1000.0,
    }
    converted.update(marker_params)
    return converted


def _convert_rigid_body(rigid_body: object) -> dict[str, object]:
    mean_error = getattr(rigid_body, "mean_error", getattr(rigid_body, "error", 0.0))
    return {
        "id": int(getattr(rigid_body, "id_num")),
        "position": [float(value) for value in getattr(rigid_body, "pos")],
        "quaternion_xyzw": [float(value) for value in getattr(rigid_body, "rot")],
        "mean_error": float(mean_error),
        "tracking_valid": bool(getattr(rigid_body, "tracking_valid")),
    }


def convert_frame(
    data_dict: Mapping[str, object],
    received_time_ns: int,
    dropped_frames: int = 0,
) -> dict[str, object]:
    """Copy one NatNet callback payload into a validated wire frame."""

    mocap_data = data_dict["mocap_data"]
    markers = _items(getattr(mocap_data, "labeled_marker_data", None), "labeled_marker_list")
    rigid_bodies = _items(getattr(mocap_data, "rigid_body_data", None), "rigid_body_list")
    frame: dict[str, object] = {
        "schema_version": SCHEMA_VERSION,
        "frame_number": int(data_dict["frame_number"]),
        "motive_timestamp": float(data_dict["timestamp"]),
        "publisher_received_time_ns": int(received_time_ns),
        "coordinate_system": COORDINATE_SYSTEM,
        "unit": UNIT,
        "publisher_dropped_frames": int(dropped_frames),
        "markers": [_convert_marker(marker) for marker in markers],
        "rigid_bodies": [_convert_rigid_body(body) for body in rigid_bodies],
    }
    # Preserve raw server-clock ticks; do not label these as nanoseconds.
    suffix = getattr(mocap_data, "suffix_data", None)
    frame["natnet_timing"] = {
        name: (int(getattr(suffix, name)) if getattr(suffix, name, -1) >= 0 else None)
        for name in ("stamp_camera_mid_exposure", "stamp_data_received", "stamp_transmit",
                     "prec_timestamp_secs", "prec_timestamp_frac_secs")
    }
    frame["natnet_timing"]["tick_frequency_hz"] = None
    frame["publisher_time_domain"] = "linux-clock-monotonic"
    validate_frame(frame)
    return frame


def create_natnet_client(server_ip: str, client_ip: str) -> object:
    """Create, but do not start, the built-in NatNet direct client."""

    from ._sdk.NatNetClient import NatNetClient

    client = NatNetClient()
    client.set_server_address(server_ip)
    client.set_client_address(client_ip)
    client.set_use_multicast(False)
    client.set_print_level(0)
    return client
