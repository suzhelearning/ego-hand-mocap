import math

import pytest

from natnet_zenoh.schema import (
    FrameValidationError,
    decode_frame,
    encode_frame,
    validate_frame,
)


def valid_frame():
    return {
        "schema_version": 1,
        "frame_number": 42,
        "motive_timestamp": 1.25,
        "publisher_received_time_ns": 123456789,
        "coordinate_system": "motive_x_forward_z_up_right_handed",
        "unit": "meter",
        "publisher_dropped_frames": 0,
        "markers": [],
        "rigid_bodies": [],
    }


def valid_marker():
    return {
        "raw_id": 18,
        "model_id": 0,
        "member_id": 18,
        "id_kind": "point_cloud",
        "position": [0.1, 0.2, 0.3],
        "size": 0.009,
        "residual_m_per_ray": 0.0001,
        "occluded": False,
        "point_cloud_solved": True,
        "model_filled": False,
        "has_model": False,
        "unlabeled": True,
        "active": False,
        "established": True,
        "measurement": False,
    }


def test_round_trip_empty_frame():
    frame = valid_frame()

    assert decode_frame(encode_frame(frame)) == frame


def test_rejects_missing_frame_number():
    frame = valid_frame()
    del frame["frame_number"]

    with pytest.raises(FrameValidationError, match="frame_number"):
        validate_frame(frame)


def test_rejects_unknown_schema_version():
    frame = valid_frame()
    frame["schema_version"] = 2

    with pytest.raises(FrameValidationError, match="schema_version"):
        validate_frame(frame)


def test_rejects_boolean_schema_version():
    frame = valid_frame()
    frame["schema_version"] = True

    with pytest.raises(FrameValidationError, match="schema_version"):
        validate_frame(frame)


def test_rejects_non_list_markers():
    frame = valid_frame()
    frame["markers"] = {}

    with pytest.raises(FrameValidationError, match="markers"):
        validate_frame(frame)


@pytest.mark.parametrize("position", [[1.0, 2.0], [1.0, 2.0, math.nan], "1,2,3"])
def test_rejects_invalid_marker_position(position):
    frame = valid_frame()
    marker = valid_marker()
    marker["position"] = position
    frame["markers"] = [marker]

    with pytest.raises(FrameValidationError, match=r"markers\[0\]\.position"):
        validate_frame(frame)


def test_rejects_marker_without_raw_id():
    frame = valid_frame()
    marker = valid_marker()
    del marker["raw_id"]
    frame["markers"] = [marker]

    with pytest.raises(FrameValidationError, match="raw_id"):
        validate_frame(frame)


def test_rejects_non_string_marker_id_kind_as_validation_error():
    frame = valid_frame()
    marker = valid_marker()
    marker["id_kind"] = []
    frame["markers"] = [marker]

    with pytest.raises(FrameValidationError, match="id_kind"):
        validate_frame(frame)


def test_rejects_invalid_rigid_body_quaternion():
    frame = valid_frame()
    frame["rigid_bodies"] = [{"position": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0]}]

    with pytest.raises(FrameValidationError, match="quaternion_xyzw"):
        validate_frame(frame)


def test_decode_rejects_invalid_json():
    with pytest.raises(FrameValidationError, match="JSON"):
        decode_frame(b"not json")
