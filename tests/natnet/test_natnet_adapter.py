import struct
from types import SimpleNamespace

import pytest

from natnet_zenoh._sdk.NatNetClient import NatNetClient
from natnet_zenoh.natnet_adapter import (
    convert_frame,
    create_natnet_client,
    decode_marker_id,
    decode_marker_params,
)


def mocap_data(markers=None, rigid_bodies=None):
    return SimpleNamespace(
        labeled_marker_data=SimpleNamespace(labeled_marker_list=markers or []),
        rigid_body_data=SimpleNamespace(rigid_body_list=rigid_bodies or []),
    )


def test_decode_marker_id_uses_high_and_low_words():
    assert decode_marker_id((7 << 16) | 11) == (7, 11)


def test_decode_all_marker_parameter_bits():
    assert decode_marker_params(0xFF) == {
        "occluded": True,
        "point_cloud_solved": True,
        "model_filled": True,
        "has_model": True,
        "unlabeled": True,
        "active": True,
        "established": True,
        "measurement": True,
    }


def test_convert_frame_preserves_marker_and_rigid_body_fields():
    marker = SimpleNamespace(
        id_num=(3 << 16) | 9,
        pos=(0.1, 0.2, 0.3),
        size=0.009,
        residual=0.4,
        param=0x5B,
    )
    rigid_body = SimpleNamespace(
        id_num=2,
        pos=(1.0, 2.0, 3.0),
        rot=(0.1, 0.2, 0.3, 0.9),
        error=0.0007,
        tracking_valid=True,
    )
    data_dict = {
        "frame_number": 123,
        "timestamp": 9.5,
        "mocap_data": mocap_data([marker], [rigid_body]),
    }

    frame = convert_frame(data_dict, received_time_ns=555, dropped_frames=4)

    assert frame["frame_number"] == 123
    assert frame["motive_timestamp"] == 9.5
    assert frame["publisher_received_time_ns"] == 555
    assert frame["publisher_dropped_frames"] == 4
    assert frame["markers"] == [
        {
            "raw_id": (3 << 16) | 9,
            "model_id": 3,
            "member_id": 9,
            "id_kind": "asset_member",
            "position": [0.1, 0.2, 0.3],
            "size": 0.009,
            "residual_m_per_ray": 0.0004,
            "occluded": True,
            "point_cloud_solved": True,
            "model_filled": False,
            "has_model": True,
            "unlabeled": True,
            "active": False,
            "established": True,
            "measurement": False,
        }
    ]
    assert frame["rigid_bodies"] == [
        {
            "id": 2,
            "position": [1.0, 2.0, 3.0],
            "quaternion_xyzw": [0.1, 0.2, 0.3, 0.9],
            "mean_error": 0.0007,
            "tracking_valid": True,
        }
    ]


def test_convert_frame_handles_missing_collections_without_mutation():
    data = mocap_data()
    data_dict = {"frame_number": 1, "timestamp": 0.0, "mocap_data": data}

    frame = convert_frame(data_dict, received_time_ns=10)

    assert frame["markers"] == []
    assert frame["rigid_bodies"] == []
    assert data.labeled_marker_data.labeled_marker_list == []


def test_convert_frame_marks_active_ids_without_claiming_asset_identity():
    marker = SimpleNamespace(
        id_num=77,
        pos=(0.1, 0.2, 0.3),
        size=0.009,
        residual=0.2,
        param=0x20,
    )

    converted = convert_frame(
        {"frame_number": 1, "timestamp": 0.0, "mocap_data": mocap_data([marker])},
        received_time_ns=10,
    )

    assert converted["markers"][0]["id_kind"] == "active"


def test_create_natnet_client_configures_unicast_without_starting():
    client = create_natnet_client("192.168.10.1", "192.168.10.2")

    assert client.get_server_address() == "192.168.10.1"
    assert client.get_client_address() == "192.168.10.2"
    assert client.use_multicast is False
    assert client.data_thread is None


def test_natnet_client_print_level_zero_suppresses_per_frame_output(capsys):
    client = create_natnet_client("192.168.10.1", "192.168.10.2")
    mocap_data = SimpleNamespace(prefix_data=SimpleNamespace(frame_number=42))
    client._NatNetClient__unpack_mocap_data = lambda *_args: (0, mocap_data)
    packet = client.NAT_FRAMEOFDATA.to_bytes(2, "little", signed=True) + b"\x00\x00"

    client._NatNetClient__process_message(packet, print_level=0)

    assert capsys.readouterr().out == ""


def test_natnet_4_4_suffix_uses_precision_timestamp_layout():
    client = NatNetClient()
    payload = struct.pack(
        "<IIdQQQIIh",
        0x01020304,
        7,
        12.5,
        101,
        202,
        303,
        1_700_000_000,
        500_000_000,
        0x03,
    )

    offset, suffix = client._NatNetClient__unpack_frame_suffix_data(
        payload,
        len(payload),
        4,
        4,
    )

    assert offset == len(payload)
    assert suffix.timestamp == 12.5
    assert suffix.stamp_camera_mid_exposure == 101
    assert suffix.stamp_data_received == 202
    assert suffix.stamp_transmit == 303
    assert suffix.prec_timestamp_secs == 1_700_000_000
    assert suffix.prec_timestamp_frac_secs == 500_000_000
    assert suffix.is_recording is True
    assert suffix.tracked_models_changed is True


def test_natnet_client_shutdown_is_safe_after_partial_socket_startup():
    class FakeSocket:
        def __init__(self):
            self.closed = False

        def close(self):
            self.closed = True

    client = NatNetClient()
    data_socket = FakeSocket()
    client.data_socket = data_socket
    client.command_socket = None

    client.shutdown()

    assert data_socket.closed
    assert client.data_socket is None


def test_natnet_4_4_binary_frame_depacketizes_all_marker_id_kinds_and_rigid_bodies():
    def bundle(count, payload=b""):
        return struct.pack("<ii", count, len(payload)) + payload

    rigid_body_payload = b"".join(
        (
            struct.pack("<i3f4ffh", 1, 0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0, 0.001, 1),
            struct.pack("<i3f4ffh", 2, 1.1, 1.2, 1.3, 0.1, 0.2, 0.3, 0.9, 0.002, 1),
        )
    )
    marker_payload = b"".join(
        (
            struct.pack(
                "<i3ffhf",
                (2 << 16) | 5,
                0.10,
                0.20,
                0.30,
                0.009,
                0x4D,
                0.0002,
            ),
            struct.pack("<i3ffhf", 18, 0.40, 0.50, 0.60, 0.009, 0x52, 0.0003),
            struct.pack("<i3ffhf", 77, 0.70, 0.80, 0.90, 0.009, 0x60, 0.0004),
        )
    )
    suffix = struct.pack("<IIdQQQIIhi", 0, 0, 12.5, 101, 202, 303, 10, 20, 0, 0)
    payload = b"".join(
        (
            struct.pack("<i", 321),
            bundle(0),
            bundle(0),
            bundle(2, rigid_body_payload),
            bundle(0),
            bundle(0),
            bundle(3, marker_payload),
            bundle(0),
            bundle(0),
            suffix,
        )
    )
    client = NatNetClient()

    offset, mocap = client._NatNetClient__unpack_mocap_data(payload, len(payload), 4, 4)
    frame = convert_frame(
        {"frame_number": 321, "timestamp": mocap.suffix_data.timestamp, "mocap_data": mocap},
        received_time_ns=999,
    )

    assert offset == len(payload)
    assert [marker["id_kind"] for marker in frame["markers"]] == [
        "asset_member",
        "point_cloud",
        "active",
    ]
    assert frame["markers"][0]["model_filled"] is True
    assert frame["markers"][0]["residual_m_per_ray"] == pytest.approx(0.0002)
    assert [body["id"] for body in frame["rigid_bodies"]] == [1, 2]
