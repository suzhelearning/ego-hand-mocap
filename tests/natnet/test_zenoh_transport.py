import json

import pytest

from natnet_zenoh.schema import decode_frame
from natnet_zenoh.zenoh_transport import build_peer_config, publish_json


def valid_frame():
    return {
        "schema_version": 1,
        "frame_number": 1,
        "motive_timestamp": 0.1,
        "publisher_received_time_ns": 10,
        "coordinate_system": "motive_x_forward_z_up_right_handed",
        "unit": "meter",
        "publisher_dropped_frames": 0,
        "markers": [],
        "rigid_bodies": [],
    }


def test_build_connecting_peer_config_disables_discovery():
    config = build_peer_config(connect_endpoint="tcp/192.168.1.20:7447")

    assert json.loads(config.get_json("mode")) == "peer"
    assert json.loads(config.get_json("connect/endpoints")) == ["tcp/192.168.1.20:7447"]
    assert json.loads(config.get_json("scouting/multicast/enabled")) is False
    assert json.loads(config.get_json("scouting/gossip/enabled")) is False


def test_build_listening_peer_config():
    config = build_peer_config(listen_endpoint="tcp/0.0.0.0:7447")

    assert json.loads(config.get_json("listen/endpoints")) == ["tcp/0.0.0.0:7447"]


@pytest.mark.parametrize(
    ("connect_endpoint", "listen_endpoint"),
    [(None, None), ("tcp/host:1", "tcp/host:2")],
)
def test_build_peer_config_requires_exactly_one_endpoint(connect_endpoint, listen_endpoint):
    with pytest.raises(ValueError, match="exactly one"):
        build_peer_config(connect_endpoint=connect_endpoint, listen_endpoint=listen_endpoint)


def test_publish_json_writes_a_valid_encoded_frame():
    class FakePublisher:
        def __init__(self):
            self.payload = None

        def put(self, payload):
            self.payload = payload

    publisher = FakePublisher()
    frame = valid_frame()

    publish_json(publisher, frame)

    assert decode_frame(publisher.payload) == frame
