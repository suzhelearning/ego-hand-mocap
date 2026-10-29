import socket
import threading

import zenoh

from natnet_zenoh.schema import FRAME_KEY, decode_frame
from natnet_zenoh.zenoh_transport import build_peer_config, publish_json


def unused_tcp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def test_one_complete_frame_crosses_a_real_tcp_peer_connection():
    frame = {
        "schema_version": 1,
        "frame_number": 7,
        "motive_timestamp": 0.25,
        "publisher_received_time_ns": 100,
        "coordinate_system": "motive_x_forward_z_up_right_handed",
        "unit": "meter",
        "publisher_dropped_frames": 0,
        "markers": [],
        "rigid_bodies": [],
    }
    port = unused_tcp_port()
    received = []
    ready = threading.Event()

    def listener(sample):
        received.append(decode_frame(sample.payload.to_string()))
        ready.set()

    listen_config = build_peer_config(listen_endpoint=f"tcp/127.0.0.1:{port}")
    connect_config = build_peer_config(connect_endpoint=f"tcp/127.0.0.1:{port}")

    with zenoh.open(listen_config) as receiver_session:
        subscriber = receiver_session.declare_subscriber(FRAME_KEY, listener)
        try:
            with zenoh.open(connect_config) as sender_session:
                publisher = sender_session.declare_publisher(
                    FRAME_KEY,
                    encoding=zenoh.Encoding.APPLICATION_JSON,
                )
                publish_json(publisher, frame)
                assert ready.wait(5.0), "Zenoh loopback sample was not received within five seconds"
        finally:
            subscriber.undeclare()

    assert received == [frame]
