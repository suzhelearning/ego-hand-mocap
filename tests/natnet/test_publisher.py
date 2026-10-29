import argparse
import threading
import time
from types import SimpleNamespace

import pytest
import zenoh

from natnet_zenoh.publisher import (
    RIGID_BODY_NAMES_KEY,
    build_parser,
    run_publisher,
)
from natnet_zenoh.schema import FRAME_KEY, decode_frame


def args(**overrides):
    values = {
        "motive_server_ip": "192.168.1.10",
        "natnet_client_ip": "192.168.1.10",
        "zenoh_endpoint": "tcp/192.168.1.20:7447",
        "queue_capacity": 4,
        "key": FRAME_KEY,
        "startup_timeout": 0.1,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


def empty_callback_frame():
    mocap_data = SimpleNamespace(
        labeled_marker_data=SimpleNamespace(labeled_marker_list=[]),
        rigid_body_data=SimpleNamespace(rigid_body_list=[]),
    )
    return {"frame_number": 9, "timestamp": 1.5, "mocap_data": mocap_data}


class FakePublisher:
    def __init__(self):
        self.payloads = []

    def put(self, payload):
        self.payloads.append(payload)


class FakeSession:
    def __init__(self):
        self.publisher = None
        self.publishers = {}
        self.declared_keys = []
        self.declared_encodings = {}
        self.closed = False

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.closed = True

    def declare_publisher(self, key, *, encoding):
        publisher = FakePublisher()
        self.publishers[key] = publisher
        self.declared_keys.append(key)
        self.declared_encodings[key] = encoding
        if key == FRAME_KEY:
            self.publisher = publisher
        return publisher

class FakeZenoh:
    Encoding = zenoh.Encoding

    def __init__(self):
        self.session = FakeSession()

    def open(self, _config):
        return self.session


class FakeNatNetClient:
    def __init__(self, stop_event, send_frame=True, run_result=True):
        self.new_frame_with_data_listener = None
        self.stop_event = stop_event
        self.send_frame = send_frame
        self.run_result = run_result
        self.run_mode = None
        self.shutdown_called = False

    def run(self, mode):
        self.run_mode = mode
        if not self.run_result:
            return False
        if self.send_frame:
            self.new_frame_with_data_listener(empty_callback_frame())
            self.stop_event.set()
        return True

    def shutdown(self):
        self.shutdown_called = True


class MalformedFrameNatNetClient(FakeNatNetClient):
    def __init__(self, stop_event):
        super().__init__(stop_event)
        self.callback_exception = None

    def run(self, mode):
        self.run_mode = mode
        self.new_frame_with_data_listener(empty_callback_frame())
        try:
            self.new_frame_with_data_listener({"frame_number": 10})
        except Exception as exc:
            self.callback_exception = exc
        finally:
            self.stop_event.set()
        return True


class DeadDataThreadNatNetClient(FakeNatNetClient):
    def run(self, mode):
        self.run_mode = mode
        self.new_frame_with_data_listener(empty_callback_frame())
        self.data_thread = SimpleNamespace(is_alive=lambda: False)
        return True


class StopAfterSecondWait:
    def __init__(self):
        self.wait_calls = 0
        self.stopped = False

    def is_set(self):
        return self.stopped

    def set(self):
        self.stopped = True

    def wait(self, _timeout):
        self.wait_calls += 1
        return self.stopped or self.wait_calls >= 2


def test_run_publisher_publishes_frame_and_closes_resources():
    stop_event = threading.Event()
    client = FakeNatNetClient(stop_event)
    fake_zenoh = FakeZenoh()

    result = run_publisher(
        args(),
        client_factory=lambda _server, _client: client,
        zenoh_module=fake_zenoh,
        stop_event=stop_event,
    )

    assert result == 0
    assert client.run_mode == "d"
    assert client.shutdown_called
    assert fake_zenoh.session.closed
    assert fake_zenoh.session.declared_keys == [
        FRAME_KEY, RIGID_BODY_NAMES_KEY]
    assert fake_zenoh.session.declared_encodings[FRAME_KEY] == (
        zenoh.Encoding.APPLICATION_JSON)
    assert len(fake_zenoh.session.publisher.payloads) == 1
    assert decode_frame(
        fake_zenoh.session.publisher.payloads[0])["frame_number"] == 9


def test_run_publisher_returns_two_when_natnet_cannot_start():
    stop_event = threading.Event()
    client = FakeNatNetClient(stop_event, run_result=False)

    result = run_publisher(
        args(),
        client_factory=lambda _server, _client: client,
        zenoh_module=FakeZenoh(),
        stop_event=stop_event,
    )

    assert result == 2
    assert client.shutdown_called


def test_run_publisher_times_out_when_no_natnet_frame_arrives():
    stop_event = threading.Event()
    client = FakeNatNetClient(stop_event, send_frame=False)

    result = run_publisher(
        args(startup_timeout=0.01),
        client_factory=lambda _server, _client: client,
        zenoh_module=FakeZenoh(),
        stop_event=stop_event,
    )

    assert result == 2
    assert client.shutdown_called


def test_run_publisher_returns_three_when_zenoh_open_fails():
    class BrokenZenoh(FakeZenoh):
        def open(self, _config):
            raise RuntimeError("no route")

    result = run_publisher(
        args(),
        client_factory=lambda _server, _client: None,
        zenoh_module=BrokenZenoh(),
        stop_event=threading.Event(),
    )

    assert result == 3


def test_run_publisher_contains_callback_failure_and_reports_runtime_error():
    stop_event = threading.Event()
    client = MalformedFrameNatNetClient(stop_event)

    result = run_publisher(
        args(),
        client_factory=lambda _server, _client: client,
        zenoh_module=FakeZenoh(),
        stop_event=stop_event,
    )

    assert result == 3
    assert client.callback_exception is None
    assert client.shutdown_called


def test_publisher_cli_rejects_non_positive_runtime_limits():
    common = [
        "--motive-server-ip",
        "127.0.0.1",
        "--natnet-client-ip",
        "127.0.0.1",
        "--zenoh-endpoint",
        "tcp/127.0.0.1:7447",
    ]

    with pytest.raises(SystemExit):
        build_parser().parse_args([*common, "--queue-capacity", "0"])
    with pytest.raises(SystemExit):
        build_parser().parse_args([*common, "--startup-timeout", "0"])
    with pytest.raises(SystemExit):
        build_parser().parse_args([*common, "--startup-timeout", "nan"])


def test_run_publisher_reports_dead_natnet_data_thread():
    stop_event = StopAfterSecondWait()
    client = DeadDataThreadNatNetClient(stop_event)

    result = run_publisher(
        args(),
        client_factory=lambda _server, _client: client,
        zenoh_module=FakeZenoh(),
        stop_event=stop_event,
    )

    assert result == 3
    assert client.shutdown_called


def test_run_publisher_drains_frame_when_stop_was_already_requested():
    stop_event = threading.Event()
    stop_event.set()
    client = FakeNatNetClient(stop_event)
    fake_zenoh = FakeZenoh()

    def delayed_client_factory(_server, _client):
        time.sleep(0.1)
        return client

    result = run_publisher(
        args(),
        client_factory=delayed_client_factory,
        zenoh_module=fake_zenoh,
        stop_event=stop_event,
    )

    assert result == 0
    assert len(fake_zenoh.session.publisher.payloads) == 1


def test_run_publisher_reports_natnet_factory_failure_accurately(caplog):
    def broken_client_factory(_server, _client):
        raise RuntimeError("SDK sources missing")

    result = run_publisher(
        args(),
        client_factory=broken_client_factory,
        zenoh_module=FakeZenoh(),
        stop_event=threading.Event(),
    )

    assert result == 3
    assert "NatNet publisher runtime failed" in caplog.text
