import argparse
import json
import threading

import pytest

from natnet_zenoh.schema import FRAME_KEY, encode_frame
from natnet_zenoh.subscriber import (
    FrameHandler,
    FrameStats,
    JsonlRecorder,
    build_parser,
    run_subscriber,
)


def valid_frame(number=1):
    return {
        "schema_version": 1,
        "frame_number": number,
        "motive_timestamp": 0.1,
        "publisher_received_time_ns": 10,
        "coordinate_system": "motive_x_forward_z_up_right_handed",
        "unit": "meter",
        "publisher_dropped_frames": 0,
        "markers": [],
        "rigid_bodies": [],
    }


def args(**overrides):
    values = {
        "listen_endpoint": "tcp/127.0.0.1:7447",
        "key": FRAME_KEY,
        "output": None,
        "overwrite": False,
        "stats_interval": 0.01,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


def test_frame_stats_counts_missing_and_out_of_order_frames():
    stats = FrameStats()

    stats.observe(10, marker_count=5, publisher_dropped_frames=1)
    stats.observe(13, marker_count=6, publisher_dropped_frames=3)
    stats.observe(12, marker_count=4, publisher_dropped_frames=3)

    assert stats.received_frames == 3
    assert stats.missing_frames == 2
    assert stats.out_of_order_frames == 1
    assert stats.last_marker_count == 4
    assert stats.publisher_dropped_frames == 3


def test_frame_stats_reports_interval_rate_and_source_restart():
    timestamps = iter((10.0, 12.0))
    stats = FrameStats(clock=lambda: next(timestamps))

    stats.observe(500)
    stats.observe(0)
    stats.observe(1)

    snapshot = stats.snapshot()

    assert snapshot["frame_rate_hz"] == 1.5
    assert snapshot["source_restarts"] == 1
    assert snapshot["out_of_order_frames"] == 0


def test_jsonl_recorder_creates_parent_and_writes_one_frame_per_line(tmp_path):
    output = tmp_path / "nested" / "session.jsonl"
    recorder = JsonlRecorder(output)

    recorder.write(encode_frame(valid_frame()))
    recorder.close()

    lines = output.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 1
    assert json.loads(lines[0])["frame_number"] == 1


def test_jsonl_recorder_refuses_to_overwrite_existing_capture(tmp_path):
    output = tmp_path / "session.jsonl"
    output.write_text("existing capture\n", encoding="utf-8")

    with pytest.raises(FileExistsError):
        JsonlRecorder(output)

    assert output.read_text(encoding="utf-8") == "existing capture\n"


def test_jsonl_recorder_overwrites_only_when_explicitly_requested(tmp_path):
    output = tmp_path / "session.jsonl"
    output.write_text("existing capture\n", encoding="utf-8")

    recorder = JsonlRecorder(output, overwrite=True)
    recorder.write(encode_frame(valid_frame(4)))
    recorder.close()

    assert json.loads(output.read_text(encoding="utf-8"))["frame_number"] == 4


def test_frame_handler_rejects_invalid_message_without_recording(tmp_path):
    output = tmp_path / "session.jsonl"
    recorder = JsonlRecorder(output)
    stats = FrameStats()
    handler = FrameHandler(stats, recorder=recorder)

    assert handler.handle_json("not json") is None
    recorder.close()

    assert stats.invalid_messages == 1
    assert output.read_text(encoding="utf-8") == ""


def test_frame_handler_observes_publisher_drop_counter():
    frame = valid_frame(3)
    frame["publisher_dropped_frames"] = 7
    stats = FrameStats()
    handler = FrameHandler(stats)

    handler.handle_json(encode_frame(frame))

    assert stats.publisher_dropped_frames == 7


def test_frame_handler_stops_after_recording_failure():
    class BrokenRecorder:
        def write(self, _raw_json):
            raise OSError("disk full")

    stop_event = threading.Event()
    handler = FrameHandler(FrameStats(), recorder=BrokenRecorder(), stop_event=stop_event)

    handler.handle_json(encode_frame(valid_frame()))

    assert isinstance(handler.fatal_error, OSError)
    assert stop_event.is_set()


class FakePayload:
    def __init__(self, value):
        self.value = value

    def to_string(self):
        return self.value


class FakeSample:
    def __init__(self, value):
        self.payload = FakePayload(value)


class FakeSubscriber:
    def __init__(self):
        self.undeclared = False

    def undeclare(self):
        self.undeclared = True


class FakeSession:
    def __init__(self, stop_event, sample):
        self.stop_event = stop_event
        self.sample = sample
        self.closed = False
        self.key = None
        self.subscriber = FakeSubscriber()

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.closed = True

    def declare_subscriber(self, key, handler):
        self.key = key
        handler(self.sample)
        self.stop_event.set()
        return self.subscriber


class FakeZenoh:
    def __init__(self, session):
        self.session = session

    def open(self, _config):
        return self.session


def test_run_subscriber_records_a_valid_sample_and_closes(tmp_path):
    stop_event = threading.Event()
    session = FakeSession(stop_event, FakeSample(encode_frame(valid_frame(8))))
    output = tmp_path / "capture.jsonl"

    result = run_subscriber(
        args(output=output),
        zenoh_module=FakeZenoh(session),
        stop_event=stop_event,
    )

    assert result == 0
    assert session.closed
    assert session.key == FRAME_KEY
    assert session.subscriber.undeclared
    assert json.loads(output.read_text(encoding="utf-8"))["frame_number"] == 8


def test_run_subscriber_returns_four_after_recording_failure():
    class BrokenRecorder:
        def __init__(self, _path, **_kwargs):
            pass

        def write(self, _raw_json):
            raise OSError("disk full")

        def close(self):
            pass

    stop_event = threading.Event()
    session = FakeSession(stop_event, FakeSample(encode_frame(valid_frame())))

    result = run_subscriber(
        args(output="capture.jsonl"),
        zenoh_module=FakeZenoh(session),
        stop_event=stop_event,
        recorder_factory=BrokenRecorder,
    )

    assert result == 4


def test_run_subscriber_returns_four_when_capture_close_fails():
    class CloseFailureRecorder:
        def __init__(self, _path, **_kwargs):
            pass

        def write(self, _raw_json):
            pass

        def close(self):
            raise OSError("flush failed")

    stop_event = threading.Event()
    session = FakeSession(stop_event, FakeSample(encode_frame(valid_frame())))

    result = run_subscriber(
        args(output="capture.jsonl"),
        zenoh_module=FakeZenoh(session),
        stop_event=stop_event,
        recorder_factory=CloseFailureRecorder,
    )

    assert result == 4


def test_subscriber_cli_rejects_non_positive_stats_interval():
    with pytest.raises(SystemExit):
        build_parser().parse_args(["--stats-interval", "0"])
    with pytest.raises(SystemExit):
        build_parser().parse_args(["--stats-interval", "nan"])
