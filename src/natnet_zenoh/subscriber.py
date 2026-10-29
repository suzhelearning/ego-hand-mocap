"""Ubuntu Zenoh marker-frame subscriber and JSONL recorder."""

from __future__ import annotations

import argparse
import logging
import math
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any, TextIO

import zenoh

from .schema import FRAME_KEY, FrameValidationError, decode_frame, encode_frame
from .zenoh_transport import build_peer_config, build_client_config

LOGGER = logging.getLogger(__name__)


def _positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0:
        raise argparse.ArgumentTypeError("must be finite and greater than 0")
    return parsed


class FrameStats:
    """Thread-safe frame continuity and validity counters."""

    def __init__(self, clock: Callable[[], float] = time.monotonic):
        self._lock = threading.Lock()
        self._clock = clock
        self._received_frames = 0
        self._missing_frames = 0
        self._out_of_order_frames = 0
        self._source_restarts = 0
        self._invalid_messages = 0
        self._publisher_dropped_frames = 0
        self._last_publisher_drop_counter: int | None = None
        self._last_frame_number: int | None = None
        self._last_marker_count = 0
        self._rate_sample_time = clock()
        self._rate_sample_received = 0

    @property
    def received_frames(self) -> int:
        with self._lock:
            return self._received_frames

    @property
    def missing_frames(self) -> int:
        with self._lock:
            return self._missing_frames

    @property
    def out_of_order_frames(self) -> int:
        with self._lock:
            return self._out_of_order_frames

    @property
    def invalid_messages(self) -> int:
        with self._lock:
            return self._invalid_messages

    @property
    def last_marker_count(self) -> int:
        with self._lock:
            return self._last_marker_count

    @property
    def publisher_dropped_frames(self) -> int:
        with self._lock:
            return self._publisher_dropped_frames

    def observe(
        self,
        frame_number: int,
        marker_count: int = 0,
        publisher_dropped_frames: int = 0,
    ) -> None:
        with self._lock:
            self._received_frames += 1
            self._last_marker_count = marker_count
            previous_drop_counter = self._last_publisher_drop_counter
            if previous_drop_counter is None:
                self._publisher_dropped_frames += publisher_dropped_frames
            elif publisher_dropped_frames >= previous_drop_counter:
                self._publisher_dropped_frames += (
                    publisher_dropped_frames - previous_drop_counter
                )
            else:
                # 计数器回落 = 发布端进程重启,新累计从 0 开始。旧进程的累计
                # 已通过上面的增量路径全部计入,此处直接把新进程从 0 起的
                # 累计值当作增量计入(下一帧起恢复正常增量路径)。
                self._publisher_dropped_frames += publisher_dropped_frames
            self._last_publisher_drop_counter = publisher_dropped_frames

            if self._last_frame_number is None:
                self._last_frame_number = frame_number
            elif frame_number > self._last_frame_number:
                # 口径说明:missing(帧号缺口)与 publisher_dropped(发布端计数器
                # 增量)对同一次发布端队列驱逐会重复计数——驱逐同时表现为缺口与
                # 计数器增量;纯传输丢失只表现为缺口。两者之和 ≠ 真实丢帧数。
                self._missing_frames += max(0, frame_number - self._last_frame_number - 1)
                self._last_frame_number = frame_number
            elif frame_number < self._last_frame_number - 100:
                self._source_restarts += 1
                self._last_frame_number = frame_number
            else:
                self._out_of_order_frames += 1

    def note_invalid(self) -> None:
        with self._lock:
            self._invalid_messages += 1

    def snapshot(self) -> dict[str, int | float]:
        with self._lock:
            now = self._clock()
            elapsed = now - self._rate_sample_time
            interval_frames = self._received_frames - self._rate_sample_received
            frame_rate_hz = interval_frames / elapsed if elapsed > 0 else 0.0
            self._rate_sample_time = now
            self._rate_sample_received = self._received_frames
            return {
                "received_frames": self._received_frames,
                "missing_frames": self._missing_frames,
                "out_of_order_frames": self._out_of_order_frames,
                "source_restarts": self._source_restarts,
                "invalid_messages": self._invalid_messages,
                "last_marker_count": self._last_marker_count,
                "publisher_dropped_frames": self._publisher_dropped_frames,
                "frame_rate_hz": frame_rate_hz,
            }


class JsonlRecorder:
    """Line-buffered UTF-8 writer for validated marker frames."""

    def __init__(self, path: str | Path, *, overwrite: bool = False):
        output_path = Path(path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        mode = "w" if overwrite else "x"
        self._file: TextIO = output_path.open(mode, encoding="utf-8", buffering=1)
        self._lock = threading.Lock()

    def write(self, raw_json: str) -> None:
        with self._lock:
            self._file.write(raw_json + "\n")

    def close(self) -> None:
        with self._lock:
            self._file.close()


class FrameHandler:
    """Validate samples, update continuity statistics, and optionally record."""

    def __init__(
        self,
        stats: FrameStats,
        recorder: JsonlRecorder | object | None = None,
        stop_event: threading.Event | None = None,
    ):
        self.stats = stats
        self.recorder = recorder
        self.stop_event = stop_event
        self.fatal_error: OSError | None = None

    def handle_json(self, raw_json: str) -> dict[str, object] | None:
        try:
            frame = decode_frame(raw_json)
        except FrameValidationError as exc:
            self.stats.note_invalid()
            LOGGER.warning("Rejected invalid marker frame: %s", exc)
            return None

        self.stats.observe(
            int(frame["frame_number"]),
            len(frame["markers"]),
            int(frame["publisher_dropped_frames"]),
        )
        if self.recorder is not None:
            try:
                self.recorder.write(encode_frame(frame))
            except OSError as exc:
                self.fatal_error = exc
                LOGGER.error("JSONL recording failed: %s", exc)
                if self.stop_event is not None:
                    self.stop_event.set()
                return None
        return frame

    def __call__(self, sample: object) -> None:
        self.handle_json(sample.payload.to_string())


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Receive complete NatNet marker frames from a Windows Zenoh peer."
    )
    parser.add_argument(
        "--listen-endpoint",
        default="tcp/127.0.0.1:7447",
        help="Zenoh TCP endpoint to listen on",
    )
    parser.add_argument("--connect-endpoint", help="Connect as a client to an existing Router")
    parser.add_argument("--key", default=FRAME_KEY)
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional JSONL output path; omit for statistics-only subscribe mode",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Replace an existing --output file instead of refusing to start",
    )
    parser.add_argument("--stats-interval", type=_positive_float, default=2.0)
    return parser


def run_subscriber(
    args: argparse.Namespace,
    zenoh_module: Any = zenoh,
    stop_event: threading.Event | None = None,
    recorder_factory: Callable[..., object] = JsonlRecorder,
) -> int:
    """Listen until interrupted, returning a nonzero code for transport or disk failure."""

    active_stop_event = stop_event or threading.Event()
    recorder = None
    exit_code = 0
    try:
        if args.output is not None:
            try:
                recorder = recorder_factory(
                    args.output,
                    overwrite=getattr(args, "overwrite", False),
                )
            except OSError as exc:
                LOGGER.error("Could not open JSONL output %s: %s", args.output, exc)
                return 4

        stats = FrameStats()
        handler = FrameHandler(stats, recorder=recorder, stop_event=active_stop_event)
        config = (build_client_config(args.connect_endpoint)
                  if getattr(args, "connect_endpoint", None)
                  else build_peer_config(listen_endpoint=args.listen_endpoint))
        with zenoh_module.open(config) as session:
            subscriber = session.declare_subscriber(args.key, handler)
            try:
                while not active_stop_event.wait(args.stats_interval):
                    snapshot = stats.snapshot()
                    LOGGER.info(
                        "frames=%d rate_hz=%.1f markers=%d missing=%d out_of_order=%d "
                        "source_restarts=%d publisher_dropped=%d invalid=%d",
                        snapshot["received_frames"],
                        snapshot["frame_rate_hz"],
                        snapshot["last_marker_count"],
                        snapshot["missing_frames"],
                        snapshot["out_of_order_frames"],
                        snapshot["source_restarts"],
                        snapshot["publisher_dropped_frames"],
                        snapshot["invalid_messages"],
                    )
                exit_code = 4 if handler.fatal_error is not None else 0
            finally:
                subscriber.undeclare()
    except KeyboardInterrupt:
        active_stop_event.set()
        exit_code = 0
    except Exception:
        LOGGER.exception(
            "Subscriber failed; check listen endpoint %s and TCP port access",
            args.listen_endpoint,
        )
        exit_code = 3
    finally:
        if recorder is not None:
            try:
                recorder.close()
            except OSError:
                LOGGER.exception("Could not close JSONL output")
                exit_code = 4
    return exit_code


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    return run_subscriber(build_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
