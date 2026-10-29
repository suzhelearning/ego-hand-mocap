"""Windows NatNet to Zenoh publisher CLI."""

from __future__ import annotations

import argparse
import json
import logging
import math
import queue
import threading
import time
from collections.abc import Callable
from typing import Any

import zenoh

from .frame_queue import LatestFrameQueue
from .natnet_adapter import convert_frame, create_natnet_client
from .schema import FRAME_KEY
from .zenoh_transport import build_client_config, publish_json

LOGGER = logging.getLogger(__name__)

# 刚体名字表 key:Motive 刚体名字 → ID 的映射(一次性+周期重发,供按名字消费)
RIGID_BODY_NAMES_KEY = "mocap/rigid_body_names"
NAMES_REPUBLISH_SEC = 5.0


def _positive_int(value: str) -> int:
    parsed = int(value)
    if parsed < 1:
        raise argparse.ArgumentTypeError("must be at least 1")
    return parsed


def _positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0:
        raise argparse.ArgumentTypeError("must be finite and greater than 0")
    return parsed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Publish complete Motive/NatNet marker frames to an Ubuntu Zenoh peer."
    )
    parser.add_argument("--motive-server-ip", required=True, help="Motive/NatNet server IP")
    parser.add_argument(
        "--natnet-client-ip",
        required=True,
        help="Local Ubuntu interface IP used to receive NatNet",
    )
    parser.add_argument(
        "--zenoh-endpoint",
        required=True,
        help="Ubuntu listener endpoint, for example tcp/192.168.1.20:7447",
    )
    parser.add_argument("--queue-capacity", type=_positive_int, default=256)
    parser.add_argument("--key", default=FRAME_KEY)
    parser.add_argument(
        "--startup-timeout",
        type=_positive_float,
        default=5.0,
        help="Seconds to wait for the first NatNet frame",
    )
    return parser


def _run_with_session(
    args: argparse.Namespace,
    session: object,
    client_factory: Callable[[str, str], object],
    zenoh_module: Any,
    stop_event: threading.Event,
) -> int:
    frames = LatestFrameQueue(args.queue_capacity)
    first_frame = threading.Event()
    runtime_failed = threading.Event()
    producer_done = threading.Event()
    publisher = session.declare_publisher(
        args.key,
        encoding=zenoh_module.Encoding.APPLICATION_JSON,
    )
    names_publisher = session.declare_publisher(
        RIGID_BODY_NAMES_KEY,
        encoding=zenoh_module.Encoding.APPLICATION_JSON,
    )
    rigid_body_names: dict[int, str] = {}
    names_lock = threading.Lock()
    last_names_publish = 0.0

    def publish_names(force: bool = False) -> None:
        """发布刚体名字→ID 映射(内容变化或周期重发)。"""
        nonlocal last_names_publish
        now = time.time()
        with names_lock:
            if not rigid_body_names:
                return
            if not force and now - last_names_publish < NAMES_REPUBLISH_SEC:
                return
            last_names_publish = now
            payload = json.dumps(
                {"names": {str(rid): name for rid, name in rigid_body_names.items()}})
        try:
            names_publisher.put(payload)
        except Exception:
            LOGGER.exception("Rigid body names publication failed")

    def on_descriptions(descs: object) -> None:
        """NatNet DataDescriptions 回调:提取刚体名字→ID。"""
        try:
            mapping = {}
            for rb in getattr(descs, "rigid_body_list", []):
                name = getattr(rb, "sz_name", None)
                if isinstance(name, bytes):          # SDK 描述名为 bytes
                    name = name.decode("utf-8", errors="replace")
                rid = getattr(rb, "id_num", None)
                if name and rid is not None:
                    mapping[int(rid)] = str(name)
            if not mapping:
                return
            changed = False
            with names_lock:
                for rid, name in mapping.items():
                    if rigid_body_names.get(rid) != name:
                        rigid_body_names[rid] = name
                        changed = True
            publish_names(force=changed)
            LOGGER.info("Rigid body names: %s",
                        {rid: n for rid, n in mapping.items()})
        except Exception:
            LOGGER.exception("Data description processing failed")

    def publish_worker() -> None:
        while not producer_done.is_set() or not frames.empty():
            publish_names()                 # 周期重发名字表
            try:
                frame = frames.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                publish_json(publisher, frame)
            except Exception:
                LOGGER.exception("Zenoh frame publication failed")
                runtime_failed.set()
                stop_event.set()
                return

    worker = threading.Thread(target=publish_worker, name="zenoh-publisher", daemon=True)
    worker.start()
    client = None

    try:
        client = client_factory(args.motive_server_ip, args.natnet_client_ip)
        client.data_descriptions_listener = on_descriptions

        def on_frame(data_dict: dict[str, object]) -> None:
            try:
                # publisher_dropped_frames 由 put_latest 在入队时统一打点。
                # 物理对齐必须使用不会受 NTP/人工校时跳变影响的源单调时钟。
                frame = convert_frame(
                    data_dict,
                    received_time_ns=time.monotonic_ns(),
                )
                frames.put_latest(frame)
                first_frame.set()
            except Exception:
                LOGGER.exception("NatNet frame conversion or queueing failed")
                runtime_failed.set()
                stop_event.set()
                first_frame.set()

        client.new_frame_with_data_listener = on_frame
        if not client.run("d"):
            LOGGER.error(
                "NatNet could not start; check Motive Broadcast Frame Data, unicast mode, "
                "server IP %s and client IP %s",
                args.motive_server_ip,
                args.natnet_client_ip,
            )
            return 2
        if not first_frame.wait(args.startup_timeout):
            LOGGER.error(
                "No NatNet frame arrived within %.1f seconds; check Motive streaming and firewall",
                args.startup_timeout,
            )
            return 2

        if runtime_failed.is_set():
            return 3

        while not stop_event.wait(0.05):
            if runtime_failed.is_set():
                return 3
            data_thread = getattr(client, "data_thread", None)
            if data_thread is not None and not data_thread.is_alive():
                LOGGER.error("NatNet data thread stopped unexpectedly")
                return 3
        return 3 if runtime_failed.is_set() else 0
    except KeyboardInterrupt:
        stop_event.set()
        return 0
    except Exception:
        LOGGER.exception("NatNet publisher runtime failed")
        return 3
    finally:
        stop_event.set()
        if client is not None:
            try:
                client.shutdown()
            except Exception:
                LOGGER.exception("NatNet shutdown failed")
        producer_done.set()
        worker.join(timeout=5.0)


def run_publisher(
    args: argparse.Namespace,
    client_factory: Callable[[str, str], object] = create_natnet_client,
    zenoh_module: Any = zenoh,
    stop_event: threading.Event | None = None,
) -> int:
    """Run the bridge until interrupted or the supplied stop event is set."""

    active_stop_event = stop_event or threading.Event()
    try:
        # client 模式连 zenohd Router:本机 zenohd 对 peer→router 不路由数据
        config = build_client_config(connect_endpoint=args.zenoh_endpoint)
        with zenoh_module.open(config) as session:
            return _run_with_session(
                args,
                session,
                client_factory,
                zenoh_module,
                active_stop_event,
            )
    except KeyboardInterrupt:
        active_stop_event.set()
        return 0
    except Exception:
        LOGGER.exception(
            "Publisher failed; check Zenoh endpoint %s and TCP connectivity",
            args.zenoh_endpoint,
        )
        return 3


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    return run_publisher(build_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
