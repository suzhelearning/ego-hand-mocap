#!/usr/bin/env python3
"""
record_ctrl_bridge.py

对齐原版 exoskeleton_v2/hyper_leg_ros2/scripts/record_ctrl/record_ctrl_ros2_bridge.py
外部按键设备(通常是 RK3576 quad_cam_streamer)通过 ZMQ PUB 广播按钮事件,
本脚本作为 ROS 2 中继订阅 ZMQ,按事件触发单节点 session_recorder 录制:

    is_recording = 1  →  /session_recorder/start (带 seq 时走 start_with_seq)
    is_recording = 0  →  /session_recorder/stop

ZMQ 消息格式 (little-endian):
    legacy:
      [uint64_t timestamp_ns (8B)][uint8_t is_recording (1B)]
    current:
      [uint64_t timestamp_ns (8B)][uint8_t is_recording (1B)][uint32_t seq_num (4B)]

时间偏移(远端 CLOCK_MONOTONIC - 本地 CLOCK_MONOTONIC, ms)以 60Hz 发到
/ble_time_offset(话题名沿用原版,方便 downstream 复用)。

Usage:
  # ROS 2 env
  source /opt/ros/humble/setup.bash
  source ~/catkin_exoskeleton_ws/install/setup.bash
  python3 scripts/record_ctrl_zmq_bridge.py --host 192.168.100.1 --port 5565

前置:
  session_recorder_node 必须已运行(start_full_body.sh 默认已启)。
"""
import argparse
import socket
import struct
import threading
import time

import zmq
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import UInt32
from std_srvs.srv import Trigger
from exo_msgs.srv import StartRecorderWithSeq


KEY_EVENT_MSG_SIZE_LEGACY = 9   # uint64 timestamp_ns + uint8 is_recording
KEY_EVENT_MSG_SIZE_V2 = 13      # uint64 timestamp_ns + uint8 is_recording + uint32 seq_num
SEQ_PROPAGATION_DELAY_SEC = 0.05
UINT32_MAX_PLUS_ONE = 2**32
EVENT_MODES = ("auto", "state", "toggle")
ZMQ_POLL_TIMEOUT_MS = 1000
ZMQ_TCP_CONNECT_TIMEOUT_SEC = 1.0
ZMQ_RECONNECT_CHECK_SEC = 5.0
ZMQ_STALE_RECONNECT_SEC = 120.0


class HeadsetHealthMonitor:
    """Low-overhead headset connection probe, surfaced in the bridge tmux window."""

    def __init__(
        self,
        node: Node,
        host: str,
        key_port: int,
        interval_sec: float,
    ):
        self.node = node
        self.host = host
        self.key_port = int(key_port)
        self.interval_sec = max(5.0, float(interval_sec))
        self._lock = threading.Lock()
        self._last_ok = None

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def check_async(self, reason: str):
        threading.Thread(target=self._check_locked, args=(reason,), daemon=True).start()

    def _loop(self):
        time.sleep(3.0)
        while rclpy.ok():
            self._check_locked("periodic")
            deadline = time.monotonic() + self.interval_sec
            while rclpy.ok() and time.monotonic() < deadline:
                time.sleep(0.2)

    def _check_locked(self, reason: str):
        if not self._lock.acquire(blocking=False):
            return
        try:
            self.check_once(reason)
        finally:
            self._lock.release()

    def check_once(self, reason: str):
        tcp_ok, tcp_msg = self._check_tcp()
        status = "OK" if tcp_ok else "BAD"
        message = (
            f"[HEADSET] {status} reason={reason} "
            f"tcp={self.host}:{self.key_port} {tcp_msg}"
        )
        if tcp_ok:
            self.node.get_logger().info(message)
        else:
            self.node.get_logger().error(message)
        self._last_ok = tcp_ok

    def _check_tcp(self):
        try:
            with socket.create_connection((self.host, self.key_port), timeout=1.5):
                return True, "ok"
        except OSError as exc:
            return False, str(exc)


class RecordCtrlZmqBridge(Node):
    def __init__(
        self,
        start: str = "/session_recorder/start",
        stop: str = "/session_recorder/stop",
        start_with_seq: str = "/session_recorder/start_with_seq",
        service_wait_sec: float = 10.0,
    ):
        super().__init__("record_ctrl_zmq_bridge")

        self.start_cli = self.create_client(Trigger, start)
        self.stop_cli = self.create_client(Trigger, stop)
        self.start_seq_cli = self.create_client(StartRecorderWithSeq, start_with_seq)

        self.offset_pub = self.create_publisher(Float64, "/ble_time_offset", 10)
        self.seq_pub = self.create_publisher(UInt32, "/record_seq_num", 10)
        self.latest_offset_ms = None
        self.headset_health_monitor = None
        self.create_timer(1.0 / 60.0, self._publish_offset)

        if service_wait_sec > 0.0:
            self.get_logger().info(
                f"Waiting for session_recorder services ({service_wait_sec:.1f}s each)..."
            )
            for cli, name in [
                (self.start_cli, start),
                (self.stop_cli, stop),
                (self.start_seq_cli, start_with_seq),
            ]:
                if not cli.wait_for_service(timeout_sec=service_wait_sec):
                    self.get_logger().warn(
                        f"Service {name} not ready after {service_wait_sec:.1f}s — will call anyway"
                    )
        else:
            self.get_logger().warn(
                "Skipping session_recorder service wait; useful for camera/ZMQ signal tests."
            )
        self.get_logger().info(
            "Ready. /ble_time_offset @ 60Hz; triggers session_recorder on each event."
        )

    def _publish_offset(self):
        if self.latest_offset_ms is None:
            return
        msg = Float64()
        msg.data = float(self.latest_offset_ms)
        self.offset_pub.publish(msg)

    def update_offset(self, offset_ms: float):
        self.latest_offset_ms = offset_ms

    def publish_seq(self, seq_num: int):
        msg = UInt32()
        msg.data = int(seq_num)
        self.seq_pub.publish(msg)

    def _log_result(self, label: str, future):
        """Log a service result, escalating failures to ERROR so empty/refused
        recordings (e.g. keypoints stream not live) are not buried in INFO."""
        try:
            res = future.result()
        except Exception as e:  # noqa: BLE001 — surface any call failure loudly
            self.get_logger().error(f"{label} → call failed: {e}")
            return
        if res.success:
            self.get_logger().info(f"{label} → success=True {res.message}")
        else:
            self.get_logger().error(f"{label} → FAILED: {res.message}")

    def _async_call(self, cli, label: str):
        """Fire-and-forget, log result in callback."""
        future = cli.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self._log_result(label, f))

    def call_start_all(self):
        self._async_call(self.start_cli, "[session start]")

    def _async_call_start_with_seq(self, cli, seq_num: int, label: str):
        req = StartRecorderWithSeq.Request()
        req.sequence_number = int(seq_num)
        future = cli.call_async(req)
        future.add_done_callback(lambda f: self._log_result(f"{label} seq={seq_num}", f))

    def call_start_all_with_seq(self, seq_num: int):
        if self.start_seq_cli.service_is_ready():
            self._async_call_start_with_seq(self.start_seq_cli, seq_num, "[session start_with_seq]")
        else:
            # 退路:seq 走 /record_seq_num 话题,留一点传播时间再普通 start
            self.publish_seq(seq_num)
            time.sleep(SEQ_PROPAGATION_DELAY_SEC)
            self.call_start_all()

    def call_stop_all(self):
        self._async_call(self.stop_cli, "[session stop ]")

    def set_headset_health_monitor(self, monitor: HeadsetHealthMonitor):
        self.headset_health_monitor = monitor

    def check_headset_async(self, reason: str):
        if self.headset_health_monitor is not None:
            self.headset_health_monitor.check_async(reason)


def _make_zmq_sub(ctx, addr: str):
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.LINGER, 0)
    sub.setsockopt(zmq.RCVHWM, 10)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    # Make half-open TCP links fail faster when the headset loses power.
    for opt_name, value in (
        ("RECONNECT_IVL", 1000),
        ("RECONNECT_IVL_MAX", 5000),
        ("TCP_KEEPALIVE", 1),
        ("TCP_KEEPALIVE_IDLE", 10),
        ("TCP_KEEPALIVE_INTVL", 3),
        ("TCP_KEEPALIVE_CNT", 3),
    ):
        opt = getattr(zmq, opt_name, None)
        if opt is not None:
            sub.setsockopt(opt, value)
    sub.connect(addr)
    return sub


def _tcp_reachable(host: str, port: int):
    try:
        with socket.create_connection(
            (host, int(port)),
            timeout=ZMQ_TCP_CONNECT_TIMEOUT_SEC,
        ):
            return True, "ok"
    except OSError as exc:
        return False, str(exc)


def zmq_loop(
    node: RecordCtrlZmqBridge,
    host: str,
    port: int,
    event_mode: str = "auto",
    reconnect_check_sec: float = ZMQ_RECONNECT_CHECK_SEC,
    stale_reconnect_sec: float = ZMQ_STALE_RECONNECT_SEC,
):
    addr = f"tcp://{host}:{port}"
    ctx = zmq.Context()
    poller = zmq.Poller()
    sub = None

    toggle_recording = False
    generated_seq = time.time_ns() % UINT32_MAX_PLUS_ONE
    active_toggle_seq = None
    auto_locked_mode = None
    last_rx_ts = time.monotonic()
    last_tcp_check_ts = 0.0
    last_tcp_ok = None
    reconnect_count = 0
    reconnect_check_sec = max(1.0, float(reconnect_check_sec))
    stale_reconnect_sec = max(0.0, float(stale_reconnect_sec))

    def close_socket():
        nonlocal sub
        if sub is None:
            return
        try:
            poller.unregister(sub)
        except (KeyError, zmq.ZMQError):
            pass
        sub.close(0)
        sub = None

    def recreate_socket(reason: str):
        nonlocal sub, reconnect_count, last_rx_ts
        close_socket()
        sub = _make_zmq_sub(ctx, addr)
        poller.register(sub, zmq.POLLIN)
        reconnect_count += 1
        last_rx_ts = time.monotonic()
        node.get_logger().warn(
            f"[ZMQ] Recreated key-event SUB socket reason={reason} "
            f"count={reconnect_count} addr={addr}"
        )

    recreate_socket("initial")
    node.get_logger().info(
        f"Listening for key events on {addr} (event_mode={event_mode}, "
        f"reconnect_check={reconnect_check_sec:.1f}s, "
        f"stale_reconnect={stale_reconnect_sec:.1f}s)"
    )

    while rclpy.ok():
        now = time.monotonic()
        if now - last_tcp_check_ts >= reconnect_check_sec:
            last_tcp_check_ts = now
            tcp_ok, tcp_msg = _tcp_reachable(host, port)
            if not tcp_ok:
                if last_tcp_ok is not False:
                    node.get_logger().error(
                        f"[ZMQ] Headset key port {host}:{port} unreachable: {tcp_msg}; "
                        "waiting for reconnect"
                    )
                last_tcp_ok = False
                close_socket()
                continue
            if last_tcp_ok is False:
                node.get_logger().warn(
                    f"[ZMQ] Headset key port {host}:{port} reachable again; "
                    "reconnecting SUB socket"
                )
                recreate_socket("tcp_recovered")
                node.check_headset_async("after ZMQ reconnect")
            elif sub is None:
                recreate_socket("tcp_available")
            elif stale_reconnect_sec > 0.0 and now - last_rx_ts >= stale_reconnect_sec:
                recreate_socket(f"stale_no_messages_{stale_reconnect_sec:.0f}s")
            last_tcp_ok = True

        try:
            events = dict(poller.poll(ZMQ_POLL_TIMEOUT_MS))
        except zmq.ContextTerminated:
            return
        except Exception as e:  # noqa: BLE001
            node.get_logger().warn(f"ZMQ poll error: {e}")
            recreate_socket("poll_error")
            continue

        if sub is None or sub not in events:
            continue

        try:
            data = sub.recv(flags=zmq.NOBLOCK)
        except zmq.Again:
            continue
        except zmq.ContextTerminated:
            return
        except Exception as e:  # noqa: BLE001
            node.get_logger().warn(f"ZMQ recv error: {e}")
            recreate_socket("recv_error")
            continue

        last_rx_ts = time.monotonic()
        last_tcp_ok = True

        if len(data) == KEY_EVENT_MSG_SIZE_LEGACY:
            remote_ts_ns = struct.unpack("<Q", data[:8])[0]
            is_recording = data[8] != 0
            seq_num = None
        elif len(data) == KEY_EVENT_MSG_SIZE_V2:
            remote_ts_ns, is_recording_u8, seq_num = struct.unpack("<QBI", data)
            is_recording = is_recording_u8 != 0
        else:
            node.get_logger().warn(
                f"Unexpected message size: {len(data)} bytes "
                f"(expected {KEY_EVENT_MSG_SIZE_LEGACY} or {KEY_EVENT_MSG_SIZE_V2}), skipping"
            )
            continue

        local_ts_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        offset_ms = (local_ts_ns - remote_ts_ns) / 1e6
        node.update_offset(offset_ms)

        raw_is_recording = is_recording
        raw_seq_num = seq_num
        effective_event_mode = event_mode
        if event_mode == "auto":
            if auto_locked_mode is None:
                auto_locked_mode = (
                    "toggle" if seq_num == 0 and not raw_is_recording else "state"
                )
                node.get_logger().info(
                    f"Auto event mode locked to {auto_locked_mode} "
                    f"(first raw_rec={int(raw_is_recording)} raw_seq={raw_seq_num})"
                )
            effective_event_mode = auto_locked_mode

        if effective_event_mode == "toggle":
            toggle_recording = not toggle_recording
            is_recording = toggle_recording
            if is_recording:
                generated_seq = (generated_seq + 1) % UINT32_MAX_PLUS_ONE
                seq_num = generated_seq
                active_toggle_seq = seq_num
            else:
                seq_num = active_toggle_seq

        seq_text = f" seq={seq_num}" if seq_num is not None else ""
        mode_text = (
            f" mode={effective_event_mode}"
            f"{' requested=auto' if event_mode == 'auto' else ''}"
            f" raw_rec={int(raw_is_recording)} raw_seq={raw_seq_num}"
        )
        if is_recording:
            if seq_num is not None:
                node.call_start_all_with_seq(seq_num)
            else:
                node.call_start_all()
            node.get_logger().info(
                f"[KEY] START remote_ts={remote_ts_ns}{seq_text} "
                f"offset={offset_ms:+.1f}ms{mode_text}"
            )
            node.check_headset_async(f"after START{seq_text}")
        else:
            # 先发 stop 再打日志,缩短按键→停录间隔
            node.call_stop_all()
            node.get_logger().info(
                f"[KEY] STOP  remote_ts={remote_ts_ns}{seq_text} "
                f"offset={offset_ms:+.1f}ms{mode_text}"
            )
            node.check_headset_async(f"after STOP{seq_text}")


def main():
    parser = argparse.ArgumentParser(
        description="ZMQ key-event → ROS 2 relay (session_recorder 录制触发器)"
    )
    parser.add_argument(
        "--host",
        default="192.168.100.1",
        help="Remote machine IP (default: 192.168.100.1)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5565,
        help="Remote key-event ZMQ port (default: 5565)",
    )
    parser.add_argument(
        "--start-service",
        default="/session_recorder/start",
        help="session recorder start service",
    )
    parser.add_argument(
        "--stop-service",
        default="/session_recorder/stop",
        help="session recorder stop service",
    )
    parser.add_argument(
        "--start-with-seq-service",
        default="/session_recorder/start_with_seq",
        help="session recorder start_with_seq service",
    )
    parser.add_argument(
        "--event-mode",
        choices=EVENT_MODES,
        default="auto",
        help=(
            "auto: old driver uses payload state, new driver with rec_u8=0 seq=0 "
            "uses local toggle + generated seq. "
            "state: force is_recording byte. "
            "toggle: force alternating START/STOP."
        ),
    )
    parser.add_argument(
        "--service-wait-sec",
        type=float,
        default=10.0,
        help="Seconds to wait for each recorder service before listening to ZMQ; 0 skips waiting.",
    )
    parser.add_argument(
        "--zmq-reconnect-check-sec",
        type=float,
        default=ZMQ_RECONNECT_CHECK_SEC,
        help="Seconds between headset TCP reachability checks for ZMQ auto-reconnect.",
    )
    parser.add_argument(
        "--zmq-stale-reconnect-sec",
        type=float,
        default=ZMQ_STALE_RECONNECT_SEC,
        help="Recreate the ZMQ SUB socket after this many seconds without key packets; 0 disables.",
    )
    parser.add_argument(
        "--headset-health",
        dest="headset_health",
        action="store_true",
        default=True,
        help="Periodically print headset key-port TCP connectivity in this bridge window.",
    )
    parser.add_argument(
        "--no-headset-health",
        dest="headset_health",
        action="store_false",
        help="Disable headset health checks.",
    )
    parser.add_argument(
        "--headset-health-interval-sec",
        type=float,
        default=30.0,
        help="Seconds between headset TCP connectivity checks.",
    )
    args = parser.parse_args()

    rclpy.init()
    node = RecordCtrlZmqBridge(
        start=args.start_service,
        stop=args.stop_service,
        start_with_seq=args.start_with_seq_service,
        service_wait_sec=args.service_wait_sec,
    )

    if args.headset_health:
        monitor = HeadsetHealthMonitor(
            node=node,
            host=args.host,
            key_port=args.port,
            interval_sec=args.headset_health_interval_sec,
        )
        node.set_headset_health_monitor(monitor)
        monitor.start()

    threading.Thread(
        target=zmq_loop,
        args=(
            node,
            args.host,
            args.port,
            args.event_mode,
            args.zmq_reconnect_check_sec,
            args.zmq_stale_reconnect_sec,
        ),
        daemon=True,
    ).start()

    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
