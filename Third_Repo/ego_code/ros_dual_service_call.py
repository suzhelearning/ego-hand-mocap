#!/usr/bin/env python3
"""
ros_dual_service_call.py — 用同一个 rclpy 进程并发调用多个 std_srvs/Trigger 服务。
避免多个独立 `ros2 service call` CLI 各自做 DDS 发现造成 >1s 错位。

用法:
    ros_dual_service_call.py <svc_a> <svc_b> [svc_c ...]

输出一行一条 (顺序和参数一致):
    <svc>|success=<True/False>|message=<...>

退出码:
    0  所有调用已返回(不论 success 字段)
    2  任一 service 不可用或超时
"""
import sys

import rclpy
from std_srvs.srv import Trigger


def main() -> int:
    if len(sys.argv) < 3:
        print("usage: ros_dual_service_call.py <svc_a> <svc_b> [svc_c ...]", file=sys.stderr)
        return 2

    services = sys.argv[1:]

    rclpy.init()
    node = rclpy.create_node("_dual_trigger_caller")

    clients = [(svc, node.create_client(Trigger, svc)) for svc in services]

    # 等服务可用(共享同一 DDS 发现)
    for svc, cli in clients:
        if not cli.wait_for_service(timeout_sec=5.0):
            print(f"{svc}|success=False|message=service unavailable")
            rclpy.shutdown()
            return 2

    futures = [(svc, cli.call_async(Trigger.Request())) for svc, cli in clients]

    # 等所有 future 都完成(共享事件循环,近似同时发出请求)
    deadline = 5.0
    spun = 0.0
    step = 0.02
    while not all(fut.done() for _, fut in futures) and spun < deadline:
        rclpy.spin_once(node, timeout_sec=step)
        spun += step

    def fmt(svc: str, fut) -> str:
        if not fut.done():
            return f"{svc}|success=False|message=timeout"
        r = fut.result()
        msg = (r.message or "").replace("|", "/")
        return f"{svc}|success={r.success}|message={msg}"

    for svc, fut in futures:
        print(fmt(svc, fut))

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
