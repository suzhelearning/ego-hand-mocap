"""保留旧仓库命令名称，只提供本项目迁移的模块。"""
import sys


def main():
    args = sys.argv[1:]
    if args[:1] == ["record"]:
        from joint_capture.cli import main as run
        return run(args[1:])
    if args[:1] == ["visualize"]:
        from motion_web.server import main as run
        return run(args[1:])
    if args[:2] == ["publish", "mocap"]:
        from natnet_zenoh.publisher import main as run
        sys.argv = [sys.argv[0], *args[2:]]
        return run()
    if args[:2] == ["subscribe", "mocap"]:
        from natnet_zenoh.subscriber import main as run
        sys.argv = [sys.argv[0], *args[2:]]
        return run()
    if args[:2] == ["calibrate", "ego"]:
        from mocap_tools.calibrate_ego import main as run
        return run(args[2:])
    if args[:1] == ["track-rigid"]:
        from mocap_tools.track_rigid import main as run
        return run(args[1:])
    print("用法: mocap {record | publish mocap | subscribe mocap | track-rigid | calibrate ego | visualize} [参数]")
    return 0 if not args or args == ["--help"] else 2
