"""Unified read-only inspection and comparison export entry points."""
import argparse
import json
from pathlib import Path
import sys
import subprocess
import signal
from .inspection import inspect_session


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    try:
        if args[:1] == ['export']:
            from .comparison import export_main
            export_main(args[1:])
            return 0
        parser = argparse.ArgumentParser(description='会话流程：status 检查依赖，export 生成统一对比')
        sub = parser.add_subparsers(dest='command', required=True)
        status = sub.add_parser('status', help='只读检查会话依赖与处理版本')
        status.add_argument('--session', required=True)
        status.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
        status.add_argument('--slam-run');status.add_argument('--vision-run')
        status.add_argument('--json', action='store_true')
        run=sub.add_parser('run',help='依次运行视觉、SLAM、MANO 与对齐导出，按指纹续跑')
        run.add_argument('--session',required=True);run.add_argument('--profile',type=Path,required=True)
        run.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[2])
        run.add_argument('--adopt-existing',action='store_true',help='首次显式接管 profile 指定的已有完整 H5 结果，不宣称重新计算')
        run.add_argument('--force',action='append',default=[],choices=['vision','slam','mano','export'])
        run.add_argument('--until',default='export',choices=['vision','slam','mano','export'])
        sub.add_parser('export', help='生成固定初始化的统一对比；export --help 查看参数')
        a = parser.parse_args(args)
        if a.command=='run':
            from .runner import run_pipeline
            def interrupted(signum,frame):raise KeyboardInterrupt(f'signal {signum}')
            previous=signal.signal(signal.SIGTERM,interrupted)
            try:run_pipeline(a.root,a.session,a.profile,a.adopt_existing,a.force,a.until)
            finally:signal.signal(signal.SIGTERM,previous)
            return 0
        result = inspect_session(a.root, a.session, a.slam_run, a.vision_run)
        if a.json:
            print(json.dumps(result, ensure_ascii=False, indent=2))
        else:
            for check in result['checks']:
                print(f"[{'就绪' if check['ready'] else '缺失'}] {check['stage']}: {check['detail']}")
            for note in result['limitations']:print(note)
        return 0 if result['ready'] else 1
    except KeyboardInterrupt:
        print('[pipeline] 已中断；保留阶段状态，下次运行将续跑。',file=sys.stderr)
        return 130
    except (ValueError, KeyError, OSError, subprocess.CalledProcessError) as exc:
        print(f'[pipeline] {exc}', file=sys.stderr)
        return 1
