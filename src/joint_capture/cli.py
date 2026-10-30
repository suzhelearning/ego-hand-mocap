"""Command-line entry for coordinated Ego + NatNet raw capture."""
import argparse
import json
import math
from pathlib import Path
import signal
import threading
from .session import run_session

PROJECT_ROOT=Path(__file__).resolve().parents[2]
NETWORK_CONFIG=PROJECT_ROOT/'configs'/'network'/'lab.json'


def network_defaults():
    if NETWORK_CONFIG.is_file():
        return json.loads(NETWORK_CONFIG.read_text(encoding='utf8'))
    return {}


def positive(value):
    n=float(value)
    if not math.isfinite(n) or n<=0:raise argparse.ArgumentTypeError('必须是大于0的有限秒数')
    return n


class CaptureParser(argparse.ArgumentParser):
    def parse_args(self, args=None, namespace=None):
        result=super().parse_args(args,namespace)
        result.mode=result.mode or ('h5' if result.calibration else 'take')
        if result.calibration:
            if result.mode!='h5':self.error('--calibration 仅支持 --mode h5')
            if result.duration is not None:self.error('标定流程自动控制时长，不能同时指定 --duration')
            if result.no_clock_probes:self.error('标定录制需要对时，不能使用 --no-clock-probes')
        elif result.preview_calibration:self.error('--preview-calibration 需要 --calibration rbc 或 tbc')
        return result


def parser():
    defaults=network_defaults()
    p=CaptureParser(prog='mocap record',description='两种联合采集：EGO+Ubuntu H5 或 EGO+Windows Take')
    p.add_argument('--mode',choices=['h5','take'],default=None,help='采集方式，默认take；指定--calibration时默认h5')
    p.add_argument('--calibration',type=str.lower,choices=['rbc','tbc'],help='旋转或平移标定动作引导，完成后自动保存')
    p.add_argument('--preview-calibration',action='store_true',help='仅显示标定动作与时长，不连接设备或录制')
    p.add_argument('--windows-take-root',help='Windows保存根目录，默认远端 ~/Desktop/syz/ego-hand/remote；例如 D:\\MocapSessions')
    p.add_argument('--duration',type=positive,default=None,help='录制秒数；省略则按Ctrl+C停止并保存')
    p.add_argument('--output-root',type=Path,default=PROJECT_ROOT,help='输出项目根目录；会话数据写入data/sessions/<session_id>/')
    p.add_argument('--session-id',help='可选会话名；已有文件绝不覆盖')
    p.add_argument('--ego-serial',help='ADB设备序列号；默认要求仅连接一个设备')
    p.add_argument('--ego-ip',default=defaults.get('ego_ip','192.168.100.1'),help='Ego USB网络IP')
    p.add_argument('--motive-server-ip',default=defaults.get('motive_server_ip','169.254.213.247'))
    p.add_argument('--natnet-client-ip',default=None,help='默认按到Motive的路由自动选择')
    p.add_argument('--windows-host',default=defaults.get('windows_host','windows-pc'),help='Windows SSH主机名，用于对时及Take文件核验')
    p.add_argument('--no-clock-probes',action='store_true',help='明确跳过对时，只保存原始采集时间；不推荐正式同步采集使用')
    p.add_argument('--startup-timeout',type=positive,default=15)
    p.add_argument('--export-timeout',type=positive,default=3600,help='EGO导出及Windows Take文件核验超时秒数，默认3600')
    p.set_defaults(warmup=1.0,tail=.75)
    return p


def main(argv=None):
    args=parser().parse_args(argv);stop=threading.Event()
    if args.preview_calibration:
        from .guidance import build_plan
        plan=build_plan(args.calibration)
        print(f"{args.calibration.upper()} 标定流程，计划 {sum(s['duration_s'] for s in plan)} 秒；动作时间仅为提示，不代表检测到实际动作。")
        for i,step in enumerate(plan,1):print(f"{i:02d}. {step['instruction']}（{step['duration_s']} 秒）")
        return 0
    def request_stop(signum,frame):
        if not stop.is_set():print('\n收到停止请求，正在保存；请等待导出完成。',flush=True)
        stop.set()
    previous={s:signal.signal(s,request_stop) for s in (signal.SIGINT,signal.SIGTERM)}
    try:
        result=run_session(args,stop)
        return 0 if result['status']=='complete' else 1
    except Exception as e:
        print('采集未启动或失败：'+str(e),flush=True);return 1
    finally:
        for sig,handler in previous.items():signal.signal(sig,handler)
