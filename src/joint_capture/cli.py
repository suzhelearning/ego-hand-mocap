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


def parser():
    defaults=network_defaults()
    p=argparse.ArgumentParser(prog='mocap record',description='协调录制Ego及Ubuntu动捕H5；不插值、不修改曝光')
    p.add_argument('--duration',type=positive,default=None,help='录制秒数；省略则按Ctrl+C停止并保存')
    p.add_argument('--output-root',type=Path,default=PROJECT_ROOT,help='输出项目根目录；会话数据写入data/sessions/<session_id>/')
    p.add_argument('--session-id',help='可选会话名；已有文件绝不覆盖')
    p.add_argument('--ego-serial',help='ADB设备序列号；默认要求仅连接一个设备')
    p.add_argument('--ego-ip',default=defaults.get('ego_ip','192.168.100.1'),help='Ego USB网络IP')
    p.add_argument('--motive-server-ip',default=defaults.get('motive_server_ip','169.254.213.247'))
    p.add_argument('--natnet-client-ip',default=None,help='默认按到Motive的路由自动选择')
    p.add_argument('--windows-host',default=defaults.get('windows_host','windows-pc'),help='已配置SSH的Windows主机名，用于对时')
    p.add_argument('--no-clock-probes',action='store_true',help='明确跳过对时，只保存原始采集时间；不推荐正式同步采集使用')
    p.add_argument('--startup-timeout',type=positive,default=15)
    p.add_argument('--export-timeout',type=positive,default=3600,help='解密/复制超时秒数；长录制可增大，默认3600')
    p.set_defaults(warmup=1.0,tail=.75)
    return p


def main(argv=None):
    args=parser().parse_args(argv);stop=threading.Event()
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
