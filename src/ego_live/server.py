"""Local HTTP viewer; read-only NatNet connection, bounded latest-frame state."""
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import mimetypes
from pathlib import Path
import threading
import time
import numpy as np
from joint_capture.natnet import NatNetStream
from joint_capture.clock import local_address
from joint_capture.cli import network_defaults
from .model import installation, configured_installation, load_urdf, visible_pose, world_base

ROOT=Path(__file__).resolve().parents[2]


class LiveState:
    def __init__(self,name,body_id,offset,world_up):
        self.name=name;self.body_id=body_id;self.offset=offset
        self.lock=threading.Lock();self.frame=None;self.names={};self.error=None
        self.world=np.eye(4)
        if world_up=='y':self.world[:3,:3]=[[1,0,0],[0,0,-1],[0,1,0]]

    def push(self,frame,names):
        with self.lock:self.frame=frame;self.names=dict(names);self.error=None

    def snapshot(self):
        with self.lock:frame=self.frame;names=dict(self.names);error=self.error
        result=dict(valid=False,status=error or '等待 NatNet 数据',available_bodies=names)
        if frame is None:return result
        age=(time.monotonic_ns()-frame['publisher_received_time_ns'])/1e9
        bodies=[b for b in frame['rigid_bodies'] if (b['id']==self.body_id if self.body_id is not None else names.get(b['id'],'').casefold()==self.name.casefold())]
        result.update(frame_number=frame['frame_number'],age_s=age,motive_timestamp=frame['motive_timestamp'])
        if len(bodies)!=1:
            result['status']='目标刚体未找到或名字不唯一；检查名称或使用 --body-id';return result
        b=bodies[0];result.update(body_id=b['id'],body_name=names.get(b['id']),mean_error_m=b['mean_error'])
        result['valid']=visible_pose(b['tracking_valid'],age) and error is None
        result['status']='实时跟踪' if result['valid'] else (error or ('数据中断' if age>.5 else '刚体丢跟踪'))
        if result['valid']:
            wb=self.world@world_base(b['position'],b['quaternion_xyzw'],np.eye(4))
            result.update(T_WB=wb.tolist(),T_Wbase=(wb@self.offset).tolist())
        return result

class LiveStream(NatNetStream):
    def __init__(self,*args,state,**kwargs):super().__init__(*args,**kwargs);self.state=state
    def _on_frame(self,data):
        # Use the existing parser but never enable its recording queue.
        from natnet_zenoh.natnet_adapter import convert_frame
        try:
            now=time.monotonic_ns();frame=convert_frame(data,now)
            self.last_received=now;self.state.push(frame,self.names)
            if frame['rigid_bodies']:self.ready.set()
        except Exception as exc:
            self.error=str(exc);self.ready.set()


def main(argv=None):
    p=argparse.ArgumentParser(prog='mocap live-device',description='NatNet EGO 设备实时三维显示（不录制）')
    p.add_argument('--config',type=Path,default=ROOT/'config/ego_device_geometry.json')
    p.add_argument('--motive-server-ip',default=network_defaults().get('motive_server_ip','169.254.213.247'))
    p.add_argument('--natnet-client-ip');p.add_argument('--body-name');p.add_argument('--body-id',type=int)
    p.add_argument('--world-up',choices=['z','y'],help='NatNet 输入世界的向上轴，页面始终 Z-up')
    p.add_argument('--port',type=int,default=8768)
    args=p.parse_args(argv);config=json.loads(args.config.read_text())
    urdf=(ROOT/config['urdf']).resolve();model=load_urdf(urdf)
    offset=configured_installation(config,args.world_up)
    state=LiveState(args.body_name or config['rigid_body_name'],args.body_id,offset,args.world_up or config['natnet_world_up'])
    manifest=dict(model=model,config=config,T_Bbase=offset.tolist(),world_up=args.world_up or config['natnet_world_up'])
    files={'/':Path(__file__).parent/'static/index.html','/app.js':Path(__file__).parent/'static/app.js'}
    for name in ('three.module.min.js','OrbitControls.js'):
        files['/'+name]=ROOT/'src/motion_web/static/vendor'/name
    for visual in model['visuals']:files['/model/'+visual['file']]=urdf.parent/visual['file']
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path=self.path.split('?')[0]
            if path in ('/api/state','/api/model'):
                data=json.dumps(state.snapshot() if path.endswith('state') else manifest,allow_nan=False).encode();mime='application/json'
            elif path in files:
                data=files[path].read_bytes();mime=mimetypes.guess_type(str(files[path]))[0] or 'application/octet-stream'
            else:self.send_error(404);return
            self.send_response(200);self.send_header('Content-Type',mime);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)));self.end_headers()
            try:self.wfile.write(data)
            except (BrokenPipeError,ConnectionResetError):pass
        def log_message(self,*args):pass
    http=ThreadingHTTPServer(('127.0.0.1',args.port),Handler);stop=threading.Event()
    def receive():
        while not stop.is_set():
            stream=None
            try:
                stream=LiveStream(args.motive_server_ip,args.natnet_client_ip or local_address(args.motive_server_ip),timeout=5,state=state)
                stream.start()
                while not stop.wait(.2):stream.check()
            except Exception as exc:
                with state.lock:state.error='NatNet 连接异常：'+str(exc)
                print(state.error,flush=True)
            finally:
                if stream:stream.close()
            stop.wait(2)
    worker=threading.Thread(target=receive,daemon=True);worker.start()
    print(f'设备实时页面：http://127.0.0.1:{args.port}/；Ctrl+C 停止；不启动录制',flush=True)
    try:http.serve_forever()
    except KeyboardInterrupt:pass
    finally:stop.set();http.server_close();worker.join(timeout=10)
    return 0
