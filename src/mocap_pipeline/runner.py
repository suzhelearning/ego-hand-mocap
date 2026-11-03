"""Restartable stage runner with content fingerprints and a single-session lock."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import time
import h5py
import numpy as np
from joint_capture.paths import ProjectPaths
from .adapters import ADAPTERS
from .comparison import export_main

STAGES=('vision','slam','mano','export')
OUTPUTS={'vision':('ego.h5','vision'),'slam':('ego.h5','okvis'),'mano':('mocap.h5','mano')}


def hash_file(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(4*1024*1024),b''):digest.update(chunk)
    return digest.hexdigest()


def hash_h5(path, keys):
    digest=hashlib.sha256()
    def value(v):
        a=np.asarray(v)
        digest.update(str((a.shape,str(a.dtype))).encode())
        if a.dtype.kind in 'OUS':digest.update(repr(a.tolist()).encode())
        else:digest.update(a.tobytes())
    def visit(name,obj):
        digest.update(name.encode())
        # Exclude whole-file hashes: unrelated stages modify the containing H5.
        for key in sorted(obj.attrs):
            if key not in ['input_file_sha256']:digest.update(key.encode());value(obj.attrs[key])
        if isinstance(obj,h5py.Dataset):
            if obj.ndim and obj.shape[0]:
                for start in range(0,obj.shape[0],128):value(obj[start:start+128])
            else:value(obj[()])
    with h5py.File(path,'r') as source:
        for key in keys:
            obj=source[key];visit(key,obj)
            if isinstance(obj,h5py.Group):obj.visititems(lambda n,x:visit(key+'/'+n,x))
    return digest.hexdigest()


def completed_output(session,stage,version):
    if not version:return None
    filename,family=OUTPUTS[stage];key=f'processing/{family}/{version}'
    required={'vision':['arrays/vertices_m','arrays/valid','arrays/faces','arrays/timestamp_ns'],
              'slam':['pose/timestamp_ns','pose/position','pose/quaternion_xyzw','pose/tracking_good','metadata/okvis.yaml'],
              'mano':['mesh/vertices','mesh/faces']}[stage]
    try:
        with h5py.File(session/filename,'r') as f:
            if key not in f or not f[key].attrs.get('complete',False):return None
            if any(name not in f[key] for name in required):return None
        return hash_h5(session/filename,[key])
    except (OSError,KeyError):return None


def fingerprint(root,session,profile,stage,selected):
    digest=hashlib.sha256()
    config={k:v for k,v in profile.items() if k not in ['existing_versions']}
    digest.update(json.dumps(config,sort_keys=True).encode());digest.update(stage.encode())
    for name in ['runner.py','adapters.py','comparison.py','reference.py','quality.py','pose_contract.py','slam_import.py']:
        digest.update(Path(__file__).with_name(name).read_bytes())
    if stage=='mano':
        for p in sorted(Path(__file__).with_name('workers').glob('*.py')):digest.update(p.read_bytes())
    if stage in ['vision','slam']:
        keys=['sensor/camera','calib']+(['sensor/imu'] if stage=='slam' else [])
        digest.update(hash_h5(session/'ego.h5',keys).encode())
        for side in ['left','right']:digest.update(hash_file(session/'video'/f'RGB_{side}.mp4').encode())
    elif stage=='mano':digest.update(hash_h5(session/'mocap.h5',['raw']).encode())
    else:
        for parent,version in selected.items():digest.update((completed_output(session,parent,version) or 'missing').encode())
        digest.update(hash_h5(session/'aligned.h5',['processing/world_preview/'+profile['body_run'],'processing/slam_registration/'+profile['registration_run']]).encode())
        digest.update(hash_h5(session/'mocap.h5',['raw']).encode())
        digest.update(hash_h5(session/'ego.h5',['sensor/camera/RGB_left/timestamp_ns']).encode())
        for p in sorted(Path(__file__).with_name('static').iterdir()):digest.update(p.read_bytes())
        from .device import device_inputs
        for p in device_inputs(root)[2]:digest.update(hash_file(p).encode())
    for item in profile.get('fingerprint_files',{}).get(stage,[]):
        path=Path(item);paths=sorted(p for p in path.rglob('*') if p.is_file() and p.suffix in ['.py','.json','.yaml','.yml','.sh','.cpp','.hpp','.h','.toml'] and '__pycache__' not in p.parts) if path.is_dir() else [path]
        for p in paths:digest.update(str(p).encode());digest.update(hash_file(p).encode())
    return digest.hexdigest()


def save_state(path,state):
    temp=path.with_suffix('.tmp');temp.write_text(json.dumps(state,ensure_ascii=False,indent=2));os.replace(temp,path)


def export_digest(web, session=None):
    names=['index.html','initialization.json','initial_body.bin','initial_vision.bin','meta.json','slam_mocap.bin','slam_ego_pose.bin','slam_ego_markers.bin','slam_ego_body.json','faces.bin','vision_faces.bin','sensor_frames.json','sensor_frames.js','three.module.min.js','OrbitControls.js']
    names+=['device.json','device.js']
    if (web/'device.json').is_file():
        device=json.loads((web/'device.json').read_text());names += [v['file'] for v in device['model']['visuals']]
        if device['available']:names += ['slam_ego_base.bin','mocap_ego_base.bin']
    if not all((web/n).is_file() for n in names):return None
    d=hashlib.sha256()
    for name in names:d.update(name.encode());d.update(hash_file(web/name).encode())
    if session is not None:
        report=json.loads((web/'initialization.json').read_text())
        version='initial-'+hashlib.sha256(json.dumps(report,sort_keys=True).encode()).hexdigest()[:16]
        key='processing/initialization/'+version
        try:
            with h5py.File(session/'aligned.h5','r') as f:
                if key not in f or not f[key].attrs.get('complete',False):return None
            d.update(hash_h5(session/'aligned.h5',[key]).encode())
        except (OSError,KeyError):return None
    return d.hexdigest()


def run_pipeline(root,session_id,profile_path,adopt_existing=False,force=(),until='export'):
    root=Path(root).resolve();session=ProjectPaths(root).session(session_id)
    if not (session/'session.json').is_file():raise FileNotFoundError(session/'session.json')
    profile=json.loads(Path(profile_path).read_text())
    if profile.get('session') not in [None,session_id]:raise ValueError('此配置绑定另一会话，拒绝混用')
    state_path=session/'pipeline.json';report=root/'reports'/('session_'+session_id);web=report/'world3d'
    report.mkdir(parents=True,exist_ok=True)
    with (session/'.pipeline.lock').open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise ValueError('该会话已有 pipeline 正在运行')
        state=json.loads(state_path.read_text()) if state_path.exists() else {'schema':1,'session':session_id,'stages':{}}
        selected={}
        for stage in STAGES:
            if stage=='mano' and profile.get('skip_mocap_hand',False):
                state['stages'][stage]={'status':'skipped','reason':'user: no mocap hand markers; visual hands retained'}
                save_state(state_path,state)
                if stage==until:break
                continue
            fp=fingerprint(root,session,profile,stage,selected);previous=state['stages'].get(stage,{})
            version=previous.get('version')
            current=export_digest(web,session) if stage=='export' else completed_output(session,stage,version)
            if stage not in force and previous.get('status')=='complete' and previous.get('fingerprint')==fp and current and current==previous.get('output_sha256'):
                print(f'[复用] {stage}: {version or web}',flush=True)
            elif stage!='export' and adopt_existing and not previous and stage not in force and completed_output(session,stage,profile.get('existing_versions',{}).get(stage)):
                version=profile['existing_versions'][stage]
                state['stages'][stage]={'status':'complete','fingerprint':fp,'version':version,'output_sha256':completed_output(session,stage,version),'origin':'adopted_existing','note':'当前输入下接管已有完整产物；非本次重新推理，历史计算输入关系未独立认证'}
                save_state(state_path,state);print(f'[接管已有结果] {stage}: {version}',flush=True)
            else:
                attempt=int(previous.get('attempt',0))+1;version=f'{stage}-{fp[:12]}-{attempt}'
                log_path=report/f'pipeline-{version}.log'
                record={'status':'running','fingerprint':fp,'version':version,'attempt':attempt,'log':str(log_path),'started_at':time.time(),'origin':'executed'}
                state['stages'][stage]=record;save_state(state_path,state)
                print(f'[运行] {stage}，日志：{log_path}',flush=True)
                try:
                    with log_path.open('wb') as log:
                        if stage=='export':
                            args=['--root',str(root),'--session',session_id,'--slam-run',selected['slam'],'--vision-run',selected['vision'],'--registration-run',profile['registration_run'],'--body-run',profile['body_run']]
                            if profile.get('allow_unregistered_preview'):args.append('--allow-unregistered-preview')
                            if profile.get('skip_mocap_hand'):args.append('--skip-mocap-hand')
                            else:args.extend(['--mano-run',selected['mano']])
                            for key,value in profile.get('quality',{}).items():args.extend(['--'+key.replace('_','-'),str(value)])
                            export_main(args)
                        else:ADAPTERS[stage](root,session,profile,version,log)
                    result=export_digest(web,session) if stage=='export' else completed_output(session,stage,version)
                    if not result:raise ValueError('命令退出但输出契约未满足')
                    record.update(status='complete',output_sha256=result,finished_at=time.time());save_state(state_path,state)
                except BaseException as exc:
                    record.update(status='failed',error=str(exc),finished_at=time.time());save_state(state_path,state);raise
            if stage!='export':selected[stage]=version
            if stage==until:break
    return state
