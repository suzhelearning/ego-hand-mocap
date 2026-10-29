"""Executed on Ego through ADB. Never prints encryption keys or passwords."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = Path('/mnt/sdcard/SD')
OWNER = Path('/tmp/ego_joint_capture_owner.json')


def control(payload):
    import zmq
    context=zmq.Context();s=context.socket(zmq.REQ)
    s.setsockopt(zmq.RCVTIMEO,8000);s.setsockopt(zmq.LINGER,0)
    s.connect('tcp://127.0.0.1:5570')
    try:
        s.send_json(payload);reply=s.recv_json()
        if reply.get('status')!='ok':raise RuntimeError('Ego control rejected: '+json.dumps(reply))
        return reply
    finally:s.close();context.term()


def record_name(name):
    if not re.fullmatch(r'head_[0-9]+',name):raise ValueError('Invalid device recording name')
    return name


def recording_identity(root):
    paths=list(root.glob('.head_*.recording'))
    if len(paths)!=1:return None
    p=paths[0]
    return {'directory':p.name,'inode':p.stat().st_ino}


def record_action(action,token,state_path=OWNER,root=ROOT):
    with open(str(state_path)+'.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        state=json.loads(state_path.read_text()) if state_path.exists() else {}
        if action=='start':
            if control({'query':True})['recording']['active']:
                raise RuntimeError('Ego already recording; refusing to change it')
            if list(root.glob('.head_*.recording')):
                raise RuntimeError('Previous Ego recording still finalizing; try again later')
            state={'token':token,'before':[p.name for p in root.glob('head_*')],
                   'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                   'start_request_monotonic_ns':time.monotonic_ns(),'stopped':False}
            state_path.write_text(json.dumps(state))
            control({'record_toggle':True})
            if not control({'query':True})['recording']['active']:
                raise RuntimeError('Ego recording failed to start')
            state['identity']=recording_identity(root)
            if state['identity'] is None:
                raise RuntimeError('Cannot identify active Ego recording; inspect device state')
            state['recording_name']=record_name(state['identity']['directory'][1:-10])
            state['start_confirm_monotonic_ns']=time.monotonic_ns()
            state_path.write_text(json.dumps(state))
        elif action=='stop':
            if state.get('token')!=token:return {'owned':False}
            if not state.get('stopped'):
                active=control({'query':True})['recording']['active']
                if active:
                    if not state.get('identity') or recording_identity(root)!=state['identity']:
                        return {'owned':False,'ownership_lost':True}
                    control({'record_toggle':True})
                else:state['premature']=True
                if control({'query':True})['recording']['active']:
                    raise RuntimeError('Ego still recording after stop request')
                state.update(stopped=True,stop_confirm_monotonic_ns=time.monotonic_ns())
                state_path.write_text(json.dumps(state))
        elif action=='check':
            if state.get('token')!=token or not state.get('identity'):
                raise RuntimeError('Ego recording ownership lost')
            if not control({'query':True})['recording']['active']:
                raise RuntimeError('Ego recording stopped prematurely')
            if recording_identity(root)!=state['identity']:
                raise RuntimeError('Ego recording replaced externally; refusing to stop replacement')
        else:raise ValueError('Invalid recording action')
        return dict(state,owned=True)


def export_recording(token, timeout=3600):
    state=json.loads(OWNER.read_text())
    if state.get('token')!=token or not state.get('stopped'):
        raise RuntimeError('Cannot export recording without matching stopped session')
    deadline=time.monotonic()+30;folder=None
    while time.monotonic()<deadline:
        names={state['recording_name']} if state.get('recording_name') else set()
        if names:
            name=record_name(next(iter(names)));candidate=ROOT/name
            if (candidate/(name+'.h5')).exists() or (candidate/(name+'.h5.enc')).exists():
                folder=candidate;break
        time.sleep(.25)
    if folder is None:raise RuntimeError('Ego H5 not finalized within 30 seconds')
    # Wait for file sizes to settle before authentication/copy.
    previous=None;stable=0
    while time.monotonic()<deadline:
        sizes={str(p.relative_to(folder)):p.stat().st_size for p in folder.rglob('*') if p.is_file()}
        stable=stable+1 if sizes==previous else 0
        if stable>=3:break
        previous=sizes;time.sleep(.5)
    if stable<3:raise RuntimeError('Ego files still changing; keep originals and retry export')
    path=folder;temporary=None
    if any(folder.rglob('*.enc')):
        sys.path.insert(0,'/usr/share/web_viewer/gateway')
        from file_encryption import DatasetEncryption
        encryption=DatasetEncryption({'board':True});status=encryption.status()
        if not status.get('unlocked'):raise RuntimeError('Ego dataset is locked; encrypted originals retained')
        meta=encryption._file_meta();key=encryption._file_configured_key(meta['key_identifier'])
        if key is None:raise RuntimeError('No configured export key')
        temporary_root=ROOT/'.joint_exports'
        temporary_root.mkdir(exist_ok=True)
        required=sum(p.stat().st_size for p in folder.rglob('*') if p.is_file())
        if shutil.disk_usage(temporary_root).free<required+64*1024*1024:
            raise RuntimeError('Insufficient SD space for decrypted copy; encrypted originals retained')
        temporary=Path(tempfile.mkdtemp(prefix='ego_joint_export_',dir=temporary_root))
        path=temporary/folder.name
        fd=os.memfd_create('ego-export-key');os.write(fd,key);os.lseek(fd,0,0)
        try:
            result=subprocess.run(['/usr/bin/qcs-decrypt','--input',str(folder),
                '--output',str(path),'--key-file',f'/proc/self/fd/{fd}'],
                pass_fds=(fd,),capture_output=True,text=True,timeout=timeout)
            if result.returncode:raise RuntimeError('qcs-decrypt failed: '+result.stderr[-1000:])
        except BaseException:
            shutil.rmtree(temporary)
            raise
        finally:os.close(fd)
    try:
        hashes={}
        for p in path.rglob('*'):
            if p.is_symlink():raise RuntimeError('Unexpected symlink in recording')
            if p.is_file():
                h=hashlib.sha256()
                with p.open('rb') as stream:
                    for chunk in iter(lambda:stream.read(1024*1024),b''):h.update(chunk)
                hashes[str(p.relative_to(path))]=h.hexdigest()
    except BaseException:
        if temporary:shutil.rmtree(temporary)
        raise
    return {'device_source':str(folder),'export_path':str(path),
            'temporary_directory':str(temporary) if temporary else None,'sha256':hashes,
            'h5_name':folder.name+'.h5','boot_id':state['boot_id']}


def run(request):
    action=request['action']
    if action=='status':
        reply=control({'query':True})
        result={'control':reply,'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip()}
        try:
            result['sensor_controls']=subprocess.check_output(['v4l2-ctl','-d','/dev/v4l-subdev3',
                '--get-ctrl','exposure,vertical_blanking,analogue_gain'],text=True,timeout=3)
        except Exception as e:result['sensor_controls_error']=str(e)
    elif action in ('start','stop','check'):result=record_action(action,request['token'])
    elif action=='export':result=export_recording(request['token'],request.get('timeout',3600))
    elif action=='cleanup':
        path=Path(request['path'])
        if path.parent!=ROOT/'.joint_exports' or not path.name.startswith('ego_joint_export_') or path.is_symlink():
            raise ValueError('Invalid temporary export path')
        shutil.rmtree(path);result={'removed':str(path)}
    else:raise ValueError('Unknown device action')
    print(json.dumps(result,ensure_ascii=False),flush=True)
