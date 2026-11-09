"""Persistent Ego/NatNet clock connections, individually owned action recordings."""
from concurrent.futures import ThreadPoolExecutor
import fcntl
import json
import os
from pathlib import Path
import threading
import time
import uuid
from .clock import ClockProbe,local_address
from .ego import EgoController
from .motive import MotiveTakeController,DEFAULT_TAKE_ROOT
from .natnet import NatNetStream
from .storage import RigidH5Writer,save_json


class JointCollection:
    def __init__(self,folder,config,*,ego_factory=EgoController,stream_factory=NatNetStream,
                 probe_factory=ClockProbe,take_factory=MotiveTakeController,alignment_factory=None):
        from .action_alignment import align_action
        self.alignment_factory=alignment_factory or align_action
        self.folder=Path(folder);self.config=config;self.ego_factory=ego_factory;self.stream_factory=stream_factory
        self.probe_factory=probe_factory;self.take_factory=take_factory;self.probes=[];self.stream=None
        self.active=None;self.error=None;self.stop=threading.Event();self.thread=None;self.lock=threading.RLock()
        self.health_thread=None;self.file_lock=None;self.opened=False;self.last_state={};self.boot_id=None

    def open(self):
        if self.opened:return
        self.folder.mkdir(parents=True,exist_ok=True);(self.folder/'clocks').mkdir(exist_ok=True)
        lock_path=Path(self.config.get('lock_path',str(Path(__file__).resolve().parents[2]/'data/sessions/.capture.lock')))
        lock_path.parent.mkdir(parents=True,exist_ok=True)
        self.file_lock=lock_path.open('a')
        try:
            fcntl.flock(self.file_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            ego=self.ego_factory(token='collection-'+uuid.uuid4().hex,serial=self.config.get('ego_serial'))
            initial=ego.status()
            if initial['control']['recording']['active']:raise RuntimeError('Ego已有录制，拒绝接管')
            self.boot_id=initial['boot_id'];self.pc_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
            self.client_ip=self.config.get('natnet_client_ip') or local_address(self.config['motive_server_ip'])
            self.stream=self.stream_factory(self.config['motive_server_ip'],self.client_ip,timeout=15)
            self.stream.start()
            if self.stream.motive_recording is not False:raise RuntimeError('Motive已有录制或状态未知')
            for kind,ip in [('ego',self.config['ego_ip']),('windows',self.config['motive_server_ip'])]:
                p=self.probe_factory(kind,self.folder/'clocks'/f'{kind}.jsonl',ip,adb=ego.adb,ssh_host=self.config['windows_host'])
                self.probes.append(p);p.start()
            for p in self.probes:
                if not p.ready.wait(25):raise RuntimeError(p.kind+'对时未就绪')
                p.check()
            if self.probes[1].frequency!=self.stream.frequency:raise RuntimeError('Motive时钟频率不一致')
            save_json(self.folder/'collection.json',dict(status='open',ego_boot_id=self.boot_id,pc_boot_id=self.pc_boot_id,config=self.config))
            self.opened=True;self.thread=threading.Thread(target=self._monitor,daemon=True);self.thread.start()
            self.health_thread=threading.Thread(target=self._check_ego,daemon=True);self.health_thread.start()
        except Exception:
            self.close();raise

    def _monitor(self):
        while not self.stop.wait(.05):
            try:
                self.stream.check()
                for p in self.probes:p.check()
                with self.lock:
                    active=self.active
                    frames=self.stream.drain()
                    if active and active.get('writer'):active['writer'].append(frames)
                    if active and active.get('started'):
                        active['take'].check()

            except Exception as exc:
                self.error=str(exc)

    def _check_ego(self):
        # Slow ADB checks must never hold the frame writer/stop lock.
        while not self.stop.wait(.2):
            with self.lock:active=self.active if self.active and self.active.get('started') else None
            if active:
                try:active['ego'].check()
                except Exception as exc:
                    with self.lock:
                        if self.active is active and active.get('started'):self.error=str(exc)
                self.stop.wait(.8)

    def check(self):
        if self.error:raise RuntimeError(self.error)
        if not self.opened:raise RuntimeError('联合采集整轮连接未建立')
        self.stream.check()
        for p in self.probes:p.check()

    def status(self):
        return dict(connected=self.opened and not self.error,error=self.error,
                    recording=bool(self.active and self.active.get('started')),clock_probes={p.kind:getattr(p,'count',0) for p in self.probes},
                    last_action=self.last_state)

    def start_action(self,path,session_id,on_ready=None):
        self.check()
        if self.active:raise RuntimeError('上一动作尚未保存')
        folder=Path(path)/'joint';folder.mkdir(exist_ok=False)
        ego=self.ego_factory(token=session_id,serial=self.config.get('ego_serial'))
        ego.export_timeout=self.config.get('export_timeout',180)
        initial=ego.status()
        if initial['boot_id']!=self.boot_id:raise RuntimeError('Ego已重启，请结束本轮重新连接')
        if initial['control']['recording']['active']:raise RuntimeError('Ego已有录制')
        take=self.take_factory(server_ip=self.config['motive_server_ip'],client_ip=self.client_ip,
            windows_host=self.config['windows_host'],take_root=self.config.get('windows_take_root') or DEFAULT_TAKE_ROOT,
            session_id=session_id,timeout=15,export_timeout=self.config.get('export_timeout',180))
        meta=dict(session_id=session_id,take_id=session_id,status='starting',collection_directory=os.path.relpath(self.folder,folder),
                  ego_boot_id=self.boot_id,pc_boot_id=self.pc_boot_id,errors=[],physical_sync_valid=False,
                  requested_start_monotonic_ns=time.monotonic_ns())
        active=dict(ego=ego,take=take,folder=folder,meta=meta,started=False,ego_attempted=False)
        self.active=active;save_json(folder/'manifest.json',meta)
        try:
            meta['motive_prepared']=take.prepare(self.stream,start_stream=False)
            with self.lock:
                self.stream.drain();active['writer']=RigidH5Writer(folder/('mocap_'+session_id+'.h5'),self.stream.frequency,session_id)
                active['writer'].file.attrs['pc_boot_id']=self.pc_boot_id
                active['writer'].file.attrs['take_id']=session_id
                active['writer'].file.create_dataset('server_info_packet',data=list(self.stream.server_packet),dtype='u1')
                self.stream.recording.set()
            if on_ready:on_ready()
            active['ego_attempted']=True
            with ThreadPoolExecutor(max_workers=2) as pool:
                motive_future=pool.submit(take.start);ego_future=pool.submit(ego.start)
                meta['motive_start']=motive_future.result();meta['ego_start']=ego_future.result()
            active['started']=True;meta['status']='recording';meta['confirmed_start_monotonic_ns']=time.monotonic_ns()
            save_json(folder/'manifest.json',meta)
            return meta
        except Exception as exc:
            meta['errors'].append(str(exc));self.finish_action();raise

    def finish_action(self):
        if not self.active:return self.last_state
        active=self.active;meta=active['meta'];folder=active['folder'];ego=active['ego'];take=active['take']
        # Prevent monitor treating intentional stops as disconnections.
        with self.lock:active['started']=False
        meta['status']='saving';meta['requested_stop_monotonic_ns']=time.monotonic_ns()
        if self.error:meta['errors'].append(self.error)
        owned=False
        if active['ego_attempted']:
            try:
                meta['ego_stop']=ego.stop();owned=meta['ego_stop'].get('owned',False)
                if not owned or meta['ego_stop'].get('premature'):raise RuntimeError('Ego停止归属或完整性异常')
            except Exception as exc:meta['errors'].append('ego_stop: '+str(exc))
        if take.attempted:
            try:meta['motive_stop']=take.stop()
            except Exception as exc:meta['errors'].append('motive_stop: '+str(exc))
        with self.lock:
            self.stream.recording.clear()
            if active.get('writer'):
                try:
                    active['writer'].append(self.stream.drain());meta['mocap_frames']=active['writer'].count
                    active['writer'].close(complete=not meta['errors'],names=self.stream.names)
                except Exception as exc:meta['errors'].append('mocap_save: '+str(exc))
                finally:active['writer']=None
        if take.attempted:
            try:
                meta['take']=take.copy_to(folder/('motive_'+meta['take_id']+'.tak'))
                meta['take']['local_path']=os.path.relpath(meta['take']['local_path'],folder)
            except Exception as exc:meta['errors'].append('take_verify: '+str(exc))
        if owned:
            try:
                meta['ego_export']=ego.export(folder/('ego_'+meta['take_id']))
                import h5py
                ego_h5=Path(meta['ego_export']['h5'])
                with h5py.File(ego_h5,'r') as f:
                    counts={}
                    for camera in ('RGB_left','RGB_right'):
                        group=f['sensor/camera/'+camera]
                        counts[camera]=len(group['timestamp_ns'])
                        video=ego_h5.parent/str(group.attrs.get('video_path','video/'+camera+'.mp4'))
                        if not counts[camera] or not video.is_file() or video.stat().st_size==0:
                            raise RuntimeError('Ego原始视频/时间戳缺失: '+camera)
                    meta['ego_camera_frames']=counts
                for key in ('local_directory','h5'):
                    if key in meta['ego_export']:meta['ego_export'][key]=os.path.relpath(meta['ego_export'][key],folder)
            except Exception as exc:meta['errors'].append('ego_export: '+str(exc))
        try:self.check()
        except Exception as exc:
            if str(exc) not in meta['errors']:meta['errors'].append(str(exc))
        if owned and 'ego_export' in meta and meta.get('take',{}).get('copied_to_ubuntu'):
            try:
                meta['alignment']=self.alignment_factory(folder,folder/meta['ego_export']['h5'],
                    folder/('mocap_'+meta['take_id']+'.h5'),self.folder/'clocks',meta['take_id'],self.boot_id)
            except Exception as exc:meta['errors'].append('clock_alignment: '+str(exc))
        meta['assets']=dict(mocap_h5='mocap_'+meta['take_id']+'.h5',
            motive_take=meta.get('take',{}).get('local_path'),ego_h5=meta.get('ego_export',{}).get('h5'),
            alignment_h5='alignment.h5' if (folder/'alignment.h5').exists() else None)
        meta['status']='failed' if meta['errors'] else 'complete';meta['finished_monotonic_ns']=time.monotonic_ns()
        save_json(folder/'manifest.json',meta);save_json(folder/'ego_control.json',ego.events);save_json(folder/'motive_control.json',take.events)
        self.last_state=meta;self.active=None
        if meta['errors']:self.error='上一动作未完整保存；结束本轮后排查：'+'; '.join(meta['errors'])
        return meta

    def close(self):
        if self.active:
            try:self.finish_action()
            except Exception as exc:self.error=str(exc)
        self.stop.set()
        if self.thread:self.thread.join(timeout=5)
        if self.health_thread:self.health_thread.join(timeout=21)
        for p in self.probes:
            try:p.close()
            except Exception as exc:self.error=str(exc)
        if self.stream:
            try:self.stream.close()
            except Exception as exc:self.error=str(exc)
        if self.file_lock:self.file_lock.close();self.file_lock=None
        if self.opened:
            save_json(self.folder/'collection.json',dict(status='failed' if self.error else 'closed',error=self.error,
                ego_boot_id=self.boot_id,pc_boot_id=self.pc_boot_id,clocks={p.kind:p.summary() for p in self.probes}))
        self.opened=False
