"""One EGO recording plus either Ubuntu NatNet H5 or Windows Motive Take."""
import fcntl
import hashlib
from contextlib import ExitStack
import json
import os
from pathlib import Path
import re
import time
import uuid
from datetime import datetime
from zoneinfo import ZoneInfo
import h5py
import numpy as np
from .storage import RigidH5Writer, save_json
from .natnet import NatNetStream
from .ego import EgoController
from .clock import ClockProbe, local_address
from .paths import ProjectPaths
from .motive import MotiveTakeController, validate_take_root, DEFAULT_TAKE_ROOT


def run_session(config, stop_event, *, ego_factory=EgoController,
                stream_factory=NatNetStream, probe_factory=ClockProbe, take_factory=MotiveTakeController):
    mode=getattr(config,'mode','take')
    if mode not in ('h5','take'):raise ValueError('未知采集模式')
    calibration=getattr(config,'calibration',None)
    if calibration and (calibration not in ('rbc','tbc') or mode!='h5' or config.duration is not None):
        raise ValueError('标定引导需要h5模式且不能指定duration')
    if mode=='take':validate_take_root(getattr(config,'windows_take_root',None))
    elif getattr(config,'windows_take_root',None):raise ValueError('--windows-take-root 仅用于take模式')
    root=Path(config.output_root).resolve()
    paths=ProjectPaths(root)
    prefix=f'calibration_{calibration}_' if calibration else ('calibration_' if mode=='h5' else '')
    session_id=config.session_id or (prefix+datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
    if not re.fullmatch(r'[A-Za-z0-9_-]+',session_id):raise ValueError('session-id只允许字母、数字、下划线和横线')
    sessions=root/'data'/'sessions';sessions.mkdir(parents=True,exist_ok=True)
    legacy_sessions=root/'capture_sessions';legacy_sessions.mkdir(parents=True,exist_ok=True)
    with ExitStack() as stack:
        for lock_path in (sessions/'.capture.lock',legacy_sessions/'.capture.lock'):
            lock=stack.enter_context(lock_path.open('a'))
            try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:raise RuntimeError('已有联合采集程序运行，请勿重复启动') from None
        session=paths.session(session_id);mocap=paths.mocap_h5(session_id);ego_path=session/'.ego-export'
        if session.exists() or (legacy_sessions/session_id).exists():raise FileExistsError('会话已存在，不覆盖: '+session_id)
        (session/'clocks').mkdir(parents=True);paths.capture_records(session_id).mkdir(parents=True)
        print(('标定输出目录：' if mode=='h5' else '采集输出目录：')+str(session.resolve()),flush=True)
        return _capture(config,stop_event,session_id,session,mocap,ego_path,
                        ego_factory,stream_factory,probe_factory,take_factory)


def _capture(config,stop_event,session_id,session,mocap_path,ego_path,ego_factory,stream_factory,probe_factory,take_factory):
    mode=getattr(config,'mode','take')
    use_take=mode=='take'
    manifest={'capture_purpose':'ego_extrinsic_calibration' if mode=='h5' else 'general_capture','capture_mode':mode,'session_id':session_id,'status':'preparing','pc_boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        'schema_version':1,'layout_version':2,
        'created_at':datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),'requested_duration_s':config.duration,
        'capture_config':{
            'mode':mode,'windows_take_root':(getattr(config,'windows_take_root',None) or DEFAULT_TAKE_ROOT) if use_take else None,
            'ego_serial':config.ego_serial,'ego_ip':config.ego_ip,
            'motive_server_ip':config.motive_server_ip,'natnet_client_ip':config.natnet_client_ip,
            'windows_host':config.windows_host,'clock_probes_enabled':not config.no_clock_probes,
            'startup_timeout_s':config.startup_timeout,'export_timeout_s':getattr(config,'export_timeout',3600),
            'warmup_s':config.warmup,'mocap_tail_s':config.tail,
        },
        'physical_exposure_calibrated':False,'interpolated':False,'errors':[],
        'mocap':({'storage':'windows','format':'motive_take'} if use_take else {'file':'mocap.h5','storage':'ubuntu','format':'natnet_h5'}),'ego':{},'clock_probes_enabled':not config.no_clock_probes}
    manifest_path=session/'session.json'
    def record_event(name,**extra):
        with (session/'capture'/'events.jsonl').open('a') as f:
            f.write(json.dumps({'event':name,'pc_monotonic_ns':time.monotonic_ns(),**extra},ensure_ascii=False)+'\n')
    def fail(stage,error):
        manifest['errors'].append({'stage':stage,'error':str(error)})
        print(f'[{stage}] {error}',flush=True)
    save_json(manifest_path,manifest)
    stream=None;ego=None;writer=None;take=None;probes=[];attempted=False;stopped=False
    guide=None
    try:
        if getattr(config,'calibration',None):
            from .guidance import CalibrationGuide
            guide=CalibrationGuide(config.calibration,session/'capture'/'calibration_guide.json',record_event)
            manifest['calibration']={'target':config.calibration,'status':'pending','guide_file':'capture/calibration_guide.json','planned_duration_s':guide.state['planned_duration_s']}
            save_json(manifest_path,manifest)
            print(f"{config.calibration.upper()} 动作引导：计划 {guide.state['planned_duration_s']} 秒，设备确认启动后开始倒计时。",flush=True)
        print('检查Ego、NatNet及对时通道……',flush=True)
        ego=ego_factory(token=session_id,serial=config.ego_serial)
        if hasattr(ego,'export_timeout'):ego.export_timeout=getattr(config,'export_timeout',3600)
        initial=ego.status();manifest['ego']['initial_status']=initial
        if initial['control']['recording']['active']:
            raise RuntimeError('Ego已经在录制，未作任何启动/停止操作')
        client_ip=config.natnet_client_ip or local_address(config.motive_server_ip)
        stream=stream_factory(config.motive_server_ip,client_ip,timeout=config.startup_timeout)
        if use_take:
            take=take_factory(server_ip=config.motive_server_ip,client_ip=client_ip,
                windows_host=config.windows_host,take_root=getattr(config,'windows_take_root',None) or DEFAULT_TAKE_ROOT,session_id=session_id,
                timeout=config.startup_timeout,export_timeout=getattr(config,'export_timeout',3600))
            manifest['mocap']['windows']=take.prepare(stream)
        else:stream.start()
        manifest['mocap']['tick_frequency_hz']=stream.frequency
        if stop_event.is_set():raise RuntimeError('启动前取消')
        if not config.no_clock_probes:
            for kind,ip in [('ego',config.ego_ip),('windows',config.motive_server_ip)]:
                probe=probe_factory(kind,session/'clocks'/(kind+'.jsonl'),ip,adb=ego.adb,ssh_host=config.windows_host)
                probes.append(probe);probe.start()
            for probe in probes:
                if not probe.ready.wait(config.startup_timeout+10):raise RuntimeError(probe.kind+'对时通道未就绪')
                probe.check()
            if probes[1].frequency!=stream.frequency:raise RuntimeError('NatNet与Windows探针频率不同')
            if stop_event.wait(config.warmup):raise RuntimeError('启动前取消')
        stream.check()
        if use_take:
            record_event('motive_start_request')
            manifest['mocap']['start']=take.start();record_event('motive_start_confirmed')
        else:
            writer=RigidH5Writer(mocap_path,stream.frequency,session_id)
            writer.file.create_dataset('server_info_packet',data=np.frombuffer(stream.server_packet,dtype=np.uint8))
            writer.file.attrs['pc_boot_id']=manifest['pc_boot_id']
            writer.file.attrs['motive_server_ip']=config.motive_server_ip
            stream.recording.set();record_event('ubuntu_recording_enabled')
        if stop_event.is_set():raise RuntimeError('EGO启动前取消')
        attempted=True;record_event('ego_start_request')
        manifest['ego']['start']=ego.start();record_event('ego_start_confirmed')
        start=time.monotonic();last_ego_check=start;manifest['status']='recording';save_json(manifest_path,manifest)
        print(f'正在录制：{session_id}。按 Ctrl+C 停止并保存。',flush=True)
        if guide:guide.start(time.monotonic_ns())
        while True:
            if writer:writer.append(stream.drain())
            stream.check()
            if take:take.check()
            for probe in probes:probe.check()
            if time.monotonic()-last_ego_check>=1:
                ego.check();last_ego_check=time.monotonic()
            if stop_event.is_set() or (config.duration is not None and time.monotonic()-start>=config.duration):break
            if guide and guide.tick(time.monotonic_ns()):break
            stop_event.wait(.05)
        record_event('stop_requested')
    except Exception as e:fail('capture',e)
    finally:
        if guide:
            try:
                status=guide.finish(time.monotonic_ns())
                manifest['calibration']['status']=status
                if status!='complete':fail('calibration_incomplete','标定动作未全部完成，已有数据仍会保存')
            except Exception as e:fail('calibration_save',e)
        if attempted and ego:
            manifest['status']='saving'
            try:save_json(manifest_path,manifest)
            except Exception as e:fail('manifest_save',e)
            print('停止录制并保存……',flush=True)
            try:
                manifest['ego']['stop']=ego.stop();stopped=manifest['ego']['stop'].get('owned',False)
                if not stopped:fail('ego_stop','录制归属变化，未停止其他录制')
                if manifest['ego']['stop'].get('premature'):fail('ego_stop','Ego提前停止，数据作为部分录制保留')
                record_event('ego_stop_confirmed')
            except Exception as e:fail('ego_stop',e)
        if (writer or (take and take.attempted)) and stream:
            try:
                until=time.monotonic()+config.tail
                while time.monotonic()<until:
                    if writer:writer.append(stream.drain())
                    if take:take.check()
                    time.sleep(.05)
                stream.check()
            except Exception as e:fail('mocap_tail',e)
        if take and take.attempted:
            try:
                manifest['mocap']['stop']=take.stop()
                record_event('motive_stop_confirmed')
            except Exception as e:fail('motive_stop',e)
        if stream:
            try:stream.close()
            except Exception as e:fail('natnet_close',e)
        if writer:
            try:
                writer.append(stream.drain())
                manifest['mocap'].update(frames=writer.count,valid_body_observations=writer.valid_bodies,
                    missing_frame_numbers=writer.missing_frames)
                if not writer.count or not writer.valid_bodies:fail('mocap_validate','没有有效刚体数据，原始数据已保留')
                writer.close(complete=not manifest['errors'],names=stream.names)
            except Exception as e:
                fail('h5_close',e)
                try:writer.close(complete=False)
                except Exception:pass
        if probes:time.sleep(.4)
        for probe in probes:
            try:probe.close()
            except Exception as e:fail('clock_close',e)
            if probe.error:fail('clock_close',probe.error)
        manifest['sync']={p.kind:p.summary() for p in probes}
        if take and take.attempted:
            try:manifest['remote_assets']={'motive_take':take.verify()}
            except Exception as e:fail('motive_file_verify',e)
        if take:
            try:save_json(session/'capture'/'motive_control.json',take.events)
            except Exception as e:fail('motive_log',e)
        if stopped:
            try:
                print('导出Ego文件并校验哈希……',flush=True)
                exported=ego.export(ego_path)
                exported=publish_ego_export(session,ego_path,exported)
                manifest['ego'].update(exported)
                with h5py.File(exported['h5'],'r') as f:
                    counts={name:len(g['timestamp_ns']) for name,g in f['sensor/camera'].items() if 'timestamp_ns' in g}
                    manifest['ego']['camera_frames']=counts
                    manifest['ego']['rgb_frames']=counts.get('RGB_left',0)
                    if not counts.get('RGB_left'):raise RuntimeError('Ego H5没有左RGB帧')
            except Exception as e:fail('ego_export',e)
        if ego:save_json(session/'capture'/'ego_control.json',ego.events)
        manifest['status']='failed' if manifest['errors'] or not stopped else 'complete'
        manifest['finished_at']=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat()
        def sha256(path):
            digest=hashlib.sha256()
            with Path(path).open('rb') as source:
                for chunk in iter(lambda:source.read(1024*1024),b''):digest.update(chunk)
            return digest.hexdigest()
        assets={}
        candidates={
            'calibration_guide':session/'capture'/'calibration_guide.json',
            'mocap_h5':mocap_path,
            'ego_h5':Path(manifest['ego']['h5']) if manifest.get('ego',{}).get('h5') else None,
            'ego_clock_probes':session/'clocks'/'ego.jsonl',
            'windows_clock_probes':session/'clocks'/'windows.jsonl',
        }
        for asset_id,path in candidates.items():
            if path and Path(path).is_file():
                assets[asset_id]={'path':Path(path).resolve().relative_to(session.resolve()).as_posix(),
                                  'sha256':sha256(path)}
        indexed_paths={item['path'] for item in assets.values()}
        for i,relative in enumerate(sorted(manifest.get('ego',{}).get('sha256',{})),start=1):
            path=(session/relative).resolve()
            if path.is_file() and path.is_relative_to(session.resolve()):
                rel=path.relative_to(session.resolve()).as_posix()
                if rel not in indexed_paths:
                    assets[f'ego_media_{i:04d}']={'path':rel,'sha256':sha256(path)}
                    indexed_paths.add(rel)
        manifest['assets']=assets
        manifest['legacy_paths']={
            'mocap_file':None if use_take else str(mocap_path.resolve()),
            'ego_local_directory':manifest.get('ego',{}).get('local_directory'),
            'ego_h5':manifest.get('ego',{}).get('h5'),
        }
        if not use_take:manifest.setdefault('mocap',{})['file']='mocap.h5'
        if 'ego_h5' in assets:
            manifest['ego']['local_directory']='.'
            manifest['ego']['h5']=assets['ego_h5']['path']
        save_json(manifest_path,manifest)
        print(f"状态：{manifest['status']}；会话记录：{manifest_path}",flush=True)
    return manifest


def publish_ego_export(session, staging, exported):
    """Move a verified ADB export into the flat session, retaining every source file."""
    session=Path(session);staging=Path(staging)
    h5=Path(exported['h5']).resolve()
    if not h5.is_relative_to(staging.resolve()):raise ValueError('EGO导出H5不在临时目录内')
    files=sorted(p for p in staging.rglob('*') if p.is_file())
    mapping={}
    for source in files:
        rel=source.relative_to(staging)
        if source.resolve()==h5:destination=session/'ego.h5'
        elif rel.parts[0]=='video':destination=session/rel
        elif source.suffix.lower()=='.mp4':destination=session/'video'/rel
        else:destination=session/'capture'/'ego_export'/rel
        if destination.exists():raise FileExistsError(destination)
        mapping[source]=destination
    for source,destination in mapping.items():
        destination.parent.mkdir(parents=True,exist_ok=True)
        source.rename(destination)
    for directory in sorted((p for p in staging.rglob('*') if p.is_dir()),key=lambda p:len(p.parts),reverse=True):directory.rmdir()
    staging.rmdir()
    from .h5store import file_sha256
    result=dict(exported)
    result['source_export_manifest']=exported
    result['h5']=str((session/'ego.h5').resolve());result['local_directory']=str(session.resolve())
    result['sha256']={dest.relative_to(session).as_posix():file_sha256(dest) for dest in mapping.values()}
    return result
