"""Executable adapters. External solvers retain their own Python environments."""
import csv
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import signal
import h5py
import numpy as np
from joint_capture.import_data import import_npz


def execute(command, cwd, log, env=None):
    merged=os.environ.copy();merged.update(env or {})
    log.write(('\n$ '+repr([str(x) for x in command])+'\n').encode());log.flush()
    process=subprocess.Popen([str(x) for x in command],cwd=cwd,env=merged,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        code=process.wait()
        if code:raise subprocess.CalledProcessError(code,command)
    except BaseException:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGTERM)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        raise


def prepare_vision(session, work, ffmpeg):
    import cv2
    work.mkdir(parents=True,exist_ok=True);config={};cal={}
    with h5py.File(session/'ego.h5') as f:
        c=json.loads(f['calib/calibration_full.json'][()])
        for side,key in [('left','cam0_imx334_m00'),('right','cam1_imx334_m01')]:
            camera='RGB_'+side;video=session/'video'/f'{camera}.mp4'
            cap=cv2.VideoCapture(str(video));n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));width=int(cap.get(3));height=int(cap.get(4));cap.release()
            g=f['sensor/camera/'+camera];idx=g['frame_index'][:];ts=g['timestamp_ns'][:]
            if n<len(ts) or not np.array_equal(idx,np.arange(len(ts))) or np.any(np.diff(ts)<=0):raise ValueError('视频与 H5 索引/时间戳不一致: '+camera)
            if n!=len(ts):
                cropped=work/(camera+'.mp4')
                subprocess.run([ffmpeg,'-v','error','-y','-i',str(video),'-map','0:v:0','-frames:v',str(len(ts)),'-c:v','copy',str(cropped)],check=True)
                video=cropped
                cap=cv2.VideoCapture(str(video));actual=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));cap.release()
                if actual!=len(ts):raise ValueError('裁剪视频帧数错误')
            path=work/('timestamps_'+side+'.csv')
            with path.open('w') as stream:
                writer=csv.writer(stream);writer.writerow(['frame_id','timestamp_ns']);writer.writerows(zip(idx.tolist(),ts.tolist()))
            k=c['cameras'][key]
            if k['resolution']!=[width,height] or k['distortion_model']!='equidistant':raise ValueError('相机标定格式/分辨率不匹配')
            fx,fy,cx,cy=k['intrinsics'];cal[side]={'model':'KB4','K':[[fx,0,cx],[0,fy,cy],[0,0,1]],'D':k['distortion_coefficients'],'width':width,'height':height}
            config[side+'_video']=str(video);config['timestamps_'+side]=str(path)
        transform=np.linalg.inv(np.array(c['canonical_camera_transforms']['transforms']['cam1']['matrix']).reshape(4,4))
    cal['T_left_right']={'R':transform[:3,:3].tolist(),'t_m':transform[:3,3].tolist()}
    (work/'calibration.json').write_text(json.dumps(cal))
    config.update(calibration=str(work/'calibration.json'),timestamp_clock='same-device EGO raw timestamps',sync_source='ego.h5; physical mocap synchronization remains unvalidated',max_skew_ms=5.)
    (work/'input.json').write_text(json.dumps(config))
    return work/'input.json'


def vision(root, session, profile, version, log):
    repo=Path(profile['vision_repo']);work=Path(profile['work_root'])/session.name/version
    input_config=prepare_vision(session,work,profile['ffmpeg'])
    # Each attempt uses a fresh run ID because deployment.infer refuses existing outputs.
    output_id=session.name+'_'+version
    execute(['bash',repo/'run_benchmark.sh','--config',input_config,'--output',output_id],repo,log,{'PATH':str(Path(profile['ffmpeg']).parent)+os.pathsep+os.environ['PATH']})
    output=repo/'ego_hand_benchmark/results'/output_id/'reconstruction/mano_optimized.npz'
    with np.load(output,allow_pickle=False) as data:values={k:data[k] for k in data.files}
    with h5py.File(session/'ego.h5') as f:ts=f['sensor/camera/RGB_left/timestamp_ns'][:]
    if len(values['vertices_m'])!=len(ts):raise ValueError('视觉输出帧数不匹配')
    values.update(timestamp_ns=ts,rgb_index=np.arange(len(ts)),time_s=(ts-ts[0])/1e9,hand_order=np.array(['left','right']))
    converted=work/'vision_import.npz';np.savez_compressed(converted,**values)
    import_npz(converted,session,'vision',version,units='m',coordinate_frame='camera')


def slam(root, session, profile, version, log):
    repo=Path(profile['slam_repo']);work=Path(profile['work_root'])/session.name/version
    inp=work/'input';inp.mkdir(parents=True,exist_ok=True)
    # Preserve device identity required by the existing calibrated adapter profile.
    with h5py.File(session/'ego.h5') as source:
        dataset_name=json.loads(source['metadata/dataset.json'][()])['files']['hdf5']
    if Path(dataset_name).name!=dataset_name:raise ValueError('Invalid dataset H5 basename')
    (inp/dataset_name).symlink_to(session/'ego.h5')
    (inp/'video').symlink_to(session/'video',target_is_directory=True)
    python=repo/'.pixi/envs/default/bin/python'
    env={'PATH':str(python.parent)+os.pathsep+os.environ['PATH']}
    execute([python,repo/'tools/head_dataset/head_to_okvis.py','--dataset',inp,'--output',work/'export','--acc-noise',str(profile.get('acc_noise',.05))],repo,log,env)
    execute([python,repo/'tools/head_dataset/verify_export.py',work/'export'],repo,log,env)
    execute(['bash',repo/'tools/head_dataset/run.sh',work/'export',work/'run'],repo,log,env)
    if profile.get('slam_pose_frame')=='cam0_flu':
        from .slam_import import import_cam0
        import_cam0(work/'run',session,version,repo)
    else:
        execute([sys.executable,root/'tools/okvis_pose_to_h5.py',work/'run',session,'--version',version],root,log)


def mano(root, session, profile, version, log):
    base=Path(profile['mano_root']);work=base/'runs'/(session.name+'_'+version);work.mkdir(parents=True,exist_ok=False)
    layout=json.loads((base/'hand20/left_layout.json').read_text());wanted=list(layout['markersets'][0]['indices'])
    import re
    with h5py.File(session/'mocap.h5') as f:
        if f['raw/markers'].attrs['units']!='m':raise ValueError('MANO 输入必须为米')
        labels=f['raw/markers/labels'].asstr()[:].tolist();lookup={}
        for i,label in enumerate(labels):
            m=re.fullmatch(r'MANO_L_20_(LH_\d{2}):(\w+)',label)
            if m:
                name=m.group(1)+'_'+m.group(2)
                if name in lookup:raise ValueError('重复 marker 标签')
                lookup[name]=i
        if set(lookup)!=set(wanted):raise ValueError('当前适配器需要 MANO_L_20 左手20点标签')
        ids=[lookup[n] for n in wanted]
        points=f['raw/markers/positions'][:][:,ids];valid=f['raw/markers/valid'][:][:,ids];points[~valid]=np.nan
        rate=float(f['raw/frames'].attrs['rate_hz'])
    np.savez(work/'capture.npz',markers=points,labels=np.array(wanted),frame_rate=rate)
    (work/'marker_layout.json').write_text(json.dumps(layout))
    cfg=json.loads((base/'smoke_20_left/config.json').read_text());cfg['mocap'].update(fname=str(work/'capture.npz'),unit='m',end_fidx=-1,only_markers=wanted)
    cfg['surface_model']['fname']=str(base/'hand20/assets/MANO_LEFT.pkl')
    cfg['moshpp']['stagei_frame_picker'].update(num_frames=12,stagei_mocap_fnames=[str(work/'capture.npz')])
    cfg['dirs'].update(work_base_dir=str(work/'work'),stagei_fname=str(work/'work/stagei.pkl'),stageii_fname=str(work/'work/stageii.pkl'),log_fname=str(work/'work/moshpp.log'))
    cfg['dirs']['marker_layout']['fname']=str(work/'marker_layout.json');cfg['opt_settings']['maxiter']=100
    (work/'config.json').write_text(json.dumps(cfg))
    env={'PYTHONPATH':str(base/'source/src'),'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'}
    python=base/'py37/.pixi/envs/default/bin/python'
    for script in ['run','validate','preview']:
        target=work/(script+'.py');shutil.copyfile(Path(__file__).with_name('workers')/('mano_'+script+'.py'),target)
        execute([python,target],base.parent,log,env)
    import_npz(work/(work.name+'_mano.npz'),session,'mano',version,units='m',coordinate_frame='mocap_world',mesh=work/'hand_mesh_sequence.npz',quality_json=work/'quality_report.json')

ADAPTERS={'vision':vision,'slam':slam,'mano':mano}
