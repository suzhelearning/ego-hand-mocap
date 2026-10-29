"""Package existing nearest-frame results into one H5 per session."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import h5py
import numpy as np
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from joint_capture.manifests import load_session_manifest, resolve_session_asset
from joint_capture.paths import ProjectPaths
from joint_capture.runs import finish_stage


def export(root,session_id,run_id=None):
    paths=ProjectPaths(root)
    session_dir=paths.session(session_id)
    canonical_session=(session_dir/'session.json').is_file()
    if not (session_dir/'session.json').is_file():
        session_dir=root/'capture_sessions'/session_id
    if canonical_session and not run_id:
        raise ValueError('新布局请通过 --run-id 明确选择配对结果所属运行，避免隐式选择历史结果')
    if run_id and not (paths.run(session_id,run_id)/'run.json').is_file():
        raise FileNotFoundError(f'处理运行清单不存在: {paths.run(session_id,run_id)/"run.json"}')
    source=(paths.run(session_id,run_id)/'sync') if run_id else root/'results/alignment_20261029'/session_id
    summary=json.loads((source/'summary.json').read_text())
    session=load_session_manifest(session_dir/'session.json')
    mocap_path=resolve_session_asset(root,session,'mocap_h5')
    ego_path=resolve_session_asset(root,session,'ego_h5')
    destination=paths.run(session_id,run_id)/'aligned' if run_id else root/'data'/'aligned_h5'/session_id
    destination.mkdir(parents=True,exist_ok=True)
    path=destination/(session_id+'_aligned.h5' if not run_id else 'aligned.h5')
    ego_dir=session_dir/'raw'/'ego'
    if not ego_dir.is_dir():
        ego_dir=Path(session.get('ego',{}).get('local_directory',''))
    try:video_reference=ego_dir.resolve().relative_to(session_dir.resolve()).as_posix()
    except (ValueError, OSError):video_reference=str(ego_dir)
    def source_reference(path):
        try:return path.resolve().relative_to(session_dir.resolve()).as_posix()
        except ValueError:return str(path.resolve())
    with h5py.File(mocap_path,'r') as mocap,h5py.File(path,'x') as out:
        out.attrs.update(schema='ego-mocap-nearest-aligned-v1',session_id=session_id,complete=False,
            interpolated=False,physical_exposure_alignment_valid=False,
            pc_time_domain='Ubuntu CLOCK_MONOTONIC',position_unit='m',quaternion_order='xyzw',
            ego_source_h5=source_reference(ego_path),mocap_source_h5=source_reference(mocap_path),
            source_capture_status=summary['source_status'],
            image_storage='Video remains in original ego directory; no image pixels embedded',
            delta_definition='mocap_pc_ns - rgb_pc_ns')
        for name in ['summary','clock_models']:
            out.create_dataset('metadata/'+name+'_json',data=(source/(name+'.json')).read_text(),dtype=h5py.string_dtype('utf-8'))
        for camera in summary['cameras']:
            with (source/(camera+'_nearest.csv')).open() as f:rows=list(csv.DictReader(f))
            group=out.create_group('cameras/'+camera)
            for key in rows[0]:
                values=[r[key] for r in rows]
                is_bool=values[0] in ('True','False')
                group.create_dataset(key,data=np.array([v=='True' for v in values],dtype=bool) if is_bool else np.array(values,dtype=np.int64),compression='gzip')
            group.attrs['video_path']=video_reference+'/video/'+camera+'.mp4'
            group.attrs['rgb_index_meaning']='Zero-based row in original camera H5 timestamp array; video frame correspondence retained from source'
            indices=group['mocap_index'][:]
            valid=group['within_mocap_coverage'][:]&~group['clock_extrapolated'][:]
            group.create_dataset('clock_pair_usable',data=valid)
            for body_id,body in mocap['rigid_bodies'].items():
                target=group.create_group('rigid_bodies/'+body_id)
                for key,value in body.attrs.items():target.attrs[key]=value
                for key,ds in body.items():target.create_dataset(key,data=ds[:][indices],compression='gzip')
                target.create_dataset('usable',data=valid&target['present'][:]&target['tracking_valid'][:])
            group.create_dataset('mocap_received_pc_ns',data=mocap['frames/publisher_received_time_ns'][:][indices])
        out.attrs['complete']=True
    if run_id:
        digest=hashlib.sha256(path.read_bytes()).hexdigest()
        finish_stage(paths.run(session_id,run_id),'aligned_export','complete',
                     outputs=[{'path':'aligned/'+path.name,'sha256':digest,'size':path.stat().st_size}])
    return path


if __name__=='__main__':
    p=argparse.ArgumentParser(description='将既有最近邻配对打包为H5，不插值，不覆盖已有文件')
    p.add_argument('sessions',nargs='+');p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    p.add_argument('--run-id',help='将结果写入指定会话的该处理运行；仅可写入既有运行')
    a=p.parse_args()
    for session_id in a.sessions:print(export(a.root.resolve(),session_id,a.run_id))
