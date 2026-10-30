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


def export_legacy(root,session_id,run_id=None):
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


def export(root,session_id,run_id=None):
    """Publish paired observations inside session/aligned.h5; run_id is an internal version."""
    import io
    from joint_capture.h5store import write_version, resolve_version, file_sha256
    root=Path(root).resolve();session_dir=ProjectPaths(root).session(session_id)
    if not (session_dir/'session.json').is_file():
        session_dir=root/'capture_sessions'/session_id
        if not (session_dir/'aligned.h5').is_file():
            return export_legacy(root,session_id,run_id)
    manifest=load_session_manifest(session_dir/'session.json')
    destination=session_dir/'aligned.h5'
    arrays={}
    if destination.is_file():
        with h5py.File(destination,'r') as source:
            sync=resolve_version(source,'sync',run_id)
            version=sync.name.rsplit('/',1)[1]
            if 'summary_json' in sync:
                summary_text=sync['summary_json'].asstr()[()];models_text=sync['clock_models_json'].asstr()[()]
                arrays={name:{key:ds[:] for key,ds in group.items()} for name,group in sync['cameras'].items()}
            else:
                # Migrated legacy outputs retain exact CSV/JSON bytes under artifacts.
                def artifact(name):return bytes(sync['artifacts/sync/'+name][:]).decode('utf8')
                summary_text=artifact('summary.json');models_text=artifact('clock_models.json')
                for camera in json.loads(summary_text)['cameras']:
                    rows=list(csv.DictReader(io.StringIO(artifact(camera+'_nearest.csv'))))
                    arrays[camera]={}
                    for key in rows[0]:
                        v=[r[key] for r in rows]
                        arrays[camera][key]=np.array([x=='True' for x in v],dtype=bool) if v[0] in ('True','False') else np.array(v,dtype=np.int64)
    else:
        if not run_id:raise ValueError('尚未计算时钟配对，请先运行joint_capture.alignment')
        source=ProjectPaths(root).run(session_id,run_id)/'sync'
        summary_text=(source/'summary.json').read_text();models_text=(source/'clock_models.json').read_text();version=run_id
        for camera in json.loads(summary_text)['cameras']:
            with (source/(camera+'_nearest.csv')).open() as f:rows=list(csv.DictReader(f))
            arrays[camera]={key:np.array([r[key]=='True' for r in rows],dtype=bool) if rows[0][key] in ('True','False') else np.array([r[key] for r in rows],dtype=np.int64) for key in rows[0]}
    summary=json.loads(summary_text)
    mocap_path=resolve_session_asset(root,manifest,'mocap_h5');ego_path=resolve_session_asset(root,manifest,'ego_h5')
    def source_reference(path):
        try:return path.relative_to(session_dir).as_posix()
        except ValueError:return str(path)
    references={'ego':{'path':source_reference(ego_path),'sha256':file_sha256(ego_path)},
                'mocap':{'path':source_reference(mocap_path),'sha256':file_sha256(mocap_path)}}
    video_directory=session_dir/'video'
    if not video_directory.is_dir():
        ego_directory=session_dir/'raw/ego'
        if not ego_directory.is_dir():
            ego_directory=Path(manifest.get('ego',{}).get('local_directory',str(ego_directory)))
            if not ego_directory.is_absolute():ego_directory=session_dir/ego_directory
        video_directory=ego_directory/'video'
    def write(group):
        group.attrs.update(schema='ego-mocap-nearest-aligned-v1',session_id=session_id,
            physical_exposure_alignment_valid=False,interpolated=False,
            ego_source_h5=references['ego']['path'],mocap_source_h5=references['mocap']['path'],
            position_unit='m',delta_definition='mocap_pc_ns - rgb_pc_ns',sync_version=version)
        group.create_dataset('sources/json',data=json.dumps(references),dtype=h5py.string_dtype())
        group.create_dataset('metadata/summary_json',data=summary_text,dtype=h5py.string_dtype())
        group.create_dataset('metadata/clock_models_json',data=models_text,dtype=h5py.string_dtype())
        with h5py.File(mocap_path,'r') as mocap:
            if 'rigid_bodies' not in mocap:raise ValueError('C3D数据需要独立时间锚点和手部配对适配，不能当NatNet刚体H5导出')
            for camera,columns in arrays.items():
                target=group.create_group('cameras/'+camera)
                for key,data in columns.items():target.create_dataset(key,data=data,compression='gzip')
                indices=columns['mocap_index']
                valid=columns['within_mocap_coverage'] & ~columns['clock_extrapolated']
                target.create_dataset('clock_pair_usable',data=valid)
                target.attrs['video_path']=source_reference(video_directory/(camera+'.mp4'))
                for body_id,body in mocap['rigid_bodies'].items():
                    body_out=target.create_group('rigid_bodies/'+body_id)
                    for key,value in body.attrs.items():body_out.attrs[key]=value
                    for key,ds in body.items():body_out.create_dataset(key,data=ds[:][indices],compression='gzip')
                    body_out.create_dataset('usable',data=valid & body_out['present'][:] & body_out['tracking_valid'][:])
                target.create_dataset('mocap_received_pc_ns',data=mocap['frames/publisher_received_time_ns'][:][indices])
        for key in ('cameras','sources'):
            if key in group.file:del group.file[key]
            group.file[key]=group[key]
        group.file.attrs['active_alignment_version']=version
        group.file.attrs['ego_source_h5']=references['ego']['path']
        group.file.attrs['mocap_source_h5']=references['mocap']['path']
    write_version(destination,'alignment',version,write,metadata={'sources':references,'sync_version':version})
    return destination


if __name__=='__main__':
    p=argparse.ArgumentParser(description='将最近邻配对写入session/aligned.h5内部版本，不插值')
    p.add_argument('sessions',nargs='+');p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    p.add_argument('--version','--run-id',dest='run_id',help='H5内部sync版本；唯一版本时可省略')
    a=p.parse_args()
    for session_id in a.sessions:print(export(a.root.resolve(),session_id,a.run_id))
