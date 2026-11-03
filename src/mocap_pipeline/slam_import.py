"""Import explicit cam0-FLU outputs from the current OKVIS adapter."""
import csv
import hashlib
import json
from pathlib import Path
import h5py
import numpy as np
from joint_capture.h5store import write_version
from .pose_contract import R_OPTICAL_FLU


def import_cam0(run, session, version, repo):
    run=Path(run);repo=Path(repo)
    source=next((run/f'trajectory_cam0_{name}.csv' for name in ('ba','final') if (run/f'trajectory_cam0_{name}.csv').is_file()),None)
    if source is None:raise FileNotFoundError('缺少新版 cam0 FLU 最终轨迹')
    expected=['timestamp_ns','cam0_tx_world_flu_m','cam0_ty_world_flu_m','cam0_tz_world_flu_m','cam0_flu_qx','cam0_flu_qy','cam0_flu_qz','cam0_flu_qw']
    with source.open() as stream:
        reader=csv.reader(stream)
        if next(reader)!=expected:raise ValueError('cam0 FLU CSV 列定义不匹配')
        rows={int(row[0]):[float(v) for v in row[1:]] for row in reader}
    timestamps=np.array(sorted(rows),dtype=np.int64);poses=np.array([rows[int(t)] for t in timestamps])
    if len(timestamps)<2 or poses.shape!=(len(timestamps),7) or not np.isfinite(poses).all():raise ValueError('cam0 轨迹无效')
    if not np.allclose(np.linalg.norm(poses[:,3:],axis=1),1,atol=1e-5):raise ValueError('cam0 四元数未归一化')
    world=json.loads((run/'world_frame.json').read_text())
    if not np.allclose(world['R_optical_from_cam0_flu'],R_OPTICAL_FLU):raise ValueError('未知 FLU 轴变换')
    quality={}
    with (run/'tracking_quality.csv').open() as stream:
        for row in csv.DictReader(stream):quality[int(row['timestamp_ns'])]=int(row['quality'])
    q=np.array([quality.get(int(t),-1) for t in timestamps],dtype=np.int8)
    dataset=Path(json.loads((run/'viewer_manifest.json').read_text())['dataset'])
    source_files=[repo/'tools/head_dataset'/name for name in ('head_to_okvis.py','pose_frames.py','summarize_run.py','viewer.py')]
    provenance={'source_run':str(run),'repository':str(repo),'trajectory_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'source_fingerprints':{str(p.relative_to(repo)):hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files}}
    def save(group):
        group.attrs.update(schema='ego-okvis-cam0-flu-v1',body_frame='cam0_flu',world_frame='world_flu',transform='T_worldFLU_cam0FLU',position_unit='m',quaternion_order='xyzw',time_domain='EGO CLOCK_MONOTONIC',accuracy_validated=False,source_trajectory=str(source))
        for name,data in [('timestamp_ns',timestamps),('position',poses[:,:3]),('quaternion_xyzw',poses[:,3:]),('tracking_quality',q),('tracking_good',q==0)]:group.create_dataset('pose/'+name,data=data)
        for path in [run/'world_frame.json',run/'summary.json',run/'viewer_manifest.json',dataset/'okvis.yaml',dataset/'adapter_report.json',dataset/'frame_mapping.csv']:
            group.create_dataset('metadata/'+path.name,data=path.read_text(),dtype=h5py.string_dtype())
    write_version(Path(session)/'ego.h5','okvis',version,save,metadata=provenance,require_existing=True)
