"""Export real OKVIS CSV estimates without changing source trajectories."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import h5py
import numpy as np
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from joint_capture.runs import finish_stage


def export(run,output):
    run=Path(run);output=Path(output)
    csv_path=run/'okvis2-slam-final-ba_trajectory.csv'
    if not csv_path.exists():csv_path=run/'okvis2-slam-final_trajectory.csv'
    unique={}
    with csv_path.open() as f:
        reader=csv.reader(f);header=[v.strip() for v in next(reader)]
        if header[:4]!=['timestamp','p_WS_W_x','p_WS_W_y','p_WS_W_z']:raise ValueError('Unexpected OKVIS columns')
        for row in reader:
            if len(row)<17:raise ValueError('Expected at least 17 trajectory columns')
            unique[int(row[0].strip())]=[float(x) for x in row[1:17]]
    if not unique:raise ValueError('No estimated poses; refusing empty H5')
    stamps=np.array(sorted(unique),dtype=np.int64);values=np.array([unique[int(t)] for t in stamps])
    if not np.isfinite(values).all():raise ValueError('Non-finite poses')
    if not np.allclose(np.linalg.norm(values[:,3:7],axis=1),1,atol=1e-3):raise ValueError('Invalid quaternion norms')
    quality={}
    with (run/'tracking_quality.csv').open() as f:
        for row in csv.DictReader(f):quality[int(row['timestamp_ns'])]=int(row['quality'])
    q=np.array([quality.get(int(t),-1) for t in stamps],dtype=np.int8)
    output.parent.mkdir(parents=True,exist_ok=True)
    with h5py.File(output,'x') as f:
        f.attrs.update(schema='ego-okvis-pose-v1',transform='T_world_imu',world_frame='OKVIS initialized local world',
            body_frame='imu0_output',time_domain='EGO CLOCK_MONOTONIC',position_unit='m',quaternion_order='xyzw',
            source_trajectory=str(csv_path.resolve()),interpolated=False,accuracy_validated=False,
            quality_semantics='-1=unknown,0=Good,1=Marginal,2=Lost; latest callback at exact timestamp',complete=False)
        for name,data in [('timestamp_ns',stamps),('position',values[:,:3]),('quaternion_xyzw',values[:,3:7]),
            ('velocity_world_m_s',values[:,7:10]),('gyro_bias_rad_s',values[:,10:13]),('accel_bias_m_s2',values[:,13:16]),
            ('tracking_quality',q),('tracking_good',q==0)]:f.create_dataset('pose/'+name,data=data)
        for name in ['summary.json','viewer_manifest.json']:
            if (run/name).exists():f.create_dataset('metadata/'+name,data=(run/name).read_text(),dtype=h5py.string_dtype())
        manifest=run/'viewer_manifest.json'
        if manifest.exists():
            dataset=Path(json.loads(manifest.read_text())['dataset'])
            for name in ['adapter_report.json','okvis.yaml','frame_mapping.csv']:
                f.create_dataset('metadata/'+name,data=(dataset/name).read_text(),dtype=h5py.string_dtype())
        f.attrs['complete']=True
    output=output.resolve()
    parts=output.parts
    if 'sessions' in parts and 'runs' in parts:
        # The output layout is session/runs/run/ego/okvis/pose.h5.
        actual_run_dir=output.parents[2]
        if actual_run_dir.parent.name=='runs' and actual_run_dir.joinpath('run.json').is_file():
            digest=hashlib.sha256(output.read_bytes()).hexdigest()
            finish_stage(actual_run_dir,'okvis_pose_export','complete',
                         outputs=[{'path':output.relative_to(actual_run_dir).as_posix(),'sha256':digest,'size':output.stat().st_size}])
    return output


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();print(export(a.run,a.output))
