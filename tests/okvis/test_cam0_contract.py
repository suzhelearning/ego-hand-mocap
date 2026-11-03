import json
from pathlib import Path
import numpy as np
import h5py
import pytest
from scipy.spatial.transform import Rotation
from mocap_pipeline.pose_contract import source_transforms, R_OPTICAL_FLU
from mocap_pipeline.slam_import import import_cam0


def transform(rot, xyz):
    t=np.eye(4);t[:3,:3]=Rotation.from_euler('xyz',rot).as_matrix();t[:3,3]=xyz;return t


def test_cam0_flu_and_imu_chains_are_equivalent_after_fixed_anchor():
    li=np.array([transform([.3,.1,.7],[.2,.3,.4]),transform([.4,-.2,.9],[.7,.3,.2])])
    ic=transform([.5,.2,-.3],[.01,-.02,.03]);ib=transform([-.5,.8,.1],[.1,.2,.3])
    h=transform([0,0,.6],[-.1,.2,.4]);wb=transform([.2,.4,.6],[1,2,3])
    b=np.eye(4);b[:3,:3]=R_OPTICAL_FLU
    lf=h@li@ic@b
    fb,fc=source_transforms('cam0_flu',ib,ic)
    old_wl=wb@np.linalg.inv(li[0]@ib);new_wl=wb@np.linalg.inv(lf[0]@fb)
    np.testing.assert_allclose(new_wl@lf@fb,old_wl@li@ib,atol=1e-12)
    np.testing.assert_allclose(new_wl@lf@fc,old_wl@li@ic,atol=1e-12)
    np.testing.assert_allclose(fc[:3,:3]@np.array([1,2,3]),[3,-1,-2])
    with pytest.raises(ValueError):source_transforms('unknown',ib,ic)


def test_import_cam0_preserves_timestamp_and_declares_pose_frame(tmp_path):
    run=tmp_path/'run';run.mkdir();dataset=tmp_path/'dataset';dataset.mkdir();repo=tmp_path/'repo/tools/head_dataset';repo.mkdir(parents=True)
    for name in ['head_to_okvis.py','pose_frames.py','summarize_run.py','viewer.py']:(repo/name).write_text('# fixture')
    stamp=12345678901234567
    header='timestamp_ns,cam0_tx_world_flu_m,cam0_ty_world_flu_m,cam0_tz_world_flu_m,cam0_flu_qx,cam0_flu_qy,cam0_flu_qz,cam0_flu_qw\n'
    (run/'trajectory_cam0_ba.csv').write_text(header+f'{stamp},0,0,0,0,0,0,1\n{stamp+1},1,2,3,0,0,0,1\n')
    (run/'tracking_quality.csv').write_text(f'timestamp_ns,quality\n{stamp},0\n{stamp+1},2\n')
    (run/'world_frame.json').write_text(json.dumps({'R_optical_from_cam0_flu':R_OPTICAL_FLU.tolist()}))
    (run/'viewer_manifest.json').write_text(json.dumps({'dataset':str(dataset)}));(run/'summary.json').write_text('{}')
    for name in ['okvis.yaml','adapter_report.json','frame_mapping.csv']:(dataset/name).write_text('fixture')
    with h5py.File(tmp_path/'ego.h5','w') as f:f['original']=[42]
    import_cam0(run,tmp_path,'cam0-test',repo.parents[1])
    with h5py.File(tmp_path/'ego.h5') as f:
        g=f['processing/okvis/cam0-test'];assert g.attrs['body_frame']=='cam0_flu';assert g['pose/timestamp_ns'][0]==stamp
        assert g['pose/tracking_good'][:].tolist()==[True,False];assert f['original'][0]==42
