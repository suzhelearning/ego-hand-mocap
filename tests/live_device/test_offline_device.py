from pathlib import Path
import numpy as np
from mocap_pipeline.device import prepare_device
ROOT=Path(__file__).resolve().parents[2]

def test_unknown_reference_body_does_not_guess_transform(tmp_path):
    report=prepare_device(ROOT,{'body_source':'world_preview/old','T_B_ego_rigidbody':None},tmp_path)
    assert not report['available'] and report['T_B_base'] is None
    assert len(report['model']['visuals'])==6
    assert all((tmp_path/v['file']).is_file() for v in report['model']['visuals'])

def test_known_reference_bridge_composes_once(tmp_path):
    bridge=np.eye(4);bridge[:3,3]=[.1,.2,.3]
    report=prepare_device(ROOT,{'body_source':'verified','T_B_ego_rigidbody':bridge},tmp_path)
    assert report['available']
    np.testing.assert_allclose(report['T_B_base'],bridge@np.array(report['T_ego_rigidbody_base']))

def test_nine_marker_recovery_and_urdf_composition(tmp_path):
    import json
    from scipy.spatial.transform import Rotation
    from mocap_pipeline.reference import fit_device_markers
    from ego_live.model import configured_installation
    config=json.loads((ROOT/'config/ego_device_geometry.json').read_text())
    c=np.array(config['stream_local_from_native_marker'])
    local=np.array(config['native_marker_template']['positions_m'])@c[:3,:3].T
    expected=np.eye(4);expected[:3,:3]=Rotation.from_euler('xyz',[.2,-.3,.4]).as_matrix();expected[:3,3]=[1,2,3]
    measured=local@expected[:3,:3].T+expected[:3,3]
    poses,rms,valid=fit_device_markers(local,np.array([measured,measured]),np.array([True,False]))
    np.testing.assert_allclose(poses[0],expected,atol=1e-12)
    assert rms[0]<1e-12 and valid.tolist()==[True,False]
    report=prepare_device(ROOT,{'body_source':'9-marker','T_B_ego_rigidbody':np.eye(4)},tmp_path)
    np.testing.assert_allclose(poses[0]@report['T_B_base'],expected@configured_installation(config),atol=1e-12)
