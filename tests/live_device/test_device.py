from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from ego_live.model import installation, world_base, load_urdf, visible_pose

ROOT=Path(__file__).resolve().parents[2]

def test_motive_pyr_is_xyz_not_ros_rpy():
    t=installation([-2,-13,3],[-1,0,10])
    rx=Rotation.from_rotvec(np.deg2rad(-1)*np.array([1,0,0])).as_matrix()
    rz=Rotation.from_rotvec(np.deg2rad(10)*np.array([0,0,1])).as_matrix()
    np.testing.assert_allclose(t[:3,:3],rx@rz,atol=1e-15)
    np.testing.assert_allclose(t[:3,3],[-.002,-.013,.003])

def test_offset_rotates_with_body():
    q=Rotation.from_euler('z',90,degrees=True).as_quat()
    t=world_base([1,2,3],q,installation([100,0,0],[0,0,0]))
    np.testing.assert_allclose(t[:3,3],[1,2.1,3],atol=1e-12)

def test_urdf_geometry_and_joint():
    model=load_urdf(ROOT/'assets/head_ring_description/head_ring.urdf')
    assert model['root']=='base' and len(model['visuals'])==6
    c=np.array(model['frames']['left_cam'])
    np.testing.assert_allclose(c[:3,3],[-.018232200227934,-.067545877323258,.011136163289632])
    expected=Rotation.from_euler('xyz',[0,np.pi/4,-np.pi/12]).as_matrix()
    np.testing.assert_allclose(c[:3,:3],expected,atol=1e-12)

def test_no_stale_or_invalid_pose():
    assert visible_pose(True,.1)
    assert not visible_pose(False,.1)
    assert not visible_pose(True,.6)

def test_live_state_selects_named_body_and_hides_lost_or_missing():
    import time
    from ego_live.server import LiveState
    state=LiveState('ego',None,installation([-2,-13,3],[-1,0,10]),'z')
    assert not state.snapshot()['valid']
    body={'id':10,'position':[1,2,3],'quaternion_xyzw':[0,0,0,1], 'tracking_valid':True,'mean_error':.001}
    frame={'frame_number':42,'motive_timestamp':1.,'publisher_received_time_ns':time.monotonic_ns(),'rigid_bodies':[body]}
    state.push(frame,{10:'ego'})
    result=state.snapshot()
    assert result['valid'] and result['body_id']==10
    np.testing.assert_allclose(np.array(result['T_Wbase'])[:3,3],[.998,1.987,3.003])
    body['tracking_valid']=False
    assert not state.snapshot()['valid'] and 'T_Wbase' not in state.snapshot()
    body['tracking_valid']=True
    frame['publisher_received_time_ns']-=1_000_000_000
    assert not state.snapshot()['valid']
    frame['publisher_received_time_ns']=time.monotonic_ns()
    state.push(frame,{10:'other'})
    assert not state.snapshot()['valid']

def test_y_up_changes_world_only_not_body_local_offset():
    import time
    from ego_live.server import LiveState
    state=LiveState('ego',10,installation([0,100,0],[0,0,0]),'y')
    state.push({'frame_number':1,'motive_timestamp':0,'publisher_received_time_ns':time.monotonic_ns(),
                'rigid_bodies':[dict(id=10,position=[1,2,3],quaternion_xyzw=[0,0,0,1],tracking_valid=True,mean_error=0)]},{10:'ego'})
    np.testing.assert_allclose(np.array(state.snapshot()['T_Wbase'])[:3,3],[1,-3,2.1])

def test_motive_marker_basis_precedes_geometry_offset():
    from ego_live.model import marker_basis
    c=marker_basis('motive_y_up')
    # Native Motive local +Z is streamed-body -Y; +Y is streamed-body +Z.
    np.testing.assert_allclose(c[:3,:3]@np.array([0,0,1]),[0,-1,0],atol=1e-12)
    g=installation([-2,-13,3],[-1,0,10])
    np.testing.assert_allclose((c@g)[:3,3],[-.002,-.003,-.013],atol=1e-12)
    np.testing.assert_allclose(marker_basis('natnet'),np.eye(4))

def test_recorded_markers_confirm_streamed_local_basis():
    # Independent sample from Motive: corresponding native-local and world marker.
    from ego_live.model import marker_basis
    q=[.013816013000905514,.11905994266271591,-.05257837101817131,.9913977384567261]
    p=[.2735781669616699,.051591500639915466,.024188965559005737]
    local=np.array([.007133756764233112,-.0008635020931251347,.026956139132380486,1])
    measured=np.array([.27755242586135864,.0241157915443182,.021171366795897484])
    predicted=world_base(p,q,marker_basis('motive_y_up'))@local
    assert np.linalg.norm(predicted[:3]-measured)<.0005
    assert np.linalg.norm((world_base(p,q,np.eye(4))@local)[:3]-measured)>.02

def test_live_config_reproduces_motive_obj_and_stream_axes():
    import json
    from ego_live.server import configured_installation
    config=json.loads((ROOT/'config/ego_device_geometry.json').read_text())
    c=np.eye(4);c[:3,:3]=[[1,0,0],[0,0,-1],[0,1,0]]
    d=c.T  # measured STL -> Blender OBJ conversion, zero origin shift
    g=installation([-2,-13,3],[-1,0,10])
    np.testing.assert_allclose(configured_installation(config),c@g@d,atol=1e-15)
    np.testing.assert_allclose(configured_installation(config,'y'),g@d,atol=1e-15)
    # Same mesh vertex in streamed Z-up world via native-world and viewer chains.
    w=np.eye(4);w[:3,:3]=Rotation.from_euler('xyz',[.3,-.5,.7]).as_matrix();w[:3,3]=[1,2,3]
    streamed=c@w@c.T
    v=np.array([-.023159,.0775,-.008582,1])
    np.testing.assert_allclose(streamed@configured_installation(config)@v,c@w@g@d@v,atol=1e-12)
