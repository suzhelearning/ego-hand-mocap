from types import SimpleNamespace
import json
import numpy as np
import pytest
from acquisition.alignment import interpolate_rigid_poses
from acquisition.object_offset import pose_matrix, camera_world_pose, world_points_to_camera
from natnet_zenoh.natnet_adapter import convert_frame
from mocap_tools.track_rigid import RigidTracker


def test_interpolate_large_timestamp_rotation_and_reject_gaps():
    base = 1_700_000_000_000_000_000
    t = np.array([base, base+100, base+300], dtype=np.int64)
    p = [[0,0,0], [2,0,0], [4,0,0]]
    q = [[0,0,0,1], [0,0,1,0], [0,0,1,0]]
    targets = np.array([base-1, base+50, base+200, base+300, base+301], dtype=np.int64)
    pos, rot, valid = interpolate_rigid_poses(t,p,q,[True]*3,targets,max_gap_ns=100)
    assert valid.tolist() == [False,True,False,True,False]
    np.testing.assert_allclose(pos[1], [1,0,0])
    np.testing.assert_allclose(np.abs(rot[1]), [0,0,2**-.5,2**-.5], atol=1e-12)


def test_invalid_tracking_blocks_interpolation():
    _, _, valid = interpolate_rigid_poses(np.array([0,100]), [[0,0,0]]*2,
        [[0,0,0,1]]*2, [True,False], np.array([0,50,100]),max_gap_ns=100)
    assert valid.tolist() == [True,False,False]


def test_transform_roundtrip_with_rotation_and_offset():
    wb = pose_matrix([1,2,3], [0,0,2**-.5,2**-.5])
    bc = pose_matrix([.1,.2,.3], [2**-.5,0,0,2**-.5])
    wc = camera_world_pose(wb, bc)
    pc = np.array([[0,0,1],[.2,.3,.4]])
    pw = pc @ wc[:3,:3].T + wc[:3,3]
    np.testing.assert_allclose(world_points_to_camera(pw,wb,bc), pc, atol=1e-12)


def test_raw_ticks_preserved_without_inventing_clock_frequency():
    frame = convert_frame({'frame_number':1,'timestamp':1.,'mocap_data':SimpleNamespace(
        suffix_data=SimpleNamespace(stamp_camera_mid_exposure=123456789012345678))},10)
    assert frame['natnet_timing']['stamp_camera_mid_exposure'] == 123456789012345678
    assert frame['natnet_timing']['stamp_transmit'] is None
    assert frame['natnet_timing']['tick_frequency_hz'] is None


def test_record_preserves_timing_and_refuses_overwrite(tmp_path):
    tracker = RigidTracker(names=[], ids=[1], connect_endpoint='tcp/127.0.0.1:7447',
        output=None, stats_interval_s=2., duration_s=1.)
    tracker._wanted_ids = {1}
    import io
    tracker._stream = io.StringIO()
    frame = {'frame_number':2,'motive_timestamp':1.5,'publisher_received_time_ns':100,
        'natnet_timing':{'stamp_camera_mid_exposure':123},'rigid_bodies':[
        {'id':1,'position':[0,0,0],'quaternion_xyzw':[0,0,0,1],'tracking_valid':False}]}
    tracker._on_frame(SimpleNamespace(payload=SimpleNamespace(to_string=lambda:json.dumps(frame))))
    record = json.loads(tracker._stream.getvalue())
    assert record['natnet_timing'] == frame['natnet_timing']
    assert record['publisher_received_time_ns'] == 100
    assert record['tracking_valid'] is False
    path = tmp_path/'existing.jsonl'
    path.write_text('keep')
    tracker._output = path
    with pytest.raises(FileExistsError):
        tracker.run()
    assert path.read_text() == 'keep'
