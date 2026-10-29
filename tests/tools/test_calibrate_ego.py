import json
from types import SimpleNamespace
import pytest
from mocap_tools.calibrate_ego import CalibrationRecorder, attach_image


def sample(frame):
    return SimpleNamespace(payload=SimpleNamespace(to_string=lambda:json.dumps(frame)))


def frame(valid=True):
    return {'schema_version':1,'frame_number':123,'motive_timestamp':10.,
        'publisher_received_time_ns':100,'coordinate_system':'motive_x_forward_z_up_right_handed',
        'unit':'meter','publisher_dropped_frames':0,'markers':[],
        'natnet_timing':{'stamp_camera_mid_exposure':999},'rigid_bodies':[
            {'id':10,'position':[1,2,3],'quaternion_xyzw':[0,0,0,1],
             'mean_error':.001,'tracking_valid':valid}]}


def test_capture_groups_preserve_original_frames_and_invalid_tracking(tmp_path):
    root=tmp_path/'capture'
    r=CalibrationRecorder(root)
    with pytest.raises(ValueError): r.start_group()
    r.on_names(sample({'names':{'10':'ego'}}))
    r.on_frame(sample(frame()))
    assert r.start_group()=='0001'
    r.on_frame(sample(frame()))
    r.on_frame(sample(frame(False)))
    f=frame(); f['rigid_bodies']=[]; r.on_frame(sample(f))
    group=r.end_group()
    assert group['received_frames']==3 and group['valid_rigid_frames']==1
    r.close()
    rows=[json.loads(x) for x in (root/'mocap.jsonl').read_text().splitlines()]
    assert len(rows)==4 and rows[0]['frame']==frame()
    poses=[json.loads(x) for x in (root/'samples/0001/ego.jsonl').read_text().splitlines()]
    assert poses[-1]['rigid_body'] is None
    assert not poses[1]['rigid_body']['tracking_valid']
    assert json.loads((root/'session.json').read_text())['status']=='complete'
    with pytest.raises(FileExistsError): CalibrationRecorder(root)


def test_close_preserves_active_group_and_interruption(tmp_path):
    r=CalibrationRecorder(tmp_path/'session',rigid_id=10)
    r.start_group();r.on_frame(sample(frame()));r.close('interrupted');r.close()
    meta=json.loads((r.root/'session.json').read_text())
    assert meta['status']=='interrupted' and len(meta['groups'])==1


def test_attach_image_preserves_timestamp_and_refuses_replacement(tmp_path):
    root=tmp_path/'session';r=CalibrationRecorder(root,rigid_id=10)
    r.start_group();r.on_frame(sample(frame()));r.close()
    image=tmp_path/'source.png';image.write_bytes(b'test-fixture')
    args=SimpleNamespace(session=root,group='0001',image=image,timestamp_ns=123456789,
        clock_domain='ego-device-boot-123',frame_index=42)
    attach_image(args)
    info=json.loads((root/'samples/0001/camera/image.json').read_text())
    assert info['capture_timestamp_ns']==123456789
    assert not info['synchronization_verified']
    with pytest.raises(FileExistsError): attach_image(args)


def test_no_overwrite_or_image_attach_during_recording(tmp_path):
    r=CalibrationRecorder(tmp_path/'session',rigid_id=10)
    with pytest.raises(ValueError):
        attach_image(SimpleNamespace(session=r.root))
    r.close()
