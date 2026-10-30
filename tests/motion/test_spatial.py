import json
import numpy as np
import h5py
import pytest
from motion_web.spatial import to_mocap_world


def motion():
    return {'coordinate_system': 'cam0_optical', 'warnings': [], 'frames': [
        {'time_s': 0., 'points': [[1., 0., 0.], None], 'valid': True, 'rgb_index': 7},
        {'time_s': .1, 'points': [[1., 0., 0.], None], 'valid': True, 'rgb_index': 8}]}


@pytest.fixture
def setup(tmp_path):
    session = tmp_path / 'data/sessions/s1'
    session.mkdir(parents=True)
    calibration = tmp_path / 'extrinsics.json'
    T = np.eye(4); T[:3, 3] = [0, 2, 0]
    calibration.write_text(json.dumps({'T_BC': T.tolist(), 'from_frame': 'cam0_optical',
        'to_frame': 'ego', 'rigid_body_id': 10, 'position_unit': 'm',
        'status': 'candidate_not_independently_validated'}))
    with h5py.File(session / 'aligned.h5', 'w') as f:
        g = f.create_group('processing/alignment/v1'); g.attrs.update(complete=True, position_unit='m')
        c = g.create_group('cameras/RGB_left')
        c['rgb_index'] = [6, 7, 8]; c['rgb_pc_ns'] = [1000000000, 1100000000, 1200000000]
        c['clock_pair_usable'] = [True, True, True]; c['delta_ns'] = [0, 100, 200]
        b = c.create_group('rigid_bodies/10'); b.attrs['name'] = 'ego'
        b['position'] = [[0., 0, 0], [10, 0, 0], [20, 0, 0]]
        b['quaternion_xyzw'] = [[0., 0, 0, 1], [0, 0, 2**-.5, 2**-.5], [0, 0, 0, 1]]
        b['usable'] = [True, True, False]
    return session, calibration


def test_camera_transform_uses_rgb_indices_and_dynamic_pose(setup):
    session, calibration = setup
    source = motion()
    result = to_mocap_world(source, session, calibration, 'v1')
    assert result['coordinate_system'] == 'mocap_world'
    assert result['frames'][0]['points'][0] == pytest.approx([8., 1., 0.])
    assert result['frames'][0]['points'][1] is None
    assert result['frames'][0]['time_s'] == pytest.approx(.1)
    assert result['frames'][1]['valid'] is False
    assert source['frames'][0]['points'][0] == [1., 0, 0]
    assert result['spatial']['calibration_status'] == 'candidate_not_independently_validated'


def test_camera_requires_pairing_and_exact_frame_identity(setup):
    session, calibration = setup
    data = motion(); del data['frames'][0]['rgb_index']
    with pytest.raises(ValueError, match='rgb_index'): to_mocap_world(data, session, calibration, 'v1')
    with pytest.raises(ValueError): to_mocap_world(motion(), session, calibration, 'missing')


def test_world_identity_and_unknown_frame(setup):
    session, calibration = setup
    data = motion(); data['coordinate_system'] = 'mocap_world'
    assert to_mocap_world(data, session, calibration)['frames'] == data['frames']
    data['coordinate_system'] = 'wrist_relative'
    with pytest.raises(ValueError): to_mocap_world(data, session, calibration)


def test_explicit_session_world_transform(setup):
    session, calibration = setup
    T = np.eye(4); T[0, 3] = 3
    (session / 'visualization.json').write_text(json.dumps({'world_transforms': {
        'ego_world': {'T_WSource': T.tolist(), 'position_unit': 'm', 'status': 'candidate'}}}))
    data = motion(); data['coordinate_system'] = 'ego_world'
    result = to_mocap_world(data, session, calibration)
    assert result['frames'][0]['points'][0] == [4, 0, 0]


def test_http_world_conversion_and_cross_session_rejection(setup):
    from motion_web.server import make_server
    from urllib.request import urlopen
    from urllib.error import HTTPError
    import threading
    session, calibration = setup
    source = motion(); source.update(version=1, unit='m')
    path = session / 'vision.json'; path.write_text(json.dumps(source))
    server = make_server(session.parents[1], port=0, extrinsics=calibration)
    thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
    try:
        base = f'http://127.0.0.1:{server.server_port}/api/load?path=sessions/s1/vision.json&target=mocap_world&alignment_version=v1'
        result = json.load(urlopen(base))
        assert result['frames'][0]['points'][0] == pytest.approx([8., 1., 0.])
        assert result['spatial']['mode'] == 'camera_dynamic'
        assert result['timeline_id'] == 's1/alignment/v1/RGB_left'
        with pytest.raises(HTTPError) as error:
            urlopen(base + '&alignment_session=other')
        assert error.value.code == 400
        assert '其他会话' in json.load(error.value)['error']
        assert json.loads(path.read_text())['coordinate_system'] == 'cam0_optical'
    finally:
        server.shutdown(); server.server_close(); thread.join()


def test_camera_cannot_select_ambiguous_version_or_invent_unmatched_frames(setup):
    session, calibration = setup
    with h5py.File(session / 'aligned.h5', 'a') as f:
        f.copy('processing/alignment/v1', 'processing/alignment/v2')
    with pytest.raises(ValueError, match='显式指定'):
        to_mocap_world(motion(), session, calibration)
    data = motion(); data['frames'][0]['rgb_index'] = 100
    with pytest.raises(ValueError, match='不在'):
        to_mocap_world(data, session, calibration, 'v1')
