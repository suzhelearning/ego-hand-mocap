import copy
import pytest
from motion_web.data import normalize_motion, demo_pair, load_bytes


def sample():
    return {'version': 1, 'unit': 'mm', 'coordinate_system': 'camera',
            'bones': [[0, 1]], 'frames': [
                {'time_s': 10, 'points': [[1000, 0, 0], None]},
                {'time_s': 10.1, 'points': [[0, 200, 0], [0, 0, 0]], 'valid': False}]}


def test_converts_units_preserves_missing_and_invalid():
    data = normalize_motion(sample())
    assert data['frames'][0]['points'] == [[1, 0, 0], None]
    assert data['frames'][0]['time_s'] == 0
    assert data['source_start_s'] == 10
    assert data['frames'][1]['valid'] is False


@pytest.mark.parametrize('change', [
    lambda d: d.update(unit='inch'),
    lambda d: d.update(coordinate_system=''),
    lambda d: d.update(frames=[]),
    lambda d: d['frames'][1].update(time_s=10),
    lambda d: d['frames'][0].update(points=[[float('nan'), 0, 0], None]),
    lambda d: d.update(bones=[[0, 5]]),
    lambda d: d['frames'][1].update(points=[[0, 0, 0]]),
    lambda d: d['frames'][0].update(valid='false'),
])
def test_rejects_invalid_motion(change):
    data = copy.deepcopy(sample())
    change(data)
    with pytest.raises(ValueError):
        normalize_motion(data)


def test_demo_is_explicit_and_has_moving_paired_hands():
    pair = demo_pair()
    for data in pair.values():
        assert data['synthetic'] is True
        assert len(data['frames'][0]['points']) == 42
        assert data['frames'][0]['points'] != data['frames'][30]['points']
    assert pair['ego']['frames'][0]['points'] != pair['mocap']['frames'][0]['points']


def test_unsupported_format_and_malformed_json_are_actionable():
    for name, body in [('x.c3d', b'raw'), ('x.json', b'{'), ('x.json', b'[]')]:
        with pytest.raises(ValueError):
            load_bytes(name, body)


def test_natnet_h5_only_previews_valid_rigid_positions():
    import io
    h5py = pytest.importorskip('h5py')
    stream = io.BytesIO()
    with h5py.File(stream, 'w') as f:
        f.attrs['schema'] = 'ego-natnet-raw-rigid-v1'
        f.attrs['position_unit'] = 'm'
        f['frames/motive_timestamp'] = [120.0, 120.1]
        f['rigid_bodies/1/position'] = [[1, 2, 3], [4, 5, 6]]
        f['rigid_bodies/1/tracking_valid'] = [True, False]
    result = load_bytes('rigid.h5', stream.getvalue())
    assert result['kind'] == 'rigid_positions'
    assert result['bones'] == []
    assert result['frames'][0]['points'] == [[1, 2, 3]]
    assert result['frames'][1]['points'] == [None]


def test_ego_h5_does_not_invent_hand_joints():
    import io
    h5py = pytest.importorskip('h5py')
    stream = io.BytesIO()
    with h5py.File(stream, 'w') as f:
        f['imu'] = [1, 2, 3]
    with pytest.raises(ValueError, match='motion JSON'):
        load_bytes('ego.h5', stream.getvalue())


@pytest.mark.parametrize('change', [
    lambda d: d.update(unit=[]),
    lambda d: d['frames'][0].update(time_s=10**400),
    lambda d: d['frames'][0].update(points=[[10**400, 0, 0], None]),
    lambda d: (d['frames'][0].update(time_s=-1e308), d['frames'][1].update(time_s=1e308)),
])
def test_extreme_json_values_rejected_as_validation_errors(change):
    data = sample()
    change(data)
    with pytest.raises(ValueError):
        normalize_motion(data)


def test_aligned_h5_uses_rgb_clock_and_masks_unusable():
    import io
    import h5py
    from motion_web.data import load_bytes
    buf=io.BytesIO()
    with h5py.File(buf,'w') as f:
        f.attrs['schema']='ego-mocap-nearest-aligned-v1'
        g=f.create_group('cameras/RGB_left')
        g['rgb_pc_ns']=[100000000000,100033333333]
        g['rigid_bodies/10/position']=[[1.,2.,3.],[4.,5.,6.]]
        g['rigid_bodies/10/usable']=[True,False]
    result=load_bytes('aligned.h5',buf.getvalue())
    assert result['frames'][0]['time_s']==0
    assert abs(result['frames'][1]['time_s']-.033333333)<1e-9
    assert result['frames'][0]['points']==[[1.,2.,3.]]
    assert result['frames'][1]['points']==[None]
