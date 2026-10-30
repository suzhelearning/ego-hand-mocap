import importlib.util
from pathlib import Path
import h5py
import pytest
spec=importlib.util.spec_from_file_location('pose_export',Path(__file__).resolve().parents[2]/'tools/okvis_pose_to_h5.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def test_exact_nanoseconds_latest_pose_and_quality(tmp_path):
    header='timestamp,p_WS_W_x,p_WS_W_y,p_WS_W_z,qx,qy,qz,qw,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz\n'
    stamp=12345678901234567
    (tmp_path/'okvis2-slam-final_trajectory.csv').write_text(header+f'{stamp},1,2,3,0,0,0,1,0,0,0,0,0,0,0,0,0\n'+f'{stamp},4,5,6,0,0,0,1,0,0,0,0,0,0,0,0,0\n')
    (tmp_path/'tracking_quality.csv').write_text(f'timestamp_ns,state_id,quality,is_keyframe\n{stamp},1,2,1\n')
    p=m.export(tmp_path,tmp_path/'pose.h5')
    with h5py.File(p) as f:
        assert int(f['pose/timestamp_ns'][0])==stamp
        assert len(f['pose/timestamp_ns'])==1
        assert f['pose/position'][0].tolist()==[4,5,6]
        assert not f['pose/tracking_good'][0]
    with pytest.raises(FileExistsError):m.export(tmp_path,p)


def test_append_retains_raw_and_rejects_duplicate(tmp_path):
    (tmp_path/'okvis2-slam-final_trajectory.csv').write_text('timestamp,p_WS_W_x,p_WS_W_y,p_WS_W_z,qx,qy,qz,qw,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz\n123,1,2,3,0,0,0,1,0,0,0,0,0,0,0,0,0\n')
    (tmp_path/'tracking_quality.csv').write_text('timestamp_ns,quality\n123,0\n')
    with h5py.File(tmp_path/'ego.h5','w') as f:f['original']=[42]
    m.append_to_ego(tmp_path,tmp_path,'v1')
    with h5py.File(tmp_path/'ego.h5') as f:
        assert f['original'][:].tolist()==[42]
        assert f['processing/okvis/v1/pose/timestamp_ns'][:].tolist()==[123]
    before=(tmp_path/'ego.h5').read_bytes()
    with pytest.raises(FileExistsError):m.append_to_ego(tmp_path,tmp_path,'v1')
    assert (tmp_path/'ego.h5').read_bytes()==before
