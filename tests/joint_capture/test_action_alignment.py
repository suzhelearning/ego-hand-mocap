import json
import h5py
import numpy as np
import pytest
from joint_capture.action_alignment import align_action


def fixture(tmp_path):
    clocks=tmp_path/'clocks';clocks.mkdir()
    rows=[dict(index=i,pc_send_ns=i*10**9+4999000,pc_receive_ns=i*10**9+5001000,
        remote_receive_ticks=i*10**9,remote_send_ticks=i*10**9) for i in range(10)]
    for k in ('ego','windows'):
        (clocks/(k+'.jsonl')).write_text(json.dumps({'type':'metadata','remote_frequency_hz':10**9})+'\n'+''.join(json.dumps(r)+'\n' for r in rows))
    ego=tmp_path/'ego.h5'
    with h5py.File(ego,'w') as f:
        f['metadata/dataset.json']=json.dumps({'clock':{'boot_id':'BOOT','domain':'CLOCK_MONOTONIC'}})
        for k in ('RGB_left','RGB_right'):f['sensor/camera/'+k+'/timestamp_ns']=np.array([2,3,4])*10**9
        f['sensor/imu/imu0/timestamp_ns']=np.array([2,3,4])*10**9
    mocap=tmp_path/'mocap.h5'
    with h5py.File(mocap,'w') as f:
        f.attrs['tick_frequency_hz']=10**9
        f['frames/natnet_timing/stamp_camera_mid_exposure']=np.array([2,3,4])*10**9
    return clocks,ego,mocap


def test_action_mapping_has_same_take_and_pc_clock_without_overwriting_raw(tmp_path):
    clocks,ego,mocap=fixture(tmp_path)
    result=align_action(tmp_path,ego,mocap,clocks,'TAKE','BOOT')
    with h5py.File(tmp_path/'alignment.h5') as f:
        assert f.attrs['take_id']=='TAKE'
        assert f['ego/camera/RGB_left/sync_valid'][:].all()
        assert np.max(abs(f['ego/camera/RGB_left/pc_timestamp_ns'][:]-f['mocap/pc_timestamp_ns'][:]))<=1
        assert 'ego' in f['clock_evidence']
    with h5py.File(ego) as f:assert 'pc_timestamp_ns' not in f['sensor/camera/RGB_left']
    assert result['status']=='mapped'


def test_boot_mismatch_is_rejected(tmp_path):
    clocks,ego,mocap=fixture(tmp_path)
    with pytest.raises(ValueError,match='boot'):align_action(tmp_path,ego,mocap,clocks,'T','WRONG')
