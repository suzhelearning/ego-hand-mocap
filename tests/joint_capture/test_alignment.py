import numpy as np
import pytest
from joint_capture.alignment import nearest, fit_clock, map_time


def test_nearest_tie_and_edges():
    assert nearest(np.array([10,20,30]),np.array([0,15,29,40])).tolist()==[0,0,2,2]
    with pytest.raises(ValueError):nearest(np.array([20,10]),np.array([15]))


def test_clock_fit_preserves_causal_bounds():
    rows=[dict(pc_send_ns=i*1000000000+4999000,pc_receive_ns=i*1000000000+5001000,
               remote_receive_ticks=i*1000000000,remote_send_ticks=i*1000000000) for i in range(10)]
    m=fit_clock(rows,1000000000)
    mapped=map_time(np.arange(10,dtype=np.int64)*1000000000,m)
    assert np.all(np.abs(mapped-(np.arange(10)*1000000000+5000000))<=1000)


def test_incompatible_probes_rejected():
    rows=[dict(pc_send_ns=10000*i,pc_receive_ns=10000*i+1,remote_receive_ticks=1,remote_send_ticks=1) for i in range(4)]
    with pytest.raises(ValueError):fit_clock(rows,1000000000)


def test_h5_sync_publication_without_run_directory(tmp_path,monkeypatch):
    import json,h5py
    import joint_capture.alignment as module
    base=tmp_path/'data/sessions/example';base.mkdir(parents=True)
    (base/'session.json').write_text(json.dumps({'session_id':'example','assets':{}}))
    def fixture(session,output):
        output.mkdir()
        (output/'clock_models.json').write_text('{"ego":{},"windows":{}}')
        (output/'summary.json').write_text('{"cameras":{"RGB_left":{}},"physical_exposure_alignment_valid":false}')
        (output/'RGB_left_nearest.csv').write_text('rgb_index,rgb_pc_ns,delta_ns,within_mocap_coverage\n0,100,2,True\n')
        return {'cameras':{'RGB_left':{}}}
    monkeypatch.setattr(module,'align',fixture)
    path=module.align_to_h5(base,'v001')
    assert path==base/'aligned.h5'
    assert not (base/'runs').exists()
    with h5py.File(path) as f:
        assert f['processing/sync/v001/cameras/RGB_left/delta_ns'][:].tolist()==[2]
        assert f['processing/sync/v001'].attrs['complete']
