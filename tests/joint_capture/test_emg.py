import struct
from binascii import crc_hqx
import numpy as np
from joint_capture.emg import Parser, unwrap, fit_passive


def frame(seq, stamp, kind=1):
    payload=struct.pack('<I16i',0,*range(16)) if kind==1 else struct.pack('<II',0,0)
    body=struct.pack('<4sBBHIH',b'EMG0',1,kind,seq,stamp,len(payload))+payload
    return body+struct.pack('<H',crc_hqx(body,0xffff))


def test_split_corrupt_and_shared_sequence():
    p=Parser();good=frame(65535,0xfffffff0)
    assert not p.feed(good[:9])
    assert p.feed(good[9:])[0]['seq']==65535
    corrupt=bytearray(frame(0,0,2));corrupt[-1]^=1
    rows=p.feed(bytes(corrupt)+frame(0,0,2)+frame(1,20))
    assert [r['seq'] for r in rows]==[0,1]
    assert p.crc_errors==1


def test_unwrap_across_types_and_rollover():
    value,high=unwrap(0xffffff00,None)
    value,high=unwrap(0x2e8,high)
    assert value==0x1000002e8
    older,high2=unwrap(0xffffff80,high)
    assert older==0xffffff80 and high2==high


def test_passive_keeps_unknown_delay_and_uses_batches():
    rows=[]
    for i in range(50):
        d=i*1_000_000
        rows.append((i,10_000_000_000+int(d*1000*1.00002)+2_000_000,d))
    m=fit_passive(np.array(rows,dtype=np.int64))
    assert m['status']=='passive_coarse'
    assert m['unknown_fixed_delay'] is True
    assert abs(m['apparent_rate_ppm']-20)<.01
    assert fit_passive(np.array(rows[:2]))['status']=='insufficient_data'


def test_mapping_keeps_extrapolation_visible():
    from joint_capture.emg import map_passive
    model=dict(status='passive_coarse',a=1.,b_relative_s=.005,device_origin_us=1000,
               host_origin_ns=1000000000,device_valid_interval_us=[1000,3000])
    mapped,extrapolated=map_passive(np.array([0,1000,2000,4000]),model)
    assert mapped.tolist()==[1004000000,1005000000,1006000000,1008000000]
    assert extrapolated.tolist()==[True,False,False,True]


def test_writer_flushes_partial_tail_and_keeps_read_boundaries(tmp_path):
    import h5py
    from joint_capture.emg import Wristband
    w=Wristband('left','unused',tmp_path);w.identity='fixture';w.stop.set()
    w.q.put((7,1000000000,frame(1,100)+frame(2,1100)))
    w._write()
    assert not w.error
    with h5py.File(tmp_path/'left.h5') as f:
        assert f['emg'].shape==(2,23)
        assert f['emg'][:,0].tolist()==[7,7]
        assert f['emg'][:,1].tolist()==[1000000000,1000000000]
        assert f.attrs['complete']
        assert 'emg_coarse_alignment' not in f


def test_writer_preserves_tail_on_sequence_reset(tmp_path):
    import h5py
    from joint_capture.emg import Wristband
    w=Wristband('left','unused',tmp_path);w.identity='fixture';w.stop.set()
    w.q.put((0,1000000000,frame(10,100)+frame(1,1100)))
    w._write()
    assert w.error
    with h5py.File(tmp_path/'left.h5') as f:
        assert len(f['emg'])==1
        assert not f.attrs['complete']
    assert (tmp_path/'left.bin').stat().st_size==168
