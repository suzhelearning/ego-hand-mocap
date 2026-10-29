import struct
import h5py
import numpy as np
import pytest
from joint_capture.storage import RigidH5Writer
from joint_capture.natnet import parse_server_info


def frame(n, bodies):
    return {'frame_number': n, 'motive_timestamp': n/120,
            'publisher_received_time_ns': n*10000000,
            'natnet_timing': {'stamp_camera_mid_exposure': n*83333,
                             'stamp_data_received': n*83333+30000,
                             'stamp_transmit': n*83333+50000},
            'rigid_bodies': bodies}


def body(id):
    return {'id': id, 'position': [1., 2., 3.], 'quaternion_xyzw': [0., 0., 0., 1.],
            'mean_error': .001, 'tracking_valid': True}


def test_server_frequency_requires_complete_serverinfo():
    data=bytearray(272);data[260:264]=bytes([4,2,0,0]);struct.pack_into('<Q',data,264,10000000)
    assert parse_server_info(struct.pack('<HH',1,len(data))+data)==10000000
    for packet in [b'', struct.pack('<HH',1,272)+data[:260],struct.pack('<HH',7,272)+data]:
        with pytest.raises(ValueError):parse_server_info(packet)
    struct.pack_into('<Q',data,264,0)
    with pytest.raises(ValueError):parse_server_info(struct.pack('<HH',1,272)+data)


def test_h5_streaming_preserves_ticks_and_missing_bodies(tmp_path):
    p=tmp_path/'capture.h5'
    with RigidH5Writer(p, 10000000, 'session') as w:
        w.append([frame(10,[body(1)]),frame(11,[])])
        w.append([frame(12,[body(2)])])
    with h5py.File(p) as f:
        assert f['frames/frame_number'][:].tolist()==[10,11,12]
        assert f['frames/natnet_timing/stamp_camera_mid_exposure'][:].tolist()==[833330,916663,999996]
        assert f['rigid_bodies/1/present'][:].tolist()==[True,False,False]
        assert f['rigid_bodies/2/present'][:].tolist()==[False,False,True]
        assert np.isnan(f['rigid_bodies/2/position'][0]).all()
        assert not f.attrs['interpolated']
        assert f.attrs['tick_frequency_hz']==10000000


def test_h5_never_overwrites_existing_file(tmp_path):
    p=tmp_path/'capture.h5';p.write_bytes(b'preserve')
    with pytest.raises(FileExistsError):RigidH5Writer(p,10000000,'session')
    assert p.read_bytes()==b'preserve'
