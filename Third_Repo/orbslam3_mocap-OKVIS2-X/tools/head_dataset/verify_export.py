#!/usr/bin/env python3
"""Check exported sensor chains, nanosecond times, units and actual decoded pixels."""
import csv
import json
from pathlib import Path
import sys
import cv2
import h5py
import numpy as np

out = Path(sys.argv[1])
r = json.loads((out/'adapter_report.json').read_text())
src = Path(r['source'])
fs = cv2.FileStorage(str(out/'okvis.yaml'), cv2.FILE_STORAGE_READ)
mapping = list(csv.DictReader((out/'frame_mapping.csv').open()))
with h5py.File(next(src.glob('*.h5'))) as f:
    c = json.loads(f['calib/calibration_full.json'][()])
    # Supplied datasets use the cam0 -> imu0 legacy profile.
    TSB = np.array(c['imus']['imu0']['extrinsic']['matrix']).reshape(4,4)
    for k, logical in enumerate(['RGB_left','RGB_right']):
        node = fs.getNode('cameras').at(k)
        TSC = np.array([node.getNode('T_SC').at(i).real() for i in range(16)]).reshape(4,4)
        TCB = np.array(c['canonical_camera_transforms']['transforms']['cam'+str(k)]['matrix']).reshape(4,4)
        point = np.array([.2,.1,2.,1.])
        assert np.allclose(np.linalg.solve(TSB,TSC@point),np.linalg.solve(TCB,point)), 'extrinsic chain'
        rows = list(csv.reader((out/f'mav0/cam{k}/data.csv').open()))[1:]
        stamps = np.array([int(a) for a,b in rows])
        assert np.all(np.diff(stamps)>0)
        assert len(rows)==r['frames'][k]==len(mapping)
        assert all((out/f'mav0/cam{k}/data'/b).exists() for a,b in rows)
        indices = f['sensor/camera/'+logical+'/frame_index'][:]
        rawts = f['sensor/camera/'+logical+'/timestamp_ns'][:]
        lookup = dict(zip(indices.tolist(),rawts.tolist()))
        col = 'left' if k==0 else 'right'
        for row, (stamp,name) in zip(mapping,rows):
            assert lookup[int(row[col+'_frame_index'])]==int(stamp)==int(row[col+'_ns'])
        # Verify that the H5 frame index selects the intended video frame.
        cap = cv2.VideoCapture(str(src/'video'/f'{logical}.mp4'))
        cap.set(cv2.CAP_PROP_POS_FRAMES,int(mapping[0][col+'_frame_index']))
        ok,img = cap.read(); assert ok
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        size = tuple(int(node.getNode('image_dimension').at(i).real()) for i in range(2))
        if r.get('projection')=='pinhole':
            binding = c['sensor_bindings']['cam'+str(k)]
            sensor = c['cameras'][binding['legacy_calibration_key']]
            width,height = binding['runtime_resolution']
            norms = np.array(sensor['intrinsics_normalized'])
            fx,fy,cx,cy = norms * [width,height,width,height]
            K = np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])
            focal = node.getNode('focal_length').at(0).real()
            newK = np.array([[focal,0,(size[0]-1)/2],[0,focal,(size[1]-1)/2],[0,0,1.]])
            maps = cv2.fisheye.initUndistortRectifyMap(K,np.array(sensor['distortion_coefficients'][:4]),np.eye(3),newK,size,cv2.CV_32FC1)
            expected = cv2.remap(gray,*maps,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
        else:
            expected = cv2.resize(gray,size,interpolation=cv2.INTER_AREA)
        actual = cv2.imread(str(out/f'mav0/cam{k}/data'/rows[0][1]),cv2.IMREAD_GRAYSCALE)
        assert np.array_equal(expected,actual), 'decoded image / projection mismatch'
        cap.release()
    rawt = f['sensor/imu/imu0/timestamp_ns'][:]
    rawg = f['sensor/imu/imu0/gyro_rad_s'][:]
    rawa = f['sensor/imu/imu0/accel_m_s2'][:]
    data = np.loadtxt(out/'mav0/imu0/data.csv',delimiter=',',skiprows=1)
    assert np.isfinite(data).all() and np.all(np.diff(data[:,0])>0)
    offset = round(r['imu_offset_s']*1e9)
    for i in [0,len(data)//2,len(data)-1]:
        j = np.searchsorted(rawt,int(data[i,0])-offset)
        assert rawt[j]+offset==int(data[i,0])
        assert np.allclose(data[i,1:4],rawg[j]) and np.allclose(data[i,4:7],rawa[j])
fs.release()
print('PASS', out.name, ': extrinsics, timestamp/frame mapping, pixels, IMU axes/units/offset')
