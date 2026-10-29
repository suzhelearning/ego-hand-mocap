#!/usr/bin/env python3
"""Export head-v3 H5 + MP4 to OKVIS' EuRoC reader, preserving capture timestamps.

Uses the bundled camera/IMU layout modules to resolve calibration and input axes.
The input dataset is never modified. Output must be a new directory.
"""
import argparse
import csv
import json
import os
from pathlib import Path

import cv2
import h5py
import numpy as np


def configure(template, cameras, imu, noise_acc):
    text = template.read_text()
    text = text[:text.index('cameras:')] + 'cameras:\n' + ''.join(
        '    - {' + ', '.join(k + ': ' + json.dumps(v) for k,v in c.items()) + '}\n' for c in cameras
    ) + text[text.index('# additional camera parameters'):]
    values = {
        'timestamp_tolerance': 0.001, 'image_delay': 0.0,
        'deep_stereo_indices': [], 'a_max': 39.2266, 'g_max': 17.872,
        'sigma_g_c': imu['gyroscope_noise_density'],
        'sigma_a_c': noise_acc if noise_acc is not None else imu['accelerometer_noise_density'],
        'sigma_gw_c': imu['gyroscope_random_walk'],
        'sigma_aw_c': imu['accelerometer_random_walk'],
        'sigma_bg': 0.05, 'sigma_ba': 0.3, 'g': 9.80665,
        'g0': [0., 0., 0.], 'a0': [0., 0., 0.],
        'detection_threshold': 25.0, 'max_num_keypoints': 1200,
        'display_topview': False, 'display_matches': False,
        'display_overhead': False, 'enable_submapping': False,
    }
    import re
    for key, value in values.items():
        text, n = re.subn(r'(?m)^(\s*' + key + r':)[^\n]*',
                          lambda m: m[1] + ' ' + json.dumps(value), text)
        if n != 1:
            raise ValueError(f'template entry {key}: found {n}')
    # Output body is the IMU frame; T_BS is therefore identity.
    return text


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--width', type=int, default=640)
    p.add_argument('--projection', choices=['pinhole', 'native'], default='pinhole',
                   help='pinhole reprojects the >180 degree source to a valid forward view')
    p.add_argument('--fov', type=float, default=140., help='virtual pinhole horizontal FOV, degrees')
    p.add_argument('--max-frames', type=int, help='optional short validation export')
    p.add_argument('--imu-offset', type=float, help='seconds added to raw IMU timestamps; default follows ORB profile')
    p.add_argument('--acc-noise', type=float, help='override accelerometer noise density; default uses dataset metadata')
    args = p.parse_args()
    if args.width <= 0 or (args.max_frames is not None and args.max_frames <= 0):
        p.error('width and max-frames must be positive')
    if not 30 <= args.fov <= 160:
        p.error('fov must be within 30..160 degrees')
    if args.acc_noise is not None and (not np.isfinite(args.acc_noise) or args.acc_noise <= 0):
        p.error('acc-noise must be positive and finite')
    from dataset_layout import DatasetLayout
    from merge_dataset_camera_settings import imu_input_calibration
    os.environ['SLAM_H5_READER'] = 'h5py'
    layout = DatasetLayout.open(args.dataset)
    layout.validate(cameras=['cam0', 'cam1'], imu_ids=['imu0'], require_calibration=True)
    imu = layout.load_imu_sensor()
    T_SB = np.array(imu['T_BS']['data']).reshape(4, 4)
    inp = imu_input_calibration(imu, T_SB, layout.imu_sensor_path())
    if inp is None:
        raise ValueError('unresolved IMU input profile; refusing to guess axes')
    offset = args.imu_offset if args.imu_offset is not None else inp['time_offset_s']
    if not np.isfinite(offset) or abs(offset) > .2:
        p.error('IMU offset must be finite and within +/-0.2 seconds')
    cams = [layout.load_camera_sensor(k) for k in ('cam0', 'cam1')]
    calibration = layout.load_calibration()
    with h5py.File(layout.head_h5) as f:
        streams = [f['sensor/camera/'+name] for name in ('RGB_left', 'RGB_right')]
        ts = [s['timestamp_ns'][:] for s in streams]
        ix = [s['frame_index'][:] for s in streams]
        t = f['sensor/imu/imu0/timestamp_ns'][:] + round(offset * 1e9)
        gyro = f['sensor/imu/imu0/gyro_rad_s'][:] @ np.asarray(inp['gyro_rotation']).T
        acc = f['sensor/imu/imu0/accel_m_s2'][:] @ np.asarray(inp['accel_rotation']).T
    bias = inp.get('gyro_bias')
    if bias is not None and np.any(bias):
        gyro -= np.asarray(inp['gyro_rotation']) @ np.asarray(bias).reshape(3)
    # Keep unique nearest neighbours and IMU margin required by DatasetReader.
    j = np.searchsorted(ts[1], ts[0]); j = np.clip(j, 1, len(ts[1])-1)
    j = np.where(abs(ts[1][j-1]-ts[0]) < abs(ts[1][j]-ts[0]), j-1, j)
    pairs = []; used = set()
    for i, r in enumerate(j):
        stamp = min(int(ts[0][i]), int(ts[1][r]))
        if r not in used and abs(int(ts[1][r])-int(ts[0][i])) <= 1_000_000 and t[0]+25_000_000 < stamp < t[-1]-25_000_000:
            pairs.append((i, int(r))); used.add(int(r))
    if args.max_frames:
        pairs = pairs[:args.max_frames]
    if len(pairs) < 2:
        raise ValueError('no sufficient synchronized frames with IMU coverage')
    out = args.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    output_cameras = []; counts = []
    for k, sensor in enumerate(cams):
        if sensor['distortion_model'] != 'equidistant':
            raise ValueError('this adapter currently requires equidistant fisheye cameras')
        w, h = sensor['resolution']; size = (args.width, round(h * args.width / w))
        scale = np.array([size[0]/w, size[1]/h]*2)
        intrinsics = np.array(sensor['intrinsics']) * scale
        maps = None
        distortion = sensor['distortion_coefficients'][:4]
        if args.projection == 'pinhole':
            fx, fy, cx, cy = sensor['intrinsics']
            K = np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])
            focal = size[0] / (2*np.tan(np.deg2rad(args.fov)/2))
            intrinsics = np.array([focal,focal,(size[0]-1)/2,(size[1]-1)/2])
            newK = np.array([[focal,0,intrinsics[2]],[0,focal,intrinsics[3]],[0,0,1.]])
            maps = cv2.fisheye.initUndistortRectifyMap(K, np.asarray(distortion), np.eye(3), newK, size, cv2.CV_32FC1)
            distortion = [0.,0.,0.,0.]
        T_CB = np.array(sensor['T_BS']['data']).reshape(4, 4)
        # Dataset: camera <- rig; OKVIS: IMU <- camera.
        T_SC = T_SB @ np.linalg.inv(T_CB)
        output_cameras.append(dict(T_SC=T_SC.reshape(-1).tolist(), image_dimension=list(size),
            distortion_coefficients=distortion, distortion_type='radialtangential' if maps is not None else 'equidistant',
            focal_length=intrinsics[:2].tolist(), principal_point=intrinsics[2:].tolist(),
            cam_model='pinhole', camera_type='gray', mapping=False, mapping_rectification=False, slam_use='okvis'))
        folder = out/'mav0'/f'cam{k}'; (folder/'data').mkdir(parents=True)
        cap = cv2.VideoCapture(str(layout.camera_video(f'cam{k}')))
        if not cap.isOpened():
            raise ValueError('cannot open video')
        next_index = 0
        with (folder/'data.csv').open('w') as stream:
            writer = csv.writer(stream); writer.writerow(['#timestamp [ns]', 'filename'])
            for n, pair in enumerate(pairs):
                row = pair[k]; frame_index = int(ix[k][row])
                image = None
                while next_index <= frame_index:
                    ok, image = cap.read(); next_index += 1
                    if not ok:
                        raise ValueError(f'video decode failed cam{k} at index {next_index-1}')
                if image.shape[1::-1] != (w,h):
                    raise ValueError('video resolution does not match runtime calibration')
                image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                if maps is None:
                    image = cv2.resize(image, size, interpolation=cv2.INTER_AREA)
                else:
                    image = cv2.remap(image, *maps, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
                stamp = int(ts[k][row]); filename = f'{stamp}.png'
                if not cv2.imwrite(str(folder/'data'/filename), image, [cv2.IMWRITE_PNG_COMPRESSION, 1]):
                    raise OSError('image write failed')
                writer.writerow([stamp, filename])
                if n % 500 == 0:
                    print(f'cam{k}: {n}/{len(pairs)}', flush=True)
        cap.release(); counts.append(len(pairs))
    folder = out/'mav0/imu0'; folder.mkdir()
    begin = min(int(ts[k][pairs[0][k]]) for k in range(2)) - 1_000_000_000
    end = max(int(ts[k][pairs[-1][k]]) for k in range(2)) + 100_000_000
    selected = (t >= begin) & (t <= end)
    with (folder/'data.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['#timestamp [ns]', 'w_RS_S_x [rad s^-1]', 'w_RS_S_y [rad s^-1]', 'w_RS_S_z [rad s^-1]',
                         'a_RS_S_x [m s^-2]', 'a_RS_S_y [m s^-2]', 'a_RS_S_z [m s^-2]'])
        for stamp, g, a in zip(t[selected], gyro[selected], acc[selected]):
            writer.writerow([int(stamp), *g.tolist(), *a.tolist()])
    repo = Path(__file__).resolve().parents[2]
    (out/'okvis.yaml').write_text(configure(repo/'config/euroc/okvis2.yaml', output_cameras, imu, args.acc_noise))
    fs = cv2.FileStorage(str(out/'okvis.yaml'), cv2.FILE_STORAGE_READ)
    if not fs.isOpened() or fs.getNode('cameras').size() != 2 or not fs.getNode('cameras').at(0).getNode('T_SC').isSeq():
        raise ValueError('generated config failed OpenCV parsing')
    fs.release()
    with (out/'frame_mapping.csv').open('w') as stream:
        writer = csv.writer(stream); writer.writerow(['pair', 'left_frame_index', 'right_frame_index', 'left_ns', 'right_ns'])
        for n,(l,r) in enumerate(pairs): writer.writerow([n,int(ix[0][l]),int(ix[1][r]),int(ts[0][l]),int(ts[1][r])])
    report = dict(source=str(layout.root), reference=str(Path(__file__).resolve().parent), frames=counts,
        projection=args.projection, virtual_horizontal_fov_degrees=args.fov if args.projection=='pinhole' else None,
        imu_samples=int(selected.sum()), imu_offset_s=offset, imu_offset_source='CLI override' if args.imu_offset is not None else inp['source'],
        body_frame='imu0_output', extrinsic_equation='T_SC = T_imu_from_rig @ inverse(T_camera_from_rig)',
        timestamps='original CLOCK_MONOTONIC camera ns; IMU ns plus offset',
        accel_noise=imu['accelerometer_noise_density'] if args.acc_noise is None else args.acc_noise,
        stereo_max_skew_ms=max(abs(int(ts[0][l])-int(ts[1][r])) for l,r in pairs)*1e-6,
        six_face_quality=calibration['imus']['imu0'].get('six_face',{}).get('result',{}).get('quality',{}),
        first_camera_ns=min(int(ts[k][pairs[0][k]]) for k in range(2)),
        last_camera_ns=max(int(ts[k][pairs[-1][k]]) for k in range(2)))
    (out/'adapter_report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='six_face_quality'}, indent=2))


if __name__ == '__main__':
    main()
