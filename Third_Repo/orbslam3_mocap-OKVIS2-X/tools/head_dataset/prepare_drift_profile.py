#!/usr/bin/env python3
"""Create an isolated head export variant without modifying images or the source."""
import argparse
import csv
import json
import re
from pathlib import Path
import cv2
import h5py
import numpy as np
import yaml


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--body-mask', type=Path)
    p.add_argument('--imu-offset', type=float)
    p.add_argument('--acc-noise', type=float)
    p.add_argument('--gyro-noise', type=float)
    p.add_argument('--detection-radius', type=float)
    p.add_argument('--features', type=int)
    args=p.parse_args()
    base=args.base.resolve();out=args.output.resolve()
    report=json.loads((base/'adapter_report.json').read_text())
    text=(base/'okvis.yaml').read_text();config=yaml.safe_load(text.split('\n',1)[1])
    old_offset=report['imu_offset_s'];offset=old_offset if args.imu_offset is None else args.imu_offset
    if not np.isfinite(offset) or abs(offset)>.2:p.error('offset must be finite within +/-0.2s')
    out.mkdir(parents=True,exist_ok=False);(out/'mav0/imu0').mkdir(parents=True)
    imu=[]
    for row in list(csv.reader((base/'mav0/imu0/data.csv').open()))[1:]:
        row[0]=str(int(row[0])+round((offset-old_offset)*1e9));imu.append(row)
    with (out/'mav0/imu0/data.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(next(csv.reader((base/'mav0/imu0/data.csv').open())));writer.writerows(imu)
    mapping=list(csv.DictReader((base/'frame_mapping.csv').open()))
    kept=[row for row in mapping if int(imu[0][0])+25_000_000<min(int(row['left_ns']),int(row['right_ns'])) and max(int(row['left_ns']),int(row['right_ns']))<int(imu[-1][0])-25_000_000]
    for k,col in enumerate(['left','right']):
        folder=out/'mav0'/f'cam{k}';folder.mkdir();(folder/'data').symlink_to(base/'mav0'/f'cam{k}'/'data',target_is_directory=True)
        allowed={row[col+'_ns'] for row in kept}
        with (folder/'data.csv').open('w') as f:
            writer=csv.writer(f);writer.writerow(['#timestamp [ns]','filename'])
            writer.writerows(row for row in list(csv.reader((base/f'mav0/cam{k}/data.csv').open()))[1:] if row[0] in allowed)
    with (out/'frame_mapping.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(mapping[0]));writer.writeheader();writer.writerows(kept)
    if args.body_mask:
        polygons=json.loads(args.body_mask.read_text())
        with h5py.File(next(Path(report['source']).glob('*.h5'))) as f:cal=json.loads(f['calib/calibration_full.json'][()])
        (out/'masks').mkdir()
        for k,col in enumerate(['left','right']):
            binding=cal['sensor_bindings']['cam'+str(k)];sensor=cal['cameras'][binding['legacy_calibration_key']]
            w,h=binding['runtime_resolution'];raw=np.full((h,w),255,np.uint8)
            poly=np.round(np.asarray(polygons[col])*[w-1,h-1]).astype(np.int32)
            if poly.shape[1:]!=(2,) or len(poly)<3:raise ValueError('invalid polygon')
            cv2.fillPoly(raw,[poly],0);cam=config['cameras'][k];size=tuple(cam['image_dimension'])
            if report.get('projection')=='pinhole':
                fx,fy,cx,cy=np.asarray(sensor['intrinsics_normalized'])*[w,h,w,h]
                K=np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])
                fu,fv=cam['focal_length'];cu,cv=cam['principal_point'];newK=np.array([[fu,0,cu],[0,fv,cv],[0,0,1.]])
                maps=cv2.fisheye.initUndistortRectifyMap(K,np.asarray(sensor['distortion_coefficients'][:4]),np.eye(3),newK,size,cv2.CV_32FC1)
                mask=cv2.remap(raw,*maps,cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT)
            else:mask=cv2.resize(raw,size,interpolation=cv2.INTER_NEAREST)
            mask=cv2.erode(mask,np.ones((17,17),np.uint8))
            path=out/'masks'/f'cam{k}.png';cv2.imwrite(str(path),mask);cam['mask_file']=str(path)
            rows=list(csv.reader((out/f'mav0/cam{k}/data.csv').open()))[1:]
            image=cv2.imread(str(out/f'mav0/cam{k}/data'/rows[0][1]));overlay=image.copy();overlay[mask==0]=[30,60,210]
            cv2.imwrite(str(out/'masks'/f'cam{k}_preview.jpg'),cv2.addWeighted(image,.65,overlay,.35,0))
        camera_text='cameras:\n'+''.join('    - {'+', '.join(k+': '+json.dumps(v) for k,v in c.items())+'}\n' for c in config['cameras'])
        text=text[:text.index('cameras:')]+camera_text+text[text.index('# additional camera parameters'):]
    for key,value in [('sigma_a_c',args.acc_noise),('sigma_g_c',args.gyro_noise),
                      ('max_num_keypoints',args.features),('detection_threshold',args.detection_radius)]:
        if value is not None:
            if not np.isfinite(value) or value<=0:raise ValueError(key+' must be positive and finite')
            text,n=re.subn(r'(?m)^(\s*'+key+r':)[^\n]*',lambda m:m[1]+' '+str(value),text)
            if n!=1:raise ValueError('missing config '+key)
    (out/'okvis.yaml').write_text(text)
    report.update(base_export=str(base),frames=[len(kept)]*2,imu_offset_s=offset,
                  imu_offset_source='profile override' if args.imu_offset is not None else report['imu_offset_source'],
                  body_mask_source=str(args.body_mask.resolve()) if args.body_mask else None,
                  first_camera_ns=min(int(kept[0]['left_ns']),int(kept[0]['right_ns'])),
                  last_camera_ns=max(int(kept[-1]['left_ns']),int(kept[-1]['right_ns'])))
    if args.acc_noise is not None:report['accel_noise']=args.acc_noise
    report['profile_overrides']={key:value for key,value in [('gyro_noise',args.gyro_noise),('acc_noise',args.acc_noise),('features',args.features),('detection_radius',args.detection_radius)] if value is not None}
    (out/'adapter_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    fs=cv2.FileStorage(str(out/'okvis.yaml'),cv2.FILE_STORAGE_READ);assert fs.isOpened();fs.release()
    print(json.dumps({'output':str(out),'frames':len(kept),'offset':offset,'mask':report['body_mask_source']},indent=2))


if __name__=='__main__':main()
