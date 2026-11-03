"""Convert the estimator's IMU pose to the calibrated cam0 optical frame."""
from pathlib import Path
import csv
import cv2
import numpy as np


def cam0_extrinsic(dataset, result):
    config = Path(result)/'okvis_gui.yaml'
    if not config.exists():
        config = Path(dataset)/'okvis.yaml'
    storage = cv2.FileStorage(str(config), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise ValueError('Cannot read cam0 calibration: '+str(config))
    try:
        online = storage.getNode('camera_parameters').getNode('online_calibration')
        for key in ('do_extrinsics', 'do_extrinsics_final_ba'):
            flag = online.getNode(key)
            enabled = (flag.string().split('#',1)[0].strip().lower() not in ('false', 'no', 'off', '0')
                       if flag.isString() else bool(flag.real())) if not flag.empty() else False
            if enabled:
                raise ValueError('cam0 export requires fixed camera extrinsics; online calibration is enabled')
        node = storage.getNode('cameras').at(0).getNode('T_SC')
        transform = (np.array([node.at(i).real() for i in range(node.size())]).reshape(4,4)
                     if node.isSeq() else node.mat())
    finally:
        storage.release()
    if (transform is None or transform.shape != (4,4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0,0,0,1])
            or not np.allclose(transform[:3,:3].T@transform[:3,:3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(transform[:3,:3]), 1, atol=1e-6)):
        raise ValueError('Invalid cam0 T_SC in '+str(config))
    return transform


def to_cam0(rows, transform):
    """Rows: ns, xyz, quaternion xyzw, IMU velocity/bias, optional quality.

    Only pose changes. Auxiliary state values still belong to the IMU.
    Timestamp integers are preserved without a float round trip.
    """
    if not len(rows):
        return []
    pose = np.asarray([row[1:8] for row in rows], dtype=float)
    q = pose[:,3:7]; q = q/np.linalg.norm(q,axis=1)[:,None]
    offset = np.broadcast_to(transform[:3,3], (len(rows),3))
    vector, scalar = q[:,:3], q[:,3:4]
    positions = pose[:,:3] + offset + 2*np.cross(vector, np.cross(vector,offset)+scalar*offset)
    # Rodrigues handles both near-zero and pi rotations of the fixed extrinsic.
    axis, _ = cv2.Rodrigues(transform[:3,:3])
    angle = np.linalg.norm(axis)
    extrinsic = np.r_[axis.ravel()*np.sinc(angle/(2*np.pi))/2, np.cos(angle/2)]
    ev, ew = extrinsic[:3], extrinsic[3]
    orientations = np.column_stack((scalar*ev + ew*vector + np.cross(vector,ev),
                                   q[:,3]*ew - vector@ev))
    orientations /= np.linalg.norm(orientations,axis=1)[:,None]
    return [[row[0], *p, *orientation, *row[8:]]
            for row,p,orientation in zip(rows,positions,orientations)]


# Columns: camera FLU axes expressed in the original optical RDF frame.
R_OPTICAL_FLU = np.array([[0.,-1.,0.], [0.,0.,-1.], [1.,0.,0.]])


def quaternion_product(a, b):
    a, b = np.asarray(a), np.asarray(b)
    return np.concatenate((a[...,3:4]*b[...,:3]+b[...,3:4]*a[...,:3]
                           +np.cross(a[...,:3],b[...,:3]),
                           a[...,3:4]*b[...,3:4]-np.sum(a[...,:3]*b[...,:3],axis=-1,keepdims=True)),axis=-1)


def rotation_quaternion(rotation):
    axis, _ = cv2.Rodrigues(rotation)
    angle = np.linalg.norm(axis)
    return np.r_[axis.ravel()*np.sinc(angle/(2*np.pi))/2, np.cos(angle/2)]


def quaternion_rotation(q):
    q=np.asarray(q,dtype=float);q=q/np.linalg.norm(q)
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def world_flu(streams, preferred):
    """Initial optical centre origin, gravity-up Z, horizontal initial-forward X.

    Pose rotation maps the camera's local FLU axes into the new world FLU.
    One rigid world transform is shared across all streams and map points.
    """
    reference=streams.get(preferred,[])
    origin=np.asarray(reference[0][1:4],dtype=float) if reference else np.zeros(3)
    world_rotation=np.eye(3)
    if reference:
        forward=quaternion_rotation(reference[0][4:8])[:,2].copy()
        forward[2]=0.
        if np.linalg.norm(forward)<1e-6:
            raise ValueError('Initial cam0 points vertically: horizontal FLU heading is undefined')
        forward/=np.linalg.norm(forward)
        up=np.array([0.,0.,1.]);left=np.cross(up,forward)
        world_rotation=np.stack([forward,left,up])
    qw=rotation_quaternion(world_rotation);qb=rotation_quaternion(R_OPTICAL_FLU)
    converted={}
    for name,rows in streams.items():
        converted[name]=[]
        for row in rows:
            position=world_rotation@(np.asarray(row[1:4])-origin)
            orientation=quaternion_product(quaternion_product(qw,row[4:8]),qb)
            orientation/=np.linalg.norm(orientation)
            auxiliary=list(row[8:])
            if len(auxiliary)>=3:
                auxiliary[:3]=(world_rotation@np.asarray(auxiliary[:3])).tolist()
            converted[name].append([row[0],*position,*orientation,*auxiliary])
    return converted,origin,world_rotation


def export_cam0(result, streams, world_frame=None):
    """Write pose-only CSVs with exact nanosecond stamps and a final TUM."""
    result = Path(result)
    for name, rows in streams.items():
        with (result/f'trajectory_cam0_{name}.csv').open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['timestamp_ns','cam0_tx_world_flu_m','cam0_ty_world_flu_m','cam0_tz_world_flu_m',
                             'cam0_flu_qx','cam0_flu_qy','cam0_flu_qz','cam0_flu_qw'])
            for row in rows:
                writer.writerow([row[0], *[format(float(v),'.15g') for v in row[1:8]]])
    preferred = next((streams[key] for key in ('ba','final','live') if streams.get(key)), [])
    with (result/'trajectory_cam0.tum').open('w') as stream:
        stream.write('# CLOCK_MONOTONIC seconds; T_worldFLU_cam0FLU; initial cam0 origin; X forward Y left Z up; tx ty tz qx qy qz qw\n')
        for row in preferred:
            stamp = int(row[0]); seconds, nano = divmod(stamp,1_000_000_000)
            stream.write(f'{seconds}.{nano:09d} '+' '.join(format(float(v),'.15g') for v in row[1:8])+'\n')
    if world_frame is not None:
        import json
        (result/'world_frame.json').write_text(json.dumps(world_frame,indent=2)+'\n')
        source_map = result/'okvis2-slam-final_map.csv'
        if source_map.exists():
            origin = np.asarray(world_frame['origin_in_estimator_world_m'])
            rotation = np.asarray(world_frame['R_world_flu_from_estimator_world'])
            with source_map.open() as source, (result/'landmarks_cam0_world.csv').open('w') as target:
                writer = csv.writer(target)
                writer.writerow(['landmark_id','world_x_m','world_y_m','world_z_m'])
                for row in csv.reader(source):
                    if row == ['landmarks:']:
                        continue
                    if len(row) != 4:
                        break  # The remaining map sections are not landmark positions.
                    point = rotation@(np.asarray(row[1:4],dtype=float)-origin)
                    if np.isfinite(point).all():
                        writer.writerow([row[0], *[format(float(v),'.15g') for v in point]])
