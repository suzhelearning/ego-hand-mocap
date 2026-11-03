"""Explicit Motive geometry offsets and URDF (different Euler conventions)."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


def marker_basis(frame):
    """Native Motive model-description coordinates -> streamed Z-up local basis."""
    t=np.eye(4)
    if frame=='motive_y_up':
        t[:3,:3]=[[1,0,0],[0,0,-1],[0,1,0]]
    elif frame!='natnet':
        raise ValueError('Unknown geometry reference frame: '+str(frame))
    return t


def installation(gl_mm, go_pyr_deg):
    values=np.asarray([gl_mm,go_pyr_deg],dtype=float)
    if values.shape!=(2,3) or not np.isfinite(values).all():
        raise ValueError('GL / GO must each contain three finite values')
    t=np.eye(4)
    t[:3,:3]=Rotation.from_euler('XYZ',values[1],degrees=True).as_matrix()
    t[:3,3]=values[0]/1000
    return t


def world_base(position, quaternion_xyzw, body_from_base):
    t=np.eye(4)
    t[:3,:3]=Rotation.from_quat(quaternion_xyzw).as_matrix()
    t[:3,3]=position
    return t@body_from_base


def visible_pose(tracking_valid, age_s):
    return bool(tracking_valid and 0<=age_s<=.5)


def origin(element):
    t=np.eye(4)
    if element is not None:
        t[:3,3]=np.fromstring(element.get('xyz','0 0 0'),sep=' ')
        t[:3,:3]=Rotation.from_euler('xyz',np.fromstring(element.get('rpy','0 0 0'),sep=' ')).as_matrix()
    return t


def load_urdf(path):
    path=Path(path).resolve(); robot=ET.parse(path).getroot()
    children={j.find('child').get('link') for j in robot.findall('joint')}
    roots=[l.get('name') for l in robot.findall('link') if l.get('name') not in children]
    if roots!=['base']:raise ValueError('Expected base-rooted URDF')
    transforms={'base':np.eye(4)}; pending=list(robot.findall('joint'))
    while pending:
        progressed=False
        for joint in pending[:]:
            if joint.get('type')!='fixed':raise ValueError('Device URDF must be rigid')
            parent=joint.find('parent').get('link');child=joint.find('child').get('link')
            if parent not in transforms:continue
            transforms[child]=transforms[parent]@origin(joint.find('origin'))
            pending.remove(joint);progressed=True
        if not progressed:raise ValueError('Disconnected or cyclic URDF')
    visuals=[]
    for link in robot.findall('link'):
        for visual in link.findall('visual'):
            mesh=visual.find('geometry/mesh')
            if mesh is None:raise ValueError('Only mesh visuals supported')
            filename=mesh.get('filename').removeprefix('package://head_ring_description/')
            file=(path.parent/filename).resolve()
            if not file.is_relative_to(path.parent) or not file.is_file():raise ValueError('Invalid mesh path')
            color=visual.find('material/color')
            visuals.append(dict(file=filename,transform=(transforms[link.get('name')]@origin(visual.find('origin'))).tolist(),
                scale=[float(v) for v in mesh.get('scale','1 1 1').split()],
                rgba=[float(v) for v in (color.get('rgba') if color is not None else '.7 .7 .7 1').split()]))
    return dict(root='base',visuals=visuals,frames={k:v.tolist() for k,v in transforms.items()})


def configured_installation(config, world_up=None):
    geometry=installation(config['gl_xyz_mm'],config['go_pyr_deg'])
    if config.get('geometry_reference_frame')=='ego_rigidbody':
        return geometry
    if config.get('geometry_reference_frame')!='motive_native_marker':
        raise ValueError('Unknown geometry reference frame')
    def transform(key):
        value=np.asarray(config[key],dtype=float)
        if value.shape!=(4,4) or not np.isfinite(value).all():raise ValueError('Invalid '+key)
        if not np.allclose(value[3],[0,0,0,1]) or not np.allclose(value[:3,:3].T@value[:3,:3],np.eye(3)) or not np.isclose(np.linalg.det(value[:3,:3]),1):raise ValueError('Non-rigid '+key)
        return value
    # C is verified against native-description and streamed marker pairs.
    # D is verified against actual Windows OBJ and source STL vertices.
    c=transform('stream_local_from_native_marker')
    d=transform('obj_from_urdf_base')
    if (world_up or config['natnet_world_up'])=='y':c=np.eye(4)
    return c@geometry@d
