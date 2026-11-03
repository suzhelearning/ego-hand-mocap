"""Rebuild display references exclusively from canonical H5 datasets."""
import json
import re
import numpy as np
import h5py
from joint_capture.h5store import resolve_version


def load_reference(session, slam_run, mano_run=None, registration_run=None, body_run=None, device_config=None, skip_mocap_hand=False):
    with h5py.File(session/'ego.h5') as f:
        slam=resolve_version(f,'okvis',slam_run)
        yaml=slam['metadata/okvis.yaml'][()]
        if isinstance(yaml,bytes):yaml=yaml.decode()
        matrices=[json.loads(x) for x in re.findall(r'T_SC:\s*(\[[^]]+\])',yaml)]
        if len(matrices)!=2:raise ValueError('需要 H5 内保存的双目 OKVIS T_SC')
    with h5py.File(session/'aligned.h5') as f:
        reg=resolve_version(f,'slam_registration',registration_run)
        report=json.loads(reg.attrs['report_json'])
        calibration={'T_IB':reg['T_IB'][:], 'T_IC':reg['T_IC'][:], 'offset_s':float(report['offset_s']), 'source':reg.name,'status':report.get('status','prior_candidate')}
        body=resolve_version(f,'world_preview',body_run)
        poses=body['T_WB'][:];rms=body['marker_fit_rms_m'][:];body_source=body.name
        bridge=body['T_B_ego_rigidbody'][:] if 'T_B_ego_rigidbody' in body else None
    with h5py.File(session/'mocap.h5') as f:
        mano=None if skip_mocap_hand else resolve_version(f,'mano',mano_run)
        if mano is not None and (mano.attrs.get('units')!='m' or mano.attrs.get('coordinate_frame')!='mocap_world'):
            raise ValueError('MANO 参考必须是 m / mocap_world')
        vertices=mano['mesh/vertices'][:] if mano is not None else np.zeros((len(poses),778,3));faces=mano['mesh/faces'][:] if mano is not None else np.empty((0,3),dtype=np.uint32);mano_source=mano.name if mano is not None else 'skipped: no mocap hand markers'
        times=f['raw/frames/time_s'][:]
        labels=f['raw/markers/labels'].asstr()[:].tolist()
        template=(device_config or {}).get('native_marker_template',{})
        use_current=bool(template) and all(n in labels for n in template['names'])
        names=template['names'] if use_current else [f'ego:Marker_{i:03d}' for i in range(1,8)]
        indices=[labels.index(n) for n in names]
        markers=f['raw/markers/positions'][:][:,indices];valid=f['raw/markers/valid'][:][:,indices].all(axis=1)
        if 'raw/rigid_bodies/ego/T_WB' in f:
            poses=f['raw/rigid_bodies/ego/T_WB'][:];valid=f['raw/rigid_bodies/ego/tracking_valid'][:]
            bridge=np.eye(4);body_source='mocap.h5::raw/rigid_bodies/ego (recorded NatNet Take playback)'
        elif use_current:
            c=np.array(device_config['stream_local_from_native_marker'])
            local=np.array(template['positions_m'])@c[:3,:3].T+c[:3,3]
            poses,rms,valid=fit_device_markers(local,markers,valid)
            bridge=np.eye(4);body_source='mocap.h5::raw/markers + confirmed 9-marker template'
    if not (len(times)==len(vertices)==len(poses)==len(rms)==len(markers)):
        raise ValueError('动捕时间、刚体、MANO 帧数不一致')
    if not np.isfinite(times).all() or np.any(np.diff(times)<=0):raise ValueError('动捕原始时间必须严格递增')
    if not np.allclose(matrices[0],calibration['T_IC'].ravel(),atol=1e-7):raise ValueError('候选外参与当前 SLAM 左相机外参不一致')
    return dict(mocap_hand_available=not skip_mocap_hand,T_B_ego_rigidbody=bridge,calibration=calibration,poses=poses,rms=rms,times=times,vertices=vertices,faces=faces,
                markers=markers,valid=valid,marker_names=names,mano_source=mano_source,body_source=body_source,
                sensors={'T_IC_left':matrices[0],'T_IC_right':matrices[1],'units':'m','source':f'ego.h5::{slam_run}/metadata/okvis.yaml'})


def sample_reference(ref, rgb_times, offset, max_rms_mm):
    times=ref['times'];query=rgb_times+offset
    j=np.clip(np.searchsorted(times,query),0,len(times)-1);previous=np.maximum(j-1,0)
    j=np.where(abs(query-times[previous])<=abs(query-times[j]),previous,j)
    covered=(query>=times[0])&(query<=times[-1])
    device_valid=covered&ref['valid'][j]&np.isfinite(ref['poses'][j]).all((1,2))&np.isfinite(ref['rms'][j])&(ref['rms'][j]*1000<=max_rms_mm)
    hand_valid=ref.get('mocap_hand_available',True)&covered&np.isfinite(ref['vertices'][j]).all((1,2))
    return dict(indices=j,covered=covered,device_valid=device_valid,hand_valid=hand_valid,
                delta_s=times[j]-query)


def fit_device_markers(local, markers, valid):
    """Recover streamed-body pose from the explicitly selected 9-marker template."""
    poses=np.tile(np.eye(4),(len(markers),1,1));rms=np.full(len(markers),np.inf)
    valid=valid.copy() & np.isfinite(markers).all(axis=(1,2))
    center=local.mean(0);q=local-center
    if np.linalg.matrix_rank(q)<2:raise ValueError('Degenerate marker template')
    for i in np.flatnonzero(valid):
        obs=markers[i];u,_,vt=np.linalg.svd(q.T@(obs-obs.mean(0)))
        r=vt.T@np.diag([1,1,np.linalg.det(vt.T@u.T)])@u.T
        t=obs.mean(0)-r@center;poses[i,:3,:3]=r;poses[i,:3,3]=t
        rms[i]=np.sqrt(np.mean(np.sum((local@r.T+t-obs)**2,axis=1)))
    return poses,rms,valid
