"""Anchor SLAM once to a time-corresponding mocap pose; export read-only preview.
Existing mounting/time candidates are explicit inputs, not independently calibrated.
"""
import argparse,json,sys,hashlib
from .pose_contract import source_transforms
from .device import prepare_device, device_inputs
from .quality import sample_poses, choose_anchor
from .inspection import require_ready
from .reference import load_reference, sample_reference
from joint_capture.h5store import write_version
from joint_capture.paths import ProjectPaths
import shutil
import tempfile
import os
from pathlib import Path
import numpy as np,h5py
from scipy.spatial.transform import Rotation,Slerp

def export_main(argv=None):
 p=argparse.ArgumentParser();p.add_argument('--session',required=True)
 p.add_argument('--slam-run');p.add_argument('--vision-run');p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[2])
 p.add_argument('--mano-run');p.add_argument('--registration-run');p.add_argument('--body-run');p.add_argument('--output-dir',type=Path)
 p.add_argument('--max-gap-s',type=float,default=.1);p.add_argument('--max-initial-delay-s',type=float,default=2.)
 p.add_argument('--max-marker-rms-mm',type=float,default=3.);p.add_argument('--mocap-rate-hz',type=float,default=120.,help='兼容旧命令；当前使用 H5 原始 time_s')
 p.add_argument('--allow-unregistered-preview',action='store_true',help='SLAM全程无效时仅导出动捕参考和算法失败状态，不生成世界配准')
 p.add_argument('--skip-mocap-hand',action='store_true',help='无动捕手标记：保留视觉手和设备，仅跳过动捕手参考')
 a=p.parse_args(argv)
 for name in ['max_gap_s','max_initial_delay_s','max_marker_rms_mm','mocap_rate_hz']:
  if not np.isfinite(getattr(a,name)) or getattr(a,name)<=0:p.error(name+' must be positive and finite')
 root=a.root.resolve();s=ProjectPaths(root).session(a.session);w=root/'reports'/('session_'+a.session)/'world3d'
 w=a.output_dir.resolve() if a.output_dir else w
 selected=require_ready(root,a.session,a.slam_run,a.vision_run,a.skip_mocap_hand);a.slam_run=selected['slam'];a.vision_run=selected['vision']
 device_config,_,_=device_inputs(root)
 reference=load_reference(s,a.slam_run,a.mano_run,a.registration_run,a.body_run,device_config,a.skip_mocap_hand)
 old=reference['calibration'];ib=old['T_IB'];ic=old['T_IC'];dt=old['offset_s']
 with h5py.File(s/'ego.h5') as f:
  slam_group=f[f'processing/okvis/{a.slam_run}'];pose_frame=slam_group.attrs.get('body_frame','unknown');pose_world=slam_group.attrs.get('world_frame','unknown')
  g=slam_group['pose'];ts=g['timestamp_ns'][:];good=g['tracking_good'][:];rgb=f['sensor/camera/RGB_left/timestamp_ns'][:]
  li=np.tile(np.eye(4),(len(ts),1,1));li[:,:3,:3]=Rotation.from_quat(g['quaternion_xyzw'][:]).as_matrix();li[:,:3,3]=g['position'][:]
  vg=f[f'processing/vision/{a.vision_run}/arrays'];hands=vg['vertices_m'][:];hand_valid=vg['valid'][:]&np.isfinite(hands).all(axis=(2,3));v=hands.reshape(len(hands),-1,3);vv=hand_valid.any(axis=1)
  per_hand_vertices=hands.shape[2];visual_faces=vg['faces'][:];visual_faces=np.concatenate([visual_faces+i*per_hand_vertices for i in range(hands.shape[1])])
 if reference['vertices'].shape[1]<v.shape[1]:reference['vertices']=np.pad(reference['vertices'],((0,0),(0,v.shape[1]-reference['vertices'].shape[1]),(0,0)))
 wb=reference['poses'];rms=reference['rms']
 if len(rgb)!=len(v) or len(vv)!=len(rgb):raise ValueError('RGB 与视觉结果帧数不一致')
 if not np.isfinite(dt):raise ValueError('时间偏移无效')
 st=(ts-rgb[0])/1e9;rt=(rgb-rgb[0])/1e9;mt=reference['times']
 if np.any(np.diff(rgb)<=0):raise ValueError('RGB timestamps must be strictly increasing')
 mocap_good=reference['valid']&np.isfinite(rms)&(rms<=a.max_marker_rms_mm/1000)
 refs,ref_valid=sample_poses(mt,wb,mocap_good,st+dt,a.max_gap_s)
 pose_from_body,pose_from_camera=source_transforms(pose_frame,ib,ic)
 registered=bool(np.any(good))
 if not registered and a.allow_unregistered_preview:
  init=None;wl=np.eye(4)
 else:
  init=choose_anchor(st,good,ref_valid,a.max_initial_delay_s);ref=refs[init]
  wl=ref@np.linalg.inv(li[init]@pose_from_body)
  assert np.allclose(wl@li[init]@pose_from_body,ref,atol=1e-9)
  registered=True
 lip,valid=sample_poses(st,li,good,rt,a.max_gap_s)
 valid &= (rt>=st[init]) if registered else False
 body=wl@lip@pose_from_body;cam=wl@lip@pose_from_camera
 verts=np.einsum('fij,fvj->fvi',cam[:,:3,:3],v)+cam[:,None,:3,3];vis=valid&vv;verts[~vis]=0
 verts.reshape(hands.shape)[~hand_valid]=0
 pairing=sample_reference(reference,rt,dt,a.max_marker_rms_mm);idx=pairing['indices']
 meta={'frames':len(rgb),'vertices':v.shape[1],'times':rt.tolist(),'valid':vis.tolist(),'offset_s':dt}
 report={'method':'single_first_good_pose_anchor' if registered else 'unregistered_invalid_slam','initial_pose_index':init,'initial_ego_time_s':float(st[init]) if registered else None,'initial_mocap_time_s':float(st[init]+dt) if registered else None,'initial_timestamp_ns':int(ts[init]) if registered else None,'T_WL':wl.tolist() if registered else None,'T_IB':ib.tolist(),'T_IC':ic.tolist(),'offset_s':dt,'valid':valid.tolist(),'vision_valid':vis.tolist(),'calibration_status':'mounting_and_time_reused_from_previous_full_sequence_fit','independent_evaluation':False,'note':'Only T_WL is initialized from one matched pose. Mounting/time remain prior empirical candidates; no subsequent mocap correction.'}
 # Fingerprint consumed arrays/configuration rather than hashing unrelated H5 contents.
 digest=hashlib.sha256()
 for array in [ts,good,rgb,li,v,vv,wb,rms,mt,reference['markers'],reference['valid'],reference['vertices'],reference['faces']]:
  array=np.ascontiguousarray(array);digest.update(str((array.shape,str(array.dtype))).encode());digest.update(array.tobytes())
 digest.update(json.dumps({'T_IB':ib.tolist(),'T_IC':ic.tolist(),'offset_s':dt},sort_keys=True).encode())
 report['schema_version']=3
 report['registration_available']=registered
 report['algorithm_status']={'slam_good_poses':int(good.sum()),'slam_output_poses':len(good),'vision_valid_by_hand':[]}
 with h5py.File(s/'ego.h5') as source:report['algorithm_status']['vision_valid_by_hand']=source[f'processing/vision/{a.vision_run}/arrays/valid'][:].sum(axis=0).tolist()
 report['mocap_hand_available']=not a.skip_mocap_hand
 report['calibration_status']=old.get('status','candidate')
 report['note']='Fixed first-good-pose anchor; no later mocap correction. Mounting and time are explicitly unvalidated candidates.'
 report['slam_pose_frame']=str(pose_frame);report['slam_world_frame']=str(pose_world)
 report['T_pose_B']=pose_from_body.tolist();report['T_pose_C_optical']=pose_from_camera.tolist()
 report['provenance']={'session':a.session,'slam_run':a.slam_run,'vision_run':a.vision_run,'consumed_inputs_sha256':digest.hexdigest(),'exporter_sha256':hashlib.sha256(Path(__file__).read_bytes()+Path(__file__).with_name('quality.py').read_bytes()+Path(__file__).with_name('pose_contract.py').read_bytes()).hexdigest(),'calibration_source':'aligned.h5::'+old['source'],'mano_source':reference['mano_source'],'body_source':reference['body_source']}
 report['quality_settings']={'max_gap_s':a.max_gap_s,'max_initial_delay_s':a.max_initial_delay_s,'max_marker_rms_mm':a.max_marker_rms_mm,'mocap_rate_hz':a.mocap_rate_hz,'mocap_time_source':'mocap.h5::raw/frames/time_s (relative; physical anchor remains candidate)'}
 report['coverage']={'rgb_frames':len(rgb),'slam_valid_frames':int(valid.sum()),'vision_input_valid_frames':int(vv.sum()),'world_hand_valid_frames':int(vis.sum()),'vision_valid_by_hand':hand_valid.sum(axis=0).tolist(),'slam_valid_fraction':float(valid.mean()),'world_hand_valid_fraction':float(vis.mean())}
 report['provenance']['web_template_sha256']=hashlib.sha256(Path(__file__).with_name('static').joinpath('index.html').read_bytes()+Path(__file__).with_name('static').joinpath('sensor_frames.js').read_bytes()).hexdigest()
 report['pending_validation']=['IMU/body mounting','camera/IMU calibration applicability','physical clock synchronization','mocap rigid template version','hand joint semantics']
 # Prepare all outputs first. Per-file atomic replacement prevents truncated files;
 # this is not a multi-file transaction (export with playback stopped).
 w.mkdir(parents=True,exist_ok=True)
 static=Path(__file__).with_name('static')
 with tempfile.TemporaryDirectory(prefix='.comparison-',dir=w) as temporary:
  staged=Path(temporary)
  device=prepare_device(root,reference,staged)
  report['device']=device
  if device['available']:
   base_slam=body@np.array(device['T_B_base']);base_mocap=reference['poses'][idx]@np.array(device['T_B_base'])
   base_slam.astype('<f4').tofile(staged/'slam_ego_base.bin');base_mocap.astype('<f4').tofile(staged/'mocap_ego_base.bin')
  reference['vertices'][idx].astype('<f4').tofile(staged/'slam_mocap.bin')
  reference['poses'][idx].astype('<f4').tofile(staged/'slam_ego_pose.bin')
  reference['markers'][idx].astype('<f4').tofile(staged/'slam_ego_markers.bin')
  reference['faces'].astype('<u4').tofile(staged/'faces.bin')
  visual_faces.astype('<u4').tofile(staged/'vision_faces.bin')
  (staged/'meta.json').write_text(json.dumps(meta))
  (staged/'sensor_frames.json').write_text(json.dumps(reference['sensors']))
  body_meta={'frames':len(rgb),'marker_names':reference['marker_names'],'valid':pairing['device_valid'].tolist(),'hand_valid':pairing['hand_valid'].tolist(),'fit_rms_mm':np.nan_to_num(rms[idx]*1000,nan=-1,posinf=-1).tolist(),'source':'three canonical H5 files'}
  (staged/'slam_ego_body.json').write_text(json.dumps(body_meta))
  body.astype('<f4').tofile(staged/'initial_body.bin');verts.astype('<f4').tofile(staged/'initial_vision.bin')
  (staged/'initialization.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
  (staged/'index.html').write_text((static/'index.html').read_text().replace('__SESSION_ID__',a.session))
  shutil.copyfile(static/'sensor_frames.js',staged/'sensor_frames.js')
  shutil.copyfile(static/'device.js',staged/'device.js')
  for name in ['three.module.min.js','OrbitControls.js','THREE-LICENSE.txt']:
   shutil.copyfile(root/'src/motion_web/static/vendor'/name,staged/name)
  for asset in staged.iterdir():os.replace(asset,w/asset.name)
 alignment_version='initial-'+hashlib.sha256(json.dumps(report,sort_keys=True).encode()).hexdigest()[:16]
 with h5py.File(s/'aligned.h5') as source:exists=f'processing/initialization/{alignment_version}' in source
 if not exists:
  def save(g):
   for name,data in {'T_WB_slam':body,'valid':valid,'vision_valid':vis,'rgb_timestamp_ns':rgb,'mocap_index':idx,'pair_delta_s':pairing['delta_s'],'mocap_covered':pairing['covered']}.items():g.create_dataset(name,data=data)
   if registered:g.create_dataset('T_WL',data=wl)
   g.attrs['registration_available']=registered
   g.create_dataset('T_ego_rigidbody_base',data=device['T_ego_rigidbody_base'])
   g.attrs['device_pose_available']=device['available']
   if device['available']:
    g.create_dataset('T_B_base',data=device['T_B_base']);g.create_dataset('T_Wbase_slam',data=base_slam);g.create_dataset('T_Wbase_mocap',data=base_mocap)
   g.attrs['report_json']=json.dumps(report);g.attrs['physical_sync_validated']=False
  write_version(s/'aligned.h5','initialization',alignment_version,save,metadata=report)
 print(json.dumps({k:report[k] for k in ['method','initial_pose_index','initial_ego_time_s','initial_mocap_time_s','independent_evaluation']},ensure_ascii=False))
