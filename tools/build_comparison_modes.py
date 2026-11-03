"""Build three comparison modes from an existing world3d session preview.
Run after that session's add_slam_view.py. Never modifies source H5 data.
"""
import argparse,json,re
from pathlib import Path
import numpy as np
import h5py

def main():
 p=argparse.ArgumentParser();p.add_argument('--session',required=True);a=p.parse_args()
 root=Path(__file__).resolve().parents[1];w=root/'reports'/('session_'+a.session)/'world3d'
 j=json.loads((w/'slam.json').read_text());bm=json.loads((w/'slam_ego_body.json').read_text());n=len(j['valid'])
 wb=np.fromfile(w/'slam_ego_pose.bin','<f4').reshape(n,4,4)
 bc=np.linalg.inv(np.array(j['T_IB']))@np.array(j['T_IC'])
 with h5py.File(root/'data/sessions'/a.session/'ego.h5') as f:
  g=f['processing/vision/sch-v21-temporal-v1/arrays'];v=g['vertices_m'][:,0];valid=g['valid'][:,0]&np.array(bm['valid'])
 wc=wb@bc;world=np.einsum('fij,fvj->fvi',wc[:,:3,:3],v)+wc[:,None,:3,3];world[~valid]=0
 assert np.isfinite(world).all()
 # No framewise fit: both hand modes use the same constant T_BC.
 world.astype('<f4').tofile(w/'hand_algorithm.bin');(w/'comparison_modes.json').write_text(json.dumps({'hand_valid':valid.tolist(),'T_BC':bc.tolist(),'status':j['status'],'reference':'same nearest mocap frames and fixed extrinsic in all modes'}))
 s=(w/'index.html').read_text()
 s=s.replace('<option value="slam">SLAM 配准</option><option value="mocap">动捕位姿（原预览）</option>','<option value="device">设备对比</option><option value="hand">手部算法对比</option><option value="end">端到端对比</option>')
 s=s.replace('视觉定位 <select','对比模式 <select')
 marker='const scene=new THREE.Scene();'
 if 'const modeData=' not in s:
  s=s.replace(marker,"const modeData=await (await fetch('comparison_modes.json')).json();const handAlgorithm=new Float32Array(await (await fetch('hand_algorithm.bin')).arrayBuffer());\n"+marker)
 start=s.index('function setRoute(){');end=s.index('\nconst sensorCalibration=',start)
 # This builder's setRoute ends immediately before sensorCalibration.
 s=s[:start]+'''function setRoute(){
 const mode=$('route').value;vv=mode==='hand'?handAlgorithm:sb[1];mm=sb[2];headPoses=sb[3];headMarkers=sb[4];bodyMeta=slamBodyMeta;
 meta.valid=mode==='hand'?modeData.hand_valid:slamMeta.vision_valid;meta.offset_s=slamMeta.offset_s;
 $('m').checked=$('v').checked=mode!=='device';$('m').disabled=$('v').disabled=mode==='device';$('head').checked=true;$('slam').checked=mode!=='hand';
 $('info').style.display='block';
 $('warning').textContent=({device:'设备对比：动捕 EGO 与 SLAM EGO，统一为同一刚体原点。',hand:'手部算法对比：使用动捕设备位姿定位视觉手，尽量隔离 SLAM 轨迹误差。',end:'端到端对比：使用 SLAM 设备位姿定位视觉手，检查整套 EGO 系统。'})[mode]+' 共用时间偏移 '+slamMeta.offset_s.toFixed(4)+' s 和同一安装外参；本会话经验拟合，非独立精度验证。';update();
}
''' + s[end:]
 s=s.replace("$('route').value==='slam'?'OKVIS + 固定配准':'动捕位姿 + 候选外参'","({device:'设备对比',hand:'手部算法对比 · 动捕定位',end:'端到端对比 · SLAM定位'})[$('route').value]")
 s=s.replace("ghost.visible=$('m').checked;solid.visible=$('v').checked&&meta.valid[frame];","ghost.visible=$('route').value!=='device'&&$('m').checked&&bodyMeta.valid[frame];solid.visible=$('route').value!=='device'&&$('v').checked&&meta.valid[frame];")
 s=s.replace("$('route').onchange=()=>{setRoute();fit()}","$('route').onchange=()=>{setRoute();if($('route').value==='device')sensorFrames.focus();else fit()}")
 s=s.replace('setRoute();fit();',"setRoute();sensorFrames.focus();")
 s=s.replace("${meta.valid[frame]?'视觉实体有效':'该帧无有效视觉实体，已隐藏'}","${$('route').value==='device'?'设备对比不显示手部':(meta.valid[frame]?'视觉实体有效':'该帧无有效视觉实体，已隐藏')}")
 (w/'index.html').write_text(s)
 print('Built device / hand / end modes:',w)
if __name__=='__main__':main()
