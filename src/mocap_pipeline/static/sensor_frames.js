import * as THREE from 'three';
// All poses below share the same SLAM world registration; no independent offsets.
export function installSensorFrames({scene,camera,controls,bodyPoses,registration,calibration,isVisible}) {
  const matrix=a=>new THREE.Matrix4().set(...a.flat());
  const imuFromBody=matrix(registration.T_IB),bodyFromImu=imuFromBody.clone().invert();
  const imuFromLeft=matrix(calibration.T_IC_left),imuFromRight=matrix(calibration.T_IC_right);
  const displayLeft=imuFromLeft.clone();
  if(registration.slam_pose_frame==='cam0_flu')displayLeft.multiply(matrix([[0,-1,0,0],[0,0,-1,0],[1,0,0,0],[0,0,0,1]]));
  const entries=[['IMU 原点',0xffffff,new THREE.Matrix4()],[registration.slam_pose_frame==='cam0_flu'?'左光心 C0 · FLU':'左光心 C0 · RDF',0x66e6ff,displayLeft],['右相机光心 C1',0xffdf75,imuFromRight],['EGO 刚体 B · SLAM',0xee79ff,imuFromBody]];
  const axisLength=.035;
  function label(text,color,size){
    const c=document.createElement('canvas');c.width=text.length===1?80:512;c.height=80;
    const x=c.getContext('2d');x.font='bold 32px sans-serif';x.textAlign='center';
    x.fillStyle='rgba(10,18,28,.85)';x.fillRect(0,0,c.width,80);x.fillStyle=color;x.fillText(text,c.width/2,52);
    const s=new THREE.Sprite(new THREE.SpriteMaterial({map:new THREE.CanvasTexture(c),depthTest:false}));s.scale.set(size,size*80/c.width,1);s.renderOrder=10;return s;
  }
  const groups=entries.map(([name,color])=>{
    const g=new THREE.Group();g.matrixAutoUpdate=false;g.add(new THREE.AxesHelper(axisLength));
    g.add(new THREE.Mesh(new THREE.SphereGeometry(.0025,12,8),new THREE.MeshBasicMaterial({color})));
    ['X','Y','Z'].forEach((name,k)=>{const s=label(name,['#ff6666','#77ff99','#7799ff'][k],.013);s.position.setComponent(k,axisLength+.008);g.add(s)});
    scene.add(g);return g;
  });
  const labels=entries.map(([name,color])=>{const s=label(name,'#'+new THREE.Color(color).getHexString(),.105);scene.add(s);return s});
  const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(new Float32Array(6*2*3),3));
  const links=new THREE.LineSegments(geometry,new THREE.LineBasicMaterial({color:0xc4d3e1,transparent:true,opacity:.6,depthTest:false}));links.frustumCulled=false;scene.add(links);
  let positions=[],valid=false;
  function update(frame){
    valid=isVisible()&&registration.valid[frame];groups.forEach(g=>g.visible=valid);labels.forEach(g=>g.visible=valid);links.visible=valid;if(!valid)return;
    const worldFromBody=matrix(Array.from(bodyPoses.subarray(frame*16,(frame+1)*16)));
    const worldFromImu=worldFromBody.multiply(bodyFromImu);
    positions=entries.map(([, ,local],i)=>{const t=worldFromImu.clone().multiply(local);groups[i].matrix.copy(t);groups[i].matrixWorldNeedsUpdate=true;return new THREE.Vector3().setFromMatrixPosition(t)});
    // Names are offset in world coordinates, with leaders to actual origins.
    const offsets=[[0,0,.09],[0,0,.045],[0,0,-.045],[0,0,-.09]];
    const a=geometry.attributes.position.array;let k=0;
    labels.forEach((s,i)=>{s.position.copy(positions[i]).add(new THREE.Vector3(...offsets[i]));positions[i].toArray(a,k);k+=3;s.position.toArray(a,k);k+=3});
    // Physical baseline and mounting link, drawn between actual origins.
    for(const [i,j] of [[1,2],[0,3]]){positions[i].toArray(a,k);k+=3;positions[j].toArray(a,k);k+=3}geometry.attributes.position.needsUpdate=true;
  }
  function focus(){if(!valid)return;const box=new THREE.Box3().setFromPoints([...positions,...labels.map(x=>x.position)]);box.getCenter(controls.target);const size=box.getSize(new THREE.Vector3()).length();camera.position.copy(controls.target).add(new THREE.Vector3(.6,-1,.6).normalize().multiplyScalar(Math.max(.35,size*1.8)));controls.update()}
  const distance=(a,b)=>new THREE.Vector3().setFromMatrixPosition(a).distanceTo(new THREE.Vector3().setFromMatrixPosition(b))*1000;
  return {update,focus,description:`安装距离：IMU→左光心 ${distance(new THREE.Matrix4(),imuFromLeft).toFixed(1)} mm；IMU→右光心 ${distance(new THREE.Matrix4(),imuFromRight).toFixed(1)} mm；双目基线 ${distance(imuFromLeft,imuFromRight).toFixed(1)} mm；IMU→EGO ${distance(new THREE.Matrix4(),imuFromBody).toFixed(1)} mm（候选安装参数）`};
}
