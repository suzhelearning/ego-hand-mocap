import * as THREE from 'three';
export async function installDevice({scene,description,binary}){
  const groups=[new THREE.Group(),new THREE.Group()];groups.forEach(g=>{g.matrixAutoUpdate=false;g.visible=false;scene.add(g)});
  const matrix=a=>new THREE.Matrix4().set(...a.flat());
  for(const visual of description.model.visuals){
    const buffer=await binary(visual.file),d=new DataView(buffer),n=d.getUint32(80,true);
    if(buffer.byteLength!==84+n*50)throw Error('Invalid binary STL: '+visual.file);
    const p=new Float32Array(n*9);for(let i=0;i<n;i++)for(let j=0;j<9;j++)p[i*9+j]=d.getFloat32(84+i*50+12+j*4,true);
    const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(p,3));geometry.computeVertexNormals();
    groups.forEach((g,i)=>{const mesh=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color:i?0xc98be8:0xffbd65,roughness:.65,transparent:!i,opacity:i?1:.25,depthWrite:!!i}));mesh.applyMatrix4(matrix(visual.transform));mesh.scale.multiply(new THREE.Vector3(...visual.scale));g.add(mesh)});
  }
  groups.forEach(g=>{g.add(new THREE.AxesHelper(.045));g.add(new THREE.Mesh(new THREE.SphereGeometry(.002,12,8),new THREE.MeshBasicMaterial({color:0x78edbb})));});
  return {update(mocap,slam,mocapVisible,slamVisible,opacity){
    groups.forEach((g,i)=>{g.visible=description.available&&(i?slamVisible:mocapVisible);if(!g.visible)return;g.matrix.copy(matrix(i?slam:mocap)).multiply(matrix(description.T_B_base));g.matrixWorldNeedsUpdate=true;if(!i)g.traverse(o=>{if(o.isMesh){o.material.transparent=true;o.material.opacity=opacity;o.material.depthWrite=false}})});
  }};
}
