export function createMeshGeometry(THREE, data) {
  const geometry = new THREE.BufferGeometry();
  // Invalid frames contain null vertices; reserve XYZ capacity independently of values.
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(data.frames[0].points.length * 3, 3));
  geometry.setIndex(data.faces.flat());
  return geometry;
}
