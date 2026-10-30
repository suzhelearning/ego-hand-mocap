import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {createMeshGeometry} from '../../src/motion_web/static/mesh.mjs';

const vendor = await readFile(new URL('../../src/motion_web/static/vendor/three.module.min.js', import.meta.url), 'utf8');
const THREE = await import('data:text/javascript;base64,' + Buffer.from(vendor).toString('base64'));

test('a hidden first mesh frame keeps capacity for the next valid frame', () => {
  const geometry = createMeshGeometry(THREE, {frames: [{valid: false, points: [null, null, null]}], faces: [[0, 1, 2]]});
  const positions = geometry.attributes.position;
  assert.equal(positions.count, 3);
  [[0, 0, 0], [1, 0, 0], [0, 1, 0]].forEach((point, i) => positions.setXYZ(i, ...point));
  geometry.computeVertexNormals();
  assert.deepEqual(Array.from(positions.array), [0, 0, 0, 1, 0, 0, 0, 1, 0]);
  assert.deepEqual(Array.from(geometry.attributes.normal.array), [0, 0, 1, 0, 0, 1, 0, 0, 1]);
  geometry.dispose();
});
