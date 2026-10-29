import test from 'node:test';
import assert from 'node:assert/strict';
import {nearestFrame, timelineRange} from '../../src/motion_web/static/timeline.mjs';
const data = {frames: [{time_s: 0, valid: true}, {time_s: .1, valid: false}, {time_s: .2, valid: true}]};
test('nearest frame preserves invalid sample and rejects out of range', () => {
  assert.equal(nearestFrame(data, .08), 1);
  assert.equal(nearestFrame(data, -.01), -1);
  assert.equal(nearestFrame(data, .21), -1);
  assert.equal(nearestFrame(data, .2), 2);
});
test('positive offset delays MOCAP and timeline includes both extents', () => {
  assert.equal(nearestFrame(data, .9, 1), -1);
  assert.equal(nearestFrame(data, 1.08, 1), 1);
  assert.deepEqual(timelineRange(data, data, 1), [0, 1.2]);
  assert.deepEqual(timelineRange(null, data, -1), [-1, -.8]);
  assert.deepEqual(timelineRange(null, null, 0), [0, 0]);
});
