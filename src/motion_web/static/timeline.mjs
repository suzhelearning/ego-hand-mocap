// Positive offset moves MOCAP later on the shared preview timeline.
export function nearestFrame(data, time, offset = 0) {
  if (!data?.frames.length) return -1;
  const frames = data.frames, t = time - offset;
  if (t < frames[0].time_s - 1e-9 || t > frames.at(-1).time_s + 1e-9) return -1;
  let low = 0, high = frames.length - 1;
  while (low < high) {
    const mid = Math.floor((low + high) / 2);
    if (frames[mid].time_s < t) low = mid + 1;
    else high = mid;
  }
  return low > 0 && t - frames[low - 1].time_s <= frames[low].time_s - t ? low - 1 : low;
}
export function timelineRange(ego, mocap, offset) {
  const ranges = [];
  if (ego) ranges.push([ego.frames[0].time_s, ego.frames.at(-1).time_s]);
  if (mocap) ranges.push([offset + mocap.frames[0].time_s, offset + mocap.frames.at(-1).time_s]);
  return ranges.length ? [Math.min(...ranges.map(r => r[0])), Math.max(...ranges.map(r => r[1]))] : [0, 0];
}
