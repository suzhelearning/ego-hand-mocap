"""Quality-aware pose sampling. Invalid intervals never bridge tracking loss."""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def sample_poses(times, poses, good, query, max_gap):
    times = np.asarray(times, dtype=float)
    poses = np.asarray(poses, dtype=float)
    query = np.asarray(query, dtype=float)
    good = np.asarray(good, dtype=bool).copy()
    if len(times) < 2 or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError('Pose timestamps must be finite and strictly increasing')
    if max_gap <= 0 or not np.isfinite(max_gap):
        raise ValueError('max_gap must be positive and finite')
    if poses.shape != (len(times), 4, 4) or good.shape != times.shape:
        raise ValueError('Pose/validity shapes do not match timestamps')
    good &= np.isfinite(poses).all(axis=(1, 2))
    out = np.tile(np.eye(4), (len(query), 1, 1))
    valid = np.zeros(len(query), dtype=bool)
    for k, t in enumerate(query):
        if not np.isfinite(t) or t < times[0] or t > times[-1]:
            continue
        j = int(np.searchsorted(times, t))
        if j < len(times) and t == times[j]:
            if good[j]:
                out[k] = poses[j]; valid[k] = True
            continue
        if j == 0 or j == len(times) or not (good[j-1] and good[j]):
            continue
        if times[j] - times[j-1] > max_gap:
            continue
        fraction = (t - times[j-1]) / (times[j] - times[j-1])
        out[k, :3, :3] = Slerp(times[j-1:j+1], Rotation.from_matrix(poses[j-1:j+1, :3, :3]))([t]).as_matrix()[0]
        out[k, :3, 3] = (1-fraction)*poses[j-1, :3, 3] + fraction*poses[j, :3, 3]
        valid[k] = True
    return out, valid


def choose_anchor(times, slam_good, mocap_good, max_delay):
    times = np.asarray(times)
    indices = np.flatnonzero(np.asarray(slam_good, bool) & np.asarray(mocap_good, bool)
                             & (times >= 0) & (times <= max_delay))
    if not len(indices):
        raise ValueError('初始化失败：起始窗口内无同时满足 SLAM 与动捕质量条件的位姿；请检查时间对应与跟踪质量')
    return int(indices[0])
