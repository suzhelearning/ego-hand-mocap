"""从旧 AlignmentEngine 提取的离线插值：按图像目标时间采样，不强制60Hz。"""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def interpolate_rigid_poses(time_ns, positions, quaternion_xyzw, tracking_valid,
                            target_time_ns, *, max_gap_ns):
    """返回 (positions, quaternion_xyzw, valid)。时间须已映射到同一时钟域。

    不外推、不跨无效端点或长缺口插值；无效结果为 NaN。
    输入时间戳要求整数，先减整数原点再转浮点，避免大时间戳精度损失。
    """
    t = np.asarray(time_ns)
    target = np.asarray(target_time_ns)
    p = np.asarray(positions, dtype=float)
    q = np.asarray(quaternion_xyzw, dtype=float)
    valid = np.asarray(tracking_valid, dtype=bool)
    if t.ndim != 1 or target.ndim != 1 or t.dtype.kind not in 'iu' or target.dtype.kind not in 'iu':
        raise ValueError('时间戳必须为一维整数数组')
    n = len(t)
    if p.shape != (n, 3) or q.shape != (n, 4) or valid.shape != (n,):
        raise ValueError('位姿与有效性长度不匹配')
    if np.any(t[1:] <= t[:-1]) or max_gap_ns <= 0:
        raise ValueError('源时间必须严格递增，max_gap_ns 必须为正')
    result_p = np.full((len(target), 3), np.nan)
    result_q = np.full((len(target), 4), np.nan)
    result_valid = np.zeros(len(target), dtype=bool)
    valid = valid & np.isfinite(p).all(axis=1) & np.isfinite(q).all(axis=1) & (np.linalg.norm(q, axis=1) > 1e-12)
    for k, stamp in enumerate(target):
        j = int(np.searchsorted(t, stamp))
        if j < n and t[j] == stamp:
            if valid[j]:
                result_p[k], result_q[k] = p[j], q[j] / np.linalg.norm(q[j])
                result_valid[k] = True
            continue
        if j == 0 or j == n or not (valid[j-1] and valid[j]):
            continue
        gap = int(t[j]) - int(t[j-1])
        if gap > max_gap_ns:
            continue
        alpha = (int(stamp) - int(t[j-1])) / gap
        result_p[k] = (1-alpha)*p[j-1] + alpha*p[j]
        result_q[k] = Slerp([0., 1.], Rotation.from_quat(q[j-1:j+1]))([alpha]).as_quat()[0]
        result_valid[k] = True
    return result_p, result_q, result_valid
