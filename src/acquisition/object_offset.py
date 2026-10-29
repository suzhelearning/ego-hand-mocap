"""从旧物体固定外参应用逻辑适配：T_AB 将 B 系列向量变换到 A 系。"""
import numpy as np
from scipy.spatial.transform import Rotation


def pose_matrix(position, quaternion_xyzw):
    p = np.asarray(position, dtype=float)
    q = np.asarray(quaternion_xyzw, dtype=float)
    if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError('要求有限的 position(3)、非零 quaternion_xyzw(4)')
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(q).as_matrix()
    transform[:3, 3] = p
    return transform


def validate_transform(transform):
    t = np.asarray(transform, dtype=float)
    if t.shape != (4,4) or not np.isfinite(t).all() or not np.allclose(t[3], [0,0,0,1]):
        raise ValueError('要求有限的4×4齐次变换')
    r = t[:3, :3]
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(r), 1, atol=1e-6):
        raise ValueError('旋转必须属于 SO(3)')
    return t


def camera_world_pose(T_WB, T_BC):
    """已知头戴位姿和固定外参，返回 T_WC；不负责标定求解。"""
    return validate_transform(T_WB) @ validate_transform(T_BC)


def world_points_to_camera(points_world, T_WB, T_BC):
    points = np.asarray(points_world, dtype=float)
    if points.ndim < 1 or points.shape[-1] != 3:
        raise ValueError('点坐标最后一维必须为3')
    t = camera_world_pose(T_WB, T_BC)
    return (points - t[:3, 3]) @ t[:3, :3]
