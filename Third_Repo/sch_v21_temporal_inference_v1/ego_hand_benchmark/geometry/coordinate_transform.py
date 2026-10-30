import numpy as np


def transform_points(points, T_dst_src):
    """Apply a rigid source-to-destination transform; translations are meters."""
    transform = np.asarray(T_dst_src, dtype=np.float64)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("T_dst_src must be a finite 4x4 matrix")
    rotation = transform[:3, :3]
    if (
        not np.allclose(transform[3], [0, 0, 0, 1])
        or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
        or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6)
    ):
        raise ValueError("T_dst_src must be a proper rigid transform")
    return np.asarray(points) @ rotation.T + transform[:3, 3]
