"""Explicit source pose conventions. T_AB maps coordinates from B into A."""
import numpy as np

R_OPTICAL_FLU = np.array([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])


def source_transforms(body_frame, imu_from_body, imu_from_camera):
    if body_frame == 'imu0_output':
        return np.asarray(imu_from_body), np.asarray(imu_from_camera)
    if body_frame == 'cam0_flu':
        flu_from_optical = np.eye(4)
        flu_from_optical[:3, :3] = R_OPTICAL_FLU.T
        flu_from_body = flu_from_optical @ np.linalg.inv(imu_from_camera) @ imu_from_body
        return flu_from_body, flu_from_optical
    raise ValueError(f'未知 SLAM 位姿坐标系，拒绝猜测: {body_frame}')
