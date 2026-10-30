"""MANO with the audited upstream WiLoR mirrored-right convention.

No MANO_LEFT asset is claimed or needed. Finger rotations remain canonical
right-model rotations; reflect the *posed* local geometry, never just a root
axis-angle component. Physical left root is S @ canonical_root @ S.
"""
from pathlib import Path

import numpy as np
import torch
from torch import nn


DEFAULT_MANO_PATH = Path(__file__).resolve().parents[3] / "WiLoR/mano_data/MANO_RIGHT.pkl"
# WiLoR/wilor/models/mano_wrapper.py and smplx.vertex_ids['mano'].
TIP_IDS = (744, 320, 443, 554, 671)
MANO_TO_OPENPOSE = (0, 13, 14, 15, 16, 1, 2, 3, 17, 4, 5, 6, 18, 10, 11, 12, 19, 7, 8, 9, 20)


def rotation_6d(matrix):
    return matrix[..., :, :2].transpose(-1, -2).reshape(*matrix.shape[:-2], 6)


def rotation_matrix(value):
    """Continuous 6D Gram-Schmidt rotation, first two columns encoded rowwise."""
    a, b = value[..., :3], value[..., 3:]
    x = torch.nn.functional.normalize(a, dim=-1, eps=1e-8)
    y = torch.nn.functional.normalize(b - (x * b).sum(-1, keepdim=True) * x, dim=-1, eps=1e-8)
    return torch.stack((x, y, torch.cross(x, y, dim=-1)), -1)


class MirroredMANO(nn.Module):
    def __init__(self, path=None):
        super().__init__()
        import smplx
        from smplx.vertex_ids import vertex_ids

        path = Path(path or DEFAULT_MANO_PATH)
        if not path.is_file():
            raise FileNotFoundError(f"Licensed MANO_RIGHT asset required: {path}")
        if tuple(vertex_ids['mano'].values()) != TIP_IDS:
            raise ValueError("smplx fingertip convention differs from audited WiLoR")
        self.layer = smplx.MANOLayer(str(path), is_rhand=True, use_pca=False,
                                    flat_hand_mean=True)
        self.layer.requires_grad_(False)
        self.asset_path = str(path.resolve())

    def canonical(self, finger_pose, betas, root_rotation=None):
        """Raw upstream WiLoR output, including nonzero regressed wrist."""
        n = finger_pose.shape[0]
        if root_rotation is None:
            root_rotation = torch.eye(3, device=finger_pose.device, dtype=finger_pose.dtype).expand(n, 3, 3)
        out = self.layer(betas=betas, global_orient=root_rotation[:, None], hand_pose=finger_pose)
        joints = torch.cat((out.joints[:, :16], out.vertices[:, TIP_IDS]), 1)
        return joints[:, MANO_TO_OPENPOSE], out.vertices

    def forward(self, finger_pose, betas, root_rotation, wrist_t_m, is_right):
        joints, vertices = self.canonical(finger_pose, betas)
        wrist = joints[:, :1]
        sign = torch.ones((len(joints), 1, 3), device=joints.device, dtype=joints.dtype)
        sign[:, 0, 0] = torch.as_tensor(is_right, device=joints.device, dtype=joints.dtype) * 2 - 1
        joints = ((joints - wrist) * sign) @ root_rotation.transpose(-1, -2) + wrist_t_m[:, None]
        vertices = ((vertices - wrist) * sign) @ root_rotation.transpose(-1, -2) + wrist_t_m[:, None]
        return joints, vertices


def raw_to_source(model, record):
    """Reconstruct one cached WiLoR record for numerical convention auditing.

    cam_t is used ONLY here to compare the historical monocular output,
    never as metric fitting initialization.
    """
    dtype, device = model.layer.v_template.dtype, model.layer.v_template.device
    def tensor(key):
        return torch.as_tensor(np.asarray(record[key]), dtype=dtype, device=device)
    pose = tensor('mano_hand_pose').reshape(1, 15, 3, 3)
    root = tensor('mano_global_orient').reshape(1, 3, 3)
    beta = tensor('mano_betas').reshape(1, 10)
    joints, vertices = model.canonical(pose, beta, root)
    sign = joints.new_tensor([2 * int(record['is_right']) - 1, 1, 1])
    rotation = tensor('R_virtual_from_source')
    translation = tensor('cam_t_virtual')
    return joints[0], (joints[0] * sign + translation) @ rotation, (vertices[0] * sign + translation) @ rotation
