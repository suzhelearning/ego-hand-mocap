"""Canonical finger fusion; physical roots and shape are never averaged together."""
import numpy as np


def source_root(root, virtual, hand, camera, R_left_right):
    s = np.diag([-1 if hand == 0 else 1, 1, 1])
    return (R_left_right if camera else np.eye(3)) @ virtual.T @ s @ root @ s


def chordal_mean(rotations, weights):
    """Weighted SO(3) projection, independently for each canonical finger."""
    matrix = np.einsum('v,v...ij->...ij', np.asarray(weights), np.asarray(rotations))
    u, _, vt = np.linalg.svd(matrix)
    correction = np.broadcast_to(np.eye(3), matrix.shape).copy()
    correction[..., -1, -1] = np.linalg.det(u @ vt)
    return u @ correction @ vt


def select_prior(candidates, mode):
    """Candidates contain camera, pose, physical root, beta and metric score.

    Geometry scores are rigid-aligned stereo MSE in m² (no GT). Missing
    geometry falls back to left-first. Dual weights use 15 mm noise scale;
    only fingers are fused. Root and beta come from the best single view.
    """
    if mode == 'left':
        selected = candidates[0]
    else:
        selected = min(candidates, key=lambda x: x['score'])
    pose = selected['pose']
    if mode == 'dual' and len(candidates) > 1:
        scores = np.array([x['score'] for x in candidates])
        weights = np.where(np.isfinite(scores), 1 / (1 + scores / .015**2), 0.)
        if not weights.any():
            weights[:] = 1
        pose = chordal_mean([x['pose'] for x in candidates], weights / weights.sum())
    return pose, selected
