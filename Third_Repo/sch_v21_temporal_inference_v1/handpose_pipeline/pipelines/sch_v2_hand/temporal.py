"""Actual-time temporal penalties; never bridge a long gap or invalid hand."""
import torch


def temporal_loss(joints, roots, fingers, timestamps, valid, max_gap_s=0.2):
    zero = joints.sum() * 0
    if len(joints) < 2:
        return zero
    dt = timestamps[1:] - timestamps[:-1]
    edges = valid[1:] & valid[:-1] & (dt[:, None] > 0) & (dt[:, None] <= max_gap_s)
    safe_dt = dt.clamp_min(1e-6)
    # Chordal SO(3) differences avoid axis-angle wrapping and acos singularities.
    angular = ((roots[1:] - roots[:-1]) / safe_dt[:, None, None, None]).square().mean((-1, -2))
    finger = ((fingers[1:] - fingers[:-1]) / safe_dt[:, None, None, None, None]).square().mean((-1, -2, -3))
    loss = (angular + finger)[edges].mean() if edges.any() else zero
    if len(joints) > 2:
        velocity = (joints[1:] - joints[:-1]) / safe_dt[:, None, None, None]
        acceleration = 2 * (velocity[1:] - velocity[:-1]) / (safe_dt[1:] + safe_dt[:-1])[:, None, None, None]
        triples = edges[1:] & edges[:-1]
        if triples.any():
            # 10 m/s² normalization; regularize all joints, including the wrist.
            loss = loss + (acceleration[triples] / 10).square().mean()
    return loss
