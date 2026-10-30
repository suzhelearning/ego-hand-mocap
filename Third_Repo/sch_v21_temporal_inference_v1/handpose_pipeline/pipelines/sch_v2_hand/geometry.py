"""Sparse closest-ray triangulation in the left optical camera, in metres."""

import numpy as np

from .cameras import pixel_to_unit_ray, project, valid_pixels, validate_transform


DEFAULT_CONFIG = {"min_confidence": 0.3, "min_angle_deg": 1.0,
                  "max_ray_gap_m": 0.025, "max_reproj_px": 8.0,
                  "min_depth_m": 0.05, "max_depth_m": 3.0}
REASON_BITS = {"missing": 1, "low_confidence": 2, "parallel": 4,
               "ray_gap": 8, "negative_depth": 16, "depth_range": 32,
               "reprojection": 64, "sync": 128}


def triangulate_observations(observations, left_camera, right_camera, T_left_right, config=None):
    """NaN invalid output; diagnostics remain available for rejected solutions.

    depth limits are distances *along each unit ray*, appropriate even for
    fisheye rays beyond 90 degrees (negative optical z is not negative depth).
    reason_mask is a bit field documented in REASON_BITS.
    """
    cfg = dict(DEFAULT_CONFIG)
    if config:
        unknown = set(config) - set(cfg)
        if unknown:
            raise ValueError(f"Unknown geometry settings: {sorted(unknown)}")
        cfg.update(config)
    if not all(np.isfinite(v) and v >= 0 for v in cfg.values()) or cfg["max_depth_m"] <= cfg["min_depth_m"] or cfg["min_angle_deg"] >= 90 or cfg["min_confidence"] > 1:
        raise ValueError("Invalid triangulation thresholds")
    transform = validate_transform(T_left_right)
    r, t = transform[:3, :3], transform[:3, 3]
    uv = np.asarray(observations["keypoints2d"], dtype=np.float64)
    if uv.ndim != 5 or uv.shape[1:] != (2, 2, 21, 2):
        raise ValueError("Expected keypoints2d (T,2 cameras,2 hands,21,2)")
    confidence = np.asarray(observations["confidence"])
    if confidence.shape != uv.shape[:-1]:
        raise ValueError("Confidence shape mismatch")
    available = np.isfinite(uv).all(-1)
    available &= np.asarray(observations.get("joint_valid", available), bool)
    if "hand_valid" in observations:
        available &= np.asarray(observations["hand_valid"], bool)[..., None]
    for c, camera in enumerate((left_camera, right_camera)):
        available[:, c] &= valid_pixels(uv[:, c], camera)
    a = pixel_to_unit_ray(uv[:, 0], left_camera)
    b = pixel_to_unit_ray(uv[:, 1], right_camera) @ r.T
    finite = np.isfinite(a).all(-1) & np.isfinite(b).all(-1)
    dot = np.clip(np.sum(a*b, axis=-1), -1, 1)
    angle = np.degrees(np.arccos(abs(dot)))
    denominator = 1-dot*dot
    solvable = finite & (denominator > 1e-12)
    at, bt = a @ t, b @ t
    s = np.divide(at-dot*bt, denominator, out=np.full_like(dot, np.nan), where=solvable)
    u = np.divide(dot*at-bt, denominator, out=np.full_like(dot, np.nan), where=solvable)
    p, q = s[..., None]*a, t+u[..., None]*b
    xyz = (p+q)/2
    gap = np.linalg.norm(p-q, axis=-1)
    reproj = np.stack([np.linalg.norm(project(xyz, left_camera)-uv[:, 0], axis=-1),
                       np.linalg.norm(project((xyz-t) @ r, right_camera)-uv[:, 1], axis=-1)], axis=-1)
    conditions = {
        "missing": ~available.all(axis=1) | ~finite,
        "low_confidence": (~np.isfinite(confidence) | (confidence < cfg["min_confidence"]) | (confidence > 1)).any(axis=1),
        "parallel": finite & (~solvable | (angle < cfg["min_angle_deg"])),
        "ray_gap": solvable & (gap > cfg["max_ray_gap_m"]),
        "negative_depth": solvable & ((s <= 0) | (u <= 0)),
        "depth_range": solvable & ((s < cfg["min_depth_m"]) | (u < cfg["min_depth_m"]) | (s > cfg["max_depth_m"]) | (u > cfg["max_depth_m"])),
        "reprojection": solvable & (~np.isfinite(reproj).all(-1) | (reproj > cfg["max_reproj_px"]).any(-1)),
        "sync": np.broadcast_to(~np.asarray(observations.get("sync_valid", np.ones(len(uv), bool)))[:, None, None], dot.shape),
    }
    reason = np.zeros(dot.shape, np.uint16)
    for name, mask in conditions.items():
        reason[mask] |= REASON_BITS[name]
    valid = reason == 0
    return {"joints3d_m": np.where(valid[..., None], xyz, np.nan), "valid": valid,
            "angle_deg": angle, "ray_gap_m": gap, "reproj_px": reproj,
            "ray_depth_m": np.stack([s, u], axis=-1), "reason_mask": reason,
            **{f"reason_{name}": mask for name, mask in conditions.items()}}
