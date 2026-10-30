"""SCH adapter for the existing hp-v2 observable-only association.

Dependencies: handpose_pipeline.v2.observations.associate/Camera and its
stereo.geometry.triangulate (NumPy, SciPy, OpenCV; no model/assets/GPU).
hp-v2 uses all usable hand21 joints, positive ray depths and median maximum
two-view midpoint reprojection error, with a fixed 2 px label penalty. Its
ray-gap/angle gates affect triangulation validity, NOT association cost.
The SCH loader has already rejected duplicate same-side detections: this
adapter cannot recover those detections or establish anatomical identity
when both cameras are mislabeled. Left slots define output anatomy.
"""

from copy import deepcopy
from itertools import product

import numpy as np

from .cameras import valid_pixels, validate_transform


DEFAULT_CONFIG = dict(mode="off", min_ray_angle_deg=2.0, max_ray_gap_m=0.035,
                      max_reprojection_px=18.0, min_depth_m=0.08,
                      max_depth_m=1.5, association_min_points=6,
                      association_margin_px=3.0, reprojection_sigma_px=8.0)
OBS_SLOT_FIELDS = ("keypoints2d", "confidence", "joint_valid", "hand_valid",
                   "ambiguous_hand")
MONO_LIMIT = ("Cross-label right-slot MANO predictions are invalidated, not mirrored: "
              "canonical-right pose/beta and reflected posed geometry cannot be "
              "reinterpreted as the opposite anatomy by relabeling root rotation.")


def resolve_config(config=None):
    if config is not None and not isinstance(config, dict):
        raise ValueError("association config must be a dictionary")
    unknown = set(config or {}) - set(DEFAULT_CONFIG)
    if unknown:
        raise ValueError(f"Unknown association config keys: {sorted(unknown)}")
    cfg = {**DEFAULT_CONFIG, **(config or {})}
    if cfg["mode"] not in ("off", "diagnose", "geometric"):
        raise ValueError("association mode must be off, diagnose or geometric")
    for key, value in cfg.items():
        if key == "mode":
            continue
        if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.number)) or not np.isfinite(value) or value <= 0:
            raise ValueError(f"{key} must be finite and positive")
    points = cfg["association_min_points"]
    if int(points) != points or not 1 <= points <= 21:
        raise ValueError("association_min_points must be an integer in [1,21]")
    if cfg["min_depth_m"] >= cfg["max_depth_m"]:
        raise ValueError("min_depth_m must be below max_depth_m")
    return cfg


def associate_observations(observations, mono_mano, left_camera, right_camera,
                           T_left_right, config=None):
    """Return independent copies and diagnostics; default off is a strict no-op.

    SCH arrays are [frame,camera,anatomical-slot,...]. right_slot in diagnostics
    is the proposed original right slot (-1 unmatched), including diagnose mode.
    No GT or monocular 3D coordinates are passed to hp-v2. Geometric mode drops
    rejected right evidence, retains hp-v2 same-label right-only fallback, and
    clears cross-label mono fields with prediction_ok=False. Diagnose never
    changes observations or validity. T_left_right maps right into left metres.
    """
    cfg = resolve_config(config)
    obs, mono = deepcopy(observations), deepcopy(mono_mano)
    diagnostics = {"mode": cfg["mode"], "config": cfg,
                   "algorithm": "handpose_pipeline.v2.observations.associate",
                   "mono_limit": MONO_LIMIT,
                   "detection_limit": "Only loader-retained slots; duplicate same-side detections cannot be recovered."}
    if cfg["mode"] == "off":
        return obs, mono, diagnostics

    from handpose_pipeline.v2.observations import Camera, associate

    pixels = np.asarray(observations["keypoints2d"])
    n = len(pixels)
    if pixels.shape != (n, 2, 2, 21, 2):
        raise ValueError("Expected keypoints2d [frame,camera,hand,21,2]")
    mask = np.asarray(observations["joint_valid"], bool).copy()
    if mask.shape != (n, 2, 2, 21):
        raise ValueError("Invalid joint_valid shape")
    mask &= np.asarray(observations["hand_valid"], bool)[..., None]
    mask[:, 1] &= np.asarray(observations["sync_valid"], bool)[:, None, None]
    for c, camera in enumerate((left_camera, right_camera)):
        mask[:, c] &= valid_pixels(pixels[:, c], camera)
    data = {"pixels": np.where(mask[..., None], pixels, np.nan).swapaxes(1, 2),
            "valid": mask.any(-1).swapaxes(1, 2),
            "frame_id": np.arange(n, dtype=np.int64)}
    # hp-v2 expects X_right = T_right_left X_left, opposite to SCH.
    result, _ = associate(data, (Camera(left_camera), Camera(right_camera)),
                          np.linalg.inv(validate_transform(T_left_right)), cfg)
    slots, cost = result["right_slot"], result["association_cost_px"]
    ambiguous = np.zeros((n, 2), bool)
    # Reconstruct only diagnostic ambiguity; assignment itself is hp-v2's.
    assignments = [p for p in product((-1, 0, 1), repeat=2)
                   if p[0] < 0 or p[0] != p[1]]
    for i in range(n):
        ranked = sorted((sum(cfg["max_reprojection_px"] if b < 0 else cost[i, a, b]
                             for a, b in enumerate(p)), p) for p in assignments)
        if ranked[1][0] - ranked[0][0] < cfg["association_margin_px"]:
            ambiguous[i] = np.asarray(ranked[0][1]) != ranked[1][1]
    disagreement = (slots >= 0) & (slots != np.arange(2))
    diagnostics.update(right_slot=slots.copy(), ambiguous=ambiguous,
                       association_cost_px=cost.copy(), label_disagreement=disagreement,
                       unmatched=slots < 0, mono_invalidated=disagreement.copy(),
                       applied=cfg["mode"] == "geometric")
    if cfg["mode"] == "diagnose":
        diagnostics["mono_invalidated"] = np.zeros_like(disagreement)
        return obs, mono, diagnostics
    for i in range(n):
        for hand, other in enumerate(slots[i]):
            for key in OBS_SLOT_FIELDS:
                if key not in obs:
                    continue
                value = observations[key]
                obs[key][i, 1, hand] = value[i, 1, other] if other >= 0 else (False if value.dtype == bool else np.nan)
            if "ambiguous_hand" in obs:
                obs["ambiguous_hand"][i, 1, hand] |= ambiguous[i, hand] or disagreement[i, hand]
            for key, value in mono_mano.items():
                if not isinstance(value, np.ndarray) or value.shape[:3] != (n, 2, 2):
                    continue
                # Do not let stale physical/canonical geometry leak into fitting.
                mono[key][i, 1, hand] = value[i, 1, other] if other == hand else (False if value.dtype == bool else np.nan)
    return obs, mono, diagnostics
