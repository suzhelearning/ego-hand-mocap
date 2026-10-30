"""Audited AnyHand calibrated-cache adapter (no model or GT dependency).

Camera axis is [left,right], hand axis is anatomical [left,right]. Native
WiLoR MANO matrices describe the canonical right model BEFORE the left x
reflection; they must not be fed to a left MANO model without conversion.
"""

from pathlib import Path

import numpy as np

from .cameras import project, valid_pixels, validate_camera, validate_rotation
from .sync import pair_timestamps

JOINT_NAMES = ("wrist", "thumb_cmc", "thumb_mcp", "thumb_ip", "thumb_tip",
               "index_mcp", "index_pip", "index_dip", "index_tip",
               "middle_mcp", "middle_pip", "middle_dip", "middle_tip",
               "ring_mcp", "ring_pip", "ring_dip", "ring_tip",
               "little_mcp", "little_pip", "little_dip", "little_tip")
JOINT_PARENTS = (-1, 0, 1, 2, 3, 0, 5, 6, 7, 0, 9, 10, 11, 0, 13, 14, 15, 0, 17, 18, 19)
# Verified benchmark hand21_v1 is WiLoR's reordered 16 MANO joints + tips.
ANYHAND_TO_HAND21 = tuple(range(21))


def load_hot3d_observations(left_sequence, right_sequence, left_prediction_path,
                           right_prediction_path, *, max_skew_ms=5.0):
    """Return (observations, mono_mano, metadata), keeping every left frame.

    Raw files are adjacent to predictions.npz as raw/{frame_id:09d}.npz.
    Missing raw/hand data stay NaN; malformed or inconsistent provenance raises.
    Multiple raw detections of one side are rejected rather than score-selected.
    """
    sequences = (left_sequence, right_sequence)
    for key in ("sequence_id", "timestamp_clock"):
        if left_sequence[key] != right_sequence[key]:
            raise ValueError(f"Stereo {key} mismatch")
    if left_sequence["camera_id"] == right_sequence["camera_id"]:
        raise ValueError("Stereo cameras must be distinct")
    times = [np.asarray([f["timestamp_ns"] for f in s["frames"]], dtype=np.int64) for s in sequences]
    pairing = pair_timestamps(*times, max_skew_ms=max_skew_ms)
    n = len(times[0])
    shape = (n, 2, 2)
    obs = {"keypoints2d": np.full((*shape, 21, 2), np.nan),
           "confidence": np.full((*shape, 21), np.nan),
           "joint_valid": np.zeros((*shape, 21), bool), "hand_valid": np.zeros(shape, bool),
           "timestamp_ns": times[0], "timestamps_s": times[0].astype(float)/1e9,
           "frame_id": np.asarray([f["frame_id"] for f in left_sequence["frames"]], dtype=np.int64),
           "camera_timestamp_ns": np.full((n, 2), -1, np.int64),
           "camera_frame_id": np.full((n, 2), -1, np.int64),
           "skew_ms": pairing["skew_ms"], "sync_valid": pairing["valid"],
           "right_reused": pairing["right_reused"], "ambiguous_hand": np.zeros(shape, bool)}
    fields = {"mano_global_orient": (1, 3, 3), "mano_hand_pose": (15, 3, 3),
              "betas": (10,), "R_virtual_from_source": (3, 3), "K_virtual": (3, 3),
              "local_joints_virtual_m": (21, 3), "local_joints_source_m": (21, 3),
              "pred_keypoints_3d": (21, 3), "joints_camera_source": (21, 3),
              "cam_t_virtual": (3,), "cam_t_source": (3,)}
    mono = {key: np.full((*shape, *tail), np.nan) for key, tail in fields.items()}
    mono["prediction_ok"] = np.zeros(shape, bool)
    missing_raw = 0
    for c, (sequence, prediction_path) in enumerate(zip(sequences, (left_prediction_path, right_prediction_path))):
        camera = sequence["camera"]
        validate_camera(camera)
        path = Path(prediction_path)
        with np.load(path, allow_pickle=False) as saved:
            prediction = {k: saved[k] for k in saved.files}
        for key in ("sequence_id", "camera_id", "coordinate_frame", "timestamp_clock"):
            if str(prediction[key].item()) != sequence[key]:
                raise ValueError(f"Prediction {key} mismatch")
        if str(prediction["joint_set"].item()) != "hand21_v1" or tuple(prediction["joint_names"].tolist()) != JOINT_NAMES:
            raise ValueError("Unverified joint mapping")
        if str(prediction["source_model"].item()) not in ("anyhand_wilor_calibrated_raw", "wilor_calibrated_raw") or str(prediction["keypoints_source"].item()) != "reprojected_3d":
            raise ValueError("Expected AnyHand/WiLoR calibrated MANO projection cache")
        if not set(prediction["hand"].tolist()) <= {"left", "right"}:
            raise ValueError("Unknown anatomical hand labels")
        frame_ids = [f["frame_id"] for f in sequence["frames"]]
        if len(set(frame_ids)) != len(frame_ids):
            raise ValueError("Duplicate sequence frame IDs")
        for i in range(n):
            j = i if c == 0 else pairing["right_index"][i]
            if j < 0:
                continue
            frame = sequence["frames"][j]
            fid, timestamp = frame["frame_id"], frame["timestamp_ns"]
            obs["camera_timestamp_ns"][i, c] = timestamp
            obs["camera_frame_id"][i, c] = fid
            if c == 1 and not pairing["valid"][i]:
                continue
            raw_path = path.parent / "raw" / f"{fid:09d}.npz"
            if not raw_path.exists():
                missing_raw += 1
                continue
            with np.load(raw_path, allow_pickle=False) as raw:
                if int(raw["source_frame_id"]) != fid or int(raw["timestamp_ns"]) != timestamp:
                    raise ValueError("Raw frame/timestamp mismatch")
                if "is_right" not in raw:
                    continue
                if not np.isin(raw["is_right"], [0, 1]).all() or not np.isin(raw["classes"], [0, 1]).all():
                    raise ValueError("Unverified detector handedness")
                for h, hand in enumerate(("left", "right")):
                    ids = np.flatnonzero(raw["is_right"] == h)
                    rows = np.flatnonzero((prediction["frame_id"] == fid) & (prediction["hand"] == hand))
                    # Reject even detections discarded by upstream highest-score selection.
                    if len(ids) > 1 or np.sum(raw["classes"] == h) > 1 or len(rows) > 1:
                        obs["ambiguous_hand"][i, c, h] = True
                        continue
                    if len(ids) != 1 or len(rows) != 1:
                        continue
                    k, row = ids[0], rows[0]
                    if int(prediction["timestamp_ns"][row]) != timestamp:
                        raise ValueError("Prediction timestamp mismatch")
                    if not prediction["valid"][row]:
                        continue
                    detection = int(raw["predicted_detection_index"][k])
                    if detection < 0 or detection >= len(raw["scores"]) or int(raw["classes"][detection]) != h:
                        raise ValueError("Raw detection mapping mismatch")
                    score = float(raw["scores"][detection])
                    if not np.isfinite(score) or not 0 <= score <= 1:
                        raise ValueError("Invalid detection confidence")
                    rotation = validate_rotation(raw["R_virtual_from_source"][k])
                    validate_rotation(raw["mano_global_orient"][k])
                    validate_rotation(raw["mano_hand_pose"][k])
                    local = raw["pred_keypoints_3d"][k].astype(float).copy()
                    local[:, 0] *= 2*h-1
                    virtual = local + raw["cam_t_virtual"][k]
                    source = virtual @ rotation
                    if not np.allclose(virtual, raw["joints_camera_virtual"][k], atol=2e-6) or not np.allclose(source, raw["joints_camera_source"][k], atol=2e-6):
                        raise ValueError("Raw virtual/source mirror or unit mapping mismatch")
                    if not np.allclose(source, prediction["joints21_m"][row], atol=2e-6):
                        raise ValueError("Prediction/raw metric joints mismatch")
                    pixels = project(source, camera)
                    if not np.allclose(pixels, prediction["keypoints21_px"][row], atol=2e-3, equal_nan=True):
                        raise ValueError("Raw full-image projection mismatch")
                    valid = valid_pixels(pixels, camera) & prediction["projection_valid"][row] & prediction["joint_valid"][row]
                    if "input_view_joint_valid" in prediction:
                        valid &= prediction["input_view_joint_valid"][row]
                    obs["keypoints2d"][i, c, h] = np.where(valid[:, None], pixels, np.nan)
                    obs["confidence"][i, c, h] = np.where(valid, score, np.nan)
                    obs["joint_valid"][i, c, h] = valid
                    obs["hand_valid"][i, c, h] = valid.any()
                    values = {key: raw[key][k] for key in fields if key in raw}
                    values.update(betas=raw["mano_betas"][k], local_joints_virtual_m=local,
                                  local_joints_source_m=local @ rotation)
                    for key, value in values.items():
                        if np.shape(value) != fields[key] or not np.isfinite(value).all():
                            raise ValueError(f"Invalid raw MANO field {key}")
                        mono[key][i, c, h] = value
                    mono["prediction_ok"][i, c, h] = True
    metadata = {"keypoints_source": "mano_projection_calibrated_raw_full_image",
                "confidence_source": "hand_detection_score_broadcast_to_21_joints",
                "camera_order": [s["camera_id"] for s in sequences], "hand_order": ["left", "right"],
                "joint_names": list(JOINT_NAMES), "joint_parents": list(JOINT_PARENTS),
                "timestamp_reference": "left camera; signed skew = right minus left",
                "max_skew_ms": max_skew_ms, "missing_raw_frames": missing_raw,
                "ambiguous_hands": int(obs["ambiguous_hand"].sum()),
                "mono_convention": "native canonical-right MANO rotmats before left x reflection; local_joints_virtual_m already mirrored",
                "mono_translation": "metre convention, monocular estimate; NOT stereo metric depth",
                "virtual_rotation": "X_virtual = R_virtual_from_source @ X_source",
                "prediction_paths": [str(left_prediction_path), str(right_prediction_path)],
                "identity_limit": "handedness only; a single other-person hand cannot be identified reliably"}
    return obs, mono, metadata
