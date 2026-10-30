"""Independent AnyHand-WiLoR worker using the pinned upstream predictor and shared geometry."""

import importlib.util
import os
import sys
import time
from pathlib import Path

import numpy as np

from ego_hand_benchmark.core.io import read_json, save_predictions, sha256, write_json
from ego_hand_benchmark.core.schema import empty_predictions, set_prediction
from ego_hand_benchmark.geometry.fisheye import pinhole_camera, project
from ego_hand_benchmark.models.base import PROJECT_ROOT, ModelAdapter, repository_state
from ego_hand_benchmark.models.worker_utils import (
    finish_metadata,
    load_job,
    read_frame,
    synchronize,
)

DEFAULT_CHECKPOINT = PROJECT_ROOT / "weights/anyhand/wilor/anyhand_wilor.ckpt"
DEFAULT_CONFIG = PROJECT_ROOT / "weights/anyhand/wilor/model_config_wilor.yaml"
DETECTOR = PROJECT_ROOT / "WiLoR/pretrained_models/detector.pt"
UPSTREAM = PROJECT_ROOT / "AnyHand"


def resolve_assets(config):
    return {
        "checkpoint": str(Path(config.get("checkpoint", DEFAULT_CHECKPOINT)).expanduser().resolve()),
        "model_config": str(Path(config.get("model_config", DEFAULT_CONFIG)).expanduser().resolve()),
        "detector": str(DETECTOR),
        "mano": str(UPSTREAM / "mano_data/MANO_RIGHT.pkl"),
        "mean_params": str(UPSTREAM / "mano_data/mano_mean_params.npz"),
    }


def verify_assets(config, *, require_checkpoint=True):
    sources = read_json(PROJECT_ROOT / "deployment/anyhand-sources.json")
    paths = resolve_assets(config)
    hashes = {}
    for key, value in paths.items():
        if key == "checkpoint" and not require_checkpoint:
            continue
        path = Path(value)
        if not path.is_file():
            raise FileNotFoundError(f"AnyHand {key} missing: {path}; supply --checkpoint for downloaded weights")
        name = {"checkpoint": "anyhand_wilor.ckpt", "model_config": "model_config_wilor.yaml"}.get(key)
        if name and path.stat().st_size != sources["files"][name]["bytes"]:
            raise ValueError(f"Incomplete or incorrect AnyHand asset: {path}")
        hashes[value] = sha256(path)
        if name and hashes[value] != sources["files"][name]["sha256"]:
            raise ValueError(f"Asset differs from pinned AnyHand release: {path}")
    for folder, expected in ((UPSTREAM, sources["commit"]), (UPSTREAM / "WiLoR", sources["wilor_commit"])):
        if repository_state(folder)["commit"] != expected:
            raise ValueError(f"Unexpected upstream revision: {folder}")
    return hashes


class AnyHandAdapter(ModelAdapter):
    name = "anyhand"
    source_directory = "AnyHand"
    python_environment = "anyhand"

    def assets(self, config):
        return tuple(resolve_assets(config).values())

    def source_state(self):
        return {"anyhand": repository_state(UPSTREAM), "wilor": repository_state(UPSTREAM / "WiLoR")}

    def worker_environment(self, config):
        return {"PYTHONPATH": os.pathsep.join([str(UPSTREAM / "WiLoR"), str(PROJECT_ROOT)]),
                "HF_HUB_OFFLINE": "1", "TRANSFORMERS_OFFLINE": "1",
                "PYTHONDONTWRITEBYTECODE": "1"}


def predictor_class():
    # The workspace also has a scripts namespace. Import this exact file, never scripts.*.
    name = "_anyhand_official_rgb_predictor"
    spec = importlib.util.spec_from_file_location(name, UPSTREAM / "scripts/rgb_predictor.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module.AnyHandPredictor


def selected_detections(boxes, scores, classes, seconds):
    boxes, scores, classes = np.asarray(boxes), np.asarray(scores), np.asarray(classes)
    count = len(scores)
    if boxes.shape != (count, 4) or classes.shape != (count,):
        raise ValueError("Malformed detection arrays")
    if not np.isfinite(boxes).all() or not np.isfinite(scores).all() or not np.isin(classes, [0, 1]).all():
        raise ValueError("Invalid detector geometry/scores/hand labels")
    if count and (np.any(boxes[:, 2:] <= boxes[:, :2]) or np.any((scores < 0) | (scores > 1))):
        raise ValueError("Invalid detection box or score")
    if not np.isfinite(seconds) or seconds < 0:
        raise ValueError("Invalid cached detection timing")
    chosen = [int(ids[np.argmax(scores[ids])]) for side in range(2)
              if len(ids := np.flatnonzero(classes == side))]
    return {"boxes_xyxy": boxes.astype(np.float32), "scores": scores.astype(np.float32),
            "classes": classes.astype(np.int64), "selected_detection_indices": np.array(chosen, dtype=np.int64),
            "detector_seconds": np.array(seconds)}


def validate_same_source(reference, sequence):
    for key in ("sequence_id", "camera_id", "coordinate_frame", "timestamp_clock", "source_video_sha256", "camera"):
        if reference.get(key) != sequence.get(key):
            raise ValueError(f"Detection cache input mismatch: {key}")
    frames = {f["frame_id"]: f for f in reference["frames"]}
    for frame in sequence["frames"]:
        original = frames.get(frame["frame_id"])
        if original is None or original["timestamp_ns"] != frame["timestamp_ns"]:
            raise ValueError("Detection cache frame/timestamp mismatch")


class DetectionCache:
    def __init__(self, folder, sequence, detector_sha256=None):
        self.folder = Path(folder).resolve()
        metadata_path = self.folder.parent / "metadata.json"
        meta = read_json(metadata_path)
        if meta.get("status") != "complete":
            raise ValueError("Detection cache worker must be complete")
        if detector_sha256 is not None:
            recorded = [value for key, value in meta.get("assets_sha256", {}).items()
                        if Path(key).name == "detector.pt"]
            if recorded != [detector_sha256]:
                raise ValueError("Detection cache uses a different or unrecorded detector asset")
        job_path = self.folder.parent / "job.json"
        reference_path = Path(read_json(job_path)["sequence_path"])
        if sha256(reference_path) != meta["sequence_sha256"]:
            raise ValueError("Detection cache source sequence has changed")
        validate_same_source(read_json(reference_path), sequence)
        manifest_path = self.folder.parent / "raw_manifest.json"
        self.expected = read_json(manifest_path)["sha256"] if manifest_path.is_file() else {}
        self.hashes = {str(metadata_path): sha256(metadata_path), str(reference_path): sha256(reference_path),
                       str(job_path): sha256(job_path)}
        if manifest_path.is_file():
            self.hashes[str(manifest_path)] = sha256(manifest_path)
        self.metadata = meta

    def load(self, frame):
        path = self.folder / f"{frame['frame_id']:09d}.npz"
        value = sha256(path)
        if path.name in self.expected and value != self.expected[path.name]:
            raise ValueError(f"Modified cached detection: {path}")
        if str(path) in self.hashes and value != self.hashes[str(path)]:
            raise ValueError(f"Detection changed during run: {path}")
        self.hashes[str(path)] = value
        with np.load(path, allow_pickle=False) as raw:
            detection = selected_detections(raw["boxes_xyxy"], raw["scores"], raw["classes"], float(raw["detector_seconds"]))
            if not np.array_equal(detection["selected_detection_indices"], raw["selected_detection_indices"]):
                raise ValueError("Detection cache uses a different per-side selection policy")
        return detection


def camera_joints(hand):
    # Upstream already mirrored the left-hand x axis. Translation is still separate.
    return np.asarray(hand.keypoints_3d) + np.asarray(hand.cam_t)[None, :]


def infer_native(predictor, image, detection):
    selected = detection["selected_detection_indices"]
    raw = dict(detection)
    if not len(selected):
        return [], raw, []
    hands = predictor._run_wilor(image, detection["boxes_xyxy"][selected],
                                detection["classes"][selected].astype(bool), detection["scores"][selected])
    predictions = [(int(h.is_right), camera_joints(h), h.score) for h in hands]
    fields = {"mano_pose_axis_angle": "mano_pose", "mano_shape": "mano_shape",
              "vertices_mirrored": "vertices", "keypoints_3d_mirrored": "keypoints_3d",
              "keypoints_2d_native_px": "keypoints_2d", "cam_t_full": "cam_t",
              "is_right": "is_right", "focal_length_px": "focal_length"}
    raw.update({key: np.stack([getattr(h, attr) for h in hands]) for key, attr in fields.items()})
    raw["joints_camera_source"] = np.stack([camera_joints(h) for h in hands])
    raw["vertices_camera_source"] = np.stack([h.vertices + h.cam_t[None, :] for h in hands])
    return predictions, raw, []


def write_progress(job, completed, total):
    progress = {"stage": "inference", "method": job["config"]["id"],
                "sequence_path": job["sequence_path"], "completed_frames": completed, "total_frames": total}
    write_json(Path(job["output"]) / "progress.json", progress)
    if run_path := job["config"].get("run_status_path"):
        path = Path(run_path)
        state = read_json(path)
        state["progress"] = progress
        temporary = path.with_suffix(".worker.tmp")
        write_json(temporary, state)
        temporary.replace(path)


def main():
    import torch

    from ego_hand_benchmark.models.wilor_calibrated import infer_calibrated

    job, sequence, output = load_job()
    config = job["config"]
    paths = resolve_assets(config)
    device = config.get("device", "cuda")
    calibrated = config["camera_mode"] == "calibrated_virtual"
    if config["camera_mode"] not in ("native_heuristic", "calibrated_virtual"):
        raise ValueError("Unknown AnyHand camera mode")
    cache = DetectionCache(config["detections_from"], sequence, sha256(paths["detector"])) if config.get("detections_from") else None
    start = time.perf_counter()
    predictor = predictor_class()(backend="wilor", device=device, wilor_ckpt=paths["checkpoint"],
                                  wilor_cfg=paths["model_config"], detector_pt=paths["detector"],
                                  det_conf=config["detector_confidence"], det_iou=config["detector_iou"],
                                  rescale_factor=config["rescale_factor"], batch_size=config["batch_size"])
    labels = predictor._detector.names
    if "left" not in labels[0].lower() or "right" not in labels[1].lower():
        raise ValueError(f"Unexpected hand detector labels: {labels}")
    model_cfg = predictor._wilor_model_cfg
    write_json(output / "resolved_model_config.json", dict_config(model_cfg))
    synchronize(torch, device)
    load_seconds = time.perf_counter() - start
    source_camera = sequence["camera"]
    width, height = source_camera["image_width"], source_camera["image_height"]
    focal = float(model_cfg.EXTRA.FOCAL_LENGTH / model_cfg.MODEL.IMAGE_SIZE * max(width, height))
    camera = source_camera if calibrated else pinhole_camera(width, height, focal)

    def infer(image, detection):
        if detection is None:
            synchronize(torch, device)
            tick = time.perf_counter()
            boxes, right, scores = predictor._detect_hands(image)
            synchronize(torch, device)
            detection = selected_detections(boxes, scores, right.astype(int), time.perf_counter() - tick)
        if calibrated:
            return infer_calibrated(image, detection, predictor._wilor_model, model_cfg, device,
                                    config, source_camera)
        return infer_native(predictor, image, detection)

    frames = sequence["frames"]
    warmup = min(config.get("warmup_frames", 8), len(frames))
    for frame in frames[:warmup]:
        infer(read_frame(frame), cache.load(frame) if cache else None)
    synchronize(torch, device)
    if str(device).startswith("cuda"):
        torch.cuda.reset_peak_memory_stats(device)
    result = empty_predictions(sequence, config["id"])
    (output / "raw").mkdir()
    samples, raw_hashes = [], {}
    write_progress(job, 0, len(frames))
    for position, frame in enumerate(frames):
        image = read_frame(frame)
        if image.shape[:2] != (height, width):
            raise ValueError("Source image dimensions differ from calibration")
        detection = cache.load(frame) if cache else None
        synchronize(torch, device)
        tick = time.perf_counter()
        predictions, raw, _ = infer(image, detection)
        synchronize(torch, device)
        measured = time.perf_counter() - tick
        reused = float(raw["detector_seconds"]) if cache else 0.0
        samples.append({"frames": 1, "seconds": measured + reused, "measured_worker_seconds": measured,
                        "reused_detector_seconds": reused, "view_generation_seconds": float(raw.get("view_generation_seconds", 0))})
        for side, joints, score in predictions:
            set_prediction(result, position * 2 + side, joints, project(joints, camera),
                           detection_confidence=score, allow_negative_z=calibrated and source_camera["model"] in ("KB4", "FISHEYE624"))
        if calibrated:
            for side in range(2):
                if not result["valid"][position * 2 + side]:
                    result["invalid_reason"][position * 2 + side] = raw["failure_reason_per_side"][side] or "invalid_geometry"
        raw["source_frame_id"] = np.array(frame["frame_id"])
        raw["timestamp_ns"] = np.array(frame["timestamp_ns"])
        path = output / f"raw/{frame['frame_id']:09d}.npz"
        np.savez_compressed(path, **raw)
        raw_hashes[path.name] = sha256(path)
        if (position + 1) % 25 == 0 or position + 1 == len(frames):
            write_progress(job, position + 1, len(frames))
            print(f"{config['id']}: {position + 1}/{len(frames)}", flush=True)
    from ego_hand_benchmark.models.hot3d_monocular import add_input_view_flags

    add_input_view_flags(result, camera)
    save_predictions(output / "predictions.npz", result)
    write_json(output / "raw_manifest.json", {"schema": "anyhand.raw.v1", "sha256": raw_hashes})
    if cache:
        for path, expected in cache.hashes.items():
            if sha256(path) != expected:
                raise ValueError(f"Detection source changed during inference: {path}")
        write_json(output / "detection_source_hashes.json", cache.hashes)
    finish_metadata(output, torch, config, load_seconds, samples, {
        "algorithm": "AnyHand", "backend": "wilor", "input_modalities": "monocular_rgb",
        "projection_camera": camera, "source_calibration_used": calibrated,
        "input_view_flag_note": "input_view_joint_valid means finite projection inside source pixel bounds; not occlusion or GT visibility",
        "camera_recovery": "calibrated_local_pinhole_to_source" if calibrated else "native_heuristic_on_source_pixels",
        "input_distortion_corrected": calibrated,
        "scale_source": "monocular_MANO_prior_and_calibrated_virtual_focal" if calibrated else "monocular_MANO_prior_and_assumed_focal",
        "camera_note": f"AnyHand trained focal={model_cfg.EXTRA.FOCAL_LENGTH:g}; " + (
            "Calibrated local pinhole views; recover with actual virtual K, rotate back to source optical frame."
            if calibrated else f"Native full-image focal={focal:g}px; source distortion ignored."),
        "effective_warmup_frames": warmup,
        "selection_policy": "Highest detector score per anatomical side; single wearer",
        "confidence_note": "Detector scores only; joint confidence unavailable (NaN).",
        "keypoints_note": "Standard 2D is projected from translated 3D; upstream crop-derived 2D retained only in raw.",
        "detections_from": str(cache.folder) if cache else None,
        "detector_source_config": cache.metadata.get("method_config") if cache else config,
        "timing_note": "Measured worker time plus original per-frame detector cost; cache I/O excluded." if cache else "Synchronized native detection and pose.",
        "timing_scope": "Composite: measured preprocessing/pose plus cached measured detector time; excludes cache I/O" if cache else None,
        "view_generation_total_seconds": sum(s["view_generation_seconds"] for s in samples),
        "measured_worker_total_seconds": sum(s["measured_worker_seconds"] for s in samples),
        "reused_detector_total_seconds": sum(s["reused_detector_seconds"] for s in samples),
    })


def dict_config(config):
    import yaml

    return yaml.safe_load(config.dump())


if __name__ == "__main__":
    main()
