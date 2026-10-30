"""Raw stereo video preparation and audited monocular adapter reuse (no GT required).

CSV timestamps are exposure times in a caller-declared common clock, with columns
``frame_id,timestamp_ns`` (integer nanoseconds) or ``frame_id,timestamp_s``.
FPS fallback is an explicit estimated clock, never evidence of hardware sync.
"""

import csv
import hashlib
import json
from decimal import Decimal
from pathlib import Path

import cv2
import numpy as np

from ego_hand_benchmark.core.io import load_predictions, read_json, sha256, write_json
from ego_hand_benchmark.experiments import PROJECT_ROOT
from ego_hand_benchmark.geometry.fisheye import validate_camera


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def load_calibration(path):
    """Normalize task-book KB4 or native benchmark FISHEYE624 dictionaries."""
    data = read_json(path)
    cameras = []
    for side in ("left", "right"):
        source = data[side]
        model = source.get("model", data.get("model"))
        model = {"opencv_fisheye": "KB4", "kb4": "KB4"}.get(model, model)
        if model not in ("KB4", "FISHEYE624"):
            raise ValueError(f"Unsupported native video calibration: {model}")
        if "K" in source:
            camera = {"model": model, "camera_matrix": {"matrix_3x3": source["K"]},
                      "distortion_coefficients": source["D"],
                      "image_width": source["width"], "image_height": source["height"]}
        else:
            camera = {**source, "model": model}
        validate_camera(camera)
        for key in ("image_width", "image_height"):
            if int(camera[key]) != camera[key]:
                raise ValueError("Calibration dimensions must be integers")
        cameras.append(camera)
    extrinsic = data["T_left_right"]
    if isinstance(extrinsic, dict):
        transform = np.eye(4)
        transform[:3, :3] = np.asarray(extrinsic["R"], dtype=float)
        transform[:3, 3] = np.asarray(extrinsic["t_m"], dtype=float)
    else:
        if data.get("translation_units") != "m":
            raise ValueError("Matrix T_left_right requires translation_units='m'")
        transform = np.asarray(extrinsic, dtype=float)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("Invalid T_left_right")
    rotation = transform[:3, :3]
    if (not np.allclose(transform[3], [0, 0, 0, 1])
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-7)
            or np.linalg.norm(transform[:3, 3]) <= 1e-9):
        raise ValueError("T_left_right must be a proper rigid transform with nonzero baseline in meters")
    return cameras[0], cameras[1], transform


def read_timestamps(path):
    with Path(path).open(newline="") as stream:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or [])
        units = fields & {"timestamp_ns", "timestamp_s"}
        if "frame_id" not in fields or len(units) != 1:
            raise ValueError("Timestamp CSV needs frame_id and exactly one timestamp_ns or timestamp_s column")
        unit = units.pop()
        times = []
        for index, row in enumerate(reader):
            if int(row["frame_id"]) != index:
                raise ValueError("CSV frame_id must be contiguous decode indices starting at zero")
            value = Decimal(row[unit]) * (1 if unit == "timestamp_ns" else 1_000_000_000)
            if not value.is_finite() or value != value.to_integral_value():
                raise ValueError("Timestamps must be finite and exactly representable in nanoseconds")
            times.append(int(value))
    if not times or any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("Timestamps must be nonempty and strictly increasing")
    return np.asarray(times, dtype=np.int64)


def pair_timestamps(left_ns, right_ns, *, max_skew_ms):
    """Nearest-right matching; reused right frames are explicit, as in core sync."""
    if not np.isfinite(max_skew_ms) or max_skew_ms < 0:
        raise ValueError("max_skew_ms must be finite and nonnegative")
    left, right = np.asarray(left_ns, dtype=np.int64), np.asarray(right_ns, dtype=np.int64)
    if any(len(x) == 0 or np.any(np.diff(x) <= 0) for x in (left, right)):
        raise ValueError("Both clocks must be nonempty and strictly increasing")
    candidates = []
    for i, timestamp in enumerate(left):
        j = int(np.searchsorted(right, timestamp))
        nearest = min({max(0, j - 1), min(len(right) - 1, j)},
                      key=lambda k: (abs(int(right[k]) - int(timestamp)), k))
        skew = int(right[nearest]) - int(timestamp)
        if abs(skew) <= max_skew_ms * 1_000_000:
            candidates.append((abs(skew), i, nearest, skew))
    used, pairs = set(), []
    counts = {}
    for _, _, j, _ in candidates:
        counts[j] = counts.get(j, 0) + 1
    for _, i, j, skew in candidates:
        pairs.append({"left_frame_id": i, "right_frame_id": j, "skew_ns": skew,
                      "right_reused": counts[j] > 1})
        used.add(j)
    pairs.sort(key=lambda p: p["left_frame_id"])
    return {"policy": "nearest_right_reuse_explicit", "max_skew_ms": max_skew_ms,
            "pairs": pairs, "unmatched_left": len(left) - len(pairs),
            "unmatched_right": len(right) - len(used),
            "max_observed_skew_ms": max((abs(p["skew_ns"]) / 1e6 for p in pairs), default=None)}


def _decode(video, folder, camera, timestamps, fps, start_time, progress):
    folder.mkdir(parents=True)
    capture = cv2.VideoCapture(str(video))
    frames = []
    try:
        if not capture.isOpened():
            raise OSError(f"Cannot open video: {video}")
        declared_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        while True:
            ok, image = capture.read()
            if not ok:
                break
            index = len(frames)
            if image.shape[:2] != (camera["image_height"], camera["image_width"]):
                raise ValueError(f"Video/calibration resolution mismatch: {video}, frame {index}")
            if timestamps is not None:
                if index >= len(timestamps):
                    raise ValueError("More decoded frames than CSV timestamps")
                timestamp = int(timestamps[index])
            else:
                timestamp = int((Decimal(str(start_time)) + Decimal(index) / Decimal(str(fps))) * 1_000_000_000)
            target = folder / f"{index:09d}.png"
            if not cv2.imwrite(str(target), image):
                raise OSError(f"Cannot write decoded image: {target}")
            frames.append({"frame_id": index, "timestamp_ns": timestamp,
                           "image_path": str(target), "image_sha256": sha256(target)})
            if progress:
                progress({"stage": "decode", "video": str(video), "completed_frames": len(frames),
                          "container_frame_count": declared_count})
        if not frames or (declared_count > 0 and len(frames) != declared_count):
            raise ValueError("Empty/truncated video or inconsistent container frame count")
        if timestamps is not None and len(frames) != len(timestamps):
            raise ValueError("Decoded frame count differs from CSV timestamp count")
    finally:
        capture.release()
    return frames


def prepare_video_inputs(*, left_video, right_video, calibration, cache_root,
                         max_skew_ms, timestamp_clock, sync_source,
                         timestamps_left=None, timestamps_right=None,
                         fps_left=None, fps_right=None, start_time_left=None, start_time_right=None,
                         progress=None):
    """Create/reuse hash-validated standard sequences under data/derived.

    CSV wins over supplied FPS. A partial/altered cache is rejected, never overwritten.
    ``sync_source`` describes acquisition evidence; it does not assert hardware sync.
    """
    if not timestamp_clock or not sync_source:
        raise ValueError("An explicit common timestamp_clock and sync_source are required")
    cache_root = Path(cache_root).expanduser().resolve()
    derived = (PROJECT_ROOT / "data/derived").resolve()
    if cache_root != derived and derived not in cache_root.parents:
        raise ValueError(f"Decoded caches must live under {derived}")
    paths = {"left_video": left_video, "right_video": right_video, "calibration": calibration,
             "timestamps_left": timestamps_left, "timestamps_right": timestamps_right}
    inputs = {key: {"path": str(Path(value).expanduser().resolve()),
                    "sha256": sha256(Path(value).expanduser())} for key, value in paths.items() if value is not None}
    left_camera, right_camera, transform = load_calibration(inputs["calibration"]["path"])
    clocks = []
    for side, csv_path, fps, start in (("left", timestamps_left, fps_left, start_time_left),
                                       ("right", timestamps_right, fps_right, start_time_right)):
        if csv_path is not None:
            clocks.append({"source": "timestamp_csv", "timestamps": read_timestamps(Path(csv_path).expanduser()).tolist()})
        else:
            if fps is None or start is None or not np.isfinite([fps, start]).all() or fps <= 0:
                raise ValueError(f"{side}: provide timestamp CSV or explicit positive fps and start-time")
            clocks.append({"source": "estimated_fps_start_time", "fps": fps, "start_time_s": start})
    specification = {"version": 1, "inputs": inputs, "clocks": clocks,
                     "timestamp_clock": timestamp_clock, "sync_source": sync_source, "max_skew_ms": max_skew_ms,
                     "decoder": {"library": "opencv", "version": cv2.__version__, "format": "lossless_png"}}
    key = fingerprint(specification)
    root = cache_root / key
    manifest = root / "prepared.json"
    if root.exists():
        if not manifest.is_file():
            raise ValueError(f"Incomplete cache retained for diagnosis; choose a new cache root: {root}")
        prepared = read_json(manifest)
        if prepared["specification"] != specification:
            raise ValueError("Decode cache specification mismatch")
        for side in ("left", "right"):
            entry = prepared["sequences"][side]
            if sha256(entry["path"]) != entry["sha256"]:
                raise ValueError("Modified cached sequence")
            for frame in read_json(entry["path"])["frames"]:
                if sha256(frame["image_path"]) != frame["image_sha256"]:
                    raise ValueError("Modified decoded frame")
        return prepared
    root.mkdir(parents=True, exist_ok=False)
    write_json(root / "preparation_status.json", {"status": "running", "specification": specification})
    try:
        sequences = {}
        times = []
        for side, camera, clock in zip(("left", "right"), (left_camera, right_camera), clocks):
            frames = _decode(inputs[f"{side}_video"]["path"], root / side / "frames", camera,
                             clock.get("timestamps"), clock.get("fps"), clock.get("start_time_s"), progress)
            sequence = {"sequence_id": key, "camera_id": side, "coordinate_frame": f"{side}_optical",
                        "timestamp_clock": timestamp_clock, "timestamp_source": clock["source"],
                        "sync_source": sync_source, "source_type": "stereo_video", "camera": camera,
                        "source_video": inputs[f"{side}_video"]["path"],
                        "source_video_sha256": inputs[f"{side}_video"]["sha256"],
                        "selection": {"start_frame": 0, "stride": 1, "max_frames": len(frames)}, "frames": frames}
            path = root / side / "sequence.json"
            write_json(path, sequence)
            sequences[side] = {"path": str(path), "sha256": sha256(path)}
            times.append([f["timestamp_ns"] for f in frames])
        sync = pair_timestamps(*times, max_skew_ms=max_skew_ms)
        if not sync["pairs"]:
            raise ValueError("No stereo frame pairs within the declared maximum skew")
        sync.update(source=sync_source, timestamp_clock=timestamp_clock,
                    timestamp_sources=[c["source"] for c in clocks], hardware_sync_verified=False,
                    clock_correction="none; caller must supply times in a common clock")
        prepared = {"specification": specification, "sequences": sequences, "sync": sync,
                    "left_camera": left_camera, "right_camera": right_camera,
                    "T_left_right": transform.tolist(), "transform_convention": "X_left=R X_right+t (meters)",
                    "cache_path": str(root)}
        write_json(manifest, prepared)
        write_json(root / "preparation_status.json", {"status": "complete"})
        return prepared
    except BaseException as error:
        write_json(root / "preparation_status.json", {"status": "failed", "error": repr(error)})
        raise


def run_video_inference(prepared, output, *, backend="anyhand_wilor", keypoints_source="mano_projection",
                        method_config=None, prediction_caches=None, progress=None):
    """Reuse deployed adapters; only reuse caches sealed by this function.

    Returns left/right predictions.npz paths for observations.load_hot3d_observations.
    Raw MANO and local-perspective transforms remain beside each prediction archive.
    """
    if keypoints_source != "mano_projection":
        raise NotImplementedError("independent_2d is reserved; no independent detector is implemented")
    if backend == "anyhand_wilor":
        from ego_hand_benchmark.models.anyhand_adapter import AnyHandAdapter
        adapter = AnyHandAdapter()
    elif backend == "wilor":
        from ego_hand_benchmark.models.wilor_adapter import WiLoRAdapter
        adapter = WiLoRAdapter()
    else:
        raise ValueError(f"Unknown backend: {backend}")
    config = {"backend": "wilor", "device": "cuda",
              "batch_size": 8, "warmup_frames": 0, "detector_confidence": .3, "detector_iou": .3,
              "rescale_factor": 2., "virtual_size": 512, "max_virtual_fov_deg": 160.,
              **(method_config or {}), "camera_mode": "calibrated_virtual",
              "id": "anyhand_wilor_calibrated_raw" if backend == "anyhand_wilor" else "wilor_calibrated_raw"}
    if config.get("detections_from"):
        raise ValueError("Use sealed prediction caches; external detection reuse is not supported here")
    asset_hashes = {str((PROJECT_ROOT / p).resolve()): sha256(PROJECT_ROOT / p) for p in adapter.assets(config)}
    source = adapter.source_state()
    code = {str(p.relative_to(PROJECT_ROOT)): sha256(p) for folder in ("models", "geometry", "core")
            for p in (PROJECT_ROOT / "ego_hand_benchmark" / folder).glob("*.py")}
    paths = {}
    for side in ("left", "right"):
        entry = prepared["sequences"][side]
        if sha256(entry["path"]) != entry["sha256"]:
            raise ValueError("Sequence changed after preparation")
        for frame in read_json(entry["path"])["frames"]:
            if sha256(frame["image_path"]) != frame["image_sha256"]:
                raise ValueError("Decoded image changed after preparation")
        spec = {"backend": backend, "config": config, "assets_sha256": asset_hashes,
                "source": source, "code_sha256": code, "sequence_sha256": entry["sha256"],
                "keypoints_source": keypoints_source}
        cached = (prediction_caches or {}).get(side)
        folder = Path(cached).expanduser().resolve() if cached else Path(output) / side
        if progress:
            progress({"stage": "inference", "camera": side, "reusing_cache": bool(cached)})
        seal = folder / "video_cache.json"
        if cached:
            record = read_json(seal)
            if record["specification"] != spec:
                raise ValueError(f"Prediction cache specification mismatch: {side}")
            for name, expected in record["files_sha256"].items():
                if sha256(folder / name) != expected:
                    raise ValueError(f"Modified prediction cache file: {name}")
            path = folder / "predictions.npz"
        else:
            path = adapter.run(entry["path"], config, folder)
            files = [folder / "predictions.npz", folder / "metadata.json", folder / "job.json"]
            files += sorted((folder / "raw").glob("*.npz"))
            if len(files) != 3 + len(read_json(entry["path"])["frames"]):
                raise ValueError("Adapter must retain one raw MANO/geometry record per frame")
            write_json(seal, {"specification": spec,
                              "files_sha256": {str(p.relative_to(folder)): sha256(p) for p in files}})
        meta = read_json(folder / "metadata.json")
        if meta.get("status") != "complete" or meta.get("sequence_sha256") != entry["sha256"]:
            raise ValueError("Incomplete or inconsistent prediction metadata")
        prediction = load_predictions(path)
        sequence = read_json(entry["path"])
        if (str(prediction["camera_id"]) != side
                or not np.array_equal(prediction["timestamp_ns"], np.repeat([f["timestamp_ns"] for f in sequence["frames"]], 2))
                or not np.array_equal(prediction["frame_id"], np.repeat([f["frame_id"] for f in sequence["frames"]], 2))):
            raise ValueError("Prediction camera/frame/timestamps mismatch")
        paths[side] = str(path)
    return paths
