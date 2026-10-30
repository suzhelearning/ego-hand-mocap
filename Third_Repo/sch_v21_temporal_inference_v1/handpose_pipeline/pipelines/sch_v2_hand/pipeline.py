"""GT-free orchestration and benchmark export for SCH v2 hand reconstruction."""

from pathlib import Path
import time
import numpy as np

from ego_hand_benchmark.core.io import save_predictions
from ego_hand_benchmark.core.schema import empty_predictions, set_prediction
from deployment.infer import atomic_json

METHODS = {"stereo": "sch-v2-hand-stereo", "mano": "sch-v2-hand-mano",
           "mano_temporal": "sch-v2-hand"}
METHODS_V21 = {"stereo": "sch-v2.1-hand-stereo", "mano": "sch-v2.1-hand-mano",
               "mano_temporal": "sch-v2.1-hand"}


def validate_config(config):
    """Reject misspelled top-level options before any inference or output writes."""
    if not isinstance(config, dict):
        raise ValueError("Pipeline config must be an object")
    unknown = set(config) - {"max_skew_ms", "render", "geometry", "optimizer", "association"}
    if unknown:
        raise ValueError(f"Unknown pipeline settings: {sorted(unknown)}")
    if "render" in config and not isinstance(config["render"], bool):
        raise ValueError("render must be boolean")
    skew = config.get("max_skew_ms", 5.0)
    if isinstance(skew, bool) or not isinstance(skew, (int, float)) or not np.isfinite(skew) or skew < 0:
        raise ValueError("max_skew_ms must be finite and nonnegative")
    for name in ("geometry", "optimizer", "association"):
        if name in config and not isinstance(config[name], dict):
            raise ValueError(f"{name} must be an object")
    return config


def save_arrays(path, data):
    arrays = {k: np.asarray(v) for k, v in data.items()
              if isinstance(v, (np.ndarray, list, tuple, str, int, float, bool))}
    arrays = {k: v for k, v in arrays.items() if v.dtype != object}
    np.savez_compressed(path, **arrays)


def export_prediction(sequence, result, camera, method, path):
    joints = np.asarray(result["joints3d_m"], float)
    mask = np.asarray(result["valid"], bool)
    if mask.ndim == 2:
        mask = np.broadcast_to(mask[..., None], joints.shape[:-1])
    if joints.shape != (len(sequence["frames"]), 2, 21, 3) or mask.shape != joints.shape[:-1]:
        raise ValueError("Reconstruction shape differs from selected frames")
    prediction = empty_predictions(sequence, method)
    for t in range(len(joints)):
        for h in range(2):
            points = np.where(mask[t, h, :, None], joints[t, h], np.nan)
            pixels = camera.project(points)
            set_prediction(prediction, 2*t+h, points, pixels,
                           allow_negative_z=camera.camera["model"] in ("KB4", "FISHEYE624"))
    save_predictions(path, prediction)
    return str(Path(path).resolve())


def run_pipeline(left_sequence, right_sequence, left_prediction_path, right_prediction_path,
                 T_left_right, output, *, stage="mano_temporal", config=None, progress=None,
                 methods=None):
    from .observations import load_hot3d_observations
    from .geometry import triangulate_observations
    from .cameras import validate_transform
    from ego_hand_benchmark.models.hot3d_monocular import BenchmarkCamera as Camera
    from .visualize import render_overlays, plot_losses

    config = validate_config({} if config is None else config)
    if stage not in METHODS:
        raise ValueError(f"Unknown reconstruction stage: {stage}")
    methods = METHODS if methods is None else methods
    if set(methods) != set(METHODS) or len(set(methods.values())) != len(METHODS):
        raise ValueError("Expected three distinct stage method IDs")
    T_left_right = validate_transform(T_left_right)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    timings = {}

    def update(label, **kwargs):
        payload = {"status": "running", "stage": label, **kwargs}
        atomic_json(output / "progress.json", payload)
        if progress:
            progress(payload)

    start = time.perf_counter()
    update("cache_io")
    obs, mono, metadata = load_hot3d_observations(
        left_sequence, right_sequence, left_prediction_path, right_prediction_path,
        max_skew_ms=config.get("max_skew_ms", 5.0))
    if config.get("association"):
        from .association import associate_observations
        obs, mono, association = associate_observations(
            obs, mono, left_sequence["camera"], right_sequence["camera"],
            T_left_right, config=config["association"])
        save_arrays(output / "association.npz", association)
        atomic_json(output / "association_summary.json", {
            key: value for key, value in association.items()
            if not isinstance(value, np.ndarray)})
        metadata["association_config"] = config["association"]
    save_arrays(output / "observations.npz", obs)
    save_arrays(output / "mono_mano.npz", mono)
    atomic_json(output / "observation_metadata.json", metadata)
    timings["cache_io_seconds"] = time.perf_counter() - start
    cameras = [Camera(s["camera"]) for s in (left_sequence, right_sequence)]
    update("geometry")
    start = time.perf_counter()
    calibration = [s["camera"] for s in (left_sequence, right_sequence)]
    tri = triangulate_observations(obs, *calibration, T_left_right, config=config.get("geometry"))
    save_arrays(output / "triangulated.npz", tri)
    timings["geometry_seconds"] = time.perf_counter() - start
    results = {"stereo": tri}
    if stage != "stereo":
        from .optimizer import fit_mano
        update(stage)
        start = time.perf_counter()
        settings = {**config.get("optimizer", {})}
        if stage == "mano":
            settings["iterations_c"] = 0
        fitted = fit_mano(obs, tri, mono, *calibration, T_left_right, config=settings,
                          progress=lambda value: update(stage, optimizer=value))
        results["mano"] = fitted["per_frame"]
        save_arrays(output / "mano_per_frame.npz", results["mano"])
        if stage == "mano_temporal":
            results["mano_temporal"] = fitted["temporal"]
            save_arrays(output / "mano_optimized.npz", results["mano_temporal"])
        diagnostics = fitted.get("diagnostics", {})
        atomic_json(output / "optimizer_diagnostics.json", diagnostics)
        plot_losses(diagnostics, output / "optimizer")
        timings["optimization_seconds"] = time.perf_counter() - start
        timings["optimizer_stage_wall_seconds"] = {
            key: float(value["wall_seconds"]) for key, value in diagnostics.get("stage_timings", {}).items()
            if value.get("executed")}
    predictions = {}
    for name, result in results.items():
        folder = output / methods[name]
        folder.mkdir(exist_ok=True)
        predictions[methods[name]] = export_prediction(left_sequence, result, cameras[0], methods[name],
                                                       folder / "predictions.npz")
        atomic_json(folder / "metadata.json", {
            "status": "complete", "input_modalities": "stereo", "output_stage": name,
            "coordinate_frame": left_sequence["coordinate_frame"], "units": "m",
            "timing_note": "Cached AnyHand inference excluded; see clip profiling.json for stage costs",
            "timing": {"scope": "geometry plus executed optimization stages; excludes cached IO, initialization, export, plots, original AnyHand inference and rendering",
                       "evaluated_frames": len(left_sequence["frames"]),
                       "pipeline_seconds": timings["geometry_seconds"] + sum(
                           timings.get("optimizer_stage_wall_seconds", {}).get(key, 0.)
                           for key in ({"stereo": (), "mano": ("A", "B"), "mano_temporal": ("A", "B", "C")}[name]))},
        })
    update("render")
    start = time.perf_counter()
    if config.get("render", True):
        render_overlays(left_sequence, right_sequence, obs, results, cameras, T_left_right, output)
    timings["render_seconds"] = time.perf_counter() - start
    quality = {"ground_truth_used": False, "coordinate_frame": left_sequence["coordinate_frame"],
               "units": "m", "stages": {}}
    def stats(values):
        values = np.asarray(values)
        values = values[np.isfinite(values)]
        return {"count": int(values.size), "mean": float(values.mean()) if values.size else None,
                "median": float(np.median(values)) if values.size else None,
                "max": float(values.max()) if values.size else None}
    quality["geometry"] = {key: stats(tri[key][tri["valid"]]) for key in
                           ("angle_deg", "ray_gap_m", "reproj_px", "ray_depth_m") if key in tri}
    quality["geometry_candidates_including_rejected"] = {key: stats(tri[key]) for key in
                           ("angle_deg", "ray_gap_m", "reproj_px", "ray_depth_m") if key in tri}
    quality["per_joint_valid_ratio"] = tri["valid"].mean(axis=(0, 1)).tolist()
    quality["geometry_complete_hand_ratio"] = float(tri["valid"].all(-1).mean())
    quality["geometry_any_hand_ratio"] = float(tri["valid"].any(-1).mean())
    quality["skew_ms"] = stats(obs["skew_ms"])
    quality["failure_counts"] = {key: int(np.asarray(value).sum()) for key, value in tri.items()
                                 if key.startswith("reason_") and key != "reason_mask"}
    for name, result in results.items():
        mask = np.asarray(result["valid"], bool)
        quality["stages"][name] = {"valid_ratio": float(mask.mean()), "valid_count": int(mask.sum()),
                                  "mask_shape": list(mask.shape)}
    atomic_json(output / "quality.json", quality)
    atomic_json(output / "profiling.json", timings)
    update("complete")
    return predictions
