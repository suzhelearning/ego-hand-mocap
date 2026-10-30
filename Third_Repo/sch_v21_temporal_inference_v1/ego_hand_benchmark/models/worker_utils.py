import argparse
from pathlib import Path

import cv2

from ego_hand_benchmark.core.io import read_json, write_json


def load_job():
    parser = argparse.ArgumentParser()
    parser.add_argument("--job", required=True)
    args = parser.parse_args()
    job = read_json(args.job)
    sequence = read_json(job["sequence_path"])
    return job, sequence, Path(job["output"])


def read_frame(frame):
    image = cv2.imread(frame["image_path"])
    if image is None:
        raise OSError(f"Cannot decode shared frame {frame['image_path']}")
    return image


def synchronize(torch, device):
    if str(device).startswith("cuda"):
        torch.cuda.synchronize(device)


def finish_metadata(output, torch, config, load_seconds, samples, extra):
    frame_count = sum(s["frames"] for s in samples)
    seconds = sum(s["seconds"] for s in samples)
    device = config.get("device", "cuda")
    metadata = {
        "torch_version": str(torch.__version__),
        "cuda_version": torch.version.cuda,
        "device": torch.cuda.get_device_name(device) if str(device).startswith("cuda") else "cpu",
        "model_load_seconds": load_seconds,
        "timing": {
            "scope": "Synchronized in-memory detection + preprocessing + pose; excludes shared decoding, image loading, warmup, serialization and rendering",
            "samples": samples,
            "evaluated_frames": frame_count,
            "pipeline_seconds": seconds,
            "pipeline_mean_ms_per_frame": seconds * 1000 / frame_count,
            "pipeline_throughput_fps": frame_count / seconds,
            "warmup_requested_frames": config.get("warmup_frames", 8),
            "peak_torch_allocated_mb": torch.cuda.max_memory_allocated(device) / 1024**2
            if str(device).startswith("cuda")
            else None,
            "memory_scope": "PyTorch allocator only; excludes ONNX Runtime and other CUDA allocations",
            "latency_note": "Batch-normalized throughput, not online per-frame latency; compare batch settings explicitly",
        },
        **extra,
    }
    if extra.get("timing_scope"):
        metadata["timing"]["scope"] = extra["timing_scope"]
    write_json(output / "worker_metadata.json", metadata)
