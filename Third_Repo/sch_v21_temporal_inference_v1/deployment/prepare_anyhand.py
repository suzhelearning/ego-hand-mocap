"""Prepare small, pinned configuration and links to existing shared assets (no checkpoints)."""

import hashlib
import json
from pathlib import Path
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]


def link_asset(source, destination):
    if not source.is_file():
        raise FileNotFoundError(f"Shared asset missing: {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists() or destination.is_symlink():
        if destination.resolve() != source.resolve():
            raise FileExistsError(f"Inspect existing asset before replacing: {destination}")
        return
    destination.symlink_to(source.resolve())


def main():
    sources = json.loads((ROOT / "deployment/anyhand-sources.json").read_text())
    weights = ROOT / "weights/anyhand/wilor"
    weights.mkdir(parents=True, exist_ok=True)
    (ROOT / "weights/anyhand/hamer").mkdir(parents=True, exist_ok=True)
    name = "model_config_wilor.yaml"
    target = weights / name
    expected = sources["files"][name]
    if target.exists():
        data = target.read_bytes()
    else:
        url = f"https://huggingface.co/{sources['model_repository']}/resolve/{sources['model_revision']}/{name}"
        with urlopen(url, timeout=60) as response:
            data = response.read()
    if len(data) != expected["bytes"] or hashlib.sha256(data).hexdigest() != expected["sha256"]:
        raise ValueError(f"Configuration hash mismatch: {target}")
    if not target.exists():
        target.write_bytes(data)
    for name in ("MANO_RIGHT.pkl", "mano_mean_params.npz"):
        link_asset(ROOT / "WiLoR/mano_data" / name, ROOT / "AnyHand/mano_data" / name)
    link_asset(ROOT / "WiLoR/pretrained_models/detector.pt", ROOT / "AnyHand/pretrained_models/detector.pt")
    print(f"Configuration ready: {target}")
    print("Checkpoint supplied by user; pass --checkpoint /absolute/path/anyhand_wilor.ckpt.")


if __name__ == "__main__":
    main()
