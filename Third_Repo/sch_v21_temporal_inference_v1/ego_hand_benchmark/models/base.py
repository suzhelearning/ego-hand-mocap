"""Each adapter owns a long-lived worker in its existing Python environment."""

import hashlib
import os
import subprocess
import time
from pathlib import Path

from ego_hand_benchmark.core.io import read_json, sha256, write_json

PROJECT_ROOT = Path(__file__).resolve().parents[2]


def repository_state(path):
    path = Path(path)
    manifest = read_json(path / "SOURCE.json")
    for relative, expected in manifest["files"].items():
        if sha256(path / relative) != expected:
            raise ValueError(f"Exported source changed: {path / relative}")
    return {"commit": manifest["commit"], "status": ["source-only export"],
            "tracked_diff_sha256": None, "export_sha256": sha256(path / "SOURCE.json")}


class ModelAdapter:
    name = ""
    source_directory = ""
    python_environment = ""
    asset_paths = ()
    repository_directory = None

    def assets(self, config):
        return self.asset_paths

    def source_state(self):
        return repository_state(PROJECT_ROOT / (self.repository_directory or self.source_directory))

    def worker_environment(self, config):
        return {}

    def run(self, sequence_path, config, output):
        output = Path(output).resolve()
        output.mkdir(parents=True, exist_ok=False)
        python = PROJECT_ROOT / ".venvs" / self.python_environment / "bin/python"
        if not python.is_file():
            raise FileNotFoundError(python)
        assets = {}
        for relative in self.assets(config):
            path = PROJECT_ROOT / relative
            if not path.is_file():
                raise FileNotFoundError(path)
            assets[relative] = sha256(path)
        sequence_path = Path(sequence_path).resolve()
        job = {
            "sequence_path": str(sequence_path),
            "config": config,
            "output": str(output),
            "project_root": str(PROJECT_ROOT),
        }
        write_json(output / "job.json", job)
        env = os.environ.copy()
        source = PROJECT_ROOT / self.source_directory
        env["PYTHONPATH"] = os.pathsep.join([str(PROJECT_ROOT), str(source)])
        env["PYOPENGL_PLATFORM"] = "egl"
        env["TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD"] = "1"
        env["YOLO_AUTOINSTALL"] = "false"
        env["HF_HOME"] = str(PROJECT_ROOT / ".cache/huggingface")
        env["OMP_NUM_THREADS"] = "8"
        env["OPENBLAS_NUM_THREADS"] = "8"
        env["PYTHONUNBUFFERED"] = "1"
        env.update(self.worker_environment(config))
        command = [
            str(python),
            "-m",
            f"ego_hand_benchmark.models.{self.name}_adapter",
            "--job",
            str(output / "job.json"),
        ]
        provenance = {
            "adapter": self.name,
            "method_config": config,
            "source": self.source_state(),
            "assets_sha256": assets,
            "python": str(python),
            "command": command,
            "sequence_sha256": sha256(sequence_path),
            "status": "running",
        }
        write_json(output / "metadata.json", provenance)
        started = time.perf_counter()
        with (output / "worker.log").open("w") as log:
            completed = subprocess.run(
                command, cwd=source, env=env, stdout=log, stderr=subprocess.STDOUT, check=False
            )
        provenance["process_wall_seconds"] = time.perf_counter() - started
        provenance["status"] = "complete" if completed.returncode == 0 else "failed"
        if (output / "worker_metadata.json").is_file():
            provenance.update(read_json(output / "worker_metadata.json"))
        write_json(output / "metadata.json", provenance)
        if completed.returncode:
            tail = "\n".join((output / "worker.log").read_text(errors="replace").splitlines()[-35:])
            raise RuntimeError(f"{self.name} worker failed; see {output / 'worker.log'}\n{tail}")
        provenance["predictions_sha256"] = sha256(output / "predictions.npz")
        write_json(output / "metadata.json", provenance)
        return output / "predictions.npz"
