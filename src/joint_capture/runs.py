"""Create immutable-by-convention processing run records."""
from datetime import datetime
from datetime import timezone
from importlib import metadata
import json
import hashlib
from pathlib import Path
import sys
from .paths import ProjectPaths, checked_id


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _version(name: str):
    try:
        return metadata.version(name)
    except metadata.PackageNotFoundError:
        return None


def create_run(paths: ProjectPaths, session_id: str, run_id: str,
               stages: list[str], config: dict | None = None) -> Path:
    session_id = checked_id(session_id, "session-id")
    run_id = checked_id(run_id, "run-id")
    session = paths.session(session_id)
    if not (session / "session.json").is_file():
        raise FileNotFoundError(session / "session.json")
    session_manifest = json.loads((session / "session.json").read_text(encoding="utf8"))
    if not stages:
        raise ValueError("a processing run must name at least one requested stage")
    run_dir = paths.run(session_id, run_id)
    run_dir.mkdir(parents=True, exist_ok=False)
    for name in ("config", "ego", "mocap", "sync", "aligned", "evaluation", "visualization", "logs", "work"):
        (run_dir / name).mkdir()
    effective_config = config or {}
    config_path = run_dir / "config" / "effective.json"
    config_path.write_text(json.dumps(effective_config, ensure_ascii=False, indent=2) + "\n", encoding="utf8")
    source_root = paths.root
    source_files = [
        source_root / "src" / "joint_capture" / "alignment.py",
        source_root / "src" / "joint_capture" / "manifests.py",
        source_root / "src" / "joint_capture" / "paths.py",
        source_root / "src" / "joint_capture" / "runs.py",
        source_root / "configs" / "network" / "lab.json",
        source_root / "pyproject.toml",
    ]
    fingerprints = {
        p.relative_to(source_root).as_posix(): _sha256(p)
        for p in source_files if p.is_file()
    }
    run = {
        "schema_version": 1, "session_id": session_id, "run_id": run_id,
        "status": "running", "created_at": datetime.now(timezone.utc).isoformat(),
        "stages": {name: {"status": "running", "started_at": datetime.now(timezone.utc).isoformat()} for name in stages},
        "inputs": [{"asset_id": name, **asset} for name, asset in session_manifest.get("assets", {}).items()],
        "config": {"snapshot": "config/effective.json", "values": effective_config},
        "code_revision": None, "source_fingerprints": fingerprints,
        "environment": {"python": sys.version.split()[0],
                        "packages": {name: _version(name) for name in ("numpy", "scipy", "h5py")}},
        "notes": ["此项目目录不含Git元数据；使用source_fingerprints记录关键源码哈希。"],
    }
    (run_dir / "run.json").write_text(json.dumps(run, ensure_ascii=False, indent=2) + "\n", encoding="utf8")
    return run_dir


def finish_stage(run_dir: Path, stage: str, status: str, outputs: list[dict] | None = None,
                 error: dict | None = None) -> None:
    if status not in {"complete", "failed", "reused", "not_run", "unknown"}:
        raise ValueError("invalid processing stage status")
    path = Path(run_dir) / "run.json"
    run = json.loads(path.read_text(encoding="utf8"))
    if stage not in run["stages"]:
        run["stages"][stage] = {"status": "not_run"}
    stage_record = {"status": status, "outputs": outputs or [],
                    "finished_at": datetime.now(timezone.utc).isoformat()}
    if error:
        stage_record["error"] = error
    run["stages"][stage] = stage_record
    statuses = [item["status"] for item in run["stages"].values()]
    if "failed" in statuses:
        run["status"] = "failed"
        run["finished_at"] = datetime.now(timezone.utc).isoformat()
    elif all(value not in {"running", "not_run"} for value in statuses):
        run["status"] = "failed" if "failed" in statuses else ("unknown" if "unknown" in statuses else "complete")
        run["finished_at"] = datetime.now(timezone.utc).isoformat()
    path.write_text(json.dumps(run, ensure_ascii=False, indent=2) + "\n", encoding="utf8")

