"""Copy legacy project files into the session-centered layout without deleting sources."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import shutil as disk_usage
import uuid
from .paths import checked_id


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _walk(root: Path):
    return sorted(p for p in root.rglob("*") if p.is_file())


def build_migration_plan(root: Path, migration_id: str) -> dict:
    root = Path(root).resolve()
    migration_id = checked_id(migration_id, "migration-id")
    (root / "data" / "sessions").mkdir(parents=True, exist_ok=True)
    entries = []

    def add(source: Path, destination: Path, category: str):
        source, destination = Path(source), Path(destination)
        if not source.is_file():
            return
        if not source.resolve().is_relative_to(root):
            raise ValueError(f"migration source escapes project root: {source}")
        if not destination.parent.resolve().is_relative_to(root):
            raise ValueError(f"migration destination escapes project root: {destination}")
        entries.append({
            "source": source.relative_to(root).as_posix(),
            "destination": destination.relative_to(root).as_posix(),
            "category": category, "size": source.stat().st_size, "sha256": sha256(source),
        })

    capture_root = root / "capture_sessions"
    capture_ids = sorted(p.name for p in capture_root.iterdir() if p.is_dir()) if capture_root.is_dir() else []
    ego_root = root / "data" / "ego_data"
    ego_ids = sorted(p.name for p in ego_root.iterdir() if p.is_dir()) if ego_root.is_dir() else []
    session_ids = sorted(set(capture_ids) | set(ego_ids))

    for sid in capture_ids:
        base = capture_root / sid
        target = root / "data" / "sessions" / sid
        for src in _walk(base):
            rel = src.relative_to(base)
            if rel.as_posix() == "session.json":
                dest = target / "capture" / "session.source.json"
            elif rel.name.endswith(".before_data_move"):
                dest = target / "capture" / "legacy_backups" / rel.name
            elif rel.parts[0] == "sync":
                dest = target / "raw" / "clocks" / Path(*rel.parts[1:])
            else:
                dest = target / "capture" / rel
            add(src, dest, "capture")

    for sid in ego_ids:
        src_dir = ego_root / sid
        for src in _walk(src_dir):
            add(src, root / "data" / "sessions" / sid / "raw" / "ego" / src.relative_to(src_dir), "ego_raw")

    mocap_root = root / "data" / "mocap_data"
    if mocap_root.is_dir():
        for src in _walk(mocap_root):
            sid = src.name.removesuffix(".h5")
            if sid in capture_ids and src.suffix.lower() == ".h5":
                dest = root / "data" / "sessions" / sid / "raw" / "mocap" / "natnet.h5"
                category = "mocap_raw"
            else:
                folder = root / "data" / "unassigned" / ("mocap-" + src.name.replace(".", "-"))
                dest, category = folder / "raw" / src.name, "unassigned_mocap"
            add(src, dest, category)

    c3d_root = root / "data" / "mocap_c3d"
    if c3d_root.is_dir():
        for src in _walk(c3d_root):
            add(src, root / "data" / "samples" / "c3d-format-egotest-001" / "motive" / src.name, "sample_c3d")

    align_root = root / "results" / "alignment_20261029"
    if align_root.is_dir():
        for sid_dir in sorted(p for p in align_root.iterdir() if p.is_dir()):
            for src in _walk(sid_dir):
                add(src, root / "data" / "sessions" / sid_dir.name / "runs" / "legacy-alignment-20261029" / "sync" / src.relative_to(sid_dir), "alignment_result")

    aligned_root = root / "data" / "aligned_h5"
    if aligned_root.is_dir():
        for sid_dir in sorted(p for p in aligned_root.iterdir() if p.is_dir()):
            for src in _walk(sid_dir):
                if src.suffix.lower() in {".h5", ".hdf5"}:
                    add(src, root / "data" / "sessions" / sid_dir.name / "runs" / "legacy-alignment-20261029" / "aligned" / "aligned.h5", "aligned_h5")
        readme = aligned_root / "README.md"
        add(readme, root / "docs" / "formats" / "aligned-h5.md", "format_documentation")

    pose_root = root / "results" / "okvis_pose"
    if pose_root.is_dir():
        for sid_dir in sorted(p for p in pose_root.iterdir() if p.is_dir()):
            for src in _walk(sid_dir):
                rel = src.relative_to(sid_dir)
                add(src, root / "data" / "sessions" / sid_dir.name / "runs" / "legacy-okvis-20261029" / "ego" / "okvis" / "source" / rel, "okvis_result")
    pose_h5_root = root / "data" / "ego_pose_h5"
    if pose_h5_root.is_dir():
        for src in _walk(pose_h5_root):
            sid = src.name.split("_ego_pose.h5")[0]
            add(src, root / "data" / "sessions" / sid / "runs" / "legacy-okvis-20261029" / "ego" / "okvis" / "pose.h5", "okvis_pose_h5")

    audit_root = root / "results"
    for folder in sorted(p for p in audit_root.glob("sync_audit_*") if p.is_dir()):
        for src in _walk(folder):
            add(src, root / "reports" / "sync_audits" / folder.name / src.relative_to(folder), "sync_audit")
    for src in audit_root.glob("joint_capture*.log"):
        add(src, root / "reports" / "capture_logs" / src.name, "capture_log")

    calibration = root / "calibration"
    if calibration.is_dir():
        for src in _walk(calibration):
            rel = src.relative_to(calibration)
            if rel.parts and rel.parts[0] == "devices":
                continue
            if src.name == "head_000205_calibration_full.json":
                dest = root / "calibration" / "devices" / "head_000205" / "extracted-from-h5" / src.name
                category = "device_calibration"
            else:
                dest = root / "data" / "samples" / "calibration-captures" / rel
                category = "calibration_sample"
            add(src, dest, category)

    # Reject two different sources mapped to one canonical destination.
    destinations = {}
    for entry in entries:
        previous = destinations.get(entry["destination"])
        if previous and previous["sha256"] != entry["sha256"]:
            raise ValueError(f"migration destination collision: {entry['destination']}")
        destinations[entry["destination"]] = entry
    entries = list(destinations.values())
    return {
        "schema_version": 1, "migration_id": migration_id,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "sessions": session_ids, "entries": entries,
        "total_bytes": sum(item["size"] for item in entries),
        "sources_retained": True,
    }


def _canonical_session(root: Path, session_id: str, base_override: Path | None = None):
    def add_ego_inventory(value, base):
        assets = value.setdefault("assets", {})
        indexed = {item.get("path") for item in assets.values() if isinstance(item, dict)}
        changed = False
        for index, path in enumerate(sorted((base / "raw" / "ego").rglob("*")), start=1):
            if not path.is_file():
                continue
            relative = path.relative_to(base).as_posix()
            if relative in indexed:
                continue
            key = f"ego_media_{index:04d}"
            while key in assets:
                index += 1
                key = f"ego_media_{index:04d}"
            assets[key] = {"path": relative, "sha256": sha256(path)}
            indexed.add(relative)
            changed = True
        return changed

    base = Path(base_override) if base_override else root / "data" / "sessions" / session_id
    manifest_path = base / "session.json"
    if manifest_path.exists():
        current = json.loads(manifest_path.read_text(encoding="utf8"))
        if current.get("legacy_paths"):
            if add_ego_inventory(current, base):
                manifest_path.write_text(json.dumps(current, ensure_ascii=False, indent=2) + "\n", encoding="utf8")
            return
        if not (base / "capture" / "session.source.json").is_file():
            if add_ego_inventory(current, base):
                manifest_path.write_text(json.dumps(current, ensure_ascii=False, indent=2) + "\n", encoding="utf8")
            return
    source_manifest = base / "capture" / "session.source.json"
    if source_manifest.is_file():
        legacy = json.loads(source_manifest.read_text(encoding="utf8"))
        status = legacy.get("status", "unknown")
        result = dict(legacy)
        result["schema_version"] = 1
        result["migration"] = {"source_manifest": "capture/session.source.json", "legacy_fields_preserved": True}
        result["legacy_paths"] = {
            "mocap_file": legacy.get("mocap", {}).get("file"),
            "ego_local_directory": legacy.get("ego", {}).get("local_directory"),
            "ego_h5": legacy.get("ego", {}).get("h5"),
        }
        assets = {}
        candidates = {
            "ego_h5": base / "raw" / "ego" / Path(legacy.get("ego", {}).get("h5", "")).name,
            "mocap_h5": base / "raw" / "mocap" / "natnet.h5",
            "ego_clock_probes": base / "raw" / "clocks" / "ego.jsonl",
            "windows_clock_probes": base / "raw" / "clocks" / "windows.jsonl",
        }
        for key, path in candidates.items():
            if path.is_file():
                assets[key] = {"path": path.relative_to(base).as_posix(), "sha256": sha256(path)}
        result["assets"] = assets
        if "mocap_h5" in assets:
            result.setdefault("mocap", {})["file"] = assets["mocap_h5"]["path"]
        if "ego_h5" in assets:
            result.setdefault("ego", {})["local_directory"] = "raw/ego"
            result["ego"]["h5"] = assets["ego_h5"]["path"]
        add_ego_inventory(result, base)
        result["status"] = status
    else:
        ego_files = sorted((base / "raw" / "ego").glob("*.h5"))
        assets = {}
        for i, path in enumerate(ego_files):
            key = "ego_h5" if i == 0 else f"ego_h5_{i}"
            assets[key] = {"path": path.relative_to(base).as_posix(), "sha256": sha256(path)}
        result = {
            "schema_version": 1, "session_id": session_id, "status": "unknown",
            "errors": [], "origin": {"kind": "legacy_ego_recording", "source": f"data/ego_data/{session_id}"},
            "assets": assets, "migration": {"source_manifest": None, "association_confidence": "directory_name_only"},
        }
        add_ego_inventory(result, base)
    manifest_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf8")


def _write_historical_runs(root: Path, session_ids: list[str], base_overrides: dict[str, Path] | None = None):
    for sid in session_ids:
        base = (base_overrides or {}).get(sid, root / "data" / "sessions" / sid)
        if not (base / "session.json").is_file():
            continue
        for run_id, stage_names in (
            ("legacy-alignment-20261029", ["clock_alignment", "aligned_export"]),
            ("legacy-okvis-20261029", ["okvis"]),
        ):
            run_dir = base / "runs" / run_id
            if not run_dir.is_dir():
                continue
            run_manifest = run_dir / "run.json"
            if run_manifest.exists():
                current = json.loads(run_manifest.read_text(encoding="utf8"))
                if current.get("status_basis") == "historical_artifacts_only":
                    continue
            files = [p for p in run_dir.rglob("*") if p.is_file()]
            stages = {}
            for name in stage_names:
                outputs = []
                if name == "clock_alignment":
                    selected = [p for p in files if p.parent.name == "sync"]
                elif name == "aligned_export":
                    selected = [p for p in files if p.name == "aligned.h5"]
                elif name == "okvis":
                    selected = [p for p in files if p.name == "pose.h5" or p.name.startswith("okvis2-slam-final")]
                else:
                    selected = []
                outputs = [{"path": p.relative_to(run_dir).as_posix(), "sha256": sha256(p), "size": p.stat().st_size} for p in selected]
                stages[name] = {"status": "unknown", "outputs": outputs,
                                "reason": "历史文件存在只能证明产物可用，不能单独证明阶段执行状态。"}
            metadata = {
                "schema_version": 1, "session_id": sid, "run_id": run_id,
                "status": "historical_unknown",
                "status_basis": "historical_artifacts_only",
                "execution_state": "unknown",
                "created_at": "unknown", "stages": stages,
                "inputs": [{"asset_id": key, **asset} for key, asset in json.loads((base / "session.json").read_text(encoding="utf8")).get("assets", {}).items()],
                "source_roots": [
                    "results/alignment_20261029", "results/okvis_pose",
                    "data/aligned_h5", "data/ego_pose_h5",
                ],
                "config": {}, "config_status": "unknown", "code_revision": None,
                "notes": ["历史产物存在只证明文件可用，不能单独证明对应处理阶段执行成功。"],
                "quality": {"accuracy_validated": False, "provenance": "historical_artifacts"},
            }
            (run_dir / "run.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf8")


def _write_unassigned_manifests(root: Path):
    for area in ("unassigned", "samples"):
        area_root = root / "data" / area
        if not area_root.is_dir():
            continue
        for folder in sorted(p for p in area_root.iterdir() if p.is_dir()):
            metadata_path = folder / "metadata.json"
            desired_status = "sample" if area == "samples" else "unassigned"
            if metadata_path.exists():
                current = json.loads(metadata_path.read_text(encoding="utf8"))
                if current.get("status") == desired_status:
                    continue
            files = [
                {"path": p.relative_to(folder).as_posix(), "size": p.stat().st_size, "sha256": sha256(p)}
                for p in _walk(folder) if p != metadata_path
            ]
            metadata_path.write_text(json.dumps({
                "schema_version": 1, "record_id": folder.name, "status": desired_status,
                "association": "not_assumed", "assets": files,
            }, ensure_ascii=False, indent=2) + "\n", encoding="utf8")


def apply_migration_plan(root: Path, plan: dict) -> dict:
    root = Path(root).resolve()
    checked_id(plan.get("migration_id"), "migration-id")
    migration_dir = root / "reports" / "migrations" / plan["migration_id"]
    if not migration_dir.parent.resolve().is_relative_to(root):
        raise ValueError("migration report path escapes project root")
    migration_dir.mkdir(parents=True, exist_ok=True)
    applied, conflicts, session_groups, other_entries = [], [], {}, []
    for entry in plan["entries"]:
        source_rel, destination_rel = Path(entry["source"]), Path(entry["destination"])
        if source_rel.is_absolute() or destination_rel.is_absolute() or ".." in source_rel.parts or ".." in destination_rel.parts:
            raise ValueError("migration plan paths must be project-relative and cannot traverse")
        source = root / source_rel
        destination = root / destination_rel
        if not source.resolve().is_relative_to(root) or not destination.parent.resolve().is_relative_to(root):
            raise ValueError("migration plan path escapes project root")
        if not source.is_file() or sha256(source) != entry["sha256"]:
            conflicts.append({"entry": entry, "reason": "source_changed"})
        elif destination.is_symlink():
            conflicts.append({"entry": entry, "reason": "destination_symlink"})
        elif destination.exists() and (not destination.is_file() or sha256(destination) != entry["sha256"]):
            conflicts.append({"entry": entry, "reason": "destination_conflict"})
        if destination_rel.parts[:2] == ("data", "sessions") and len(destination_rel.parts) >= 4:
            session_groups.setdefault(destination_rel.parts[2], []).append(entry)
        else:
            other_entries.append(entry)
    for sid, entries in session_groups.items():
        final_dir = root / "data" / "sessions" / sid
        missing = [e for e in entries if not (root / e["destination"]).exists()]
        if final_dir.exists() and missing:
            conflicts.append({"session_id": sid, "reason": "incomplete_final_session"})
        if final_dir.exists() and not (final_dir / "session.json").is_file() and not missing:
            conflicts.append({"session_id": sid, "reason": "final_session_manifest_missing"})
    if conflicts:
        report = {"schema_version": 1, "migration_id": plan["migration_id"], "applied": [],
                  "conflicts": conflicts, "sources_retained": True}
        (migration_dir / "results.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf8")
        raise RuntimeError(f"migration stopped before copying: {len(conflicts)} source/destination conflicts; see {migration_dir / 'results.json'}")
    stage_root = root / "data" / ".migration_staging" / plan["migration_id"]
    if not stage_root.resolve().is_relative_to(root):
        raise ValueError("migration staging path escapes project root")
    progress = {"schema_version": 1, "migration_id": plan["migration_id"], "sessions": {}, "other_files_complete": 0}
    progress_path = migration_dir / "progress.json"

    def copy_one(entry, destination):
        source = root / entry["source"]
        if not source.resolve().is_relative_to(root) or not destination.parent.resolve().is_relative_to(root):
            raise ValueError("migration path changed to a location outside the project root")
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_name(destination.name + ".migration-" + uuid.uuid4().hex[:8])
        try:
            shutil.copy2(source, temporary)
            actual = sha256(temporary)
            if actual != entry["sha256"]:
                raise OSError(f"copy hash mismatch for {entry['destination']}")
            os.replace(temporary, destination)
        finally:
            if temporary.exists():
                temporary.unlink()
        return {"destination": entry["destination"], "status": "copied", "sha256": entry["sha256"]}

    required = 0
    for entry in plan["entries"]:
        destination = root / entry["destination"]
        if destination.exists():
            continue
        destination_rel = Path(entry["destination"])
        if destination_rel.parts[:2] == ("data", "sessions"):
            sid = destination_rel.parts[2]
            staging = stage_root / sid / Path(*destination_rel.parts[3:])
            if not staging.parent.resolve().is_relative_to(root):
                raise ValueError("staged migration path escapes project root")
            if staging.is_file() and sha256(staging) == entry["sha256"]:
                continue
        required += int(entry["size"])
    if disk_usage.disk_usage(root).free < required + 256 * 1024 * 1024:
        raise OSError(f"insufficient free space for copy migration ({required} bytes plus safety margin)")

    try:
        for sid, entries in session_groups.items():
            final_dir = root / "data" / "sessions" / sid
            if final_dir.is_dir():
                for entry in entries:
                    destination = root / entry["destination"]
                    applied.append({"destination": entry["destination"], "status": "already_identical", "sha256": entry["sha256"]})
                progress["sessions"][sid] = "already_identical"
                progress_path.write_text(json.dumps(progress, ensure_ascii=False, indent=2) + "\n", encoding="utf8")
                continue
            staging_dir = stage_root / sid
            staging_dir.mkdir(parents=True, exist_ok=True)
            for entry in entries:
                destination_rel = Path(entry["destination"])
                staged = staging_dir / Path(*destination_rel.parts[3:])
                if staged.is_file():
                    if sha256(staged) != entry["sha256"]:
                        raise RuntimeError(f"staged file differs from plan: {staged}")
                    applied.append({"destination": entry["destination"], "status": "staged_resume", "sha256": entry["sha256"]})
                    continue
                applied.append(copy_one(entry, staged))
            _canonical_session(root, sid, staging_dir)
            _write_historical_runs(root, [sid], {sid: staging_dir})
            final_dir.parent.mkdir(parents=True, exist_ok=True)
            if final_dir.exists():
                raise FileExistsError(f"session appeared during migration: {final_dir}")
            os.replace(staging_dir, final_dir)
            progress["sessions"][sid] = "promoted"
            progress_path.write_text(json.dumps(progress, ensure_ascii=False, indent=2) + "\n", encoding="utf8")

        for entry in other_entries:
            destination = root / entry["destination"]
            if destination.exists():
                applied.append({"destination": entry["destination"], "status": "already_identical", "sha256": entry["sha256"]})
                continue
            applied.append(copy_one(entry, destination))
            progress["other_files_complete"] += 1
            progress_path.write_text(json.dumps(progress, ensure_ascii=False, indent=2) + "\n", encoding="utf8")
    except Exception as exc:
        progress["failed"] = {"error": str(exc), "at": datetime.now(timezone.utc).isoformat()}
        progress_path.write_text(json.dumps(progress, ensure_ascii=False, indent=2) + "\n", encoding="utf8")
        report = {"schema_version": 1, "migration_id": plan["migration_id"], "applied": applied,
                  "conflicts": [], "progress": str(progress_path), "sources_retained": True}
        (migration_dir / "results.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf8")
        raise

    stage_parent = stage_root
    if stage_parent.exists():
        shutil.rmtree(stage_parent)
    session_ids = plan["sessions"]
    for sid in session_ids:
        _canonical_session(root, sid)
    _write_historical_runs(root, session_ids)
    _write_unassigned_manifests(root)
    report = {
        "schema_version": 1, "migration_id": plan["migration_id"],
        "finished_at": datetime.now(timezone.utc).isoformat(),
        "applied": applied, "conflicts": conflicts,
        "sessions_written": [sid for sid in session_ids if (root / "data" / "sessions" / sid / "session.json").is_file()],
        "sources_retained": True,
    }
    (migration_dir / "results.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf8")
    return report

