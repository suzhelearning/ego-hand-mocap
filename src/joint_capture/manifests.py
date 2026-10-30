"""Read legacy capture manifests and resolve canonical session assets."""
import json
from pathlib import Path
from .paths import ProjectPaths


def load_session_manifest(path: Path) -> dict:
    value = json.loads(Path(path).read_text(encoding="utf8"))
    if not isinstance(value, dict):
        raise ValueError("session manifest must be a JSON object")
    return value


def resolve_session_asset(root: Path, manifest: dict, asset_id: str) -> Path:
    """Resolve canonical IDs, falling back to legacy absolute manifest fields."""
    session_id = manifest.get("session_id")
    if not session_id:
        raise ValueError("session manifest has no session_id")
    asset = manifest.get("assets", {}).get(asset_id)
    if isinstance(asset, dict) and asset.get("path"):
        path = ProjectPaths(root).relative_asset(session_id, asset["path"])
        if path.is_file():
            return path
        raise FileNotFoundError(path)
    legacy = {
        "ego_h5": ("ego", "h5"),
        "mocap_h5": ("mocap", "file"),
    }.get(asset_id)
    if legacy:
        value = manifest.get(legacy[0], {}).get(legacy[1])
        if value:
            path = Path(value)
            if path.is_file():
                resolved = path.resolve()
                if not resolved.is_relative_to(ProjectPaths(root).root):
                    raise ValueError(f"legacy manifest asset escapes the project root: {asset_id}")
                return resolved
    raise FileNotFoundError(f"manifest asset unavailable: {asset_id}")


def resolve_clock_directory(session_dir: Path) -> Path:
    flat = Path(session_dir) / "clocks"
    if flat.is_dir():return flat
    current = Path(session_dir) / "raw" / "clocks"
    legacy = Path(session_dir) / "sync"
    return current if current.is_dir() else legacy


def resolve_hdf5_reference(project_root: Path, h5_path: Path, attribute: str) -> Path:
    """Resolve stored HDF5 source-path attributes through the owning session manifest."""
    import h5py

    project_root = Path(project_root).resolve()
    h5_path = Path(h5_path).resolve()
    with h5py.File(h5_path, "r") as handle:
        value = handle.attrs.get(attribute)
        session_id = handle.attrs.get("session_id")
    if value is None:
        raise KeyError(f"HDF5 attribute is absent: {attribute}")
    if isinstance(value, bytes):
        value = value.decode("utf8", errors="replace")
    value = str(value)

    session_dir = next((parent for parent in h5_path.parents if (parent / "session.json").is_file()), None)
    if session_dir is None and session_id:
        if isinstance(session_id, bytes):
            session_id = session_id.decode("utf8", errors="replace")
        candidate = ProjectPaths(project_root).session(str(session_id))
        if (candidate / "session.json").is_file():
            session_dir = candidate

    canonical_id = {"ego_source_h5": "ego_h5", "mocap_source_h5": "mocap_h5"}.get(attribute)
    if session_dir is not None:
        manifest = load_session_manifest(session_dir / "session.json")
        if canonical_id and manifest.get("assets", {}).get(canonical_id):
            resolved = resolve_session_asset(project_root, manifest, canonical_id)
            if resolved.is_file():
                return resolved

        if attribute == "source_trajectory":
            run_dir = next((parent for parent in h5_path.parents if (parent / "run.json").is_file()), None)
            if run_dir is not None:
                matches = [p for p in run_dir.rglob(Path(value).name) if p.is_file()]
                if len(matches) == 1:
                    return matches[0].resolve()

        stored = Path(value)
        if not stored.is_absolute():
            candidate = ProjectPaths(project_root).relative_asset(
                str(manifest["session_id"]), value
            )
            if candidate.is_file():
                return candidate

    stored = Path(value)
    if stored.is_absolute() and stored.is_file() and stored.resolve().is_relative_to(project_root):
        return stored.resolve()
    raise FileNotFoundError(f"could not resolve HDF5 source reference {attribute}: {value}")

