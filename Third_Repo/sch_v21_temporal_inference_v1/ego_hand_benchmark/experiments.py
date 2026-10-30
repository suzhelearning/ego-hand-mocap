"""One output root for all new benchmark, monocular and stereo experiments."""

from datetime import datetime, timezone
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
RESULTS_ROOT = PROJECT_ROOT / "ego_hand_benchmark/results"


def resolve_output(value=None, *, family="experiment", resume=False, results_root=None):
    root = Path(results_root or RESULTS_ROOT).resolve()
    if value is None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        path = root / f"{family}_{stamp}"
    else:
        requested = Path(value).expanduser()
        # Bare names are run IDs; explicit paths must point into the result root.
        path = (
            root / requested
            if not requested.is_absolute() and len(requested.parts) == 1
            else requested.resolve()
        )
    path = path.resolve()
    if path == root or root not in path.parents:
        if resume and (path / "run.json").is_file():
            return path
        raise ValueError(
            f"New experiment outputs must be under {root}. Use a run ID such as --output my_experiment."
        )
    return path
