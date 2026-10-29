"""Canonical project paths for session-centered data and processing runs."""
from pathlib import Path
import re


_ID = re.compile(r"^[A-Za-z0-9_-]{1,128}$")


def checked_id(value: str, kind: str = "ID") -> str:
    if not isinstance(value, str) or not _ID.fullmatch(value):
        raise ValueError(f"{kind}只能包含字母、数字、下划线和横线，且必须以字母或数字开头")
    return value


class ProjectPaths:
    def __init__(self, root: Path):
        self.root = Path(root).resolve()

    @property
    def data(self) -> Path:
        return self.root / "data"

    def session(self, session_id: str) -> Path:
        return self.data / "sessions" / checked_id(session_id, "session-id")

    def raw_ego(self, session_id: str) -> Path:
        return self.session(session_id) / "raw" / "ego"

    def raw_mocap(self, session_id: str) -> Path:
        return self.session(session_id) / "raw" / "mocap"

    def clock_probes(self, session_id: str) -> Path:
        return self.session(session_id) / "raw" / "clocks"

    def capture_records(self, session_id: str) -> Path:
        return self.session(session_id) / "capture"

    def runs(self, session_id: str) -> Path:
        return self.session(session_id) / "runs"

    def run(self, session_id: str, run_id: str) -> Path:
        return self.runs(session_id) / checked_id(run_id, "run-id")

    def relative_asset(self, session_id: str, value: str) -> Path:
        """Resolve a session-relative asset, rejecting traversal and symlink escape."""
        base = self.session(session_id).resolve()
        candidate = (base / value).resolve()
        if not candidate.is_relative_to(base):
            raise ValueError("会话资产路径越界")
        return candidate

