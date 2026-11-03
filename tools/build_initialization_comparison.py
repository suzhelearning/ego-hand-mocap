"""Compatibility entry point; implementation lives in mocap_pipeline."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from mocap_pipeline.cli import main
if __name__ == '__main__':
    raise SystemExit(main(['export', *sys.argv[1:]]))
