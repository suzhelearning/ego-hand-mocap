"""Compatibility imports for the former standalone helper."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from mocap_pipeline.quality import sample_poses, choose_anchor
