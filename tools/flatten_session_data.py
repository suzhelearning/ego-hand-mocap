#!/usr/bin/env python3
"""Dry-run by default; --apply archives and verifies before removing old paths."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from joint_capture.flatten import build_flatten_plan, apply_flatten_plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--session', action='append', dest='sessions')
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    plan = build_flatten_plan(args.root, args.sessions)
    print(json.dumps(apply_flatten_plan(args.root, plan) if args.apply else plan, ensure_ascii=False, indent=2))
    return 1 if plan['conflicts'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
