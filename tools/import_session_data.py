#!/usr/bin/env python3
"""Import external C3D/MANO/vision results into canonical session HDF5 assets."""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from joint_capture.import_data import import_c3d, import_npz


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='kind', required=True)
    for kind in ('c3d', 'mano', 'vision'):
        command = commands.add_parser(kind)
        command.add_argument('--session-dir', type=Path, required=True)
        command.add_argument('--source', type=Path, required=True)
        if kind != 'c3d':
            command.add_argument('--version', required=True)
            command.add_argument('--units', required=True, help='Explicit source coordinate units (no conversion)')
            command.add_argument('--coordinate-frame', required=True)
            command.add_argument('--array', action='append', dest='arrays')
            command.add_argument('--mesh', type=Path, help='NPZ vertices (V,3) or (F,V,3), integer faces (T,3)')
            command.add_argument('--quality-json', type=Path)
    args = parser.parse_args(argv)
    try:
        if args.kind == 'c3d':
            result = import_c3d(args.source, args.session_dir)
        else:
            result = import_npz(args.source, args.session_dir, args.kind, args.version,
                                units=args.units, coordinate_frame=args.coordinate_frame,
                                arrays=args.arrays, mesh=args.mesh, quality_json=args.quality_json)
    except (ValueError, OSError, RuntimeError, KeyError) as exc:
        parser.exit(2, f'Import failed: {exc}\n')
    print(result)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
