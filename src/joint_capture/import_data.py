"""Safe, provenance-preserving imports into the three-file session layout."""
import json
import fcntl
import os
from pathlib import Path
import shutil
import tempfile

import h5py
import numpy as np

from .h5store import file_sha256, refresh_asset, write_version, publish_with_rollback
from .storage import save_json


def _source(path):
    path = Path(path)
    return {'source_name': path.name, 'source_sha256': file_sha256(path)}


def import_c3d(source, session_dir):
    session_dir = Path(session_dir)
    session_dir.mkdir(parents=True, exist_ok=True)
    with (session_dir / '.h5-write.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        return _import_c3d(source, session_dir)


def _import_c3d(source, session_dir):
    """Create mocap.h5 once; C3D frame times are relative, never wall time."""
    source, session_dir = Path(source).resolve(), Path(session_dir)
    destination = session_dir / 'mocap.h5'
    if destination.exists():
        raise FileExistsError(f'{destination} already exists; C3D replacement is not implicit')
    try:
        import ezc3d
    except ImportError as exc:
        raise RuntimeError('C3D import requires ezc3d in the active Python environment') from exc
    parsed = ezc3d.c3d(str(source))
    points = np.asarray(parsed['data']['points'])
    if points.ndim != 3 or points.shape[0] < 4:
        raise ValueError('C3D points must have shape (4, markers, frames)')
    positions = np.moveaxis(points[:3], (0, 1, 2), (2, 1, 0)).astype(float, copy=True)
    residuals = np.asarray(parsed['data']['meta_points']['residuals'])
    if residuals.shape != (1, points.shape[1], points.shape[2]):
        raise ValueError('C3D residuals must have shape (1, markers, frames)')
    residual = residuals[0].T
    valid = np.isfinite(positions).all(axis=2) & np.isfinite(residual) & (residual >= 0)
    positions[~valid] = np.nan
    point_parameters = parsed['parameters']['POINT']
    labels = list(point_parameters['LABELS']['value'])
    if len(labels) != positions.shape[1]:
        raise ValueError('C3D marker labels do not match point count')
    rate = float(point_parameters['RATE']['value'][0])
    if not np.isfinite(rate) or rate <= 0:
        raise ValueError('C3D point rate must be positive and finite')
    units = point_parameters.get('UNITS', {}).get('value', ['unknown'])[0]
    provenance = _source(source)
    session_dir.mkdir(parents=True, exist_ok=True)
    copied = session_dir / 'labeled.c3d'
    if copied.exists() and file_sha256(copied) != provenance['source_sha256']:
        raise FileExistsError(f'different C3D source already exists: {copied}')
    if source != copied.resolve() and not copied.exists():
        # Source copy is published without replacing an existing source.
        with source.open('rb') as inp, copied.open('xb') as out:
            shutil.copyfileobj(inp, out)
    fd, temporary = tempfile.mkstemp(prefix='.mocap-import-', suffix='.h5', dir=session_dir)
    os.close(fd)
    try:
        with h5py.File(temporary, 'w') as f:
            f.attrs.update(schema='joint_capture.mocap', schema_version=1, session_id=session_dir.name)
            raw = f.create_group('raw')
            raw.attrs.update(**provenance, source_original_name=source.name, source_copy=copied.relative_to(session_dir).as_posix())
            markers = raw.create_group('markers')
            markers.attrs.update(units=str(units), coordinate_frame='c3d_native')
            markers.create_dataset('positions', data=positions, compression='gzip')
            markers.create_dataset('valid', data=valid)
            markers.create_dataset('residual', data=residual)
            markers.create_dataset('labels', data=np.asarray(labels, dtype=object), dtype=h5py.string_dtype('utf-8'))
            frames = raw.create_group('frames')
            first = int(parsed['header']['points']['first_frame'])
            frames.create_dataset('index', data=np.arange(positions.shape[0], dtype=np.int64) + first)
            frames.create_dataset('time_s', data=np.arange(positions.shape[0], dtype=np.float64) / rate)
            frames.attrs.update(rate_hz=rate, time_basis='relative_to_first_c3d_frame', has_absolute_time_anchor=False)
            f.attrs['complete'] = True
        def update_manifest():
            refresh_asset(destination, 'mocap_h5')
            manifest = session_dir / 'session.json'
            if manifest.is_file():
                value = json.loads(manifest.read_text(encoding='utf-8'))
                value.setdefault('assets', {})['c3d_source'] = {
                    'path': copied.relative_to(session_dir).as_posix(),
                    'sha256': provenance['source_sha256'], 'size': copied.stat().st_size,
                }
                save_json(manifest, value)
        publish_with_rollback(destination, lambda: os.link(temporary, destination), update_manifest)
    finally:
        Path(temporary).unlink(missing_ok=True)
    return destination


def import_npz(source, session_dir, stage, version, *, units, coordinate_frame,
               arrays=None, mesh=None, quality_json=None):
    """Import selected numeric/string arrays without ever enabling NumPy pickle."""
    if stage not in ('mano', 'vision'):
        raise ValueError('stage must be mano or vision')
    if not str(units).strip() or not str(coordinate_frame).strip():
        raise ValueError('units and coordinate_frame must be explicitly provided')
    source, session_dir = Path(source), Path(session_dir)
    selected = {}
    skipped_keys = []
    with np.load(source, allow_pickle=False) as archive:
        keys = list(archive.files if arrays is None else arrays)
        if not keys:
            raise ValueError('no arrays selected')
        for key in keys:
            if key not in archive.files:
                raise ValueError(f'missing NPZ array: {key}')
            if not key or '/' in key or key in ('.', '..'):
                raise ValueError(f'unsafe NPZ key: {key}')
            try:
                value = archive[key]
            except ValueError as exc:
                if arrays is None and 'Object arrays' in str(exc):
                    skipped_keys.append(key)
                    continue
                raise
            if value.dtype.hasobject or value.dtype.kind not in 'biufcSU':
                raise ValueError(f'unsupported or object NPZ array: {key}')
            if value.dtype.kind in 'SU':
                value = value.astype(str)
                if any('\x00' in item for item in value.flat):
                    raise ValueError(f'NUL in string array: {key}')
            selected[key] = value
    if not selected:
        raise ValueError('no safe numeric/string arrays selected; object arrays are unsupported')
    provenance = _source(source)
    attachments = {}
    mesh_arrays = {}
    if mesh is not None and Path(mesh).suffix.lower() != '.npz':
        raise ValueError('mesh must be a numeric NPZ containing vertices and faces')
    if mesh is not None:
        with np.load(mesh, allow_pickle=False) as archive:
            for key in ('vertices', 'faces'):
                if key not in archive.files:
                    raise ValueError(f'mesh NPZ missing {key}')
                value = archive[key]
                if value.dtype.hasobject or value.dtype.kind not in 'iuf':
                    raise ValueError(f'mesh {key} must be numeric')
                if key == 'vertices' and (value.ndim not in (2, 3) or value.shape[-1] != 3):
                    raise ValueError('mesh vertices must have shape (V, 3) or (F, V, 3)')
                if key == 'faces' and (value.ndim != 2 or value.shape[1] != 3 or value.dtype.kind not in 'iu'):
                    raise ValueError('mesh faces must be integer array with shape (T, 3)')
                mesh_arrays[key] = value

    for name, path in [('quality_json', quality_json)]:
        if path is not None:
            path = Path(path)
            data = path.read_bytes()
            if name == 'quality_json':
                json.loads(data.decode('utf-8'))
            attachments[name] = (data, _source(path))

    def writer(group):
        group.attrs.update(units=units, coordinate_frame=coordinate_frame, skipped_keys=json.dumps(skipped_keys), **provenance)
        if mesh_arrays:
            mesh_group = group.create_group('mesh')
            mesh_group.attrs.update(**_source(mesh))
            for key, value in mesh_arrays.items():
                mesh_group.create_dataset(key, data=value)
        numeric = group.create_group('arrays')
        parameters = group.create_group('parameters')
        for key, value in selected.items():
            if value.dtype.kind in 'SU':
                parameters.create_dataset(key, data=value.astype(object), dtype=h5py.string_dtype('utf-8'))
            else:
                numeric.create_dataset(key, data=value)
        if attachments:
            assets = group.create_group('sources')
            for name, (data, metadata) in attachments.items():
                dataset = assets.create_dataset(name, data=np.frombuffer(data, dtype=np.uint8))
                dataset.attrs.update(**metadata)

    destination = session_dir / ('mocap.h5' if stage == 'mano' else 'ego.h5')
    if not destination.is_file():
        raise FileNotFoundError(f'raw session asset must exist before importing processing results: {destination}')
    write_version(destination, stage, version, writer, metadata=provenance, require_existing=True)
    return destination
