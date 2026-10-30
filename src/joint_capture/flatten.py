"""Lossless, resumable conversion of session raw/runs trees to canonical HDF5."""
import fcntl
from contextlib import ExitStack
import hashlib
import json
from pathlib import Path
import shutil
import h5py
import numpy as np
from .h5store import file_sha256, write_version
from .storage import save_json


def _safe(base, path):
    if path.is_symlink() or not path.resolve().is_relative_to(base.resolve()):
        raise ValueError(f'unsafe migration path: {path}')
    return path


def _files(base, folder):
    for p in sorted(folder.rglob('*')):
        _safe(base, p)
        if p.is_file():
            yield p


def _destination(relative, ego_h5):
    parts = relative.parts
    if relative == ego_h5:
        return Path('ego.h5')
    if parts[:3] == ('raw', 'ego', 'video'):
        return Path('video', *parts[3:])
    if parts[:2] == ('raw', 'ego'):
        return Path('capture', 'ego_export', *parts[2:])
    if relative == Path('raw/mocap/natnet.h5'):
        return Path('mocap.h5')
    if parts[:2] == ('raw', 'clocks'):
        return Path('clocks', *parts[2:])
    if relative.suffix.lower() == '.c3d':
        return Path(relative.name)
    return Path('capture', 'raw_other', *parts[1:])


def build_flatten_plan(root, session_ids=None):
    """Read-only preflight; all paths and SHA256 values are project relative."""
    root = Path(root).resolve()
    sessions = root / 'data/sessions'
    selected = [sessions / s for s in session_ids] if session_ids is not None else sorted(sessions.glob('*'))
    plan = {'layout_version': 2, 'sessions': [], 'conflicts': []}
    for base in selected:
        _safe(root, base)
        if not base.is_dir():
            continue
        if not (base / 'session.json').is_file():
            plan['conflicts'].append(f'{base.name}: missing session.json')
            continue
        _safe(root, base / 'session.json')
        raw = list(_files(base, base / 'raw'))
        runs = list(_files(base, base / 'runs'))
        ego = [p.relative_to(base) for p in raw if p.suffix.lower() in ('.h5', '.hdf5') and p.relative_to(base).parts[:2] == ('raw', 'ego')]
        if len(ego) > 1:
            plan['conflicts'].append(f'{base.name}: multiple ego HDF5 sources')
            continue
        item = {'session': base.relative_to(root).as_posix(), 'moves': [], 'archives': []}
        targets = {}
        for src in raw:
            rel = src.relative_to(base)
            dst = _safe(base, base / _destination(rel, ego[0] if ego else None))
            digest = file_sha256(src)
            if dst.exists() and (not dst.is_file() or file_sha256(dst) != digest):
                plan['conflicts'].append(f'{dst}: destination differs')
            if str(dst) in targets and targets[str(dst)] != digest:
                plan['conflicts'].append(f'{dst}: destination collision')
            targets[str(dst)] = digest
            item['moves'].append({'source': str(rel), 'destination': str(dst.relative_to(base)), 'sha256': digest})
        for src in runs:
            rel = src.relative_to(base / 'runs')
            if len(rel.parts) < 2:
                plan['conflicts'].append(f'{src}: unscoped run file')
                continue
            run, local = rel.parts[0], Path(*rel.parts[1:])
            stage = 'okvis' if ('okvis' in local.parts or 'okvis' in run) else ('alignment' if local.parts[0] == 'aligned' else 'sync')
            target = 'ego.h5' if stage == 'okvis' else 'aligned.h5'
            _safe(base, base / target)
            entry = {'source': str(Path('runs') / rel), 'local': str(local), 'run': run, 'stage': stage, 'target': target, 'sha256': file_sha256(src)}
            if (base / target).exists() and str(base / target) not in targets:
                with h5py.File(base / target, 'r') as f:
                    key = f'processing/{stage}/{run}'
                    if key in f and not _archived(f[key], entry):
                        plan['conflicts'].append(f'{base / target}::{key}: archive conflict')
            item['archives'].append(entry)
        grouped = {}
        for entry in item['archives']:
            if Path(entry['source']).suffix.lower() in ('.h5', '.hdf5'):
                key = (entry['target'], entry['stage'], entry['run'])
                grouped.setdefault(key, []).append(entry)
                with h5py.File(base / entry['source'], 'r') as f:
                    if 'artifacts' in f:
                        plan['conflicts'].append(f'{entry["source"]}: reserved artifacts group exists')
        for key, values in grouped.items():
            if len(values) > 1:
                plan['conflicts'].append(f'{key}: multiple HDF5 sources')
        if raw or runs or (base / 'raw').exists() or (base / 'runs').exists():
            plan['sessions'].append(item)
    return plan


def _semantic_digest(group, names):
    digest = hashlib.sha256()
    def value(v):
        a = np.asarray(v)
        digest.update(str(a.shape).encode())
        digest.update(str(a.dtype).encode())
        if a.dtype.kind == 'O':
            for x in a.flat:
                raw = x if isinstance(x, bytes) else str(x).encode()
                digest.update(len(raw).to_bytes(8, 'big'))
                digest.update(raw)
        else:
            digest.update(a.tobytes())
    def visit(name, obj):
        digest.update(name.encode())
        for attr in sorted(obj.attrs):
            digest.update(attr.encode())
            value(obj.attrs[attr])
        if isinstance(obj, h5py.Dataset):
            digest.update(str(obj.shape).encode())
            digest.update(str(obj.dtype).encode())
            if obj.shape is None:
                return
            if obj.ndim == 0:
                value(obj[()])
            else:
                for offset in range(0, len(obj), 1024):
                    value(obj[offset:offset + 1024])
        else:
            for child in sorted(obj):
                link = obj.get(child, getlink=True)
                if not isinstance(link, h5py.HardLink):
                    raise ValueError('historical HDF5 contains nonlocal links')
                visit(name + '/' + child, obj[child])
    for name in sorted(names):
        visit(name, group[name])
    return digest.hexdigest()


def _archived(group, entry):
    key = 'artifacts/' + entry['local']
    if key not in group or not isinstance(group[key], h5py.Dataset):
        return False
    ds = group[key]
    if 'semantic_sha256' in ds.attrs:
        names = json.loads(ds.attrs['root_names'])
        return (ds.attrs.get('sha256') == entry['sha256'] and
                _semantic_digest(group, names) == ds.attrs['semantic_sha256'])
    digest = hashlib.sha256()
    for offset in range(0, ds.size, 1024 * 1024):
        digest.update(ds[offset:offset + 1024 * 1024].tobytes())
    return digest.hexdigest() == entry['sha256']


def _archive(group, base, entries):
    group.attrs['history_execution_status'] = 'unknown'
    group.attrs['status_basis'] = 'historical_artifacts_only'
    group.attrs['source_execution_state'] = 'unknown'
    h5_sources = [e for e in entries if Path(e['source']).suffix.lower() in ('.h5', '.hdf5')]
    if len(h5_sources) > 1:
        raise ValueError('multiple HDF5 inputs in one historical stage')
    for entry in h5_sources:
        with h5py.File(base / entry['source'], 'r') as source:
            for key in source:
                source.copy(key, group, name=key)
            for key, value in source.attrs.items():
                group.attrs['source_' + key] = value
            group.attrs['source_complete'] = bool(source.attrs.get('complete', False))
    for entry in entries:
        src = _safe(base, base / entry['source'])
        if entry in h5_sources:
            with h5py.File(src, 'r') as source:
                names = list(source)
                fingerprint = _semantic_digest(source, names)
                if _semantic_digest(group, names) != fingerprint:
                    raise OSError(f'HDF5 semantic verification failed: {src}')
            ds = group.create_dataset('artifacts/' + entry['local'], shape=(0,), dtype='u1')
            ds.attrs['sha256'] = entry['sha256']
            ds.attrs['semantic_sha256'] = fingerprint
            ds.attrs['root_names'] = json.dumps(names)
            ds.attrs['storage'] = 'semantic_hdf5_copy'
            continue
        ds = group.create_dataset('artifacts/' + entry['local'], shape=(src.stat().st_size,), dtype='u1', **({'chunks': True} if src.stat().st_size else {}))
        ds.attrs['sha256'] = entry['sha256']
        with src.open('rb') as handle:
            offset = 0
            while chunk := handle.read(1024 * 1024):
                ds[offset:offset + len(chunk)] = np.frombuffer(chunk, dtype=np.uint8)
                offset += len(chunk)
        if not _archived(group, entry):
            raise OSError(f'archive verification failed: {src}')


def _apply_flatten_plan(root, plan):
    root = Path(root).resolve()
    if plan.get('conflicts'):
        raise ValueError('migration conflicts: ' + '; '.join(plan['conflicts']))
    # Validate the entire plan before the first write.
    for item in plan['sessions']:
        base = _safe(root, root / item['session'])
        for entry in item['moves'] + item['archives']:
            src = _safe(base, base / entry['source'])
            if file_sha256(src) != entry['sha256']:
                raise ValueError(f'migration source changed: {src}')
        for entry in item['moves']:
            dst = _safe(base, base / entry['destination'])
            if dst.exists() and file_sha256(dst) != entry['sha256']:
                raise ValueError(f'migration destination changed: {dst}')
    report_dir = _safe(root, root / 'reports/flatten')
    report_dir.mkdir(parents=True, exist_ok=True)
    save_json(report_dir / 'plan.json', plan)
    result = {'layout_version': 2, 'sessions': []}
    for item in plan['sessions']:
        base = root / item['session']
        manifest_path = base / 'session.json'
        backup = _safe(base, base / 'capture/session.before_flatten.json')
        backup.parent.mkdir(parents=True, exist_ok=True)
        if not backup.exists():
            shutil.copy2(manifest_path, backup)
        # The backup also recovers path rewrites after interrupted raw moves.
        original = json.loads(backup.read_text())
        ego_original = next((Path(a['path']) for a in original.get('assets', {}).values() if isinstance(a, dict) and str(a.get('path', '')).startswith('raw/ego/') and Path(a['path']).suffix.lower() in ('.h5', '.hdf5')), None)
        for entry in item['moves']:
            src, dst = base / entry['source'], base / entry['destination']
            dst.parent.mkdir(parents=True, exist_ok=True)
            if not dst.exists():
                src.rename(dst)
            if file_sha256(dst) != entry['sha256']:
                raise OSError(f'move verification failed: {dst}')
            if src.exists():
                if file_sha256(src) != entry['sha256']:
                    raise OSError(f'source changed: {src}')
                src.unlink()
        groups = {}
        for e in item['archives']:
            groups.setdefault((e['target'], e['stage'], e['run']), []).append(e)
        for (target, stage, run), entries in groups.items():
            path = _safe(base, base / target)
            key = f'processing/{stage}/{run}'
            exists = False
            if path.exists():
                with h5py.File(path, 'r') as f:
                    exists = key in f
            if not exists:
                write_version(path, stage, run, lambda g: _archive(g, base, entries), metadata={'origin': 'historical_artifacts', 'execution_status': 'unknown'})
            with h5py.File(path, 'r') as f:
                for e in entries:
                    if not _archived(f[key], e):
                        raise OSError(f'archive verification failed: {e["source"]}')
            for e in entries:
                src = _safe(base, base / e['source'])
                if file_sha256(src) != e['sha256']:
                    raise OSError(f'source changed: {src}')
                src.unlink()
        manifest = json.loads(manifest_path.read_text())
        for name, asset in original.get('assets', {}).items():
            if not isinstance(asset, dict) or not str(asset.get('path', '')).startswith('raw/'):
                continue
            destination = _destination(Path(asset['path']), ego_original)
            path = _safe(base, base / destination)
            if path.is_file():
                manifest.setdefault('assets', {})[name] = dict(asset, path=str(destination), original_sha256=asset.get('original_sha256', asset.get('sha256', file_sha256(path))), sha256=file_sha256(path), size=path.stat().st_size)
        for name in ('ego', 'mocap', 'aligned'):
            path = base / (name + '.h5')
            if path.is_file():
                asset = manifest.setdefault('assets', {}).setdefault(name + '_h5', {})
                asset.setdefault('original_sha256', asset.get('sha256', file_sha256(path)))
                asset.update(path=path.name, sha256=file_sha256(path), size=path.stat().st_size)
        if (base / 'ego.h5').exists():
            manifest.setdefault('ego', {}).update(h5='ego.h5', local_directory='.')
        if (base / 'mocap.h5').exists():
            manifest.setdefault('mocap', {})['file'] = 'mocap.h5'
        for probe in manifest.get('sync',{}).values():
            if isinstance(probe,dict) and probe.get('path'):
                previous=Path(probe['path'])
                if previous.parent.name=='clocks' and (base/'clocks'/previous.name).exists():probe['path']=str((base/'clocks'/previous.name).resolve())
        manifest['layout_version'] = 2
        save_json(manifest_path, manifest)
        for tree in (base / 'raw', base / 'runs'):
            if tree.exists():
                for directory in sorted((p for p in tree.rglob('*') if p.is_dir()), key=lambda p: len(p.parts), reverse=True):
                    directory.rmdir()
                tree.rmdir()
        result['sessions'].append(item['session'])
        save_json(report_dir / 'results.json', result)
    return result


def apply_flatten_plan(root, plan):
    root=Path(root).resolve()
    with ExitStack() as stack:
        for folder in (root/'data/sessions',root/'capture_sessions'):
            folder.mkdir(parents=True,exist_ok=True)
            lock=stack.enter_context((folder/'.capture.lock').open('a'))
            try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:raise RuntimeError('采集正在运行，禁止迁移') from None
        return _apply_flatten_plan(root,plan)
