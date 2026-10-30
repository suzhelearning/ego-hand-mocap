"""Versioned HDF5 processing results, published atomically under one writer lock."""
from datetime import datetime, timezone
from pathlib import Path
import fcntl
import hashlib
import json
import os
import shutil
import tempfile
import h5py
from .paths import checked_id
from .storage import save_json


def file_sha256(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()


def refresh_asset(path, asset_id=None):
    path=Path(path).resolve();manifest=path.parent/'session.json'
    if not manifest.is_file():return
    asset_id=asset_id or {'ego.h5':'ego_h5','mocap.h5':'mocap_h5','aligned.h5':'aligned_h5'}.get(path.name)
    if not asset_id:raise ValueError('asset_id required for noncanonical file')
    d=json.loads(manifest.read_text());previous=d.setdefault('assets',{}).get(asset_id,{})
    value=dict(previous,path=path.name,sha256=file_sha256(path),size=path.stat().st_size)
    value.setdefault('original_sha256',previous.get('sha256',value['sha256']))
    d['assets'][asset_id]=value
    d['layout_version']=2
    save_json(manifest,d)


def publish_with_rollback(path, publish, update_manifest):
    """Publish under the caller's H5 writer lock, restoring both files on error.

    Callbacks must replace files atomically, never mutate the original inodes.
    This handles Python exceptions; it is not a power-loss transaction journal.
    """
    path = Path(path)
    backups = {}
    published = False
    committed = False
    try:
        # Hard links preserve old bytes without copying large raw H5 data again.
        for original in (path, path.parent / 'session.json'):
            if original.is_file():
                fd, name = tempfile.mkstemp(prefix='.' + original.name + '.', suffix='.rollback', dir=path.parent)
                os.close(fd)
                backup = Path(name)
                backup.unlink()
                os.link(original, backup)
                backups[original] = backup
        publish()
        published = True
        update_manifest()
        committed = True
    except BaseException:
        if published:
            if path not in backups:
                path.unlink(missing_ok=True)
            for original, backup in backups.items():
                os.replace(backup, original)
            published = False
        raise
    finally:
        # Preserve recovery links if rollback itself fails.
        if not published or committed:
            for backup in backups.values():
                backup.unlink(missing_ok=True)


def write_version(path, stage, version, writer, metadata=None, require_existing=False):
    path=Path(path).resolve();stage=checked_id(stage,'stage');version=checked_id(version,'version')
    path.parent.mkdir(parents=True,exist_ok=True)
    # A persistent lock inode coordinates all canonical H5 writes and manifest updates.
    with (path.parent/'.h5-write.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        if require_existing and not path.is_file():raise FileNotFoundError(path)
        original_hash=file_sha256(path) if path.exists() else None
        fd,tmp=tempfile.mkstemp(prefix='.'+path.name+'.',suffix='.tmp',dir=path.parent)
        os.close(fd);temporary=Path(tmp)
        try:
            if path.exists():shutil.copy2(path,temporary)
            with h5py.File(temporary,'a' if path.exists() else 'w') as f:
                key=f'processing/{stage}/{version}'
                if key in f:raise FileExistsError(f'{path}::{key}')
                if path.name=='aligned.h5':f.attrs['schema']='ego-session-aligned-v2'
                g=f.create_group(key);g.attrs['complete']=False
                g.attrs['created_at']=datetime.now(timezone.utc).isoformat()
                g.attrs['input_file_sha256']=original_hash or ''
                g.attrs['provenance_json']=json.dumps(metadata or {},ensure_ascii=False)
                writer(g)
                g.attrs['complete']=True
                f.flush()
            with temporary.open('rb') as f:os.fsync(f.fileno())
            publish_with_rollback(path, lambda: os.replace(temporary, path), lambda: refresh_asset(path))
        finally:
            temporary.unlink(missing_ok=True)
    return f'/processing/{stage}/{version}'


def resolve_version(file, stage, version=None):
    root=file.get('processing/'+stage)
    if root is None:raise KeyError('缺少处理阶段: '+stage)
    completed=[name for name,g in root.items() if isinstance(g,h5py.Group) and bool(g.attrs.get('complete',False))]
    if version is None:
        if len(completed)!=1:raise ValueError(f'{stage}有{len(completed)}个完成版本，请显式指定版本')
        version=completed[0]
    if version not in completed:raise ValueError(f'处理版本不存在或未完成: {stage}/{version}')
    return root[version]
