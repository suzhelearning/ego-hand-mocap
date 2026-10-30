import json
import h5py
import pytest
from joint_capture.flatten import build_flatten_plan, apply_flatten_plan
from joint_capture.h5store import file_sha256


def fixture_session(tmp_path):
    base = tmp_path / 'data/sessions/s1'
    (base / 'raw/ego/video').mkdir(parents=True)
    (base / 'raw/mocap').mkdir(parents=True)
    with h5py.File(base / 'raw/ego/source.h5', 'w') as f:
        f['sensor/raw'] = [1, 2]
    with h5py.File(base / 'raw/mocap/natnet.h5', 'w') as f:
        f['frames'] = [3]
    (base / 'raw/ego/video/a.mp4').write_bytes(b'video')
    (base / 'raw/ego/session.json').write_text('ego metadata')
    (base / 'runs/r1/aligned').mkdir(parents=True)
    with h5py.File(base / 'runs/r1/aligned/aligned.h5', 'w') as f:
        f['cameras/left/t'] = [4]
        f.attrs['complete'] = False
    (base / 'runs/r1/sync').mkdir()
    (base / 'runs/r1/sync/x.csv').write_bytes(b'a,b\n1,2\n')
    (base / 'runs/okvis1/ego/okvis').mkdir(parents=True)
    with h5py.File(base / 'runs/okvis1/ego/okvis/pose.h5', 'w') as f:
        f['trajectory/t'] = [5]
    (base / 'runs/okvis1/ego/okvis/frame.png').write_bytes(bytes(range(255)))
    digest = file_sha256(base / 'raw/ego/source.h5')
    (base / 'session.json').write_text(json.dumps({'session_id': 's1', 'assets': {'ego_h5': {'path': 'raw/ego/source.h5', 'sha256': digest}}}))
    return base, digest


def test_lossless_flatten_and_idempotent(tmp_path):
    base, digest = fixture_session(tmp_path)
    source_manifest = (base / 'session.json').read_bytes()
    before = {str(p.relative_to(base)): p.read_bytes() for p in base.rglob('*') if p.is_file()}
    plan = build_flatten_plan(tmp_path)
    assert not plan['conflicts']
    assert not (base / 'ego.h5').exists()
    apply_flatten_plan(tmp_path, plan)
    assert not (base / 'raw').exists()
    assert not (base / 'runs').exists()
    assert (base / 'capture/session.before_flatten.json').read_bytes() == source_manifest
    assert (base / 'capture/ego_export/session.json').read_text() == 'ego metadata'
    for e in plan['sessions'][0]['archives']:
        with h5py.File(base / e['target']) as f:
            ds = f[f'processing/{e["stage"]}/{e["run"]}/artifacts/{e["local"]}']
            if e['source'].endswith('.h5'):
                assert ds.attrs['storage'] == 'semantic_hdf5_copy'
                assert ds.size == 0
            else:
                assert ds[:].tobytes() == before[e['source']]
    with h5py.File(base / 'ego.h5') as f:
        assert f['sensor/raw'][:].tolist() == [1, 2]
        assert f['processing/okvis/okvis1/trajectory/t'][0] == 5
    with h5py.File(base / 'aligned.h5') as f:
        assert f['processing/alignment/r1/cameras/left/t'][0] == 4
        assert not f['processing/alignment/r1'].attrs['source_complete']
        assert f.attrs['schema'] == 'ego-session-aligned-v2'
    manifest = json.loads((base / 'session.json').read_text())
    assert manifest['layout_version'] == 2
    assert manifest['assets']['ego_h5']['original_sha256'] == digest
    assert manifest['assets']['ego_h5']['sha256'] == file_sha256(base / 'ego.h5')
    assert build_flatten_plan(tmp_path)['sessions'] == []


def test_conflict_leaves_everything_intact(tmp_path):
    base, _ = fixture_session(tmp_path)
    (base / 'ego.h5').write_bytes(b'conflict')
    # Existing conflicting destination need not be a readable HDF5.
    with pytest.raises((ValueError, OSError)):
        apply_flatten_plan(tmp_path, build_flatten_plan(tmp_path))
    assert (base / 'raw/ego/source.h5').exists()
    assert (base / 'runs/r1/aligned/aligned.h5').exists()


def test_symlink_escape_is_rejected(tmp_path):
    base, _ = fixture_session(tmp_path)
    external = tmp_path.parent / (tmp_path.name + '-outside')
    external.write_bytes(b'keep')
    (base / 'raw/ego/escape').symlink_to(external)
    with pytest.raises(ValueError, match='unsafe'):
        build_flatten_plan(tmp_path)
    assert external.read_bytes() == b'keep'


def test_changed_source_preflight_no_mutation(tmp_path):
    base, _ = fixture_session(tmp_path)
    plan = build_flatten_plan(tmp_path)
    (base / 'runs/r1/sync/x.csv').write_text('changed')
    with pytest.raises(ValueError, match='source changed'):
        apply_flatten_plan(tmp_path, plan)
    assert not (base / 'ego.h5').exists()


def test_resume_after_archive_publish_before_source_removal(tmp_path, monkeypatch):
    import joint_capture.flatten as flatten
    base, digest = fixture_session(tmp_path)
    real = flatten.write_version
    def interrupted(*args, **kwargs):
        real(*args, **kwargs)
        raise RuntimeError('interrupted after publish')
    monkeypatch.setattr(flatten, 'write_version', interrupted)
    with pytest.raises(RuntimeError, match='interrupted'):
        apply_flatten_plan(tmp_path, build_flatten_plan(tmp_path))
    assert (base / 'runs/r1/aligned/aligned.h5').exists()
    monkeypatch.setattr(flatten, 'write_version', real)
    resumed = build_flatten_plan(tmp_path)
    assert not resumed['conflicts']
    apply_flatten_plan(tmp_path, resumed)
    manifest = json.loads((base / 'session.json').read_text())
    assert manifest['assets']['ego_h5']['original_sha256'] == digest
    assert not (base / 'runs').exists()


def test_archive_corruption_blocks_source_deletion(tmp_path, monkeypatch):
    import joint_capture.flatten as flatten
    base, _ = fixture_session(tmp_path)
    real = flatten.write_version
    def interrupted(*args, **kwargs):
        real(*args, **kwargs)
        raise RuntimeError('interrupt')
    monkeypatch.setattr(flatten, 'write_version', interrupted)
    with pytest.raises(RuntimeError):
        apply_flatten_plan(tmp_path, build_flatten_plan(tmp_path))
    with h5py.File(base / 'ego.h5', 'a') as f:
        f['processing/okvis/okvis1/trajectory/t'][0] = 999
    plan = build_flatten_plan(tmp_path)
    assert plan['conflicts']
    with pytest.raises(ValueError, match='conflicts'):
        apply_flatten_plan(tmp_path, plan)
    assert (base / 'runs/r1/aligned/aligned.h5').exists()
