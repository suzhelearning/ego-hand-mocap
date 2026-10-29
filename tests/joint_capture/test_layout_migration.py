import json
import pytest
from joint_capture import migration


def legacy_project(root):
    capture = root / 'capture_sessions/recording'
    capture.mkdir(parents=True)
    (capture / 'session.json').write_text(json.dumps({
        'session_id': 'recording', 'status': 'complete',
        'ego': {'h5': '/old/head.h5'}, 'mocap': {}}))
    ego = root / 'data/ego_data/recording'
    ego.mkdir(parents=True)
    (ego / 'head.h5').write_bytes(b'fixture raw bytes')
    return migration.build_migration_plan(root, 'test-migration')


def test_first_migration_and_repeat_preserve_sources(tmp_path):
    plan = legacy_project(tmp_path)
    migration.apply_migration_plan(tmp_path, plan)
    base = tmp_path / 'data/sessions/recording'
    manifest = json.loads((base / 'session.json').read_text())
    assert manifest['assets']['ego_h5']['path'] == 'raw/ego/head.h5'
    before = (base / 'session.json').read_bytes()
    report = migration.apply_migration_plan(tmp_path, plan)
    assert all(item['status'] == 'already_identical' for item in report['applied'])
    assert (base / 'session.json').read_bytes() == before
    for entry in plan['entries']:
        assert (tmp_path / entry['source']).read_bytes() == (tmp_path / entry['destination']).read_bytes()


def test_interrupted_session_is_hidden_and_resumable(tmp_path, monkeypatch):
    plan = legacy_project(tmp_path)
    original = migration.shutil.copy2
    count = 0
    def interrupted(source, target):
        nonlocal count
        count += 1
        if count == 2:
            raise OSError('simulated interruption')
        return original(source, target)
    monkeypatch.setattr(migration.shutil, 'copy2', interrupted)
    with pytest.raises(OSError, match='simulated interruption'):
        migration.apply_migration_plan(tmp_path, plan)
    assert not (tmp_path / 'data/sessions/recording').exists()
    monkeypatch.setattr(migration.shutil, 'copy2', original)
    report = migration.apply_migration_plan(tmp_path, plan)
    assert any(item['status'] == 'staged_resume' for item in report['applied'])
    assert (tmp_path / 'data/sessions/recording/session.json').is_file()


def test_conflicting_destination_is_never_overwritten(tmp_path):
    plan = legacy_project(tmp_path)
    migration.apply_migration_plan(tmp_path, plan)
    target = tmp_path / 'data/sessions/recording/raw/ego/head.h5'
    target.write_bytes(b'user modification')
    with pytest.raises(RuntimeError, match='conflicts'):
        migration.apply_migration_plan(tmp_path, plan)
    assert target.read_bytes() == b'user modification'
