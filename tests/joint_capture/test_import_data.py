import sys
from types import SimpleNamespace

import h5py
import numpy as np
import pytest

from joint_capture.import_data import import_c3d, import_npz


def test_c3d_preserves_nan_and_relative_time(tmp_path, monkeypatch):
    source = tmp_path / 'input.c3d'
    source.write_bytes(b'fixture')
    points = np.ones((4, 2, 3))
    points[0, 1, 1] = np.nan
    residuals = np.ones((1, 2, 3))
    residuals[0, 0, 2] = -1
    parsed = {'data': {'points': points, 'meta_points': {'residuals': residuals}}, 'parameters': {'POINT': {
        'LABELS': {'value': ['a', 'b']}, 'UNITS': {'value': ['mm']},
        'RATE': {'value': [100.0]}}}, 'header': {'points': {'first_frame': 10}}}
    monkeypatch.setitem(sys.modules, 'ezc3d', SimpleNamespace(c3d=lambda _: parsed))
    session = tmp_path / 'session'
    path = import_c3d(source, session)
    with h5py.File(path) as f:
        assert f['raw/markers/positions'].shape == (3, 2, 3)
        assert np.isnan(f['raw/markers/positions'][1, 1, 0])
        assert not f['raw/markers/valid'][1, 1]
        assert not f['raw/markers/valid'][2, 0]
        assert np.isnan(f['raw/markers/positions'][2, 0]).all()
        np.testing.assert_array_equal(f['raw/frames/index'][:], [10, 11, 12])
        np.testing.assert_allclose(f['raw/frames/time_s'][:], [0, .01, .02])
        assert not f['raw/frames'].attrs['has_absolute_time_anchor']
    assert (session / 'labeled.c3d').read_bytes() == b'fixture'
    assert not (session / 'sources').exists()
    reused = tmp_path / 'reused'
    reused.mkdir()
    (reused / 'labeled.c3d').write_bytes(b'fixture')
    import_c3d(reused / 'labeled.c3d', reused)
    conflict = tmp_path / 'conflict'
    conflict.mkdir()
    (conflict / 'labeled.c3d').write_bytes(b'other')
    with pytest.raises(FileExistsError, match='different C3D'):
        import_c3d(source, conflict)
    assert not (conflict / 'mocap.h5').exists()
    with pytest.raises(FileExistsError):
        import_c3d(source, session)


def test_npz_version_and_safe_parameters(tmp_path):
    _raw_files(tmp_path / 'session')
    source = tmp_path / 'fit.npz'
    np.savez(source, joints=np.zeros((3, 21, 3)), model=np.array('mano'))
    path = import_npz(source, tmp_path / 'session', 'mano', 'v1', units='mm', coordinate_frame='mocap')
    with h5py.File(path) as f:
        g = f['processing/mano/v1']
        assert g['arrays/joints'].shape == (3, 21, 3)
        assert g['parameters/model'].asstr()[()] == 'mano'
        assert g.attrs['units'] == 'mm'
        np.testing.assert_array_equal(f['raw/fixture'][:], [123])


def test_npz_rejects_pickle_without_publishing(tmp_path):
    source = tmp_path / 'bad.npz'
    np.savez(source, unsafe=np.array([{'exec': 'no'}], dtype=object))
    with pytest.raises(ValueError, match='object|Object'):
        import_npz(source, tmp_path / 'session', 'vision', 'v1', units='m', coordinate_frame='camera')
    assert not (tmp_path / 'session' / 'ego.h5').exists()


def test_npz_explicit_selection_and_unknown_key(tmp_path):
    _raw_files(tmp_path / 'session')
    source = tmp_path / 'mixed.npz'
    np.savez(source, joints=np.zeros((1, 21, 3)), unsafe=np.array([{}], dtype=object))
    import_npz(source, tmp_path / 'session', 'vision', 'v1', units='m', coordinate_frame='camera', arrays=['joints'])
    with pytest.raises(ValueError, match='missing'):
        import_npz(source, tmp_path / 'other', 'vision', 'v1', units='m', coordinate_frame='camera', arrays=['absent'])


def test_default_skips_object_and_imports_mesh_and_quality(tmp_path):
    import json
    _raw_files(tmp_path / 'session')
    source = tmp_path / 'fit.npz'
    np.savez(source, joints=np.zeros((2, 21, 3)), config=np.array({'unsafe': True}, dtype=object))
    mesh = tmp_path / 'mesh.npz'
    np.savez(mesh, vertices=np.zeros((2, 778, 3)), faces=np.array([[0, 1, 2]]))
    quality = tmp_path / 'quality.json'
    quality.write_text('{"rmse": 0.1}')
    path = import_npz(source, tmp_path / 'session', 'mano', 'v1', units='mm', coordinate_frame='c3d_native', mesh=mesh, quality_json=quality)
    with h5py.File(path) as f:
        g = f['processing/mano/v1']
        assert json.loads(g.attrs['skipped_keys']) == ['config']
        assert g['mesh/vertices'].shape == (2, 778, 3)
        assert 'sources/mesh' not in g
        assert g['mesh/faces'].dtype.kind == 'i'
        assert bytes(g['sources/quality_json'][:]).decode() == quality.read_text()
    with pytest.raises(ValueError, match='Object'):
        import_npz(source, tmp_path / 'session', 'mano', 'v2', units='mm', coordinate_frame='c3d_native', arrays=['config'])
    with h5py.File(path) as f:
        assert 'processing/mano/v2' not in f


def test_cli_error_is_concise_and_does_not_publish(tmp_path):
    import subprocess
    from pathlib import Path
    source = tmp_path / 'bad.npz'
    np.savez(source, unsafe=np.array([{}], dtype=object))
    command = Path(__file__).resolve().parents[2] / 'tools' / 'import_session_data.py'
    result = subprocess.run([sys.executable, str(command), 'vision', '--session-dir', str(tmp_path / 'session'), '--source', str(source), '--version', 'v1', '--units', 'm', '--coordinate-frame', 'camera'], text=True, capture_output=True)
    assert result.returncode == 2
    assert 'Import failed:' in result.stderr
    assert 'Traceback' not in result.stderr
    assert not (tmp_path / 'session' / 'ego.h5').exists()


def _raw_files(session):
    session.mkdir(parents=True, exist_ok=True)
    for name in ('ego.h5', 'mocap.h5'):
        with h5py.File(session / name, 'w') as f:
            f.create_dataset('raw/fixture', data=[123])


def test_npz_requires_existing_raw_asset(tmp_path):
    source = tmp_path / 'valid.npz'
    np.savez(source, joints=np.zeros((1, 21, 3)))
    with pytest.raises(FileNotFoundError):
        import_npz(source, tmp_path / 'session', 'mano', 'v1', units='mm', coordinate_frame='mocap')
    assert not (tmp_path / 'session' / 'mocap.h5').exists()


def test_mesh_rejects_float_faces(tmp_path):
    _raw_files(tmp_path / 'session')
    source = tmp_path / 'valid.npz'
    np.savez(source, joints=np.zeros((1, 21, 3)))
    mesh = tmp_path / 'mesh.npz'
    np.savez(mesh, vertices=np.zeros((2, 778, 3)), faces=np.zeros((3, 3), dtype=float))
    with pytest.raises(ValueError, match='integer'):
        import_npz(source, tmp_path / 'session', 'mano', 'v1', units='mm', coordinate_frame='mocap', mesh=mesh)


@pytest.mark.parametrize('failed_save', [1, 2])
@pytest.mark.parametrize('after_save', [False, True])
def test_c3d_manifest_failure_rolls_back_and_allows_retry(tmp_path, monkeypatch, failed_save, after_save):
    import json
    import joint_capture.h5store as store
    import joint_capture.import_data as importer
    source = tmp_path / 'input.c3d'
    source.write_bytes(b'fixture')
    parsed = {'data': {'points': np.ones((4, 1, 2)),
                       'meta_points': {'residuals': np.ones((1, 1, 2))}},
              'parameters': {'POINT': {'LABELS': {'value': ['a']}, 'RATE': {'value': [100]}}},
              'header': {'points': {'first_frame': 0}}}
    monkeypatch.setitem(sys.modules, 'ezc3d', SimpleNamespace(c3d=lambda _: parsed))
    session = tmp_path / 'session'
    session.mkdir()
    manifest = session / 'session.json'
    manifest.write_text(json.dumps({'session_id': 'session', 'assets': {}}))
    before = manifest.read_bytes()
    real_save = store.save_json
    calls = 0

    def fail_save(*args):
        nonlocal calls
        calls += 1
        if calls != failed_save or after_save:
            real_save(*args)
        if calls == failed_save:
            raise OSError('manifest write failed')

    monkeypatch.setattr(store, 'save_json', fail_save)
    monkeypatch.setattr(importer, 'save_json', fail_save)
    with pytest.raises(OSError, match='manifest write failed'):
        import_c3d(source, session)
    assert not (session / 'mocap.h5').exists()
    assert manifest.read_bytes() == before
    assert (session / 'labeled.c3d').read_bytes() == b'fixture'
    monkeypatch.setattr(store, 'save_json', real_save)
    monkeypatch.setattr(importer, 'save_json', real_save)
    path = import_c3d(source, session)
    assets = json.loads(manifest.read_text())['assets']
    assert assets['mocap_h5']['sha256'] == store.file_sha256(path)
    assert assets['c3d_source']['sha256'] == store.file_sha256(source)
