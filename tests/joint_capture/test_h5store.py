import json
import h5py
import pytest
from joint_capture.h5store import write_version


def test_append_preserves_raw_and_updates_asset(tmp_path):
    path=tmp_path/'ego.h5'
    with h5py.File(path,'w') as f:f['sensor/test']=[1,2,3]
    (tmp_path/'session.json').write_text(json.dumps({'assets':{'ego_h5':{'path':'ego.h5','sha256':'initial'}}}))
    write_version(path,'vision','v1',lambda g:g.create_dataset('joints',data=[4,5]))
    with h5py.File(path) as f:
        assert f['sensor/test'][:].tolist()==[1,2,3]
        assert f['processing/vision/v1'].attrs['complete']
    d=json.loads((tmp_path/'session.json').read_text())
    assert d['assets']['ego_h5']['original_sha256']=='initial'
    assert d['assets']['ego_h5']['sha256']!='initial'


def test_failed_write_preserves_entire_file(tmp_path):
    path=tmp_path/'ego.h5'
    with h5py.File(path,'w') as f:f['sensor/test']=[1]
    before=path.read_bytes()
    def fail(g):g['partial']=[1];raise RuntimeError('fail')
    with pytest.raises(RuntimeError):write_version(path,'vision','v1',fail)
    assert path.read_bytes()==before
    assert not list(tmp_path.glob('*.tmp'))


def test_versions_cannot_be_overwritten(tmp_path):
    path=tmp_path/'aligned.h5'
    write_version(path,'sync','v1',lambda g:g.create_dataset('x',data=[1]))
    before=path.read_bytes()
    with pytest.raises(FileExistsError):write_version(path,'sync','v1',lambda g:None)
    assert path.read_bytes()==before


@pytest.mark.parametrize('existing', [False, True])
@pytest.mark.parametrize('after_save', [False, True])
def test_manifest_failure_rolls_back_and_allows_retry(tmp_path, monkeypatch, existing, after_save):
    import joint_capture.h5store as store
    path = tmp_path / 'aligned.h5'
    if existing:
        with h5py.File(path, 'w') as f:
            f['raw/data'] = [17]
    before = path.read_bytes() if existing else None
    manifest = tmp_path / 'session.json'
    manifest.write_text(json.dumps({'assets': {}}))
    manifest_before = manifest.read_bytes()
    real_save = store.save_json

    def fail_save(*args):
        if after_save:
            real_save(*args)
        raise OSError('manifest write failed')

    monkeypatch.setattr(store, 'save_json', fail_save)
    with pytest.raises(OSError, match='manifest write failed'):
        store.write_version(path, 'sync', 'v1', lambda g: g.create_dataset('x', data=[2]))
    assert (path.read_bytes() if path.exists() else None) == before
    assert manifest.read_bytes() == manifest_before
    monkeypatch.setattr(store, 'save_json', real_save)
    store.write_version(path, 'sync', 'v1', lambda g: g.create_dataset('x', data=[2]))
    assert json.loads(manifest.read_text())['assets']['aligned_h5']['sha256'] == store.file_sha256(path)
    assert not list(tmp_path.glob('*.rollback'))
