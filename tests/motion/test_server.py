import json
import threading
from urllib.request import urlopen, Request
from urllib.error import HTTPError
import pytest
from motion_web.server import make_server


@pytest.fixture
def server(tmp_path):
    (tmp_path / 'ego_data').mkdir()
    (tmp_path / 'mocap_data').mkdir()
    (tmp_path / 'ego_data' / 'motion.json').write_text(json.dumps({
        'version': 1, 'unit': 'm', 'coordinate_system': 'test',
        'frames': [{'time_s': 0, 'points': [[0, 0, 0]]}]}))
    (tmp_path / 'secret.json').write_text('secret')
    (tmp_path / 'ego_data' / 'escape.json').symlink_to(tmp_path / 'secret.json')
    httpd = make_server(tmp_path, port=0)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    yield f'http://127.0.0.1:{httpd.server_port}'
    httpd.shutdown()
    httpd.server_close()
    thread.join()


def test_catalog_and_load(server):
    files = json.load(urlopen(server + '/api/files'))['files']
    assert [f['path'] for f in files] == ['ego_data/motion.json']
    data = json.load(urlopen(server + '/api/load?path=ego_data/motion.json'))
    assert data['frames'][0]['points'] == [[0, 0, 0]]


@pytest.mark.parametrize('path', ['../secret.json', 'secret.json', 'ego_data/escape.json'])
def test_path_restriction(server, path):
    with pytest.raises(HTTPError) as error:
        urlopen(server + '/api/load?path=' + path)
    assert error.value.code == 400


def test_upload_error_is_json(server):
    with pytest.raises(HTTPError) as error:
        urlopen(Request(server + '/api/upload?name=bad.json', data=b'{}', method='POST'))
    assert error.value.code == 400
    assert 'error' in json.load(error.value)


def test_demo_and_static_page(server):
    assert json.load(urlopen(server + '/api/demo'))['ego']['synthetic']
    with urlopen(server + '/') as response:
        assert 'text/html' in response.headers['Content-Type']


@pytest.mark.parametrize('payload', [
    {'version': 1, 'unit': []},
    {'version': 1, 'unit': 'm', 'coordinate_system': 'test',
     'frames': [{'time_s': 10**400, 'points': [[0, 0, 0]]}]},
    {'version': True, 'unit': 'm', 'coordinate_system': 'test',
     'frames': [{'time_s': 0, 'points': [[0, 0, 0]]}]},
])
def test_invalid_field_types_return_json_not_broken_connection(server, payload):
    with pytest.raises(HTTPError) as error:
        urlopen(Request(server + '/api/upload?name=bad.json',
                        data=json.dumps(payload).encode(), method='POST'))
    assert error.value.code == 400
    assert json.load(error.value)['error']


def test_canonical_catalog_versions_omit_legacy_and_ego(tmp_path):
    import h5py
    from motion_web.server import discover_files
    session = tmp_path / 'sessions/s1'
    (session / 'runs/r1/aligned').mkdir(parents=True)
    (session / 'raw/mocap').mkdir(parents=True)
    for name in ('mocap.h5', 'ego.h5', 'raw/mocap/old.h5', 'runs/r1/aligned/old.h5'):
        with h5py.File(session / name, 'w'):
            pass
    with h5py.File(session / 'aligned.h5', 'w') as f:
        for name in ('v1', 'v2', 'pending', 'invalid_source'):
            g = f.create_group('processing/alignment/' + name)
            g.attrs['complete'] = name != 'pending'
            g.attrs['source_complete'] = name != 'invalid_source'
    files = discover_files(tmp_path)
    assert [(f['path'], f['version']) for f in files] == [
        ('sessions/s1/mocap.h5', None), ('sessions/s1/aligned.h5', 'v1'),
        ('sessions/s1/aligned.h5', 'v2')]


def test_server_reads_h5_path_without_read_bytes_and_passes_version(server, tmp_path, monkeypatch):
    import h5py
    from pathlib import Path
    path = tmp_path / 'mocap_data/aligned.h5'
    with h5py.File(path, 'w') as f:
        f.attrs['schema'] = 'ego-session-aligned-v2'
        for version in ('v1', 'v2'):
            g = f.create_group('processing/alignment/' + version)
            g.attrs['complete'] = True
            g['cameras/RGB_left/rgb_pc_ns'] = [100]
            g['cameras/RGB_left/rigid_bodies/1/position'] = [[1., 2., 3.]]
            g['cameras/RGB_left/rigid_bodies/1/usable'] = [True]
    def forbidden(self):
        raise AssertionError('H5 must not be read wholly into bytes')
    monkeypatch.setattr(Path, 'read_bytes', forbidden)
    data = json.load(urlopen(server + '/api/load?path=mocap_data/aligned.h5&version=v2'))
    assert data['frames'][0]['points'] == [[1., 2., 3.]]
    with pytest.raises(HTTPError) as error:
        urlopen(server + '/api/load?path=mocap_data/aligned.h5')
    assert error.value.code == 400


def test_catalog_discovers_processing_versions_in_three_h5_layout(tmp_path):
    import h5py
    from motion_web.server import discover_files
    session = tmp_path / 'sessions/new_session'
    session.mkdir(parents=True)
    for filename, stage in [('ego.h5', 'vision'), ('mocap.h5', 'mano')]:
        with h5py.File(session / filename, 'w') as f:
            for version in ['v1', 'pending']:
                g = f.create_group(f'processing/{stage}/{version}')
                g.attrs['complete'] = version == 'v1'
    assets = discover_files(tmp_path)
    processed = [(a['path'], a.get('stage'), a.get('version')) for a in assets if a.get('stage')]
    assert processed == [('sessions/new_session/ego.h5', 'vision', 'v1'),
                         ('sessions/new_session/mocap.h5', 'mano', 'v1')]


def test_http_load_passes_mesh_stage_and_version(server, tmp_path):
    import h5py
    path = tmp_path / 'mocap_data/processed.h5'
    with h5py.File(path, 'w') as f:
        f.attrs['schema'] = 'joint_capture.mocap'
        g = f.create_group('processing/mano/mesh-v1')
        g.attrs.update(complete=True, units='m', coordinate_frame='world')
        g['mesh/vertices'] = [[[0., 0, 0], [1, 0, 0], [0, 1, 0]]]
        g['mesh/faces'] = [[0, 1, 2]]
        g['arrays/time_s'] = [0.]
    result = json.load(urlopen(server + '/api/load?path=mocap_data/processed.h5&stage=mano&version=mesh-v1'))
    assert result['kind'] == 'mesh'
    assert result['processing_version'] == 'mesh-v1'
    assert result['faces'] == [[0, 1, 2]]
