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
