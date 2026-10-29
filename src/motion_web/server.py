"""Local-only motion viewer; all uploads are processed in memory, never saved."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

from .data import MAX_BYTES, demo_pair, load_bytes
from joint_capture.manifests import resolve_hdf5_reference

STATIC = Path(__file__).parent / 'static'


def resolve_data(root, relative):
    root = Path(root).resolve()
    supplied = Path(relative)
    if supplied.is_absolute():
        raise ValueError('文件路径必须相对于数据目录。')
    path = (root / supplied).resolve()
    if not path.is_relative_to(root):
        raise ValueError('文件解析后越出了数据根目录。')
    allowed = [root / name for name in ('sessions', 'unassigned', 'samples',
                                        'ego_data', 'mocap_data', 'aligned_h5')]
    if not any(path.is_relative_to(folder.resolve()) for folder in allowed) or not path.is_file():
        raise ValueError('只能读取数据根目录中已允许的数据目录。')
    return path


def discover_files(root):
    """List only assets with a known reader; omit legacy copies once sessions exist."""
    root = Path(root).resolve()
    files = []

    def add(path, session_id=None, run_id=None, asset_type=None, record_id=None):
        if path.suffix.lower() not in ('.json', '.h5', '.hdf5'):
            return
        relative = path.relative_to(root).as_posix()
        try:
            resolved = resolve_data(root, relative)
        except ValueError:
            return
        files.append({'path': relative, 'size': resolved.stat().st_size,
                      'session_id': session_id, 'run_id': run_id, 'asset_type': asset_type,
                      'record_id': record_id})
        if resolved.suffix.lower() in ('.h5', '.hdf5'):
            sources = {}
            for key in ('ego_source_h5', 'mocap_source_h5', 'source_trajectory'):
                try:
                    source = resolve_hdf5_reference(root.parent, resolved, key)
                except (KeyError, OSError, ValueError):
                    continue
                sources[key] = source.relative_to(root.parent).as_posix()
            if sources:
                files[-1]['resolved_sources'] = sources

    sessions = root / 'sessions'
    if sessions.is_dir():
        for session in sorted(p for p in sessions.iterdir() if p.is_dir()):
            for path in sorted((session / 'raw' / 'mocap').glob('*.h5')):
                add(path, session.name, asset_type='mocap_raw_h5')
            runs = session / 'runs'
            if not runs.is_dir():
                continue
            for run in sorted(p for p in runs.iterdir() if p.is_dir()):
                for path in sorted((run / 'aligned').glob('*.h5')):
                    add(path, session.name, run.name, 'mocap_aligned_h5')
                for path in sorted((run / 'ego' / 'hands').glob('*.json')):
                    add(path, session.name, run.name, 'ego_hand_motion')
        unassigned = root / 'unassigned'
        if unassigned.is_dir():
            for record in sorted(p for p in unassigned.iterdir() if p.is_dir()):
                for path in sorted((record / 'raw').rglob('*.h5')):
                    add(path, record_id=record.name, asset_type='mocap_raw_h5')
    else:
        for folder, kind in (('ego_data', 'ego_legacy'), ('mocap_data', 'mocap_raw_h5'),
                             ('aligned_h5', 'mocap_aligned_h5')):
            base = root / folder
            if base.is_dir():
                for path in sorted(base.rglob('*')):
                    if path.is_file() and path.suffix.lower() in ('.json', '.h5', '.hdf5'):
                        add(path, asset_type=kind)
    return files


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, root, **kwargs):
        self.root = root
        super().__init__(*args, directory=str(STATIC), **kwargs)

    def json_response(self, value, status=200):
        body = json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def end_headers(self):
        self.send_header('X-Content-Type-Options', 'nosniff')
        super().end_headers()

    def do_GET(self):
        parsed = urlsplit(self.path)
        query = parse_qs(parsed.query)
        try:
            if parsed.path == '/api/demo':
                self.json_response(demo_pair())
            elif parsed.path == '/api/files':
                self.json_response({'files': discover_files(self.root)})
            elif parsed.path == '/api/load':
                path = resolve_data(self.root, query.get('path', [''])[0])
                if path.stat().st_size > MAX_BYTES:
                    raise ValueError('文件超过 64 MiB，请导出或裁剪动作数据。')
                self.json_response(load_bytes(path.name, path.read_bytes()))
            elif parsed.path.startswith('/api/'):
                self.json_response({'error': '接口不存在。'}, 404)
            else:
                super().do_GET()
        except (ValueError, OSError) as exc:
            self.json_response({'error': str(exc)}, 400)

    def do_POST(self):
        parsed = urlsplit(self.path)
        if parsed.path != '/api/upload':
            self.json_response({'error': '接口不存在。'}, 404)
            return
        # Reject cross-origin browser uploads to this local process.
        origin = self.headers.get('Origin')
        if origin and origin != 'http://' + self.headers.get('Host', ''):
            self.json_response({'error': '不允许跨站上传。'}, 403)
            return
        try:
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 < size <= MAX_BYTES:
                raise ValueError('上传文件必须为 1 字节至 64 MiB。')
            self.connection.settimeout(30)
            body = self.rfile.read(size)
            if len(body) != size:
                raise ValueError('上传未完成。')
            name = parse_qs(parsed.query).get('name', [''])[0]
            self.json_response(load_bytes(name, body))
        except (ValueError, OSError) as exc:
            self.json_response({'error': str(exc)}, 400)


def make_server(root, host='127.0.0.1', port=8765):
    root = Path(root).resolve()
    if (root / 'data').is_dir():
        root = root / 'data'
    return ThreadingHTTPServer((host, port), partial(Handler, root=root))


def main(argv=None):
    parser = argparse.ArgumentParser(description='EGO / MOCAP 本地动作可视化')
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2] / 'data', help='数据目录或含data/的项目目录，默认本项目data/')
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args(argv)
    with make_server(args.root, port=args.port) as server:
        print(f'Motion 可视化：http://127.0.0.1:{server.server_port} （Ctrl+C 退出）', flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
    return 0
