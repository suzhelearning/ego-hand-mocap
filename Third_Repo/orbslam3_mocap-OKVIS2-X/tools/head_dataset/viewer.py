#!/usr/bin/env python3
"""Local, read-only 3D trajectory viewer and self-contained offline replay export."""
import argparse
import bisect
import csv
import hashlib
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse
import webbrowser

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
ASSETS = Path(__file__).with_name('viewer')
FILES = {'live': 'okvis2-slam_trajectory.csv', 'final': 'okvis2-slam-final_trajectory.csv',
         'ba': 'okvis2-slam-final-ba_trajectory.csv'}
CACHE = {}
LOCK = threading.RLock()


def dataset_for(result):
    manifest = result/'viewer_manifest.json'
    if manifest.exists():
        try:
            return Path(json.loads(manifest.read_text())['dataset']).resolve()
        except (OSError, ValueError, KeyError):
            pass
    if (result.parent/'mav0').is_dir():
        return result.parent
    candidates = sorted((ROOT/'datasets').glob('*/adapter_report.json'), key=lambda p: -len(p.parent.name))
    for report in candidates:
        if result.name.startswith(report.parent.name+'_'):
            return report.parent
    return None


def runs(extra=None):
    folders = list((ROOT/'results').glob('*')) + list((ROOT/'datasets').glob('*/results'))
    if extra is not None:
        folders.append(extra)
        if extra.parent.name == 'runs' and extra.parent.parent.name == 'slam':
            folders.extend(extra.parent.glob('*'))
    found = {}
    for folder in folders:
        if not folder.is_dir() or (not any((folder/name).exists() for name in FILES.values()) and folder.resolve()!=extra):
            continue
        folder = folder.resolve()
        key = hashlib.sha256(str(folder).encode()).hexdigest()[:16]
        dataset = dataset_for(folder)
        found[key] = {'id': key, 'name': folder.name, 'path': str(folder),
                      'dataset': dataset.name if dataset else '',
                      'modified': max(((folder/n).stat().st_mtime for n in FILES.values() if (folder/n).exists()), default=folder.stat().st_mtime)}
    return sorted(found.values(), key=lambda r: -r['modified'])


def read_trajectory(path, quality, keep_first=False):
    unique = {}
    if not path.exists():
        return []
    with path.open() as stream:
        for row in csv.reader(stream):
            if len(row) < 17:
                continue
            try:
                stamp = int(row[0]); values = [float(x) for x in row[1:17]]
            except ValueError:
                continue  # header or partially written live line
            if not all(math.isfinite(x) for x in values) or not 0.8 < np.linalg.norm(values[3:7]) < 1.2:
                continue
            if not keep_first or stamp not in unique:
                unique[stamp] = [stamp, *values, quality.get(stamp, -1)]
    return [unique[t] for t in sorted(unique)]


def load_data(result, name=None):
    tracked = [result/n for n in [*FILES.values(), 'tracking_quality.csv', 'okvis2-slam-final_map.csv']]
    fingerprint = tuple((p.stat().st_mtime_ns, p.stat().st_size) if p.exists() else (0,0) for p in tracked)
    with LOCK:
        cached = CACHE.get(str(result))
        if cached and cached[0] == fingerprint:
            return cached[1]
        quality = {}
        p = result/'tracking_quality.csv'
        if p.exists():
            for row in csv.DictReader(p.open()):
                try:
                    quality[int(row['timestamp_ns'])] = int(row['quality'])
                except (ValueError, TypeError, KeyError):
                    continue
        trajectories = {key: read_trajectory(result/file, quality, keep_first=(key=='live')) for key,file in FILES.items()}
        preferred = next((key for key in ('ba','final','live') if trajectories[key]), 'live')
        rows = trajectories[preferred]
        points = []
        map_file = result/'okvis2-slam-final_map.csv'
        if rows and map_file.exists():
            xyz = np.asarray([r[1:4] for r in rows]); low = xyz.min(axis=0); high = xyz.max(axis=0)
            margin = np.maximum(high-low, 3.)
            for row in csv.reader(map_file.open()):
                if row == ['landmarks:']:
                    continue
                if len(row) != 4:
                    break  # only the landmark section, not keyframe descriptor rows
                try:
                    pt = np.array([float(v) for v in row[1:4]])
                except ValueError:
                    break
                if np.isfinite(pt).all() and np.all(pt >= low-margin) and np.all(pt <= high+margin):
                    points.append(pt.tolist())
            if len(points)>15000:
                points = [points[i] for i in np.linspace(0,len(points)-1,15000,dtype=int)]
        dataset = dataset_for(result)
        source = dataset.name if dataset else result.name
        data = {'schema': 1, 'name': name or result.name, 'dataset': source, 'result': str(result),
                'revision': hashlib.sha256(str(fingerprint).encode()).hexdigest()[:16],
                'trajectories': trajectories, 'preferred': preferred, 'points': points,
                'hasFrames': bool(dataset and (dataset/'mav0/cam0/data.csv').exists()),
                'completed': bool(trajectories['ba']), 'updated': time.time(),
                'frameConvention': 'T_world_imu; Z up; metres; absolute yaw is not externally referenced'}
        CACHE[str(result)] = (fingerprint, data)
        return data


def offline_html(data):
    page = (ASSETS/'index.html').read_text()
    js = (ASSETS/'plotly.min.js').read_text().replace('</script', '<\\/script')
    page = page.replace('<script src="/plotly.min.js"></script>', '<script>'+js+'</script>')
    packed = json.dumps(data, ensure_ascii=False, separators=(',', ':'), allow_nan=False).replace('<', '\\u003c')
    return page.replace('/*__INITIAL_DATA__*/', 'window.OFFLINE_DATA='+packed+';')


def export(result, output=None):
    target = output or result/'replay_3d.html'
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(offline_html(load_data(result)))
    print(f'离线三维回放：{target}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path)
    parser.add_argument('--port', type=int, default=8787)
    parser.add_argument('--no-open', action='store_true')
    parser.add_argument('--url-file', type=Path)
    parser.add_argument('--pid-file', type=Path)
    parser.add_argument('--export', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = args.result.resolve() if args.result else None
    if result and not result.is_dir():
        parser.error('result must be an existing directory')
    if args.export:
        if not result:
            parser.error('--export requires --result')
        export(result, args.output)
        return
    if not (ASSETS/'plotly.min.js').exists():
        parser.error('missing local Plotly bundle')

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *unused):
            pass

        def reply(self, body, content_type='application/json; charset=utf-8', status=200, disposition=None):
            if isinstance(body, (dict,list)):
                body = json.dumps(body, ensure_ascii=False, allow_nan=False).encode()
            elif isinstance(body,str):
                body = body.encode()
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            if disposition:
                self.send_header('Content-Disposition', disposition)
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            url = urlparse(self.path); query = parse_qs(url.query)
            try:
                if url.path == '/':
                    self.reply((ASSETS/'index.html').read_bytes(), 'text/html; charset=utf-8'); return
                if url.path == '/plotly.min.js':
                    self.reply((ASSETS/'plotly.min.js').read_bytes(), 'text/javascript; charset=utf-8'); return
                listed = runs(result)
                if url.path == '/api/runs':
                    self.reply(listed); return
                chosen = next((r for r in listed if r['id'] == query.get('run',[''])[0]), None)
                if not chosen:
                    self.reply({'error': '结果目录尚无轨迹，等待运行开始或选择其他结果。'}, status=404); return
                folder = Path(chosen['path'])
                if url.path == '/api/data':
                    self.reply(load_data(folder)); return
                if url.path == '/download':
                    self.reply(offline_html(load_data(folder)), 'text/html; charset=utf-8',
                               disposition='attachment; filename="replay_3d.html"'); return
                if url.path == '/offline':
                    self.reply(offline_html(load_data(folder)), 'text/html; charset=utf-8'); return
                if url.path == '/frame':
                    cam = int(query.get('cam',['0'])[0]); stamp = int(query.get('t',['0'])[0])
                    if cam not in (0,1):
                        raise ValueError('invalid camera')
                    dataset = dataset_for(folder)
                    if not dataset:
                        raise ValueError('no source images available')
                    base = dataset/'mav0'/f'cam{cam}'
                    entries = []
                    for row in list(csv.reader((base/'data.csv').open()))[1:]:
                        if len(row)>=2: entries.append((int(row[0]),row[1]))
                    times = [e[0] for e in entries]
                    index = min(bisect.bisect_left(times,stamp),len(times)-1)
                    if index and abs(times[index-1]-stamp)<abs(times[index]-stamp): index-=1
                    path = (base/'data'/entries[index][1]).resolve()
                    if path.parent != (base/'data').resolve():
                        raise ValueError('invalid image path')
                    self.reply(path.read_bytes(), 'image/png'); return
                self.reply({'error': '未知请求'}, status=404)
            except (OSError, ValueError, IndexError, KeyError) as exc:
                self.reply({'error': str(exc)}, status=400)

    try:
        server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    except OSError:
        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    listed = runs(result)
    selected = next((r['id'] for r in listed if result and r['path']==str(result)), listed[0]['id'] if listed else '')
    url = f'http://127.0.0.1:{server.server_port}/'+('#'+selected if selected else '')
    if args.url_file:
        args.url_file.write_text(url+'\n')
    if args.pid_file:
        args.pid_file.write_text(str(os.getpid())+'\n')
    print(f'三维可视化：{url}\n仅在本机访问；Ctrl+C 停止服务。', flush=True)
    if result:
        def save_when_finished():
            while True:
                try:
                    log = result/'run.log'
                    if log.exists():
                        with log.open('rb') as stream:
                            stream.seek(max(0, log.stat().st_size-4096))
                            finished = b'total processing time' in stream.read()
                        if finished and (result/FILES['ba']).exists():
                            export(result)
                            return
                except (OSError, ValueError):
                    pass
                time.sleep(2)
        threading.Thread(target=save_when_finished, daemon=True).start()
    if not args.no_open:
        threading.Timer(.6, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
