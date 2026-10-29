"""Ego 相机外参标定的数据记录与图像归档；不在此阶段求解外参。"""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import threading
import time

from natnet_zenoh.schema import decode_frame


def write_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf8')
    temporary.replace(path)


class CalibrationRecorder:
    """每个采样组保存一个静止窗口，完整帧始终独立保存。"""
    def __init__(self, output, *, rigid_name='ego', rigid_id=None, camera_id='cam0_optical'):
        self.root = Path(output)
        self.root.mkdir(parents=True, exist_ok=False)
        self.lock = threading.RLock()
        self.group = None
        self.fatal_error = None
        self.meta = {'schema':'ego-calibration-capture', 'schema_version':1,
            'status':'recording', 'created_utc':datetime.now(timezone.utc).isoformat(),
            'rigid_name':rigid_name, 'rigid_id':rigid_id, 'camera_id':camera_id,
            'receiver_time_domain':'linux-clock-monotonic',
            'time_alignment':'not_calibrated', 'extrinsics':'not_calibrated',
            'raw_frames':0, 'invalid_messages':0, 'groups':[]}
        self.raw = (self.root/'mocap.jsonl').open('x',encoding='utf8',buffering=1)
        self.events = (self.root/'events.jsonl').open('x',encoding='utf8',buffering=1)
        write_json(self.root/'session.json', self.meta)

    def on_names(self, sample):
        try:
            payload = json.loads(sample.payload.to_string())
            with self.lock:
                if self.meta['rigid_id'] is None:
                    ids = [int(k) for k,v in payload.get('names',{}).items() if v == self.meta['rigid_name']]
                    if len(ids) == 1:
                        self.meta['rigid_id'] = ids[0]
                self.meta['rigid_body_names'] = payload
        except (ValueError, TypeError, AttributeError):
            pass

    def on_frame(self, sample):
        received = time.monotonic_ns()
        with self.lock:
            try:
                frame = decode_frame(sample.payload.to_string())
            except (ValueError, AttributeError):
                self.meta['invalid_messages'] += 1
                return
            try:
                self.raw.write(json.dumps({'receiver_time_ns':received, 'frame':frame}, allow_nan=False)+'\n')
                self.meta['raw_frames'] += 1
                if self.group is not None:
                    matches = [b for b in frame['rigid_bodies'] if b['id'] == self.meta['rigid_id']]
                    self.group['received_frames'] += 1
                    if len(matches) == 1 and matches[0]['tracking_valid']:
                        self.group['valid_rigid_frames'] += 1
                    self.group['stream'].write(json.dumps({'receiver_time_ns':received,
                        'frame_number':frame['frame_number'], 'motive_timestamp':frame['motive_timestamp'],
                        'natnet_timing':frame.get('natnet_timing'),
                        'rigid_body': matches[0] if len(matches)==1 else None},allow_nan=False)+'\n')
            except (OSError, ValueError) as exc:
                self.fatal_error = str(exc)

    def start_group(self):
        with self.lock:
            if self.group is not None:
                raise ValueError('已有采样组正在录制')
            if self.meta['rigid_id'] is None:
                raise ValueError('尚未收到 ego 名称映射，请等待或使用 --rigid-id')
            number = len(self.meta['groups'])+1
            folder = self.root/'samples'/f'{number:04d}'
            folder.mkdir(parents=True)
            self.group = {'group_id':f'{number:04d}', 'start_receiver_time_ns':time.monotonic_ns(),
                'received_frames':0, 'valid_rigid_frames':0,
                'stream':(folder/'ego.jsonl').open('x',encoding='utf8',buffering=1)}
            self.events.write(json.dumps({'event':'group_start','group_id':self.group['group_id'],
                'receiver_time_ns':self.group['start_receiver_time_ns']})+'\n')
            return self.group['group_id']

    def end_group(self):
        with self.lock:
            if self.group is None:
                raise ValueError('没有正在录制的采样组')
            group = self.group
            group.pop('stream').close()
            group['end_receiver_time_ns'] = time.monotonic_ns()
            group['usable_rigid_data'] = group['valid_rigid_frames'] > 0
            self.meta['groups'].append(group)
            write_json(self.root/'samples'/group['group_id']/'sample.json', group)
            self.events.write(json.dumps({'event':'group_end',**group})+'\n')
            self.group = None
            write_json(self.root/'session.json',self.meta)
            return group

    def close(self, status='complete'):
        with self.lock:
            if self.raw.closed:
                return
            if self.group is not None:
                self.end_group()
            self.meta['status'] = 'failed' if self.fatal_error else status
            self.meta['error'] = self.fatal_error
            self.meta['closed_utc'] = datetime.now(timezone.utc).isoformat()
            self.raw.close()
            self.events.close()
            write_json(self.root/'session.json',self.meta)


def attach_image(args):
    root = args.session
    meta = json.loads((root/'session.json').read_text())
    if meta['status'] == 'recording':
        raise ValueError('请先停止录制，再导入图像')
    if args.group not in [g['group_id'] for g in meta['groups']]:
        raise ValueError('采样组不存在')
    source = args.image.resolve(strict=True)
    if not source.is_file() or source.suffix.lower() not in {'.png','.jpg','.jpeg','.bmp','.tif','.tiff'}:
        raise ValueError('需要相机导出的图像文件')
    folder = root/'samples'/args.group/'camera'
    folder.mkdir(exist_ok=False)
    destination = folder/('image'+source.suffix.lower())
    shutil.copyfile(source,destination)
    write_json(folder/'image.json', {'camera_id':meta['camera_id'], 'image':destination.name,
        'source_path':str(source), 'sha256':hashlib.sha256(destination.read_bytes()).hexdigest(),
        'capture_timestamp_ns':args.timestamp_ns, 'clock_domain':args.clock_domain,
        'frame_index':args.frame_index, 'association':'manual_static_pose_group',
        'synchronization_verified':False})
    return 0


def record(args):
    import zenoh
    from natnet_zenoh.zenoh_transport import build_client_config
    recorder = CalibrationRecorder(args.output,rigid_name=args.rigid_name,
        rigid_id=args.rigid_id,camera_id=args.camera_id)
    status = 'interrupted'
    try:
        with zenoh.open(build_client_config(args.connect_endpoint)) as session:
            names = session.declare_subscriber('mocap/rigid_body_names',recorder.on_names)
            frames = session.declare_subscriber('mocap/hands/frame',recorder.on_frame)
            try:
                if args.duration:
                    deadline = time.monotonic()+args.startup_timeout
                    while recorder.meta['rigid_id'] is None and time.monotonic()<deadline:
                        time.sleep(.05)
                    recorder.start_group()
                    time.sleep(args.duration)
                    recorder.end_group()
                else:
                    print('连续保存动捕流。停稳后输入 s 开始一组，e 结束一组，q 保存退出。')
                    while True:
                        if recorder.fatal_error:
                            raise OSError(recorder.fatal_error)
                        command = input('ego-calibration> ').strip().lower()
                        try:
                            if command == 's': print('开始采样组',recorder.start_group())
                            elif command == 'e': print('结束采样组',recorder.end_group())
                            elif command == 'q': break
                        except ValueError as exc:
                            print(str(exc))
                if not recorder.meta['raw_frames']:
                    raise ValueError('没有收到动捕帧')
                if not any(g['usable_rigid_data'] for g in recorder.meta['groups']):
                    raise ValueError('没有采集到含有效目标刚体的采样组')
                status = 'complete'
            finally:
                frames.undeclare()
                names.undeclare()
    except (KeyboardInterrupt, EOFError):
        print('录制已中断，已收到的数据将保留。')
    except Exception as exc:
        recorder.fatal_error = str(exc)
        raise
    finally:
        recorder.close(status)
    return 0 if status == 'complete' else 130


def positive_seconds(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('必须为有限正数')
    return number


def nonnegative_integer(value):
    number = int(value)
    if number < 0:
        raise argparse.ArgumentTypeError('必须为非负整数')
    return number


def main(argv=None):
    parser = argparse.ArgumentParser(description='Ego 外参标定数据录制与图像归档')
    sub = parser.add_subparsers(dest='command',required=True)
    p = sub.add_parser('record',help='连续保存动捕流并标记静止采样组')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--rigid-name',default='ego')
    p.add_argument('--rigid-id',type=nonnegative_integer)
    p.add_argument('--camera-id',default='cam0_optical')
    p.add_argument('--connect-endpoint',default='tcp/127.0.0.1:7447')
    p.add_argument('--duration',type=positive_seconds,help='自动录制单组的秒数；省略则交互采集')
    p.add_argument('--startup-timeout',type=positive_seconds,default=12.)
    p.set_defaults(handler=record)
    p = sub.add_parser('attach-image',help='将设备导出的图像归档到静止采样组')
    p.add_argument('--session',type=Path,required=True)
    p.add_argument('--group',required=True)
    p.add_argument('--image',type=Path,required=True)
    p.add_argument('--timestamp-ns',type=nonnegative_integer,required=True)
    p.add_argument('--clock-domain',required=True,help='包含设备或 boot_id 的相机时钟域')
    p.add_argument('--frame-index',type=nonnegative_integer,required=True)
    p.set_defaults(handler=attach_image)
    args = parser.parse_args(argv)
    try:
        return args.handler(args)
    except (ValueError,OSError) as exc:
        parser.exit(2,f'错误：{exc}\n')


if __name__ == '__main__':
    raise SystemExit(main())
