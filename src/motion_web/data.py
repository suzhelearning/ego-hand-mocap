"""Motion adapters. Never infer clock synchronization or hand joints from raw sensors."""
import io
import json
import math
from pathlib import Path

MAX_BYTES = 64 * 1024 * 1024
MAX_FRAMES = 50000
MAX_POINTS = 128


def number(value):
    try:
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    except OverflowError:
        return False


def normalize_motion(data, *, max_points=MAX_POINTS):
    if not isinstance(data, dict) or type(data.get('version')) is not int or data['version'] != 1:
        raise ValueError('需要 version=1 的 motion JSON 对象，请参阅 docs/motion_web.md。')
    unit = data.get('unit')
    scale = {'m': 1, 'mm': .001, 'cm': .01}.get(unit) if isinstance(unit, str) else None
    if scale is None:
        raise ValueError('unit 必须明确为 m、cm 或 mm。')
    coordinate = data.get('coordinate_system')
    if not isinstance(coordinate, str) or not coordinate.strip() or len(coordinate) > 200:
        raise ValueError('coordinate_system 必须明确标注，例如 mocap_world 或 ego_world。')
    frames = data.get('frames')
    if not isinstance(frames, list) or not 1 <= len(frames) <= MAX_FRAMES:
        raise ValueError(f'frames 必须包含 1–{MAX_FRAMES} 帧。')
    normalized, previous, count = [], None, None
    for i, frame in enumerate(frames):
        if not isinstance(frame, dict):
            raise ValueError(f'第 {i} 帧不是对象。')
        t, points = frame.get('time_s'), frame.get('points')
        if not number(t) or (previous is not None and t <= previous):
            raise ValueError(f'第 {i} 帧 time_s 必须有限且严格递增，单位秒。')
        if not isinstance(points, list) or not 1 <= len(points) <= max_points:
            raise ValueError(f'第 {i} 帧 points 必须包含 1–{max_points} 个三维点。')
        count = len(points) if count is None else count
        if len(points) != count:
            raise ValueError('每帧点数必须一致，缺失点请使用 null。')
        valid = frame.get('valid', True)
        if not isinstance(valid, bool):
            raise ValueError('valid 必须为布尔值。')
        converted = []
        for p in points:
            if p is None:
                converted.append(None)
            elif isinstance(p, list) and len(p) == 3 and all(number(v) for v in p):
                converted.append([v * scale for v in p])
            else:
                raise ValueError(f'第 {i} 帧含非法坐标，请用 null 标记缺失点。')
        relative_time = t - frames[0]['time_s']
        if not number(relative_time):
            raise ValueError('相对时间溢出，请使用合理的秒级时间戳。')
        normalized.append({'time_s': relative_time, 'points': converted, 'valid': valid})
        if 'rgb_index' in frame:
            if type(frame['rgb_index']) is not int or frame['rgb_index'] < 0:
                raise ValueError('rgb_index 必须为非负整数。')
            normalized[-1]['rgb_index'] = frame['rgb_index']
        previous = t
    bones = data.get('bones', [])
    if not isinstance(bones, list) or len(bones) > MAX_POINTS * 4:
        raise ValueError('bones 必须为连接索引数组。')
    for edge in bones:
        if (not isinstance(edge, list) or len(edge) != 2 or
                any(type(j) is not int or not 0 <= j < count for j in edge) or edge[0] == edge[1]):
            raise ValueError('bones 需要有效且不同的两个点索引。')
    return {'version': 1, 'unit': 'm', 'coordinate_system': coordinate,
            'name': str(data.get('name', '动作数据'))[:200],
            'kind': str(data.get('kind', 'points'))[:100],
            'synthetic': data.get('synthetic') is True,
            'source_start_s': frames[0]['time_s'], 'frames': normalized, 'bones': bones,
            'warnings': ['按各路首帧归零预览；尚未完成跨设备时间同步和空间标定。']}


def demo_pair():
    bones = []
    for side in range(2):
        base = side * 21
        for finger in range(5):
            first = base + 1 + finger * 4
            bones.append([base, first])
            bones.extend([[first + j, first + j + 1] for j in range(3)])
    pair = {}
    for role in ('ego', 'mocap'):
        frames = []
        for i in range(240):
            t = i / 30
            points = []
            for side in range(2):
                sign = -1 if side == 0 else 1
                origin = [sign * .16 + .025 * math.sin(t), .06 * math.sin(t * 1.4), 0]
                points.append(origin)
                for finger in range(5):
                    for joint in range(4):
                        curl = .35 + .55 * math.sin(t * 2 + finger * .25 + side)
                        length = (.023 if finger == 0 else .029) * (joint + 1)
                        x = origin[0] + sign * ((finger - 2) * .023 + (joint + 1) * (finger - 2) * .002)
                        y = origin[1] + .028 + length * math.cos(curl)
                        z = length * math.sin(curl)
                        if role == 'ego':
                            x += .007 * math.cos(t + joint)
                            z += .006
                        points.append([x, y, z])
            frames.append({'time_s': t, 'points': points})
        pair[role] = normalize_motion({'version': 1, 'unit': 'm', 'coordinate_system': 'demo_world',
            'name': f'{role.upper()} 模拟双手', 'kind': 'hand21', 'synthetic': True,
            'bones': bones, 'frames': frames})
    return pair


def load_h5(body, version=None, stage=None):
    try:
        import h5py
    except ImportError as exc:
        raise ValueError('读取 H5 需要 h5py：请安装本项目的 [motion] 可选依赖。JSON 和模拟示例仍可使用。') from exc
    try:
        with h5py.File(io.BytesIO(body) if isinstance(body, bytes) else body, 'r') as f:
            schema = f.attrs.get('schema', '')
            if 'complete' in f.attrs and not bool(f.attrs['complete']):
                raise ValueError('H5 尚未完成，不能预览。')
            if not bool(f.attrs.get('source_complete', True)):
                raise ValueError('H5 源数据未完成，不能预览。')
            if stage is not None:
                from .processed import load_mesh
                return load_mesh(f, stage, version)
            if schema in ('ego-mocap-nearest-aligned-v1', 'ego-session-aligned-v2'):
                source = f
                if schema == 'ego-session-aligned-v2':
                    from joint_capture.h5store import resolve_version
                    source = resolve_version(f, 'alignment', version)
                if not bool(source.attrs.get('source_complete', True)):
                    raise ValueError('对齐版本的源数据未完成，不能预览。')
                group = source['cameras/RGB_left']
                times = group['rgb_pc_ns']
                ids = sorted(group['rigid_bodies'])
                if times.ndim != 1 or not 1 <= len(times) <= MAX_FRAMES or not 1 <= len(ids) <= MAX_POINTS:
                    raise ValueError('对齐H5帧数或刚体数量超出限制。')
                ticks = times[:]
                bodies = []
                for rid in ids:
                    g = group['rigid_bodies'][rid]
                    if g['position'].shape != (len(ticks), 3) or g['usable'].shape != (len(ticks),):
                        raise ValueError('对齐H5刚体形状与RGB时间轴不一致。')
                    bodies.append((g['position'][:], g['usable'][:]))
                frames = [{'time_s': (int(t)-int(ticks[0]))/1e9,
                           'points': [p[i].tolist() if bool(v[i]) and all(math.isfinite(float(x)) for x in p[i]) else None
                                      for p,v in bodies]} for i,t in enumerate(ticks)]
                data = normalize_motion({'version': 1, 'unit': 'm', 'coordinate_system': 'mocap_world',
                    'kind': 'rigid_positions', 'name': '对齐动捕 · 左RGB时间轴', 'frames': frames})
                data['warnings'].append('按左RGB时间轴显示最近邻动捕刚体中心；外推或跟踪无效帧隐藏。不是EGO算法轨迹，也不是手部骨架；RGB视频未加载，曝光物理相位未标定。')
                if schema == 'ego-session-aligned-v2':
                    data['alignment_version'] = source.name.split('/')[-1]
                return data
            if schema == 'joint_capture.mocap':
                markers = f['raw/markers']
                positions, valid, times = markers['positions'], markers['valid'], f['raw/frames/time_s']
                if (positions.ndim != 3 or positions.shape[2] != 3 or times.ndim != 1
                        or not 1 <= len(times) <= MAX_FRAMES
                        or not 1 <= positions.shape[1] <= MAX_POINTS
                        or positions.shape[0] != len(times) or valid.shape != positions.shape[:2]):
                    raise ValueError('C3D marker 形状或数量超出加载限制。')
                if markers['labels'].shape != (positions.shape[1],):
                    raise ValueError('C3D marker 标签数量与轨迹不一致。')
                labels = markers['labels'].asstr()[:].tolist()
                positions, valid, times = positions[:], valid[:], times[:]
                frames = [{'time_s': float(t), 'points': [
                    p.tolist() if bool(v) and all(math.isfinite(float(x)) for x in p) else None
                    for p, v in zip(positions[i], valid[i])]} for i, t in enumerate(times)]
                data = normalize_motion({'version': 1, 'unit': str(markers.attrs.get('units', '')).lower(),
                    'coordinate_system': str(markers.attrs.get('coordinate_frame', 'c3d_native')),
                    'kind': 'markers', 'name': 'C3D 标记点（非手部骨架）', 'frames': frames})
                data['labels'] = labels
                data['warnings'].append('仅显示 C3D 原始标记点，不推断21关节或骨架连接；时间为相对首帧秒数。')
                return data
            if schema != 'ego-natnet-raw-rigid-v1':
                raise ValueError('该 H5 尚无已适配的动作字段。原始 EGO 图像/IMU 不能直接变成手部动作；请先导出 motion JSON。')
            if 'rigid_bodies' not in f or 'frames/motive_timestamp' not in f:
                raise ValueError('NatNet H5 缺少 rigid_bodies 或 frames/motive_timestamp。')
            times_ds = f['frames/motive_timestamp']
            ids = sorted(f['rigid_bodies'].keys())
            if times_ds.ndim != 1 or not 1 <= len(times_ds) <= MAX_FRAMES or not 1 <= len(ids) <= MAX_POINTS:
                raise ValueError('H5 帧数或刚体数量超出加载限制。')
            times = times_ds[:]
            bodies = []
            for rid in ids:
                g = f['rigid_bodies'][rid]
                if g['position'].shape != (len(times), 3) or g['tracking_valid'].shape != (len(times),):
                    raise ValueError('H5 刚体轨迹形状与时间轴不一致。')
                bodies.append((g['position'][:], g['tracking_valid'][:]))
            frames = []
            for i, t in enumerate(times):
                points = [p[i].tolist() if bool(v[i]) and all(math.isfinite(float(x)) for x in p[i]) else None
                          for p, v in bodies]
                frames.append({'time_s': float(t), 'points': points})
            data = normalize_motion({'version': 1, 'unit': str(f.attrs.get('position_unit', '')),
                'coordinate_system': 'mocap_world', 'kind': 'rigid_positions',
                'name': 'NatNet 刚体位置（非手部骨架）', 'frames': frames})
            data['warnings'].append('仅显示刚体中心位置，不是手部 Ground Truth；时间使用 Motive 相对时间，仅供动作预览。')
            return data
    except (OSError, KeyError, TypeError) as exc:
        raise ValueError(f'无法解析 H5：{exc}') from exc


def load_bytes(name, body, version=None, stage=None):
    if len(body) > MAX_BYTES:
        raise ValueError('文件不能超过 64 MiB，请裁剪动作片段。')
    suffix = Path(name).suffix.lower()
    if suffix in ('.h5', '.hdf5'):
        return load_h5(body, version=version, stage=stage)
    if suffix != '.json':
        raise ValueError('当前支持 motion JSON 与已知 NatNet 刚体 H5。C3D 请先转换为统一 motion JSON；尚未实现 MANO 拟合。')
    try:
        data = json.loads(body)
        if isinstance(data, dict) and data.get('kind') == 'mesh':
            raise ValueError('motion JSON 不支持 mesh；请加载 H5 中包含拓扑的处理版本。')
        return normalize_motion(data)
    except (UnicodeDecodeError, json.JSONDecodeError, RecursionError) as exc:
        raise ValueError('文件不是有效的 UTF-8 motion JSON。') from exc


def load_path(path, version=None, stage=None):
    """Read only motion datasets; integrated H5 may contain large image datasets."""
    path = Path(path)
    if path.suffix.lower() in ('.h5', '.hdf5'):
        return load_h5(path, version=version, stage=stage)
    if path.stat().st_size > MAX_BYTES:
        raise ValueError('文件不能超过 64 MiB，请裁剪动作片段。')
    return load_bytes(path.name, path.read_bytes(), version=version, stage=stage)
