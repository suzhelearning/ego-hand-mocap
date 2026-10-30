"""Explicit adapters for versioned vision/MANO mesh results in session H5 files."""
import numpy as np


def load_mesh(file, stage, version):
    from joint_capture.h5store import resolve_version
    from .data import normalize_motion, MAX_FRAMES
    if stage not in ('mano', 'vision'):
        raise ValueError('仅支持 mano 或 vision 处理阶段。')
    group = resolve_version(file, stage, version)
    if not bool(group.attrs.get('source_complete', True)):
        raise ValueError('该处理版本的源数据未完成。')
    if 'mesh/vertices' not in group or 'mesh/faces' not in group:
        raise ValueError('此版本尚无可显示的 mesh/vertices 与 mesh/faces；参数数组不能直接当作三维动作。')
    vertices, faces = group['mesh/vertices'], group['mesh/faces']
    if (vertices.ndim != 3 or vertices.shape[-1] != 3 or
            not 1 <= vertices.shape[0] <= MAX_FRAMES or not 3 <= vertices.shape[1] <= 4096 or
            vertices.shape[0] * vertices.shape[1] > 2_000_000):
        raise ValueError('网格需要 (帧数, 顶点数, 3)，最多4096顶点、200万帧顶点样本；请裁剪长片段。')
    if faces.ndim != 2 or faces.shape[1] != 3 or not 1 <= len(faces) <= 8192 or faces.dtype.kind not in 'iu':
        raise ValueError('网格面需要最多8192个整数三角形索引。')
    triangles = faces[:]
    if np.any(triangles < 0) or np.any(triangles >= vertices.shape[1]):
        raise ValueError('网格面包含越界顶点索引。')
    arrays = group.get('arrays', {})
    if 'time_s' in arrays:
        if arrays['time_s'].shape != (len(vertices),):
            raise ValueError('time_s 与网格帧数不一致。')
        times = arrays['time_s'][:]
    else:
        rate_key = next((key for key in ('mocap_frame_rate', 'frame_rate') if key in arrays), None)
        if rate_key is None or arrays[rate_key].shape != ():
            raise ValueError('网格缺少明确时间轴：需要 arrays/time_s 或标量 frame_rate/mocap_frame_rate。')
        rate = float(arrays[rate_key][()])
        if not np.isfinite(rate) or rate <= 0:
            raise ValueError('网格帧率必须为正有限数。')
        times = np.arange(len(vertices)) / rate
    coords = vertices[:]
    if not np.isfinite(coords).all():
        raise ValueError('网格包含非有限顶点坐标，请先处理无效帧。')
    valid = np.ones(len(vertices), dtype=bool)
    if 'valid' in arrays:
        if arrays['valid'].shape != (len(vertices),) or arrays['valid'].dtype.kind != 'b':
            raise ValueError('网格 valid 必须为逐帧布尔数组。')
        valid = arrays['valid'][:]
    result = normalize_motion({'version': 1, 'unit': str(group.attrs.get('units', '')),
        'coordinate_system': str(group.attrs.get('coordinate_frame', '')),
        'kind': 'mesh', 'name': f'{stage.upper()} 网格 · {group.name.split("/")[-1]}',
        'frames': [{'time_s': float(t), 'points': points.tolist(), 'valid': bool(ok)}
                   for t, points, ok in zip(times, coords, valid)]}, max_points=4096)
    result['faces'] = triangles.tolist()
    result['stage'] = stage
    result['processing_version'] = group.name.split('/')[-1]
    if 'rgb_index' in arrays:
        indices = arrays['rgb_index']
        if indices.shape != (len(vertices),) or indices.dtype.kind not in 'iu' or np.any(indices[:] < 0):
            raise ValueError('rgb_index 必须为与网格帧一一对应的非负整数数组。')
        for frame, index in zip(result['frames'], indices[:]):
            frame['rgb_index'] = int(index)
    result['warnings'].append('显示已保存的手部表面网格；网格顶点不是21关节。相对首帧预览，不推断跨设备同步。')
    return result
