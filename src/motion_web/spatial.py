"""Read-only, load-time coordinate conversion. No inferred frame/time correspondences."""
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
from acquisition.object_offset import pose_matrix, validate_transform


def _matrix(value):
    try:
        return validate_transform(np.asarray(value, dtype=float))
    except (TypeError, ValueError) as exc:
        raise ValueError('空间变换必须是米制、有限、有效的4×4刚体变换。') from exc


def _peer(session, name):
    path = (session / name).resolve()
    if not path.is_relative_to(session.resolve()):
        raise ValueError('会话关联文件不能越出本会话目录。')
    return path


def _apply(frame, transform):
    return [None if p is None else (transform[:3, :3] @ np.asarray(p) + transform[:3, 3]).tolist()
            for p in frame['points']]


def to_mocap_world(data, session=None, calibration=None, alignment_version=None):
    result = deepcopy(data)
    source = data['coordinate_system']
    result['spatial'] = {'source_frame': source, 'target_frame': 'mocap_world', 'mode': 'identity'}
    if source == 'mocap_world':
        result['warnings'].append('已声明为动捕世界系，未再次施加变换；坐标声明与空间精度仍需独立核实。')
        return result
    if session is None:
        raise ValueError('此数据不在动捕世界系。请从会话目录加载以关联外参和时间配对，或切换原坐标预览。')
    session = Path(session).resolve()
    config_path = _peer(session, 'visualization.json')
    config = json.loads(config_path.read_text()) if config_path.is_file() else {}
    if not isinstance(config, dict):
        raise ValueError('visualization.json 必须为对象。')
    transforms = config.get('world_transforms', {})
    if not isinstance(transforms, dict):
        raise ValueError('world_transforms 必须为按源坐标系命名的对象。')
    if source in transforms:
        entry = transforms[source]
        if not isinstance(entry, dict) or entry.get('position_unit') != 'm':
            raise ValueError('会话世界变换须明确 position_unit=m。')
        T = _matrix(entry.get('T_WSource'))
        for frame in result['frames']:
            frame['points'] = _apply(frame, T)
        result['coordinate_system'] = 'mocap_world'
        result['spatial'].update(mode='session_static', calibration_status=entry.get('status', 'unvalidated'),
                                 transform=T.tolist())
        result['warnings'].append('加载时应用本会话的固定世界变换；状态：' + str(entry.get('status', 'unvalidated')) + '。此转换不建立时间同步。')
        return result
    if calibration is None or not Path(calibration).is_file():
        raise ValueError('缺少相机—头戴刚体外参；不能转换到动捕世界系。')
    extrinsic = json.loads(Path(calibration).read_text())
    if not isinstance(extrinsic, dict):
        raise ValueError('外参配置必须为对象。')
    if source != extrinsic.get('from_frame'):
        raise ValueError(f'坐标系 {source} 尚无到动捕世界系的变换。请在本会话 visualization.json 提供 T_WSource；不猜测尺度、轴方向或腕部平移。')
    if extrinsic.get('position_unit') != 'm':
        raise ValueError('外参平移必须为米。')
    T_BC = _matrix(extrinsic.get('T_BC'))
    body_id = str(extrinsic.get('rigid_body_id', ''))
    if not body_id:
        raise ValueError('外参缺少 rigid_body_id。')
    if any(type(f.get('rgb_index')) is not int for f in data['frames']):
        raise ValueError('相机系动作每帧必须保留原始 rgb_index，不能用帧序号或相对秒数猜配对。')
    aligned = _peer(session, 'aligned.h5')
    if not aligned.is_file():
        raise ValueError('此会话缺少 aligned.h5 时间配对及逐帧头戴位姿；无法转换相机系。')
    import h5py
    from joint_capture.h5store import resolve_version
    try:
        with h5py.File(aligned, 'r') as file:
            if not bool(file.attrs.get('complete', True)) or not bool(file.attrs.get('source_complete', True)):
                raise ValueError('对齐文件尚未完成。')
            group = resolve_version(file, 'alignment', alignment_version)
            if not bool(group.attrs.get('source_complete', True)):
                raise ValueError('配对源数据未完成。')
            if group.attrs.get('position_unit') != 'm':
                raise ValueError('头戴刚体位置单位必须明确为 m。')
            # The bundled calibration is specifically for the left RGB optical camera.
            camera = group['cameras/RGB_left']
            rgb, times = camera['rgb_index'][:], camera['rgb_pc_ns'][:]
            if (rgb.ndim != 1 or times.shape != rgb.shape or len(rgb) == 0 or
                    rgb.dtype.kind not in 'iu' or times.dtype.kind not in 'iu' or
                    len(set(map(int, rgb))) != len(rgb) or np.any(np.diff(times) <= 0)):
                raise ValueError('对齐文件 RGB 索引或时间轴不合法。')
            body = camera['rigid_bodies/' + body_id]
            if 'name' in body.attrs and body.attrs['name'] != extrinsic.get('to_frame'):
                raise ValueError('头戴刚体系名称与标定外参不一致。')
            p, q = body['position'][:], body['quaternion_xyzw'][:]
            valid, clock_valid = body['usable'][:], camera['clock_pair_usable'][:]
            if (p.shape != (len(rgb), 3) or q.shape != (len(rgb), 4) or
                    valid.shape != rgb.shape or clock_valid.shape != rgb.shape):
                raise ValueError('头戴刚体位姿与时间配对行数不一致。')
            lookup = {int(value): i for i, value in enumerate(rgb)}
            selected_times = []
            for frame in result['frames']:
                row = lookup.get(frame['rgb_index'])
                if row is None:
                    raise ValueError(f"rgb_index={frame['rgb_index']} 不在所选配对版本中。")
                selected_times.append(int(times[row]))
                frame['time_s'] = (int(times[row]) - int(times[0])) / 1e9
                frame['valid'] = bool(frame['valid'] and valid[row] and clock_valid[row])
                if frame['valid']:
                    try:
                        T_WC = pose_matrix(p[row], q[row]) @ T_BC
                        frame['points'] = _apply(frame, T_WC)
                    except ValueError:
                        frame['valid'] = False
                if not frame['valid']:
                    frame['points'] = [None] * len(frame['points'])
            if any(b <= a for a, b in zip(selected_times, selected_times[1:])):
                raise ValueError('动作 rgb_index 必须按原始采样顺序严格递增。')
            version = group.name.split('/')[-1]
            result['timeline_id'] = f'{session.name}/alignment/{version}/RGB_left'
            result['spatial'].update(mode='camera_dynamic', alignment_version=version, rigid_body_id=body_id,
                calibration_status=extrinsic.get('status', 'unvalidated'), transform_chain='T_WB(rgb_index) @ T_BC',
                calibration_file=str(calibration), invalid_frames=sum(not f['valid'] for f in result['frames']))
    except (KeyError, OSError, TypeError) as exc:
        raise ValueError(f'空间转换缺少有效的时间配对或头戴位姿字段：{exc}') from exc
    result['coordinate_system'] = 'mocap_world'
    result['warnings'] = [w for w in result['warnings'] if '各路首帧归零' not in w and '相对首帧预览' not in w]
    result['warnings'].append('加载时已按 RGB 索引应用 T_WB(t)·T_BC；无效位姿隐藏。外参状态：' + str(extrinsic.get('status', 'unvalidated')) + '。空间转换不代表曝光同步或精度已验证。')
    return result
