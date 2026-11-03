"""Read-only dependency inspection; file presence is not a precision claim."""
import json
from pathlib import Path
import h5py
from joint_capture.paths import ProjectPaths, checked_id



def runs_in(path, group):
    if not path.is_file():
        return []
    with h5py.File(path, 'r') as source:
        return sorted(name for name,g in source[group].items() if isinstance(g,h5py.Group) and g.attrs.get('complete',False)) if group in source else []


def select_run(available, requested, kind):
    if requested:
        checked_id(requested, kind)
        if requested not in available:
            raise ValueError(f'{kind}不存在: {requested}; 可用版本: {available}')
        return requested
    if len(available) != 1:
        raise ValueError(f'{kind}需要明确选择，不能猜测版本；可用版本: {available}')
    return available[0]


def inspect_session(root, session_id, slam_run=None, vision_run=None, skip_mocap_hand=False):
    paths = ProjectPaths(root)
    session = paths.session(session_id)
    web = paths.root / 'reports' / ('session_' + session_id) / 'world3d'
    checks = []
    def add(name, ready, detail):
        checks.append({'stage': name, 'ready': bool(ready), 'detail': detail})
    add('capture_manifest', (session/'session.json').is_file(), str(session/'session.json'))
    for name in ['ego', 'mocap', 'aligned']:
        add(name+'_h5', (session/(name+'.h5')).is_file(), str(session/(name+'.h5')))
    selected = {}
    for kind, family, requested in [('slam', 'okvis', slam_run), ('vision', 'vision', vision_run)]:
        try:
            versions = runs_in(session/'ego.h5', 'processing/'+family)
            selected[kind] = select_run(versions, requested, kind)
            suffix = 'pose' if kind == 'slam' else 'arrays'
            required = ['timestamp_ns', 'tracking_good', 'position', 'quaternion_xyzw'] if kind == 'slam' else ['vertices_m', 'valid']
            with h5py.File(session/'ego.h5', 'r') as source:
                group = f'processing/{family}/{selected[kind]}/{suffix}'
                missing = [key for key in required if group+'/'+key not in source]
            add(kind, not missing, {'selected': selected[kind], 'available': versions, 'missing': missing})
        except (OSError, ValueError, KeyError) as exc:
            add(kind, False, str(exc))
    try:
        with h5py.File(session/'aligned.h5', 'r') as source:
            keys = ['T_WB', 'marker_fit_rms_m']
            missing = [key for key in keys if 'processing/world_preview/v1/'+key not in source]
        add('mocap_reference', not missing, {'missing': missing})
    except OSError as exc:
        add('mocap_reference', False, str(exc))
    try:
        with h5py.File(session/'mocap.h5','r') as source:
            needed=['raw/frames/time_s','raw/markers/positions','raw/markers/valid','raw/markers/labels'] + ([] if skip_mocap_hand else ['processing/mano'])
            missing=[key for key in needed if key not in source]
        add('mocap_h5_reference',not missing,{'missing':missing})
        with h5py.File(session/'aligned.h5','r') as source:
            add('candidate_registration','processing/slam_registration' in source,'H5 内的候选安装外参与时间偏移')
    except OSError as exc:add('h5_reference',False,str(exc))
    return {'session': session_id, 'ready': all(c['ready'] for c in checks),
            'selected_runs': selected, 'checks': checks,
            'limitations': ['ready 表示输入依赖就绪，不代表精度或标定已通过验证',
                            '参考缓存从三 H5 重建；物理外参和时间偏移仍为候选值']}


def require_ready(root, session_id, slam_run=None, vision_run=None, skip_mocap_hand=False):
    status = inspect_session(root, session_id, slam_run, vision_run, skip_mocap_hand)
    failed = [c for c in status['checks'] if not c['ready']]
    if failed:
        raise ValueError('流程依赖未就绪：\n'+json.dumps(failed, ensure_ascii=False, indent=2))
    return status['selected_runs']
