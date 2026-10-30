"""Staged metric MANO fitting. Units: metres, pixels, seconds.

The temporal stage is a joint full-sequence solve with gap-masked edges, not
a sliding-window approximation. Caller controls sequence length/device.
"""
import time
from numbers import Integral, Real

import numpy as np
import torch

from .mano import MirroredMANO, rotation_6d, rotation_matrix
from .temporal import temporal_loss
from .priors import source_root, select_prior


DEFAULT_CONFIG = dict(iterations_a=40, iterations_b=60, iterations_c=40, lr=.01,
                      device='cpu', max_gap_s=.2, fallback_gap_s=.15, min_stereo_joints=3,
                      sigma_px=5., sigma_m=.015, save_vertices=False, mano_path=None,
                      freeze_beta_temporal=False, mono_prior_mode='left',
                      beta_mode='shared_optimize', articulation_mode='optimize',
                      reprojection_mode='pixels', sigma_bearing=.012)
DEFAULT_WEIGHTS = dict(reprojection=1., stereo=1., pose=.05, shape=.005, temporal=.005, anatomy=.001)


def resolve_config(config=None):
    """Validate overrides and return an independent, JSON-compatible effective config."""
    overrides = dict(config or {})
    unknown = set(overrides) - set(DEFAULT_CONFIG) - {'weights'}
    if unknown:
        raise ValueError(f'Unknown optimizer settings: {sorted(unknown)}')
    cfg = {**DEFAULT_CONFIG, **overrides}
    weight_overrides = dict(cfg.pop('weights', {}))
    unknown = set(weight_overrides) - set(DEFAULT_WEIGHTS)
    if unknown:
        raise ValueError(f'Unknown loss weights: {sorted(unknown)}')
    weights = {**DEFAULT_WEIGHTS, **weight_overrides}
    for key in ('iterations_a', 'iterations_b', 'iterations_c', 'min_stereo_joints'):
        value = cfg[key]
        if isinstance(value, bool) or not isinstance(value, Integral) or value < 0:
            raise ValueError(f'{key} must be a nonnegative integer')
        cfg[key] = int(value)
    if not 3 <= cfg['min_stereo_joints'] <= 6:
        raise ValueError('min_stereo_joints must be in [3,6] for the stable-joint subset')
    for key in ('lr', 'sigma_px', 'sigma_m', 'sigma_bearing', 'max_gap_s', 'fallback_gap_s'):
        value = cfg[key]
        if isinstance(value, bool) or not isinstance(value, Real) or not np.isfinite(value) or value < 0 or (value == 0 and key != 'fallback_gap_s'):
            raise ValueError(f'{key} must be finite and positive (fallback_gap_s may be zero)')
        cfg[key] = float(value)
    for key, value in weights.items():
        if isinstance(value, bool) or not isinstance(value, Real) or not np.isfinite(value) or value < 0:
            raise ValueError(f'Weight {key} must be finite and nonnegative')
        weights[key] = float(value)
    if not isinstance(cfg['save_vertices'], bool):
        raise ValueError('save_vertices must be boolean')
    if not isinstance(cfg['freeze_beta_temporal'], bool):
        raise ValueError('freeze_beta_temporal must be boolean')
    for key, choices in dict(mono_prior_mode=('left', 'best_geometry', 'dual'),
                             beta_mode=('shared_optimize', 'shared_fixed', 'native_fixed'),
                             articulation_mode=('optimize', 'fixed_native'),
                             reprojection_mode=('pixels', 'bearings')).items():
        if cfg[key] not in choices:
            raise ValueError(f'{key} must be one of {choices}')
    cfg['device'] = str(torch.device(cfg['device']))
    if cfg['mano_path'] is not None:
        cfg['mano_path'] = str(cfg['mano_path'])
    return {**cfg, 'weights': weights}


def pose_validity(result):
    """Audit declared-valid poses without silently removing failed entries.

    No valid predictions means passed=None, not a vacuous accuracy claim.
    """
    valid = np.asarray(result['valid'], bool)
    count = int(valid.sum())
    def so3(value):
        value = value[valid].reshape(-1, 3, 3)
        finite = np.isfinite(value).all((-1, -2))
        good = np.zeros(len(value), bool)
        v = value[finite]
        good[finite] = (np.max(np.abs(v.swapaxes(-1, -2) @ v - np.eye(3)), axis=(-1, -2)) <= 1e-4) & (np.abs(np.linalg.det(v)-1) <= 1e-4)
        return int((~good).sum())
    root_bad, finger_bad = so3(result['root_rotation']), so3(result['finger_pose'])
    joints_bad = int((~np.isfinite(result['joints3d_m'][valid]).all((-1, -2))).sum())
    wrist_bad = int((~np.isfinite(result['wrist_t_m'][valid]).all(-1)).sum())
    beta_values = np.asarray(result['betas'])
    if beta_values.ndim == 3:
        beta_values = beta_values[valid]
    beta_finite = bool(np.isfinite(beta_values).all())
    beta_ok = bool(beta_finite and (np.abs(beta_values) <= 3+1e-6).all())
    beta_pass = beta_finite if result.get('beta_mode') == 'native_fixed' else beta_ok
    return dict(checked_valid_hands=count, root_rotation_failures=root_bad, finger_rotation_failures=finger_bad,
                joints_nonfinite_hands=joints_bad, wrist_nonfinite_hands=wrist_bad, betas_finite_and_bounded=beta_ok,
                passed=bool(not (root_bad or finger_bad or joints_bad or wrist_bad) and beta_pass) if count else None)


def _camera(camera):
    if not isinstance(camera, dict):
        camera = vars(camera)
    model = str(camera.get('model', 'opencv_fisheye')).upper()
    k = camera.get('K', camera.get('camera_matrix'))
    if isinstance(k, dict):
        k = k['matrix_3x3']
    d = camera.get('D', camera.get('distortion_coefficients'))
    return model, np.asarray(k, dtype=float), np.asarray(d, dtype=float).reshape(-1)


def _radial_limit(camera):
    model, _, coefficients = _camera(camera)
    if model in ('FISHEYE624', 'CAMERAMODELTYPE.FISHEYE624'):
        from ego_hand_benchmark.geometry.hot3d_camera import theta_limit
        return theta_limit(tuple(coefficients))
    from ego_hand_benchmark.geometry.fisheye import kb4_theta_limit
    return kb4_theta_limit(tuple(coefficients))


def unproject_bearings(pixels, camera):
    """Audited native inverse; no surrogate intrinsics or angular pixel units."""
    from ego_hand_benchmark.geometry.fisheye import unproject
    model, k, d = _camera(camera)
    if model in ('OPENCV_FISHEYE', 'FISHEYE', 'EQUIDISTANT'):
        model = 'KB4'
    if model == 'CAMERAMODELTYPE.FISHEYE624':
        model = 'FISHEYE624'
    # Dimensions are validation metadata only; inverse geometry uses actual K/D.
    return unproject(pixels, dict(model=model, camera_matrix={'matrix_3x3': k},
        distortion_coefficients=d, image_width=1, image_height=1))


def domain_barrier(points, camera):
    """Zero in the native domain; differentiable continuation beyond its fold.

    This guides the loss but is NOT the validity guarantee: initialization,
    best-state acceptance and snapshots separately check native domain masks.
    The zero-range escape term chooses the positive optical axis.
    """
    radius = points[..., :2].square().sum(-1).clamp_min(1e-24).sqrt()
    theta = torch.atan2(radius, points[..., 2])
    radial = torch.relu(theta - _radial_limit(camera)).square()
    zero_range = torch.linalg.vector_norm(points, dim=-1) <= 1e-12
    escape = (torch.relu(1e-4 - points[..., 2]) / 1e-4).square()
    return radial + torch.where(zero_range, escape, torch.zeros_like(escape))


def project_torch(points, camera, return_valid=False):
    """Native KB4 or FISHEYE624 (6 radial + 2 tangential + 4 prism).

    return_valid=True returns finite coordinates and a domain mask for fitting;
    default returns NaN outside the calibrated radial branch. Optical-axis
    limit has finite, correct derivatives, including exactly x=y=0.
    """
    model, kn, dn = _camera(camera)
    k, d = points.new_tensor(kn), points.new_tensor(dn)
    fisheye624 = model in ('FISHEYE624', 'CAMERAMODELTYPE.FISHEYE624')
    if not fisheye624 and model not in ('KB4', 'OPENCV_FISHEYE', 'FISHEYE', 'EQUIDISTANT'):
        raise ValueError(f'Unsupported differentiable camera model: {model}')
    if len(d) != (12 if fisheye624 else 4):
        raise ValueError('Camera coefficient count does not match model')
    finite = torch.isfinite(points).all(-1)
    p = torch.nan_to_num(points, nan=0., posinf=0., neginf=0.)
    r2 = p[..., :2].square().sum(-1)
    radius = r2.clamp_min(1e-16).sqrt()
    theta = torch.atan2(radius, p[..., 2])
    theta2 = theta.square()
    polynomial = torch.ones_like(theta)
    power = theta2
    for coefficient in d[:6 if fisheye624 else 4]:
        polynomial = polynomial + coefficient * power
        power = power * theta2
    scale = theta * polynomial / radius
    xy = p[..., :2] * scale[..., None]
    if fisheye624:
        x, y = xy.unbind(-1)
        rr = x*x + y*y
        p1, p2, s1, s2, s3, s4 = d[6:]
        xy = torch.stack((x + 2*p2*x*y + p1*(rr+2*x*x) + s1*rr+s2*rr*rr,
                          y + 2*p1*x*y + p2*(rr+2*y*y) + s3*rr+s4*rr*rr), -1)
        from ego_hand_benchmark.geometry.hot3d_camera import theta_limit
        limit = theta_limit(tuple(dn))
        valid = finite & (p.square().sum(-1) > 1e-16) & (theta < limit)
    else:
        # Match geometry.fisheye.project exactly, including >90-degree rays,
        # native fold margin, negative optical-axis and zero-range rejection.
        native_radius = torch.linalg.vector_norm(p[..., :2], dim=-1)
        native_theta = torch.atan2(native_radius, p[..., 2])
        valid = finite & (torch.linalg.vector_norm(p, dim=-1) > 1e-12)
        valid &= ~((native_radius < 1e-12) & (p[..., 2] < 0))
        valid &= native_theta < _radial_limit(camera)
    uv = xy * torch.stack((k[0, 0], k[1, 1])) + k[:2, 2]
    if return_valid:
        return uv, valid
    return torch.where(valid[..., None], uv, torch.full_like(uv, float('nan')))


def _extrinsic(value):
    if isinstance(value, dict):
        return np.asarray(value['R'], float), np.asarray(value.get('t_m', value.get('t')), float)
    value = np.asarray(value, float)
    return value[:3, :3], value[:3, 3]


def _rigid(local, target):
    a, b = local.mean(0), target.mean(0)
    u, _, vt = np.linalg.svd((local-a).T @ (target-b))
    correction = np.eye(3)
    correction[-1, -1] = np.linalg.det(vt.T @ u.T)
    r = vt.T @ correction @ u.T
    return r, b - a @ r.T


def fit_mano(observations, triangulated, mono_mano, left_camera, right_camera,
             T_left_right, config=None, progress=None):
    """Return final arrays plus ``per_frame``, ``temporal`` and diagnostics.

    mono_mano uses WiLoR canonical-right matrices: hand_pose/mano_hand_pose
    (T,2 cameras,2 hands,15,3,3), global_orient/mano_global_orient (...,3,3)
    or (...,1,3,3), betas/mano_betas (...,10), prediction_ok (...), and
    R_virtual_from_source (...,3,3).
    Left-hand posed geometry is mirrored and root conjugated by diag(-1,1,1).
    Outputs are wrist-centered physical root matrices, canonical finger
    matrices, shared beta (2,10) or native_fixed beta (T,2,10), left-camera
    metre joints and explicit masks. Native beta is retained without clipping;
    missing native pose/beta invalidates that frame/hand in the fixed modes.
    fixed_native articulation always uses the selected single-view fingers,
    including with dual (whose fused prior is used only by optimize mode).
    save_vertices adds vertices_m and canonical-right mesh faces (reverse
    winding for left-hand surface rendering; wireframe uses the same edges).
    No monocular translation participates in metric initialization.
    """
    cfg = resolve_config(config)
    weights = cfg['weights']
    device = torch.device(cfg['device'])
    model = MirroredMANO(cfg.get('mano_path')).to(device)
    dtype = torch.float32
    def tensor(a, **kw):
        return torch.as_tensor(a, device=device, dtype=kw.get('dtype', dtype))
    xyz = np.asarray(triangulated['joints3d_m'])
    n = len(xyz)
    if xyz.shape != (n, 2, 21, 3):
        raise ValueError('Expected triangulated joints3d_m (T,2,21,3)')
    times = np.asarray(observations['timestamps_s'], dtype=float)
    if times.shape != (n,) or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError('timestamps_s must be finite and strictly increasing')
    valid3 = np.asarray(triangulated['valid'], bool) & np.isfinite(xyz).all(-1)
    uv = np.asarray(observations['keypoints2d'])
    conf = np.asarray(observations['confidence']).copy()
    if uv.shape != (n, 2, 2, 21, 2) or conf.shape != (n, 2, 2, 21):
        raise ValueError('Expected observations keypoints2d (T,2,2,21,2), confidence (T,2,2,21)')
    valid2 = np.isfinite(uv).all(-1) & np.isfinite(conf) & (conf > 0)
    valid2 &= np.asarray(observations.get('joint_valid', np.ones_like(conf)), bool)
    valid2 &= np.asarray(observations['hand_valid'], bool)[..., None]
    rays = None
    if cfg['reprojection_mode'] == 'bearings':
        rays = np.stack([unproject_bearings(uv[:, c], camera)
                         for c, camera in enumerate((left_camera, right_camera))], 1)
        valid2 &= np.isfinite(rays).all(-1)
    conf = np.where(valid2, np.clip(conf, 0, 1), 0)
    Rlr, tlr = _extrinsic(T_left_right)
    pose = np.broadcast_to(np.eye(3), (n, 2, 15, 3, 3)).copy()
    roots = np.broadcast_to(np.eye(3), (n, 2, 3, 3)).copy()
    beta_samples = [[], []]
    native_betas = np.zeros((n, 2, 10))
    native_ok = np.zeros((n, 2), bool)
    native_beta_ok = np.zeros((n, 2), bool)
    selected_views = np.full((n, 2), -1, int)
    mono = mono_mano or {}
    mp = mono.get('hand_pose', mono.get('mano_hand_pose'))
    mr = mono.get('global_orient', mono.get('mano_global_orient'))
    mb = mono.get('betas', mono.get('mano_betas'))
    if 'pose' in mono and mp is None:
        raise ValueError('Ambiguous axis-angle mono pose; supply audited WiLoR rotation matrices')
    for t in range(n):
        for h in range(2):
            candidates = []
            for c in range(2):
                if mp is None or mr is None:
                    break
                if not np.asarray(mono.get('prediction_ok', np.ones((n, 2, 2), bool)))[t, c, h]:
                    continue
                p = np.asarray(mp[t, c, h]).reshape(15, 3, 3)
                r = np.asarray(mr[t, c, h]).reshape(3, 3)
                if not (np.isfinite(p).all() and np.isfinite(r).all()):
                    continue
                virtual = np.asarray(mono.get('R_virtual_from_source', np.broadcast_to(np.eye(3), (n, 2, 2, 3, 3))))[t, c, h]
                if not np.isfinite(virtual).all():
                    raise ValueError('Valid mono pose needs finite virtual-to-source rotation')
                b = np.asarray(mb[t, c, h]) if mb is not None else None
                b = b if b is not None and np.isfinite(b).all() else None
                score = float('inf')
                if cfg['mono_prior_mode'] != 'left':
                    ids = np.flatnonzero(valid3[t, h])
                    if len(ids) >= 3:
                        with torch.no_grad():
                            candidate_j, _ = model(tensor(p[None]), tensor((b if b is not None else np.zeros(10))[None]),
                                tensor(np.eye(3)[None]), tensor(np.zeros((1, 3))), tensor([h]))
                        candidate_j = candidate_j[0].cpu().numpy()
                        if np.linalg.matrix_rank(candidate_j[ids]-candidate_j[ids].mean(0)) >= 2:
                            cr, ct = _rigid(candidate_j[ids], xyz[t, h, ids])
                            score = float(np.mean((candidate_j[ids] @ cr.T + ct - xyz[t, h, ids])**2))
                candidates.append(dict(camera=c, pose=p, root=source_root(r, virtual, h, c, Rlr), beta=b, score=score))
                if cfg['mono_prior_mode'] == 'left':
                    break
            if candidates:
                pose[t, h], selected = select_prior(candidates, cfg['mono_prior_mode'])
                if cfg['articulation_mode'] == 'fixed_native':
                    pose[t, h] = selected['pose']
                roots[t, h] = selected['root']
                selected_views[t, h] = selected['camera']
                native_ok[t, h] = True
                if selected['beta'] is not None:
                    beta_samples[h].append(selected['beta'])
                    native_betas[t, h] = selected['beta']
                    native_beta_ok[t, h] = True
    betas = np.stack([np.median(x, axis=0) if len(x) else np.zeros(10) for x in beta_samples])
    betas = np.clip(betas, -3, 3)
    initial_betas = native_betas if cfg['beta_mode'] == 'native_fixed' else np.tile(betas, (n, 1)).reshape(n, 2, 10)
    side = tensor(np.tile([0, 1], n))
    with torch.no_grad():
        local, _ = model(tensor(pose.reshape(-1, 15, 3, 3)), tensor(initial_betas.reshape(-1, 10)),
                         tensor(np.broadcast_to(np.eye(3), (n*2, 3, 3)).copy()), tensor(np.zeros((n*2, 3))), side)
        local = local.cpu().numpy().reshape(n, 2, 21, 3)
    trans = np.zeros((n, 2, 3))
    active = np.zeros((n, 2), bool)
    status = np.full((n, 2), 'invalid', dtype='<U16')
    stable = np.array([0, 1, 5, 9, 13, 17])
    for t in range(n):
        for h in range(2):
            ids = stable[valid3[t, h, stable]]
            if len(ids) >= cfg['min_stereo_joints'] and np.linalg.matrix_rank(local[t, h, ids]-local[t, h, ids].mean(0)) >= 2:
                roots[t, h], trans[t, h] = _rigid(local[t, h, ids], xyz[t, h, ids])
                active[t, h], status[t, h] = True, 'stereo'
    # Borrow depth only inside the same timestamp-continuous segment, bounded
    # by fallback_gap_s. Long missing stretches remain invalid, not zero hands.
    segments = np.cumsum(np.r_[False, np.diff(times) > cfg['max_gap_s']])
    for h in range(2):
        anchors = np.flatnonzero(active[:, h])
        for t in np.flatnonzero(~active[:, h]):
            candidates = anchors[(segments[anchors] == segments[t]) & (np.abs(times[anchors]-times[t]) <= cfg['fallback_gap_s'])]
            if not len(candidates) or not valid2[t, :, h].any():
                continue
            before, after = candidates[candidates < t], candidates[candidates > t]
            if len(before) and len(after):
                a, b = before[-1], after[0]
                f = (times[t]-times[a])/(times[b]-times[a])
                trans[t, h] = trans[a, h]*(1-f)+trans[b, h]*f
            else:
                a = candidates[np.argmin(np.abs(times[candidates]-times[t]))]
                trans[t, h] = trans[a, h]
            active[t, h], status[t, h] = True, 'mono_fallback'
    if cfg['articulation_mode'] == 'fixed_native':
        active &= native_ok
    if cfg['beta_mode'] == 'native_fixed':
        active &= native_beta_ok
    status[~active] = 'invalid'
    valid3 &= active[..., None]
    # Reject only affected frame/hands before defining the feasible best state.
    # Preserve the stereo data externally; an out-of-domain MANO initialization
    # is explicitly invalid, not falsely advertised as a fitted stereo result.
    with torch.no_grad():
        initial_j, _ = model(tensor(pose.reshape(-1, 15, 3, 3)), tensor(initial_betas.reshape(-1, 10)),
                             tensor(roots.reshape(-1, 3, 3)), tensor(trans.reshape(-1, 3)), side)
        initial_j = initial_j.reshape(n, 2, 21, 3)
        initial_domains = torch.stack((project_torch(initial_j, left_camera, True)[1],
            project_torch((initial_j-tensor(tlr)) @ tensor(Rlr), right_camera, True)[1]), 1).cpu().numpy()
    initial_rejected = active & np.any((conf > 0) & ~initial_domains, axis=(1, 3))
    active[initial_rejected] = False
    status[initial_rejected] = 'invalid'
    valid3 &= active[..., None]
    conf *= active[:, None, :, None]
    # Stereo quality only downweights already accepted triangulation.
    quality = np.ones_like(valid3, dtype=float)
    if 'angle_deg' in triangulated:
        quality *= np.clip(np.nan_to_num(triangulated['angle_deg'])/5, 0, 1)
    if 'ray_gap_m' in triangulated:
        quality *= 1/(1+(np.nan_to_num(triangulated['ray_gap_m'], nan=1)/.01)**2)
    quality[..., [4, 8, 12, 16, 20]] *= .5
    w3 = tensor(quality * valid3)
    target3 = tensor(np.nan_to_num(xyz, nan=0., posinf=0., neginf=0.))
    target2 = tensor(np.nan_to_num(uv, nan=0., posinf=0., neginf=0.))
    target_rays = tensor(np.nan_to_num(rays)) if rays is not None else None
    w2 = tensor(conf)
    active_t = tensor(active, dtype=torch.bool)
    r6 = torch.nn.Parameter(rotation_6d(tensor(roots)))
    p6 = torch.nn.Parameter(rotation_6d(tensor(pose)))
    translation = torch.nn.Parameter(tensor(trans))
    beta = torch.nn.Parameter(tensor(native_betas if cfg['beta_mode'] == 'native_fixed' else betas))
    prior = tensor(pose)
    all_parameters = [r6, translation, p6, beta]
    diagnostics = dict(loss_curves=[], failures=[], model_convention='WiLoR mirrored MANO_RIGHT; canonical finger matrices',
                       mano_asset=model.asset_path, config={**cfg, 'weights': weights}, temporal_mode='full_sequence_gap_masked',
                       initialization='rigid wrist/MCP stereo, no monocular cam_t', source_counts={s: int((status == s).sum()) for s in ('stereo', 'mono_fallback', 'invalid')})
    diagnostics['device'] = str(device)
    diagnostics['selected_mono_views'] = selected_views.tolist()
    diagnostics['beta_layout'] = 'T,hand,10' if cfg['beta_mode'] == 'native_fixed' else 'hand,10'
    diagnostics['bearing_residual'] = 'unit_direction_chord / sigma_bearing; dimensionless'
    diagnostics['domain_rejection'] = dict(initialization_hands=int(initial_rejected.sum()),
        initialization_indices=np.argwhere(initial_rejected).tolist(), reason='observed_joint_outside_native_domain',
        candidate_hands=0, fallback_anatomical_hands=0, snapshot_hands=0)
    diagnostics['gpu_name'] = torch.cuda.get_device_name(device) if device.type == 'cuda' else None
    diagnostics['stage_timings'] = {s: dict(executed=False, wall_seconds=0., max_memory_allocated_bytes=None,
                                           max_memory_reserved_bytes=None) for s in ('A', 'B', 'C')}

    def forward():
        r = rotation_matrix(r6)
        p = prior if cfg['articulation_mode'] == 'fixed_native' else rotation_matrix(p6)
        frame_beta = beta if cfg['beta_mode'] == 'native_fixed' else beta[None].expand(n, -1, -1)
        j, v = model(p.reshape(-1, 15, 3, 3), frame_beta.reshape(-1, 10),
                     r.reshape(-1, 3, 3), translation.reshape(-1, 3), side)
        return j.reshape(n, 2, 21, 3), v.reshape(n, 2, 778, 3), r, p

    def robust(delta, weight):
        # Mask before reduction; all targets were sanitized before subtraction.
        value = torch.nn.functional.smooth_l1_loss(delta, torch.zeros_like(delta), reduction='none').mean(-1)
        return (value * weight).sum() / weight.sum().clamp_min(1)

    def objective(stage):
        j, _, r, p = forward()
        terms = {}
        terms['stereo'] = robust((j-target3)/cfg['sigma_m'], w3)
        jr = (j-tensor(tlr)) @ tensor(Rlr)
        projected, domains = zip(*(project_torch(x, c, True) for x, c in ((j, left_camera), (jr, right_camera))))
        projection = torch.stack(projected, 1)
        domain = torch.stack(domains, 1)
        # Never drop a measured residual when its prediction leaves the branch.
        # In-domain residual/normalization remains exactly the previous loss.
        if cfg['reprojection_mode'] == 'bearings':
            directions = torch.nn.functional.normalize(torch.stack((j, jr), 1), dim=-1)
            terms['reprojection'] = robust((directions-target_rays)/cfg['sigma_bearing'], w2)
        else:
            terms['reprojection'] = robust((projection-target2)/cfg['sigma_px'], w2)
        barrier = torch.stack((domain_barrier(j, left_camera), domain_barrier(jr, right_camera)), 1)
        terms['reprojection'] = terms['reprojection'] + (barrier*w2).sum()/w2.sum().clamp_min(1)
        terms['pose'] = ((p-prior).square().mean((-1, -2, -3))*active_t).sum()/active_t.sum().clamp_min(1)
        terms['shape'] = beta.square().mean()
        # Mild rotation prior only, not a claim of anatomical joint-limit enforcement.
        terms['anatomy'] = ((p-torch.eye(3, device=device)).square().mean((-1, -2, -3))*active_t).sum()/active_t.sum().clamp_min(1)
        terms['temporal'] = temporal_loss(j, r, p, tensor(times-times[0]), active_t, cfg['max_gap_s']) if stage == 'C' else j.sum()*0
        total = sum(terms[key]*weights[key] for key in weights)
        bad_domain = ((w2 > 0) & ~domain).any(dim=1).any(dim=-1) & active_t
        return total, terms, bad_domain

    def snapshot():
        with torch.no_grad():
            j, v, r, p = forward()
            result = dict(joints3d_m=j.cpu().numpy().copy(), wrist_t_m=translation.detach().cpu().numpy().copy(),
                          root_rotation=r.cpu().numpy().copy(), finger_pose=p.cpu().numpy().copy(), betas=beta.detach().cpu().numpy().copy(),
                            valid=active.copy(), source_status=status.copy())
            result['beta_mode'] = cfg['beta_mode']
            domains = torch.stack((project_torch(j, left_camera, True)[1],
                project_torch((j-tensor(tlr)) @ tensor(Rlr), right_camera, True)[1]), 1)
            rejected = (((w2 > 0) & ~domains).any(1).any(-1) & active_t).cpu().numpy()
            diagnostics['domain_rejection']['snapshot_hands'] += int(rejected.sum())
            result['valid'][rejected] = False
            result['source_status'][rejected] = 'invalid'
            if cfg['save_vertices']:
                result['vertices_m'] = v.cpu().numpy().copy()
                result['faces'] = np.asarray(model.layer.faces, dtype=np.int64).copy()
            for key in ('joints3d_m', 'wrist_t_m', 'root_rotation', 'finger_pose', 'vertices_m'):
                if key in result:
                    result[key][~result['valid']] = np.nan
            return result

    def optimize(stage, iterations):
        if device.type == 'cuda':
            torch.cuda.synchronize(device)
            torch.cuda.reset_peak_memory_stats(device)
        started = time.perf_counter()
        parameters = all_parameters[:2] if stage == 'A' else all_parameters
        parameters = [x for x in parameters if not (
            (x is p6 and cfg['articulation_mode'] == 'fixed_native') or
            (x is beta and (cfg['beta_mode'] != 'shared_optimize' or
                           (stage == 'C' and cfg['freeze_beta_temporal']))))]
        for parameter in all_parameters:
            parameter.requires_grad_(any(parameter is x for x in parameters))
        optimizer = torch.optim.Adam(parameters, lr=cfg['lr'])
        best = [x.detach().clone() for x in all_parameters]
        best_loss = float('inf')
        def restore():
            with torch.no_grad():
                for parameter, value in zip(all_parameters, best):
                    parameter.copy_(value)
        for iteration in range(iterations + 1):
            optimizer.zero_grad(set_to_none=True)
            loss, terms, bad_domain = objective(stage)
            # Keep the other anatomical hand's proposal. A shared beta rollback
            # restores all frames of the offending side, preserving feasibility.
            # Nonfinite failures still use the existing full-stage rollback path.
            if torch.isfinite(loss) and bad_domain.any():
                rejected_sides = bad_domain.any(0)
                diagnostics['domain_rejection']['candidate_hands'] += int(bad_domain.sum())
                diagnostics['domain_rejection']['fallback_anatomical_hands'] += int(rejected_sides.sum())
                with torch.no_grad():
                    for parameter, value in zip(all_parameters, best):
                        if parameter is beta and cfg['beta_mode'] != 'native_fixed':
                            parameter[rejected_sides] = value[rejected_sides]
                        else:
                            parameter[:, rejected_sides] = value[:, rejected_sides]
                        for state_value in optimizer.state.get(parameter, {}).values():
                            if torch.is_tensor(state_value) and state_value.shape == parameter.shape:
                                if parameter is beta and cfg['beta_mode'] != 'native_fixed':
                                    state_value[rejected_sides] = 0
                                else:
                                    state_value[:, rejected_sides] = 0
                loss, terms, bad_domain = objective(stage)
                if bad_domain.any():
                    # Defensive guarantee: never accept an infeasible best state.
                    diagnostics['failures'].append(dict(stage=stage, iteration=iteration, reason='domain_fallback_failed'))
                    restore()
                    break
            number = float(loss.detach())
            values = {key: float(value.detach()) for key, value in terms.items()}
            finite = np.isfinite(number) and all(np.isfinite(v) for v in values.values())
            row = dict(stage=stage, iteration=iteration, loss=number if np.isfinite(number) else None,
                       **{key: value if np.isfinite(value) else None for key, value in values.items()})
            if not finite:
                diagnostics['failures'].append(dict(stage=stage, iteration=iteration, reason='nonfinite_loss'))
                restore()
                row['fallback'] = 'best_finite_or_stage_input'
            diagnostics['loss_curves'].append(row)
            if progress:
                progress(row)
            if not finite:
                break
            if number < best_loss:
                best_loss, best = number, [x.detach().clone() for x in all_parameters]
            if iteration == iterations:
                break
            loss.backward()
            if any(x.grad is not None and not torch.isfinite(x.grad).all() for x in parameters):
                diagnostics['failures'].append(dict(stage=stage, iteration=iteration, reason='nonfinite_gradient'))
                break
            torch.nn.utils.clip_grad_norm_(parameters, 10.)
            optimizer.step()
            with torch.no_grad():
                if cfg['beta_mode'] == 'shared_optimize':
                    beta.clamp_(-3, 3)
        restore()
        if device.type == 'cuda':
            torch.cuda.synchronize(device)
        diagnostics['stage_timings'][stage] = dict(executed=True, wall_seconds=time.perf_counter()-started,
            max_memory_allocated_bytes=int(torch.cuda.max_memory_allocated(device)) if device.type == 'cuda' else None,
            max_memory_reserved_bytes=int(torch.cuda.max_memory_reserved(device)) if device.type == 'cuda' else None)

    if active.any():
        optimize('A', int(cfg['iterations_a']))
        optimize('B', int(cfg['iterations_b']))
    per_frame = snapshot()
    if active.any() and cfg['iterations_c'] > 0:
        optimize('C', int(cfg['iterations_c']))
    temporal = snapshot()
    diagnostics['per_frame_pose_validity'] = pose_validity(per_frame)
    diagnostics['final_pose_validity'] = pose_validity(temporal)
    return {**temporal, 'per_frame': per_frame, 'temporal': temporal, 'diagnostics': diagnostics}
