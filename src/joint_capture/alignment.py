"""Offline causal affine clock mapping and nearest-frame pairing; no interpolation."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import h5py
import numpy as np
from scipy.optimize import linprog
from .manifests import load_session_manifest, resolve_clock_directory, resolve_session_asset


def fit_clock(rows, frequency):
    if len(rows)<4 or frequency<=0:raise ValueError('对时样本不足或频率无效')
    remote=lambda v:int(v)*10**9//frequency
    origin=remote(rows[len(rows)//2]['remote_receive_ticks'])
    offset=rows[0]['pc_send_ns']-remote(rows[0]['remote_receive_ticks'])
    A=[];b=[]
    for r in rows:
        t2=remote(r['remote_receive_ticks']);t3=remote(r['remote_send_ticks'])
        if t3<t2 or r['pc_receive_ns']<r['pc_send_ns']:raise ValueError('非法探针时间顺序')
        A.extend([[-1,-(t2-origin)/1e9],[1,(t3-origin)/1e9]])
        b.extend([-(r['pc_send_ns']-t2-offset)/1000,(r['pc_receive_ns']-t3-offset)/1000])
    A=np.array(A);b=np.array(b)
    def solve(c):
        result=linprog(c,A_ub=A,b_ub=b,bounds=[(None,None)]*2,method='highs')
        if not result.success:raise ValueError('无法建立有界且满足全部探针的仿射映射: '+result.message)
        return result
    lo=solve([1,0]);hi=solve([-1,0]);theta=(lo.x+hi.x)/2
    rate=[solve([0,1]).x[1],solve([0,-1]).x[1]]
    if 1+theta[1]/1e6<=0:raise ValueError('非法时钟速率')
    return dict(remote_origin_ns=origin,approx_offset_ns=offset,offset_correction_us=float(theta[0]),
        rate_error_ppm=float(theta[1]),rate_feasible_ppm=list(map(float,rate)),
        origin_offset_interval_ns=[offset+lo.fun*1000,offset-hi.fun*1000],
        max_constraint_violation_us=float(max(0,np.max(A@theta-b))),samples=len(rows),frequency_hz=frequency,
        remote_probe_interval_ns=[remote(rows[0]['remote_receive_ticks']),remote(rows[-1]['remote_send_ticks'])],
        formula='pc_ns = remote_ns + approx_offset_ns + 1000*(offset_correction_us + rate_error_ppm*(remote_ns-remote_origin_ns)/1e9)')


def map_time(t,m):
    t=np.asarray(t,dtype=np.int64)
    return t+m['approx_offset_ns']+np.rint(1000*(m['offset_correction_us']+m['rate_error_ppm']*((t-m['remote_origin_ns'])/1e9))).astype(np.int64)


def nearest(times,targets):
    if len(times)==0 or np.any(np.diff(times)<=0):raise ValueError('动捕时间戳必须严格递增且非空')
    r=np.clip(np.searchsorted(times,targets),0,len(times)-1);l=np.maximum(r-1,0)
    return np.where(np.abs(times[l]-targets)<=np.abs(times[r]-targets),l,r)


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def align(session,output):
    session=Path(session).resolve();manifest=session/'session.json';d=load_session_manifest(manifest);models={};sources=[manifest]
    project_root=session.parents[2] if session.parent.name=='sessions' else session.parents[1]
    clock_dir=resolve_clock_directory(session)
    for kind in ['ego','windows']:
        p=clock_dir/f'{kind}.jsonl';sources.append(p)
        rows=[json.loads(line) for line in p.read_text().splitlines()]
        models[kind]=fit_clock(rows[1:],int(rows[0]['remote_frequency_hz']))
    ep=resolve_session_asset(project_root,d,'ego_h5');mp=resolve_session_asset(project_root,d,'mocap_h5');sources.extend([ep,mp])
    with h5py.File(mp) as f:
        if 'frames/natnet_timing/stamp_camera_mid_exposure' not in f:
            raise ValueError('该mocap.h5没有NatNet曝光时钟；C3D/Take需先补齐时间锚点，不能按首帧猜测对齐')
        if str(f.attrs['pc_boot_id'])!=d['pc_boot_id']:raise ValueError('PC boot_id不一致')
        freq=int(f.attrs['tick_frequency_hz'])
        if freq!=models['windows']['frequency_hz']:raise ValueError('Windows频率不一致')
        ticks=f['frames/natnet_timing/stamp_camera_mid_exposure'][:]
        if np.any(ticks<=0):raise ValueError('动捕曝光时钟缺失，拒绝用接收时间替代')
        mt=np.array([int(t)*10**9//freq for t in ticks],dtype=np.int64)
        frame=f['frames/frame_number'][:]
        bodies={k:(g['present'][:]&g['tracking_valid'][:]) for k,g in f['rigid_bodies'].items()}
    with h5py.File(ep) as f:
        meta=json.loads(f['metadata/dataset.json'][()])
        if meta['clock']['boot_id']!=d['ego']['boot_id']:raise ValueError('EGO boot_id不一致')
        rgb={k:g['timestamp_ns'][:].astype(np.int64) for k,g in f['sensor/camera'].items() if k.startswith('RGB')}
    if not rgb:raise ValueError('没有RGB时间戳')
    output=Path(output)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f'对齐输出目录非空，不覆盖: {output}')
    output.mkdir(parents=True,exist_ok=True)
    mpc=map_time(mt,models['windows']);summary=dict(session_id=d['session_id'],source_status=d['status'],
        source_errors=d['errors'],physical_exposure_alignment_valid=False,interpolated=False,cameras={},
        reference='EGO CIF FS vs Motive camera mid-exposure, mapped to Ubuntu CLOCK_MONOTONIC; no RGB physical phase correction',
        source_sha256={p.resolve().relative_to(project_root).as_posix():digest(p) for p in sources})
    for camera,t in rgb.items():
        if len(t)==0 or np.any(np.diff(t)<=0):raise ValueError('RGB时间戳必须严格递增且非空')
        pc=map_time(t,models['ego']);idx=nearest(mpc,pc);dt=mpc[idx]-pc
        covered=(pc>=mpc[0])&(pc<=mpc[-1])
        elo,ehi=models['ego']['remote_probe_interval_ns'];wlo,whi=models['windows']['remote_probe_interval_ns']
        extrap=(t<elo)|(t>ehi)|(mt[idx]<wlo)|(mt[idx]>whi)
        with (output/f'{camera}_nearest.csv').open('w') as f:
            w=csv.writer(f);w.writerow(['rgb_index','rgb_cif_ns','rgb_pc_ns','mocap_index','mocap_frame_number','mocap_exposure_ticks','mocap_pc_ns','delta_ns','within_mocap_coverage','clock_extrapolated',*[f'body_{k}_tracking_valid' for k in bodies],'physical_exposure_alignment_valid'])
            for i,j in enumerate(idx):w.writerow([i,int(t[i]),int(pc[i]),int(j),int(frame[j]),int(ticks[j]),int(mpc[j]),int(dt[i]),bool(covered[i]),bool(extrap[i]),*[bool(v[j]) for v in bodies.values()],False])
        summary['cameras'][camera]=dict(frames=len(t),covered=int(covered.sum()),clock_extrapolated=int(extrap.sum()),
            abs_delta_ms_p50=float(np.median(np.abs(dt))/1e6),abs_delta_ms_p95=float(np.percentile(np.abs(dt),95)/1e6),
            abs_delta_ms_max=float(np.max(np.abs(dt))/1e6),invalid_body_pairs={k:int((~v[idx]).sum()) for k,v in bodies.items()})
    (output/'clock_models.json').write_text(json.dumps(models,indent=2)+'\n')
    (output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    return summary


def align_to_h5(session, version):
    """Compute with temporary scratch files, publish only a version inside aligned.h5."""
    import tempfile
    from .h5store import write_version
    session=Path(session).resolve()
    with tempfile.TemporaryDirectory(prefix='ego-alignment-') as temporary:
        output=Path(temporary)/'sync'
        summary=align(session,output)
        def write(group):
            for name in ('clock_models','summary'):
                group.create_dataset(name+'_json',data=(output/(name+'.json')).read_text(),dtype=h5py.string_dtype())
            cameras=group.create_group('cameras')
            for camera in summary['cameras']:
                with (output/(camera+'_nearest.csv')).open() as stream:rows=list(csv.DictReader(stream))
                if not rows:raise ValueError('配对表为空')
                target=cameras.create_group(camera)
                for key in rows[0]:
                    values=[row[key] for row in rows]
                    data=np.array([x=='True' for x in values],dtype=bool) if values[0] in ('True','False') else np.array(values,dtype=np.int64)
                    target.create_dataset(key,data=data,compression='gzip')
            group['pairs']=cameras
            group.attrs['physical_exposure_alignment_valid']=False
            group.attrs['interpolated']=False
            # Root aliases expose the most recently published sync result, versions remain immutable.
            for key,value in [('pairs',cameras),('clocks',group['clock_models_json'])]:
                if key in group.file:del group.file[key]
                group.file[key]=value
            group.file.attrs['active_sync_version']=version
        write_version(session/'aligned.h5','sync',version,write,
                      metadata={'pipeline':'causal_affine_nearest_v1','summary':summary})
    return session/'aligned.h5'


def main():
    p=argparse.ArgumentParser(description='离线时钟映射及最近邻配对，默认写入会话aligned.h5内部版本')
    p.add_argument('session',help='会话ID或会话目录')
    p.add_argument('--output',type=Path,help='兼容旧工具：显式导出CSV目录')
    p.add_argument('--version','--run-id',dest='version',help='H5内处理版本；省略自动生成')
    a=p.parse_args()
    from datetime import datetime
    import uuid
    from .paths import ProjectPaths
    candidate=Path(a.session)
    if candidate.is_dir():session=candidate.resolve()
    else:
        root=Path(__file__).resolve().parents[2];paths=ProjectPaths(root)
        session=paths.session(a.session) if paths.session(a.session).is_dir() else root/'capture_sessions'/a.session
    if a.output:
        print(json.dumps(align(session,a.output),ensure_ascii=False,indent=2));return
    version=a.version or datetime.now().strftime('%Y%m%dT%H%M%S')+'_'+uuid.uuid4().hex[:6]
    print(align_to_h5(session,version))

if __name__=='__main__':main()
