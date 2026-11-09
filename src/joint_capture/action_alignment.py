"""Per-Take Ego/Motive mappings; original files are never rewritten."""
import json
from pathlib import Path
import h5py
import numpy as np
from .alignment import fit_clock, map_time
from .storage import save_json


def align_action(folder,ego_h5,mocap_h5,clocks,take_id,ego_boot_id):
    folder=Path(folder);clocks=Path(clocks)
    with h5py.File(ego_h5,'r') as ego:
        metadata=json.loads(ego['metadata/dataset.json'][()])
        if metadata['clock']['boot_id']!=ego_boot_id:raise ValueError('Ego boot_id mismatch')
        if metadata['clock']['domain']!='CLOCK_MONOTONIC':raise ValueError('Ego clock domain mismatch')
    models={};evidence={}
    (folder/'clocks').mkdir(exist_ok=True)
    for kind in ('ego','windows'):
        raw=(clocks/(kind+'.jsonl')).read_bytes()
        raw=raw[:raw.rfind(b'\n')+1]
        rows=[json.loads(x) for x in raw.splitlines()]
        freq=int(rows[0]['remote_frequency_hz'])
        probes=[{k:int(v) for k,v in r.items()} for r in rows[1:]]
        models[kind]=fit_clock(probes,freq)
        if models[kind]['max_constraint_violation_us']>0.001:raise ValueError(kind+' causal fit violation')
        evidence[kind]=raw.decode()
        (folder/'clocks'/(kind+'.jsonl')).write_bytes(raw)
    counts={}
    with h5py.File(folder/'alignment.h5','x') as out:
        out.attrs.update(take_id=take_id,host_clock='CLOCK_MONOTONIC',ego_boot_id=ego_boot_id,
                         pc_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                         physical_event_accuracy_verified=False)
        def write(name,ticks,kind,frequency):
            ticks=np.asarray(ticks,dtype=np.int64)
            if not len(ticks) or np.any(ticks<=0):raise ValueError(name+' exposure/sample clock missing')
            ns=np.array([int(t)*10**9//frequency for t in ticks],dtype=np.int64)
            mapped=map_time(ns,models[kind])
            if np.any(np.diff(mapped)<=0):raise ValueError(name+' non-monotonic mapped clock')
            lo,hi=models[kind]['remote_probe_interval_ns'];outside=(ns<lo)|(ns>hi)
            g=out.require_group(name)
            g['source_timestamp_ticks']=ticks;g['pc_timestamp_ns']=mapped
            g['clock_extrapolated']=outside;g['sync_valid']=~outside
            g.attrs.update(frequency_hz=frequency,take_id=take_id,host_clock='CLOCK_MONOTONIC')
            counts[name]=dict(samples=len(ns),mapped=int((~outside).sum()),extrapolated=int(outside.sum()))
        with h5py.File(ego_h5,'r') as ego:
            for kind in ('camera','imu'):
                if 'sensor/'+kind not in ego:continue
                for name,g in ego['sensor/'+kind].items():
                    if 'timestamp_ns' in g:write('ego/'+kind+'/'+name,g['timestamp_ns'][:],'ego',10**9)
        with h5py.File(mocap_h5,'r') as mocap:
            freq=int(mocap.attrs['tick_frequency_hz'])
            if freq!=models['windows']['frequency_hz']:raise ValueError('Motive clock frequency mismatch')
            write('mocap',mocap['frames/natnet_timing/stamp_camera_mid_exposure'][:],'windows',freq)
        for kind,raw in evidence.items():out['clock_evidence/'+kind]=raw
        out['clock_models_json']=json.dumps(models)
    summary=dict(status='mapped',take_id=take_id,host_clock='CLOCK_MONOTONIC',models=models,streams=counts,
                 physical_event_accuracy_verified=False,file='alignment.h5')
    save_json(folder/'alignment.json',summary)
    return summary
