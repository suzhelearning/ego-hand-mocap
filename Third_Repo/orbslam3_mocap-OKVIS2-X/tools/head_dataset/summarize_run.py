#!/usr/bin/env python3
"""Summarize a finished run without mistaking IMU-predicted poses for good tracking."""
import csv
import json
from pathlib import Path
import sys
import numpy as np

dataset = Path(sys.argv[1])
result = Path(sys.argv[2]) if len(sys.argv)>2 else dataset/'results'
adapter = json.loads((dataset/'adapter_report.json').read_text())
quality = {}
with (result/'tracking_quality.csv').open() as stream:
    for row in csv.DictReader(stream): quality[int(row['state_id'])]=row
log = (result/'run.log').read_text()
report = dict(exported_pairs=adapter['frames'][0], unique_reported_states=len(quality),
    tracking_quality={name:sum(int(r['quality'])==k for r in quality.values()) for k,name in enumerate(['Good','Marginal','Lost'])},
    lost_state_ids=[sid for sid,row in quality.items() if int(row['quality'])==2],
    finished='Finished!' in log, final_ba_completed='total processing time' in log,
    visual_lost_warnings=log.count('3d2d tracking lost.'), weak_warnings=log.count('3d2d tracking weak:'),
    warnings_about_accuracy='No ground truth supplied; full frame coverage and Good states do not establish metric accuracy.')
filename = result/'okvis2-slam-final-ba_trajectory.csv'
if not filename.exists(): filename=result/'okvis2-slam-final_trajectory.csv'
if filename.exists():
    rows = list(csv.reader(filename.open()))[1:]
    # Final optimization can publish a timestamp more than once. Keep its
    # latest estimate and allow failed initialization to have zero/one pose.
    unique = {row[0].strip(): [float(x.strip()) for x in row[:17]]
              for row in rows if len(row) >= 17}
    data = np.array(sorted(unique.values(), key=lambda row: row[0])).reshape(-1, 17)
if filename.exists() and len(data):
    t = data[:,0]*1e-9; xyz = data[:,1:4]; delta = np.linalg.norm(np.diff(xyz,axis=0),axis=1);dt=np.diff(t)
    report['trajectory'] = dict(file=str(filename), poses=len(data), finite=bool(np.isfinite(data).all()),
        duration_s=float(t[-1]-t[0]), max_gap_s=float(dt.max()) if len(dt) else 0.0, path_length_m=float(delta.sum()),
        axis_extent_m=np.ptp(xyz,axis=0).tolist(), max_step_m=float(delta.max()) if len(delta) else 0.0,
        implied_speed_p50_p95_max_m_s=np.percentile(delta/dt,[50,95,100]).tolist() if len(dt) else [],
        end_displacement_m=float(np.linalg.norm(xyz[-1]-xyz[0])),
        gyro_bias_norm_p50_max_rad_s=np.percentile(np.linalg.norm(data[:,11:14],axis=1),[50,100]).tolist(),
        accel_bias_norm_p50_max_m_s2=np.percentile(np.linalg.norm(data[:,14:17],axis=1),[50,100]).tolist())
    with (result/'trajectory_imu.tum').open('w') as f:
        f.write('# CLOCK_MONOTONIC seconds; T_world_imu; tx ty tz qx qy qz qw\n')
        for row in data:
            f.write(f'{row[0]*1e-9:.9f} '+' '.join(f'{v:.12g}' for v in row[1:8])+'\n')
(result/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
