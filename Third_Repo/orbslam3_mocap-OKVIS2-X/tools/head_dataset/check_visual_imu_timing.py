#!/usr/bin/env python3
"""Fit camera/IMU time shift using masked temporal epipolar residuals.

No stereo depth or SLAM trajectory is used. Translation is a nuisance variable.
The estimate is accepted only with enough diverse, high-motion image pairs and
agreement between disjoint subsets of pairs.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from visual_constraints import Measurements


def skew(v):
    x,y,z=v;return np.array([[0,-z,y],[z,0,-x],[-y,x,0.]])


def score(R,p0,p1):
    A=np.cross((R@p0.T).T,p1)
    rng=np.random.default_rng(123)
    pairs=rng.integers(0,len(A),size=(40,2))
    translations=np.cross(A[pairs[:,0]],A[pairs[:,1]])
    translations=np.r_[translations,np.linalg.svd(A,full_matrices=False)[2][-1:]]
    norms=np.linalg.norm(translations,axis=1);translations=translations[norms>1e-10]/norms[norms>1e-10,None]
    if not len(translations):return 100.
    Es=np.array([skew(t)@R for t in translations]);Ep0=np.einsum('hij,nj->hni',Es,p0);ETp1=np.einsum('hji,nj->hni',Es,p1)
    errors=abs(np.einsum('ni,hni->hn',p1,Ep0))/np.sqrt(np.sum(Ep0[:,:,:2]**2+ETp1[:,:,:2]**2,axis=2)+1e-20)
    return float(np.min(np.median(errors,axis=1)))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dataset',type=Path,required=True);p.add_argument('--mask-dataset',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();m=Measurements(args.dataset,args.mask_dataset);imu=np.loadtxt(args.dataset/'mav0/imu0/data.csv',delimiter=',',skiprows=1)
    old_offset=json.loads((args.dataset/'adapter_report.json').read_text())['imu_offset_s'];t=imu[:,0]*1e-9-old_offset
    qs=[];q=Rotation.identity();qs.append(q.as_quat())
    for i in range(1,len(t)):
        q=q*Rotation.from_rotvec((imu[i-1,1:4]+imu[i,1:4])*.5*(t[i]-t[i-1]));qs.append(q.as_quat())
    interp=Slerp(t,Rotation.from_quat(qs));pairs=[];Kinv=np.linalg.inv(m.K[0]);RSC=m.T[0][:3,:3];n=len(m.files[0])
    for i in range(30,n-8,60):
        ta=int(m.files[0][i][0])*1e-9;tb=int(m.files[0][i+6][0])*1e-9
        use=(t>=ta-.12)&(t<=tb+.12)
        if np.median(np.linalg.norm(imu[use,1:4],axis=1))<.2:continue
        p0,d0=m.features(0,i);p1,d1=m.features(0,i+6);matches=m.matches(d0,d1)
        if len(matches)<40:continue
        ids=np.array(matches);a=(Kinv@np.c_[p0[ids[:,0]],np.ones(len(ids))].T).T;b=(Kinv@np.c_[p1[ids[:,1]],np.ones(len(ids))].T).T
        pairs.append((ta,tb,a,b));print('pair',i,'matches',len(ids),flush=True)
    shifts=np.linspace(-.12,.12,121);values=[]
    for shift in shifts:
        errors=[]
        for ta,tb,a,b in pairs:
            if ta-shift<t[0] or tb-shift>t[-1]:errors.append(np.nan);continue
            R=RSC.T@interp(tb-shift).as_matrix().T@interp(ta-shift).as_matrix()@RSC
            errors.append(score(R,a,b)*m.K[0][0,0])
        values.append(errors)
    values=np.array(values);curve=np.nanmedian(values,axis=1);half=[int(np.nanargmin(np.nanmedian(values[:,j::2],axis=1))) for j in range(2)];best=int(np.nanargmin(curve))
    reliable=bool(len(pairs)>=25 and abs(shifts[best])<.115 and abs(shifts[half[0]]-shifts[half[1]])<=.015)
    report=dict(pairs=len(pairs),candidate_offset_s=float(shifts[best]),reliable=reliable,
        split_offsets_s=[float(shifts[j]) for j in half],median_epipolar_error_px=float(curve[best]),
        curve=[dict(offset_s=float(s),median_error_px=float(e)) for s,e in zip(shifts,curve)],
        note='Visual/gyro epipolar consistency, not ground truth. Offset is added to raw IMU timestamps.')
    args.output.write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='curve'},indent=2))


if __name__=='__main__':main()
