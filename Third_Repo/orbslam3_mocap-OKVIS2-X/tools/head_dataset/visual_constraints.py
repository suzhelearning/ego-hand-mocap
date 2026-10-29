#!/usr/bin/env python3
"""Independent masked SIFT stereo/PnP constraints and visual/gyro timing checks.

These are consistency measurements from the same calibrated images, not ground truth.
"""
import argparse
import csv
import json
from pathlib import Path
import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation, Slerp

cv2.setNumThreads(2)
cv2.setRNGSeed(12345)


class Measurements:
    def __init__(self, dataset, mask_dataset=None):
        self.dataset=dataset
        self.config=yaml.safe_load((dataset/'okvis.yaml').read_text().split('\n',1)[1])
        self.cams=self.config['cameras'];self.K=[];self.T=[];self.masks=[]
        for k,c in enumerate(self.cams):
            fu,fv=c['focal_length'];cu,cv=c['principal_point'];self.K.append(np.array([[fu,0,cu],[0,fv,cv],[0,0,1.]]));self.T.append(np.array(c['T_SC']).reshape(4,4))
            self.masks.append(cv2.imread(str(mask_dataset/'masks'/f'cam{k}.png'),0) if mask_dataset else None)
        self.right_from_left=np.linalg.inv(self.T[1])@self.T[0]
        self.files=[list(csv.reader((dataset/f'mav0/cam{k}/data.csv').open()))[1:] for k in range(2)]
        self.sift=cv2.SIFT_create(nfeatures=3000,contrastThreshold=.015)
        self.matcher=cv2.BFMatcher();self.cache={}

    def features(self,k,index):
        key=(k,index)
        if key not in self.cache:
            image=cv2.imread(str(self.dataset/f'mav0/cam{k}/data'/self.files[k][index][1]),0)
            kp,desc=self.sift.detectAndCompute(image,self.masks[k]);self.cache[key]=(np.array([p.pt for p in kp],np.float32),desc)
        return self.cache[key]

    def matches(self,a,b):
        if a is None or b is None:return []
        forward=self.matcher.knnMatch(a,b,k=2);reverse=self.matcher.knnMatch(b,a,k=2)
        back={m.queryIdx:m.trainIdx for m,n in reverse if m.distance<.75*n.distance}
        return [(m.queryIdx,m.trainIdx) for m,n in forward if m.distance<.75*n.distance and back.get(m.trainIdx)==m.queryIdx]

    def stereo(self,index):
        p0,d0=self.features(0,index);p1,d1=self.features(1,index);pairs=self.matches(d0,d1)
        if len(pairs)<10:return None
        ids=np.array(pairs);uv0=p0[ids[:,0]];uv1=p1[ids[:,1]]
        P0=self.K[0]@np.eye(4)[:3];P1=self.K[1]@self.right_from_left[:3]
        hp=cv2.triangulatePoints(P0,P1,uv0.T,uv1.T).T;points=hp[:,:3]/hp[:,3:4]
        predicted0=(P0@np.c_[points,np.ones(len(points))].T).T;predicted1=(P1@np.c_[points,np.ones(len(points))].T).T
        err0=np.linalg.norm(predicted0[:,:2]/predicted0[:,2:3]-uv0,axis=1);err1=np.linalg.norm(predicted1[:,:2]/predicted1[:,2:3]-uv1,axis=1)
        valid=np.isfinite(points).all(axis=1)&(points[:,2]>.3)&(points[:,2]<15)&(predicted1[:,2]>.2)&(err0<1.5)&(err1<1.5)
        return points[valid],d0[ids[:,0][valid]],uv0[valid]

    def pnp(self,anchor,target,stereo=None):
        stereo=stereo if stereo is not None else self.stereo(anchor)
        if stereo is None or len(stereo[0])<8:return None
        pts,desc,anchor_uv=stereo;uv,tdesc=self.features(0,target);pairs=self.matches(desc,tdesc)
        if len(pairs)<8:return None
        ids=np.array(pairs);obj=pts[ids[:,0]];img=uv[ids[:,1]]
        ok,r,t,inliers=cv2.solvePnPRansac(obj,img,self.K[0],None,iterationsCount=300,reprojectionError=1.5,confidence=.999,flags=cv2.SOLVEPNP_EPNP)
        if not ok or inliers is None or len(inliers)<8:return None
        ii=inliers[:,0];r,t=cv2.solvePnPRefineLM(obj[ii],img[ii],self.K[0],None,r,t)
        if cv2.contourArea(cv2.convexHull(img[ii]))<1000:return None
        projected=cv2.projectPoints(obj[ii],r,t,self.K[0],None)[0][:,0]
        pose=np.eye(4);pose[:3,:3]=cv2.Rodrigues(r)[0];pose[:3,3]=t[:,0]
        return dict(anchor=anchor,target=target,anchor_ns=int(self.files[0][anchor][0]),target_ns=int(self.files[0][target][0]),
                    inliers=len(ii),matches=len(ids),rms_px=float(np.sqrt(np.mean(np.sum((projected-img[ii])**2,axis=1)))),
                    current_from_anchor=pose.tolist(),positions_px=img[ii].tolist())


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dataset',type=Path,required=True);p.add_argument('--mask-dataset',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--timing',action='store_true');args=p.parse_args();m=Measurements(args.dataset,args.mask_dataset)
    n=min(map(len,m.files));anchors=[0,n//4,n//2,3*n//4];observations=[]
    for anchor in anchors:
        stereo=m.stereo(anchor)
        for target in sorted(set([n-1,*range(0,n,300)])):
            if target==anchor:continue
            row=m.pnp(anchor,target,stereo)
            if row and row['inliers']>=8 and row['rms_px']<1:observations.append(row)
        print('anchor',anchor,'stereo',0 if stereo is None else len(stereo[0]),'constraints',len(observations),flush=True)
    report={'dataset':str(args.dataset),'mask_dataset':str(args.mask_dataset),'global_constraints':observations}
    if args.timing:
        local=[]
        for i in range(30,n-6,60):
            row=m.pnp(i,i+3)
            if row and row['inliers']>=25 and row['rms_px']<1:
                T=np.asarray(row['current_from_anchor'])
                if np.linalg.norm(T[:3,3])<.3:local.append(row)
        imu=np.loadtxt(args.dataset/'mav0/imu0/data.csv',delimiter=',',skiprows=1)
        old_offset=json.loads((args.dataset/'adapter_report.json').read_text())['imu_offset_s']
        t=imu[:,0]*1e-9-old_offset;quats=[];orientation=Rotation.identity();quats.append(orientation.as_quat())
        for i in range(1,len(t)):
            orientation=orientation*Rotation.from_rotvec((imu[i-1,1:4]+imu[i,1:4])*.5*(t[i]-t[i-1]));quats.append(orientation.as_quat())
        interp=Slerp(t,Rotation.from_quat(quats));RSC=m.T[0][:3,:3];scores=[]
        ta=np.array([o['anchor_ns']*1e-9 for o in local]);tb=np.array([o['target_ns']*1e-9 for o in local]);measured=np.array([np.array(o['current_from_anchor'])[:3,:3] for o in local])
        for offset in np.linspace(-.08,.08,81):
            valid=(ta-offset>=t[0])&(tb-offset<=t[-1]);Ra=interp(ta[valid]-offset).as_matrix();Rb=interp(tb[valid]-offset).as_matrix()
            pred=RSC.T@np.swapaxes(Rb,1,2)@Ra@RSC;errors=Rotation.from_matrix(np.swapaxes(measured[valid],1,2)@pred).magnitude()*180/np.pi
            scores.append({'offset_s':float(offset),'median_error_deg':float(np.median(errors)),'p75_error_deg':float(np.percentile(errors,75)),'count':len(errors)})
        candidate=min(scores,key=lambda s:s['median_error_deg'])
        reliable=len(local)>=30 and abs(candidate['offset_s'])<.076
        report.update(local_rotation_constraints=local,timing_scores=scores,timing_candidate=candidate,
                      timing_best=candidate if reliable else None,
                      timing_reliable=reliable)
        print('local constraints',len(local),'best',report['timing_best'],flush=True)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
