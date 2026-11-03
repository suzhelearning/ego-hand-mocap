import json,pickle
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from moshpp.mosh_head import MoSh
root=Path(__file__).resolve().parent
inp=np.load(str(root/'capture.npz'));n=len(inp['markers']);rate=float(inp['frame_rate'])
with (root/'work/stageii.pkl').open('rb') as f:d=pickle.load(f)
for key,shape in [('fullpose',(n,48)),('trans',(n,3)),('betas',(10,)),('markers_latent',(20,3))]:
 a=np.asarray(d[key]);assert a.shape==shape,(key,a.shape);assert np.isfinite(a).all(),key
extra=d['stageii_debug_details'];labels=list(d['latent_labels']);err=np.full((n,20),np.nan)
assert len(extra['markers_sim'])==len(extra['markers_obs'])==n
for t,(sim,obs,names) in enumerate(zip(extra['markers_sim'],extra['markers_obs'],extra['labels_obs'])):
 sim=np.asarray(sim);obs=np.asarray(obs);assert sim.shape==obs.shape==(len(names),3)
 assert np.isfinite(sim).all() and np.isfinite(obs).all()
 for name,val in zip(names,np.linalg.norm(sim-obs,axis=-1)*1000):err[t,labels.index(name)]=val
missing=~np.isfinite(inp['markers']).all(axis=2);assert int(np.isnan(err).sum())==int(missing.sum())
f,j=np.unravel_index(np.nanargmax(err),err.shape)
r={'frames':n,'fps':rate,'missing_samples':int(missing.sum()),'mean_marker_distance_mm':float(np.nanmean(err)),'median_marker_distance_mm':float(np.nanmedian(err)),'p95_marker_distance_mm':float(np.nanpercentile(err,95)),'max_marker_distance_mm':float(np.nanmax(err)),'max_error_frame_zero_based':int(f),'max_error_time_seconds':float(f/rate),'max_error_label':labels[j],'frames_with_error_above_5mm':np.where(np.nanmax(err,axis=1)>5)[0].tolist(),'betas':np.asarray(d['betas']).tolist(),'per_marker_mean_mm':dict(zip(labels,np.nanmean(err,axis=0).tolist())),'stagei_seconds':float(d['stagei_debug_details']['stagei_elapsed_time']),'stageii_seconds':float(extra['stageii_elapsed_time']),'caveat':'Distances are relative to exported Motive trajectories, not ground truth; missing observations excluded. Physical marker offset is an unmeasured 1 mm assumption.'}
(root/'quality_report.json').write_text(json.dumps(r,indent=2));np.save(str(root/'marker_distances_mm.npy'),err)
MoSh.load_as_amass_npz(str(root/'work/stageii.pkl'),str(root/(root.name+'_mano.npz')),include_markers=True)
with np.load(str(root/(root.name+'_mano.npz')),allow_pickle=True) as out:assert out['poses'].shape==(n,48);assert out['trans'].shape==(n,3)
fig,ax=plt.subplots(2,1,figsize=(11,8),constrained_layout=True);t=np.arange(n)/rate
ax[0].plot(t,np.nanmean(err,axis=1));ax[0].set(xlabel='Time (s)',ylabel='Mean marker distance (mm)',title='Official MoSh++ fit; missing observations excluded')
ax[1].bar(np.arange(20),np.nanmean(err,axis=0));ax[1].set_xticks(np.arange(20));ax[1].set_xticklabels(labels,rotation=75,ha='right',fontsize=8);ax[1].set_ylabel('Mean marker distance (mm)')
fig.savefig(str(root/'fit_quality.png'),dpi=150);plt.close(fig);print(json.dumps(r,indent=2))
