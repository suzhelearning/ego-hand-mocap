import pickle
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from moshpp.models.smpl_fast_derivatives import load_surface_model
root=Path(__file__).resolve().parent;base=root.parents[1]
d=pickle.load(open(str(root/'work/stageii.pkl'),'rb'))
model_path=base/'hand20/assets/MANO_LEFT.pkl'
raw=pickle.load(open(str(model_path),'rb'),encoding='latin1')
m=load_surface_model(str(model_path),surface_model_type='mano',dof_per_hand=45,use_hands_mean=False)
m.betas[:]=d['betas'];inv=np.linalg.inv(raw['hands_components'][:45])
verts=[]; joints=[]
for pose,trans in zip(d['fullpose'],d['trans']):
 m.pose[:3]=pose[:3];m.pose[3:]=(pose[3:]-raw['hands_mean']).dot(inv);m.trans[:]=trans
 assert np.allclose(m.fullpose.r,pose,atol=1e-6)
 verts.append(m.r.copy())
 joints.append(np.concatenate([m.J_transformed.r[:16]+np.asarray(trans), m.r[[744,320,443,554,671]]],axis=0)[[0,13,14,15,16,1,2,3,17,4,5,6,18,10,11,12,19,7,8,9,20]])
np.savez_compressed(str(root/'hand_joints21.npz'),joints=np.asarray(joints),time_s=np.arange(len(joints))/120.,units='m',coordinate_frame='mocap_world',joint_order='wrist_thumb_index_middle_ring_little',tip_vertex_ids=np.array([744,320,443,554,671]))
verts=np.asarray(verts);assert verts.shape==(len(d['fullpose']),778,3) and np.isfinite(verts).all()
np.savez_compressed(str(root/'hand_mesh_sequence.npz'),vertices=verts,faces=m.f,frame_rate=120.,units='m')
fig=plt.figure(figsize=(15,6))
for idx,frame in enumerate([0,len(verts)//2,len(verts)-1]):
 ax=fig.add_subplot(1,3,idx+1,projection='3d');v=verts[frame]*1000;obs=np.asarray(d['stageii_debug_details']['markers_obs'][frame])*1000
 center=v.mean(axis=0);v-=center;obs-=center
 ax.add_collection3d(Poly3DCollection(v[np.asarray(m.f)],facecolor='#c6d7e8',edgecolor='#72869a',linewidth=.08,alpha=.65))
 ax.scatter(*obs.T,color='#e83b35',s=12,label='Input markers')
 sim=np.asarray(d['stageii_debug_details']['markers_sim'][frame])*1000-center
 ax.scatter(*sim.T,color='#125bbb',s=10,label='Fitted markers')
 radius=max(np.ptp(v,axis=0).max(),np.ptp(obs,axis=0).max())*.57
 for setter,c in zip([ax.set_xlim,ax.set_ylim,ax.set_zlim],[0,0,0]): setter(c-radius,c+radius)
 ax.set_box_aspect((1,1,1));ax.view_init(elev=25,azim=-60);ax.set_title('Frame %d / %.2f s'%(frame,frame/120));ax.set_xlabel('X (mm)');ax.set_ylabel('Y (mm)');ax.set_zlabel('Z (mm)')
 if idx==0: ax.legend(fontsize=7)
fig.suptitle('MANO left hand: official MoSh++ pipeline; Motive input; missing samples retained')
fig.tight_layout();fig.savefig(str(root/'fit_preview.png'),dpi=150)
print('Mesh sequence and preview saved')
