from pathlib import Path
from omegaconf import OmegaConf
from moshpp.mosh_head import MoSh
from moshpp.chmosh import mosh_stagei, mosh_stageii
root=Path(__file__).resolve().parent
cfg=OmegaConf.to_container(OmegaConf.load(str(root/'config.json')),resolve=False)
mp=MoSh(dict_cfg=cfg)
for section,keys in {'mocap':['exclude_markers','exclude_marker_types'],'moshpp':['betas_fname','v_template_fname','pose_body_prior_fname','pose_hand_prior_fname','head_marker_corr_fname']}.items():
 for key in keys:
  if str(mp.cfg[section][key]).lower() in ('none','null',''): mp.cfg[section][key]=None
mp.mosh_stagei(mosh_stagei)
mp.mosh_stageii(mosh_stageii)
print('STAGE I AND II COMPLETE',flush=True)
