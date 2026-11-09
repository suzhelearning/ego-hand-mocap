#!/usr/bin/env python3
"""Launch the current EMG UI with raw Ego/Motive recording and shared clocks."""
import argparse
import os
from pathlib import Path
import shlex

ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(add_help=False)
p.add_argument('--emg-ui-root',type=Path,default=Path(os.environ.get('EMG_UI_ROOT','/data/emg_glove_ws_ego_preview_20261105')))
args,rest=p.parse_known_args()
ui=args.emg_ui_root.resolve()
if not (ui/'src/emg_glove_bringup/emg_glove_bringup/trial_joint_capture.py').exists():
    raise SystemExit('当前 UI 缺少联合录制模块，请指定 --emg-ui-root')
env=dict(os.environ)
env['PYTHONPATH']=str(ROOT/'src')+os.pathsep+env.get('PYTHONPATH','')
command=[str(ui/'.pixi/envs/default/bin/python'),str(ui/'scripts/visual_trial.py'),
         '--capture-mode','mocap','--semg','current','--joint-config',str(ROOT/'configs/network/lab.json'),
         '--output-root',str(ROOT/'data/joint_ui'),*rest]
script='source '+shlex.quote(str(ui/'.pixi/envs/default/setup.bash'))+'\nsource '+shlex.quote(str(ui/'install/setup.bash'))+'\nexec '+shlex.join(command)
os.chdir(ui)
os.execve('/bin/bash',['bash','-c',script],env)
