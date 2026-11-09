#!/usr/bin/env python3
"""Render a saved EMG capture as a local signal-quality report (not sync validation)."""
import argparse
import html
import json
from pathlib import Path
import h5py
import numpy as np


def report(folder):
    folder=Path(folder)
    if (folder/'emg').is_dir():folder=folder/'emg'
    parts=['<!doctype html><meta charset="utf-8"><title>肌电采集检查</title><style>body{font:16px sans-serif;max-width:1200px;margin:30px auto;background:#fafafa;color:#222}section{background:white;padding:20px;margin:20px 0}.grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}svg{width:100%;height:90px;background:#eef3f7}pre{white-space:pre-wrap}.warn{color:#b42318}</style><h1>肌电采集检查</h1><p>原始 ADC 波形包络；每通道独立纵轴。时间仅为单向粗映射，不能用本页证明精确同步。</p>']
    for side,label in [('left','左手'),('right','右手')]:
        path=folder/(side+'.h5')
        if not path.exists():continue
        with h5py.File(path) as f:
            if 'emg' not in f:continue
            ds=f['emg'];n=len(ds)
            if not n:continue
            edges=np.linspace(0,n,min(500,n)+1,dtype=int)
            lows=[];highs=[];sat=np.zeros(16,dtype=np.int64)
            for a,b in zip(edges[:-1],edges[1:]):
                values=ds[a:b,7:23];lows.append(values.min(axis=0));highs.append(values.max(axis=0));sat+=np.sum((values==-8388608)|(values==8388607),axis=0)
            lows=np.array(lows);highs=np.array(highs)
            duration=(int(ds[-1,3])-int(ds[0,3]))/1e6
            model=json.loads(f.attrs.get('clock_model_json','{}'))
            parts.append(f'<section><h2>{label}</h2><p>{n} 帧；设备时间跨度 {duration:.3f} 秒；平均 {(n-1)/duration if duration>0 else 0:.2f} Hz</p>')
            parts.append('<p class="warn">达到 ADC 极值的样本比例：'+', '.join(f'CH{i+1}: {v/n:.1%}' for i,v in enumerate(sat))+'</p><div class="grid">')
            for ch in range(16):
                low=int(lows[:,ch].min());high=int(highs[:,ch].max());span=max(1,high-low)
                points=[]
                for i in range(len(lows)):
                    x=500*i/max(1,len(lows)-1);y1=80-70*(lows[i,ch]-low)/span;y2=80-70*(highs[i,ch]-low)/span
                    points.append(f'M{x:.2f},{y1:.2f}L{x:.2f},{y2:.2f}')
                parts.append(f'<div>CH{ch+1}：{low} ～ {high}<svg viewBox="0 0 500 90"><path d="'+''.join(points)+'" fill="none" stroke="#1672b8" stroke-width="1"/></svg></div>')
            parts.append('</div><details><summary>粗时间映射参数</summary><pre>'+html.escape(json.dumps(model,ensure_ascii=False,indent=2))+'</pre></details></section>')
    path=folder/'quality.html';path.write_text(''.join(parts),encoding='utf8');return path


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path)
    print(report(p.parse_args().folder).resolve())
