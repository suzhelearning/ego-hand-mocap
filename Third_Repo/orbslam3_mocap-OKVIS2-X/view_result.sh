#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "$0")" && pwd)
if [ "$#" -ne 1 ]; then
  echo "用法：$0 数据集文件夹或运行结果文件夹" >&2
  exit 2
fi
folder=$(realpath -e -- "$1")
result=$(python3 - "$repo" "$folder" <<'PY'
from pathlib import Path
import json
import sys
repo, folder = map(Path, sys.argv[1:])
search_source=folder
adapter=folder/'adapter_report.json'
if adapter.is_file():
    search_source=Path(json.loads(adapter.read_text())['source']).resolve()
def has_pose(path):
    return path.is_dir() and any(path.glob('okvis2-*_trajectory.csv'))
if has_pose(folder):
    chosen=folder
else:
    matches=[]
    candidates=list((search_source/'slam/runs').glob('*'))
    candidates += list((folder/'runs').glob('*')) if folder.name=='slam' else []
    candidates += list((repo/'results').glob('*'))
    for p in candidates:
        manifest=p/'viewer_manifest.json'
        if not has_pose(p) or not manifest.is_file():
            continue
        try:
            dataset=Path(json.loads(manifest.read_text())['dataset'])
            source=Path(json.loads((dataset/'adapter_report.json').read_text())['source']).resolve()
            if source == search_source or dataset.resolve() == folder or p.parent == folder/'runs':
                matches.append(p)
        except (OSError, ValueError, KeyError):
            continue
    if matches:
        chosen=max(matches,key=lambda p:p.stat().st_mtime)
    elif has_pose(folder/'results'):
        chosen=folder/'results'
    else:
        chosen=repo/'datasets'/(folder.name+'_pinhole')/'results'
if not has_pose(chosen):
    raise SystemExit('没有找到轨迹，请传入包含 okvis2-slam 轨迹CSV的运行结果目录。')
print(chosen)
PY
)
exec python3 "$repo/tools/head_dataset/viewer.py" --result "$result"
