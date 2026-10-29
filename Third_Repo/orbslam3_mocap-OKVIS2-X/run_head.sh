#!/usr/bin/env bash
# Run an existing head export or export a new head-v3 dataset, then run OKVIS.
set -euo pipefail
repo=$(cd "$(dirname "$0")" && pwd)
usage() {
  cat <<EOF
用法：
  $repo/run_head.sh /完整路径/head_数据集目录

例如：
  $repo/run_head.sh /数据集/head_000009
  $repo/run_head.sh /数据集/head_000010

若已有匹配的数据转换缓存，自动复用；否则先导出 H5+MP4。
也支持直接传入包含 okvis.yaml 和 mav0 的导出目录。
每次结果保存在输入数据集的 slam/runs 下的新目录，与 video/ 同级。
新转换缓存放在 slam/exports 中；已有缓存可继续复用。
默认打开相机特征匹配窗口和可拖动的三维浏览器界面。
运行结束后保存 replay_3d.html，可离线查看。
EOF
}
if [ "$#" -eq 1 ] && { [ "$1" = '-h' ] || [ "$1" = '--help' ]; }; then
  usage; exit 0
fi
if [ "$#" -ne 1 ]; then
  usage >&2; exit 2
fi
if [ ! -x "$repo/build/okvis_app_synchronous" ]; then
  echo '缺少可执行程序，先运行：' >&2
  echo "bash $repo/tools/head_dataset/build.sh" >&2
  exit 1
fi
source_dir=$(realpath -e -- "$1")
slam_source="$source_dir"
if [ -f "$source_dir/adapter_report.json" ] && [ -d "$source_dir/mav0" ]; then
  slam_source=$(python3 - "$source_dir" <<'PY'
import json, sys
from pathlib import Path
folder=Path(sys.argv[1])
source=Path(json.loads((folder/'adapter_report.json').read_text())['source']).resolve()
if not source.is_dir():
    raise SystemExit('原始数据集目录不存在：'+str(source))
print(source)
PY
  )
fi
slam_root="$slam_source/slam"
    if [ ! -d "$source_dir" ]; then
      echo "数据集必须是目录：$source_dir" >&2; exit 2
    fi
    if [ -f "$source_dir/okvis.yaml" ] && [ -d "$source_dir/mav0" ]; then
      dataset="$source_dir"
    else
      # Reuse only an export whose recorded source is this exact directory.
      dataset=$(python3 - "$repo" "$source_dir" <<'PY'
import json
from pathlib import Path
import sys
repo, source = map(Path, sys.argv[1:])
reports = sorted((source/'slam/exports').glob('*/adapter_report.json'))
reports += sorted((repo/'datasets').glob(source.name+'_pinhole*/adapter_report.json'))
# Only use tuned parameters for a source that has completed validation.
preferences = repo/'tools/head_dataset/profile_preferences.json'
if preferences.is_file():
    settings = json.loads(preferences.read_text())
    preferred = settings.get('sources', {}).get(str(source), {}).get('export')
    if preferred:
        reports.insert(0, repo/preferred/'adapter_report.json')
for report in reports:
    try:
        metadata = json.loads(report.read_text())
        candidate = report.parent
        if (Path(metadata['source']).resolve() == source and
            metadata.get('projection') == 'pinhole' and
            (candidate/'okvis.yaml').is_file() and
            (candidate/'mav0').is_dir()):
            print(candidate)
            break
    except (OSError, ValueError, KeyError):
        continue
PY
      )
      if [ -n "$dataset" ]; then
        echo "复用转换缓存：$dataset"
      else
      mkdir -p "$slam_root/exports"
      # Reserve a unique name; the exporter itself requires a nonexistent path.
      dataset=$(mktemp -d "$slam_root/exports/$(basename "$source_dir")_pinhole_XXXXXX")
      rmdir "$dataset"
      python3 "$repo/tools/head_dataset/head_to_okvis.py" \
        --dataset "$source_dir" --output "$dataset" --acc-noise 0.05
      fi
    fi
if [ ! -f "$dataset/okvis.yaml" ] || [ ! -d "$dataset/mav0" ]; then
  echo "缺少已导出的数据：$dataset" >&2; exit 1
fi
mkdir -p "$slam_root/runs"
result=$(mktemp -d "$slam_root/runs/$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_XXXXXX")
echo "输入：$dataset"
echo "结果：$result"
echo '开始运行：打开相机特征匹配窗口和三维浏览器界面，终端同时显示进度。'
OKVIS_GUI=1 bash "$repo/tools/head_dataset/run.sh" "$dataset" "$result"
echo
echo "完成，结果目录：$result"
echo "轨迹：$result/trajectory_imu.tum"
echo "统计：$result/summary.json"
echo "日志：$result/run.log"
echo "离线三维回放：$result/replay_3d.html"
