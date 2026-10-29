#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "$0")/../.." && pwd)
if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
  echo "Usage: $0 exported-dataset-directory [result-directory]" >&2
  exit 2
fi
dataset=$(realpath "$1")
if [ "$#" -eq 2 ]; then
  result="$2"
else
  source_dir=$(python3 - "$dataset/adapter_report.json" <<'PY'
import json, sys
from pathlib import Path
source=Path(json.loads(Path(sys.argv[1]).read_text())['source']).resolve()
if not source.is_dir():
    raise SystemExit('原始数据集目录不存在：'+str(source))
print(source)
PY
  )
  mkdir -p "$source_dir/slam/runs"
  result=$(mktemp -d "$source_dir/slam/runs/$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_XXXXXX")
fi
mkdir -p "$result"
result=$(realpath "$result")
python3 - "$dataset" "$result" <<'PY'
from pathlib import Path
import json, sys
dataset, result = map(Path, sys.argv[1:])
(result/'viewer_manifest.json').write_text(json.dumps({'dataset':str(dataset),'schema':1})+'\n')
PY
config="$dataset/okvis.yaml"
if [ "${OKVIS_GUI:-0}" = 1 ]; then
  if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    echo '未连接桌面显示服务，请在图形桌面的终端运行。' >&2
    exit 1
  fi
  config="$result/okvis_gui.yaml"
  python3 - "$dataset/okvis.yaml" "$config" <<'PY'
from pathlib import Path
import re
import sys
text = Path(sys.argv[1]).read_text()
for key,value in [('display_matches','true'),('display_topview','false')]:
    text, count = re.subn(r'(?m)^(\s*' + key + r':)[^\n]*', lambda m: m[1]+' '+value, text)
    if count != 1:
        raise ValueError(f'expected one config entry: {key}')
Path(sys.argv[2]).write_text(text)
PY
  nohup setsid -f python3 "$repo/tools/head_dataset/viewer.py" --result "$result" --port 0 --no-open \
    --url-file "$result/viewer_url.txt" --pid-file "$result/viewer.pid" \
    < /dev/null > "$result/viewer.log" 2>&1 &
  for attempt in {1..30}; do
    if [ -s "$result/viewer_url.txt" ]; then break; fi
    sleep 0.1
  done
  if [ -s "$result/viewer_url.txt" ]; then
    viewer_url=$(cat "$result/viewer_url.txt")
    echo "三维窗口：$viewer_url"
    xdg-open "$viewer_url" > "$result/browser_open.log" 2>&1 &
  else
    echo "三维服务未就绪，详情见 $result/viewer.log" >&2
  fi
fi
if [ -n "${CONDA_PREFIX:-}" ]; then
  export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"
else
  export LD_LIBRARY_PATH="$(dirname "$repo")/okvis2-local-deps/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
fi
export OMP_NUM_THREADS=${OMP_NUM_THREADS:-2}
export OPENBLAS_NUM_THREADS=${OPENBLAS_NUM_THREADS:-1}
cd "$result"
"$repo/build/okvis_app_synchronous" "$config" "$dataset/mav0" "$result" 2>&1 | tee "$result/run.log"
python3 "$repo/tools/head_dataset/summarize_run.py" "$dataset" "$result"
python3 "$repo/tools/head_dataset/viewer.py" --result "$result" --export
