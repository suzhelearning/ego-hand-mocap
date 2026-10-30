#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
export PATH="$HOME/.local/bin:$PATH"
command -v uv >/dev/null
command -v ffmpeg >/dev/null
if [[ ! -x .venvs/anyhand/bin/python ]]; then
  uv venv --python 3.10 .venvs/anyhand
fi
uv --config-file deployment/uv.toml pip install --python .venvs/anyhand/bin/python \
  -r requirements.txt --extra-index-url https://download.pytorch.org/whl/cu128 --index-strategy unsafe-best-match
uv pip check --python .venvs/anyhand/bin/python
python3 tools/check_assets.py
.venvs/anyhand/bin/python -m deployment.prepare_anyhand
./run_benchmark.sh --help
