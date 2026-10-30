#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
export PYTHONPATH="$ROOT"
export PYTHONDONTWRITEBYTECODE=1
export PYOPENGL_PLATFORM=egl
export TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1
export CUBLAS_WORKSPACE_CONFIG=:4096:8
export YOLO_AUTOINSTALL=false
exec "$ROOT/.venvs/anyhand/bin/python" -m deployment.infer "$@"
