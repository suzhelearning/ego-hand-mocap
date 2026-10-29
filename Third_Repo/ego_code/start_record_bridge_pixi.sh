#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

if [ ! -f "${WS_DIR}/pixi.toml" ]; then
  echo "错误: 未找到 ${WS_DIR}/pixi.toml" >&2
  exit 1
fi

cd "${WS_DIR}"
if [ -n "${ROS_DOMAIN_ID:-}" ]; then
  export EXO_REQUESTED_ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"
fi
exec pixi run --manifest-path "${WS_DIR}/pixi.toml" -- bash "${WS_DIR}/scripts/record/start_record_bridge.sh" "$@"
