#!/usr/bin/env bash
#
# pixi 版录制控制脚本(RK3588 / aarch64 部署用)
#
# 把 record_keyboard.sh 在 pixi env 里跑起来。所有按键(a/s/r/k/t/q)行为一致。
# 原脚本通过 PIXI_PROJECT_ROOT 自动检测, 跳过 /opt/ros/humble。
#
# 前提: start_full_body_pixi.sh 已启动且 raw/cal/keypoints recorder 都在跑
#
# 用法:
#   cd ~/project/catkin_exoskeleton_ws
#   ./scripts/record/record_keyboard_pixi.sh

set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

if [ ! -f "${WS_DIR}/pixi.toml" ]; then
    echo "错误: 未找到 ${WS_DIR}/pixi.toml" >&2
    exit 1
fi

if [ ! -d "${WS_DIR}/.pixi/envs/default" ]; then
    echo "错误: pixi env 未安装,先跑: cd ${WS_DIR} && pixi install" >&2
    exit 1
fi

cd "${WS_DIR}"
exec pixi run --manifest-path "${WS_DIR}/pixi.toml" -- bash "${WS_DIR}/scripts/record/record_keyboard.sh" "$@"
