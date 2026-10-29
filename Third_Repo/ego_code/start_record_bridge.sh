#!/bin/bash
# 启动 record_ctrl_bridge (Python, 外部按键 ZMQ → recorder 服务)
#
# 对齐原版 exoskeleton_v2/hyper_leg_ros2/scripts/record_ctrl/record_ctrl_ros2_bridge.py
# 但默认触发 session_recorder 录制。
#
# 用法:
#   ./scripts/record/start_record_bridge.sh                          (默认 192.168.100.1:5565)
#   ./scripts/record/start_record_bridge.sh 192.168.100.10           (自定义 host)
#   ./scripts/record/start_record_bridge.sh 192.168.100.10 5566      (自定义 host + port)
#   ./scripts/record/start_record_bridge.sh --event-mode auto        (默认:自动兼容新旧相机)
#   ./scripts/record/start_record_bridge.sh --toggle                 (强制每次按键 START/STOP 交替)
#   ./scripts/record/start_record_bridge.sh --state                  (强制使用相机 payload 状态)
#   ./scripts/record/start_record_bridge.sh --no-service-wait        (只调相机信号,不等 recorder 服务)
#   ./scripts/record/start_record_bridge.sh --zmq-reconnect-check-sec 5
#                                                                  (头显断电恢复后自动重联)
#   ./scripts/record/start_record_bridge.sh --no-headset-health      (关闭头显 TCP 连接检查)
#   ./scripts/record/start_record_bridge.sh --legacy                 (原版 /start_collect /stop_collect)
#
# 前置:
#   - start_full_body.sh 已启动,session_recorder 节点在跑
#   - Python 环境有 pyzmq (系统 python3 或 pixi env 都行)

# Resolve WS_DIR / EXO_SDK_ROOT — honors $EXO_SDK_ROOT from installed SDKs,
# otherwise walks up from scripts/record/.
source "$(dirname "${BASH_SOURCE[0]}")/../lib/path_resolve.sh"
exo_resolve_root "${BASH_SOURCE[0]}"

# 不允许在普通 conda env 里跑；pixi/RoboStack 由 start_full_body_pixi.sh 进入,
# 通过 PIXI_PROJECT_ROOT + CONDA_PREFIX 标识,可以安全使用 install/local_setup.bash。
if [ -n "${CONDA_DEFAULT_ENV:-}" ] && { [ -z "${PIXI_PROJECT_ROOT:-}" ] || [ -z "${CONDA_PREFIX:-}" ]; }; then
    echo "错误: 检测到当前在 conda env '${CONDA_DEFAULT_ENV}' 中"
    echo "      本脚本只支持系统 ROS 2 humble 或 pixi env,请二选一:"
    echo ""
    echo "      ① 系统 ROS: conda deactivate 后再跑 ./scripts/record/start_record_bridge.sh ..."
    echo "      ② pixi:     ./scripts/start_full_body_pixi.sh ... 或 pixi run -- ./scripts/record/start_record_bridge.sh ..."
    exit 1
fi

source "${WS_DIR}/scripts/lib/runtime_env.sh"
exo_source_runtime_env

LEGACY=false
EVENT_MODE="auto"
SERVICE_WAIT_SEC=10.0
ZMQ_RECONNECT_CHECK_SEC=5.0
ZMQ_STALE_RECONNECT_SEC=120.0
HEADSET_HEALTH=true
HEADSET_HEALTH_INTERVAL_SEC=30.0
HOST="192.168.100.1"
PORT=5565
pos=0      # 位置参数计数器(HOST=0, PORT=1)

while [[ $# -gt 0 ]]; do
    case "$1" in
        --legacy) LEGACY=true; shift ;;
        --toggle|--event-toggle) EVENT_MODE="toggle"; shift ;;
        --state|--event-state) EVENT_MODE="state"; shift ;;
        --auto|--event-auto) EVENT_MODE="auto"; shift ;;
        --no-service-wait) SERVICE_WAIT_SEC=0; shift ;;
        --service-wait-sec) SERVICE_WAIT_SEC="$2"; shift 2 ;;
        --zmq-reconnect-check-sec) ZMQ_RECONNECT_CHECK_SEC="$2"; shift 2 ;;
        --zmq-stale-reconnect-sec) ZMQ_STALE_RECONNECT_SEC="$2"; shift 2 ;;
        --headset-health) HEADSET_HEALTH=true; shift ;;
        --no-headset-health) HEADSET_HEALTH=false; shift ;;
        --headset-health-interval-sec) HEADSET_HEALTH_INTERVAL_SEC="$2"; shift 2 ;;
        --event-mode)
            EVENT_MODE="$2"
            if [[ "$EVENT_MODE" != "auto" && "$EVENT_MODE" != "state" && "$EVENT_MODE" != "toggle" ]]; then
                echo "未知 event mode: $EVENT_MODE (应为 auto / state / toggle)"
                exit 1
            fi
            shift 2
            ;;
        --help|-h) sed -n '2,18p' "${BASH_SOURCE[0]}"; exit 0 ;;
        -*) echo "未知参数: $1"; exit 1 ;;
        *)
            case $pos in
                0) HOST="$1" ;;
                1) PORT="$1" ;;
                *) echo "多余位置参数: $1"; exit 1 ;;
            esac
            pos=$((pos+1))
            shift ;;
    esac
done

ARGS=(
    --host "$HOST"
    --port "$PORT"
    --event-mode "$EVENT_MODE"
    --service-wait-sec "$SERVICE_WAIT_SEC"
    --zmq-reconnect-check-sec "$ZMQ_RECONNECT_CHECK_SEC"
    --zmq-stale-reconnect-sec "$ZMQ_STALE_RECONNECT_SEC"
)
if [ "$HEADSET_HEALTH" = true ]; then
    ARGS+=(
        --headset-health
        --headset-health-interval-sec "$HEADSET_HEALTH_INTERVAL_SEC"
    )
else
    ARGS+=(--no-headset-health)
fi
if [ "$LEGACY" = true ]; then
    ARGS+=(
        --start-service /start_collect
        --stop-service /stop_collect
        --start-with-seq-service /unused_start_with_seq
    )
fi

echo "ZMQ PUB source: tcp://${HOST}:${PORT}"
echo "Mode          : $([ "$LEGACY" = true ] && echo "legacy (/start_collect /stop_collect)" || echo "session_recorder")"
echo "Event mode    : ${EVENT_MODE}"
echo "Service wait  : ${SERVICE_WAIT_SEC}s"
echo "ZMQ reconnect : check every ${ZMQ_RECONNECT_CHECK_SEC}s, stale rebuild ${ZMQ_STALE_RECONNECT_SEC}s"
echo "Headset health: $([ "$HEADSET_HEALTH" = true ] && echo "TCP connectivity check every ${HEADSET_HEALTH_INTERVAL_SEC}s" || echo "off")"
echo ""

# 直接 python3 运行,避免 ros2 run 对 argparse CLI 的包装
BRIDGE_PY="${WS_DIR}/src/exo_tools/scripts/record_ctrl_bridge.py"
if [ ! -f "$BRIDGE_PY" ]; then
    # 装进 install 的版本
    BRIDGE_PY="$(ros2 pkg prefix exo_tools)/lib/exo_tools/record_ctrl_bridge"
fi
exec python3 "$BRIDGE_PY" "${ARGS[@]}"
