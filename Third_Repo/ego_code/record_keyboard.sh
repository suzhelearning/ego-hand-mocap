#!/bin/bash
#
# 键盘热键 HDF5 录制控制 (a/s 热键)
#
# 控制单 session_recorder_node — 一次 a 起、一次 s 停。落盘:
#   recordings/<seq>_<ts>.hdf5   (单文件多层 /raw /calibrated /keypoints)
#
# 前提: session_recorder_node 已运行 (start_full_body.sh 默认起好)。
#
# 按键:
#   a  →  开始录制
#   s  →  停止保存
#   t  →  查询 recorder 状态
#   q  →  退出脚本 (不影响录制状态)
#
# 旧 r/k 按键已移除 — 三层录制现在是单节点,无法运行时只关一层。
# 想关某一层请用 start_full_body.sh --no-{raw,cal,kp}-layer。
#
# 用法:
#   cd ~/catkin_exoskeleton_ws
#   ./scripts/record/record_keyboard.sh

# 不要启用 set -e —— 交互脚本里 grep 未匹配等无害错误会误杀进程
source "$(dirname "${BASH_SOURCE[0]}")/../lib/path_resolve.sh"
exo_resolve_root "${BASH_SOURCE[0]}"

# pixi 模式自动检测:PIXI_PROJECT_ROOT 由 pixi run / pixi shell 注入。
# - 在 pixi 里 → 走 pixi env 的 ROS Humble,跳过 /opt/ros/humble
# - 不在 pixi 里且在 conda env 里 → 拒绝(conda 的 PATH 会让 ros2 cli 找不到)
if [ -z "${PIXI_PROJECT_ROOT:-}" ] && [ -n "${CONDA_DEFAULT_ENV:-}" ]; then
    echo "错误: 检测到当前在 conda env '${CONDA_DEFAULT_ENV}' 中(且不在 pixi 里)"
    echo "      系统 ROS 模式不能在 conda 里跑,请二选一:"
    echo ""
    echo "      ① 系统 ROS:  conda deactivate 后再跑 ./scripts/record/record_keyboard.sh"
    echo "      ② pixi:      ./scripts/record/record_keyboard_pixi.sh"
    exit 1
fi

source "${WS_DIR}/scripts/lib/runtime_env.sh"
exo_source_runtime_env

OLD_STTY=""

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
BOLD='\033[1m'
NC='\033[0m'

# session_recorder 服务名
SESSION_START=/session_recorder/start
SESSION_STOP=/session_recorder/stop
SESSION_STATUS=/session_recorder/status

SESSION_ON=false

restore_terminal() {
    if [ -n "${OLD_STTY:-}" ]; then
        stty "$OLD_STTY" 2>/dev/null || stty sane 2>/dev/null || true
    else
        stty sane 2>/dev/null || true
    fi
}

check_service() {
    local svc="$1"
    local svcs
    svcs=$(ros2 service list 2>/dev/null)
    echo "$svcs" | grep -q "^${svc}$"
}

check_prereq() {
    local ok=true
    for s in "$SESSION_START" "$SESSION_STOP"; do
        if ! check_service "$s"; then
            echo -e "${RED}缺少服务: ${s}${NC}"
            ok=false
        fi
    done
    if [ "$ok" = false ]; then
        echo ""
        echo -e "${YELLOW}请先启动 session_recorder:${NC}"
        echo "  ros2 run exo_sync session_recorder_node &"
        echo "或运行 ./scripts/start_full_body.sh (默认启动)"
        exit 1
    fi
}

call_service() {
    local svc="$1"
    local label="$2"
    local color="$3"
    local out ret success message
    out=$(timeout 5 ros2 service call "$svc" std_srvs/srv/Trigger "{}" 2>&1)
    ret=$?
    if [ $ret -eq 124 ]; then
        echo -e "  ${color}[${label}]${NC} ${RED}TIMEOUT${NC}"
        return 0
    fi
    # grep 未匹配返回非零,全都吞掉不要因此失败
    success=$(echo "$out" | grep -oE 'success=[a-zA-Z]+' 2>/dev/null | head -1 | cut -d= -f2 2>/dev/null || true)
    message=$(echo "$out" | grep -oE "message='[^']*'" 2>/dev/null | head -1 | sed "s/message='//;s/'$//" 2>/dev/null || true)
    if [ "$success" = "True" ]; then
        echo -e "  ${color}[${label}]${NC} ${GREEN}OK${NC}  ${message}"
    elif [[ "$message" == "already recording" || "$message" == "not recording" ]]; then
        # 幂等无副作用的状态,当作提示即可
        echo -e "  ${color}[${label}]${NC} ${YELLOW}SKIP${NC} ${message}"
    else
        echo -e "  ${color}[${label}]${NC} ${RED}FAIL${NC}  ${message:-$out}"
    fi
    return 0
}

print_status() {
    local topic="$1"
    local label="$2"
    local color="$3"
    local out
    out=$(timeout 2 ros2 topic echo "$topic" std_msgs/msg/String --once 2>/dev/null | grep "data:" | head -1 2>/dev/null || true)
    if [ -n "$out" ]; then
        echo -e "  ${color}[${label}]${NC} ${out}"
    else
        echo -e "  ${color}[${label}]${NC} ${YELLOW}(无数据)${NC}"
    fi
}

session_status() {
    print_status "$SESSION_STATUS" "session" "$CYAN"
}

banner() {
    clear
    echo -e "${CYAN}╔══════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║      HDF5 录制控制 (单 session_recorder)             ║${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${BOLD}按键:${NC}"
    echo -e "  ${GREEN}a${NC}  开始录制"
    echo -e "  ${RED}s${NC}  停止保存"
    echo -e "  ${YELLOW}t${NC}  查询 recorder 状态"
    echo -e "  ${YELLOW}q${NC}  退出 (不影响录制)"
    echo ""
    echo -e "${BOLD}输出:${NC}"
    echo -e "     ${WS_DIR}/recordings/<seq>_<ts>.hdf5"
    echo -e "     (group /raw /calibrated /keypoints — 启动时 yaml 决定哪些 layer)"
    echo -e "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
}

main() {
    check_prereq
    banner

    OLD_STTY=$(stty -g)
    # EXIT 负责终端复位;INT/TERM 显式 exit 触发 EXIT 链,不要只打印
    trap 'restore_terminal; echo -e "\n${YELLOW}exit${NC}"' EXIT
    trap 'exit 130' INT TERM
    stty cbreak -echo

    while true; do
        local ch
        IFS= read -r -n 1 ch
        case "$ch" in
            a|A)
                echo ""
                echo -e "${YELLOW}> a: 启动 recorder${NC}"
                call_service "$SESSION_START" "session" "$CYAN" && SESSION_ON=true
                ;;
            s|S)
                echo ""
                echo -e "${YELLOW}> s: 停止 recorder 并保存${NC}"
                call_service "$SESSION_STOP"  "session" "$CYAN" && SESSION_ON=false
                ;;
            t|T)
                echo ""
                echo -e "${YELLOW}> t: 状态${NC}"
                session_status
                ;;
            q|Q|$'\e'|$'\x03')
                break
                ;;
            *) ;;
        esac
    done
}

main "$@"
