#!/usr/bin/env bash
# start_router.sh — 手动启动 Zenoh Router(zenohd)
# 用法: ./mocap router；可显式运行 ./mocap router-install --start 安装服务。
#
# 安全:保留 deploy/zenoh/zenohd_acl.yaml，默认拒绝，仅放行配置声明的 key 空间。
# 仅监听本机 loopback，以及实际配置的 169.254.1.1；绝不回退通配地址。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ZENOHD="$ROOT/.pixi/envs/default/bin/zenohd"
ACL_CONFIG="$ROOT/deploy/zenoh/zenohd_acl.yaml"

if (( $# )); then
    echo "[zenohd] 用法: ./mocap router (不支持附加参数)" >&2
    exit 2
fi
for tool in ip ss; do
    command -v "$tool" >/dev/null || { echo "[zenohd] 缺少 $tool (iproute2)" >&2; exit 1; }
done

if ! [ -f "$ACL_CONFIG" ]; then
    echo "[zenohd] 错误: ACL 配置不存在: $ACL_CONFIG" >&2
    exit 1
fi

LISTENERS="$(ss -H -ltn 'sport = :7447')"
if [[ -n "$LISTENERS" ]]; then
    echo "[zenohd] TCP 7447 已被监听；不会结束或替换现有进程:" >&2
    printf '%s\n' "$LISTENERS" >&2
    echo "[zenohd] 请先确认现有实例及其 ACL 配置。" >&2
    exit 1
fi

if ! [ -x "$ZENOHD" ]; then
    echo "[zenohd] 错误: zenohd 不存在: $ZENOHD" >&2
    exit 1
fi

LISTEN_ARGS=(-l tcp/127.0.0.1:7447)
ADDRESSES="$(ip -4 -o addr show)"
while read -r _ _ family address _; do
    if [[ "$family" == inet && "${address%/*}" == 169.254.1.1 ]]; then
        LISTEN_ARGS+=(-l tcp/169.254.1.1:7447)
        break
    fi
done <<< "$ADDRESSES"

echo "[zenohd] 启动 $ZENOHD (ACL: $ACL_CONFIG)" >&2
if (( ${#LISTEN_ARGS[@]} == 4 )); then
    echo "[zenohd] 监听 tcp/127.0.0.1:7447 + tcp/169.254.1.1:7447 (本机+有线)" >&2
else
    echo "[zenohd] 未配置 169.254.1.1；仅监听 tcp/127.0.0.1:7447" >&2
fi
exec "$ZENOHD" -c "$ACL_CONFIG" "${LISTEN_ARGS[@]}"
