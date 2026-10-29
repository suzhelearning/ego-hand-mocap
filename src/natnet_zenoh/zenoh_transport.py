"""Zenoh peer configuration and frame publication helpers."""

from __future__ import annotations

import json
from collections.abc import Mapping

import zenoh

from .schema import encode_frame


def build_peer_config(
    connect_endpoint: str | None = None,
    listen_endpoint: str | None = None,
) -> zenoh.Config:
    """Build an explicit TCP peer config with automatic discovery disabled."""

    if (connect_endpoint is None) == (listen_endpoint is None):
        raise ValueError("exactly one connect_endpoint or listen_endpoint is required")

    config: dict[str, object] = {
        "mode": "peer",
        "scouting": {
            "multicast": {"enabled": False},
            "gossip": {"enabled": False},
        },
    }
    if connect_endpoint is not None:
        config["connect"] = {
            "endpoints": [connect_endpoint],
            "timeout_ms": 5000,
            "exit_on_failure": True,
        }
    else:
        config["listen"] = {
            "endpoints": [listen_endpoint],
            "timeout_ms": 5000,
            "exit_on_failure": True,
        }
    return zenoh.Config.from_json5(json.dumps(config))


def build_client_config(connect_endpoint: str) -> zenoh.Config:
    """Build an explicit TCP client config (for the zenohd Router).

    Router(本机 zenohd custom build)对「关闭 scouting 的 peer → router」
    连接不路由数据;client 模式正常(StreamHub 已验证,见 CONSUMING.md)。
    """

    config: dict[str, object] = {
        "mode": "client",
        "connect": {"endpoints": [connect_endpoint]},
    }
    return zenoh.Config.from_json5(json.dumps(config))


def publish_json(publisher: object, frame: Mapping[str, object]) -> None:
    """Validate and publish one complete frame as JSON text."""

    publisher.put(encode_frame(frame))
