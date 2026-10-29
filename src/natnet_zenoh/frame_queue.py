"""A bounded queue that keeps the newest NatNet frames."""

from __future__ import annotations

import queue
import threading
from collections import deque


class LatestFrameQueue:
    """Non-blocking producer queue that drops the oldest item on overflow."""

    def __init__(self, capacity: int):
        if capacity < 1:
            raise ValueError("capacity must be at least one")
        self._capacity = capacity
        self._items: deque[dict[str, object]] = deque()
        self._condition = threading.Condition()
        self._dropped_frames = 0

    @property
    def dropped_frames(self) -> int:
        with self._condition:
            return self._dropped_frames

    def put_latest(self, frame: dict[str, object]) -> int:
        """Insert without waiting, evicting one oldest frame when necessary."""

        with self._condition:
            if len(self._items) == self._capacity:
                self._items.popleft()
                self._dropped_frames += 1
            frame["publisher_dropped_frames"] = self._dropped_frames
            self._items.append(frame)
            self._condition.notify()
            return self._dropped_frames

    def get(self, timeout: float | None = None) -> dict[str, object]:
        with self._condition:
            if not self._condition.wait_for(lambda: bool(self._items), timeout=timeout):
                raise queue.Empty
            return self._items.popleft()

    def empty(self) -> bool:
        with self._condition:
            return not self._items
