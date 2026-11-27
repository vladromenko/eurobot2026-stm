"""Sequential background queue used by the non-blocking keyboard UI."""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable

from .controller import Robot2026


@dataclass
class WorkItem:
    channel: str
    label: str
    action: Callable[[], None]


class CommandDispatcher:
    """Run controller commands sequentially without freezing the keyboard UI."""

    def __init__(self, robot: Robot2026):
        self.robot = robot
        self._condition = threading.Condition()
        self._queue: deque[WorkItem] = deque()
        self._events: deque[tuple[str, str]] = deque()
        self._busy: WorkItem | None = None
        self._stopping = False
        self._thread = threading.Thread(
            target=self._worker,
            name="robot-2026-command-dispatcher",
            daemon=True,
        )
        self._thread.start()

    def submit(
        self,
        channel: str,
        label: str,
        action: Callable[[], None],
        *,
        replace_pending: bool = False,
        front: bool = False,
    ) -> None:
        item = WorkItem(channel, label, action)
        with self._condition:
            if self._stopping:
                return
            if replace_pending:
                self._queue = deque(
                    queued for queued in self._queue if queued.channel != channel
                )
            if front:
                self._queue.appendleft(item)
            else:
                self._queue.append(item)
            self._condition.notify()

    def submit_batch(self, items: list[WorkItem], *, front: bool = False) -> None:
        with self._condition:
            if self._stopping:
                return
            if front:
                for item in reversed(items):
                    self._queue.appendleft(item)
            else:
                self._queue.extend(items)
            self._condition.notify()

    def replace_queue(self, items: list[WorkItem]) -> None:
        with self._condition:
            if self._stopping:
                return
            self._queue.clear()
            self._queue.extend(items)
            self._condition.notify()

    def cancel_pending(self) -> None:
        with self._condition:
            self._queue.clear()
            self._condition.notify_all()

    def mechanism_pending(self) -> bool:
        with self._condition:
            if self._busy is not None and self._busy.channel != "drive":
                return True
            return any(item.channel != "drive" for item in self._queue)

    def status(self) -> tuple[str, int]:
        with self._condition:
            busy = self._busy.label if self._busy else "idle"
            return busy, len(self._queue)

    def drain_events(self) -> list[tuple[str, str]]:
        with self._condition:
            events = list(self._events)
            self._events.clear()
            return events

    def wait_idle(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        with self._condition:
            while self._busy is not None or self._queue:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._condition.wait(min(remaining, 0.20))
            return True

    def close(self, timeout: float = 10.0) -> bool:
        completed = self.wait_idle(timeout)
        with self._condition:
            self._stopping = True
            self._condition.notify_all()
        self._thread.join(timeout=1.0)
        return completed and not self._thread.is_alive()

    def _worker(self) -> None:
        while True:
            with self._condition:
                while not self._queue and not self._stopping:
                    self._condition.wait()
                if self._stopping and not self._queue:
                    return
                item = self._queue.popleft()
                self._busy = item
            previous = self.robot.action_context
            self.robot.action_context = item.label
            try:
                item.action()
            except Exception as exc:
                event = ("ERROR", f"{item.label}: {exc}")
            else:
                event = ("OK", item.label)
            finally:
                self.robot.action_context = previous
            with self._condition:
                self._events.append(event)
                self._busy = None
                self._condition.notify_all()
