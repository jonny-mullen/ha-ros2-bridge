# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Clock and timer seams for the rclpy-free core.

Every core module that needs the time or a delayed call takes a :class:`Clock`
or a :class:`Scheduler` instead of touching :mod:`time` or an rclpy timer
directly. Unit tests inject ``FakeClock`` and ``FakeScheduler`` from
``test/fakes.py``; the node (phase 2) injects :class:`SystemClock` and a
scheduler built on rclpy timers.

rclpy Jazzy has no one-shot timer, so the node's scheduler implements
:meth:`Scheduler.call_later` as a periodic timer whose callback cancels itself
on its first line. That pattern belongs to the node and is not repeated here.

Only the protocols and the real clock live in this module; no scheduler is
implemented here because every real one depends on a runtime (rclpy timers)
and every fake one depends on a fake clock.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from typing import Protocol


class TimerHandle(Protocol):
    """A pending delayed call returned by :meth:`Scheduler.call_later`."""

    def cancel(self) -> None:
        """Stop the call from happening. Calling it after the fire or twice is harmless."""

    def remaining_s(self) -> float:
        """Seconds until the call fires, or ``0.0`` once it has fired or been cancelled."""


class Clock(Protocol):
    """Source of wall-clock and monotonic time.

    ``now_ms`` feeds every wire timestamp (epoch milliseconds, UTC) and the
    freshness rule; ``monotonic`` drives timeouts and rate limits so a wall
    clock step (NTP correction) never shortens or lengthens a running task.
    """

    def now_ms(self) -> int:
        """Current wall-clock time as integer epoch milliseconds."""

    def monotonic(self) -> float:
        """Seconds on a clock that never goes backwards."""


class Scheduler(Protocol):
    """Runs a callback once after a delay, on the bridge's main thread."""

    def call_later(self, delay_s: float, fn: Callable[[], None]) -> TimerHandle:
        """Arrange for ``fn()`` to run once, ``delay_s`` seconds from now.

        The call happens on the same thread the bridge uses for every other
        callback (the rclpy executor thread in the node), so ``fn`` may touch
        robot state without locking.
        """


class SystemClock:
    """The real clock: :func:`time.time_ns` and :func:`time.monotonic`."""

    def now_ms(self) -> int:
        return time.time_ns() // 1_000_000

    def monotonic(self) -> float:
        return time.monotonic()
