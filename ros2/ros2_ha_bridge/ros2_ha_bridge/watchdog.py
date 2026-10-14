# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The Home Assistant heartbeat watchdog (``watchdog.mode``, ``watchdog.timeout_s``).

Home Assistant publishes ``{prefix}/ha/heartbeat`` every ``interval_ms``; the
node decodes each one (:func:`ros2_ha_bridge.protocol.decode_ha_heartbeat`)
and feeds it to :meth:`Watchdog.ha_heartbeat`. Every 0.5 s :meth:`Watchdog.tick`
decides whether the link is ``up`` or ``down``: it is down when no heartbeat
has been seen for longer than ``timeout_s``, clamped to at least 2.5 times the
interval HA announced, so a slow HA heartbeat never trips the watchdog by
itself. Each heartbeat's ``ts`` also yields ``ha_clock_offset_ms`` (node clock
minus HA clock), which the robot heartbeat reports.

What a link change does depends on the mode, decided in the interview (Q16:
HA going away must not affect the arms by default):

``none``
    Records only. ``state.ha_link`` follows the link and the change callback
    fires, nothing else.
``inhibit_new``
    Additionally sets ``inhibited`` on the task manager while the link is
    down, so new motion commands are refused with ``rejected_inhibited`` by
    the authoriser's state gating. Running tasks continue.
``stop_and_hold``
    Additionally, once per link loss, calls ``TaskManager.stop_and_hold``,
    which halts the executor and ends the active motion task with
    ``cancelled/inhibited``.

In the two active modes the link counts as down from start-up until the first
heartbeat, so a node that boots while HA is away refuses motion until HA
appears; ``stop_and_hold`` fires only on a transition from up to down, never
at start-up (nothing could be running yet).

Everything here runs on the main thread: ``ha_heartbeat`` from the inbound
queue drain, ``tick`` from the scheduler chain started by :meth:`Watchdog.start`
or from a node timer. ``ha_link_ok`` is read-only and safe from any thread
for the executor's :meth:`~ros2_ha_bridge.executors.base.BridgeContext.ha_link_ok`.
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from typing import Protocol

from ros2_ha_bridge.clock import Clock, Scheduler, TimerHandle
from ros2_ha_bridge.protocol import WATCHDOG_MODES, HaHeartbeat, HaLink, WatchdogMode

__all__ = ['DEFAULT_HA_LINK_TOPIC', 'MIN_TIMEOUT_FACTOR', 'TICK_S', 'Watchdog', 'WatchdogTarget']

#: Period of the watchdog tick (seconds); detection latency is the timeout plus up to this.
TICK_S = 0.5
#: ``watchdog.ros_topic`` default: the latched ``std_msgs/Bool`` that mirrors the HA link
#: (keep last 1, reliable, transient local; the node owns the publisher).
DEFAULT_HA_LINK_TOPIC = '~/ha_link'
#: ``timeout_s`` is clamped to at least this many HA heartbeat intervals.
MIN_TIMEOUT_FACTOR = 2.5

_LOGGER_NAME = 'ros2_ha_bridge.watchdog'


class WatchdogTarget(Protocol):
    """What the watchdog drives: :class:`~ros2_ha_bridge.task_manager.TaskManager` fits."""

    def set_ha_link(self, up: bool, *, inhibited: bool | None = None) -> None:
        """Record the link in ``state.ha_link`` and, when given, ``state.inhibited``."""

    def stop_and_hold(self, reason: str) -> None:
        """Halt the executor and end the active motion task ``cancelled/inhibited``."""


class Watchdog:
    """Tracks the HA heartbeat and applies ``watchdog.mode``. See the module docstring."""

    def __init__(
        self,
        *,
        clock: Clock,
        mode: WatchdogMode = 'none',
        timeout_s: float = 5.0,
        target: WatchdogTarget | None = None,
        on_link_change: Callable[[bool], None] | None = None,
        logger: logging.Logger | None = None,
    ) -> None:
        if mode not in WATCHDOG_MODES:
            raise ValueError(f'watchdog.mode must be one of {WATCHDOG_MODES}, not {mode!r}')
        if not timeout_s > 0:
            raise ValueError(f'watchdog.timeout_s must be greater than 0, not {timeout_s!r}')
        self._clock = clock
        self._mode: WatchdogMode = mode
        self._timeout_s = float(timeout_s)
        self._target = target
        self._on_link_change = on_link_change
        self._logger = logger if logger is not None else logging.getLogger(_LOGGER_NAME)

        self._link_up = False
        self._initialised = False
        self._last_seen_monotonic: float | None = None
        self._last_seq: int | None = None
        self._interval_ms: int | None = None
        self._offset_ms: int | None = None
        self._losses = 0
        self._beats = 0
        self._timer: TimerHandle | None = None
        self._running = False

    # ------------------------------------------------------------------ views

    @property
    def mode(self) -> WatchdogMode:
        return self._mode

    @property
    def timeout_s(self) -> float:
        """The configured ``watchdog.timeout_s`` before clamping."""
        return self._timeout_s

    @property
    def effective_timeout_s(self) -> float:
        """``max(timeout_s, 2.5 x announced interval)``; ``timeout_s`` before any heartbeat."""
        if self._interval_ms is None:
            return self._timeout_s
        return max(self._timeout_s, MIN_TIMEOUT_FACTOR * self._interval_ms / 1000.0)

    @property
    def link_up(self) -> bool:
        return self._link_up

    @property
    def ha_link(self) -> HaLink:
        return 'up' if self._link_up else 'down'

    @property
    def ha_clock_offset_ms(self) -> int | None:
        """Node clock minus HA clock from the last heartbeat; ``None`` before the first."""
        return self._offset_ms

    @property
    def announced_interval_ms(self) -> int | None:
        return self._interval_ms

    @property
    def last_seen_monotonic(self) -> float | None:
        return self._last_seen_monotonic

    @property
    def last_seq(self) -> int | None:
        return self._last_seq

    @property
    def heartbeats(self) -> int:
        """Heartbeats fed so far."""
        return self._beats

    @property
    def losses(self) -> int:
        """Up-to-down transitions so far."""
        return self._losses

    def ha_link_ok(self) -> bool:
        """For ``BridgeContext.ha_link_ok``: True while the link is up."""
        return self._link_up

    # ------------------------------------------------------------------ feeds (main thread)

    def ha_heartbeat(self, ts_ms: int, seq: int, interval_ms: int) -> None:
        """Record a decoded HA heartbeat and bring the link up at once if it was down."""
        self._beats += 1
        self._last_seen_monotonic = self._clock.monotonic()
        self._offset_ms = self._clock.now_ms() - ts_ms
        if interval_ms != self._interval_ms:
            self._interval_ms = interval_ms
            self._logger.info(
                'HA heartbeat interval %d ms; effective watchdog timeout %g s',
                interval_ms,
                self.effective_timeout_s,
            )
        if self._last_seq is not None and seq < self._last_seq:
            self._logger.info('HA heartbeat seq went back from %d to %d', self._last_seq, seq)
        self._last_seq = seq
        self._evaluate()

    def feed(self, beat: HaHeartbeat) -> None:
        """:meth:`ha_heartbeat` from a decoded :class:`~ros2_ha_bridge.protocol.HaHeartbeat`."""
        self.ha_heartbeat(beat.ts, beat.seq, beat.interval_ms)

    def tick(self) -> bool:
        """Re-evaluate the link against the clock; return True while it is up."""
        self._evaluate()
        return self._link_up

    # ------------------------------------------------------------------ scheduler chain

    def start(self, scheduler: Scheduler) -> None:
        """Evaluate now and then every :data:`TICK_S` through ``scheduler`` until :meth:`stop`."""
        if self._running:
            return
        self._running = True
        self._evaluate()
        self._arm(scheduler)

    def stop(self) -> None:
        self._running = False
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    def _arm(self, scheduler: Scheduler) -> None:
        def fire() -> None:
            self._timer = None
            if not self._running:
                return
            self._evaluate()
            if self._running:
                self._arm(scheduler)

        self._timer = scheduler.call_later(TICK_S, fire)

    # ------------------------------------------------------------------ decision

    def _evaluate(self) -> None:
        if self._last_seen_monotonic is None:
            up = False
        else:
            silence = self._clock.monotonic() - self._last_seen_monotonic
            up = silence <= self.effective_timeout_s
        self._apply(up)

    def _apply(self, up: bool) -> None:
        active = self._mode != 'none'
        if not self._initialised:
            # First decision: push the starting value out without treating it as a loss.
            self._initialised = True
            self._link_up = up
            self._logger.info('HA link starts %s (mode %s)', 'up' if up else 'down', self._mode)
            if self._target is not None:
                self._target.set_ha_link(up, inhibited=(not up) if active else None)
            if self._on_link_change is not None:
                self._on_link_change(up)
            return
        if up == self._link_up:
            return
        self._link_up = up
        if up:
            self._logger.info('HA link up')
        else:
            self._losses += 1
            self._logger.warning(
                'HA link down: no heartbeat for more than %g s (mode %s)',
                self.effective_timeout_s,
                self._mode,
            )
        if self._target is not None:
            self._target.set_ha_link(up, inhibited=(not up) if active else None)
            if not up and self._mode == 'stop_and_hold':
                self._target.stop_and_hold(
                    f'Home Assistant heartbeat lost for more than {self.effective_timeout_s:g} s'
                )
        if self._on_link_change is not None:
            self._on_link_change(up)
