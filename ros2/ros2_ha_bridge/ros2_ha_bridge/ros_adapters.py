# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The rclpy adapters behind the core's seams: scheduler, main-thread post, clock and log handler.

Phase 1 (``clock.py``, ``task_manager.py``, ``watchdog.py``, ``dedupe_store.py``,
the executors) never touches rclpy; it asks for a
:class:`~ros2_ha_bridge.clock.Scheduler`, a :class:`~ros2_ha_bridge.clock.Clock`
and a ``post`` callable. This module provides the real ones over an rclpy
``Node``, and a :class:`logging.Handler` that carries the core's standard
``logging`` records into the node's ROS logger. Besides the node itself it is
the only phase 2 module that knows rclpy, and it knows it through the node
object it is given and one lazy import: rclpy is imported for type checking
and inside :func:`new_steady_clock` at call time, never at module level, so
the module imports (and mypy checks it) on a machine without ROS, and the
tests that need a real node skip themselves there.

Everything here is meant for a node driven by a ``SingleThreadedExecutor``:
that executor's thread is "the main thread" the core's docstrings refer to.

``RclpyScheduler`` implements ``call_later`` with the design's self-cancelling
one-shot pattern. rclpy Jazzy has no one-shot timer (verified: ``Timer``
offers cancel, reset and a period, nothing single-fire), so each delayed call
is a periodic timer whose callback, on its first line, cancels and destroys
the timer before running the callable. Every timer runs on the steady clock
described below, and ``remaining_s`` is computed from that same clock, so
pause and resume in the task manager freeze and re-arm exactly the seconds
that were left; ``cancel`` cancels and destroys the timer at once.

``MainThreadPost`` is the ``post`` callable: ``post(fn)`` appends the closure to
a deque and triggers a ``GuardCondition`` (safe from any thread; rcl documents
guard-condition triggering as thread-safe), and the guard's callback, which
the executor runs on its own thread, drains the deque and runs each closure
with exceptions logged, never propagated. The task manager's ``TaskHandle``
and ``BridgeContext`` methods, called from executor threads and rclpy
callbacks, all arrive here.

``RclpyClock`` takes wall-clock time from :func:`time.time_ns` (every wire
timestamp is epoch milliseconds) and monotonic time from one rclpy ``Clock``
of type ``STEADY_TIME`` (:func:`new_steady_clock`), which the scheduler's
one-shots and the node's periodic timers run on as well: the node makes one
and shares it. The node's own clock is ``ROS_TIME``, which with sim time off
is the system wall clock, so an NTP step (a robot PC without a real-time
clock correcting its clock after boot) would shorten or lengthen every task
deadline, cancel grace, heartbeat and watchdog interval, which the core's
:class:`~ros2_ha_bridge.clock.Clock` contract forbids. The trade-off, recorded
here: ``use_sim_time`` no longer stretches deadlines or rate limits; they run
on real elapsed time even when the rest of the graph runs on simulated time.

``RclpyLogHandler`` forwards each record to the ROS logger at the mapped
severity (``CRITICAL`` to ``fatal``, ``ERROR`` to ``error``, ``WARNING`` to
``warning``, ``INFO`` to ``info``, everything lower to ``debug``), with the
record's logger name in front of the message so ``ros2_ha_bridge.authoriser``
lines can still be told apart in ``rosout``. :meth:`RclpyLogHandler.attach`
wires it onto the package's logger and mirrors the ROS logger's effective
severity onto it, so ``--log-level`` applies to the core's lines as well.
"""

from __future__ import annotations

import logging
import time
from collections import deque
from collections.abc import Callable
from typing import TYPE_CHECKING, Any

from ros2_ha_bridge.clock import TimerHandle

if TYPE_CHECKING:
    from rclpy.node import Node

__all__ = [
    'PACKAGE_LOGGER_NAME',
    'MainThreadPost',
    'RclpyClock',
    'RclpyLogHandler',
    'RclpyOneShot',
    'RclpyScheduler',
    'method_name_for',
    'new_steady_clock',
]

LOGGER = logging.getLogger(__name__)

#: The standard-logging name every core module logs under; what :meth:`RclpyLogHandler.attach` wires.
PACKAGE_LOGGER_NAME = 'ros2_ha_bridge'

_NS_PER_S = 1_000_000_000


# --------------------------------------------------------------------------- steady clock


def new_steady_clock() -> Any:
    """A fresh rclpy ``Clock`` of type ``STEADY_TIME``; rclpy is imported lazily."""
    from rclpy.clock import Clock, ClockType

    return Clock(clock_type=ClockType.STEADY_TIME)


def _require_steady(clock: Any) -> Any:
    """Return ``clock`` or raise ``ValueError`` when it is not a ``STEADY_TIME`` rclpy clock."""
    from rclpy.clock import ClockType

    if getattr(clock, 'clock_type', None) != ClockType.STEADY_TIME:
        raise ValueError('the bridge clock must be an rclpy Clock of type STEADY_TIME')
    return clock


# --------------------------------------------------------------------------- scheduler


class RclpyOneShot:
    """A :class:`~ros2_ha_bridge.clock.TimerHandle` over a periodic rclpy timer that fires once.

    The timer is created with the delay as its period on the steady ``clock``
    it is given; ``_fire`` cancels and destroys it on its first line, so it can
    never fire twice, and then runs the callable. The handle is used on the
    executor thread only, like every other scheduler handle in the core.
    """

    def __init__(
        self,
        node: Node,
        delay_s: float,
        fn: Callable[[], None],
        logger: logging.Logger,
        *,
        clock: Any,
    ) -> None:
        self._node = node
        self._clock = clock
        self._fn = fn
        self._logger = logger
        self._done = False
        self._fired = False
        start_ns: int = clock.now().nanoseconds
        self._due_ns: int = start_ns + round(delay_s * _NS_PER_S)
        self._timer: Any = node.create_timer(delay_s, self._fire, clock=clock)

    @property
    def fired(self) -> bool:
        """True once the callable has run (or started to run)."""
        return self._fired

    @property
    def active(self) -> bool:
        """True while the call is still pending."""
        return not self._done

    def cancel(self) -> None:
        if self._done:
            return
        self._done = True
        self._dispose()

    def remaining_s(self) -> float:
        if self._done:
            return 0.0
        now_ns: int = self._clock.now().nanoseconds
        return max(0.0, (self._due_ns - now_ns) / _NS_PER_S)

    def _fire(self) -> None:
        # First line: this is a periodic timer in rclpy, and it must never fire twice.
        self._dispose()
        if self._done:
            return
        self._done = True
        self._fired = True
        try:
            self._fn()
        except Exception:
            self._logger.exception('scheduled call raised; ignored')

    def _dispose(self) -> None:
        timer = self._timer
        if timer is None:
            return
        self._timer = None
        try:
            timer.cancel()
            self._node.destroy_timer(timer)
        except Exception:
            self._logger.exception('could not cancel and destroy a one-shot timer')


class RclpyScheduler:
    """:class:`~ros2_ha_bridge.clock.Scheduler` over ``node.create_timer`` (see the module docstring).

    ``clock`` is the ``STEADY_TIME`` rclpy clock every one-shot runs on; the
    node passes the one its :class:`RclpyClock` reads, and a fresh one is made
    when none is given.
    """

    def __init__(
        self, node: Node, *, clock: Any | None = None, logger: logging.Logger | None = None
    ) -> None:
        self._node = node
        self._clock = _require_steady(clock) if clock is not None else new_steady_clock()
        self._logger = logger if logger is not None else LOGGER

    @property
    def clock(self) -> Any:
        """The ``STEADY_TIME`` rclpy clock the timers of this scheduler run on."""
        return self._clock

    def call_later(self, delay_s: float, fn: Callable[[], None]) -> TimerHandle:
        if delay_s < 0:
            raise ValueError('delay must not be negative')
        return RclpyOneShot(self._node, float(delay_s), fn, self._logger, clock=self._clock)


# --------------------------------------------------------------------------- main-thread post


class MainThreadPost:
    """``post(fn)`` from any thread; ``fn()`` runs on the executor thread, in posting order.

    The guard condition is created on ``node`` at construction and destroyed by
    :meth:`close`. A closure that raises is logged and counted in
    :attr:`failed`; the ones behind it still run.
    """

    def __init__(self, node: Node, *, logger: logging.Logger | None = None) -> None:
        self._node = node
        self._logger = logger if logger is not None else LOGGER
        self._queue: deque[Callable[[], None]] = deque()
        self._closed = False
        self._guard: Any = node.create_guard_condition(self._drain)
        #: Closures run so far, and closures that raised.
        self.executed = 0
        self.failed = 0

    @property
    def pending(self) -> int:
        return len(self._queue)

    @property
    def closed(self) -> bool:
        return self._closed

    def post(self, fn: Callable[[], None]) -> None:
        """Queue ``fn`` for the executor thread and wake it. Thread-safe."""
        if self._closed:
            self._logger.warning('post() after close: a %r call is dropped', _name_of(fn))
            return
        self._queue.append(fn)
        self.trigger()

    def trigger(self) -> None:
        """Wake the executor so it drains the queue. Thread-safe; never raises."""
        if self._closed:
            return
        try:
            self._guard.trigger()
        except Exception:
            self._logger.exception('guard condition trigger raised; queued calls may run late')

    def close(self) -> None:
        """Stop accepting posts and destroy the guard condition; queued closures are dropped."""
        if self._closed:
            return
        self._closed = True
        dropped = len(self._queue)
        self._queue.clear()
        if dropped:
            self._logger.info('%d queued call(s) dropped at close', dropped)
        try:
            self._node.destroy_guard_condition(self._guard)
        except Exception:
            self._logger.exception('could not destroy the post guard condition')

    def _drain(self) -> None:
        while True:
            try:
                fn = self._queue.popleft()
            except IndexError:
                return
            try:
                fn()
                self.executed += 1
            except Exception:
                self.failed += 1
                self._logger.exception('posted call %r raised; ignored', _name_of(fn))


# --------------------------------------------------------------------------- clock


class RclpyClock:
    """:class:`~ros2_ha_bridge.clock.Clock` over :mod:`time` and one steady rclpy clock.

    ``clock`` is an rclpy ``Clock`` of type ``STEADY_TIME`` (a fresh one when
    omitted). The node shares it with its scheduler and its periodic timers
    through :attr:`steady`, so every timeout runs on the clock ``monotonic``
    reads, and a wall-clock step cannot move either.
    """

    def __init__(self, clock: Any | None = None) -> None:
        self._steady = _require_steady(clock) if clock is not None else new_steady_clock()

    @property
    def steady(self) -> Any:
        """The ``STEADY_TIME`` rclpy clock behind :meth:`monotonic`."""
        return self._steady

    def now_ms(self) -> int:
        return time.time_ns() // 1_000_000

    def monotonic(self) -> float:
        nanoseconds: int = self._steady.now().nanoseconds
        return nanoseconds / _NS_PER_S


# --------------------------------------------------------------------------- logging


class RclpyLogHandler(logging.Handler):
    """Forward standard ``logging`` records to a ROS logger at the mapped severity.

    ``ros_logger`` is what ``node.get_logger()`` returns (anything with
    ``debug``, ``info``, ``warning``, ``error`` and ``fatal`` methods taking a
    message). The record's logger name is prefixed to the message; a record
    with exception info carries the traceback, as :class:`logging.Formatter`
    renders it.
    """

    def __init__(self, ros_logger: Any, level: int = logging.NOTSET) -> None:
        super().__init__(level)
        self._ros_logger = ros_logger
        self._attached_to: logging.Logger | None = None
        self.setFormatter(logging.Formatter('[%(name)s] %(message)s'))

    @classmethod
    def attach(cls, node: Node, name: str = PACKAGE_LOGGER_NAME) -> RclpyLogHandler:
        """Install a handler for ``name`` that logs through ``node.get_logger()``.

        The Python logger's level is set to the ROS logger's effective
        severity (rclpy's ``LoggingSeverity`` values equal Python's), so
        ``--log-level debug`` on the node also releases the core's DEBUG
        lines. Call :meth:`detach` at node exit.
        """
        ros_logger = node.get_logger()
        handler = cls(ros_logger)
        logger = logging.getLogger(name)
        logger.addHandler(handler)
        logger.setLevel(_effective_level(ros_logger))
        handler._attached_to = logger
        return handler

    def detach(self) -> None:
        """Remove this handler from the logger :meth:`attach` put it on."""
        logger = self._attached_to
        if logger is not None:
            logger.removeHandler(self)
            self._attached_to = None
        self.close()

    def emit(self, record: logging.LogRecord) -> None:
        try:
            message = self.format(record)
            # rclpy caches its logging features per call site (file, line and function)
            # and raises "Logger severity cannot be changed between calls" when one site
            # logs at two severities, so every severity gets a call site of its own here.
            levelno = record.levelno
            if levelno >= logging.CRITICAL:
                self._ros_logger.fatal(message)
            elif levelno >= logging.ERROR:
                self._ros_logger.error(message)
            elif levelno >= logging.WARNING:
                self._ros_logger.warning(message)
            elif levelno >= logging.INFO:
                self._ros_logger.info(message)
            else:
                self._ros_logger.debug(message)
        except Exception:  # noqa: BLE001 (logging's own contract: emit never raises)
            self.handleError(record)

    def method_for(self, levelno: int) -> Callable[[str], Any]:
        """The ROS logger method for a Python level number (see the module docstring)."""
        method: Callable[[str], Any] = getattr(self._ros_logger, method_name_for(levelno))
        return method


def method_name_for(levelno: int) -> str:
    """The ROS logger method name for a Python level number."""
    if levelno >= logging.CRITICAL:
        return 'fatal'
    if levelno >= logging.ERROR:
        return 'error'
    if levelno >= logging.WARNING:
        return 'warning'
    if levelno >= logging.INFO:
        return 'info'
    return 'debug'


def _effective_level(ros_logger: Any) -> int:
    """The ROS logger's effective severity as a Python level (same numbers), INFO when unknown."""
    getter = getattr(ros_logger, 'get_effective_level', None)
    if not callable(getter):
        return logging.INFO
    try:
        level = int(getter())
    except (TypeError, ValueError):
        return logging.INFO
    return level if level > 0 else logging.INFO


def _name_of(fn: object) -> str:
    name: str | None = getattr(fn, '__qualname__', None) or getattr(fn, '__name__', None)
    return name if name else repr(fn)
