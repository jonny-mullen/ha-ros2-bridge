# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""In-process rclpy tests for :mod:`ros2_ha_bridge.ros_adapters`.

These need a real rclpy (they run under ``colcon test`` in the Jazzy
container) and skip themselves on a machine without it; the local suite
checks only that the module imports without rclpy (``test_mqtt_link.py``).
Each test owns a private rclpy context, one node and a
``SingleThreadedExecutor`` spun from the test thread, which therefore plays
the node's main thread. The module under test imports without rclpy, so it
is imported before the skip; rclpy's own classes are imported inside the
fixture, which keeps every import at the top of the file.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable, Iterator
from typing import Any

import pytest

from ros2_ha_bridge.ros_adapters import (
    MainThreadPost,
    RclpyClock,
    RclpyLogHandler,
    RclpyScheduler,
    method_name_for,
)

rclpy = pytest.importorskip('rclpy')

#: ``(node, executor)``: an ``rclpy.node.Node`` and the ``SingleThreadedExecutor`` spinning it.
Ros = tuple[Any, Any]


@pytest.fixture
def ros() -> Iterator[Ros]:
    from rclpy.executors import SingleThreadedExecutor

    context = rclpy.Context()
    rclpy.init(args=None, context=context)
    node = rclpy.create_node('test_ros_adapters', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        yield node, executor
    finally:
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.shutdown(context=context)


def spin_until(executor: Any, predicate: Callable[[], bool], timeout_s: float = 3.0) -> bool:
    """Spin the executor on this thread until ``predicate()`` holds or ``timeout_s`` passes."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        executor.spin_once(timeout_sec=0.02)
    return predicate()


def spin_for(executor: Any, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)


# --------------------------------------------------------------------------- scheduler


def test_scheduler_fires_once_and_destroys_its_timer(ros: Ros) -> None:
    node, executor = ros
    fired: list[int] = []
    handle = RclpyScheduler(node).call_later(0.05, lambda: fired.append(threading.get_ident()))
    assert len(list(node.timers)) == 1
    assert spin_until(executor, lambda: len(fired) == 1)
    assert fired == [threading.get_ident()], 'the callable ran on the spinning thread'
    spin_for(executor, 0.2)
    assert len(fired) == 1, 'a periodic rclpy timer, but it fired exactly once'
    assert handle.remaining_s() == 0.0
    assert len(list(node.timers)) == 0, 'the timer was cancelled and destroyed on its first fire'


def test_scheduler_cancel_before_fire(ros: Ros) -> None:
    node, executor = ros
    fired: list[bool] = []
    handle = RclpyScheduler(node).call_later(0.1, lambda: fired.append(True))
    handle.cancel()
    assert handle.remaining_s() == 0.0
    assert len(list(node.timers)) == 0
    spin_for(executor, 0.3)
    assert fired == []
    handle.cancel()  # a second cancel is harmless


def test_scheduler_remaining_decreases_on_the_steady_clock(ros: Ros) -> None:
    from rclpy.clock import ClockType

    node, executor = ros
    scheduler = RclpyScheduler(node)
    assert scheduler.clock.clock_type == ClockType.STEADY_TIME
    handle = scheduler.call_later(1.0, lambda: None)
    timer = next(iter(node.timers))
    assert timer.clock is scheduler.clock, 'the one-shot runs on the steady clock, not ROS_TIME'
    first = handle.remaining_s()
    assert 0.9 < first <= 1.0
    spin_for(executor, 0.15)
    second = handle.remaining_s()
    assert 0.0 < second < first
    handle.cancel()
    assert handle.remaining_s() == 0.0


def test_scheduler_refuses_a_clock_that_is_not_steady(ros: Ros) -> None:
    node, _ = ros
    with pytest.raises(ValueError):
        RclpyScheduler(node, clock=node.get_clock())
    clock = RclpyClock()
    assert RclpyScheduler(node, clock=clock.steady).clock is clock.steady


def test_scheduler_rejects_a_negative_delay(ros: Ros) -> None:
    node, _ = ros
    with pytest.raises(ValueError):
        RclpyScheduler(node).call_later(-0.1, lambda: None)


def test_scheduler_zero_delay_fires_on_the_next_spin(ros: Ros) -> None:
    node, executor = ros
    fired: list[bool] = []
    RclpyScheduler(node).call_later(0.0, lambda: fired.append(True))
    assert spin_until(executor, lambda: fired == [True], timeout_s=1.0)


def test_scheduler_logs_a_raising_callable(ros: Ros, caplog: pytest.LogCaptureFixture) -> None:
    node, executor = ros
    caplog.set_level(logging.ERROR, logger='ros2_ha_bridge.ros_adapters')

    def explode() -> None:
        raise RuntimeError('timer callback bug')

    handle = RclpyScheduler(node).call_later(0.02, explode)
    assert spin_until(executor, lambda: not handle.active)
    spin_for(executor, 0.1)
    assert any('timer callback bug' in (r.exc_text or '') for r in caplog.records)
    assert len(list(node.timers)) == 0


# --------------------------------------------------------------------------- main-thread post


def test_post_from_a_thread_runs_on_the_spinning_thread(ros: Ros) -> None:
    node, executor = ros
    post = MainThreadPost(node)
    ran_on: list[int] = []

    def worker() -> None:
        post.post(lambda: ran_on.append(threading.get_ident()))

    thread = threading.Thread(target=worker)
    thread.start()
    thread.join()
    assert spin_until(executor, lambda: len(ran_on) == 1)
    assert ran_on == [threading.get_ident()]
    assert post.executed == 1 and post.failed == 0 and post.pending == 0
    post.close()


def test_post_keeps_order_and_survives_a_raising_closure(
    ros: Ros, caplog: pytest.LogCaptureFixture
) -> None:
    node, executor = ros
    caplog.set_level(logging.ERROR, logger='ros2_ha_bridge.ros_adapters')
    post = MainThreadPost(node)
    seen: list[int] = []

    def explode() -> None:
        raise RuntimeError('posted closure bug')

    post.post(lambda: seen.append(1))
    post.post(explode)
    post.post(lambda: seen.append(2))
    assert spin_until(executor, lambda: seen == [1, 2])
    assert post.executed == 2 and post.failed == 1
    assert any('posted closure bug' in (r.exc_text or '') for r in caplog.records)
    post.close()


def test_post_after_close_is_dropped(ros: Ros, caplog: pytest.LogCaptureFixture) -> None:
    node, executor = ros
    caplog.set_level(logging.WARNING, logger='ros2_ha_bridge.ros_adapters')
    post = MainThreadPost(node)
    post.close()
    assert post.closed
    post.post(lambda: pytest.fail('must not run'))
    spin_for(executor, 0.1)
    assert post.pending == 0
    assert any('after close' in r.getMessage() for r in caplog.records)
    post.close()  # idempotent


# --------------------------------------------------------------------------- clock


def test_clock_epoch_and_steady_monotonic(ros: Ros) -> None:
    from rclpy.clock import ClockType

    node, executor = ros
    clock = RclpyClock()
    assert clock.steady.clock_type == ClockType.STEADY_TIME
    assert abs(clock.now_ms() - time.time() * 1000) < 1000
    first = clock.monotonic()
    spin_for(executor, 0.05)
    assert clock.monotonic() > first
    with pytest.raises(ValueError):
        RclpyClock(node.get_clock())


# --------------------------------------------------------------------------- log handler


class StubRosLogger:
    """Records which severity method was called with what."""

    def __init__(self) -> None:
        self.calls: list[tuple[str, str]] = []

    def debug(self, message: str) -> None:
        self.calls.append(('debug', message))

    def info(self, message: str) -> None:
        self.calls.append(('info', message))

    def warning(self, message: str) -> None:
        self.calls.append(('warning', message))

    def error(self, message: str) -> None:
        self.calls.append(('error', message))

    def fatal(self, message: str) -> None:
        self.calls.append(('fatal', message))


@pytest.mark.parametrize(
    ('level', 'method'),
    [
        (logging.DEBUG, 'debug'),
        (5, 'debug'),
        (logging.INFO, 'info'),
        (25, 'info'),
        (logging.WARNING, 'warning'),
        (logging.ERROR, 'error'),
        (logging.CRITICAL, 'fatal'),
    ],
)
def test_log_handler_maps_levels(level: int, method: str) -> None:
    assert method_name_for(level) == method
    stub = StubRosLogger()
    handler = RclpyLogHandler(stub)
    logger = logging.getLogger(f'test_ros_adapters.mapping.{level}')
    logger.propagate = False
    logger.setLevel(1)
    logger.addHandler(handler)
    try:
        logger.log(level, 'hello %s', 'world')
    finally:
        logger.removeHandler(handler)
    assert stub.calls == [(method, f'[{logger.name}] hello world')]


def test_log_handler_carries_the_traceback() -> None:
    stub = StubRosLogger()
    handler = RclpyLogHandler(stub)
    logger = logging.getLogger('test_ros_adapters.traceback')
    logger.propagate = False
    logger.addHandler(handler)
    try:
        try:
            raise ValueError('the cause')
        except ValueError:
            logger.exception('it failed')
    finally:
        logger.removeHandler(handler)
    method, message = stub.calls[0]
    assert method == 'error'
    assert message.startswith('[test_ros_adapters.traceback] it failed')
    assert 'ValueError: the cause' in message


def test_log_handler_attaches_to_the_node_logger(ros: Ros) -> None:
    node, _ = ros
    name = 'ros2_ha_bridge.test_attach'
    handler = RclpyLogHandler.attach(node, name)
    logger = logging.getLogger(name)
    try:
        assert handler in logger.handlers
        assert logger.level == int(node.get_logger().get_effective_level())
        logger.warning('a line from the core, via the ROS logger')
        logger.info('and one at info')
    finally:
        handler.detach()
    assert handler not in logger.handlers


def test_log_handler_never_raises(ros: Ros) -> None:
    class BrokenRosLogger:
        def error(self, message: str) -> None:
            raise RuntimeError('rcutils unavailable')

    handler = RclpyLogHandler(BrokenRosLogger())
    record = logging.LogRecord('x', logging.ERROR, __file__, 1, 'boom', None, None)
    handler.emit(record)  # logging's handleError path, never an exception


def test_module_is_rclpy_free_at_import() -> None:
    """The adapters know rclpy only through the node they are given."""
    import ros2_ha_bridge.ros_adapters as adapters

    source_globals: dict[str, Any] = vars(adapters)
    assert 'rclpy' not in source_globals
