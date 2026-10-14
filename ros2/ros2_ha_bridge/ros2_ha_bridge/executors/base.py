# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Executor plugin interface of ros2_ha_bridge.

A :class:`TaskExecutor` is the one piece of robot-specific code in the bridge.
The node loads exactly one per process (the ``executor`` parameter), asks it
for its task catalogue, and calls :meth:`TaskExecutor.start` and
:meth:`TaskExecutor.cancel` for commands that passed authorisation. The
executor reports progress and the terminal outcome of each task through the
:class:`TaskHandle` it was given, and reaches back into the bridge through the
:class:`BridgeContext` it received in :meth:`TaskExecutor.setup`.

This module imports neither rclpy nor paho. Everything a plugin needs from the
bridge arrives through the context, so a plugin can be unit-tested on any
machine with the fakes in ``test/fakes.py``.

Threading: the bridge calls every ``TaskExecutor`` method on the node's main
thread and times each call; a call longer than ``executor.call_warn_s`` marks
the executor degraded, and a call that never returns stalls this robot only
(recorded decision: no thread-pool guard). ``TaskHandle`` and
``BridgeContext`` methods may be called from any thread.

Joint telemetry (0.2.0, optional): an executor that knows its joints returns a
:class:`JointInfo` from :meth:`TaskExecutor.joints` (called once, after
``setup``) and reports positions through :meth:`BridgeContext.report_joints`
as often as it likes; the bridge announces the joints, throttles the reports
to ``telemetry.joints_rate_hz`` and publishes them on the ``joints`` topic.
An executor without joints leaves the default ``joints()`` alone and never
calls ``report_joints``.
"""

from __future__ import annotations

import inspect
import logging
from abc import ABC, abstractmethod
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from importlib import import_module
from importlib.metadata import entry_points
from typing import Any, Literal, Protocol, get_args

from ros2_ha_bridge.clock import Clock, Scheduler, SystemClock

TaskKind = Literal['motion', 'control', 'query']
ResultStatus = Literal['succeeded', 'cancelled', 'aborted']
HealthLevel = Literal['ok', 'degraded', 'error']
JointUnit = Literal['deg', 'rad']

#: Closed sets of the contract, as tuples for membership tests and parametrised tests.
TASK_KINDS: tuple[TaskKind, ...] = get_args(TaskKind)
RESULT_STATUSES: tuple[ResultStatus, ...] = get_args(ResultStatus)
HEALTH_LEVELS: tuple[HealthLevel, ...] = get_args(HealthLevel)
JOINT_UNITS: tuple[JointUnit, ...] = get_args(JointUnit)

#: The most joints one robot may announce (``announce.joints.names`` maxItems).
MAX_JOINTS = 32

#: ``report_joints(positions, velocities)``: what the bridge injects into the context.
ReportJointsFn = Callable[[Sequence[float], Sequence[float] | None], None]

#: ``action_msgs/GoalStatus`` values for each terminal status (Jazzy: SUCCEEDED 4,
#: CANCELED 5, ABORTED 6). The result payload carries both the word and the number.
GOAL_STATUS: Mapping[ResultStatus, int] = {'succeeded': 4, 'cancelled': 5, 'aborted': 6}

#: ``importlib.metadata`` entry-point group that names installable executors.
ENTRY_POINT_GROUP = 'ros2_ha_bridge.executors'

#: Ack code used when an executor refuses a task; the contract's closed set has no other
#: code for a plugin-side refusal.
DEFAULT_REJECTION_CODE = 'rejected_executor'


@dataclass(frozen=True)
class TaskSpec:
    """One entry of the task catalogue the node announces to Home Assistant.

    ``name`` must match ``^[a-z][a-z0-9_]{0,31}$`` (it travels in commands and
    results); ``label`` is what buttons and speech use. ``params`` is ``None``
    for a task without parameters, otherwise a params-subset JSON Schema
    (``type: object``, primitive-typed ``properties`` with optional ``enum``,
    ``minimum``, ``maximum`` and ``description``, plus ``required``). The
    bridge checks every spec with ``protocol.validate_task_spec`` at start-up.
    """

    name: str
    label: str
    kind: TaskKind = 'motion'
    timeout_s: float = 60.0
    params: Mapping[str, Any] | None = None
    description: str = ''


@dataclass(frozen=True)
class TaskRequest:
    """An authorised task command, as handed to :meth:`TaskExecutor.start`.

    ``timeout_s`` is already resolved (the command's value clamped to the cap,
    or the announced default). ``source`` is Home Assistant's own record of who
    raised the command; the bridge never interprets it and neither should a
    plugin beyond logging.
    """

    id: str
    task: str
    params: Mapping[str, Any]
    issued_at: int
    timeout_s: float
    source: Mapping[str, Any]


@dataclass(frozen=True)
class TaskResult:
    """Terminal outcome of a task, passed to :meth:`TaskHandle.finish`.

    ``code`` should match ``^[a-z][a-z0-9_]{0,63}$`` and preferably come from
    the contract's recommended vocabulary (``ok``, ``cancelled``,
    ``preempted``, ``collision``, ``planning_failed``, ``timeout`` and so on);
    the bridge normalises anything else before it reaches the wire.
    """

    status: ResultStatus
    code: str
    reason: str = ''
    data: Mapping[str, Any] = field(default_factory=dict)

    @property
    def goal_status(self) -> int:
        """The ``action_msgs/GoalStatus`` value matching :attr:`status`."""
        return GOAL_STATUS[self.status]


@dataclass(frozen=True)
class JointInfo:
    """What an executor announces about its joints (0.2.0, optional).

    ``names`` lists 1 to 32 joint names, each matching ``^[a-z][a-z0-9_]{0,31}$``,
    in the order every later :meth:`BridgeContext.report_joints` call uses;
    ``unit`` is ``deg`` or ``rad`` for every position and limit; ``limits`` is
    ``None`` when the executor knows no limits, otherwise one ``(minimum,
    maximum)`` pair per name in that unit. The bridge checks the whole object
    with ``protocol.validate_joint_info`` at start-up and refuses to start on
    a bad one, exactly as it does for the task catalogue.
    """

    names: Sequence[str]
    unit: JointUnit = 'deg'
    limits: Sequence[tuple[float, float]] | None = None

    @property
    def count(self) -> int:
        """How many positions every report must carry."""
        return len(self.names)


class TaskRejected(Exception):
    """Raised from :meth:`TaskExecutor.start` to refuse a task before it begins.

    The bridge answers the command with the ack code ``rejected_executor``
    (``code`` exists for forward compatibility and is logged, not sent) and the
    given reason, and the robot stays idle.
    """

    def __init__(self, code: str = DEFAULT_REJECTION_CODE, reason: str = '') -> None:
        super().__init__(reason or code)
        self.code = code
        self.reason = reason


class TaskHandle(Protocol):
    """One per task, created by the bridge and given to :meth:`TaskExecutor.start`.

    Every method is thread-safe and may be called from a plugin's own thread or
    an rclpy callback. Calls after the terminal :meth:`finish` (including a
    second ``finish``) are logged and ignored, so an executor need not guard
    against racing its own cancel path.
    """

    task_id: str

    def feedback(self, progress: float | None = None, stage: str = '', message: str = '') -> None:
        """Report progress (``0.0`` to ``1.0`` or ``None``), a stage name and a message.

        The bridge throttles feedback to ``reply.feedback_max_rate_hz``; a
        plugin may call this as often as it likes.
        """

    def finish(self, result: TaskResult) -> None:
        """Publish the result and release the robot's active slot. Exactly once per task."""


class BridgeContext:
    """What the bridge gives an executor in :meth:`TaskExecutor.setup`.

    Every method is thread-safe: the bridge injects callables that marshal onto
    its main thread. ``node`` is the rclpy ``Node`` when running inside the
    real bridge (phase 2) and ``None`` in unit tests, so a plugin that needs
    ROS interfaces (an action client, a topic) creates them from ``node`` and
    must cope with ``None`` when tested. ``clock`` and ``scheduler`` let a
    plugin stay deterministic under test; ``parameters`` carries the node's
    parameters that concern the executor (for example ``fake.task_duration_s``)
    as a read-only mapping keyed by their dotted names. ``report_joints`` is
    the bridge's joint-telemetry sink (0.2.0); when none is injected, as in a
    unit test that does not care about joints, reports are dropped silently.
    """

    def __init__(
        self,
        *,
        robot_id: str,
        logger: logging.Logger,
        post: Callable[[Callable[[], None]], None],
        set_error: Callable[[str, str], None],
        clear_error: Callable[[], None],
        set_paused: Callable[[bool], None],
        ha_link_ok: Callable[[], bool],
        clock: Clock | None = None,
        scheduler: Scheduler | None = None,
        parameters: Mapping[str, Any] | None = None,
        node: Any = None,
        report_joints: ReportJointsFn | None = None,
    ) -> None:
        self.robot_id = robot_id
        self.logger = logger
        self.node = node
        self.clock: Clock = clock if clock is not None else SystemClock()
        self.scheduler = scheduler
        self.parameters: Mapping[str, Any] = dict(parameters or {})
        self._post = post
        self._set_error = set_error
        self._clear_error = clear_error
        self._set_paused = set_paused
        self._ha_link_ok = ha_link_ok
        self._report_joints = report_joints
        self._joints_dropped_logged = False

    def post(self, fn: Callable[[], None]) -> None:
        """Run ``fn`` on the bridge's main thread as soon as it is free.

        This is how a plugin thread or an rclpy callback hands work back to the
        single thread that owns robot state.
        """
        self._post(fn)

    def set_error(self, code: str, reason: str) -> None:
        """Put the robot into the ``error`` state; motion is refused until cleared."""
        self._set_error(code, reason)

    def clear_error(self) -> None:
        """Leave the ``error`` state (back to ``idle`` or ``busy``)."""
        self._clear_error()

    def set_paused(self, paused: bool) -> None:
        """Set ``state.paused``; pausing also freezes the active task's timeout clock."""
        self._set_paused(paused)

    def ha_link_ok(self) -> bool:
        """True while Home Assistant's heartbeat is being seen within the watchdog window."""
        return self._ha_link_ok()

    def report_joints(
        self, positions: Sequence[float], velocities: Sequence[float] | None = None
    ) -> None:
        """Report the current joint positions (and optionally velocities) to the bridge.

        ``positions`` carries one number per name of the :class:`JointInfo`
        the executor returned from :meth:`TaskExecutor.joints`, in that order
        and in that unit; ``velocities`` is ``None`` or one number per
        position in the unit per second. Thread-safe like
        :meth:`TaskHandle.feedback`: the values are copied here and the call
        is posted to the bridge's main thread, which throttles the stream to
        ``telemetry.joints_rate_hz`` and publishes it. Call it as often as the
        driver reports; the bridge drops what it does not need. With no sink
        injected (a unit test, or an executor that announced no joints) the
        report is dropped and nothing is raised.
        """
        sink = self._report_joints
        if sink is None:
            if not self._joints_dropped_logged:
                self._joints_dropped_logged = True
                self.logger.debug('report_joints called with no telemetry sink; reports dropped')
            return
        snapshot = tuple(positions)
        speeds = None if velocities is None else tuple(velocities)
        self._post(lambda: sink(snapshot, speeds))


class TaskExecutor(ABC):
    """Base class for robot-specific executors. Subclass and implement the four abstract methods.

    Lifecycle: ``setup`` once (blocking waits for drivers are allowed here only),
    ``tasks`` once per announce, ``joints`` once right after that, then any
    number of ``start`` and ``cancel`` calls, ``health`` once per
    ``executor.health_poll_s``, ``stop_and_hold`` when the watchdog says so,
    and ``shutdown`` once at exit.
    """

    #: Plugin name and version, announced to Home Assistant.
    name: str = 'base'
    version: str = '0.0.0'

    @abstractmethod
    def setup(self, ctx: BridgeContext) -> None:
        """Prepare the executor. Blocking waits are allowed here and nowhere else."""

    @abstractmethod
    def tasks(self) -> Sequence[TaskSpec]:
        """The task catalogue before the operator's ``tasks.allow`` intersection."""

    def joints(self) -> JointInfo | None:
        """The joints this executor reports through ``ctx.report_joints``, or ``None``.

        Called once at start-up, after :meth:`setup`. The default announces
        no joints, so the node publishes no telemetry and Home Assistant
        creates no joint sensors; an executor that returns a
        :class:`JointInfo` must report positions of exactly that length and
        order.
        """
        return None

    @abstractmethod
    def start(self, req: TaskRequest, handle: TaskHandle) -> None:
        """Begin a task and return promptly; raise :class:`TaskRejected` to refuse it.

        The outcome is reported later through ``handle.finish``; an executor
        that finishes synchronously may call ``handle.finish`` before
        returning.
        """

    @abstractmethod
    def cancel(self, task_id: str) -> None:
        """Ask a running task to stop.

        It must lead to ``finish(cancelled)`` within ``executor.cancel_grace_s``;
        otherwise the bridge forces ``aborted`` with ``executor_unresponsive``.
        """

    def stop_and_hold(self, reason: str) -> None:
        """Watchdog action: halt motion and hold position. The default does nothing."""

    def health(self) -> tuple[HealthLevel, str]:
        """Current health, polled by the bridge. The default reports ``ok``."""
        return ('ok', '')

    def shutdown(self) -> None:
        """Release resources at node exit. The default does nothing."""


class ExecutorLoadError(Exception):
    """The ``executor`` parameter names nothing usable; a fatal start-up error."""


def load_executor(spec: str) -> TaskExecutor:
    """Resolve ``spec`` to an executor instance.

    A ``spec`` containing a colon is ``package.module:Attribute`` and is loaded
    with :func:`importlib.import_module` plus ``getattr``; anything else is the
    name of an entry point in the ``ros2_ha_bridge.executors`` group. The
    target may be a concrete :class:`TaskExecutor` subclass (instantiated with
    no arguments) or an instance of one. Every failure raises
    :class:`ExecutorLoadError` with a message that names the spec and what went
    wrong, because the node treats it as fatal.
    """
    spec = spec.strip()
    if not spec:
        raise ExecutorLoadError(
            "executor is empty; give an entry-point name or 'package.module:Class'"
        )
    target = _load_dotted(spec) if ':' in spec else _load_entry_point(spec)
    return _instantiate(target, spec)


def _load_dotted(spec: str) -> object:
    module_name, _, attribute_path = spec.partition(':')
    if not module_name or not attribute_path:
        raise ExecutorLoadError(
            f"executor {spec!r} must look like 'package.module:Class' (both parts non-empty)"
        )
    try:
        target: object = import_module(module_name)
    except ImportError as error:
        raise ExecutorLoadError(
            f'executor {spec!r}: cannot import module {module_name!r}: {error}'
        ) from error
    for attribute in attribute_path.split('.'):
        try:
            target = getattr(target, attribute)
        except AttributeError as error:
            raise ExecutorLoadError(
                f'executor {spec!r}: module {module_name!r} has no attribute {attribute_path!r}'
            ) from error
    return target


def _load_entry_point(name: str) -> object:
    matches = entry_points(group=ENTRY_POINT_GROUP, name=name)
    if not matches:
        available = sorted({entry.name for entry in entry_points(group=ENTRY_POINT_GROUP)})
        listing = ', '.join(available) if available else 'none installed'
        raise ExecutorLoadError(
            f'no executor entry point named {name!r} in group {ENTRY_POINT_GROUP!r} '
            f"(available: {listing}); use 'package.module:Class' to load one by path"
        )
    entry = next(iter(matches))
    try:
        target: object = entry.load()
    except (ImportError, AttributeError) as error:
        raise ExecutorLoadError(
            f'executor entry point {name!r} ({entry.value}) failed to load: {error}'
        ) from error
    return target


def _instantiate(target: object, spec: str) -> TaskExecutor:
    if isinstance(target, TaskExecutor):
        return target
    if inspect.isclass(target) and issubclass(target, TaskExecutor):
        if inspect.isabstract(target):
            missing = ', '.join(sorted(getattr(target, '__abstractmethods__', ())))
            raise ExecutorLoadError(
                f'executor {spec!r} resolved to the abstract class {target.__name__}; '
                f'it does not implement: {missing}'
            )
        try:
            return target()
        except TypeError as error:
            raise ExecutorLoadError(
                f'executor {spec!r}: {target.__name__}() must take no arguments: {error}'
            ) from error
    kind = target.__name__ if inspect.isclass(target) else type(target).__name__
    raise ExecutorLoadError(
        f'executor {spec!r} resolved to {kind}, which is neither a TaskExecutor '
        'subclass nor a TaskExecutor instance'
    )
