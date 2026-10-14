# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Robot state machine and task lifecycle of the bridge, free of rclpy and paho.

:class:`TaskManager` owns everything Home Assistant sees as the robot's state
(``idle``, ``busy``, ``error``, plus ``paused``, ``inhibited``, ``ha_link``,
the executor's health and the active task) and drives the executor plugin
through one task's life: ``start``, feedback, cancel, the task timeout with its
cancel grace, pause and resume, and the single retained result. The
authoriser hands it commands that already passed the contract's earlier
stages (size, id, schema, signature, dedupe, freshness, allowlist, params) as
the resolved :class:`~ros2_ha_bridge.executors.base.TaskRequest` (the
authoriser's ``Ack.request``, whose ``timeout_s`` is already clamped); the
manager hands exactly that request to the executor, arms its deadline from
it, returns the ack to publish and takes it from there. Step 9 of the contract
(busy, error, inhibited) is :func:`~ros2_ha_bridge.authoriser.gate_motion`,
applied here to :meth:`TaskManager.gating_state`, which the node also gives
the authoriser as its state view, so there is one gate.

The executor's word on a task is the handle's flag: ``handle.finish`` marks
the handle finished synchronously on the calling thread and posts the result
to the main thread. Wherever the manager would otherwise cancel, force or
refuse a task (a ``TaskRejected`` raised after a synchronous finish, the
deadline, the cancel grace, a watchdog stop), it checks that flag first, so a
finish that is still queued behind the node's post is honoured rather than
pre-empted, whatever the post latency.

Threading model
---------------
All robot state lives on the bridge's main thread (the rclpy executor thread
in the node, the test's own thread in unit tests). Methods fall into two
groups:

Main thread only (called by the authoriser, the watchdog, the node and
scheduler timers): :meth:`TaskManager.start`, :meth:`TaskManager.cancel`,
:meth:`TaskManager.stop_and_hold`, :meth:`TaskManager.set_inhibited`,
:meth:`TaskManager.set_ha_link`, :meth:`TaskManager.poll_health`,
:meth:`TaskManager.start_health_polling`, :meth:`TaskManager.publish_state`,
:meth:`TaskManager.motion_rejection` and the read-only properties.

Any thread (handed to the executor through :class:`TaskHandle` and
:class:`BridgeContext`): ``handle.feedback``, ``handle.finish``,
:meth:`TaskManager.set_error`, :meth:`TaskManager.clear_error` and
:meth:`TaskManager.set_paused`. Each of these does nothing but hand a closure
to the injected ``post`` callable, which the node implements as "append to the
inbound deque and trigger the guard condition"; the closure then runs on the
main thread in arrival order. A unit test may pass ``post=lambda fn: fn()`` to
apply such calls immediately.

Timers come from the injected :class:`~ros2_ha_bridge.clock.Scheduler`, whose
``call_later`` is one-shot by contract. rclpy Jazzy has no one-shot timer, so
the node's scheduler (phase 2) wraps a periodic timer whose callback cancels
itself on its first line; that adapter is the node's concern, not this
module's.

Wire behaviour, in the contract's words: an accepted motion task publishes
``busy`` with ``task_id`` and ``task``; ``handle.finish`` publishes the
retained result carrying ``kind`` and returns the robot to ``idle`` for motion
tasks; control and query tasks never touch the motion slot. A task that runs
past its timeout is asked to cancel and, after ``cancel_grace_s`` without a
finish, is forced to ``aborted/timeout`` (or ``aborted/executor_unresponsive``
when ``cancel()`` raised, or when a cancel that was not caused by the timeout
is ignored); every forced finish puts the robot into ``error`` until the
executor calls ``clear_error``. ``set_paused(True)`` keeps the robot ``busy``
with ``paused: true`` and freezes the active motion task's timeout clock by
cancelling its timer and storing the remaining seconds; ``set_paused(False)``
re-arms exactly that remainder. Every observable change bumps ``seq`` and
publishes one retained state.
"""

from __future__ import annotations

import json
import logging
import math
import re
import threading
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any

from ros2_ha_bridge.authoriser import GatingState, gate_motion
from ros2_ha_bridge.clock import Clock, Scheduler, TimerHandle
from ros2_ha_bridge.executors.base import (
    HEALTH_LEVELS,
    RESULT_STATUSES,
    BridgeContext,
    HealthLevel,
    TaskExecutor,
    TaskKind,
    TaskRejected,
    TaskRequest,
    TaskResult,
    TaskSpec,
)
from ros2_ha_bridge.protocol import (
    DEFAULT_PREFIX,
    DEFAULT_RESULT_EXPIRY_S,
    FEEDBACK_EXPIRY_S,
    AckCode,
    AckPayload,
    DecodedCommand,
    ExecutorHealth,
    FeedbackPayload,
    HaLink,
    ResultPayload,
    RobotError,
    RobotState,
    RobotTopics,
    StatePayload,
    WatchdogMode,
    local_instance,
    new_session,
    normalise_result_code,
)

__all__ = [
    'PostFn',
    'PublishFn',
    'TaskManager',
    'TaskManagerSettings',
]

#: ``publish(topic, payload, qos, retain, expiry_s) -> ok``: the node's guarded MQTT publish.
PublishFn = Callable[[str, Mapping[str, Any], int, bool, int | None], bool]
#: ``post(fn)``: run ``fn`` on the main thread as soon as it is free (thread-safe).
PostFn = Callable[[Callable[[], None]], None]

_LOGGER_NAME = 'ros2_ha_bridge.task_manager'
#: How much of an unserialisable ``data`` repr survives into the sanitised result.
_UNSERIALISABLE_REPR_CHARS = 512
#: Surrogate code points, the one thing a Python str can hold that UTF-8 cannot encode.
_SURROGATE_RE = re.compile('[\ud800-\udfff]')


@dataclass(frozen=True)
class TaskManagerSettings:
    """The node parameters that shape the task lifecycle (names as in ``bridge.example.yaml``)."""

    #: ``executor.cancel_grace_s``: how long a cancel may take before the finish is forced.
    cancel_grace_s: float = 5.0
    #: ``executor.call_warn_s``: a plugin call longer than this marks the executor degraded.
    call_warn_s: float = 0.1
    #: ``executor.health_poll_s``: period of :meth:`TaskManager.start_health_polling`.
    health_poll_s: float = 1.0
    #: ``reply.feedback_max_rate_hz``: per-task feedback throttle; 0 disables throttling.
    feedback_max_rate_hz: float = 2.0
    #: ``reply.result_expiry_s``: Message Expiry Interval of the retained result.
    result_expiry_s: int = DEFAULT_RESULT_EXPIRY_S


class _Handle:
    """The :class:`TaskHandle` given to the executor for one task.

    Thread-safe: both methods only post a closure to the main thread. The
    :attr:`finished` flag is set synchronously under the lock on the calling
    thread, so a second ``finish`` (or feedback after the finish) is logged and
    dropped there, and the main thread can tell "the executor has finished this
    task, the result is on its way" from "still running" before the posted
    closure has run. The main thread repeats the check against the manager's
    own records, so a finish that races a forced one is dropped there too.
    """

    def __init__(self, manager: TaskManager, task_id: str) -> None:
        self.task_id = task_id
        self._manager = manager
        self._lock = threading.Lock()
        self._finished = False

    @property
    def finished(self) -> bool:
        """True once ``finish`` was called (its result may still be queued) or the manager ended it."""
        with self._lock:
            return self._finished

    def feedback(self, progress: float | None = None, stage: str = '', message: str = '') -> None:
        with self._lock:
            finished = self._finished
        if finished:
            self._manager._logger.info(
                'task %s: feedback after the task finished; ignored', self.task_id
            )
            return
        manager, task_id = self._manager, self.task_id
        manager._post(lambda: manager._apply_feedback(task_id, progress, stage, message))

    def finish(self, result: TaskResult) -> None:
        with self._lock:
            if self._finished:
                self._manager._logger.warning(
                    'task %s: finish(%s/%s) called after the task already finished; ignored',
                    self.task_id,
                    getattr(result, 'status', '?'),
                    getattr(result, 'code', '?'),
                )
                return
            self._finished = True
        manager, task_id = self._manager, self.task_id
        manager._post(lambda: manager._apply_finish(task_id, result))

    def mark_finished(self) -> None:
        """Record a finish the manager made itself (forced or overridden)."""
        with self._lock:
            self._finished = True


@dataclass
class _ActiveTask:
    """Book-keeping for one accepted task until its result is published."""

    id: str
    name: str
    kind: TaskKind
    timeout_s: float
    started_at: int
    handle: _Handle
    deadline: TimerHandle | None = None
    #: Seconds left on the deadline while paused (the timer itself is cancelled).
    frozen_remaining_s: float | None = None
    grace: TimerHandle | None = None
    cancel_requested: bool = False
    cancel_failed: bool = False
    timed_out: bool = False
    #: A result that replaces whatever the executor reports (watchdog stop and hold).
    override: TaskResult | None = None
    last_feedback_monotonic: float | None = None
    finished: bool = False


class TaskManager:
    """Robot state machine plus task lifecycle for one robot. See the module docstring."""

    def __init__(
        self,
        *,
        robot_id: str,
        executor: TaskExecutor,
        publish: PublishFn,
        post: PostFn,
        clock: Clock,
        scheduler: Scheduler,
        session: str | None = None,
        instance: str | None = None,
        prefix: str = DEFAULT_PREFIX,
        watchdog_mode: WatchdogMode = 'none',
        settings: TaskManagerSettings | None = None,
        logger: logging.Logger | None = None,
    ) -> None:
        self.topics = RobotTopics(robot_id, prefix)
        self.robot_id = self.topics.robot_id
        self.session = session if session is not None else new_session()
        self.instance = instance if instance is not None else local_instance()
        self.settings = settings if settings is not None else TaskManagerSettings()
        self._executor = executor
        self._publish_fn = publish
        self._post = post
        self._clock = clock
        self._scheduler = scheduler
        self._logger = logger if logger is not None else logging.getLogger(_LOGGER_NAME)

        self._error: RobotError | None = None
        self._paused = False
        self._inhibited = False
        self._ha_link: HaLink = 'down'
        self._watchdog_mode: WatchdogMode = watchdog_mode
        self._health = ExecutorHealth()
        self._motion: _ActiveTask | None = None
        self._tasks: dict[str, _ActiveTask] = {}

        self._seq = 0
        self._since_ms = clock.now_ms()
        self._last_state_value: RobotState = 'idle'
        # What the world is assumed to know: the initial idle state. Only a change publishes;
        # the node publishes the first retained state explicitly when it connects.
        self._last_published: tuple[object, ...] = self._fingerprint()
        self._health_timer: TimerHandle | None = None
        self._polling = False

    # ------------------------------------------------------------------ read-only views

    @property
    def state(self) -> RobotState:
        """``error`` while an error is set, else ``busy`` with a motion task, else ``idle``."""
        if self._error is not None:
            return 'error'
        return 'busy' if self._motion is not None else 'idle'

    @property
    def is_busy(self) -> bool:
        return self._motion is not None

    @property
    def paused(self) -> bool:
        return self._paused

    @property
    def inhibited(self) -> bool:
        return self._inhibited

    @property
    def ha_link(self) -> HaLink:
        return self._ha_link

    @property
    def watchdog_mode(self) -> WatchdogMode:
        return self._watchdog_mode

    @property
    def error(self) -> RobotError | None:
        return self._error

    @property
    def executor_health(self) -> ExecutorHealth:
        return self._health

    @property
    def active_task_id(self) -> str | None:
        """Id of the motion task occupying the single active slot, or ``None``."""
        return None if self._motion is None else self._motion.id

    @property
    def active_task_name(self) -> str | None:
        return None if self._motion is None else self._motion.name

    @property
    def active_task_ids(self) -> tuple[str, ...]:
        """Ids of every unfinished task (motion, control and query), oldest first."""
        return tuple(self._tasks)

    @property
    def seq(self) -> int:
        """Sequence number of the last published state (0 before the first)."""
        return self._seq

    # ------------------------------------------------------------------ wiring helpers

    def bridge_context(
        self,
        *,
        ha_link_ok: Callable[[], bool],
        parameters: Mapping[str, Any] | None = None,
        node: Any = None,
    ) -> BridgeContext:
        """The :class:`BridgeContext` to give ``executor.setup``, wired to this manager.

        ``ha_link_ok`` comes from the watchdog; ``parameters`` carries the
        executor's node parameters keyed by their dotted names.
        """
        return BridgeContext(
            robot_id=self.robot_id,
            logger=self._logger,
            post=self._post,
            set_error=self.set_error,
            clear_error=self.clear_error,
            set_paused=self.set_paused,
            ha_link_ok=ha_link_ok,
            clock=self._clock,
            scheduler=self._scheduler,
            parameters=parameters,
            node=node,
        )

    def gating_state(self) -> GatingState:
        """The robot state as contract step 9 sees it; the node gives this to the authoriser."""
        motion = self._motion
        return GatingState(
            state=self.state,
            inhibited=self._inhibited,
            task_id=None if motion is None else motion.id,
            task=None if motion is None else motion.name,
            error=self._error,
        )

    def motion_rejection(self) -> tuple[AckCode, str] | None:
        """Why a motion task would be refused right now, or ``None`` when it may start.

        :func:`~ros2_ha_bridge.authoriser.gate_motion` over :meth:`gating_state`:
        the same rule and the same reason text the authoriser produces, applied
        again by :meth:`start` so the single slot can never be taken twice.
        """
        return gate_motion(self.gating_state())

    # ------------------------------------------------------------------ commands (main thread)

    def start(
        self, request: TaskRequest, spec: TaskSpec, *, received_at: int | None = None
    ) -> AckPayload:
        """Start ``request`` (the authoriser's resolved task command for ``spec``) and return the ack.

        ``request`` is ``Ack.request`` from the authoriser (or
        :func:`~ros2_ha_bridge.authoriser.resolve_request` in a test): the
        executor receives it unchanged and the deadline is armed from its
        ``timeout_s``. ``received_at`` is the node clock when the command
        arrived (defaults to now) and feeds the ack's ``received_at`` and
        ``clock_skew_ms``. The caller publishes the ack; a motion task's
        ``busy`` state is published here, so it may precede the ack on the wire
        (HA matches replies by id and does not depend on the order across
        topics).
        """
        if request.task != spec.name:
            raise ValueError(
                f'start() needs a request for task {spec.name!r}, got one for {request.task!r}'
            )
        if not request.timeout_s > 0:
            raise ValueError(
                f'request {request.id} has no usable timeout_s ({request.timeout_s!r})'
            )
        received = self._clock.now_ms() if received_at is None else received_at

        def ack(code: AckCode, reason: str = '') -> AckPayload:
            return self._ack(
                request.id, request.issued_at, 'task', spec.name, code, reason, received
            )

        if request.id in self._tasks:
            # Unreachable behind the dedupe store; refused rather than trusted.
            self._logger.error('task %s: a task with this id is already running', request.id)
            return ack('rejected_executor', 'A task with this command id is already running')
        if spec.kind == 'motion':
            rejection = self.motion_rejection()
            if rejection is not None:
                return ack(*rejection)

        task = _ActiveTask(
            id=request.id,
            name=spec.name,
            kind=spec.kind,
            timeout_s=request.timeout_s,
            started_at=self._clock.now_ms(),
            handle=_Handle(self, request.id),
        )
        # Registered before start() so a finish made inside start() finds its task.
        self._tasks[task.id] = task
        if spec.kind == 'motion':
            self._motion = task
        # The handle's flag, not the manager's record, decides below: with the node's deferred
        # post a finish made inside start() is still queued when start() returns.
        try:
            self._timed_call('start', lambda: self._executor.start(request, task.handle))
        except TaskRejected as refusal:
            if not task.handle.finished:
                self._unregister(task)
                self._logger.info(
                    'task %s (%s): refused by the executor: %s', task.id, task.name, refusal
                )
                self._publish_state_if_changed()
                return ack('rejected_executor', refusal.reason or str(refusal))
            self._logger.error(
                'task %s (%s): executor raised TaskRejected after finishing the task; '
                'the result stands',
                task.id,
                task.name,
            )
        except Exception as error:
            if not task.handle.finished:
                self._unregister(task)
                self._logger.exception(
                    'task %s (%s): executor.start raised; rejecting', task.id, task.name
                )
                self._publish_state_if_changed()
                return ack(
                    'rejected_executor', f'executor.start raised {type(error).__name__}: {error}'
                )
            self._logger.exception(
                'task %s (%s): executor.start raised after finishing the task; the result stands',
                task.id,
                task.name,
            )
        if task.handle.finished:
            self._logger.debug(
                'task %s (%s): finished inside start(); no deadline armed', task.id, task.name
            )
        else:
            task.deadline = self._scheduler.call_later(
                request.timeout_s, lambda: self._on_deadline(task.id)
            )
            self._logger.info(
                'task %s (%s, %s) started; timeout %g s',
                task.id,
                task.name,
                task.kind,
                request.timeout_s,
            )
        self._publish_state_if_changed()
        return ack('accepted')

    def cancel(self, command: DecodedCommand, *, received_at: int | None = None) -> AckPayload:
        """Handle a cancel command: resolve its target and return the (terminal) ack.

        ``target_id`` ``None`` means the active motion task. An unknown or
        finished target gives ``rejected_no_such_task``; otherwise the ack is
        ``accepted``, ``executor.cancel`` has been called and the cancelled
        task's own result follows under its own id (within ``cancel_grace_s``,
        forced if need be).
        """
        if not command.is_cancel:
            raise ValueError(f'cancel() needs a cancel command, got {command.type!r}')
        received = self._clock.now_ms() if received_at is None else received_at

        def ack(code: AckCode, reason: str = '') -> AckPayload:
            return self._ack(command.id, command.issued_at, 'cancel', None, code, reason, received)

        if command.target_id is None:
            task = self._motion
            if task is None:
                return ack('rejected_no_such_task', 'No motion task is running')
        else:
            found = self._tasks.get(command.target_id)
            if found is None:
                return ack('rejected_no_such_task', f'No running task has id {command.target_id}')
            task = found
        self._request_cancel(task, f'cancel command {command.id}')
        self._publish_state_if_changed()
        return ack('accepted')

    def stop_and_hold(self, reason: str) -> None:
        """Watchdog action: halt the executor and end the motion task ``cancelled/inhibited``.

        The executor's own ``stop_and_hold`` runs first. If it finished the
        active motion task through its handle (the result may still be queued
        behind the post), nothing more is asked of it and that finish carries
        the override when it is applied. Otherwise the task is cancelled and,
        unless the cancel finished it, completed at once with the override
        result; whatever the executor reports for it later is dropped. The
        robot does not enter ``error``: this is a deliberate action, and
        ``inhibited`` is what keeps new motion out.
        """
        self._logger.warning('stop and hold: %s', reason)
        motion = self._motion
        if motion is not None:
            motion.override = TaskResult('cancelled', 'inhibited', reason)
        try:
            self._timed_call('stop_and_hold', lambda: self._executor.stop_and_hold(reason))
        except Exception:
            self._logger.exception('executor.stop_and_hold raised')
        if motion is not None and not motion.finished:
            if motion.handle.finished:
                self._logger.debug(
                    'task %s (%s): finished by executor.stop_and_hold; the override applies '
                    'when its result arrives',
                    motion.id,
                    motion.name,
                )
            else:
                self._request_cancel(motion, 'stop and hold')
                if not motion.handle.finished and motion.override is not None:
                    self._complete(motion, motion.override, forced=False)
        self._publish_state_if_changed()

    # ------------------------------------------------------------------ flags (main thread)

    def set_inhibited(self, inhibited: bool) -> None:
        """Set ``state.inhibited`` (the watchdog's gate on new motion)."""
        if self._inhibited != inhibited:
            self._inhibited = inhibited
            self._logger.info('inhibited: %s', inhibited)
        self._publish_state_if_changed()

    def set_ha_link(self, up: bool, *, inhibited: bool | None = None) -> None:
        """Set ``state.ha_link`` and, when given, ``inhibited`` in one state publication."""
        link: HaLink = 'up' if up else 'down'
        if self._ha_link != link:
            self._ha_link = link
            self._logger.info('ha_link: %s', link)
        if inhibited is not None and self._inhibited != inhibited:
            self._inhibited = inhibited
            self._logger.info('inhibited: %s', inhibited)
        self._publish_state_if_changed()

    def publish_state(self) -> bool:
        """Publish the retained state unconditionally (on connect and reconnect)."""
        now = self._clock.now_ms()
        fingerprint = self._fingerprint()
        state = self.state
        if state != self._last_state_value:
            self._last_state_value = state
            self._since_ms = now
        self._seq += 1
        motion = self._motion
        payload = StatePayload(
            robot_id=self.robot_id,
            state=state,
            task_id=None if motion is None else motion.id,
            task=None if motion is None else motion.name,
            since=self._since_ms,
            paused=self._paused,
            inhibited=self._inhibited,
            ha_link=self._ha_link,
            watchdog_mode=self._watchdog_mode,
            executor_health=self._health,
            error=self._error,
            session=self.session,
            instance=self.instance,
            seq=self._seq,
            ts=now,
        ).to_payload()
        self._last_published = fingerprint
        return self._publish(self.topics.state, payload, 1, True, None)

    # ------------------------------------------------------------------ health (main thread)

    def poll_health(self) -> ExecutorHealth:
        """Ask the executor for its health, record it and publish the state if it changed.

        A ``health()`` that raises or returns an unknown level is recorded as
        ``error`` with the exception in the message (one traceback per distinct
        message).
        """
        level: HealthLevel
        message: str
        try:
            reported_level, reported_message = self._executor.health()
        except Exception as error:
            message = f'health() raised {type(error).__name__}: {error}'
            if message != self._health.message:
                self._logger.exception('executor.health raised')
            level = 'error'
        else:
            message = str(reported_message)
            if reported_level in HEALTH_LEVELS:
                level = reported_level
            else:
                level = 'error'
                message = f'health() returned an unknown level {reported_level!r}: {message}'
        health = ExecutorHealth(level, message)
        if health != self._health:
            log = self._logger.info if level == 'ok' else self._logger.warning
            log('executor health: %s %s', level, message)
            self._health = health
        self._publish_state_if_changed()
        return health

    def start_health_polling(self) -> None:
        """Poll ``health()`` every ``health_poll_s`` through the scheduler until stopped."""
        if self._polling:
            return
        self._polling = True
        self._arm_health_timer()

    def stop_health_polling(self) -> None:
        self._polling = False
        if self._health_timer is not None:
            self._health_timer.cancel()
            self._health_timer = None

    def _arm_health_timer(self) -> None:
        self._health_timer = self._scheduler.call_later(
            self.settings.health_poll_s, self._health_tick
        )

    def _health_tick(self) -> None:
        self._health_timer = None
        if not self._polling:
            return
        self.poll_health()
        if self._polling:
            self._arm_health_timer()

    # ------------------------------------------------------------------ executor-facing (any thread)

    def set_error(self, code: str, reason: str) -> None:
        """Put the robot into ``error`` (thread-safe; applied on the main thread)."""
        self._post(lambda: self._apply_error(code, reason))

    def clear_error(self) -> None:
        """Leave ``error`` (thread-safe; applied on the main thread)."""
        self._post(self._apply_clear_error)

    def set_paused(self, paused: bool) -> None:
        """Pause or resume the motion task's clock (thread-safe; applied on the main thread)."""
        self._post(lambda: self._apply_paused(paused))

    # ------------------------------------------------------------------ main-thread appliers

    def _apply_error(self, code: str, reason: str) -> None:
        # set_error is executor-facing: coerce both fields like every other plugin input, or
        # a non-string reason would make the state publish fail inside the encoder.
        error = RobotError(normalise_result_code(code), _wire_text(reason))
        if error != self._error:
            self._error = error
            self._logger.error('robot error set: %s: %s', error.code, error.reason)
        self._publish_state_if_changed()

    def _apply_clear_error(self) -> None:
        if self._error is not None:
            self._logger.info('robot error cleared (was %s)', self._error.code)
            self._error = None
        self._publish_state_if_changed()

    def _apply_paused(self, paused: bool) -> None:
        motion = self._motion
        if paused:
            if motion is None:
                self._logger.warning('set_paused(True) with no active motion task; ignored')
                return
            if not self._paused:
                self._paused = True
                if motion.deadline is not None:
                    motion.frozen_remaining_s = motion.deadline.remaining_s()
                    motion.deadline.cancel()
                    motion.deadline = None
                left = motion.frozen_remaining_s
                self._logger.info(
                    'task %s (%s) paused; %s s of timeout left',
                    motion.id,
                    motion.name,
                    'no' if left is None else f'{left:g}',
                )
        elif self._paused:
            self._paused = False
            if motion is not None and motion.frozen_remaining_s is not None:
                remaining = motion.frozen_remaining_s
                motion.frozen_remaining_s = None
                task_id = motion.id
                motion.deadline = self._scheduler.call_later(
                    remaining, lambda: self._on_deadline(task_id)
                )
                self._logger.info(
                    'task %s (%s) resumed; timeout re-armed for %g s',
                    motion.id,
                    motion.name,
                    remaining,
                )
        self._publish_state_if_changed()

    def _apply_feedback(
        self, task_id: str, progress: float | None, stage: str, message: str
    ) -> None:
        task = self._tasks.get(task_id)
        if task is None or task.finished:
            self._logger.debug('task %s: feedback for a finished task; ignored', task_id)
            return
        now = self._clock.monotonic()
        rate = self.settings.feedback_max_rate_hz
        last = task.last_feedback_monotonic
        if rate > 0 and last is not None and now - last < (1.0 / rate) - 1e-9:
            self._logger.debug('task %s: feedback throttled', task_id)
            return
        task.last_feedback_monotonic = now
        payload = FeedbackPayload(
            id=task.id,
            robot_id=self.robot_id,
            session=self.session,
            ts=self._clock.now_ms(),
            progress=_clamp_progress(progress),
            stage=_wire_text(stage),
            message=_wire_text(message),
        ).to_payload()
        self._publish(self.topics.feedback(task.id), payload, 0, False, FEEDBACK_EXPIRY_S)

    def _apply_finish(self, task_id: str, result: TaskResult) -> None:
        task = self._tasks.get(task_id)
        if task is None or task.finished:
            self._logger.warning(
                'task %s: finish(%s/%s) arrived after the task had already finished; ignored',
                task_id,
                getattr(result, 'status', '?'),
                getattr(result, 'code', '?'),
            )
            return
        self._complete(task, result, forced=False)
        self._publish_state_if_changed()

    # ------------------------------------------------------------------ timers (main thread)

    def _on_deadline(self, task_id: str) -> None:
        task = self._tasks.get(task_id)
        if task is None or task.finished:
            return
        task.deadline = None
        if task.handle.finished:
            self._logger.debug(
                'task %s (%s): deadline passed with its finish already queued; not cancelling',
                task.id,
                task.name,
            )
            return
        task.timed_out = True
        self._logger.warning(
            'task %s (%s) did not finish within %g s; asking the executor to cancel',
            task.id,
            task.name,
            task.timeout_s,
        )
        self._request_cancel(task, 'timeout')
        self._publish_state_if_changed()

    def _on_grace(self, task_id: str) -> None:
        task = self._tasks.get(task_id)
        if task is None or task.finished:
            return
        task.grace = None
        if task.handle.finished:
            self._logger.debug(
                'task %s (%s): grace passed with its finish already queued; not forcing',
                task.id,
                task.name,
            )
            return
        grace = self.settings.cancel_grace_s
        name, timeout = task.name, task.timeout_s
        if task.timed_out and not task.cancel_failed:
            code = 'timeout'
            reason = f"Task '{name}' did not finish within {timeout:g} s and ignored cancel"
        elif task.timed_out:
            code = 'executor_unresponsive'
            reason = f"Task '{name}' did not finish within {timeout:g} s and cancel() raised"
        elif task.cancel_failed:
            code = 'executor_unresponsive'
            reason = f"Task '{name}': cancel() raised and no finish followed in {grace:g} s"
        else:
            code = 'executor_unresponsive'
            reason = f"Task '{name}' ignored cancel for {grace:g} s"
        self._complete(task, TaskResult('aborted', code, reason), forced=True)
        self._publish_state_if_changed()

    def _request_cancel(self, task: _ActiveTask, why: str) -> None:
        task.cancel_requested = True
        self._logger.info('task %s (%s): cancelling (%s)', task.id, task.name, why)
        try:
            self._timed_call('cancel', lambda: self._executor.cancel(task.id))
        except Exception:
            task.cancel_failed = True
            self._logger.exception('task %s (%s): executor.cancel raised', task.id, task.name)
        if not task.handle.finished and task.grace is None:
            task.grace = self._scheduler.call_later(
                self.settings.cancel_grace_s, lambda: self._on_grace(task.id)
            )

    # ------------------------------------------------------------------ completion (main thread)

    def _complete(self, task: _ActiveTask, result: TaskResult, *, forced: bool) -> None:
        """Publish the result and release the task; the caller publishes the state."""
        if task.finished:
            return
        task.finished = True
        task.handle.mark_finished()
        for timer in (task.deadline, task.grace):
            if timer is not None:
                timer.cancel()
        task.deadline = task.grace = None
        self._tasks.pop(task.id, None)
        if self._motion is task:
            self._motion = None
            if self._paused:
                self._paused = False
                self._logger.info(
                    'task %s (%s) finished while paused; paused cleared', task.id, task.name
                )
        result = _sanitise_result(result, self._logger, task.id)
        if task.override is not None:
            if (result.status, result.code) != (task.override.status, task.override.code):
                self._logger.info(
                    'task %s (%s): executor reported %s/%s; overridden by %s/%s',
                    task.id,
                    task.name,
                    result.status,
                    result.code,
                    task.override.status,
                    task.override.code,
                )
            result = task.override
        if forced:
            self._error = RobotError(result.code, result.reason)
            self._logger.error(
                'task %s (%s) forced to %s/%s; robot in error until cleared: %s',
                task.id,
                task.name,
                result.status,
                result.code,
                result.reason,
            )
        else:
            self._logger.info(
                'task %s (%s) finished %s/%s %s',
                task.id,
                task.name,
                result.status,
                result.code,
                result.reason,
            )
        now = self._clock.now_ms()
        payload = ResultPayload(
            id=task.id,
            robot_id=self.robot_id,
            task=task.name,
            kind=task.kind,
            status=result.status,
            code=result.code,
            reason=result.reason,
            data=result.data,
            started_at=task.started_at,
            finished_at=now,
            session=self.session,
            ts=now,
        ).to_payload()
        self._publish(self.topics.result(task.id), payload, 1, True, self.settings.result_expiry_s)

    def _unregister(self, task: _ActiveTask) -> None:
        self._tasks.pop(task.id, None)
        if self._motion is task:
            self._motion = None
        for timer in (task.deadline, task.grace):
            if timer is not None:
                timer.cancel()
        task.deadline = task.grace = None
        task.finished = True
        task.handle.mark_finished()

    # ------------------------------------------------------------------ helpers

    def _ack(
        self,
        command_id: str,
        issued_at: int,
        kind: str,
        task: str | None,
        code: AckCode,
        reason: str,
        received_at: int,
    ) -> AckPayload:
        return AckPayload(
            id=command_id,
            robot_id=self.robot_id,
            type='task' if kind == 'task' else 'cancel',
            task=task,
            code=code,
            reason=reason,
            received_at=received_at,
            clock_skew_ms=received_at - issued_at,
            session=self.session,
            ts=self._clock.now_ms(),
        )

    def _timed_call(self, name: str, fn: Callable[[], None]) -> None:
        """Run a plugin call, marking the executor degraded when it overran ``call_warn_s``."""
        started = self._clock.monotonic()
        try:
            fn()
        finally:
            elapsed = self._clock.monotonic() - started
            if elapsed > self.settings.call_warn_s:
                self._logger.warning(
                    'executor.%s() took %.2f s (longer than %.2f s)',
                    name,
                    elapsed,
                    self.settings.call_warn_s,
                )
                if self._health.level == 'ok':
                    self._health = ExecutorHealth('degraded', f'{name}() took {elapsed:.2f} s')

    def _fingerprint(self) -> tuple[object, ...]:
        motion = self._motion
        return (
            self.state,
            None if motion is None else motion.id,
            None if motion is None else motion.name,
            self._paused,
            self._inhibited,
            self._ha_link,
            self._watchdog_mode,
            self._health,
            self._error,
        )

    def _publish_state_if_changed(self) -> bool:
        if self._fingerprint() == self._last_published:
            return False
        self.publish_state()
        return True

    def _publish(
        self, topic: str, payload: Mapping[str, Any], qos: int, retain: bool, expiry_s: int | None
    ) -> bool:
        """Call the node's publish; a publish may fail but must never break the state machine."""
        try:
            ok = self._publish_fn(topic, payload, qos, retain, expiry_s)
        except Exception:
            self._logger.exception('publish to %s raised; the link should have caught this', topic)
            return False
        if not ok:
            self._logger.debug('publish to %s reported failure', topic)
        return ok


def _clamp_progress(progress: object) -> float | None:
    """``progress`` as the schema wants it: a number in 0 to 1, or ``None``."""
    if progress is None or isinstance(progress, bool) or not isinstance(progress, (int, float)):
        return None
    value = float(progress)
    if math.isnan(value):
        return None
    return min(1.0, max(0.0, value))


def _wire_text(value: object) -> str:
    """``value`` as a string the UTF-8 encoder accepts (a lone surrogate becomes U+FFFD)."""
    text = value if isinstance(value, str) else str(value)
    try:
        text.encode('utf-8')
    except UnicodeEncodeError:
        text = _SURROGATE_RE.sub('\ufffd', text)
    return text


def _encodable(data: Mapping[str, Any]) -> bool:
    """True when ``data`` survives the wire encoding (strict JSON, UTF-8)."""
    try:
        json.dumps(dict(data), ensure_ascii=False, allow_nan=False).encode('utf-8')
    except (TypeError, ValueError):
        return False
    return True


def _sanitise_result(result: TaskResult, log: logging.Logger, task_id: str) -> TaskResult:
    """Keep a sloppy executor from producing a result the schema or the encoder would reject.

    The contract promises exactly one result per accepted task, and the
    manager publishes it exactly once with no retry, so the payload must
    encode first time: ``data`` is round-tripped through the wire encoding
    here and replaced by ``{'unserialisable': repr(...)}`` when a value
    (a datetime, bytes, a set, NaN, a tuple key) would make the encoder raise
    inside the publish.
    """
    status: object = getattr(result, 'status', None)
    code: object = getattr(result, 'code', None)
    reason: object = getattr(result, 'reason', '')
    data: object = getattr(result, 'data', None)
    problems: list[str] = []
    if status not in RESULT_STATUSES:
        problems.append(f'status {status!r}')
        status = 'aborted'
        code = 'executor_error'
    if not isinstance(code, str):
        problems.append(f'code {code!r}')
        code = 'unknown'
    if not isinstance(reason, str) or _wire_text(reason) != reason:
        problems.append(f'reason {reason!r}')
        reason = _wire_text(reason)
    if not isinstance(data, Mapping):
        problems.append(f'data {type(data).__name__}')
        data = {} if data is None else {'value': repr(data)}
    elif not _encodable(data):
        problems.append('data (not JSON-encodable)')
        data = {'unserialisable': repr(dict(data))[:_UNSERIALISABLE_REPR_CHARS]}
    if not problems:
        return result
    log.warning('task %s: executor result had invalid %s; sanitised', task_id, ', '.join(problems))
    for candidate in RESULT_STATUSES:
        if status == candidate:
            return TaskResult(candidate, code, reason, dict(data))
    return TaskResult('aborted', code, reason, dict(data))
