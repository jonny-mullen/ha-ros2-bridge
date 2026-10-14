# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The shipped fake executor: a robot that exists only as timers.

``FakeExecutor`` lets the whole bridge, and the Home Assistant integration on
the other side of the broker, be exercised with no ROS driver, no MoveIt and
no simulator. Every task is deterministic against the injected
``BridgeContext.clock`` and ``BridgeContext.scheduler``: a motion task takes
``fake.task_duration_s`` (2.0) of scheduler time and reports feedback at
``fake.feedback_hz`` (2), so a test drives it with ``FakeClock.advance()`` and
the live node drives it with real rclpy timers.

Joints (0.2.0): the fake simulates a seven-joint arm, ``j1`` to ``j7`` in
degrees, limited to [-170, 170] except ``j4`` and ``j6`` at [-120, 120]. It
starts at the ``home`` pose (all zero) and keeps its positions across tasks.
``reset`` and ``move_to`` interpolate linearly from the current positions to
the target pose over ``task_duration_s``, reporting positions and velocities
through ``ctx.report_joints`` once at ``setup``, on every feedback tick
(``1 / feedback_hz`` seconds) and at the end of every motion; ``wait`` holds;
``fail`` moves halfway towards the ``pick`` pose and then aborts; ``pause``
freezes the positions where they are and ``resume`` continues from there; a
cancel, ``stop`` or ``stop_and_hold`` freezes them where they are. The
velocities are the derivative of that interpolation, in degrees per second:
``(target - start) * motion_fraction / task_duration_s`` per joint while a
motion runs, and zero whenever the arm holds, is paused, is frozen or has
arrived, so the final report of every motion carries zeros. The named poses
are ``home``, ``pick``, ``place`` and ``wave``; ``move_to`` offers them as an
enum and refuses any other name.

Task catalogue (the ``kind`` decides how the bridge gates it):

``reset`` (motion)
    Moves to ``home`` and succeeds after ``task_duration_s`` with ``ok``.
``move_to {pose}`` (motion)
    Moves to the named pose and succeeds after ``task_duration_s`` with
    ``ok`` and ``data.pose``.
``wait {seconds}`` (motion)
    Runs for ``seconds`` with progress feedback, holding position; cancellable.
``fail {code}`` (motion)
    Moves halfway towards ``pick`` and aborts after ``task_duration_s`` with
    the chosen MoveIt failure class: ``collision``, ``self_collision``,
    ``no_trajectory``, ``planning_failed``, ``control_failed`` or
    ``invalid_goal``.
``hang`` (motion, timeout 10 s)
    Never finishes and ignores ``cancel``, so the bridge's task timeout, cancel
    grace and forced ``aborted/timeout`` can be seen end to end.
``raise_error`` / ``clear_error`` (control)
    ``ctx.set_error`` and ``ctx.clear_error``.
``stop`` (control)
    Ends the running motion task ``cancelled/preempted`` and clears any error.
``pause`` / ``resume`` (control)
    Freeze and continue the running motion task (its own timers and, through
    ``ctx.set_paused``, the bridge's timeout clock). Nothing to pause gives
    ``aborted/nothing_to_pause``; nothing to resume gives
    ``aborted/nothing_to_resume``.

It is selected with the ``executor`` parameter set to ``fake`` (the entry
point name under ``ros2_ha_bridge.executors``, declared in ``setup.py`` in
phase 2) or to ``ros2_ha_bridge.executors.fake:FakeExecutor``.

Everything here runs on the bridge's main thread (``start`` and ``cancel``
are called there and the scheduler fires there), so the fake needs no locks;
it still reports through the handle and the context like any plugin would.
"""

from __future__ import annotations

import logging
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

from ros2_ha_bridge import __version__
from ros2_ha_bridge.clock import Clock, Scheduler, TimerHandle
from ros2_ha_bridge.executors.base import (
    BridgeContext,
    HealthLevel,
    JointInfo,
    JointUnit,
    TaskExecutor,
    TaskHandle,
    TaskRejected,
    TaskRequest,
    TaskResult,
    TaskSpec,
)
from ros2_ha_bridge.protocol import validate_params

__all__ = [
    'DEFAULT_FEEDBACK_HZ',
    'DEFAULT_TASK_DURATION_S',
    'ENTRY_POINT_NAME',
    'FAILURE_CLASSES',
    'FAILURE_REASONS',
    'FAIL_MOTION_FRACTION',
    'FAIL_TOWARDS_POSE',
    'HANG_TIMEOUT_S',
    'HOME_POSE',
    'JOINT_INFO',
    'JOINT_LIMITS',
    'JOINT_NAMES',
    'JOINT_UNIT',
    'POSES',
    'FakeExecutor',
    'interpolate',
    'velocity',
]

#: The entry-point name this executor is registered under (``executor: fake``).
ENTRY_POINT_NAME = 'fake'
#: ``fake.task_duration_s`` default: how long reset, move_to and fail take.
DEFAULT_TASK_DURATION_S = 2.0
#: ``fake.feedback_hz`` default: feedback (and joint report) rate of a running motion task.
DEFAULT_FEEDBACK_HZ = 2.0
#: Announced timeout of ``hang``; the bridge forces the abort this long after acceptance.
HANG_TIMEOUT_S = 10.0

#: The MoveIt failure classes ``fail {code}`` can imitate, in the contract's vocabulary.
FAILURE_CLASSES: tuple[str, ...] = (
    'collision',
    'self_collision',
    'no_trajectory',
    'planning_failed',
    'control_failed',
    'invalid_goal',
)
FAILURE_REASONS: Mapping[str, str] = {
    'collision': 'Planned path collides with an object',
    'self_collision': 'Planned path brings the arm into collision with itself',
    'no_trajectory': 'No trajectory reaches the goal',
    'planning_failed': 'Motion planning failed',
    'control_failed': 'The controller failed to follow the trajectory',
    'invalid_goal': 'The goal is not valid for this arm',
}

# --------------------------------------------------------------------------- the seven joints

#: The simulated arm: seven joints in degrees, as the contract's golden announce lists them.
JOINT_NAMES: tuple[str, ...] = ('j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'j7')
JOINT_UNIT: JointUnit = 'deg'
JOINT_LIMITS: tuple[tuple[float, float], ...] = (
    (-170.0, 170.0),
    (-170.0, 170.0),
    (-170.0, 170.0),
    (-120.0, 120.0),
    (-170.0, 170.0),
    (-120.0, 120.0),
    (-170.0, 170.0),
)
JOINT_INFO = JointInfo(names=JOINT_NAMES, unit=JOINT_UNIT, limits=JOINT_LIMITS)
#: The pose ``reset`` returns to, and where the arm starts.
HOME_POSE = 'home'
#: Named poses in degrees, every value inside its joint's limits; ``move_to`` offers these.
POSES: Mapping[str, tuple[float, ...]] = {
    'home': (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    'pick': (35.0, -40.0, 20.0, -95.0, 10.0, 60.0, 45.0),
    'place': (-50.0, -30.0, 15.0, -80.0, -20.0, 75.0, -30.0),
    'wave': (0.0, -20.0, 0.0, -60.0, 90.0, 40.0, 0.0),
}
#: ``fail`` heads for this pose and covers this fraction of the way before it aborts.
FAIL_TOWARDS_POSE = 'pick'
FAIL_MOTION_FRACTION = 0.5

_LOGGER_NAME = 'ros2_ha_bridge.executors.fake'

_MOVE_TO_PARAMS: Mapping[str, Any] = {
    'type': 'object',
    'properties': {
        'pose': {
            'type': 'string',
            'enum': list(POSES),
            'description': 'Named pose to move to',
        }
    },
    'required': ['pose'],
}
_WAIT_PARAMS: Mapping[str, Any] = {
    'type': 'object',
    'properties': {
        'seconds': {
            'type': 'number',
            'minimum': 0,
            'maximum': 600,
            'description': 'How long to wait',
        }
    },
    'required': ['seconds'],
}
_FAIL_PARAMS: Mapping[str, Any] = {
    'type': 'object',
    'properties': {
        'code': {
            'type': 'string',
            'enum': list(FAILURE_CLASSES),
            'description': 'The MoveIt failure class to imitate',
        }
    },
    'required': ['code'],
}

_CATALOGUE: tuple[TaskSpec, ...] = (
    TaskSpec('reset', 'Reset to home', 'motion', 60, None, 'Return to the home pose'),
    TaskSpec('move_to', 'Move to named pose', 'motion', 120, _MOVE_TO_PARAMS),
    TaskSpec('wait', 'Wait', 'motion', 600, _WAIT_PARAMS, 'Wait with progress; cancellable'),
    TaskSpec('fail', 'Fail', 'motion', 60, _FAIL_PARAMS, 'Abort with a MoveIt failure class'),
    TaskSpec(
        'hang',
        'Hang',
        'motion',
        HANG_TIMEOUT_S,
        None,
        'Never finish and ignore cancel, to exercise the timeout path',
    ),
    TaskSpec('raise_error', 'Raise error', 'control', 5, None, 'Put the robot into error'),
    TaskSpec('clear_error', 'Clear error', 'control', 5, None, 'Leave the error state'),
    TaskSpec('stop', 'Stop', 'control', 5, None, 'Stop the running task and clear any error'),
    TaskSpec('pause', 'Pause', 'control', 5, None, 'Pause the running task'),
    TaskSpec('resume', 'Resume', 'control', 5, None, 'Resume the paused task'),
)
_SPECS: Mapping[str, TaskSpec] = {spec.name: spec for spec in _CATALOGUE}


def interpolate(
    start: Sequence[float], target: Sequence[float], fraction: float
) -> tuple[float, ...]:
    """Positions ``fraction`` of the way (0 to 1, clamped) from ``start`` to ``target``."""
    fraction = min(1.0, max(0.0, fraction))
    return tuple(round(a + (b - a) * fraction, 6) for a, b in zip(start, target, strict=True))


def velocity(
    start: Sequence[float], target: Sequence[float], fraction: float, duration_s: float
) -> tuple[float, ...]:
    """The derivative of :func:`interpolate` over a motion that covers ``fraction`` of the way
    from ``start`` to ``target`` in ``duration_s`` seconds: constant per joint, in the joint
    unit per second. Zero for a motion of no duration.
    """
    if duration_s <= 0:
        return (0.0,) * len(start)
    fraction = min(1.0, max(0.0, fraction))
    return tuple(
        round((b - a) * fraction / duration_s, 6) for a, b in zip(start, target, strict=True)
    )


def _still(count: int) -> tuple[float, ...]:
    """Velocities of an arm that holds, is paused or frozen, or has arrived."""
    return (0.0,) * count


@dataclass
class _Run:
    """One motion task in flight: its timers, its motion and what it will finish with."""

    request: TaskRequest
    handle: TaskHandle
    duration_s: float
    outcome: TaskResult
    started_monotonic: float
    #: Where the joints were when the task began.
    start_positions: tuple[float, ...]
    #: Where the motion heads; ``None`` holds position (``wait``).
    target_positions: tuple[float, ...] | None = None
    #: How much of the way to the target the task covers (``fail`` stops halfway).
    motion_fraction: float = 1.0
    elapsed_before_pause_s: float = 0.0
    paused_remaining_s: float | None = None
    completion: TimerHandle | None = None
    ticker: TimerHandle | None = None

    @property
    def paused(self) -> bool:
        return self.paused_remaining_s is not None


class FakeExecutor(TaskExecutor):
    """A deterministic stand-in for a robot. See the module docstring."""

    name = ENTRY_POINT_NAME
    version = __version__

    def __init__(self) -> None:
        self._ctx: BridgeContext | None = None
        self._logger = logging.getLogger(_LOGGER_NAME)
        self._runs: dict[str, _Run] = {}
        self.task_duration_s = DEFAULT_TASK_DURATION_S
        self.feedback_hz = DEFAULT_FEEDBACK_HZ
        #: What :meth:`health` reports; a test or a diagnostic may change it.
        self.health_report: tuple[HealthLevel, str] = ('ok', '')
        #: Every reason ``stop_and_hold`` was called with, oldest first.
        self.hold_reasons: list[str] = []
        self._positions: tuple[float, ...] = POSES[HOME_POSE]
        self._velocities: tuple[float, ...] = _still(len(JOINT_NAMES))

    # ------------------------------------------------------------------ TaskExecutor

    def setup(self, ctx: BridgeContext) -> None:
        if ctx.scheduler is None:
            raise RuntimeError('FakeExecutor needs ctx.scheduler to run its timers')
        self._ctx = ctx
        self._logger = ctx.logger.getChild('fake') if ctx.logger is not None else self._logger
        self.task_duration_s = _positive_float(
            ctx.parameters.get('fake.task_duration_s'), DEFAULT_TASK_DURATION_S
        )
        self.feedback_hz = _non_negative_float(
            ctx.parameters.get('fake.feedback_hz'), DEFAULT_FEEDBACK_HZ
        )
        self._logger.info(
            'fake executor ready for %s: task_duration_s=%g feedback_hz=%g joints=%d at %s',
            ctx.robot_id,
            self.task_duration_s,
            self.feedback_hz,
            len(JOINT_NAMES),
            HOME_POSE,
        )
        # The starting pose, so Home Assistant sees the joints before the first motion.
        self._report_joints()

    def tasks(self) -> Sequence[TaskSpec]:
        return _CATALOGUE

    def joints(self) -> JointInfo:
        return JOINT_INFO

    def start(self, req: TaskRequest, handle: TaskHandle) -> None:
        spec = _SPECS.get(req.task)
        if spec is None:
            raise TaskRejected(reason=f'unknown task {req.task!r}')
        problems = validate_params(spec.params, req.params)
        if problems:
            raise TaskRejected(reason='; '.join(problems))
        if req.task == 'reset':
            self._begin(
                req,
                handle,
                self.task_duration_s,
                TaskResult('succeeded', 'ok', f"Moved to pose '{HOME_POSE}'"),
                target=POSES[HOME_POSE],
            )
        elif req.task == 'move_to':
            pose = str(req.params['pose'])
            target = POSES.get(pose)
            if target is None:
                known = ', '.join(POSES)
                raise TaskRejected(reason=f'unknown pose {pose!r}; known poses: {known}')
            outcome = TaskResult('succeeded', 'ok', f"Moved to pose '{pose}'", {'pose': pose})
            self._begin(req, handle, self.task_duration_s, outcome, target=target)
        elif req.task == 'wait':
            seconds = float(req.params['seconds'])
            outcome = TaskResult('succeeded', 'ok', f'Waited {seconds:g} s', {'seconds': seconds})
            self._begin(req, handle, seconds, outcome)
        elif req.task == 'fail':
            code = str(req.params['code'])
            outcome = TaskResult(
                'aborted',
                code,
                FAILURE_REASONS[code],
                {'simulated': True, 'failure_class': code},
            )
            self._begin(
                req,
                handle,
                self.task_duration_s,
                outcome,
                target=POSES[FAIL_TOWARDS_POSE],
                motion_fraction=FAIL_MOTION_FRACTION,
            )
        elif req.task == 'hang':
            self._logger.info('task %s: hanging on purpose', req.id)
        elif req.task == 'raise_error':
            self._context().set_error('executor_error', 'Error raised by the fake executor')
            handle.finish(TaskResult('succeeded', 'ok', 'Robot error raised'))
        elif req.task == 'clear_error':
            self._context().clear_error()
            handle.finish(TaskResult('succeeded', 'ok', 'Robot error cleared'))
        elif req.task == 'stop':
            handle.finish(self._stop())
        elif req.task == 'pause':
            handle.finish(self._pause())
        else:  # resume
            handle.finish(self._resume())

    def cancel(self, task_id: str) -> None:
        run = self._runs.pop(task_id, None)
        if run is None:
            # hang is never registered, so this is also the "ignores cancel" path.
            self._logger.info('cancel for %s: nothing running under that id', task_id)
            return
        self._freeze(run)
        self._disarm(run)
        run.handle.finish(TaskResult('cancelled', 'cancelled', 'Cancelled by request'))

    def stop_and_hold(self, reason: str) -> None:
        self.hold_reasons.append(reason)
        self._logger.warning('stop and hold: %s', reason)
        for run in self._drain_runs():
            run.handle.finish(TaskResult('cancelled', 'inhibited', reason))

    def health(self) -> tuple[HealthLevel, str]:
        return self.health_report

    def shutdown(self) -> None:
        for run in self._drain_runs():
            self._logger.info('shutdown: dropping task %s', run.request.id)

    # ------------------------------------------------------------------ views

    @property
    def running_ids(self) -> tuple[str, ...]:
        """Ids of the motion tasks the fake is currently running (paused ones included)."""
        return tuple(self._runs)

    def is_paused(self, task_id: str) -> bool:
        run = self._runs.get(task_id)
        return run is not None and run.paused

    @property
    def positions(self) -> tuple[float, ...]:
        """The joints as last reported, in degrees, in :data:`JOINT_NAMES` order."""
        return self._positions

    @property
    def velocities(self) -> tuple[float, ...]:
        """The joint velocities as last reported, in degrees per second; zeros when still."""
        return self._velocities

    # ------------------------------------------------------------------ motion runs

    def _begin(
        self,
        req: TaskRequest,
        handle: TaskHandle,
        duration_s: float,
        outcome: TaskResult,
        *,
        target: tuple[float, ...] | None = None,
        motion_fraction: float = 1.0,
    ) -> None:
        run = _Run(
            req,
            handle,
            duration_s,
            outcome,
            self._clock().monotonic(),
            self._positions,
            target,
            motion_fraction,
        )
        if duration_s <= 0:
            self._arrive(run)
            handle.finish(outcome)
            return
        self._runs[req.id] = run
        self._arm(run, duration_s)

    def _arm(self, run: _Run, remaining_s: float) -> None:
        task_id = run.request.id
        run.completion = self._scheduler().call_later(remaining_s, lambda: self._finish(task_id))
        if self.feedback_hz > 0:
            run.ticker = self._scheduler().call_later(
                1.0 / self.feedback_hz, lambda: self._tick(task_id)
            )

    def _disarm(self, run: _Run) -> None:
        for timer in (run.completion, run.ticker):
            if timer is not None:
                timer.cancel()
        run.completion = run.ticker = None

    def _finish(self, task_id: str) -> None:
        run = self._runs.pop(task_id, None)
        if run is None:
            return
        self._disarm(run)
        self._arrive(run)
        run.handle.finish(run.outcome)

    def _tick(self, task_id: str) -> None:
        run = self._runs.get(task_id)
        if run is None or run.paused:
            return
        elapsed = self._elapsed(run)
        progress = min(1.0, elapsed / run.duration_s) if run.duration_s > 0 else None
        self._move_to(self._positions_at(run), self._velocities_at(run))
        run.handle.feedback(progress, 'executing', f'{elapsed:.1f} of {run.duration_s:g} s')
        run.ticker = self._scheduler().call_later(
            1.0 / self.feedback_hz, lambda: self._tick(task_id)
        )

    def _drain_runs(self) -> list[_Run]:
        runs = list(self._runs.values())
        self._runs.clear()
        for run in runs:
            self._freeze(run)
            self._disarm(run)
        return runs

    # ------------------------------------------------------------------ joints

    def _elapsed(self, run: _Run) -> float:
        """Seconds of motion so far, pauses excluded."""
        if run.paused:
            return run.elapsed_before_pause_s
        return run.elapsed_before_pause_s + (self._clock().monotonic() - run.started_monotonic)

    def _positions_at(self, run: _Run) -> tuple[float, ...]:
        """Where the joints are now along ``run``'s motion (unchanged for a holding task)."""
        if run.target_positions is None:
            return self._positions
        progress = 1.0 if run.duration_s <= 0 else min(1.0, self._elapsed(run) / run.duration_s)
        return interpolate(
            run.start_positions, run.target_positions, progress * run.motion_fraction
        )

    def _velocities_at(self, run: _Run) -> tuple[float, ...]:
        """The joint velocities now: the motion's constant rate, or zeros once it holds or ends."""
        if run.target_positions is None or run.paused:
            return _still(len(self._positions))
        if run.duration_s <= 0 or self._elapsed(run) >= run.duration_s:
            return _still(len(self._positions))
        return velocity(
            run.start_positions, run.target_positions, run.motion_fraction, run.duration_s
        )

    def _arrive(self, run: _Run) -> None:
        """Put the joints where ``run`` ends and report them, at rest."""
        if run.target_positions is not None:
            self._move_to(
                interpolate(run.start_positions, run.target_positions, run.motion_fraction),
                _still(len(self._positions)),
            )

    def _freeze(self, run: _Run) -> None:
        """Leave the joints where the motion was interrupted and report them, at rest."""
        self._move_to(self._positions_at(run), _still(len(self._positions)))

    def _move_to(self, positions: tuple[float, ...], velocities: tuple[float, ...]) -> None:
        self._positions = positions
        self._velocities = velocities
        self._report_joints()

    def _report_joints(self) -> None:
        self._context().report_joints(self._positions, self._velocities)

    # ------------------------------------------------------------------ control tasks

    def _stop(self) -> TaskResult:
        stopped: list[str] = []
        for run in self._drain_runs():
            stopped.append(run.request.id)
            run.handle.finish(
                TaskResult('cancelled', 'preempted', "Stopped by task 'stop'", {'by': 'stop'})
            )
        self._context().clear_error()
        reason = 'Nothing was running' if not stopped else f'Stopped {len(stopped)} task(s)'
        return TaskResult('succeeded', 'ok', reason, {'stopped': stopped})

    def _pause(self) -> TaskResult:
        if not self._runs:
            return TaskResult('aborted', 'nothing_to_pause', 'No task is running')
        paused: list[str] = []
        for run in self._runs.values():
            if run.paused:
                continue
            remaining = run.completion.remaining_s() if run.completion is not None else 0.0
            self._freeze(run)
            run.elapsed_before_pause_s += self._clock().monotonic() - run.started_monotonic
            self._disarm(run)
            run.paused_remaining_s = remaining
            paused.append(run.request.id)
        if not paused:
            return TaskResult('aborted', 'nothing_to_pause', 'Task is already paused')
        self._context().set_paused(True)
        return TaskResult('succeeded', 'ok', f'Paused {len(paused)} task(s)', {'paused': paused})

    def _resume(self) -> TaskResult:
        resumed: list[str] = []
        for run in self._runs.values():
            if not run.paused or run.paused_remaining_s is None:
                continue
            remaining = run.paused_remaining_s
            run.paused_remaining_s = None
            run.started_monotonic = self._clock().monotonic()
            self._arm(run, remaining)
            resumed.append(run.request.id)
        if not resumed:
            return TaskResult('aborted', 'nothing_to_resume', 'No task is paused')
        self._context().set_paused(False)
        return TaskResult(
            'succeeded', 'ok', f'Resumed {len(resumed)} task(s)', {'resumed': resumed}
        )

    # ------------------------------------------------------------------ context access

    def _context(self) -> BridgeContext:
        if self._ctx is None:
            raise RuntimeError('FakeExecutor.setup() has not been called')
        return self._ctx

    def _clock(self) -> Clock:
        return self._context().clock

    def _scheduler(self) -> Scheduler:
        scheduler = self._context().scheduler
        if scheduler is None:
            raise RuntimeError('FakeExecutor needs ctx.scheduler to run its timers')
        return scheduler


def _positive_float(value: object, default: float) -> float:
    if isinstance(value, (int, float)) and not isinstance(value, bool) and value > 0:
        return float(value)
    return default


def _non_negative_float(value: object, default: float) -> float:
    if isinstance(value, (int, float)) and not isinstance(value, bool) and value >= 0:
        return float(value)
    return default
