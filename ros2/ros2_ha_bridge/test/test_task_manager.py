# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Tests for :mod:`ros2_ha_bridge.task_manager` against a scripted stub executor.

Every payload the manager publishes (state, feedback, result) is encoded with
``protocol.encode_payload`` exactly as the node's link would, so a payload the
encoder rejects fails here, and every payload and ack is validated against the
vendored contract schemas. Time is the ``FakeClock`` and ``FakeScheduler`` from
``fakes.py``. ``post`` is immediate by default; the ``any_post_harness``
fixture runs a test under both the immediate post and the node's real deferred
post (the result of ``handle.finish`` is queued until ``settle()``), which is
how the lifecycle, cancel, stop-and-hold and synchronous-finish paths are
shown to behave the same whatever the post latency.
"""

from __future__ import annotations

import json
import logging
import threading
import uuid
from collections.abc import Callable, Sequence
from dataclasses import dataclass, replace
from datetime import UTC, datetime
from typing import Any

import pytest

from ros2_ha_bridge import protocol
from ros2_ha_bridge.authoriser import (
    DEFAULT_TASK_TIMEOUT_CAP_S,
    Ack,
    Authoriser,
    resolve_request,
)
from ros2_ha_bridge.dedupe_store import DedupeStore
from ros2_ha_bridge.executors.base import (
    BridgeContext,
    TaskExecutor,
    TaskHandle,
    TaskRejected,
    TaskRequest,
    TaskResult,
    TaskSpec,
)
from ros2_ha_bridge.protocol import AckPayload, DecodedCommand, RobotTopics
from ros2_ha_bridge.task_manager import TaskManager, TaskManagerSettings

from .conftest import assert_imports_without
from .fakes import FakeClock, FakeScheduler

ROBOT = 'arm_left'
SESSION = '9f1c7a0e-4a1b-4b2c-8d1e-0f1a2b3c4d5e'
INSTANCE = 'robotpc:4123'
LOGGER = 'ros2_ha_bridge.task_manager'

RESET = TaskSpec('reset', 'Reset to home', 'motion', 60)
WAIT = TaskSpec('wait', 'Wait', 'motion', 10)
MOVE_TO = TaskSpec(
    'move_to',
    'Move to named pose',
    'motion',
    120,
    {'type': 'object', 'properties': {'pose': {'type': 'string'}}, 'required': ['pose']},
)
STOP = TaskSpec('stop', 'Stop', 'control', 5)
STATUS = TaskSpec('status', 'Report status', 'query', 2)
#: Announced above the default timeout cap (600 s), to show the clamp reaches the executor.
SLOW = TaskSpec('slow', 'Slow motion', 'motion', 900)


def new_id() -> str:
    return str(uuid.uuid4())


@dataclass(frozen=True)
class Published:
    """One call of the injected publish function."""

    topic: str
    payload: dict[str, Any]
    qos: int
    retain: bool
    expiry_s: int | None

    @property
    def kind(self) -> str:
        return self.topic.rsplit('/', 1)[1]


class StubExecutor(TaskExecutor):
    """Records every call; ``on_start``, ``on_cancel`` and ``on_hold`` script the behaviour."""

    name = 'stub'
    version = '1.0'

    def __init__(self) -> None:
        self.ctx: BridgeContext | None = None
        self.started: list[TaskRequest] = []
        self.handles: dict[str, TaskHandle] = {}
        self.cancelled: list[str] = []
        self.holds: list[str] = []
        self.on_start: Callable[[TaskRequest, TaskHandle], None] | None = None
        self.on_cancel: Callable[[str], None] | None = None
        self.on_hold: Callable[[str], None] | None = None
        self.health_report: tuple[Any, str] = ('ok', '')
        self.health_error: Exception | None = None

    def setup(self, ctx: BridgeContext) -> None:
        self.ctx = ctx

    def tasks(self) -> Sequence[TaskSpec]:
        return [RESET, WAIT, MOVE_TO, STOP, STATUS, SLOW]

    def start(self, req: TaskRequest, handle: TaskHandle) -> None:
        self.started.append(req)
        self.handles[req.id] = handle
        if self.on_start is not None:
            self.on_start(req, handle)

    def cancel(self, task_id: str) -> None:
        self.cancelled.append(task_id)
        if self.on_cancel is not None:
            self.on_cancel(task_id)

    def stop_and_hold(self, reason: str) -> None:
        self.holds.append(reason)
        if self.on_hold is not None:
            self.on_hold(reason)

    def health(self) -> tuple[Any, str]:
        if self.health_error is not None:
            raise self.health_error
        return self.health_report

    def finish(
        self,
        task_id: str,
        status: Any = 'succeeded',
        code: str = 'ok',
        reason: str = '',
        data: dict[str, Any] | None = None,
    ) -> None:
        self.handles[task_id].finish(TaskResult(status, code, reason, data or {}))

    def feedback(self, task_id: str, progress: float | None = None, stage: str = '') -> None:
        self.handles[task_id].feedback(progress, stage, 'msg')


class Harness:
    """A TaskManager over a StubExecutor with recorded publishes and an optional post queue."""

    def __init__(
        self,
        clock: FakeClock,
        scheduler: FakeScheduler,
        *,
        settings: TaskManagerSettings | None = None,
        deferred_post: bool = False,
        watchdog_mode: Any = 'none',
    ) -> None:
        self.clock = clock
        self.scheduler = scheduler
        self.published: list[Published] = []
        self.publish_ok = True
        self.publish_raises = False
        self.queue: list[Callable[[], None]] = []
        self.deferred = deferred_post
        self.executor = StubExecutor()
        self.manager = TaskManager(
            robot_id=ROBOT,
            executor=self.executor,
            publish=self._publish,
            post=self._post,
            clock=clock,
            scheduler=scheduler,
            session=SESSION,
            instance=INSTANCE,
            watchdog_mode=watchdog_mode,
            settings=settings,
        )
        self.executor.setup(self.manager.bridge_context(ha_link_ok=lambda: True))

    def _publish(
        self, topic: str, payload: Any, qos: int, retain: bool, expiry_s: int | None
    ) -> bool:
        if self.publish_raises:
            raise OSError('socket closed')
        # Encode exactly as the node's link would: a payload the encoder rejects (a datetime,
        # NaN, a lone surrogate) raises here instead of being recorded as if it had gone out.
        wire = protocol.encode_payload(payload)
        self.published.append(Published(topic, json.loads(wire), qos, retain, expiry_s))
        return self.publish_ok

    def _post(self, fn: Callable[[], None]) -> None:
        if self.deferred:
            self.queue.append(fn)
        else:
            fn()

    def drain(self) -> int:
        count = 0
        while self.queue:
            self.queue.pop(0)()
            count += 1
        return count

    def settle(self) -> int:
        """Let the main thread run what was posted (a no-op under the immediate post)."""
        return self.drain()

    def of(self, kind: str) -> list[Published]:
        return [p for p in self.published if p.kind == kind]

    @property
    def states(self) -> list[Published]:
        return self.of('state')

    @property
    def results(self) -> list[Published]:
        return self.of('result')

    @property
    def feedbacks(self) -> list[Published]:
        return self.of('feedback')

    def task_command(
        self,
        spec: TaskSpec,
        params: dict[str, Any] | None = None,
        *,
        timeout_s: float | None = None,
        command_id: str | None = None,
    ) -> DecodedCommand:
        return decoded(
            {
                'schema': 1,
                'id': command_id or new_id(),
                'type': 'task',
                'robot_id': ROBOT,
                'task': spec.name,
                'params': params or {},
                'issued_at': self.clock.now_ms(),
                'max_age_ms': 5000,
                'timeout_s': timeout_s,
                'source': {'origin': 'ha', 'kind': 'test'},
            }
        )

    def cancel_command(
        self, target_id: str | None = None, *, command_id: str | None = None
    ) -> DecodedCommand:
        return decoded(
            {
                'schema': 1,
                'id': command_id or new_id(),
                'type': 'cancel',
                'robot_id': ROBOT,
                'target_id': target_id,
                'issued_at': self.clock.now_ms(),
                'max_age_ms': 5000,
                'source': {'origin': 'ha', 'kind': 'test'},
            }
        )

    def start(
        self,
        spec: TaskSpec,
        params: dict[str, Any] | None = None,
        *,
        timeout_s: float | None = None,
    ) -> tuple[DecodedCommand, AckPayload]:
        """Resolve the command the way the authoriser does and start it."""
        command = self.task_command(spec, params, timeout_s=timeout_s)
        return command, self.manager.start(resolve_request(command, spec), spec)

    def authoriser(self, *tasks: TaskSpec) -> Authoriser:
        """An authoriser wired to this manager's state view, as the node wires it."""
        return Authoriser(
            topics=RobotTopics(ROBOT),
            session=SESSION,
            clock=self.clock,
            dedupe=DedupeStore(clock=self.clock),
            tasks=tasks or (RESET, WAIT, MOVE_TO, STOP, STATUS, SLOW),
            state_view=self.manager.gating_state,
        )


def decoded(payload: dict[str, Any]) -> DecodedCommand:
    result = protocol.decode_command(protocol.encode_payload(payload), expected_robot_id=ROBOT)
    assert isinstance(result, DecodedCommand), result
    return result


def authorised(authoriser: Authoriser, command: DecodedCommand) -> Ack:
    ack = authoriser.handle(protocol.encode_payload(command.payload), topic=RobotTopics(ROBOT).cmd)
    assert isinstance(ack, Ack), ack
    return ack


@pytest.fixture
def harness(clock: FakeClock, scheduler: FakeScheduler) -> Harness:
    return Harness(clock, scheduler)


@pytest.fixture(params=[False, True], ids=['immediate_post', 'deferred_post'])
def any_post_harness(
    request: pytest.FixtureRequest, clock: FakeClock, scheduler: FakeScheduler
) -> Harness:
    """The same test under the immediate post and the node's deferred post."""
    return Harness(clock, scheduler, deferred_post=bool(request.param))


def exception_records(caplog: pytest.LogCaptureFixture) -> list[logging.LogRecord]:
    return [r for r in caplog.records if r.exc_info]


def late_finish_warnings(caplog: pytest.LogCaptureFixture) -> list[str]:
    """WARNING lines about a finish that arrived after the task had been ended already."""
    return [
        r.getMessage()
        for r in caplog.records
        if r.levelno >= logging.WARNING and 'already finished' in r.getMessage()
    ]


# --------------------------------------------------------------------------- hygiene


def test_module_imports_neither_rclpy_nor_paho() -> None:
    assert_imports_without('ros2_ha_bridge.task_manager', {'rclpy', 'paho'})


# --------------------------------------------------------------------------- happy path


def test_full_lifecycle_happy_path(any_post_harness: Harness, schema_validator: Any) -> None:
    harness = any_post_harness
    clock, manager, executor = harness.clock, harness.manager, harness.executor
    t0 = clock.now_ms()
    assert manager.state == 'idle' and manager.seq == 0

    command, ack = harness.start(MOVE_TO, {'pose': 'home'})
    assert ack.code == 'accepted' and ack.accepted
    assert ack.type == 'task' and ack.task == 'move_to' and ack.id == command.id
    assert ack.received_at == t0 and ack.clock_skew_ms == 0 and ack.session == SESSION
    schema_validator.assert_valid('ack', ack.to_payload())

    request = executor.started[0]
    assert request.id == command.id and request.task == 'move_to'
    assert request.params == {'pose': 'home'} and request.timeout_s == 120.0
    assert request.source == {'origin': 'ha', 'kind': 'test'}

    assert manager.state == 'busy' and manager.active_task_id == command.id
    assert [p.kind for p in harness.published] == ['state']
    busy = harness.states[0]
    assert (busy.topic, busy.qos, busy.retain, busy.expiry_s) == (
        manager.topics.state,
        1,
        True,
        None,
    )
    schema_validator.assert_valid('state', busy.payload)
    assert busy.payload['state'] == 'busy'
    assert busy.payload['task_id'] == command.id and busy.payload['task'] == 'move_to'
    assert busy.payload['seq'] == 1 and busy.payload['since'] == t0
    assert busy.payload['paused'] is False and busy.payload['error'] is None
    assert busy.payload['session'] == SESSION and busy.payload['instance'] == INSTANCE

    clock.advance(0.7)
    executor.feedback(command.id, 0.4, 'executing')
    harness.settle()
    feedback = harness.feedbacks[0]
    assert (feedback.qos, feedback.retain, feedback.expiry_s) == (0, False, 10)
    assert feedback.topic == manager.topics.feedback(command.id)
    schema_validator.assert_valid('feedback', feedback.payload)
    assert feedback.payload['progress'] == 0.4 and feedback.payload['stage'] == 'executing'

    clock.advance(1.3)
    executor.finish(command.id, 'succeeded', 'ok', 'At home', {'pose': 'home'})
    harness.settle()
    assert [p.kind for p in harness.published] == ['state', 'feedback', 'result', 'state']
    result = harness.results[0]
    assert (result.qos, result.retain, result.expiry_s) == (1, True, 300)
    assert result.topic == manager.topics.result(command.id)
    schema_validator.assert_valid('result', result.payload)
    assert result.payload['kind'] == 'motion' and result.payload['task'] == 'move_to'
    assert result.payload['status'] == 'succeeded' and result.payload['goal_status'] == 4
    assert result.payload['code'] == 'ok' and result.payload['data'] == {'pose': 'home'}
    assert result.payload['started_at'] == t0
    assert result.payload['finished_at'] == t0 + 2000 == result.payload['ts']

    idle = harness.states[-1]
    schema_validator.assert_valid('state', idle.payload)
    assert idle.payload['state'] == 'idle' and idle.payload['task_id'] is None
    assert idle.payload['task'] is None and idle.payload['seq'] == 2
    assert idle.payload['since'] == t0 + 2000
    assert manager.state == 'idle' and manager.active_task_ids == ()
    assert harness.scheduler.pending == []


def test_received_at_feeds_the_ack(harness: Harness) -> None:
    command = harness.task_command(RESET)
    request = resolve_request(command, RESET)
    ack = harness.manager.start(request, RESET, received_at=command.issued_at + 12)
    assert ack.received_at == command.issued_at + 12
    assert ack.clock_skew_ms == 12
    assert ack.ts == harness.clock.now_ms()


def test_start_refuses_a_mismatched_request(harness: Harness) -> None:
    request = resolve_request(harness.task_command(RESET), RESET)
    with pytest.raises(ValueError):
        harness.manager.start(request, WAIT)
    with pytest.raises(ValueError):
        harness.manager.start(replace(request, timeout_s=0.0), RESET)
    with pytest.raises(ValueError):
        resolve_request(harness.cancel_command(), RESET)
    with pytest.raises(ValueError):
        resolve_request(harness.task_command(RESET), WAIT)
    with pytest.raises(ValueError):
        harness.manager.cancel(harness.task_command(RESET))
    assert harness.executor.started == [] and harness.published == []


# --------------------------------------------------------------------------- cancel


def test_cancel_is_ack_terminal_and_the_result_follows_under_the_task_id(
    any_post_harness: Harness, schema_validator: Any
) -> None:
    harness = any_post_harness
    executor, manager = harness.executor, harness.manager
    executor.on_cancel = lambda task_id: executor.finish(
        task_id, 'cancelled', 'preempted', 'Stopped'
    )
    command, _ = harness.start(WAIT)

    cancel = harness.cancel_command(None)
    ack = manager.cancel(cancel)
    assert ack.code == 'accepted' and ack.type == 'cancel' and ack.task is None
    assert ack.id == cancel.id
    schema_validator.assert_valid('ack', ack.to_payload())
    assert executor.cancelled == [command.id]
    harness.settle()

    assert len(harness.results) == 1
    result = harness.results[0]
    assert result.topic == manager.topics.result(command.id)
    assert result.payload['id'] == command.id
    assert result.payload['status'] == 'cancelled' and result.payload['goal_status'] == 5
    assert result.payload['code'] == 'preempted'
    assert not [p for p in harness.published if cancel.id in p.topic], 'a cancel has no result'
    assert manager.state == 'idle' and harness.states[-1].payload['state'] == 'idle'
    assert harness.scheduler.pending == [], 'no grace timer outlives a finish the executor made'


def test_cancel_by_explicit_target_id(harness: Harness) -> None:
    command, _ = harness.start(WAIT)
    ack = harness.manager.cancel(harness.cancel_command(command.id))
    assert ack.code == 'accepted'
    assert harness.executor.cancelled == [command.id]


def test_cancel_with_nothing_running_is_rejected_no_such_task(
    harness: Harness, schema_validator: Any
) -> None:
    ack = harness.manager.cancel(harness.cancel_command(None))
    assert ack.code == 'rejected_no_such_task' and not ack.accepted
    assert 'No motion task is running' in ack.reason
    schema_validator.assert_valid('ack', ack.to_payload())
    other = new_id()
    ack = harness.manager.cancel(harness.cancel_command(other))
    assert ack.code == 'rejected_no_such_task' and other in ack.reason
    assert harness.executor.cancelled == []
    assert harness.published == []


def test_cancel_of_a_finished_task_is_rejected_no_such_task(harness: Harness) -> None:
    command, _ = harness.start(WAIT)
    harness.executor.finish(command.id)
    ack = harness.manager.cancel(harness.cancel_command(command.id))
    assert ack.code == 'rejected_no_such_task'


def test_cancel_targets_a_control_task_by_id(harness: Harness) -> None:
    command, _ = harness.start(STOP)
    ack = harness.manager.cancel(harness.cancel_command(command.id))
    assert ack.code == 'accepted'
    assert harness.executor.cancelled == [command.id]


# --------------------------------------------------------------------------- kinds


def test_control_and_query_results_carry_kind_and_leave_the_motion_slot(
    harness: Harness, schema_validator: Any
) -> None:
    manager, executor = harness.manager, harness.executor
    motion, _ = harness.start(WAIT)
    assert len(harness.states) == 1

    control, ack = harness.start(STOP)
    assert ack.code == 'accepted'
    query, ack = harness.start(STATUS)
    assert ack.code == 'accepted'
    assert manager.active_task_id == motion.id
    assert manager.active_task_ids == (motion.id, control.id, query.id)
    assert len(harness.states) == 1, 'control and query tasks do not change the state'

    executor.finish(control.id, 'succeeded', 'ok')
    executor.finish(query.id, 'succeeded', 'ok', data={'joints': 6})
    assert [p.payload['kind'] for p in harness.results] == ['control', 'query']
    for result in harness.results:
        schema_validator.assert_valid('result', result.payload)
    assert len(harness.states) == 1
    assert manager.state == 'busy' and manager.active_task_id == motion.id

    executor.finish(motion.id)
    assert harness.results[-1].payload['kind'] == 'motion'
    assert manager.state == 'idle' and len(harness.states) == 2


# --------------------------------------------------------------------------- timeout


def test_timeout_then_forced_abort_puts_the_robot_in_error(
    harness: Harness, schema_validator: Any, caplog: pytest.LogCaptureFixture
) -> None:
    caplog.set_level(logging.INFO, logger=LOGGER)
    clock, manager, executor = harness.clock, harness.manager, harness.executor
    command, _ = harness.start(WAIT)  # announced timeout 10 s

    clock.advance(9.99)
    assert executor.cancelled == []
    clock.advance(0.01)
    assert executor.cancelled == [command.id], 'the timeout asks the executor to cancel'
    assert manager.state == 'busy' and harness.results == []

    clock.advance(4.99)
    assert harness.results == []
    clock.advance(0.01)
    assert len(harness.results) == 1
    result = harness.results[0].payload
    schema_validator.assert_valid('result', result)
    assert result['status'] == 'aborted' and result['goal_status'] == 6
    assert result['code'] == 'timeout' and result['kind'] == 'motion'
    assert result['reason'] == "Task 'wait' did not finish within 10 s and ignored cancel"

    state = harness.states[-1].payload
    schema_validator.assert_valid('state', state)
    assert state['state'] == 'error' and state['task_id'] is None
    assert state['error'] == {'code': 'timeout', 'reason': result['reason']}
    assert manager.motion_rejection() == (
        'rejected_error',
        f'Robot is in error (timeout): {result["reason"]}',
    )

    _, ack = harness.start(RESET)
    assert ack.code == 'rejected_error'
    stop, ack = harness.start(STOP)
    assert ack.code == 'accepted', 'control tasks pass while in error'
    executor.finish(stop.id)

    manager.clear_error()
    assert manager.state == 'idle' and harness.states[-1].payload['state'] == 'idle'
    assert harness.states[-1].payload['error'] is None
    assert harness.scheduler.pending == [], 'no timer outlives the task'


def test_cancel_raising_on_timeout_is_executor_unresponsive(
    harness: Harness, caplog: pytest.LogCaptureFixture
) -> None:
    caplog.set_level(logging.INFO, logger=LOGGER)

    def explode(task_id: str) -> None:
        raise RuntimeError('driver gone')

    harness.executor.on_cancel = explode
    command, _ = harness.start(WAIT)
    harness.clock.advance(10)
    assert harness.executor.cancelled == [command.id]
    assert len(exception_records(caplog)) == 1
    harness.clock.advance(5)
    result = harness.results[0].payload
    assert result['status'] == 'aborted' and result['code'] == 'executor_unresponsive'
    assert 'cancel() raised' in result['reason']
    assert harness.manager.state == 'error'
    assert harness.manager.error is not None
    assert harness.manager.error.code == 'executor_unresponsive'


def test_ignored_cancel_command_is_forced_after_the_grace(
    harness: Harness, schema_validator: Any
) -> None:
    command, _ = harness.start(WAIT)
    harness.clock.advance(1)
    ack = harness.manager.cancel(harness.cancel_command(None))
    assert ack.code == 'accepted'
    harness.clock.advance(4.99)
    assert harness.results == []
    harness.clock.advance(0.01)
    result = harness.results[0].payload
    schema_validator.assert_valid('result', result)
    assert result['id'] == command.id
    assert result['status'] == 'aborted' and result['code'] == 'executor_unresponsive'
    assert result['reason'] == "Task 'wait' ignored cancel for 5 s"
    assert harness.manager.state == 'error'
    assert harness.manager.active_task_ids == ()


def test_executor_finishing_within_the_grace_wins(harness: Harness) -> None:
    command, _ = harness.start(WAIT)
    harness.clock.advance(10)
    harness.clock.advance(3)
    harness.executor.finish(command.id, 'cancelled', 'cancelled', 'Stopped late')
    assert harness.results[0].payload['status'] == 'cancelled'
    assert harness.manager.state == 'idle' and harness.manager.error is None
    harness.clock.advance(10)
    assert len(harness.results) == 1, 'the grace timer was cancelled by the finish'


def test_command_timeout_is_clamped_and_overrides_the_announced_value(harness: Harness) -> None:
    """Step 8 lives in ``resolve_request``; the manager arms its deadline from the request."""
    command = harness.task_command(WAIT)
    assert resolve_request(command, WAIT).timeout_s == 10.0
    assert resolve_request(harness.task_command(WAIT, timeout_s=3.5), WAIT).timeout_s == 3.5
    capped = resolve_request(harness.task_command(WAIT, timeout_s=10_000), WAIT)
    assert capped.timeout_s == DEFAULT_TASK_TIMEOUT_CAP_S == 600.0
    assert resolve_request(harness.task_command(SLOW), SLOW).timeout_s == 600.0
    assert (
        resolve_request(harness.task_command(SLOW), SLOW, task_timeout_cap_s=50).timeout_s == 50.0
    )
    command, _ = harness.start(WAIT, timeout_s=2)
    assert harness.executor.started[0].timeout_s == 2.0
    harness.clock.advance(2)
    assert harness.executor.cancelled == [command.id]


def test_executor_receives_the_authorisers_request_and_the_deadline_matches_it(
    harness: Harness,
) -> None:
    """The announced 900 s of ``slow`` is clamped to the 600 s cap everywhere HA could see it."""
    authoriser = harness.authoriser()
    assert authoriser.tasks['slow'].timeout_s == 600.0, 'the announced catalogue is clamped'
    assert [spec.timeout_s for spec in authoriser.catalogue] == [60, 10, 120, 5, 2, 600.0]

    for requested, expected in ((None, 600.0), (1000, 600.0), (30, 30.0)):
        command = harness.task_command(SLOW, timeout_s=requested)
        ack = authorised(authoriser, command)
        assert ack.authorised and ack.request is not None and ack.spec is not None
        assert ack.request.timeout_s == expected
        reply = harness.manager.start(ack.request, ack.spec)
        assert reply.code == 'accepted'
        assert harness.executor.started[-1] == ack.request, 'the executor sees the same request'
        assert harness.executor.started[-1] is ack.request
        assert harness.scheduler.pending[0].remaining_s() == pytest.approx(expected)
        authoriser.commit(ack)
        harness.executor.finish(command.id)
        assert harness.scheduler.pending == []


def test_settings_change_the_grace(clock: FakeClock, scheduler: FakeScheduler) -> None:
    harness = Harness(clock, scheduler, settings=TaskManagerSettings(cancel_grace_s=1.5))
    harness.start(WAIT)
    clock.advance(10)
    clock.advance(1.49)
    assert harness.results == []
    clock.advance(0.01)
    assert harness.results[0].payload['code'] == 'timeout'


# --------------------------------------------------------------------------- pause


def test_pause_stores_the_remaining_time_and_resume_rearms_it_exactly(
    harness: Harness, schema_validator: Any
) -> None:
    clock, manager, executor, scheduler = (
        harness.clock,
        harness.manager,
        harness.executor,
        harness.scheduler,
    )
    command, _ = harness.start(WAIT)
    clock.advance(4)
    assert scheduler.pending[0].remaining_s() == pytest.approx(6.0)

    manager.set_paused(True)
    assert manager.paused and manager.state == 'busy'
    paused = harness.states[-1].payload
    schema_validator.assert_valid('state', paused)
    assert paused['state'] == 'busy' and paused['paused'] is True
    assert paused['task_id'] == command.id
    assert scheduler.pending == [], 'the deadline timer is cancelled while paused'

    clock.advance(100)
    assert executor.cancelled == [] and harness.results == []
    manager.set_paused(True)
    assert len(harness.states) == 2, 'pausing twice changes nothing'

    manager.set_paused(False)
    assert not manager.paused and harness.states[-1].payload['paused'] is False
    assert len(scheduler.pending) == 1
    assert scheduler.pending[0].remaining_s() == pytest.approx(6.0)
    clock.advance(5.99)
    assert executor.cancelled == []
    clock.advance(0.01)
    assert executor.cancelled == [command.id]


def test_pause_without_a_motion_task_is_ignored_and_cleared_on_finish(
    harness: Harness, caplog: pytest.LogCaptureFixture
) -> None:
    caplog.set_level(logging.WARNING, logger=LOGGER)
    manager = harness.manager
    manager.set_paused(True)
    assert not manager.paused and harness.published == []
    assert any('no active motion task' in r.getMessage() for r in caplog.records)

    command, _ = harness.start(WAIT)
    manager.set_paused(True)
    assert manager.paused
    harness.executor.finish(command.id, 'cancelled', 'preempted')
    assert not manager.paused
    assert harness.states[-1].payload['paused'] is False
    assert harness.states[-1].payload['state'] == 'idle'
    manager.set_paused(False)
    assert len(harness.states) == 3, 'resuming when not paused changes nothing'


# --------------------------------------------------------------------------- finish rules


def test_second_finish_is_ignored_with_a_log_line(
    harness: Harness, caplog: pytest.LogCaptureFixture
) -> None:
    caplog.set_level(logging.WARNING, logger=LOGGER)
    command, _ = harness.start(RESET)
    harness.executor.finish(command.id)
    harness.executor.finish(command.id, 'aborted', 'collision')
    harness.executor.feedback(command.id, 0.5)
    assert len(harness.results) == 1
    assert harness.results[0].payload['status'] == 'succeeded'
    assert harness.feedbacks == []
    messages = [r.getMessage() for r in caplog.records]
    assert any('called after the task already finished' in m for m in messages)


def test_finish_after_a_forced_finish_is_ignored(
    harness: Harness, caplog: pytest.LogCaptureFixture
) -> None:
    caplog.set_level(logging.WARNING, logger=LOGGER)
    command, _ = harness.start(WAIT)
    harness.clock.advance(15)
    assert harness.results[0].payload['code'] == 'timeout'
    harness.executor.finish(command.id, 'cancelled', 'cancelled')
    assert len(harness.results) == 1
    assert any('already finished' in r.getMessage() for r in caplog.records)


def test_synchronous_finish_inside_start(any_post_harness: Harness, schema_validator: Any) -> None:
    harness = any_post_harness
    harness.executor.on_start = lambda req, handle: handle.finish(TaskResult('succeeded', 'ok'))
    command, ack = harness.start(RESET)
    assert ack.code == 'accepted'
    assert harness.scheduler.pending == [], 'no deadline is armed for a task already finished'
    harness.settle()
    assert len(harness.results) == 1
    assert harness.results[0].payload['id'] == command.id
    schema_validator.assert_valid('result', harness.results[0].payload)
    assert harness.manager.state == 'idle' and harness.scheduler.pending == []
    # Immediate post: the result is the only publication. Deferred post: start() returns with
    # the finish still queued, so a busy state goes out first and idle follows the result.
    kinds = [p.kind for p in harness.published]
    assert kinds in (['result'], ['state', 'result', 'state'])
    if harness.states:
        assert harness.states[-1].payload['state'] == 'idle'


@pytest.mark.parametrize(
    'raised',
    [TaskRejected(reason='changed my mind'), RuntimeError('tripped after finishing')],
    ids=['task_rejected', 'exception'],
)
def test_error_raised_after_a_synchronous_finish_leaves_the_result_standing(
    any_post_harness: Harness, caplog: pytest.LogCaptureFixture, raised: Exception
) -> None:
    """The handle's flag, not the applied record, decides: the finish may still be queued."""
    harness = any_post_harness
    caplog.set_level(logging.INFO, logger=LOGGER)

    def finish_then_raise(req: TaskRequest, handle: TaskHandle) -> None:
        handle.finish(TaskResult('succeeded', 'ok', 'done before the raise'))
        raise raised

    harness.executor.on_start = finish_then_raise
    command, ack = harness.start(RESET)
    assert ack.code == 'accepted', 'a finish already made is never turned into a rejection'
    assert harness.scheduler.pending == []
    harness.settle()
    assert [r.payload['id'] for r in harness.results] == [command.id]
    assert harness.results[0].payload['status'] == 'succeeded'
    assert harness.manager.state == 'idle' and harness.manager.active_task_ids == ()
    assert any('the result stands' in r.getMessage() for r in caplog.records)
    assert late_finish_warnings(caplog) == []


def test_sloppy_result_is_sanitised(harness: Harness, schema_validator: Any) -> None:
    command, _ = harness.start(RESET)
    sloppy = TaskResult('done', 'Bad Code!', 7, [1, 2])  # type: ignore[arg-type]
    harness.executor.handles[command.id].finish(sloppy)
    result = harness.results[0].payload
    schema_validator.assert_valid('result', result)
    assert result['status'] == 'aborted' and result['code'] == 'executor_error'
    assert result['reason'] == '7' and result['data'] == {'value': '[1, 2]'}
    command, _ = harness.start(RESET)
    harness.executor.finish(command.id, 'aborted', 'No Trajectory Found')
    assert harness.results[1].payload['code'] == 'no_trajectory_found'


@pytest.mark.parametrize(
    'data',
    [
        {'when': datetime(2026, 10, 13, 12, 0, tzinfo=UTC)},
        {'raw': b'\x00\x01'},
        {'tags': {'a', 'b'}},
        {'ratio': float('nan')},
        {('tuple', 'key'): 1},
        {'nested': {'deep': {'bad': object()}}},
    ],
    ids=['datetime', 'bytes', 'set', 'nan', 'tuple_key', 'nested_object'],
)
def test_unserialisable_result_data_is_replaced_and_the_result_still_goes_out(
    harness: Harness, schema_validator: Any, caplog: pytest.LogCaptureFixture, data: Any
) -> None:
    """The one result the contract promises must encode first time; there is no retry."""
    caplog.set_level(logging.WARNING, logger=LOGGER)
    command, _ = harness.start(MOVE_TO, {'pose': 'home'})
    harness.executor.finish(command.id, 'succeeded', 'ok', 'Moved', data)
    assert len(harness.results) == 1, 'the result reached the wire'
    result = harness.results[0].payload
    schema_validator.assert_valid('result', result)
    assert result['status'] == 'succeeded' and result['code'] == 'ok'
    assert list(result['data']) == ['unserialisable']
    assert result['data']['unserialisable'] == repr(data)[:512]
    assert harness.manager.state == 'idle' and harness.states[-1].payload['state'] == 'idle'
    assert any('not JSON-encodable' in r.getMessage() for r in caplog.records)


def test_oversize_result_data_is_replaced_so_the_result_fits_the_wire_limit(
    harness: Harness, schema_validator: Any, caplog: pytest.LogCaptureFixture
) -> None:
    """A verbose plugin must not push the one result over the 16 KiB message limit."""
    caplog.set_level(logging.WARNING, logger=LOGGER)
    command, _ = harness.start(MOVE_TO, {'pose': 'home'})
    data = {'trajectory': 'x' * 20_000}
    harness.executor.finish(command.id, 'succeeded', 'ok', 'Moved', data)
    assert len(harness.results) == 1, 'the result reached the wire'
    result = harness.results[0].payload
    schema_validator.assert_valid('result', result)
    assert result['data'] == {'truncated': True, 'bytes': len(json.dumps(data).encode())}
    assert len(json.dumps(result).encode('utf-8')) < 16384
    assert any('over 8192' in r.getMessage() for r in caplog.records)


def test_encodable_result_data_passes_through_untouched(harness: Harness) -> None:
    data = {
        'moveit_error_code': -12,
        'contacts': [{'link': 'wrist_3'}],
        'note': None,
        'k': 'Küche',
    }
    command, _ = harness.start(MOVE_TO, {'pose': 'home'})
    harness.executor.finish(command.id, 'succeeded', 'ok', 'Moved', dict(data))
    assert harness.results[0].payload['data'] == data


def test_lone_surrogate_in_a_reason_is_replaced_not_fatal(
    harness: Harness, schema_validator: Any
) -> None:
    command, _ = harness.start(RESET)
    harness.executor.finish(command.id, 'aborted', 'collision', 'bad byte \udcff here')
    result = harness.results[0].payload
    schema_validator.assert_valid('result', result)
    assert result['reason'] == 'bad byte � here'
    command, _ = harness.start(RESET)
    harness.executor.handles[command.id].feedback(0.5, 'stage \udcff', 'msg')
    assert harness.feedbacks[0].payload['stage'] == 'stage �'
    harness.executor.finish(command.id)


# --------------------------------------------------------------------------- feedback


def test_feedback_is_throttled_per_task(harness: Harness, schema_validator: Any) -> None:
    executor, clock = harness.executor, harness.clock
    motion, _ = harness.start(WAIT)
    for step in range(10):
        executor.feedback(motion.id, step / 10)
        clock.advance(0.1)
    assert [f.payload['progress'] for f in harness.feedbacks] == [0.0, 0.5]
    control, _ = harness.start(STATUS)
    executor.feedback(control.id, 0.9)
    assert harness.feedbacks[-1].payload['id'] == control.id, 'each task has its own window'
    executor.feedback(motion.id, 1.0)
    assert [f.payload['progress'] for f in harness.feedbacks] == [0.0, 0.5, 0.9, 1.0]
    for feedback in harness.feedbacks:
        schema_validator.assert_valid('feedback', feedback.payload)


def test_feedback_throttle_can_be_disabled(clock: FakeClock, scheduler: FakeScheduler) -> None:
    harness = Harness(clock, scheduler, settings=TaskManagerSettings(feedback_max_rate_hz=0))
    command, _ = harness.start(WAIT)
    for _ in range(5):
        harness.executor.feedback(command.id, 0.1)
    assert len(harness.feedbacks) == 5


def test_feedback_progress_is_clamped_to_the_schema(
    harness: Harness, schema_validator: Any
) -> None:
    command, _ = harness.start(WAIT)
    harness.executor.handles[command.id].feedback(1.7, 'a', 'b')
    harness.clock.advance(1)
    harness.executor.handles[command.id].feedback(-2)
    harness.clock.advance(1)
    harness.executor.handles[command.id].feedback('half')  # type: ignore[arg-type]
    assert [f.payload['progress'] for f in harness.feedbacks] == [1.0, 0.0, None]
    for feedback in harness.feedbacks:
        schema_validator.assert_valid('feedback', feedback.payload)


# --------------------------------------------------------------------------- health


def test_health_error_surfaces_in_state(harness: Harness, schema_validator: Any) -> None:
    manager, executor = harness.manager, harness.executor
    assert manager.poll_health() == protocol.ExecutorHealth('ok', '')
    assert harness.states == [], 'an unchanged ok health publishes nothing'

    executor.health_report = ('error', 'driver lost')
    health = manager.poll_health()
    assert health.level == 'error' and health.message == 'driver lost'
    state = harness.states[-1].payload
    schema_validator.assert_valid('state', state)
    assert state['executor_health'] == {'level': 'error', 'message': 'driver lost'}
    assert state['state'] == 'idle', 'executor health does not set the robot error'
    manager.poll_health()
    assert len(harness.states) == 1, 'unchanged health is not republished'

    executor.health_report = ('ok', '')
    manager.poll_health()
    assert harness.states[-1].payload['executor_health'] == {'level': 'ok', 'message': ''}


def test_health_raising_or_unknown_is_recorded_as_error(
    harness: Harness, caplog: pytest.LogCaptureFixture
) -> None:
    caplog.set_level(logging.WARNING, logger=LOGGER)
    harness.executor.health_error = RuntimeError('boom')
    health = harness.manager.poll_health()
    assert health.level == 'error' and 'RuntimeError: boom' in health.message
    harness.manager.poll_health()
    assert len(exception_records(caplog)) == 1, 'one traceback per distinct message'
    harness.executor.health_error = None
    harness.executor.health_report = ('fine', 'x')
    health = harness.manager.poll_health()
    assert health.level == 'error' and "unknown level 'fine'" in health.message


def test_health_polling_runs_on_the_scheduler(clock: FakeClock, scheduler: FakeScheduler) -> None:
    harness = Harness(clock, scheduler, settings=TaskManagerSettings(health_poll_s=0.25))
    harness.manager.start_health_polling()
    harness.manager.start_health_polling()
    assert len(scheduler.pending) == 1
    harness.executor.health_report = ('degraded', 'slow')
    clock.advance(0.24)
    assert harness.states == []
    clock.advance(0.01)
    assert harness.states[-1].payload['executor_health']['level'] == 'degraded'
    harness.executor.health_report = ('ok', '')
    clock.advance(0.25)
    assert harness.states[-1].payload['executor_health']['level'] == 'ok'
    harness.manager.stop_health_polling()
    assert scheduler.pending == []
    harness.executor.health_report = ('error', 'x')
    clock.advance(1)
    assert harness.states[-1].payload['executor_health']['level'] == 'ok'


def test_slow_executor_call_marks_the_executor_degraded(harness: Harness) -> None:
    clock = harness.clock

    def slow(req: TaskRequest, handle: TaskHandle) -> None:
        clock.set_monotonic(clock.monotonic() + 0.35)

    harness.executor.on_start = slow
    _, ack = harness.start(RESET)
    assert ack.code == 'accepted'
    state = harness.states[-1].payload
    assert state['executor_health'] == {'level': 'degraded', 'message': 'start() took 0.35 s'}
    harness.manager.poll_health()
    assert harness.states[-1].payload['executor_health']['level'] == 'ok'


# --------------------------------------------------------------------------- rejections


def test_task_rejected_from_start_becomes_rejected_executor(
    harness: Harness, schema_validator: Any, caplog: pytest.LogCaptureFixture
) -> None:
    caplog.set_level(logging.INFO, logger=LOGGER)

    def refuse(req: TaskRequest, handle: TaskHandle) -> None:
        raise TaskRejected(reason='gripper not homed')

    harness.executor.on_start = refuse
    _, ack = harness.start(RESET)
    assert ack.code == 'rejected_executor' and ack.reason == 'gripper not homed'
    schema_validator.assert_valid('ack', ack.to_payload())
    assert harness.manager.state == 'idle' and harness.manager.active_task_ids == ()
    assert harness.published == [] and harness.scheduler.pending == []
    assert exception_records(caplog) == []


def test_exception_from_start_becomes_rejected_executor_with_one_traceback(
    harness: Harness, caplog: pytest.LogCaptureFixture
) -> None:
    caplog.set_level(logging.INFO, logger=LOGGER)

    def explode(req: TaskRequest, handle: TaskHandle) -> None:
        raise RuntimeError('no action server')

    harness.executor.on_start = explode
    _, ack = harness.start(RESET)
    assert ack.code == 'rejected_executor'
    assert ack.reason == 'executor.start raised RuntimeError: no action server'
    assert len(exception_records(caplog)) == 1
    assert harness.manager.state == 'idle' and harness.published == []
    _, ack = harness.start(STOP)
    assert ack.code == 'rejected_executor', 'control tasks are refused the same way'


def test_motion_gating_busy_error_inhibited(harness: Harness, schema_validator: Any) -> None:
    manager = harness.manager
    assert manager.motion_rejection() is None
    running, _ = harness.start(RESET)
    assert manager.motion_rejection() == (
        'rejected_busy',
        f"Task 'reset' (id {running.id}) is running",
    )
    _, ack = harness.start(MOVE_TO, {'pose': 'rest'})
    assert ack.code == 'rejected_busy' and running.id in ack.reason
    schema_validator.assert_valid('ack', ack.to_payload())
    assert len(harness.executor.started) == 1
    _, ack = harness.start(STOP)
    assert ack.code == 'accepted', 'control passes while busy'

    harness.executor.finish(running.id)
    manager.set_error('controller_error', 'E-stop')
    assert manager.state == 'error'
    _, ack = harness.start(RESET)
    assert ack.code == 'rejected_error' and 'E-stop' in ack.reason
    manager.clear_error()

    manager.set_inhibited(True)
    _, ack = harness.start(RESET)
    assert ack.code == 'rejected_inhibited'
    _, ack = harness.start(STATUS)
    assert ack.code == 'accepted', 'query passes while inhibited'
    manager.set_inhibited(False)
    assert manager.motion_rejection() is None


def test_duplicate_running_id_is_refused(harness: Harness) -> None:
    command, _ = harness.start(WAIT)
    again = harness.task_command(STOP, command_id=command.id)
    ack = harness.manager.start(resolve_request(again, STOP), STOP)
    assert ack.code == 'rejected_executor'
    assert len(harness.executor.started) == 1


def test_busy_and_error_is_rejected_busy_through_both_gates(
    harness: Harness, schema_validator: Any
) -> None:
    """One gate (``gate_motion``): the authoriser's answer equals the manager's in every state."""
    manager = harness.manager
    authoriser = harness.authoriser()

    def through_authoriser() -> tuple[str, str]:
        ack = authorised(authoriser, harness.task_command(RESET))
        schema_validator.assert_valid('ack', ack.payload)
        return ack.code, str(ack.payload['reason'])

    def through_manager() -> tuple[str, str]:
        reply = harness.manager.start(resolve_request(harness.task_command(RESET), RESET), RESET)
        return reply.code, reply.reason

    running, _ = harness.start(WAIT)
    manager.set_error('controller_error', 'E-stop')
    assert manager.state == 'error' and manager.active_task_id == running.id
    expected = ('rejected_busy', f"Task 'wait' (id {running.id}) is running")
    assert manager.motion_rejection() == expected
    assert through_authoriser() == expected
    assert through_manager() == expected

    harness.executor.finish(running.id)
    expected = ('rejected_error', 'Robot is in error (controller_error): E-stop')
    assert manager.motion_rejection() == expected
    assert through_authoriser() == expected
    assert through_manager() == expected

    manager.set_error('bare', '')
    assert manager.motion_rejection() == ('rejected_error', 'Robot is in error (bare)')
    manager.clear_error()
    manager.set_inhibited(True)
    expected = ('rejected_inhibited', 'New motion is inhibited while the HA link is down')
    assert manager.motion_rejection() == expected
    assert through_authoriser() == expected
    assert through_manager() == expected

    manager.set_inhibited(False)
    assert manager.motion_rejection() is None
    assert authorised(authoriser, harness.task_command(RESET)).authorised
    assert len(harness.executor.started) == 1, 'every refusal above kept the executor out of it'


# --------------------------------------------------------------------------- error flag


def test_error_during_a_task_keeps_the_task_and_outlives_it(
    harness: Harness, schema_validator: Any
) -> None:
    manager = harness.manager
    command, _ = harness.start(WAIT)
    manager.set_error('executor_error', 'Raised by the plugin')
    state = harness.states[-1].payload
    schema_validator.assert_valid('state', state)
    assert state['state'] == 'error' and state['task_id'] == command.id
    harness.executor.finish(command.id)
    state = harness.states[-1].payload
    assert state['state'] == 'error' and state['task_id'] is None
    manager.set_error('executor_error', 'Raised by the plugin')
    assert len(harness.states) == 3, 'the same error again changes nothing'
    manager.clear_error()
    assert harness.states[-1].payload['state'] == 'idle'
    manager.clear_error()
    assert len(harness.states) == 4


def test_error_code_is_normalised_on_the_wire(harness: Harness, schema_validator: Any) -> None:
    harness.manager.set_error('E-Stop Pressed', 'Hardware')
    state = harness.states[-1].payload
    schema_validator.assert_valid('state', state)
    assert state['error'] == {'code': 'e_stop_pressed', 'reason': 'Hardware'}
    assert harness.manager.error is not None and harness.manager.error.code == 'e_stop_pressed'


def test_set_error_with_a_bad_code_or_reason_still_publishes_the_state(
    harness: Harness, schema_validator: Any
) -> None:
    """``set_error`` is executor-facing: a non-string reason must not break the state publish."""
    manager = harness.manager
    manager.set_error('controller_error', RuntimeError('driver gone'))  # type: ignore[arg-type]
    assert manager.state == 'error'
    state = harness.states[-1].payload
    schema_validator.assert_valid('state', state)
    assert state['error'] == {'code': 'controller_error', 'reason': 'driver gone'}
    manager.set_error({'not': 'a code'}, {'detail': 7})  # type: ignore[arg-type]
    state = harness.states[-1].payload
    schema_validator.assert_valid('state', state)
    assert state['error'] == {'code': 'unknown', 'reason': "{'detail': 7}"}
    assert len(harness.states) == 2
    assert manager.motion_rejection() == (
        'rejected_error',
        "Robot is in error (unknown): {'detail': 7}",
    )


# --------------------------------------------------------------------------- watchdog hooks


def test_stop_and_hold_ends_the_motion_task_cancelled_inhibited(
    any_post_harness: Harness, schema_validator: Any
) -> None:
    harness = any_post_harness
    manager, executor = harness.manager, harness.executor
    command, _ = harness.start(WAIT)
    manager.set_ha_link(False, inhibited=True)
    assert len(harness.states) == 2, 'link and inhibited change in one publication'
    assert harness.states[-1].payload['ha_link'] == 'down'
    assert harness.states[-1].payload['inhibited'] is True

    manager.stop_and_hold('HA heartbeat lost')
    assert executor.holds == ['HA heartbeat lost']
    assert executor.cancelled == [command.id], 'an executor that did not finish is cancelled'
    harness.settle()
    result = harness.results[0].payload
    schema_validator.assert_valid('result', result)
    assert result['status'] == 'cancelled' and result['code'] == 'inhibited'
    assert result['reason'] == 'HA heartbeat lost'
    state = harness.states[-1].payload
    assert state['state'] == 'idle' and state['error'] is None and state['inhibited'] is True
    assert manager.error is None
    assert harness.scheduler.pending == []

    executor.finish(command.id, 'cancelled', 'cancelled')
    harness.settle()
    assert len(harness.results) == 1, 'the executor finishing later is ignored'
    manager.stop_and_hold('still down')
    assert executor.holds == ['HA heartbeat lost', 'still down']
    assert len(harness.results) == 1


def test_stop_and_hold_overrides_a_result_the_cancel_produced(
    any_post_harness: Harness, caplog: pytest.LogCaptureFixture
) -> None:
    harness = any_post_harness
    caplog.set_level(logging.WARNING, logger=LOGGER)
    executor = harness.executor
    executor.on_cancel = lambda task_id: executor.finish(task_id, 'cancelled', 'preempted')
    command, _ = harness.start(WAIT)
    harness.manager.stop_and_hold('lost')
    assert executor.cancelled == [command.id]
    harness.settle()
    assert [r.payload['code'] for r in harness.results] == ['inhibited']
    assert harness.results[0].payload['reason'] == 'lost'
    assert harness.manager.state == 'idle' and harness.scheduler.pending == []
    assert late_finish_warnings(caplog) == []


def test_stop_and_hold_honours_a_finish_the_executor_made_itself(
    any_post_harness: Harness, schema_validator: Any, caplog: pytest.LogCaptureFixture
) -> None:
    """An executor whose ``stop_and_hold`` ends the task is not also asked to cancel it.

    Under the node's deferred post that finish is still queued when the
    manager looks; the handle's flag says it is on its way, so the manager
    neither cancels a dead task nor completes it a second time, and the queued
    finish carries the ``cancelled/inhibited`` override.
    """
    harness = any_post_harness
    caplog.set_level(logging.WARNING, logger=LOGGER)
    manager, executor = harness.manager, harness.executor
    command, _ = harness.start(WAIT)
    executor.on_hold = lambda reason: executor.finish(command.id, 'cancelled', 'cancelled', reason)

    manager.stop_and_hold('HA heartbeat lost')
    assert executor.holds == ['HA heartbeat lost']
    assert executor.cancelled == [], 'cancel() is never called for a task the executor finished'
    harness.settle()
    assert len(harness.results) == 1
    result = harness.results[0].payload
    schema_validator.assert_valid('result', result)
    assert result['status'] == 'cancelled' and result['code'] == 'inhibited'
    assert result['reason'] == 'HA heartbeat lost'
    assert manager.state == 'idle' and manager.error is None
    assert harness.scheduler.pending == []
    assert late_finish_warnings(caplog) == []
    assert [r.getMessage() for r in caplog.records] == ['stop and hold: HA heartbeat lost']


def test_deadline_and_grace_do_not_pre_empt_a_finish_that_is_queued(
    clock: FakeClock, scheduler: FakeScheduler, caplog: pytest.LogCaptureFixture
) -> None:
    harness = Harness(clock, scheduler, deferred_post=True)
    caplog.set_level(logging.WARNING, logger=LOGGER)
    executor = harness.executor
    # The deadline fires with the finish already posted: no cancel, no forced abort.
    command, _ = harness.start(WAIT)
    clock.advance(9.9)
    executor.finish(command.id, 'succeeded', 'ok')
    clock.advance(0.2)
    assert executor.cancelled == [] and harness.results == []
    harness.settle()
    assert harness.results[0].payload['status'] == 'succeeded'
    assert harness.manager.state == 'idle' and scheduler.pending == []
    # The grace fires with the finish already posted: the executor's result stands.
    command, _ = harness.start(WAIT)
    clock.advance(10)
    assert executor.cancelled == [command.id]
    clock.advance(4.9)
    executor.finish(command.id, 'cancelled', 'cancelled', 'late but real')
    clock.advance(0.2)
    assert len(harness.results) == 1
    harness.settle()
    assert harness.results[1].payload['code'] == 'cancelled'
    assert harness.manager.state == 'idle' and harness.manager.error is None
    assert scheduler.pending == [] and late_finish_warnings(caplog) == []


def test_set_ha_link_and_inhibited(harness: Harness, schema_validator: Any) -> None:
    manager = harness.manager
    manager.set_ha_link(True)
    assert manager.ha_link == 'up' and harness.states[-1].payload['ha_link'] == 'up'
    manager.set_ha_link(True)
    assert len(harness.states) == 1
    manager.set_inhibited(True)
    assert manager.inhibited and harness.states[-1].payload['inhibited'] is True
    manager.set_ha_link(False, inhibited=False)
    state = harness.states[-1].payload
    schema_validator.assert_valid('state', state)
    assert state['ha_link'] == 'down' and state['inhibited'] is False
    assert len(harness.states) == 3


def test_watchdog_mode_is_announced_in_state(clock: FakeClock, scheduler: FakeScheduler) -> None:
    harness = Harness(clock, scheduler, watchdog_mode='stop_and_hold')
    harness.manager.publish_state()
    assert harness.states[0].payload['watchdog_mode'] == 'stop_and_hold'


# --------------------------------------------------------------------------- publishing


def test_publish_state_on_reconnect_bumps_seq(harness: Harness, schema_validator: Any) -> None:
    manager = harness.manager
    assert manager.publish_state() is True
    assert manager.publish_state() is True
    assert [s.payload['seq'] for s in harness.states] == [1, 2]
    assert harness.states[0].payload['since'] == harness.states[1].payload['since']
    schema_validator.assert_valid('state', harness.states[-1].payload)
    harness.publish_ok = False
    assert manager.publish_state() is False
    assert harness.states[-1].payload['seq'] == 3


def test_publish_failure_never_breaks_the_state_machine(harness: Harness) -> None:
    harness.publish_raises = True
    command, ack = harness.start(RESET)
    assert ack.code == 'accepted' and harness.manager.state == 'busy'
    harness.executor.finish(command.id)
    assert harness.manager.state == 'idle'
    harness.publish_raises = False
    assert harness.manager.publish_state() is True
    assert harness.states[-1].payload['seq'] == 3, 'seq counted the failed publications'


def test_bridge_context_is_wired_to_the_manager(harness: Harness) -> None:
    ctx = harness.executor.ctx
    assert ctx is not None
    assert ctx.robot_id == ROBOT and ctx.clock is harness.clock
    assert ctx.scheduler is harness.scheduler and ctx.ha_link_ok() is True
    command, _ = harness.start(WAIT)
    ctx.set_paused(True)
    assert harness.manager.paused
    ctx.set_error('x', 'y')
    assert harness.manager.state == 'error'
    ctx.clear_error()
    assert harness.manager.state == 'busy'
    ctx.set_paused(False)
    assert not harness.manager.paused
    harness.executor.finish(command.id)


# --------------------------------------------------------------------------- threads


def test_finish_from_a_plugin_thread_is_applied_on_the_main_thread(
    clock: FakeClock, scheduler: FakeScheduler, caplog: pytest.LogCaptureFixture
) -> None:
    caplog.set_level(logging.WARNING, logger=LOGGER)
    harness = Harness(clock, scheduler, deferred_post=True)
    command, ack = harness.start(RESET)
    assert ack.code == 'accepted'
    handle = harness.executor.handles[command.id]

    def plugin_thread() -> None:
        handle.feedback(0.5, 'moving', 'half way')
        handle.finish(TaskResult('succeeded', 'ok'))
        handle.finish(TaskResult('aborted', 'late'))

    worker = threading.Thread(target=plugin_thread)
    worker.start()
    worker.join()
    assert harness.results == [] and harness.feedbacks == []
    assert harness.manager.state == 'busy', 'nothing changed until the main thread drains'
    assert any('already finished' in r.getMessage() for r in caplog.records)

    assert harness.drain() == 2
    assert [p.kind for p in harness.published] == ['state', 'feedback', 'result', 'state']
    assert harness.manager.state == 'idle'

    harness.manager.set_error('x', 'y')
    assert harness.manager.state == 'idle'
    harness.drain()
    assert harness.manager.state == 'error'
