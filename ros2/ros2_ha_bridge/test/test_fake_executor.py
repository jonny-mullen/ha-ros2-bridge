# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Tests for :mod:`ros2_ha_bridge.executors.fake` driven through a real :class:`TaskManager`.

Every task of the catalogue, every failure class, the hang path, pause and
resume, and determinism against the fake clock: two identical runs must
publish byte-identical payloads.
"""

from __future__ import annotations

import json
import uuid
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import pytest

from ros2_ha_bridge import protocol
from ros2_ha_bridge.authoriser import resolve_request
from ros2_ha_bridge.executors.base import BridgeContext, TaskSpec, load_executor
from ros2_ha_bridge.executors.fake import (
    DEFAULT_FEEDBACK_HZ,
    DEFAULT_TASK_DURATION_S,
    FAILURE_CLASSES,
    FAILURE_REASONS,
    HANG_TIMEOUT_S,
    FakeExecutor,
)
from ros2_ha_bridge.protocol import AckPayload, Announce, DecodedCommand
from ros2_ha_bridge.task_manager import TaskManager

from .conftest import assert_imports_without
from .fakes import FakeClock, FakeScheduler

ROBOT = 'fake_arm'
SESSION = '9f1c7a0e-4a1b-4b2c-8d1e-0f1a2b3c4d5e'
INSTANCE = 'robotpc:4123'
PARAMETERS = {'fake.task_duration_s': 2.0, 'fake.feedback_hz': 2}


@dataclass(frozen=True)
class Published:
    topic: str
    payload: dict[str, Any]
    qos: int
    retain: bool
    expiry_s: int | None

    @property
    def kind(self) -> str:
        return self.topic.rsplit('/', 1)[1]


class Rig:
    """A TaskManager over a FakeExecutor with the parameters the node would pass."""

    def __init__(
        self,
        clock: FakeClock,
        scheduler: FakeScheduler,
        parameters: dict[str, Any] | None = None,
        *,
        ids: Callable[[], str] | None = None,
    ) -> None:
        self.clock = clock
        self.scheduler = scheduler
        self.published: list[Published] = []
        self.executor = FakeExecutor()
        self.manager = TaskManager(
            robot_id=ROBOT,
            executor=self.executor,
            publish=self._publish,
            post=lambda fn: fn(),
            clock=clock,
            scheduler=scheduler,
            session=SESSION,
            instance=INSTANCE,
        )
        self.executor.setup(
            self.manager.bridge_context(
                ha_link_ok=lambda: True,
                parameters=PARAMETERS if parameters is None else parameters,
            )
        )
        self.specs = {spec.name: spec for spec in self.executor.tasks()}
        self._ids = ids if ids is not None else lambda: str(uuid.uuid4())

    def _publish(
        self, topic: str, payload: Any, qos: int, retain: bool, expiry_s: int | None
    ) -> bool:
        # Encode exactly as the node's link would, so a payload the encoder rejects fails here.
        wire = protocol.encode_payload(payload)
        self.published.append(Published(topic, json.loads(wire), qos, retain, expiry_s))
        return True

    def of(self, kind: str) -> list[Published]:
        return [p for p in self.published if p.kind == kind]

    @property
    def results(self) -> list[dict[str, Any]]:
        return [p.payload for p in self.of('result')]

    @property
    def states(self) -> list[dict[str, Any]]:
        return [p.payload for p in self.of('state')]

    @property
    def feedbacks(self) -> list[dict[str, Any]]:
        return [p.payload for p in self.of('feedback')]

    def run(
        self, name: str, params: dict[str, Any] | None = None, *, timeout_s: float | None = None
    ) -> tuple[DecodedCommand, AckPayload]:
        command = protocol.decode_command(
            protocol.encode_payload(
                {
                    'schema': 1,
                    'id': self._ids(),
                    'type': 'task',
                    'robot_id': ROBOT,
                    'task': name,
                    'params': params or {},
                    'issued_at': self.clock.now_ms(),
                    'max_age_ms': 5000,
                    'timeout_s': timeout_s,
                    'source': {'origin': 'ha', 'kind': 'test'},
                }
            )
        )
        assert isinstance(command, DecodedCommand), command
        spec = self.specs[name]
        return command, self.manager.start(resolve_request(command, spec), spec)

    def cancel(self, target_id: str | None = None) -> AckPayload:
        command = protocol.decode_command(
            protocol.encode_payload(
                {
                    'schema': 1,
                    'id': self._ids(),
                    'type': 'cancel',
                    'robot_id': ROBOT,
                    'target_id': target_id,
                    'issued_at': self.clock.now_ms(),
                    'max_age_ms': 5000,
                    'source': {'origin': 'ha', 'kind': 'test'},
                }
            )
        )
        assert isinstance(command, DecodedCommand), command
        return self.manager.cancel(command)

    def result_of(self, command: DecodedCommand) -> dict[str, Any]:
        matching = [r for r in self.results if r['id'] == command.id]
        assert len(matching) == 1, f'expected one result for {command.id}, got {matching}'
        return matching[0]


@pytest.fixture
def rig(clock: FakeClock, scheduler: FakeScheduler) -> Rig:
    return Rig(clock, scheduler)


# --------------------------------------------------------------------------- hygiene and catalogue


def test_module_imports_neither_rclpy_nor_paho() -> None:
    assert_imports_without('ros2_ha_bridge.executors.fake', {'rclpy', 'paho'})


def test_catalogue_is_announceable(rig: Rig, schema_validator: Any) -> None:
    specs = rig.executor.tasks()
    assert [s.name for s in specs] == [
        'reset',
        'move_to',
        'wait',
        'fail',
        'hang',
        'raise_error',
        'clear_error',
        'stop',
        'pause',
        'resume',
    ]
    for spec in specs:
        assert protocol.validate_task_spec(spec) == [], spec
    kinds = {s.name: s.kind for s in specs}
    assert all(kinds[n] == 'motion' for n in ('reset', 'move_to', 'wait', 'fail', 'hang'))
    assert all(
        kinds[n] == 'control' for n in ('raise_error', 'clear_error', 'stop', 'pause', 'resume')
    )
    assert rig.specs['hang'].timeout_s == HANG_TIMEOUT_S == 10
    assert FakeExecutor.name == 'fake' and FakeExecutor.version == protocol.__version__
    announce = Announce(
        robot_id=ROBOT,
        name='Fake arm',
        session=SESSION,
        instance=INSTANCE,
        executor=protocol.ExecutorInfo(FakeExecutor.name, FakeExecutor.version),
        tasks=specs,
        ts=rig.clock.now_ms(),
    )
    schema_validator.assert_valid('announce', announce.to_payload())


def test_loads_by_dotted_path_and_reads_parameters(
    clock: FakeClock, scheduler: FakeScheduler
) -> None:
    executor = load_executor('ros2_ha_bridge.executors.fake:FakeExecutor')
    assert isinstance(executor, FakeExecutor)
    assert executor.task_duration_s == DEFAULT_TASK_DURATION_S == 2.0
    assert executor.feedback_hz == DEFAULT_FEEDBACK_HZ == 2.0
    rig = Rig(clock, scheduler, {'fake.task_duration_s': 0.5, 'fake.feedback_hz': 0})
    assert rig.executor.task_duration_s == 0.5 and rig.executor.feedback_hz == 0.0
    command, _ = rig.run('reset')
    clock.advance(0.5)
    assert rig.result_of(command)['status'] == 'succeeded'
    assert rig.feedbacks == [], 'feedback_hz 0 sends no feedback'
    bad = Rig(clock, scheduler, {'fake.task_duration_s': -1, 'fake.feedback_hz': 'fast'})
    assert bad.executor.task_duration_s == 2.0 and bad.executor.feedback_hz == 2.0


def test_setup_requires_a_scheduler(clock: FakeClock) -> None:
    ctx = BridgeContext(
        robot_id=ROBOT,
        logger=__import__('logging').getLogger('test'),
        post=lambda fn: fn(),
        set_error=lambda code, reason: None,
        clear_error=lambda: None,
        set_paused=lambda paused: None,
        ha_link_ok=lambda: True,
        clock=clock,
    )
    with pytest.raises(RuntimeError):
        FakeExecutor().setup(ctx)


# --------------------------------------------------------------------------- motion tasks


def test_reset_succeeds_after_task_duration_with_feedback(rig: Rig, schema_validator: Any) -> None:
    command, ack = rig.run('reset')
    assert ack.code == 'accepted'
    assert rig.manager.state == 'busy' and rig.executor.running_ids == (command.id,)
    rig.clock.advance(1.99)
    assert rig.results == []
    rig.clock.advance(0.01)
    result = rig.result_of(command)
    schema_validator.assert_valid('result', result)
    assert result['status'] == 'succeeded' and result['code'] == 'ok'
    assert result['kind'] == 'motion' and result['task'] == 'reset'
    assert result['finished_at'] - result['started_at'] == 2000
    assert [f['progress'] for f in rig.feedbacks] == [0.25, 0.5, 0.75]
    assert rig.feedbacks[0]['stage'] == 'executing'
    for feedback in rig.feedbacks:
        schema_validator.assert_valid('feedback', feedback)
    assert rig.manager.state == 'idle' and rig.executor.running_ids == ()
    assert rig.scheduler.pending == []


def test_move_to_reports_the_pose_and_needs_one(rig: Rig) -> None:
    command, ack = rig.run('move_to', {'pose': 'home'})
    assert ack.code == 'accepted'
    rig.clock.advance(2)
    result = rig.result_of(command)
    assert result['status'] == 'succeeded' and result['data'] == {'pose': 'home'}
    assert result['reason'] == "Moved to pose 'home'"
    _, ack = rig.run('move_to', {})
    assert ack.code == 'rejected_executor' and "'pose' is required" in ack.reason
    _, ack = rig.run('move_to', {'pose': 7})
    assert ack.code == 'rejected_executor' and 'must be a string' in ack.reason
    assert rig.manager.state == 'idle'


def test_wait_reports_progress_and_is_cancellable(rig: Rig, schema_validator: Any) -> None:
    command, ack = rig.run('wait', {'seconds': 5})
    assert ack.code == 'accepted'
    rig.clock.advance(2)
    assert [f['progress'] for f in rig.feedbacks] == [0.1, 0.2, 0.3, 0.4]
    assert rig.feedbacks[-1]['message'] == '2.0 of 5 s'
    ack = rig.cancel()
    assert ack.code == 'accepted'
    result = rig.result_of(command)
    schema_validator.assert_valid('result', result)
    assert result['status'] == 'cancelled' and result['code'] == 'cancelled'
    assert result['reason'] == 'Cancelled by request'
    assert rig.manager.state == 'idle' and rig.executor.running_ids == ()
    rig.clock.advance(10)
    assert len(rig.results) == 1 and len(rig.feedbacks) == 4


def test_wait_completes_on_time(rig: Rig) -> None:
    command, _ = rig.run('wait', {'seconds': 3.5})
    rig.clock.advance(3.49)
    assert rig.results == []
    rig.clock.advance(0.01)
    assert rig.result_of(command)['data'] == {'seconds': 3.5}
    zero, ack = rig.run('wait', {'seconds': 0})
    assert ack.code == 'accepted'
    assert rig.result_of(zero)['status'] == 'succeeded', 'a zero wait finishes at once'
    _, ack = rig.run('wait', {'seconds': -1})
    assert ack.code == 'rejected_executor'


@pytest.mark.parametrize('code', FAILURE_CLASSES)
def test_fail_aborts_with_the_chosen_failure_class(
    rig: Rig, schema_validator: Any, code: str
) -> None:
    command, ack = rig.run('fail', {'code': code})
    assert ack.code == 'accepted'
    rig.clock.advance(2)
    result = rig.result_of(command)
    schema_validator.assert_valid('result', result)
    assert result['status'] == 'aborted' and result['goal_status'] == 6
    assert result['code'] == code and result['reason'] == FAILURE_REASONS[code]
    assert result['data'] == {'simulated': True, 'failure_class': code}
    assert rig.manager.state == 'idle', 'a failed task alone never sets the robot error'
    assert rig.states[-1]['error'] is None


def test_fail_codes_cover_the_moveit_classes_and_refuse_others(rig: Rig) -> None:
    assert FAILURE_CLASSES == (
        'collision',
        'self_collision',
        'no_trajectory',
        'planning_failed',
        'control_failed',
        'invalid_goal',
    )
    assert rig.specs['fail'].params is not None
    assert rig.specs['fail'].params['properties']['code']['enum'] == list(FAILURE_CLASSES)
    _, ack = rig.run('fail', {'code': 'meteor'})
    assert ack.code == 'rejected_executor' and 'must be one of' in ack.reason


def test_hang_never_finishes_and_is_forced_to_timeout(rig: Rig, schema_validator: Any) -> None:
    command, ack = rig.run('hang')
    assert ack.code == 'accepted'
    assert rig.executor.running_ids == (), 'hang registers nothing it could cancel'
    rig.clock.advance(10)
    assert rig.results == [], 'the timeout only asks for a cancel'
    rig.clock.advance(4.99)
    assert rig.results == []
    rig.clock.advance(0.01)
    result = rig.result_of(command)
    schema_validator.assert_valid('result', result)
    assert result['status'] == 'aborted' and result['code'] == 'timeout'
    assert result['reason'] == "Task 'hang' did not finish within 10 s and ignored cancel"
    state = rig.states[-1]
    schema_validator.assert_valid('state', state)
    assert state['state'] == 'error' and state['error']['code'] == 'timeout'
    _, ack = rig.run('reset')
    assert ack.code == 'rejected_error'

    clear, ack = rig.run('clear_error')
    assert ack.code == 'accepted'
    assert rig.result_of(clear)['kind'] == 'control'
    assert rig.manager.state == 'idle'
    _, ack = rig.run('reset')
    assert ack.code == 'accepted'


# --------------------------------------------------------------------------- control tasks


def test_raise_error_and_clear_error(rig: Rig, schema_validator: Any) -> None:
    raised, ack = rig.run('raise_error')
    assert ack.code == 'accepted'
    result = rig.result_of(raised)
    schema_validator.assert_valid('result', result)
    assert result['kind'] == 'control' and result['status'] == 'succeeded'
    state = rig.states[-1]
    assert state['state'] == 'error'
    assert state['error'] == {
        'code': 'executor_error',
        'reason': 'Error raised by the fake executor',
    }
    cleared, _ = rig.run('clear_error')
    assert rig.result_of(cleared)['status'] == 'succeeded'
    assert rig.manager.state == 'idle' and rig.states[-1]['error'] is None


def test_stop_preempts_the_running_task_and_clears_the_error(
    rig: Rig, schema_validator: Any
) -> None:
    running, _ = rig.run('wait', {'seconds': 30})
    rig.run('raise_error')
    assert rig.manager.state == 'error' and rig.manager.active_task_id == running.id
    stop, ack = rig.run('stop')
    assert ack.code == 'accepted'
    preempted = rig.result_of(running)
    schema_validator.assert_valid('result', preempted)
    assert preempted['status'] == 'cancelled' and preempted['code'] == 'preempted'
    assert preempted['reason'] == "Stopped by task 'stop'"
    stopped = rig.result_of(stop)
    assert stopped['kind'] == 'control' and stopped['status'] == 'succeeded'
    assert stopped['data'] == {'stopped': [running.id]}
    assert rig.manager.state == 'idle' and rig.manager.error is None
    assert rig.executor.running_ids == ()
    idle_stop, _ = rig.run('stop')
    assert rig.result_of(idle_stop)['data'] == {'stopped': []}
    assert rig.result_of(idle_stop)['reason'] == 'Nothing was running'


def test_pause_and_resume_freeze_both_clocks(rig: Rig, schema_validator: Any) -> None:
    clock, scheduler = rig.clock, rig.scheduler
    running, _ = rig.run('wait', {'seconds': 6}, timeout_s=10)
    clock.advance(2)
    assert [f['progress'] for f in rig.feedbacks] == pytest.approx(
        [1 / 12, 2 / 12, 3 / 12, 4 / 12]
    )

    pause, ack = rig.run('pause')
    assert ack.code == 'accepted'
    assert rig.result_of(pause)['status'] == 'succeeded'
    assert rig.executor.is_paused(running.id) and rig.manager.paused
    paused = rig.states[-1]
    schema_validator.assert_valid('state', paused)
    assert paused['state'] == 'busy' and paused['paused'] is True
    assert paused['task_id'] == running.id
    assert scheduler.pending == [], 'neither the fake nor the manager has a timer running'

    clock.advance(50)
    assert len(rig.feedbacks) == 4 and len(rig.results) == 1
    again, _ = rig.run('pause')
    assert rig.result_of(again)['code'] == 'nothing_to_pause'
    assert rig.result_of(again)['reason'] == 'Task is already paused'

    resume, _ = rig.run('resume')
    assert rig.result_of(resume)['status'] == 'succeeded'
    assert not rig.manager.paused and rig.states[-1]['paused'] is False
    remaining = sorted(t.remaining_s() for t in scheduler.pending)
    assert remaining == pytest.approx([0.5, 4.0, 8.0]), 'ticker, completion, manager deadline'
    clock.advance(3.99)
    assert len(rig.results) == 3
    clock.advance(0.01)
    result = rig.result_of(running)
    assert result['status'] == 'succeeded'
    assert result['finished_at'] - result['started_at'] == 56_000
    assert [f['progress'] for f in rig.feedbacks][-1] == pytest.approx(11 / 12)
    assert scheduler.pending == []


def test_pause_and_resume_with_nothing_to_do(rig: Rig, schema_validator: Any) -> None:
    pause, ack = rig.run('pause')
    assert ack.code == 'accepted'
    result = rig.result_of(pause)
    schema_validator.assert_valid('result', result)
    assert result['kind'] == 'control' and result['status'] == 'aborted'
    assert result['code'] == 'nothing_to_pause' and result['reason'] == 'No task is running'
    assert result['goal_status'] == 6
    resume, _ = rig.run('resume')
    result = rig.result_of(resume)
    assert result['status'] == 'aborted' and result['code'] == 'nothing_to_resume'
    assert rig.manager.state == 'idle' and not rig.manager.paused
    assert [s['paused'] for s in rig.states] == []


def test_cancel_of_a_paused_task(rig: Rig) -> None:
    running, _ = rig.run('wait', {'seconds': 6})
    rig.run('pause')
    assert rig.cancel().code == 'accepted'
    assert rig.result_of(running)['status'] == 'cancelled'
    assert not rig.manager.paused and rig.manager.state == 'idle'


# --------------------------------------------------------------------------- watchdog hooks and health


def test_stop_and_hold_ends_runs_cancelled_inhibited(rig: Rig, schema_validator: Any) -> None:
    running, _ = rig.run('wait', {'seconds': 30})
    rig.manager.stop_and_hold('HA heartbeat lost')
    assert rig.executor.hold_reasons == ['HA heartbeat lost']
    assert rig.executor.running_ids == ()
    result = rig.result_of(running)
    schema_validator.assert_valid('result', result)
    assert result['status'] == 'cancelled' and result['code'] == 'inhibited'
    assert result['reason'] == 'HA heartbeat lost'
    assert rig.manager.state == 'idle' and rig.manager.error is None


def test_health_and_shutdown(rig: Rig) -> None:
    assert rig.executor.health() == ('ok', '')
    rig.executor.health_report = ('degraded', 'simulated')
    assert rig.manager.poll_health().level == 'degraded'
    rig.run('wait', {'seconds': 30})
    rig.executor.shutdown()
    assert rig.executor.running_ids == () and rig.scheduler.pending != []
    rig.scheduler.cancel_all()


# --------------------------------------------------------------------------- determinism


def _script(rig: Rig) -> None:
    rig.run('reset')
    rig.clock.advance(2)
    waiting, _ = rig.run('wait', {'seconds': 4})
    rig.clock.advance(1.3)
    rig.run('pause')
    rig.clock.advance(7)
    rig.run('resume')
    rig.clock.advance(1)
    rig.cancel(waiting.id)
    rig.run('fail', {'code': 'collision'})
    rig.clock.advance(2)
    rig.run('hang')
    rig.clock.advance(15)
    rig.run('stop')
    rig.manager.publish_state()


def test_two_runs_against_the_fake_clock_are_identical(schema_validator: Any) -> None:
    def counter_ids() -> Callable[[], str]:
        count = iter(range(1, 1000))
        return lambda: f'00000000-0000-4000-8000-{next(count):012d}'

    runs: list[list[Published]] = []
    for _ in range(2):
        clock = FakeClock()
        rig = Rig(clock, FakeScheduler(clock), ids=counter_ids())
        _script(rig)
        runs.append(rig.published)
    assert runs[0] == runs[1]
    kinds = [p.kind for p in runs[0]]
    assert kinds.count('result') == 7, 'reset, wait, pause, resume, fail, hang, stop'
    for published in runs[0]:
        schema_validator.assert_valid(published.kind, published.payload)
    assert [r['code'] for r in (p.payload for p in runs[0] if p.kind == 'result')] == [
        'ok',
        'ok',
        'ok',
        'cancelled',
        'collision',
        'timeout',
        'ok',
    ]


def test_task_spec_shapes_match_the_design(rig: Rig) -> None:
    assert isinstance(rig.specs['move_to'], TaskSpec)
    assert rig.specs['move_to'].params == {
        'type': 'object',
        'properties': {'pose': {'type': 'string', 'description': 'Named pose to move to'}},
        'required': ['pose'],
    }
    assert rig.specs['wait'].params is not None
    assert rig.specs['wait'].params['required'] == ['seconds']
    assert rig.specs['wait'].params['properties']['seconds']['minimum'] == 0
    assert all(rig.specs[n].params is None for n in ('reset', 'hang', 'stop', 'pause', 'resume'))
