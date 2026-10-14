# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Tests for :mod:`ros2_ha_bridge.executors.moveit` without ROS, through a real :class:`TaskManager`.

A fake :class:`MoveGroupAdapter` stands in for move_group: the test decides
when a goal is accepted, what feedback it emits and which
``MoveItErrorCodes`` value it ends with, and feeds ``JointState`` messages
for the joint telemetry (0.2.0). The rclpy goal handle is exercised with fake
futures, and the message builder and the joint-state subscription with fake
message and node classes, so the parts that only run on the Isaac machine
are at least shape-checked here.
"""

from __future__ import annotations

import json
import logging
import sys
import types
import uuid
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from ros2_ha_bridge import protocol
from ros2_ha_bridge.authoriser import resolve_request
from ros2_ha_bridge.executors import moveit
from ros2_ha_bridge.executors.base import (
    BridgeContext,
    JointInfo,
    TaskRejected,
    TaskRequest,
    load_executor,
)
from ros2_ha_bridge.executors.moveit import (
    DEFAULT_ACTION_NAME,
    DEFAULT_JOINT_STATES_TOPIC,
    HOME_POSE,
    JOINT_TOLERANCE_RAD,
    MOVEIT_ERROR_CLASSES,
    MOVEIT_ERROR_NAMES,
    STATUS_ABORTED,
    STATUS_CANCELED,
    STATUS_SUCCEEDED,
    UNAVAILABLE_CODE,
    GoalOutcome,
    JointStateCallback,
    MoveItExecutor,
    MoveItSettings,
    announce_joint_names,
    build_goal,
    map_error_code,
    parse_poses,
)
from ros2_ha_bridge.protocol import AckPayload, Announce, AnnounceJoints, DecodedCommand
from ros2_ha_bridge.task_manager import TaskManager

from .conftest import assert_imports_without
from .fakes import FakeClock, FakeScheduler

ROBOT = 'sim_arm'
SESSION = '9f1c7a0e-4a1b-4b2c-8d1e-0f1a2b3c4d5e'
INSTANCE = 'robotpc:4123'
HOME = {'shoulder_pan_joint': 0.0, 'shoulder_lift_joint': -1.5708, 'elbow_joint': 1.5708}
REST = {'shoulder_pan_joint': 0.0, 'shoulder_lift_joint': -2.3562, 'elbow_joint': 2.3562}
PARAMETERS: dict[str, Any] = {
    'moveit.group_name': 'arm',
    'moveit.poses': {'home': HOME, 'rest': REST},
    'moveit.planning_time_s': 2.5,
    'moveit.num_attempts': 3,
    'moveit.velocity_scale': 0.2,
    'moveit.acceleration_scale': 0.3,
    'moveit.hold_on_cancel': True,
    # A parameter of the bridge itself, which the executor must ignore.
    'robot.id': ROBOT,
}
PACKAGE_DIR = Path(__file__).resolve().parent.parent


# --------------------------------------------------------------------------- fakes


class FakeGoalHandle:
    """A sent goal the test completes by hand."""

    def __init__(
        self, adapter: FakeAdapter, goal: dict[str, Any], feedback: Callable[[str], None]
    ):
        self.adapter = adapter
        self.goal = goal
        self._feedback = feedback
        self._callbacks: list[Callable[[GoalOutcome], None]] = []
        self.outcome: GoalOutcome | None = None
        self.cancel_requests = 0

    # GoalHandle protocol
    def add_done(self, callback: Callable[[GoalOutcome], None]) -> None:
        if self.outcome is not None:
            callback(self.outcome)
        else:
            self._callbacks.append(callback)

    def cancel(self) -> None:
        if self.adapter.cancel_raises is not None:
            raise self.adapter.cancel_raises
        self.cancel_requests += 1

    # test helpers
    @property
    def cancelled(self) -> bool:
        return self.cancel_requests > 0

    def feed(self, state: str) -> None:
        self._feedback(state)

    def finish(self, outcome: GoalOutcome) -> None:
        assert self.outcome is None, 'a goal ends once'
        self.outcome = outcome
        callbacks, self._callbacks = self._callbacks, []
        for callback in callbacks:
            callback(outcome)

    def complete(self, code: int, message: str = '', *, status: int | None = None) -> None:
        """End the goal with a MoveItErrorCodes value (PREEMPTED after a cancel reports CANCELED)."""
        if status is None:
            if code == 1:
                status = STATUS_SUCCEEDED
            elif code == -7 and self.cancelled:
                status = STATUS_CANCELED
            else:
                status = STATUS_ABORTED
        self.finish(
            GoalOutcome(
                accepted=True,
                status=status,
                error_code=code,
                message=message,
                planning_time_s=0.25,
            )
        )

    def reject(self) -> None:
        self.finish(GoalOutcome(accepted=False, detail='move_group rejected the goal'))


class FakeAdapter:
    """:class:`MoveGroupAdapter` double: records goals, lets the test drive them and feed joint states."""

    def __init__(self) -> None:
        self.available = True
        self.goals: list[FakeGoalHandle] = []
        self.closed = False
        self.send_raises: Exception | None = None
        self.cancel_raises: Exception | None = None
        self.joint_state_subscriptions: list[tuple[str, JointStateCallback]] = []

    def server_available(self) -> bool:
        return self.available

    def send_goal(self, goal: dict[str, Any], feedback: Callable[[str], None]) -> FakeGoalHandle:
        if self.send_raises is not None:
            raise self.send_raises
        handle = FakeGoalHandle(self, goal, feedback)
        self.goals.append(handle)
        return handle

    def subscribe_joint_states(self, topic: str, callback: JointStateCallback) -> None:
        self.joint_state_subscriptions.append((topic, callback))

    def close(self) -> None:
        self.closed = True

    @property
    def last(self) -> FakeGoalHandle:
        assert self.goals, 'no goal was sent'
        return self.goals[-1]

    def feed_joint_state(
        self,
        names: Sequence[str],
        positions: Sequence[float],
        velocities: Sequence[float] | None = None,
    ) -> None:
        """Deliver one ``JointState`` to every subscriber, as the rclpy adapter would."""
        for _, callback in self.joint_state_subscriptions:
            callback(names, positions, velocities)


class AdapterWithoutJointStates:
    """An older adapter that predates ``subscribe_joint_states``."""

    def __init__(self) -> None:
        self.inner = FakeAdapter()

    def server_available(self) -> bool:
        return self.inner.server_available()

    def send_goal(self, goal: dict[str, Any], feedback: Callable[[str], None]) -> FakeGoalHandle:
        return self.inner.send_goal(goal, feedback)

    def close(self) -> None:
        self.inner.close()


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


@dataclass
class Rig:
    """A TaskManager over a MoveItExecutor with a fake adapter, the way the node wires it."""

    clock: FakeClock
    scheduler: FakeScheduler
    parameters: dict[str, Any] = field(default_factory=lambda: dict(PARAMETERS))
    published: list[Published] = field(default_factory=list)
    #: Every ``ctx.report_joints`` call: (positions, velocities).
    joints: list[tuple[tuple[float, ...], tuple[float, ...] | None]] = field(default_factory=list)
    adapter: Any = None

    def __post_init__(self) -> None:
        if self.adapter is None:
            self.adapter = FakeAdapter()
        self.executor = MoveItExecutor(adapter=self.adapter)
        self.manager = TaskManager(
            robot_id=ROBOT,
            executor=self.executor,
            publish=self._publish,
            post=lambda fn: fn(),
            clock=self.clock,
            scheduler=self.scheduler,
            session=SESSION,
            instance=INSTANCE,
        )
        self.executor.setup(
            self.manager.bridge_context(
                ha_link_ok=lambda: True,
                parameters=self.parameters,
                report_joints=self._report_joints,
            )
        )
        self.specs = {spec.name: spec for spec in self.executor.tasks()}

    def _publish(
        self, topic: str, payload: Any, qos: int, retain: bool, expiry_s: int | None
    ) -> bool:
        wire = protocol.encode_payload(payload)
        self.published.append(Published(topic, json.loads(wire), qos, retain, expiry_s))
        return True

    def _report_joints(
        self, positions: Sequence[float], velocities: Sequence[float] | None
    ) -> None:
        self.joints.append((tuple(positions), None if velocities is None else tuple(velocities)))

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
                    'id': str(uuid.uuid4()),
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
                    'id': str(uuid.uuid4()),
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


def make_rig(clock: FakeClock, scheduler: FakeScheduler, **overrides: Any) -> Rig:
    parameters = dict(PARAMETERS)
    parameters.update(overrides)
    return Rig(clock, scheduler, parameters)


# --------------------------------------------------------------------------- hygiene and catalogue


def test_module_imports_neither_rclpy_nor_paho() -> None:
    assert_imports_without('ros2_ha_bridge.executors.moveit', {'rclpy', 'paho', 'moveit_msgs'})


def test_catalogue_comes_from_the_poses(rig: Rig, schema_validator: Any) -> None:
    specs = rig.executor.tasks()
    assert [s.name for s in specs] == ['move_to', 'home', 'stop', 'refresh_poses']
    for spec in specs:
        assert protocol.validate_task_spec(spec) == [], spec
    kinds = {s.name: s.kind for s in specs}
    assert kinds == {
        'move_to': 'motion',
        'home': 'motion',
        'stop': 'control',
        'refresh_poses': 'query',
    }
    move_to = rig.specs['move_to'].params
    assert move_to is not None
    assert move_to['properties']['pose']['enum'] == ['home', 'rest']
    assert move_to['required'] == ['pose']
    assert rig.specs['home'].params is None and rig.specs['stop'].params is None
    assert MoveItExecutor.name == 'moveit' and MoveItExecutor.version == protocol.__version__
    announce = Announce(
        robot_id=ROBOT,
        name='Simulated arm',
        session=SESSION,
        instance=INSTANCE,
        executor=protocol.ExecutorInfo(MoveItExecutor.name, MoveItExecutor.version),
        tasks=specs,
        ts=rig.clock.now_ms(),
    )
    schema_validator.assert_valid('announce', announce.to_payload())


def test_home_task_exists_only_with_a_home_pose(
    clock: FakeClock, scheduler: FakeScheduler
) -> None:
    rig = make_rig(clock, scheduler, **{'moveit.poses': {'rest': REST, 'tucked': HOME}})
    assert [s.name for s in rig.executor.tasks()] == ['move_to', 'stop', 'refresh_poses']
    assert rig.specs['move_to'].params is not None
    assert rig.specs['move_to'].params['properties']['pose']['enum'] == ['rest', 'tucked']


def test_loads_by_dotted_path_and_is_registered(
    clock: FakeClock, scheduler: FakeScheduler
) -> None:
    executor = load_executor('ros2_ha_bridge.executors.moveit:MoveItExecutor')
    assert isinstance(executor, MoveItExecutor)
    setup_py = (PACKAGE_DIR / 'setup.py').read_text(encoding='utf-8')
    assert 'moveit = ros2_ha_bridge.executors.moveit:MoveItExecutor' in setup_py
    package_xml = (PACKAGE_DIR / 'package.xml').read_text(encoding='utf-8')
    assert 'moveit_msgs' in package_xml
    example = (PACKAGE_DIR / 'config' / 'moveit.example.yaml').read_text(encoding='utf-8')
    assert 'executor: moveit' in example


def test_setup_needs_a_node_or_an_adapter(clock: FakeClock) -> None:
    ctx = BridgeContext(
        robot_id=ROBOT,
        logger=logging.getLogger('test'),
        post=lambda fn: fn(),
        set_error=lambda code, reason: None,
        clear_error=lambda: None,
        set_paused=lambda paused: None,
        ha_link_ok=lambda: True,
        clock=clock,
        parameters=PARAMETERS,
    )
    with pytest.raises(RuntimeError, match='ctx.node'):
        MoveItExecutor().setup(ctx)
    with pytest.raises(RuntimeError, match='setup'):
        MoveItExecutor().tasks()


# --------------------------------------------------------------------------- settings


def test_settings_defaults_and_values() -> None:
    settings = MoveItSettings.from_parameters(PARAMETERS)
    assert settings.group_name == 'arm' and settings.action_name == DEFAULT_ACTION_NAME
    assert settings.planning_time_s == 2.5 and settings.num_attempts == 3
    assert settings.velocity_scale == 0.2 and settings.acceleration_scale == 0.3
    assert settings.hold_on_cancel is True and settings.poses_file is None
    assert settings.joint_states_topic == DEFAULT_JOINT_STATES_TOPIC == '/joint_states'
    assert settings.joint_names == ()
    assert settings.joint_tolerance_rad == JOINT_TOLERANCE_RAD == 0.01
    assert settings.motion_timeout_s == moveit.MOTION_TIMEOUT_S == 120.0
    assert settings.control_timeout_s == moveit.CONTROL_TIMEOUT_S == 5.0
    assert settings.load_poses() == {'home': HOME, 'rest': REST}
    custom = MoveItSettings.from_parameters(
        {
            **PARAMETERS,
            'moveit.joint_states_topic': ' /robot/joint_states ',
            'moveit.joint_names': ['elbow_joint', 'shoulder_pan_joint '],
            'moveit.joint_tolerance_rad': 0.05,
            'moveit.motion_timeout_s': 300,
            'moveit.control_timeout_s': 2.5,
        }
    )
    assert custom.joint_states_topic == '/robot/joint_states'
    assert custom.joint_names == ('elbow_joint', 'shoulder_pan_joint')
    assert custom.joint_tolerance_rad == 0.05
    assert (custom.motion_timeout_s, custom.control_timeout_s) == (300.0, 2.5)
    assert isinstance(custom.motion_timeout_s, float), 'an integer serves a double'
    minimal = MoveItSettings.from_parameters(
        {'moveit.group_name': 'arm', 'moveit.poses': {'a': {'j': 1}}, 'moveit.num_attempts': 2.0}
    )
    assert minimal.planning_time_s == 5.0 and minimal.num_attempts == 2
    assert minimal.velocity_scale == 0.1 and minimal.acceleration_scale == 0.1
    assert minimal.hold_on_cancel is True
    assert minimal.load_poses() == {'a': {'j': 1.0}}
    assert {name for name, _, _ in moveit.SCALAR_PARAMETERS} >= {
        'moveit.joint_tolerance_rad',
        'moveit.motion_timeout_s',
        'moveit.control_timeout_s',
    }, 'declared on the node like every other moveit.* parameter'


@pytest.mark.parametrize(
    ('overrides', 'match'),
    [
        ({'moveit.group_name': ''}, 'group_name'),
        ({'moveit.group_name': 7}, 'group_name'),
        ({'moveit.action_name': ' '}, 'action_name'),
        ({'moveit.planning_time_s': 0}, 'planning_time_s'),
        ({'moveit.planning_time_s': 'fast'}, 'planning_time_s'),
        ({'moveit.num_attempts': 0}, 'num_attempts'),
        ({'moveit.num_attempts': 1.5}, 'num_attempts'),
        ({'moveit.velocity_scale': 0}, 'velocity_scale'),
        ({'moveit.velocity_scale': 1.5}, 'velocity_scale'),
        ({'moveit.acceleration_scale': -0.1}, 'acceleration_scale'),
        ({'moveit.hold_on_cancel': 'yes'}, 'hold_on_cancel'),
        ({'moveit.joint_tolerance_rad': 0}, 'joint_tolerance_rad'),
        ({'moveit.joint_tolerance_rad': -0.01}, 'joint_tolerance_rad'),
        ({'moveit.joint_tolerance_rad': 'loose'}, 'joint_tolerance_rad'),
        ({'moveit.motion_timeout_s': 0}, 'motion_timeout_s'),
        ({'moveit.motion_timeout_s': float('inf')}, 'motion_timeout_s'),
        ({'moveit.control_timeout_s': -5}, 'control_timeout_s'),
        ({'moveit.control_timeout_s': True}, 'control_timeout_s'),
        ({'moveit.poses': {}}, 'no poses'),
        ({'moveit.poses': ''}, 'no poses'),
        ({'moveit.poses': 3}, 'moveit.poses'),
        ({'moveit.poses': {'home': {}}}, 'home'),
        ({'moveit.poses': {'home': {'j': 'x'}}}, 'finite number'),
        ({'moveit.poses': {'home': {'j': float('nan')}}}, 'finite number'),
        ({'moveit.poses': {'home': {'j': True}}}, 'finite number'),
        ({'moveit.poses': {'': {'j': 1}}}, 'empty'),
        ({'moveit.poses': {'home': {' ': 1}}}, 'empty joint'),
        ({'moveit.poses': 'poses.yaml', 'moveit.poses.home.j': 1.0}, 'give one'),
        ({'moveit.poses.home.j': 1.0, 'moveit.poses': {'home': {'j': 1.0}}}, 'give one'),
        ({'moveit.joint_states_topic': ' '}, 'joint_states_topic'),
        ({'moveit.joint_states_topic': 3}, 'joint_states_topic'),
        ({'moveit.joint_names': 'elbow_joint'}, 'joint_names'),
        ({'moveit.joint_names': ['elbow_joint', 7]}, 'joint_names'),
        ({'moveit.joint_names': ['elbow_joint', ' ']}, 'joint_names'),
        ({'moveit.joint_names': ['elbow_joint', 'elbow_joint']}, 'joint_names'),
    ],
)
def test_settings_refuse_bad_values(overrides: dict[str, Any], match: str) -> None:
    parameters = dict(PARAMETERS)
    parameters.update(overrides)
    with pytest.raises((ValueError, TypeError), match=match):
        MoveItSettings.from_parameters(parameters)


def test_flattened_poses_as_rcl_gives_them(clock: FakeClock, scheduler: FakeScheduler) -> None:
    parameters = {k: v for k, v in PARAMETERS.items() if k != 'moveit.poses'}
    parameters.update(
        {
            'moveit.poses.home.shoulder_pan_joint': 0.0,
            'moveit.poses.home.elbow_joint': 1,
            'moveit.poses.rest.shoulder_pan_joint': 0.5,
            'moveit.poses.rest.gripper.finger_joint': 0.02,
        }
    )
    rig = Rig(clock, scheduler, parameters)
    assert rig.executor.poses == {
        'home': {'shoulder_pan_joint': 0.0, 'elbow_joint': 1.0},
        'rest': {'shoulder_pan_joint': 0.5, 'gripper.finger_joint': 0.02},
    }
    assert [s.name for s in rig.executor.tasks()] == ['move_to', 'home', 'stop', 'refresh_poses']


def test_unknown_parameters_are_warned_about(
    clock: FakeClock, scheduler: FakeScheduler, caplog: pytest.LogCaptureFixture
) -> None:
    with caplog.at_level(logging.WARNING):
        make_rig(clock, scheduler, **{'moveit.planning_time': 9})
    assert any('moveit.planning_time is not read' in r.message for r in caplog.records)


def test_parse_poses_accepts_a_poses_key_and_refuses_non_mappings() -> None:
    assert parse_poses({'poses': {'home': {'j': 1}}}, 'x') == {'home': {'j': 1.0}}
    assert parse_poses({'home': {'j': 2}}, 'x') == {'home': {'j': 2.0}}
    with pytest.raises(TypeError, match='mapping'):
        parse_poses(['home'], 'x')
    with pytest.raises(ValueError, match='no poses'):
        parse_poses({}, 'x')


# --------------------------------------------------------------------------- poses file


def write_poses(
    path: Path, poses: dict[str, dict[str, float]], *, under_key: bool = False
) -> None:
    lines: list[str] = []
    indent = ''
    if under_key:
        lines.append('poses:')
        indent = '  '
    for name, joints in poses.items():
        lines.append(f'{indent}{name}:')
        for joint, position in joints.items():
            lines.append(f'{indent}  {joint}: {position}')
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def test_poses_from_a_yaml_file_and_refresh(
    clock: FakeClock, scheduler: FakeScheduler, tmp_path: Path, schema_validator: Any
) -> None:
    poses_file = tmp_path / 'poses.yaml'
    write_poses(poses_file, {'home': HOME}, under_key=True)
    rig = make_rig(clock, scheduler, **{'moveit.poses': str(poses_file)})
    assert rig.executor.settings.poses_file == poses_file
    assert rig.executor.poses == {'home': HOME}
    assert [s.name for s in rig.executor.tasks()] == ['move_to', 'home', 'stop', 'refresh_poses']

    write_poses(poses_file, {'home': REST, 'rest': REST, 'tucked': HOME})
    refresh, ack = rig.run('refresh_poses')
    assert ack.code == 'accepted'
    result = rig.result_of(refresh)
    schema_validator.assert_valid('result', result)
    assert result['kind'] == 'query' and result['status'] == 'succeeded'
    assert result['data'] == {
        'source': str(poses_file),
        'poses': ['home', 'rest', 'tucked'],
        'added': ['rest', 'tucked'],
        'removed': [],
        'changed': ['home'],
    }
    assert 'next node start' in result['reason']
    assert rig.executor.poses['home'] == REST
    # The enum the node announced (and validates params against at contract step 8) is the one
    # from start-up, so a new pose name is refused at the node and by HA until the node restarts;
    # the changed 'home' positions and removed poses take effect at once. Only a direct
    # manager.start, as rig.run does, reaches the executor with the new name.
    assert rig.specs['move_to'].params is not None
    assert rig.specs['move_to'].params['properties']['pose']['enum'] == ['home']
    assert protocol.validate_params(rig.specs['move_to'].params, {'pose': 'tucked'}) == [
        "'pose' must be one of ['home']"
    ]
    assert protocol.validate_params(rig.specs['move_to'].params, {'pose': 'home'}) == []
    command, ack = rig.run('move_to', {'pose': 'tucked'})
    assert ack.code == 'accepted'
    assert rig.adapter.last.goal['request']['goal_constraints'][0]['joint_constraints'][0] == {
        'joint_name': 'shoulder_pan_joint',
        'position': 0.0,
        'tolerance_above': JOINT_TOLERANCE_RAD,
        'tolerance_below': JOINT_TOLERANCE_RAD,
        'weight': 1.0,
    }
    rig.adapter.last.complete(1)
    assert rig.result_of(command)['status'] == 'succeeded'

    # A broken file keeps the current poses and reports why.
    poses_file.write_text('home: [1, 2]\n', encoding='utf-8')
    broken, _ = rig.run('refresh_poses')
    result = rig.result_of(broken)
    assert result['status'] == 'aborted' and result['code'] == 'executor_error'
    assert 'keeping the current ones' in result['reason']
    assert result['data']['poses'] == ['home', 'rest', 'tucked']
    poses_file.write_text('home: {j: 1\n', encoding='utf-8')
    unparsable, _ = rig.run('refresh_poses')
    assert 'not valid YAML' in rig.result_of(unparsable)['reason']
    poses_file.unlink()
    missing, _ = rig.run('refresh_poses')
    assert rig.result_of(missing)['status'] == 'aborted'
    assert rig.executor.poses['home'] == REST


def test_missing_or_bad_poses_file_fails_setup(
    clock: FakeClock, scheduler: FakeScheduler, tmp_path: Path
) -> None:
    with pytest.raises(FileNotFoundError):
        make_rig(clock, scheduler, **{'moveit.poses': str(tmp_path / 'absent.yaml')})
    bad = tmp_path / 'bad.yaml'
    bad.write_text('- not\n- a mapping\n', encoding='utf-8')
    with pytest.raises(TypeError, match='mapping'):
        make_rig(clock, scheduler, **{'moveit.poses': str(bad)})
    empty = tmp_path / 'empty.yaml'
    empty.write_text('', encoding='utf-8')
    with pytest.raises(TypeError, match='mapping'):
        make_rig(clock, scheduler, **{'moveit.poses': str(empty)})


# --------------------------------------------------------------------------- goals


def test_goal_dict_shape(rig: Rig) -> None:
    settings = rig.executor.settings
    assert build_goal(settings, 'home', HOME) == {
        'pose': 'home',
        'request': {
            'group_name': 'arm',
            'goal_constraints': [
                {
                    'name': 'home',
                    'joint_constraints': [
                        {
                            'joint_name': joint,
                            'position': position,
                            'tolerance_above': JOINT_TOLERANCE_RAD,
                            'tolerance_below': JOINT_TOLERANCE_RAD,
                            'weight': 1.0,
                        }
                        for joint, position in HOME.items()
                    ],
                }
            ],
            'num_planning_attempts': 3,
            'allowed_planning_time': 2.5,
            'max_velocity_scaling_factor': 0.2,
            'max_acceleration_scaling_factor': 0.3,
        },
        'planning_options': {'plan_only': False},
    }
    rig.run('move_to', {'pose': 'rest'})
    assert rig.adapter.last.goal == build_goal(settings, 'rest', REST)


def test_joint_tolerance_and_timeouts_follow_the_parameters(
    clock: FakeClock, scheduler: FakeScheduler, tmp_path: Path
) -> None:
    """``moveit.joint_tolerance_rad`` reaches every constraint; the two timeouts reach the catalogue."""
    rig = make_rig(
        clock,
        scheduler,
        **{
            'moveit.joint_tolerance_rad': 0.05,
            'moveit.motion_timeout_s': 30,
            'moveit.control_timeout_s': 2,
        },
    )
    timeouts = {spec.name: spec.timeout_s for spec in rig.executor.tasks()}
    assert timeouts == {'move_to': 30.0, 'home': 30.0, 'stop': 2.0, 'refresh_poses': 2.0}
    for spec in rig.executor.tasks():
        assert protocol.validate_task_spec(spec) == [], spec
    rig.run('move_to', {'pose': 'home'})
    constraints = rig.adapter.last.goal['request']['goal_constraints'][0]['joint_constraints']
    assert len(constraints) == len(HOME)
    assert all(c['tolerance_above'] == c['tolerance_below'] == 0.05 for c in constraints)
    rig.adapter.last.complete(1)
    # A refresh rebuilds the catalogue with the same timeouts, not the module defaults.
    poses_file = tmp_path / 'poses.yaml'
    poses_file.write_text('home: {shoulder_pan_joint: 0.1}\n', encoding='utf-8')
    rig.executor._settings = moveit.MoveItSettings.from_parameters(
        {**PARAMETERS, 'moveit.poses': str(poses_file), 'moveit.motion_timeout_s': 30}
    )
    refresh, _ = rig.run('refresh_poses')
    assert rig.result_of(refresh)['status'] == 'succeeded'
    assert {spec.name: spec.timeout_s for spec in rig.executor.tasks()} == {
        'move_to': 30.0,
        'home': 30.0,
        'stop': 5.0,
        'refresh_poses': 5.0,
    }
    # The defaults, with nothing given, are the module constants.
    defaults = {spec.name: spec.timeout_s for spec in moveit.build_catalogue({'home': HOME})}
    assert defaults == {'move_to': 120.0, 'home': 120.0, 'stop': 5.0, 'refresh_poses': 5.0}


def test_unknown_pose_is_rejected_before_any_goal(rig: Rig) -> None:
    # The announced enum refuses it first; the executor's own check is exercised below.
    _, ack = rig.run('move_to', {'pose': 'nowhere'})
    assert ack.code == 'rejected_executor'
    assert ack.reason == "'pose' must be one of ['home', 'rest']"
    rig.executor._poses['tucked'] = REST
    _, ack = rig.run('move_to', {'pose': 'nowhere'})
    assert ack.code == 'rejected_executor'
    rig.executor._catalogue = moveit.build_catalogue(rig.executor.poses)
    _, ack = rig.run('move_to', {'pose': 'nowhere'})
    assert ack.code == 'rejected_executor'
    assert ack.reason == "'pose' must be one of ['home', 'rest', 'tucked']"
    rig.executor._poses.pop('tucked')
    _, ack = rig.run('move_to', {'pose': 'tucked'})
    assert ack.code == 'rejected_executor'
    assert ack.reason == "unknown pose 'tucked'; known poses: home, rest"
    _, ack = rig.run('move_to', {})
    assert ack.code == 'rejected_executor' and "'pose' is required" in ack.reason
    _, ack = rig.run('move_to', {'pose': 7})
    assert ack.code == 'rejected_executor' and 'must be a string' in ack.reason
    assert rig.adapter.goals == [] and rig.manager.state == 'idle'
    # home with the home pose removed underneath it (a refresh could do this).
    rig.executor._poses.pop(HOME_POSE)
    _, ack = rig.run('home')
    assert ack.code == 'rejected_executor' and "unknown pose 'home'" in ack.reason
    assert rig.adapter.goals == []


def test_unknown_task_is_rejected(rig: Rig) -> None:
    request = TaskRequest(
        id=str(uuid.uuid4()),
        task='dance',
        params={},
        issued_at=rig.clock.now_ms(),
        timeout_s=5.0,
        source={'origin': 'ha', 'kind': 'test'},
    )
    with pytest.raises(TaskRejected, match='unknown task'):
        rig.executor.start(
            request,
            SimpleNamespace(task_id='x', feedback=lambda *a: None, finish=lambda r: None),
        )
    assert rig.adapter.goals == []


def test_successful_move_with_feedback(rig: Rig, schema_validator: Any) -> None:
    command, ack = rig.run('move_to', {'pose': 'home'})
    assert ack.code == 'accepted'
    assert rig.manager.state == 'busy' and rig.executor.active_task_id == command.id
    goal = rig.adapter.last
    goal.feed('PLANNING')
    rig.clock.advance(0.5)
    goal.feed('MONITOR')
    rig.clock.advance(0.5)
    goal.feed('IDLE')
    assert [(f['stage'], f['message']) for f in rig.feedbacks] == [
        ('planning', 'move_group state: PLANNING'),
        ('monitor', 'move_group state: MONITOR'),
        ('idle', 'move_group state: IDLE'),
    ]
    assert all(f['progress'] is None for f in rig.feedbacks)
    for feedback in rig.feedbacks:
        schema_validator.assert_valid('feedback', feedback)
    goal.complete(1, 'planned and executed')
    result = rig.result_of(command)
    schema_validator.assert_valid('result', result)
    assert result['status'] == 'succeeded' and result['code'] == 'ok'
    assert result['goal_status'] == 4 and result['kind'] == 'motion'
    assert result['reason'] == "Moved to pose 'home'"
    assert result['data'] == {
        'pose': 'home',
        'group_name': 'arm',
        'action_name': '/move_action',
        'moveit_error_code': 1,
        'moveit_error_name': 'SUCCESS',
        'message': 'planned and executed',
        'goal_status': 4,
        'planning_time_s': 0.25,
        'error_class': 'succeeded',
    }
    assert rig.manager.state == 'idle' and rig.executor.active_task_id is None
    assert rig.scheduler.pending == []
    goal.feed('IDLE')
    assert len(rig.feedbacks) == 3, 'feedback after the finish is dropped'


def test_home_task_moves_to_the_home_pose(rig: Rig) -> None:
    command, ack = rig.run('home')
    assert ack.code == 'accepted'
    assert rig.adapter.last.goal['pose'] == 'home'
    assert rig.adapter.last.goal['request']['goal_constraints'][0]['name'] == 'home'
    rig.adapter.last.complete(1)
    assert rig.result_of(command)['data']['pose'] == 'home'


def test_a_second_motion_is_refused_busy(rig: Rig) -> None:
    rig.run('move_to', {'pose': 'home'})
    _, ack = rig.run('move_to', {'pose': 'rest'})
    assert ack.code == 'rejected_busy'
    assert len(rig.adapter.goals) == 1


# --------------------------------------------------------------------------- the 31 MoveIt codes


EXPECTED: dict[int, tuple[str, str]] = {
    1: ('succeeded', 'ok'),
    -1: ('aborted', 'planning_failed'),
    -2: ('aborted', 'planning_failed'),
    -3: ('aborted', 'planning_failed'),
    -31: ('aborted', 'planning_failed'),
    -10: ('aborted', 'collision'),
    -11: ('aborted', 'planning_failed'),
    -12: ('aborted', 'collision'),
    -13: ('aborted', 'planning_failed'),
    -14: ('aborted', 'planning_failed'),
    -4: ('aborted', 'control_failed'),
    -6: ('aborted', 'timeout'),
    -7: ('aborted', 'preempted'),
    -15: ('aborted', 'invalid_goal'),
    -16: ('aborted', 'invalid_goal'),
    -17: ('aborted', 'invalid_goal'),
    -18: ('aborted', 'invalid_goal'),
    -19: ('aborted', 'invalid_goal'),
    -26: ('aborted', 'invalid_goal'),
    -27: ('aborted', 'invalid_goal'),
    -28: ('aborted', 'invalid_goal'),
    -5: ('aborted', UNAVAILABLE_CODE),
    -22: ('aborted', UNAVAILABLE_CODE),
    -23: ('aborted', UNAVAILABLE_CODE),
    -24: ('aborted', UNAVAILABLE_CODE),
    -25: ('aborted', UNAVAILABLE_CODE),
    -21: ('aborted', 'executor_error'),
    -29: ('aborted', 'executor_error'),
    -30: ('aborted', 'executor_error'),
    0: ('aborted', 'executor_error'),
    99999: ('aborted', 'executor_error'),
}
EXPECTED_CLASS: dict[str, str] = {
    'ok': 'succeeded',
    'planning_failed': 'planning_failed',
    'collision': 'planning_failed',
    'control_failed': 'execution_failed',
    'timeout': 'timed_out',
    'preempted': 'preempted',
    'invalid_goal': 'rejected',
    UNAVAILABLE_CODE: 'unavailable',
    'executor_error': 'internal_error',
}


def test_the_table_covers_every_moveit_code_once() -> None:
    assert set(EXPECTED) == set(MOVEIT_ERROR_NAMES) == set(MOVEIT_ERROR_CLASSES)
    assert len(MOVEIT_ERROR_NAMES) == 31
    assert {-8, -9, -20}.isdisjoint(MOVEIT_ERROR_NAMES), 'unused in moveit_msgs 2.6.0'
    assert MOVEIT_ERROR_NAMES[1] == 'SUCCESS' and MOVEIT_ERROR_NAMES[-31] == 'NO_IK_SOLUTION'
    for code in EXPECTED.values():
        assert protocol.is_valid_result_code(code[1])


@pytest.mark.parametrize('val', sorted(EXPECTED), ids=lambda v: f'{v}_{MOVEIT_ERROR_NAMES[v]}')
def test_every_moveit_code_maps_as_the_research_proposes(
    rig: Rig, schema_validator: Any, val: int
) -> None:
    status, code = EXPECTED[val]
    mapped = map_error_code(val, message='detail')
    assert (mapped.status, mapped.code) == (status, code)
    assert mapped.error_class == EXPECTED_CLASS[code] == MOVEIT_ERROR_CLASSES[val]
    assert mapped.moveit_name == MOVEIT_ERROR_NAMES[val]

    command, _ = rig.run('move_to', {'pose': 'home'})
    rig.adapter.last.complete(val, 'detail')
    result = rig.result_of(command)
    schema_validator.assert_valid('result', result)
    assert (result['status'], result['code']) == (status, code)
    assert result['data']['moveit_error_code'] == val
    assert result['data']['moveit_error_name'] == MOVEIT_ERROR_NAMES[val]
    assert result['data']['message'] == 'detail'
    assert result['data']['error_class'] == EXPECTED_CLASS[code]
    if val != 1:
        assert f'MoveIt {MOVEIT_ERROR_NAMES[val]} ({val})' in result['reason']
        assert 'detail' in result['reason']
    assert rig.manager.state == 'idle', 'a failed move alone never puts the robot into error'
    assert rig.states[-1]['error'] is None


def test_unknown_values_are_internal_errors() -> None:
    for val in (-8, -9, -20, 12345):
        mapped = map_error_code(val)
        assert (mapped.status, mapped.code, mapped.moveit_name) == (
            'aborted',
            'executor_error',
            'UNKNOWN',
        )
        assert mapped.reason == f'MoveIt UNKNOWN ({val})'


def test_preempted_by_another_client_is_not_a_cancel(rig: Rig) -> None:
    command, _ = rig.run('move_to', {'pose': 'home'})
    rig.adapter.last.complete(-7)
    result = rig.result_of(command)
    assert result['status'] == 'aborted' and result['code'] == 'preempted'
    assert 'another move_group client' in result['reason']


# --------------------------------------------------------------------------- cancel paths


def test_cancel_waits_for_the_hold_confirmation(rig: Rig, schema_validator: Any) -> None:
    command, _ = rig.run('move_to', {'pose': 'home'})
    goal = rig.adapter.last
    ack = rig.cancel()
    assert ack.code == 'accepted'
    assert goal.cancelled and rig.results == [], 'hold_on_cancel waits for move_group'
    assert rig.manager.state == 'busy'
    goal.complete(-7, 'preempted')
    result = rig.result_of(command)
    schema_validator.assert_valid('result', result)
    assert result['status'] == 'cancelled' and result['code'] == 'cancelled'
    assert result['goal_status'] == 5 and result['reason'] == 'Cancelled by request'
    assert result['data']['moveit_error_code'] == -7
    assert result['data']['moveit_error_name'] == 'PREEMPTED'
    assert result['data']['hold_confirmed'] is True
    assert rig.manager.state == 'idle' and rig.scheduler.pending == []


def test_cancel_without_hold_finishes_at_once(
    clock: FakeClock, scheduler: FakeScheduler, schema_validator: Any
) -> None:
    rig = make_rig(clock, scheduler, **{'moveit.hold_on_cancel': False})
    command, _ = rig.run('move_to', {'pose': 'home'})
    goal = rig.adapter.last
    assert rig.cancel(command.id).code == 'accepted'
    assert goal.cancelled
    result = rig.result_of(command)
    schema_validator.assert_valid('result', result)
    assert result['status'] == 'cancelled' and result['code'] == 'cancelled'
    assert result['data']['hold_confirmed'] is False
    assert rig.manager.state == 'idle'
    goal.complete(-7)
    assert len(rig.results) == 1, 'the late confirmation is dropped'


def test_cancel_that_loses_the_race_reports_success(rig: Rig) -> None:
    command, _ = rig.run('move_to', {'pose': 'home'})
    rig.cancel()
    rig.adapter.last.complete(1)
    result = rig.result_of(command)
    assert result['status'] == 'succeeded' and result['code'] == 'ok'


def test_cancelled_goal_status_with_an_odd_code_is_still_cancelled(rig: Rig) -> None:
    command, _ = rig.run('move_to', {'pose': 'home'})
    rig.cancel()
    rig.adapter.last.complete(99999, status=STATUS_CANCELED)
    result = rig.result_of(command)
    assert result['status'] == 'cancelled' and result['code'] == 'cancelled'
    assert result['data']['moveit_error_name'] == 'FAILURE'


def test_cancel_for_an_unknown_id_is_ignored(rig: Rig, caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.INFO):
        rig.executor.cancel('not-running')
    assert any('no goal in flight' in r.message for r in caplog.records)


def test_timeout_without_confirmation_is_forced_by_the_bridge(
    rig: Rig, schema_validator: Any
) -> None:
    command, _ = rig.run('move_to', {'pose': 'home'}, timeout_s=10)
    goal = rig.adapter.last
    rig.clock.advance(10)
    assert goal.cancelled and rig.results == []
    rig.clock.advance(5)
    result = rig.result_of(command)
    schema_validator.assert_valid('result', result)
    assert result['status'] == 'aborted' and result['code'] == 'timeout'
    assert rig.manager.state == 'error'
    # The executor still tracks the goal until move_group answers.
    assert rig.executor.active_task_id == command.id
    # The operator presses stop while move_group has still not answered: a second cancel goes
    # out and the error stays until move_group confirms, so idle always means the arm is holding.
    stop, _ = rig.run('stop')
    stopped = rig.result_of(stop)
    schema_validator.assert_valid('result', stopped)
    assert stopped['status'] == 'succeeded'
    assert stopped['data'] == {'stopped': [command.id], 'awaiting_confirmation': True}
    assert 'clears when move_group confirms' in stopped['reason']
    assert goal.cancel_requests == 2
    assert rig.manager.state == 'error' and rig.manager.error is not None
    assert rig.executor.active_task_id == command.id
    # The late confirmation ends the executor's bookkeeping, the manager drops the second finish
    # for the already forced task, and the error clears as stop promised.
    goal.complete(-7)
    assert len(rig.results) == 2, 'the forced result and the stop result; nothing more'
    assert rig.executor.active_task_id is None
    assert rig.manager.state == 'idle' and rig.manager.error is None
    again, _ = rig.run('stop')
    assert rig.result_of(again)['reason'] == 'Nothing was running'
    assert rig.result_of(again)['data'] == {'stopped': [], 'awaiting_confirmation': False}
    assert rig.manager.state == 'idle'


def test_cancel_that_raises_leads_to_executor_unresponsive(rig: Rig) -> None:
    command, _ = rig.run('move_to', {'pose': 'home'})
    rig.adapter.cancel_raises = RuntimeError('no cancel service')
    assert rig.cancel().code == 'accepted'
    rig.clock.advance(5)
    result = rig.result_of(command)
    assert result['status'] == 'aborted' and result['code'] == 'executor_unresponsive'
    assert rig.manager.state == 'error'


# --------------------------------------------------------------------------- stop and stop_and_hold


def test_stop_preempts_the_move_and_clears_the_error(rig: Rig, schema_validator: Any) -> None:
    running, _ = rig.run('move_to', {'pose': 'home'})
    goal = rig.adapter.last
    rig.manager.set_error('executor_error', 'test')
    assert rig.manager.state == 'error'
    stop, ack = rig.run('stop')
    assert ack.code == 'accepted'
    stopped = rig.result_of(stop)
    schema_validator.assert_valid('result', stopped)
    assert stopped['kind'] == 'control' and stopped['status'] == 'succeeded'
    assert stopped['reason'] == (
        'Stopped 1 task(s); the robot error, if any, clears when move_group confirms the goal '
        'ended'
    )
    assert stopped['data'] == {'stopped': [running.id], 'awaiting_confirmation': True}
    assert goal.cancelled
    # With hold_on_cancel the error clears only once move_group confirms the hold, so an idle
    # robot always means the arm is holding.
    assert rig.manager.error is not None and rig.manager.state == 'error'
    assert rig.manager.active_task_id == running.id, 'the move ends when move_group confirms'
    goal.complete(-7)
    preempted = rig.result_of(running)
    schema_validator.assert_valid('result', preempted)
    assert preempted['status'] == 'cancelled' and preempted['code'] == 'preempted'
    assert preempted['reason'] == "Stopped by task 'stop'"
    assert preempted['data']['by'] == 'stop' and preempted['data']['hold_confirmed'] is True
    assert rig.manager.state == 'idle' and rig.manager.error is None
    assert rig.states[-1]['state'] == 'idle' and rig.states[-1]['error'] is None
    idle_stop, _ = rig.run('stop')
    assert rig.result_of(idle_stop)['reason'] == 'Nothing was running'
    assert rig.result_of(idle_stop)['data'] == {'stopped': [], 'awaiting_confirmation': False}


def test_stop_without_hold_ends_the_move_at_once(
    clock: FakeClock, scheduler: FakeScheduler
) -> None:
    rig = make_rig(clock, scheduler, **{'moveit.hold_on_cancel': False})
    running, _ = rig.run('move_to', {'pose': 'home'})
    rig.manager.set_error('executor_error', 'test')
    stop, _ = rig.run('stop')
    assert rig.result_of(stop)['data'] == {'stopped': [running.id], 'awaiting_confirmation': False}
    assert rig.result_of(stop)['reason'] == 'Stopped 1 task(s)'
    preempted = rig.result_of(running)
    assert preempted['status'] == 'cancelled' and preempted['code'] == 'preempted'
    assert preempted['data']['hold_confirmed'] is False
    assert rig.manager.state == 'idle' and rig.manager.error is None, 'cleared at once'


def test_stop_and_hold_cancels_the_goal_and_ends_the_task_inhibited(
    rig: Rig, schema_validator: Any
) -> None:
    running, _ = rig.run('move_to', {'pose': 'home'})
    goal = rig.adapter.last
    rig.manager.stop_and_hold('HA heartbeat lost')
    assert goal.cancelled
    result = rig.result_of(running)
    schema_validator.assert_valid('result', result)
    assert result['status'] == 'cancelled' and result['code'] == 'inhibited'
    # The manager publishes its own override for a stop-and-hold, data included.
    assert result['reason'] == 'HA heartbeat lost' and result['data'] == {}
    assert rig.manager.state == 'idle' and rig.manager.error is None
    assert rig.executor.active_task_id is None
    goal.complete(-7)
    assert len(rig.results) == 1, 'the confirmation is dropped'
    rig.manager.stop_and_hold('again, with nothing running')
    assert len(rig.results) == 1


def test_stop_and_hold_drops_a_pending_error_clear(
    rig: Rig, caplog: pytest.LogCaptureFixture
) -> None:
    running, _ = rig.run('move_to', {'pose': 'home'})
    goal = rig.adapter.last
    rig.manager.set_error('executor_error', 'test')
    rig.run('stop')
    assert rig.manager.error is not None, 'the clear waits for move_group'
    with caplog.at_level(logging.WARNING):
        rig.manager.stop_and_hold('HA heartbeat lost')
    assert rig.result_of(running)['code'] == 'inhibited'
    assert any(
        "error clear pending from 'stop' is dropped because stop_and_hold" in r.message
        for r in caplog.records
    )
    goal.complete(-7)
    assert rig.manager.error is not None, 'the dropped clear is not applied by the late outcome'
    again, _ = rig.run('stop')
    assert rig.result_of(again)['data'] == {'stopped': [], 'awaiting_confirmation': False}
    assert rig.manager.error is None and rig.manager.state == 'idle'


def test_shutdown_drops_a_pending_error_clear(rig: Rig, caplog: pytest.LogCaptureFixture) -> None:
    rig.run('move_to', {'pose': 'home'})
    goal = rig.adapter.last
    rig.run('stop')
    assert goal.cancel_requests == 1
    with caplog.at_level(logging.WARNING):
        rig.executor.shutdown()
    assert goal.cancel_requests == 2 and rig.adapter.closed
    assert any(
        "error clear pending from 'stop' is dropped because shutdown" in r.message
        for r in caplog.records
    )


# --------------------------------------------------------------------------- availability and health


def test_server_absent_gives_unavailable_and_health_error(rig: Rig, schema_validator: Any) -> None:
    assert rig.executor.health() == ('ok', '')
    assert rig.manager.poll_health().level == 'ok'
    rig.adapter.available = False
    health = rig.executor.health()
    assert health == ('error', 'move_group action server /move_action is not available')
    assert rig.manager.poll_health().level == 'error'
    assert rig.states[-1]['executor_health']['level'] == 'error'
    command, ack = rig.run('move_to', {'pose': 'home'})
    assert ack.code == 'accepted'
    result = rig.result_of(command)
    schema_validator.assert_valid('result', result)
    assert result['status'] == 'aborted' and result['code'] == UNAVAILABLE_CODE
    assert result['reason'] == 'move_group action server /move_action is not available'
    assert result['data']['moveit_error_code'] is None
    assert rig.adapter.goals == [] and rig.manager.state == 'idle'
    rig.adapter.available = True
    assert rig.manager.poll_health().level == 'ok'


def test_goal_rejected_by_move_group_gives_unavailable(rig: Rig) -> None:
    command, _ = rig.run('move_to', {'pose': 'home'})
    rig.adapter.last.reject()
    result = rig.result_of(command)
    assert result['status'] == 'aborted' and result['code'] == UNAVAILABLE_CODE
    assert result['reason'] == 'move_group rejected the goal'
    assert rig.manager.state == 'idle'


def test_cancel_before_acceptance_then_a_rejected_goal_is_cancelled(
    rig: Rig, schema_validator: Any
) -> None:
    command, _ = rig.run('move_to', {'pose': 'home'})
    goal = rig.adapter.last
    assert rig.cancel().code == 'accepted'
    assert goal.cancelled and rig.results == []
    goal.reject()
    result = rig.result_of(command)
    schema_validator.assert_valid('result', result)
    assert result['status'] == 'cancelled' and result['code'] == 'cancelled'
    assert result['reason'] == 'Cancelled by request'
    assert result['data']['hold_confirmed'] is True, 'nothing was executing'
    assert result['data']['message'] == 'move_group rejected the goal'
    assert result['data']['moveit_error_code'] is None
    assert rig.manager.state == 'idle' and rig.executor.active_task_id is None


def test_result_without_a_code_is_an_executor_error(rig: Rig) -> None:
    command, _ = rig.run('move_to', {'pose': 'home'})
    rig.adapter.last.finish(GoalOutcome(status=STATUS_ABORTED, detail='no code in the result'))
    result = rig.result_of(command)
    assert result['status'] == 'aborted' and result['code'] == 'executor_error'
    assert result['reason'] == 'no code in the result' and result['data']['goal_status'] == 6


def test_send_goal_raising_is_an_executor_error(rig: Rig) -> None:
    rig.adapter.send_raises = RuntimeError('boom')
    command, ack = rig.run('move_to', {'pose': 'home'})
    assert ack.code == 'accepted'
    result = rig.result_of(command)
    assert result['status'] == 'aborted' and result['code'] == 'executor_error'
    assert 'RuntimeError: boom' in result['reason'] and rig.manager.state == 'idle'


def test_health_when_the_check_raises_or_before_setup(rig: Rig) -> None:
    def broken() -> bool:
        raise OSError('context gone')

    rig.adapter.server_available = broken  # type: ignore[method-assign]
    level, message = rig.executor.health()
    assert level == 'error' and 'OSError: context gone' in message
    assert MoveItExecutor().health() == ('error', 'moveit executor is not set up')


def test_shutdown_cancels_the_goal_in_flight_and_closes_the_adapter(rig: Rig) -> None:
    rig.run('move_to', {'pose': 'home'})
    goal = rig.adapter.last
    rig.executor.shutdown()
    assert goal.cancelled and rig.adapter.closed
    assert rig.executor.active_task_id is None
    rig.executor.shutdown()
    assert goal.cancel_requests == 1


def test_a_new_goal_cancels_a_stale_one_the_bridge_already_forced(
    rig: Rig, caplog: pytest.LogCaptureFixture
) -> None:
    first, _ = rig.run('move_to', {'pose': 'home'}, timeout_s=1)
    stale = rig.adapter.last
    rig.clock.advance(6)
    assert rig.result_of(first)['code'] == 'timeout'
    # stop defers its error clear to move_group's confirmation, which never comes here, so the
    # robot stays in error and no motion can start; clearing the error by another route (the
    # manager's own API) is what lets the defensive stale-goal path run at all.
    rig.run('stop')
    assert rig.manager.state == 'error'
    rig.manager.clear_error()
    assert rig.manager.state == 'idle'
    with caplog.at_level(logging.WARNING):
        second, _ = rig.run('move_to', {'pose': 'rest'})
    assert stale.cancel_requests >= 2 and len(rig.adapter.goals) == 2
    assert any(
        "error clear pending from 'stop' is dropped because a newer task" in r.message
        for r in caplog.records
    )
    rig.adapter.last.complete(1)
    assert rig.result_of(second)['status'] == 'succeeded'
    stale.complete(-7)
    assert rig.manager.state == 'idle' and len(rig.results) == 3, 'the stale outcome is dropped'


# --------------------------------------------------------------------------- joint telemetry

JOINT_STATE_NAMES = [
    'gripper_finger_joint',
    'elbow_joint',
    'shoulder_lift_joint',
    'shoulder_pan_joint',
]


def test_joints_come_from_the_first_pose_in_radians_without_limits(
    rig: Rig, schema_validator: Any
) -> None:
    info = rig.executor.joints()
    assert info == JointInfo(
        names=('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint'), unit='rad', limits=None
    )
    assert rig.executor.joint_names == ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint')
    assert protocol.validate_joint_info(info) == []
    assert rig.adapter.joint_state_subscriptions[0][0] == '/joint_states'
    assert len(rig.adapter.joint_state_subscriptions) == 1
    announce = Announce(
        robot_id=ROBOT,
        name='Simulated arm',
        session=SESSION,
        instance=INSTANCE,
        tasks=rig.executor.tasks(),
        joints=AnnounceJoints.from_joint_info(info, 2.0),
        ts=0,
    )
    schema_validator.assert_valid('announce', announce.to_payload())


def test_joint_states_are_forwarded_in_announce_order_with_velocities(rig: Rig) -> None:
    assert rig.joints == [], 'nothing reported until the first JointState'
    rig.adapter.feed_joint_state(JOINT_STATE_NAMES, [0.02, 1.5, -1.5, 0.25], [0.0, 0.1, -0.1, 0.5])
    assert rig.joints == [((0.25, -1.5, 1.5), (0.5, -0.1, 0.1))]
    rig.adapter.feed_joint_state(JOINT_STATE_NAMES, [0.02, 1.6, -1.4, 0.3], [])
    assert rig.joints[-1] == ((0.3, -1.4, 1.6), None), 'an empty velocity array means none'
    rig.adapter.feed_joint_state(JOINT_STATE_NAMES, [0.02, 1.6, -1.4, 0.3], None)
    assert rig.joints[-1] == ((0.3, -1.4, 1.6), None)
    rig.adapter.feed_joint_state(JOINT_STATE_NAMES, [0, 1, -1, 2], [1, 2, 3])
    assert rig.joints[-1] == ((2.0, -1.0, 1.0), None), 'fewer velocities than names: dropped'
    assert rig.executor.joint_states_seen == 4 and rig.executor.joint_states_skipped == 0


def test_joint_state_lacking_a_configured_joint_is_skipped_and_warned_once(
    rig: Rig, caplog: pytest.LogCaptureFixture
) -> None:
    with caplog.at_level(logging.WARNING):
        rig.adapter.feed_joint_state(['shoulder_pan_joint', 'elbow_joint'], [0.1, 0.2])
        rig.adapter.feed_joint_state(['shoulder_pan_joint'], [0.1])
        rig.adapter.feed_joint_state(JOINT_STATE_NAMES, [0.0, 1.0])
    assert rig.joints == [] and rig.executor.joint_states_skipped == 3
    warnings = [r for r in caplog.records if 'lacks' in r.getMessage()]
    assert len(warnings) == 1 and 'shoulder_lift_joint' in warnings[0].getMessage()
    rig.adapter.feed_joint_state(JOINT_STATE_NAMES, [0.02, 1.5, -1.5, 0.25])
    assert rig.joints == [((0.25, -1.5, 1.5), None)]


def test_joint_names_parameter_overrides_the_pose_order_and_is_announced_cleaned(
    clock: FakeClock, scheduler: FakeScheduler
) -> None:
    rig = make_rig(
        clock,
        scheduler,
        **{
            'moveit.joint_names': ['elbow_joint', 'Gripper.Finger'],
            'moveit.joint_states_topic': '/js',
        },
    )
    assert rig.executor.joint_names == ('elbow_joint', 'Gripper.Finger')
    assert rig.executor.joints() == JointInfo(('elbow_joint', 'gripper_finger'), 'rad', None)
    assert rig.adapter.joint_state_subscriptions[0][0] == '/js'
    rig.adapter.feed_joint_state(['Gripper.Finger', 'elbow_joint'], [0.01, 2.0])
    assert rig.joints == [((2.0, 0.01), None)]


def test_joint_names_that_cannot_be_announced_switch_the_telemetry_off(
    clock: FakeClock, scheduler: FakeScheduler, caplog: pytest.LogCaptureFixture
) -> None:
    with caplog.at_level(logging.WARNING):
        rig = make_rig(clock, scheduler, **{'moveit.joint_names': ['a.b', 'a_b']})
    assert rig.executor.joints() is None and rig.executor.joint_names == ()
    assert rig.adapter.joint_state_subscriptions == []
    assert any('cannot all be announced' in r.getMessage() for r in caplog.records)
    rig.adapter.feed_joint_state(['a.b', 'a_b'], [1.0, 2.0])
    assert rig.joints == []


@pytest.mark.parametrize(
    ('raw', 'expected'),
    [
        (['shoulder_pan_joint'], ('shoulder_pan_joint',)),
        (['Gripper.Finger', 'wrist 1'], ('gripper_finger', 'wrist_1')),
        (['panda_joint1', '2nd_joint'], ('panda_joint1', 'nd_joint')),
        (['x' * 40], ('x' * 32,)),
        ([], None),
        (['j'] * 33, None),
        (['a.b', 'a_b'], None),
        (['123'], None),
        (['___'], None),
    ],
)
def test_announce_joint_names(raw: list[str], expected: tuple[str, ...] | None) -> None:
    assert announce_joint_names(raw) == expected
    if expected is not None:
        assert protocol.validate_joint_info(JointInfo(expected, 'rad')) == []


def test_an_adapter_without_joint_states_still_announces_the_joints(
    clock: FakeClock, scheduler: FakeScheduler, caplog: pytest.LogCaptureFixture
) -> None:
    with caplog.at_level(logging.WARNING):
        rig = Rig(clock, scheduler, dict(PARAMETERS), adapter=AdapterWithoutJointStates())
    assert rig.executor.joints() is not None
    assert any('subscribe_joint_states' in r.getMessage() for r in caplog.records)
    command, _ = rig.run('move_to', {'pose': 'home'})
    rig.adapter.inner.last.complete(1)
    assert rig.result_of(command)['status'] == 'succeeded'


class FakeNode:
    """The two subscription methods of ``rclpy.node.Node`` the adapter uses."""

    def __init__(self) -> None:
        self.subscriptions: list[tuple[Any, str, Callable[[Any], None], Any]] = []
        self.destroyed: list[Any] = []

    def create_subscription(
        self, message_cls: Any, topic: str, callback: Callable[[Any], None], qos: Any
    ) -> Any:
        subscription = (message_cls, topic, callback, qos)
        self.subscriptions.append(subscription)
        return subscription

    def destroy_subscription(self, subscription: Any) -> None:
        self.destroyed.append(subscription)


def test_rclpy_adapter_subscribes_with_the_sensor_qos_and_unpacks_joint_state(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    joint_state_cls = type('JointState', (), {})
    sensor_qos = object()
    sensor_msgs = types.ModuleType('sensor_msgs')
    sensor_msgs_msg = types.ModuleType('sensor_msgs.msg')
    sensor_msgs_msg.JointState = joint_state_cls  # type: ignore[attr-defined]
    rclpy_module = types.ModuleType('rclpy')
    rclpy_qos = types.ModuleType('rclpy.qos')
    rclpy_qos.qos_profile_sensor_data = sensor_qos  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, 'sensor_msgs', sensor_msgs)
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', sensor_msgs_msg)
    monkeypatch.setitem(sys.modules, 'rclpy', rclpy_module)
    monkeypatch.setitem(sys.modules, 'rclpy.qos', rclpy_qos)

    adapter = moveit.RclpyMoveGroupAdapter.__new__(moveit.RclpyMoveGroupAdapter)
    node = FakeNode()
    adapter._node = node
    adapter._logger = logging.getLogger('test')
    adapter._joint_states = None
    adapter._client = SimpleNamespace(destroy=lambda: None)
    received: list[tuple[Sequence[str], Sequence[float], Sequence[float] | None]] = []
    adapter.subscribe_joint_states(
        '/joint_states',
        lambda names, positions, velocities: received.append((names, positions, velocities)),
    )
    ((message_cls, topic, on_message, qos),) = node.subscriptions
    assert message_cls is joint_state_cls and topic == '/joint_states' and qos is sensor_qos
    on_message(SimpleNamespace(name=['a', 'b'], position=[1, 2.5], velocity=[0.1, 0.2]))
    on_message(SimpleNamespace(name=['a', 'b'], position=[1.0, 2.0], velocity=[]))
    assert received == [
        (['a', 'b'], [1.0, 2.5], [0.1, 0.2]),
        (['a', 'b'], [1.0, 2.0], None),
    ]
    adapter.subscribe_joint_states('/other', lambda *args: None)
    assert node.destroyed == [node.subscriptions[0]], 'a second subscribe replaces the first'
    adapter.close()
    assert node.destroyed[-1] == node.subscriptions[1] and adapter._joint_states is None


# --------------------------------------------------------------------------- the rclpy goal handle


class FakeFuture:
    """The slice of ``rclpy.task.Future`` the adapter uses."""

    def __init__(self) -> None:
        self._callbacks: list[Callable[[FakeFuture], None]] = []
        self._result: Any = None
        self._error: Exception | None = None
        self.done = False

    def add_done_callback(self, callback: Callable[[FakeFuture], None]) -> None:
        if self.done:
            callback(self)
        else:
            self._callbacks.append(callback)

    def result(self) -> Any:
        if self._error is not None:
            raise self._error
        return self._result

    def set_result(self, value: Any) -> None:
        self._result, self.done = value, True
        for callback in self._callbacks:
            callback(self)

    def set_exception(self, error: Exception) -> None:
        self._error, self.done = error, True
        for callback in self._callbacks:
            callback(self)


class FakeClientGoalHandle:
    def __init__(self, accepted: bool = True) -> None:
        self.accepted = accepted
        self.result_future = FakeFuture()
        self.cancel_future = FakeFuture()
        self.cancel_calls = 0

    def get_result_async(self) -> FakeFuture:
        return self.result_future

    def cancel_goal_async(self) -> FakeFuture:
        self.cancel_calls += 1
        return self.cancel_future


def wrapped_result(status: int, val: int, message: str = '', planning_time: float = 0.5) -> Any:
    return SimpleNamespace(
        status=status,
        result=SimpleNamespace(
            error_code=SimpleNamespace(val=val, message=message, source=''),
            planning_time=planning_time,
        ),
    )


def test_rclpy_goal_handle_reports_the_result_once() -> None:
    handle = moveit._RclpyGoalHandle(logging.getLogger('test'))
    outcomes: list[GoalOutcome] = []
    handle.add_done(outcomes.append)
    goal_future = FakeFuture()
    handle.attach(goal_future)
    client_handle = FakeClientGoalHandle()
    goal_future.set_result(client_handle)
    assert outcomes == []
    client_handle.result_future.set_result(wrapped_result(STATUS_ABORTED, -12, 'collides'))
    assert outcomes == [
        GoalOutcome(
            accepted=True, status=6, error_code=-12, message='collides', planning_time_s=0.5
        )
    ]
    late: list[GoalOutcome] = []
    handle.add_done(late.append)
    assert late == outcomes, 'a callback added after the end is called at once'
    handle.cancel()
    assert client_handle.cancel_calls == 0, 'nothing to cancel after the end'


def test_rclpy_goal_handle_cancel_before_and_after_acceptance() -> None:
    logger = logging.getLogger('test')
    early = moveit._RclpyGoalHandle(logger)
    goal_future = FakeFuture()
    early.attach(goal_future)
    early.cancel()
    client_handle = FakeClientGoalHandle()
    goal_future.set_result(client_handle)
    assert client_handle.cancel_calls == 1, 'a cancel before acceptance is sent on acceptance'
    client_handle.cancel_future.set_result(SimpleNamespace(return_code=0))

    late = moveit._RclpyGoalHandle(logger)
    goal_future = FakeFuture()
    late.attach(goal_future)
    client_handle = FakeClientGoalHandle()
    goal_future.set_result(client_handle)
    late.cancel()
    late.cancel()
    assert client_handle.cancel_calls == 2
    client_handle.cancel_future.set_exception(RuntimeError('cancel service gone'))
    outcomes: list[GoalOutcome] = []
    late.add_done(outcomes.append)
    client_handle.result_future.set_result(wrapped_result(STATUS_CANCELED, -7))
    assert outcomes[0].status == 5 and outcomes[0].error_code == -7


def test_rclpy_goal_handle_rejection_and_failures() -> None:
    logger = logging.getLogger('test')
    rejected = moveit._RclpyGoalHandle(logger)
    outcomes: list[GoalOutcome] = []
    rejected.add_done(outcomes.append)
    goal_future = FakeFuture()
    rejected.attach(goal_future)
    goal_future.set_result(FakeClientGoalHandle(accepted=False))
    assert outcomes == [GoalOutcome(accepted=False, detail='move_group rejected the goal')]

    raised = moveit._RclpyGoalHandle(logger)
    outcomes = []
    raised.add_done(outcomes.append)
    goal_future = FakeFuture()
    raised.attach(goal_future)
    goal_future.set_exception(RuntimeError('no server'))
    assert outcomes[0].accepted is False and 'RuntimeError: no server' in outcomes[0].detail

    no_code = moveit._RclpyGoalHandle(logger)
    outcomes = []
    no_code.add_done(outcomes.append)
    goal_future = FakeFuture()
    no_code.attach(goal_future)
    client_handle = FakeClientGoalHandle()
    goal_future.set_result(client_handle)
    client_handle.result_future.set_result(SimpleNamespace(status=6, result=SimpleNamespace()))
    assert outcomes[0].accepted and outcomes[0].error_code is None
    assert outcomes[0].detail == 'the result carries no MoveItErrorCodes value'

    result_raises = moveit._RclpyGoalHandle(logger)
    outcomes = []
    result_raises.add_done(outcomes.append)
    goal_future = FakeFuture()
    result_raises.attach(goal_future)
    client_handle = FakeClientGoalHandle()
    goal_future.set_result(client_handle)
    client_handle.result_future.set_exception(RuntimeError('result lost'))
    assert outcomes[0].error_code is None and 'result lost' in outcomes[0].detail


class _Msg(SimpleNamespace):
    """Attribute bag standing in for a generated ROS message class."""


def _goal_message_class() -> Any:
    def goal() -> _Msg:
        return _Msg(
            request=_Msg(
                group_name='',
                num_planning_attempts=0,
                allowed_planning_time=0.0,
                max_velocity_scaling_factor=0.0,
                max_acceleration_scaling_factor=0.0,
                goal_constraints=[],
            ),
            planning_options=_Msg(plan_only=True),
        )

    return SimpleNamespace(Goal=goal)


def test_rclpy_adapter_builds_the_goal_message() -> None:
    adapter = moveit.RclpyMoveGroupAdapter.__new__(moveit.RclpyMoveGroupAdapter)
    adapter._move_group = _goal_message_class()
    adapter._constraints_cls = lambda: _Msg(name='', joint_constraints=[])
    adapter._joint_constraint_cls = lambda: _Msg(
        joint_name='', position=0.0, tolerance_above=0.0, tolerance_below=0.0, weight=0.0
    )
    settings = MoveItSettings.from_parameters(PARAMETERS)
    message = adapter.build_message(build_goal(settings, 'home', HOME))
    assert message.request.group_name == 'arm'
    assert message.request.num_planning_attempts == 3
    assert message.request.allowed_planning_time == 2.5
    assert message.request.max_velocity_scaling_factor == 0.2
    assert message.request.max_acceleration_scaling_factor == 0.3
    assert message.planning_options.plan_only is False
    (constraints,) = message.request.goal_constraints
    assert constraints.name == 'home'
    assert [(c.joint_name, c.position) for c in constraints.joint_constraints] == list(
        HOME.items()
    )
    assert all(
        (c.tolerance_above, c.tolerance_below, c.weight) == (0.01, 0.01, 1.0)
        for c in constraints.joint_constraints
    )
