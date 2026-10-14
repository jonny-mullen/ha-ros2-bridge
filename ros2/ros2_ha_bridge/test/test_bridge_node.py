# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""In-process rclpy tests for :mod:`ros2_ha_bridge.bridge_node` against the fake broker.

These need a real rclpy (they run under ``colcon test`` in the Jazzy
container) and skip themselves on a machine without it. Each test owns a
private rclpy context and a ``SingleThreadedExecutor`` spun from the test
thread, which therefore plays the node's main thread; the node's paho client
is a ``FakeMqttClient`` from ``fakes.py`` bound to a ``FakeBroker``, delivering
from a real second thread exactly as paho's network thread would, so the
deque, the guard condition and the drain are exercised as in production. The
fake executor is loaded by dotted path because the package's entry points are
only registered once it is installed.

Every payload the node publishes is validated against the vendored contract
schemas. Wire order: a motion task's ``busy`` state is published inside
``TaskManager.start`` and may precede the ack on the wire (recorded decision
in ``docs/decisions.md``); the tests assert the order Home Assistant relies on
(ack, feedback, result, then the ``idle`` state) and that ``busy`` precedes
the feedback.

The node module imports rclpy at module level, so it is reached through
:class:`_Lazy` and imported on first use, after ``pytest.importorskip`` has
decided whether this file runs at all.
"""

from __future__ import annotations

import importlib
import json
import logging
import time
import uuid
from collections.abc import Callable, Iterator
from typing import Any

import pytest

from ros2_ha_bridge import __version__, parameters, protocol
from ros2_ha_bridge import mqtt_link as ml
from ros2_ha_bridge.parameters import SECRET_PARAMETERS, ConfigError
from ros2_ha_bridge.protocol import DEFAULT_PREFIX, RobotTopics
from ros2_ha_bridge.sensor_mirror import ENVELOPE_KEYS

from .conftest import VECTOR_SECRET
from .fakes import FakeBroker, FakeMqttClient

rclpy = pytest.importorskip('rclpy')


class _Lazy:
    """A module imported on first attribute access (see the module docstring)."""

    def __init__(self, module_name: str) -> None:
        self._module_name = module_name

    def __getattr__(self, name: str) -> Any:
        if name.startswith('_'):
            raise AttributeError(name)
        return getattr(importlib.import_module(self._module_name), name)


bn = _Lazy('ros2_ha_bridge.bridge_node')

ROBOT = 'fake_arm_1'
TOPICS = RobotTopics(ROBOT)
CLIENT_ID = f'ros2_ha_bridge_{ROBOT}'
#: A reserved placeholder name (RFC 2606); the fake broker never resolves it.
HOST = 'broker.example'
FAKE_EXECUTOR = 'ros2_ha_bridge.executors.fake:FakeExecutor'
LOGGER = 'ros2_ha_bridge.bridge_node'
HA_STATUS = protocol.DEFAULT_HA_STATUS_TOPIC
#: Topic suffix to schema kind for the JSON payloads the node publishes.
KIND_BY_SUFFIX = {
    'announce': 'announce',
    'state': 'state',
    'heartbeat': 'heartbeat_robot',
    'joints': 'joints',
    'ack': 'ack',
    'feedback': 'feedback',
    'result': 'result',
}
#: Fast settings so a motion task completes within a fraction of a second.
FAST: dict[str, Any] = {
    'fake.task_duration_s': 0.3,
    'fake.feedback_hz': 10.0,
    'reply.feedback_max_rate_hz': 10.0,
    'heartbeat.interval_s': 0.2,
    'announce.jitter_s': 0.2,
}


def now_ms() -> int:
    return time.time_ns() // 1_000_000


def command(
    task: str | None = 'reset',
    params: dict[str, Any] | None = None,
    *,
    command_id: str | None = None,
    robot_id: str = ROBOT,
    target_id: str | None = None,
    issued_at: int | None = None,
) -> dict[str, Any]:
    """A fresh task command (``task`` given) or cancel command (``task`` None)."""
    payload: dict[str, Any] = {
        'schema': 1,
        'id': command_id or str(uuid.uuid4()),
        'type': 'task' if task is not None else 'cancel',
        'robot_id': robot_id,
        'issued_at': now_ms() if issued_at is None else issued_at,
        'max_age_ms': 5000,
        'source': {'origin': 'ha', 'kind': 'test'},
    }
    if task is not None:
        payload['task'] = task
        payload['params'] = {} if params is None else params
        payload['timeout_s'] = None
    else:
        payload['target_id'] = target_id
    return payload


def send(broker: FakeBroker, payload: dict[str, Any], *, secret: str | None = None) -> str:
    """Publish ``payload`` on the command topic as Home Assistant would; return its id."""
    topic = TOPICS.cmd
    if secret is not None:
        payload = dict(payload, sig=protocol.compute_sig(secret, topic, payload))
    broker.publish(topic, protocol.encode_payload(payload), qos=1)
    return str(payload['id'])


def decode(record: Any) -> dict[str, Any]:
    result: dict[str, Any] = json.loads(record.payload)
    return result


class Rig:
    """One context, one executor, any number of bridge nodes and listener nodes."""

    def __init__(self, broker: FakeBroker) -> None:
        from rclpy.executors import SingleThreadedExecutor

        self.broker = broker
        self.context = rclpy.Context()
        rclpy.init(args=None, context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.nodes: list[Any] = []
        self.listeners: list[Any] = []

    # ----- nodes ---------------------------------------------------------------------

    def node(self, *, name: str = 'bridge_under_test', start: bool = True, **params: Any) -> Any:
        """Build a ``BridgeNode`` with ``params`` (parameter name to value) and start it."""
        from rclpy.parameter import Parameter

        values: dict[str, Any] = {
            'robot.id': ROBOT,
            'mqtt.host': HOST,
            'executor': FAKE_EXECUTOR,
            'data_dir': '',
        }
        values.update(params)
        overrides = [Parameter(key, value=value) for key, value in values.items()]
        node = bn.BridgeNode(
            name,
            context=self.context,
            parameter_overrides=overrides,
            client_factory=self.broker.client_factory,
        )
        self.nodes.append(node)
        self.executor.add_node(node)
        if start:
            node.start()
        return node

    def listener(self, name: str = 'listener') -> Any:
        node = rclpy.create_node(name, context=self.context)
        self.listeners.append(node)
        self.executor.add_node(node)
        return node

    def close(self) -> None:
        for node in self.nodes:
            node.shutdown()
        for node in self.nodes + self.listeners:
            self.executor.remove_node(node)
            node.destroy_node()
        self.executor.shutdown(timeout_sec=1.0)
        rclpy.shutdown(context=self.context)

    # ----- spinning ------------------------------------------------------------------

    def spin_until(self, predicate: Callable[[], bool], timeout_s: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return True
            self.executor.spin_once(timeout_sec=0.02)
        return predicate()

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)

    def connected(self, node: Any) -> bool:
        """Spin until the node's first connect burst (availability online) is on the broker."""
        return self.spin_until(
            lambda: (
                self.broker.retained_payload(node.topics.availability) == b'online'
                and self.broker.retained_payload(node.topics.state) is not None
            )
        )

    # ----- observation ---------------------------------------------------------------

    def client(self, node: Any) -> FakeMqttClient:
        client = node.link.client
        assert isinstance(client, FakeMqttClient)
        return client

    def published_by(self, node: Any, suffix: str | None = None) -> list[Any]:
        """Every record the node's client published, oldest first, optionally by suffix."""
        client_id = node.settings.mqtt.client_id
        records = [r for r in self.broker.published if r.sender == client_id]
        if suffix is not None:
            records = [r for r in records if r.topic.rsplit('/', 1)[-1] == suffix]
        return records

    def acks_for(self, node: Any, command_id: str) -> list[Any]:
        return [r for r in self.published_by(node) if r.topic == node.topics.ack(command_id)]

    def results_for(self, node: Any, command_id: str) -> list[Any]:
        return [r for r in self.published_by(node) if r.topic == node.topics.result(command_id)]


def validate_everything(rig: Rig, node: Any, schema_validator: Any) -> int:
    """Check every payload the node published against the contract; return how many."""
    checked = 0
    for record in rig.published_by(node):
        suffix = record.topic.rsplit('/', 1)[-1]
        if suffix == 'cmd':
            assert record.payload == b'' and record.retain, 'only the retained clear goes here'
            continue
        if suffix == 'availability':
            schema_validator.assert_valid('availability', record.payload.decode('utf-8'))
        else:
            schema_validator.assert_valid(KIND_BY_SUFFIX[suffix], decode(record))
        checked += 1
    return checked


@pytest.fixture
def rig(broker: FakeBroker) -> Iterator[Rig]:
    bridge = Rig(broker)
    try:
        yield bridge
    finally:
        bridge.close()


@pytest.fixture
def errors(caplog: pytest.LogCaptureFixture) -> pytest.LogCaptureFixture:
    caplog.set_level(logging.DEBUG, logger='ros2_ha_bridge')
    return caplog


# --------------------------------------------------------------------------- construction


def test_mqtt_defaults_in_the_parameter_table_match_the_link() -> None:
    defaults = parameters.default_values()
    assert defaults['mqtt.port'] == ml.DEFAULT_PORT
    assert defaults['mqtt.keepalive_s'] == ml.DEFAULT_KEEPALIVE_S
    assert defaults['mqtt.reconnect_min_s'] == ml.DEFAULT_RECONNECT_MIN_S
    assert defaults['mqtt.reconnect_max_s'] == ml.DEFAULT_RECONNECT_MAX_S
    assert defaults['mqtt.takeover_exit_after'] == ml.DEFAULT_TAKEOVER_EXIT_AFTER
    assert defaults['mqtt.inbound_queue_max'] == ml.DEFAULT_INBOUND_QUEUE_MAX


@pytest.mark.parametrize(
    ('params', 'fragment'),
    [
        ({'robot.id': 'ha'}, 'reserved'),
        ({'robot.id': ''}, 'robot.id is required'),
        ({'executor': 'ros2_ha_bridge.executors.fake:Missing'}, 'Missing'),
        ({'executor': 'no_such_entry_point'}, 'no_such_entry_point'),
        ({'topic_prefix': 'homeassistant'}, 'topic_prefix'),
        ({'mqtt.port': 'not-a-number'}, 'mqtt.port'),
    ],
)
def test_misconfiguration_raises_config_error(
    rig: Rig, params: dict[str, Any], fragment: str
) -> None:
    with pytest.raises(ConfigError) as excinfo:
        rig.node(start=False, **params)
    assert fragment in str(excinfo.value), str(excinfo.value)
    assert rig.nodes == [], 'the half-built node was destroyed, nothing to clean up'
    assert rig.broker.connected_client_ids() == []


def test_configuration_is_logged_with_secrets_masked(rig: Rig, errors: Any) -> None:
    node = rig.node(
        start=False, **{'mqtt.password': 'placeholder-password', 'command.secret': 'placeholder'}
    )
    lines = [
        r.getMessage() for r in errors.records if r.name == LOGGER and 'starting' in r.getMessage()
    ]
    assert len(lines) == 1
    assert 'placeholder-password' not in lines[0] and "command.secret='***'" in lines[0]
    assert "mqtt.password='***'" in lines[0] and f"robot.id='{ROBOT}'" in lines[0]
    assert node.settings.mqtt.client_id == CLIENT_ID
    assert node.settings.command.hmac_required


def test_bridge_attributes_never_replace_the_nodes_own(rig: Rig) -> None:
    """A bridge attribute named like one of rclpy's (``_clock``, ``_logger``) would break the node."""
    plain = rig.listener('plain_node')
    node = rig.node(start=False)
    shared = set(vars(plain)) & set(vars(node))
    assert shared, 'rclpy keeps per-node state in instance attributes'
    for name in sorted(shared):
        assert type(vars(node)[name]) is type(vars(plain)[name]), name
    assert node.get_clock() is not None and hasattr(node.get_clock(), 'now')
    assert hasattr(node.get_logger(), 'get_effective_level'), 'the rcutils logger survived'


def test_declared_parameters_cover_the_table_and_are_read_only(rig: Rig) -> None:
    node = rig.node(start=False)
    for spec in parameters.PARAMETERS:
        if spec.name in SECRET_PARAMETERS:
            assert not node.has_parameter(spec.name), f'{spec.name} is undeclared once read'
            continue
        assert node.has_parameter(spec.name), spec.name
        descriptor = node.describe_parameter(spec.name)
        assert descriptor.read_only is True
        assert descriptor.description == spec.description
    assert node.get_parameter('tasks.allow').value in ([], ())
    assert node.get_parameter('heartbeat.interval_s').value == 2.0


def test_secrets_given_as_parameters_warn_and_are_undeclared(
    rig: Rig, errors: Any, monkeypatch: pytest.MonkeyPatch
) -> None:
    node = rig.node(
        start=False, **{'mqtt.password': 'placeholder-password', 'command.secret': 'placeholder'}
    )
    warnings = [
        r.getMessage() for r in errors.records if r.name == LOGGER and r.levelno == logging.WARNING
    ]
    assert any('mqtt.password' in m and parameters.ENV_MQTT_PASSWORD in m for m in warnings)
    assert any('command.secret' in m and parameters.ENV_COMMAND_SECRET in m for m in warnings)
    assert not any('placeholder' in m for m in warnings), 'the value itself is never logged'
    assert not node.has_parameter('mqtt.password') and not node.has_parameter('command.secret')
    assert node.settings.mqtt.password == 'placeholder-password'
    assert 'placeholder' not in repr(node.settings)

    monkeypatch.setenv(parameters.ENV_MQTT_PASSWORD, 'env-password')
    monkeypatch.setenv(parameters.ENV_COMMAND_SECRET, 'env-secret')
    errors.clear()
    quiet = rig.node(name='bridge_from_env', start=False)
    assert quiet.settings.mqtt.password_from_env and quiet.settings.command.secret_from_env
    assert not [
        r
        for r in errors.records
        if r.levelno == logging.WARNING and 'environment variable' in r.getMessage()
    ]


def test_every_timer_runs_on_the_steady_clock(rig: Rig) -> None:
    from rclpy.clock import ClockType

    node = rig.node(**FAST, **{'mqtt.connect_timeout_s': 30.0})
    steady = node.bridge_clock.steady
    assert steady.clock_type == ClockType.STEADY_TIME
    assert node.get_clock().clock_type != ClockType.STEADY_TIME, 'the node clock is ROS_TIME'
    timers = list(node.timers)
    assert len(timers) == 4, (
        'watchdog tick, health poll, heartbeat and the connect-timeout one-shot'
    )
    assert all(timer.clock is steady for timer in timers)
    assert rig.connected(node)
    assert rig.spin_until(lambda: len(list(node.timers)) == 3), 'the one-shot was destroyed'
    assert all(timer.clock is steady for timer in node.timers)


# --------------------------------------------------------------------------- connect


def test_connect_publishes_retained_availability_announce_and_state(
    rig: Rig, schema_validator: Any
) -> None:
    node = rig.node(**FAST)
    assert rig.connected(node)
    broker = rig.broker
    assert broker.retained_payload(TOPICS.availability) == b'online'

    announce = json.loads(broker.retained_payload(TOPICS.announce) or b'null')
    schema_validator.assert_valid('announce', announce)
    assert announce['robot_id'] == ROBOT and announce['name'] == ROBOT
    assert announce['instance'] == node.instance and announce['session'] == node.session
    assert protocol.is_valid_instance(announce['instance'])
    assert announce['executor'] == {'name': 'fake', 'version': __version__}
    assert announce['capabilities'] == {'cancel': True, 'feedback': True, 'hmac_required': False}
    assert announce['heartbeat_interval_s'] == 0.2 and announce['watchdog_mode'] == 'none'
    names = [task['name'] for task in announce['tasks']]
    assert names == [spec.name for spec in node.executor_plugin.tasks()]

    state = json.loads(broker.retained_payload(TOPICS.state) or b'null')
    schema_validator.assert_valid('state', state)
    assert state['state'] == 'idle' and state['instance'] == node.instance
    assert state['session'] == node.session and state['seq'] == 1
    assert state['ha_link'] == 'down'

    subs = {s.filter: s for s in broker.subscriptions_of(CLIENT_ID)}
    assert subs[TOPICS.cmd].qos == 1 and subs[TOPICS.cmd].retain_as_published is True
    assert TOPICS.ha_heartbeat in subs and HA_STATUS in subs
    # The connect burst is ordered: the fake executor reports its starting pose from
    # ``setup``, and those joints never go out before the retained state.
    assert rig.spin_until(lambda: len(rig.published_by(node)) >= 4)
    order = [r.topic.rsplit('/', 1)[-1] for r in rig.published_by(node)][:4]
    assert order == ['availability', 'announce', 'state', 'joints']
    assert validate_everything(rig, node, schema_validator) >= 3


def test_tasks_allow_limits_the_announced_catalogue(rig: Rig) -> None:
    node = rig.node(**{'tasks.allow': ['reset', 'stop', 'not_offered']})
    assert rig.connected(node)
    announce = json.loads(rig.broker.retained_payload(TOPICS.announce) or b'null')
    assert [task['name'] for task in announce['tasks']] == ['reset', 'stop']


# --------------------------------------------------------------------------- joint telemetry


def test_reset_publishes_joint_telemetry_and_the_announce_describes_it(
    rig: Rig, schema_validator: Any
) -> None:
    from ros2_ha_bridge.executors.fake import JOINT_LIMITS, JOINT_NAMES, POSES

    node = rig.node(**FAST, **{'telemetry.joints_rate_hz': 20.0})
    assert rig.connected(node)
    assert node.joints_publisher is not None and node.joint_info is not None
    announce = json.loads(rig.broker.retained_payload(TOPICS.announce) or b'null')
    schema_validator.assert_valid('announce', announce)
    assert announce['joints'] == {
        'unit': 'deg',
        'names': list(JOINT_NAMES),
        'limits': [list(pair) for pair in JOINT_LIMITS],
        'rate_hz': 20.0,
    }
    assert announce['tasks'][1]['params']['properties']['pose']['enum'] == list(POSES)

    # The connect burst carries the starting pose once: QoS 0, not retained, 5 s expiry.
    assert rig.spin_until(lambda: len(rig.published_by(node, 'joints')) >= 1)
    first = rig.published_by(node, 'joints')[0]
    assert first.topic == TOPICS.joints and first.qos == 0 and not first.retain
    assert first.expiry_s == 5
    assert decode(first)['positions'] == [0.0] * 7 and decode(first)['seq'] == 1
    assert rig.broker.retained_payload(TOPICS.joints) is None
    subs = {s.filter: s for s in rig.broker.subscriptions_of(CLIENT_ID)}
    assert TOPICS.joints not in subs, 'the node publishes joints; it never subscribes to them'

    move_id = send(rig.broker, command('move_to', {'pose': 'pick'}))
    assert rig.spin_until(lambda: bool(rig.results_for(node, move_id)))
    rig.spin_for(0.1)
    assert decode(rig.published_by(node, 'joints')[-1])['positions'] == list(POSES['pick'])
    before = len(rig.published_by(node, 'joints'))

    reset_id = send(rig.broker, command('reset'))
    assert rig.spin_until(lambda: bool(rig.results_for(node, reset_id)))
    rig.spin_for(0.1)
    joints = [decode(r) for r in rig.published_by(node, 'joints')[before:]]
    assert len(joints) >= 2, 'a 0.3 s reset at 10 Hz feedback and 20 Hz telemetry'
    for payload in joints:
        schema_validator.assert_valid('joints', payload)
        assert len(payload['positions']) == 7 and payload['velocities'] is None
        assert payload['session'] == node.session and payload['instance'] == node.instance
    seqs = [payload['seq'] for payload in joints]
    assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs), 'seq increases per publication'
    assert joints[-1]['positions'] == [0.0] * 7, 'the final pose always reaches the wire'
    assert node.joints_publisher.seq == seqs[-1]
    assert validate_everything(rig, node, schema_validator) > 5

    # A re-announce republishes the latest positions once, even though nothing moved.
    count = len(rig.published_by(node, 'joints'))
    rig.broker.publish(HA_STATUS, b'online', qos=0)
    assert rig.spin_until(lambda: len(rig.published_by(node, 'announce')) == 2, timeout_s=2.0)
    assert rig.spin_until(lambda: len(rig.published_by(node, 'joints')) == count + 1)
    assert decode(rig.published_by(node, 'joints')[-1])['positions'] == [0.0] * 7


def test_joint_telemetry_off_omits_the_announce_block_and_publishes_nothing(rig: Rig) -> None:
    node = rig.node(**FAST, **{'telemetry.joints_rate_hz': 0.0})
    assert rig.connected(node)
    assert node.joints_publisher is None and node.joint_info is not None
    announce = json.loads(rig.broker.retained_payload(TOPICS.announce) or b'null')
    assert 'joints' not in announce
    move_id = send(rig.broker, command('move_to', {'pose': 'pick'}))
    assert rig.spin_until(lambda: bool(rig.results_for(node, move_id)))
    rig.spin_for(0.1)
    assert rig.published_by(node, 'joints') == []


# --------------------------------------------------------------------------- commands


@pytest.mark.parametrize('signed', [False, True], ids=['unsigned', 'signed'])
def test_valid_command_yields_ack_state_feedback_result_state(
    rig: Rig, schema_validator: Any, signed: bool
) -> None:
    params = dict(FAST)
    if signed:
        params['command.secret'] = VECTOR_SECRET
    node = rig.node(**params)
    assert rig.connected(node)
    burst = len(rig.published_by(node))

    command_id = send(
        rig.broker, command('move_to', {'pose': 'home'}), secret=VECTOR_SECRET if signed else None
    )
    assert rig.spin_until(lambda: bool(rig.results_for(node, command_id)))
    assert rig.spin_until(lambda: node.manager.state == 'idle' and not node.manager.is_busy)
    rig.spin_for(0.05)

    records = [r for r in rig.published_by(node)[burst:] if not r.topic.endswith('/heartbeat')]
    kinds = [r.topic.rsplit('/', 1)[-1] for r in records]
    acks = rig.acks_for(node, command_id)
    assert len(acks) == 1
    ack = decode(acks[0])
    schema_validator.assert_valid('ack', ack)
    assert ack['accepted'] is True and ack['code'] == 'accepted' and ack['task'] == 'move_to'
    assert acks[0].retain and acks[0].qos == 1 and acks[0].expiry_s == 120

    states = [decode(r) for r in records if r.topic == TOPICS.state]
    assert [s['state'] for s in states] == ['busy', 'idle']
    assert states[0]['task_id'] == command_id and states[0]['task'] == 'move_to'
    assert states[1]['task_id'] is None and states[1]['seq'] == states[0]['seq'] + 1

    feedback = [decode(r) for r in records if r.topic == TOPICS.feedback(command_id)]
    assert feedback, 'the fake executor reports progress at 10 Hz during a 0.3 s task'
    assert all(0.0 <= f['progress'] <= 1.0 for f in feedback)

    results = rig.results_for(node, command_id)
    assert len(results) == 1
    result = decode(results[0])
    assert result['status'] == 'succeeded' and result['code'] == 'ok'
    assert result['kind'] == 'motion' and result['goal_status'] == 4
    assert result['data'] == {'pose': 'home'}
    assert results[0].retain and results[0].qos == 1 and results[0].expiry_s == 300

    first = {kind: kinds.index(kind) for kind in ('ack', 'feedback', 'result')}
    busy_at, idle_at = (
        next(
            i for i, r in enumerate(records) if r.topic == TOPICS.state and decode(r)['state'] == s
        )
        for s in ('busy', 'idle')
    )
    assert first['ack'] < first['feedback'] < first['result'] < idle_at
    assert busy_at < first['feedback'], 'busy is published before any feedback'
    assert node.counters.accepted == 1 and node.counters.rejected == 0
    assert validate_everything(rig, node, schema_validator) > 5

    if signed:
        unsigned_id = send(rig.broker, command('reset'))
        assert rig.spin_until(lambda: bool(rig.acks_for(node, unsigned_id)))
        rejected = decode(rig.acks_for(node, unsigned_id)[0])
        assert rejected['code'] == 'rejected_unauthorised' and rejected['accepted'] is False
        assert node.counters.rejected == 1


def test_duplicate_command_republishes_the_stored_ack(rig: Rig) -> None:
    node = rig.node(**FAST)
    assert rig.connected(node)
    payload = command('reset')
    command_id = send(rig.broker, payload)
    assert rig.spin_until(lambda: bool(rig.results_for(node, command_id)))
    send(rig.broker, payload)
    assert rig.spin_until(lambda: len(rig.acks_for(node, command_id)) == 2)
    first, second = rig.acks_for(node, command_id)
    assert first.payload == second.payload, 'the stored bytes, unchanged'
    assert node.counters.duplicates == 1 and node.counters.accepted == 1
    assert len(rig.results_for(node, command_id)) == 1, 'the task ran once'


def test_reset_while_busy_is_rejected_busy_and_a_cancel_ends_the_task(
    rig: Rig, schema_validator: Any
) -> None:
    node = rig.node(**FAST)
    assert rig.connected(node)
    wait_id = send(rig.broker, command('wait', {'seconds': 30}))
    assert rig.spin_until(lambda: bool(rig.acks_for(node, wait_id)))
    assert decode(rig.acks_for(node, wait_id)[0])['code'] == 'accepted'
    assert node.manager.is_busy and node.manager.active_task_id == wait_id

    reset_id = send(rig.broker, command('reset'))
    assert rig.spin_until(lambda: bool(rig.acks_for(node, reset_id)))
    rejected = decode(rig.acks_for(node, reset_id)[0])
    schema_validator.assert_valid('ack', rejected)
    assert rejected['code'] == 'rejected_busy' and rejected['accepted'] is False
    assert 'wait' in rejected['reason'] and wait_id in rejected['reason']
    assert node.counters.rejected == 1 and node.counters.accepted == 1
    assert rig.results_for(node, reset_id) == []

    cancel_id = send(rig.broker, command(None))
    assert rig.spin_until(lambda: bool(rig.results_for(node, wait_id)))
    assert rig.spin_until(lambda: bool(rig.acks_for(node, cancel_id)))
    cancel_ack = decode(rig.acks_for(node, cancel_id)[0])
    assert cancel_ack['code'] == 'accepted' and cancel_ack['type'] == 'cancel'
    result = decode(rig.results_for(node, wait_id)[0])
    assert result['status'] == 'cancelled' and result['goal_status'] == 5
    assert rig.results_for(node, cancel_id) == [], 'a cancel is ack-terminal'
    assert rig.spin_until(lambda: not node.manager.is_busy)
    assert validate_everything(rig, node, schema_validator) > 5


def test_command_for_an_unknown_task_and_bad_params_are_rejected(rig: Rig) -> None:
    node = rig.node(**FAST)
    assert rig.connected(node)
    unknown_id = send(rig.broker, command('fly'))
    bad_id = send(rig.broker, command('move_to', {'pose': 42}))
    assert rig.spin_until(
        lambda: bool(rig.acks_for(node, unknown_id)) and bool(rig.acks_for(node, bad_id))
    )
    assert decode(rig.acks_for(node, unknown_id)[0])['code'] == 'rejected_unknown_task'
    assert decode(rig.acks_for(node, bad_id)[0])['code'] == 'rejected_invalid_params'


def test_planted_retained_command_is_dropped_and_cleared_live(rig: Rig) -> None:
    node = rig.node(**FAST)
    assert rig.connected(node)
    payload = command('reset')
    rig.broker.publish(TOPICS.cmd, protocol.encode_payload(payload), qos=1, retain=True)
    assert rig.spin_until(lambda: node.counters.dropped_retained == 1)
    assert rig.spin_until(lambda: rig.broker.retained_payload(TOPICS.cmd) is None)
    clears = [r for r in rig.published_by(node, 'cmd')]
    assert len(clears) == 1 and clears[0].payload == b'' and clears[0].retain
    rig.spin_for(0.1)
    assert rig.acks_for(node, str(payload['id'])) == [], 'no ack for a retained command'
    assert node.counters.dropped_retained == 1, 'the clearing echo is ignored, not counted'
    assert node.counters.accepted == 0


def test_planted_retained_command_before_connect_is_dropped_at_connect(rig: Rig) -> None:
    payload = command('reset')
    rig.broker.publish(TOPICS.cmd, protocol.encode_payload(payload), qos=1, retain=True)
    node = rig.node(**FAST)
    assert rig.connected(node)
    assert rig.spin_until(lambda: node.counters.dropped_retained == 1)
    assert rig.spin_until(lambda: rig.broker.retained_payload(TOPICS.cmd) is None)
    assert rig.acks_for(node, str(payload['id'])) == []


def test_command_with_a_wildcard_id_is_dropped_and_the_node_stays_alive(
    rig: Rig, errors: Any
) -> None:
    node = rig.node(**FAST)
    assert rig.connected(node)
    before = len(rig.published_by(node))
    for bad_id in ('x/#', 'x/+', 'not-a-uuid', '6A0C2B58-1D7E-4A1F-9B2A-3C4D5E6F7A8B'):
        send(rig.broker, command('reset', command_id=bad_id))
    assert rig.spin_until(lambda: node.counters.dropped_invalid_id == 4)
    rig.spin_for(0.1)
    task_topics = f'{DEFAULT_PREFIX}/{ROBOT}/task/'
    assert [r for r in rig.published_by(node)[before:] if r.topic.startswith(task_topics)] == []
    assert not node.exit_future.done() and rig.broker.callback_errors == []

    good_id = send(rig.broker, command('reset'))
    assert rig.spin_until(lambda: bool(rig.results_for(node, good_id)))
    assert decode(rig.acks_for(node, good_id)[0])['code'] == 'accepted'
    warnings = [
        r for r in errors.records if r.levelno == logging.WARNING and 'usable id' in r.getMessage()
    ]
    assert len(warnings) == 1, 'rate-limited to one WARN line'


# --------------------------------------------------------------------------- HA side


def test_ha_birth_reannounces_after_the_jitter(rig: Rig) -> None:
    node = rig.node(**FAST)
    assert rig.connected(node)
    first = decode(rig.published_by(node, 'announce')[0])
    rig.broker.publish(HA_STATUS, b'online', qos=0)
    rig.broker.publish(HA_STATUS, b'online', qos=0)
    assert rig.spin_until(lambda: len(rig.published_by(node, 'announce')) == 2, timeout_s=2.0)
    rig.spin_for(0.4)
    announces = rig.published_by(node, 'announce')
    assert len(announces) == 2, 'two births within the jitter give one re-announce'
    again = decode(announces[1])
    assert again['ts'] >= first['ts'] and again['session'] == first['session']
    assert announces[1].retain and announces[1].qos == 1
    assert rig.broker.retained_payload(TOPICS.announce) == announces[1].payload


def test_ha_link_latches_and_the_heartbeat_reports_the_clock_offset(
    rig: Rig, schema_validator: Any
) -> None:
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool

    node = rig.node(**FAST)
    assert rig.connected(node)
    assert rig.spin_until(lambda: len(rig.published_by(node, 'heartbeat')) >= 1)
    beat = decode(rig.published_by(node, 'heartbeat')[-1])
    schema_validator.assert_valid('heartbeat_robot', beat)
    assert beat['ha_clock_offset_ms'] is None and beat['instance'] == node.instance
    assert beat['uptime_s'] >= 0 and beat['counters']['accepted'] == 0
    assert not rig.published_by(node, 'heartbeat')[-1].retain

    seen: list[bool] = []
    listener = rig.listener()
    listener.create_subscription(
        Bool,
        f'/{node.get_name()}/ha_link',
        lambda message: seen.append(bool(message.data)),
        QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    assert rig.spin_until(lambda: seen == [False]), 'the late subscriber gets the latched value'

    skew = 12
    rig.broker.publish(
        TOPICS.ha_heartbeat,
        protocol.encode_payload(
            {'schema': 1, 'ts': now_ms() - skew, 'seq': 1, 'interval_ms': 2000, 'instance_id': 'x'}
        ),
        qos=0,
    )
    assert rig.spin_until(lambda: seen == [False, True])
    assert node.watchdog.link_up
    assert rig.spin_until(
        lambda: decode(rig.published_by(node, 'heartbeat')[-1])['ha_clock_offset_ms'] is not None
    )
    offset = decode(rig.published_by(node, 'heartbeat')[-1])['ha_clock_offset_ms']
    assert isinstance(offset, int) and skew - 50 <= offset <= skew + 500
    state = json.loads(rig.broker.retained_payload(TOPICS.state) or b'null')
    assert state['ha_link'] == 'up'
    assert validate_everything(rig, node, schema_validator) > 3


def test_mirror_topic_is_republished_as_an_envelope(rig: Rig) -> None:
    from std_msgs.msg import String

    node = rig.node(**FAST, **{'mirror.topics': ['zigbee2mqtt/kitchen_motion']})
    assert rig.connected(node)
    filters = {s.filter for s in rig.broker.subscriptions_of(CLIENT_ID)}
    assert {'zigbee2mqtt/kitchen_motion', 'zigbee2mqtt/kitchen_motion/availability'} <= filters

    texts: list[str] = []
    listener = rig.listener()
    listener.create_subscription(
        String, f'/{node.get_name()}/mqtt_in', lambda message: texts.append(message.data), 10
    )
    topic = f'/{node.get_name()}/mqtt_in'
    assert rig.spin_until(lambda: node.count_subscribers(topic) >= 1), 'discovery matched'
    rig.broker.publish('zigbee2mqtt/kitchen_motion/availability', b'online', qos=0, retain=True)
    rig.broker.publish('zigbee2mqtt/kitchen_motion', b'{"occupancy":true}', qos=0)
    assert rig.spin_until(lambda: len(texts) == 2)
    envelopes = [json.loads(text) for text in texts]
    assert [list(e) for e in envelopes] == [list(ENVELOPE_KEYS)] * 2
    assert envelopes[0]['topic'] == 'zigbee2mqtt/kitchen_motion/availability'
    assert envelopes[1]['topic'] == 'zigbee2mqtt/kitchen_motion'
    assert envelopes[1]['payload'] == '{"occupancy":true}'
    assert envelopes[1]['encoding'] == 'utf-8' and envelopes[1]['retain'] is False
    assert envelopes[1]['source_available'] is True
    assert abs(envelopes[1]['rx_ts_ms'] - now_ms()) < 5000


# --------------------------------------------------------------------------- lifecycle


def test_clean_shutdown_publishes_offline_and_is_idempotent(rig: Rig) -> None:
    node = rig.node(**FAST)
    assert rig.connected(node)
    node.shutdown()
    assert node.closed and node.link.closed
    assert rig.broker.retained_payload(TOPICS.availability) == b'offline'
    offline = [r for r in rig.broker.messages_on(TOPICS.availability) if r.payload == b'offline']
    assert len(offline) == 1 and offline[0].sender == CLIENT_ID, 'clean: not the will'
    assert not rig.broker.is_connected(CLIENT_ID)
    published = len(rig.broker.published)
    node.shutdown()
    rig.spin_for(0.3)
    assert len(rig.broker.published) == published, 'a second shutdown and the timers do nothing'


def test_unclean_disconnect_publishes_the_will_and_a_reconnect_republishes(
    rig: Rig, schema_validator: Any
) -> None:
    node = rig.node(**FAST)
    assert rig.connected(node)
    first_state = json.loads(rig.broker.retained_payload(TOPICS.state) or b'null')
    rig.broker.drop_connection(CLIENT_ID)
    assert rig.broker.retained_payload(TOPICS.availability) == b'offline'
    offline = [r for r in rig.broker.messages_on(TOPICS.availability) if r.payload == b'offline']
    assert offline[-1].sender is None, 'published by the broker from the will'
    assert rig.spin_until(lambda: not node.link.connected)

    rig.client(node).reconnect()
    assert rig.spin_until(lambda: node.link.connects == 2)
    assert rig.spin_until(lambda: rig.broker.retained_payload(TOPICS.availability) == b'online')
    assert rig.spin_until(
        lambda: (
            json.loads(rig.broker.retained_payload(TOPICS.state) or b'{}').get('seq', 0)
            > first_state['seq']
        )
    )
    assert len(rig.published_by(node, 'announce')) == 2
    assert validate_everything(rig, node, schema_validator) > 5
    assert not node.exit_future.done()


def test_connect_timeout_exits_non_zero(rig: Rig, errors: Any) -> None:
    rig.broker.unreachable = True
    node = rig.node(**{'mqtt.connect_timeout_s': 0.3})
    assert rig.spin_until(lambda: node.exit_future.done(), timeout_s=3.0)
    assert node.exit_code == bn.EXIT_CONNECT_TIMEOUT
    assert node.exit_reason is not None and 'CONNACK' in node.exit_reason
    assert any('CONNACK' in r.getMessage() for r in errors.records if r.levelno == logging.ERROR)


def test_two_nodes_with_the_same_robot_id_flap_until_one_exits(rig: Rig, errors: Any) -> None:
    first = rig.node(name='bridge_first', **FAST)
    assert rig.connected(first)
    second = rig.node(name='bridge_second', **FAST)
    assert rig.spin_until(lambda: first.counters.takeovers == 1)
    assert rig.spin_until(lambda: second.link.connected and not first.link.connected)
    assert first.instance == second.instance, 'one process: the instance is the same'
    assert first.session != second.session

    def flap(victim: Any, intruder: Any, expected: int) -> None:
        rig.client(victim).reconnect()
        assert rig.spin_until(lambda: intruder.counters.takeovers == expected)
        assert rig.spin_until(lambda: not intruder.link.connected and victim.link.connected)

    flap(first, second, 1)
    flap(second, first, 2)
    flap(first, second, 2)
    assert not first.exit_future.done() and not second.exit_future.done()
    flap(second, first, 3)
    assert rig.spin_until(lambda: first.exit_future.done())
    assert first.exit_code == bn.EXIT_TAKEOVER
    assert first.exit_reason is not None and 'taken over' in first.exit_reason
    assert not second.exit_future.done(), 'the survivor keeps running'
    assert second.counters.takeovers == 2
    takeover_lines = [
        r.getMessage()
        for r in errors.records
        if r.levelno == logging.ERROR and 'robot.id' in r.getMessage()
    ]
    assert takeover_lines, 'the takeover names the likely cause'
    assert any('exiting with status' in r.getMessage() for r in errors.records)
    first.shutdown()
    assert rig.spin_until(lambda: second.link.connected)
    assert rig.broker.retained_payload(TOPICS.availability) in (b'online', b'offline')
