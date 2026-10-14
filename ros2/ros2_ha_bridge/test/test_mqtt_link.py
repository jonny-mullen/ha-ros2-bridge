# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Tests for :mod:`ros2_ha_bridge.mqtt_link` against ``FakeBroker`` and ``FakeMqttClient``.

No broker and no network: the link is built with ``broker.client_factory`` so
the paho ``Client`` it would construct is a ``FakeMqttClient`` bound to an
in-memory MQTT 5 broker. Most tests run the fake client in synchronous mode
(callbacks inline on the test thread) so every assertion is deterministic;
one test keeps the real delivery thread to show the deque and the wake
callable crossing threads as they do under paho. The ``broker`` fixture fails
a test if any callback raised inside the fake client, which is how "never
raises" is enforced for every callback body of the link.

Import hygiene: ``test_module_imports_paho_but_not_rclpy`` and
``test_ros_adapters_module_imports_without_rclpy`` check the two modules' own
import graphs in a fresh interpreter (``conftest.assert_imports_without``), so
they hold under ``colcon test`` as well, where rclpy is already loaded in the
test process before any test runs.
"""

from __future__ import annotations

import logging
import threading
from collections.abc import Iterator
from typing import Any

import pytest

from ros2_ha_bridge import mqtt_link as ml
from ros2_ha_bridge import protocol
from ros2_ha_bridge.protocol import Counters, RobotTopics

from .conftest import assert_imports_without
from .fakes import (
    MQTT_ERR_INVAL,
    MQTT_ERR_NO_CONN,
    REASON_BAD_CREDENTIALS,
    REASON_NOT_AUTHORIZED,
    REASON_SERVER_UNAVAILABLE,
    REASON_UNSPECIFIED_ERROR,
    FakeBroker,
    FakeClock,
    FakeMessageInfo,
    FakeMqttClient,
    MQTTv5,
    WillSpec,
)

ROBOT = 'arm_left'
TOPICS = RobotTopics(ROBOT)
CLIENT_ID = f'ros2_ha_bridge_{ROBOT}'
#: A reserved placeholder name (RFC 2606); the fake broker never resolves it.
HOST = 'broker.example'
LOGGER = 'ros2_ha_bridge.mqtt_link'
TASK_ID = '6a0c2b58-1d7e-4a1f-9b2a-3c4d5e6f7a8b'


def default_subscriptions() -> tuple[Any, ...]:
    """What the node subscribes to: cmd with Retain As Published, the rest plain QoS 0."""
    return (
        ml.SubscriptionSpec(TOPICS.cmd, 1, retain_as_published=True),
        ml.SubscriptionSpec(TOPICS.ha_heartbeat, 0),
        ml.SubscriptionSpec(protocol.DEFAULT_HA_STATUS_TOPIC, 0),
    )


class Rig:
    """One link on one fake broker, with a counting wake and the node's shared counters."""

    def __init__(self, broker: FakeBroker, clock: FakeClock, **overrides: Any) -> None:
        self.broker = broker
        self.clock = clock
        self.wakes = 0
        self.wake_threads: list[int] = []
        self.counters = Counters()
        factory = overrides.pop('client_factory', broker.client_factory)
        subscriptions = overrides.pop('subscriptions', None)
        wake = overrides.pop('wake', self._wake)
        values: dict[str, Any] = {
            'host': HOST,
            'client_id': CLIENT_ID,
            'username': 'bridge-user',
            'password': 'placeholder-password',
        }
        values.update(overrides)
        self.config = ml.MqttLinkConfig(**values)
        self.link = ml.MqttLink(
            self.config,
            availability_topic=TOPICS.availability,
            wake=wake,
            counters=self.counters,
            clock=clock,
            subscriptions=default_subscriptions() if subscriptions is None else subscriptions,
            client_factory=factory,
        )

    def _wake(self) -> None:
        self.wakes += 1
        self.wake_threads.append(threading.get_ident())

    @property
    def client(self) -> FakeMqttClient:
        client = self.link.client
        assert isinstance(client, FakeMqttClient)
        return client

    def start(self) -> list[Any]:
        """Start the link, wait for delivery and return what the connect queued."""
        self.link.start()
        assert self.broker.wait_idle()
        return list(self.link.drain())

    def intruder(self) -> FakeMqttClient:
        """A second client with the same client id (another node with this robot.id)."""
        return self.broker.make_client(None, CLIENT_ID, synchronous=True)


@pytest.fixture
def sync_broker(broker: FakeBroker) -> FakeBroker:
    broker.synchronous = True
    return broker


@pytest.fixture
def rig(sync_broker: FakeBroker, clock: FakeClock) -> Rig:
    return Rig(sync_broker, clock)


@pytest.fixture
def errors(caplog: pytest.LogCaptureFixture) -> Iterator[pytest.LogCaptureFixture]:
    caplog.set_level(logging.DEBUG, logger=LOGGER)
    yield caplog


def records(caplog: pytest.LogCaptureFixture, level: int) -> list[logging.LogRecord]:
    return [r for r in caplog.records if r.name == LOGGER and r.levelno == level]


def messages(items: list[Any]) -> list[Any]:
    return [i for i in items if isinstance(i, ml.InboundMessage)]


# --------------------------------------------------------------------------- imports and config


def test_module_imports_paho_but_not_rclpy() -> None:
    assert_imports_without('ros2_ha_bridge.mqtt_link', {'rclpy'}, expected={'paho'})


def test_ros_adapters_module_imports_without_rclpy() -> None:
    """The adapters must import on a machine without ROS (mypy and this suite run there).

    It lives here because ``test_ros_adapters.py`` skips itself entirely
    without rclpy, and this is the only phase 2 test file the local suite runs.
    """
    assert_imports_without('ros2_ha_bridge.ros_adapters', {'rclpy'})


def test_config_repr_never_shows_the_password(rig: Rig) -> None:
    assert rig.config.password == 'placeholder-password'
    assert 'placeholder-password' not in repr(rig.config)
    assert 'placeholder-password' not in repr(rig.link.config)


@pytest.mark.parametrize(
    'overrides',
    [
        {'host': ''},
        {'host': '   '},
        {'client_id': ''},
        {'port': 0},
        {'port': 65536},
        {'keepalive_s': -1},
        {'reconnect_min_s': 0},
        {'reconnect_max_s': 0},
        {'takeover_exit_after': -1},
        {'max_payload_bytes': 0},
        {'inbound_queue_max': 0},
        {'warn_interval_s': -1.0},
    ],
)
def test_config_rejects_bad_values(overrides: dict[str, Any]) -> None:
    values: dict[str, Any] = {'host': HOST, 'client_id': CLIENT_ID}
    values.update(overrides)
    with pytest.raises(ValueError):
        ml.MqttLinkConfig(**values)


def test_subscription_spec_validates_filter_and_qos() -> None:
    with pytest.raises(ValueError):
        ml.SubscriptionSpec('a/#/b')
    with pytest.raises(ValueError):
        ml.SubscriptionSpec(TOPICS.cmd, 3)
    assert ml.SubscriptionSpec(TOPICS.cmd, 1, retain_as_published=True).retain_as_published


def test_availability_topic_is_validated(sync_broker: FakeBroker) -> None:
    with pytest.raises(ValueError):
        ml.MqttLink(
            ml.MqttLinkConfig(host=HOST, client_id=CLIENT_ID),
            availability_topic='ros2_bridge/+/availability',
            wake=lambda: None,
            client_factory=sync_broker.client_factory,
        )


# --------------------------------------------------------------------------- connection lifecycle


def test_client_is_built_as_the_design_says(rig: Rig) -> None:
    client = rig.client
    assert client.callback_api_version is not None
    assert client.callback_api_version.value == 2
    assert client.protocol == MQTTv5
    assert client.reconnect_on_failure is True
    assert client.client_id == CLIENT_ID
    assert (client.username, client.password) == ('bridge-user', 'placeholder-password')
    assert client.will == WillSpec(TOPICS.availability, b'offline', 1, True, None)
    assert client.reconnect_delay == (1, 30)
    assert client.tls_args is None and client.tls_insecure is False
    assert client.logger is not None, 'paho log lines are routed to the package logger'
    assert client.connect_args is None, 'construction opens nothing'
    assert rig.link.counters is rig.counters
    assert not rig.link.started and not rig.link.connected and not rig.link.connack_seen


def test_no_credentials_when_username_is_empty(sync_broker: FakeBroker, clock: FakeClock) -> None:
    anonymous = Rig(sync_broker, clock, username='', password='')
    assert anonymous.client.username is None and anonymous.client.password is None


def test_tls_configuration(sync_broker: FakeBroker, clock: FakeClock) -> None:
    secure = Rig(
        sync_broker,
        clock,
        tls_enabled=True,
        tls_ca_cert='/etc/example/ca.pem',
        tls_certfile='/etc/example/client.pem',
        tls_keyfile='/etc/example/client.key',
        tls_insecure=True,
    )
    args = secure.client.tls_args
    assert args is not None
    assert args['ca_certs'] == '/etc/example/ca.pem'
    assert args['certfile'] == '/etc/example/client.pem'
    assert args['keyfile'] == '/etc/example/client.key'
    assert secure.client.tls_insecure is True

    ca_only = Rig(sync_broker, clock, tls_enabled=True, tls_ca_cert='/etc/example/ca.pem')
    args = ca_only.client.tls_args
    assert args is not None
    assert args['certfile'] is None and args['keyfile'] is None
    assert ca_only.client.tls_insecure is False


def test_start_connects_with_clean_start_and_subscribes_cmd_retain_as_published(
    rig: Rig, clock: FakeClock
) -> None:
    items = rig.start()
    args = rig.client.connect_args
    assert args is not None
    assert (args['host'], args['port'], args['keepalive']) == (HOST, 1883, 15)
    assert args['clean_start'] is True
    assert rig.link.started and rig.link.connected and rig.link.connack_seen
    assert rig.link.connects == 1

    subs = {s.filter: s for s in rig.broker.subscriptions_of(CLIENT_ID)}
    assert set(subs) == {TOPICS.cmd, TOPICS.ha_heartbeat, protocol.DEFAULT_HA_STATUS_TOPIC}
    assert subs[TOPICS.cmd].qos == 1 and subs[TOPICS.cmd].retain_as_published is True
    assert subs[TOPICS.ha_heartbeat].qos == 0
    assert not subs[TOPICS.ha_heartbeat].retain_as_published
    assert not subs[protocol.DEFAULT_HA_STATUS_TOPIC].retain_as_published

    assert items == [ml.ConnectEvent(clock.now_ms(), 0, 'Success', False)]
    assert items[0].ok
    assert rig.wakes == 1, 'the connect event woke the main thread'

    rig.link.start()
    assert rig.client.connect_attempts == 1, 'a second start is a no-op'


def test_subscriptions_are_renewed_on_every_connect(rig: Rig) -> None:
    assert rig.link.subscribe('zigbee2mqtt/+', 0) is True, 'registered before start'
    assert rig.link.subscribe('zigbee2mqtt/+', 0) is True
    assert len(rig.link.subscriptions) == 4, 'no duplicate registration'
    rig.start()
    assert {s.filter for s in rig.broker.subscriptions_of(CLIENT_ID)} >= {'zigbee2mqtt/+'}

    rig.broker.drop_connection(CLIENT_ID)
    assert rig.broker.subscriptions_of(CLIENT_ID) == []
    rig.client.reconnect()
    filters = {s.filter for s in rig.broker.subscriptions_of(CLIENT_ID)}
    assert filters == {s.topic_filter for s in default_subscriptions()} | {'zigbee2mqtt/+'}
    assert rig.link.connects == 2

    assert rig.link.subscribe('zigbee2mqtt/kitchen/availability', 0) is True
    assert 'zigbee2mqtt/kitchen/availability' in {
        s.filter for s in rig.broker.subscriptions_of(CLIENT_ID)
    }, 'subscribed at once while connected'


def test_subscribe_refuses_a_bad_filter_and_reports_a_failed_call(
    rig: Rig, errors: pytest.LogCaptureFixture
) -> None:
    assert rig.link.subscribe('a/#/b') is False
    assert any('a/#/b' in r.getMessage() for r in records(errors, logging.ERROR))
    rig.start()

    def refuse(*args: Any, **kwargs: Any) -> tuple[int, None]:
        return (MQTT_ERR_NO_CONN, None)

    rig.client.subscribe = refuse  # type: ignore[method-assign]
    assert rig.link.subscribe('zigbee2mqtt/+') is False
    assert 'zigbee2mqtt/+' in records(errors, logging.ERROR)[-1].getMessage()


# --------------------------------------------------------------------------- publish


def test_publish_sets_expiry_properties(rig: Rig) -> None:
    rig.start()
    assert rig.link.publish(TOPICS.heartbeat, b'{"schema":1}', 0, False, 5) is True
    record = rig.broker.last_on(TOPICS.heartbeat)
    assert record is not None
    assert (record.payload, record.qos, record.retain, record.expiry_s) == (
        b'{"schema":1}',
        0,
        False,
        5,
    )
    assert rig.link.publish(TOPICS.state, b'{"schema":1}', 1, True, None) is True
    state = rig.broker.last_on(TOPICS.state)
    assert state is not None
    assert state.expiry_s is None and state.retain is True and state.qos == 1
    assert rig.broker.retained_payload(TOPICS.state) == b'{"schema":1}'
    assert rig.link.publish(TOPICS.ack(TASK_ID), b'{}', 1, True, 120) is True
    ack = rig.broker.last_on(TOPICS.ack(TASK_ID))
    assert ack is not None and ack.expiry_s == 120
    assert rig.counters.publish_errors == 0


def test_publish_json_uses_the_contract_encoding(
    rig: Rig, errors: pytest.LogCaptureFixture
) -> None:
    rig.start()
    payload = {'schema': 1, 'robot_id': ROBOT, 'reason': 'café'}
    assert rig.link.publish_json(TOPICS.state, payload, 1, True, None) is True
    record = rig.broker.last_on(TOPICS.state)
    assert record is not None and record.payload == protocol.encode_payload(payload)

    assert rig.link.publish_json(TOPICS.state, {'bad': {1, 2}}, 1, True, None) is False
    assert rig.counters.publish_errors == 1
    assert TOPICS.state in records(errors, logging.ERROR)[-1].getMessage()


def test_publish_guard_swallows_a_wildcard_topic(
    rig: Rig, errors: pytest.LogCaptureFixture
) -> None:
    rig.start()
    before = len(rig.broker.published)
    assert rig.link.publish('ros2_bridge/+/state', b'{}', 1, True, None) is False
    assert rig.link.publish('ros2_bridge/#', b'{}', 0, False, None) is False
    assert rig.link.publish('', b'{}', 0, False, None) is False
    assert rig.link.publish('ros2_bridge/x\x00y', b'{}', 0, False, None) is False
    assert rig.link.publish(TOPICS.state, b'{}', 1, True, -1) is False, 'negative expiry'
    assert rig.counters.publish_errors == 5
    assert len(rig.broker.published) == before, 'nothing reached the client'
    lines = [r.getMessage() for r in records(errors, logging.ERROR)]
    assert len(lines) == 5
    assert "'ros2_bridge/+/state'" in lines[0] and 'ValueError' in lines[0]


def test_publish_guard_swallows_an_oserror_from_the_client(
    sync_broker: FakeBroker, clock: FakeClock, errors: pytest.LogCaptureFixture
) -> None:
    class BrokenClient(FakeMqttClient):
        def publish(self, *args: Any, **kwargs: Any) -> FakeMessageInfo:
            raise OSError('socket closed under the network thread')

    def factory(*args: Any, **kwargs: Any) -> FakeMqttClient:
        return BrokenClient(*args, broker=sync_broker, synchronous=True, **kwargs)

    rig = Rig(sync_broker, clock, client_factory=factory)
    rig.start()
    assert rig.link.publish(TOPICS.state, b'{}', 1, True, None) is False
    assert rig.counters.publish_errors == 1
    message = records(errors, logging.ERROR)[-1].getMessage()
    assert TOPICS.state in message and 'OSError' in message and 'socket closed' in message


def test_publish_guard_reports_a_paho_error_code(
    rig: Rig, errors: pytest.LogCaptureFixture
) -> None:
    assert rig.link.publish(TOPICS.heartbeat, b'{}', 0, False, 5) is False, 'QoS 0 is lost'
    assert rig.counters.publish_errors == 1
    message = records(errors, logging.ERROR)[-1].getMessage()
    assert TOPICS.heartbeat in message and str(int(MQTT_ERR_NO_CONN)) in message
    rig.start()
    assert rig.link.publish(TOPICS.state, b'{}', 1, True, None) is True
    assert rig.counters.publish_errors == 1


def test_qos1_publish_while_disconnected_is_queued_not_failed(
    rig: Rig, errors: pytest.LogCaptureFixture
) -> None:
    """paho keeps a QoS 1 message handed to it while disconnected and sends it after the CONNACK."""
    assert rig.link.publish(TOPICS.ack(TASK_ID), b'{"queued":1}', 1, True, 120) is True
    assert rig.counters.publish_errors == 0
    assert records(errors, logging.ERROR) == []
    assert any('queued' in r.getMessage() for r in records(errors, logging.DEBUG))
    assert rig.broker.last_on(TOPICS.ack(TASK_ID)) is None, 'nothing on the broker yet'
    rig.start()
    record = rig.broker.last_on(TOPICS.ack(TASK_ID))
    assert record is not None and record.payload == b'{"queued":1}' and record.dup
    assert record.retain and record.qos == 1 and record.expiry_s == 120


def test_publish_guard_contains_an_unexpected_exception(
    sync_broker: FakeBroker, clock: FakeClock, errors: pytest.LogCaptureFixture
) -> None:
    class SurprisingClient(FakeMqttClient):
        def publish(self, *args: Any, **kwargs: Any) -> FakeMessageInfo:
            raise RuntimeError('a bug nobody expected')

    def factory(*args: Any, **kwargs: Any) -> FakeMqttClient:
        return SurprisingClient(*args, broker=sync_broker, synchronous=True, **kwargs)

    rig = Rig(sync_broker, clock, client_factory=factory)
    rig.start()
    assert rig.link.publish(TOPICS.state, b'{}', 1, True, None) is False
    assert rig.counters.publish_errors == 1
    record = records(errors, logging.ERROR)[-1]
    assert record.exc_info is not None and TOPICS.state in record.getMessage()


# --------------------------------------------------------------------------- inbound


def test_oversize_inbound_is_dropped_and_counted(
    sync_broker: FakeBroker, clock: FakeClock, errors: pytest.LogCaptureFixture
) -> None:
    rig = Rig(sync_broker, clock, max_payload_bytes=64)
    rig.start()
    wakes = rig.wakes
    sync_broker.publish(TOPICS.cmd, b'x' * 65, qos=1)
    assert rig.link.drain() == []
    assert rig.counters.dropped_oversize == 1
    assert rig.wakes == wakes, 'a dropped message wakes nobody'
    assert any('65-byte' in r.getMessage() for r in records(errors, logging.WARNING))

    sync_broker.publish(TOPICS.cmd, b'y' * 64, qos=1)
    items = messages(rig.link.drain())
    assert len(items) == 1 and items[0].payload == b'y' * 64
    assert rig.counters.dropped_oversize == 1


def test_oversize_warnings_are_rate_limited(
    sync_broker: FakeBroker, clock: FakeClock, errors: pytest.LogCaptureFixture
) -> None:
    rig = Rig(sync_broker, clock, max_payload_bytes=8, warn_interval_s=10.0)
    rig.start()
    for _ in range(4):
        sync_broker.publish(TOPICS.cmd, b'x' * 9, qos=1)
    assert rig.counters.dropped_oversize == 4
    assert len(records(errors, logging.WARNING)) == 1
    clock.advance(10.0)
    sync_broker.publish(TOPICS.cmd, b'x' * 9, qos=1)
    warnings = records(errors, logging.WARNING)
    assert len(warnings) == 2
    assert '3 similar warnings suppressed' in warnings[-1].getMessage()


def test_full_queue_drops_new_messages_and_counts(
    sync_broker: FakeBroker, clock: FakeClock, errors: pytest.LogCaptureFixture
) -> None:
    rig = Rig(sync_broker, clock, inbound_queue_max=3)
    rig.start()
    for index in range(5):
        sync_broker.publish(TOPICS.cmd, f'{{"n":{index}}}'.encode(), qos=1)
    assert rig.link.queue_length == 3
    assert rig.counters.dropped_queue_full == 2
    assert [i.payload for i in messages(rig.link.drain())] == [
        b'{"n":0}',
        b'{"n":1}',
        b'{"n":2}',
    ], 'the oldest messages are kept; the newest were dropped'
    assert len(records(errors, logging.WARNING)) == 1, 'rate-limited'
    assert rig.link.drain() == []


def test_connection_events_are_never_dropped(sync_broker: FakeBroker, clock: FakeClock) -> None:
    rig = Rig(sync_broker, clock, inbound_queue_max=2)
    rig.start()
    sync_broker.publish(TOPICS.cmd, b'first', qos=1)
    sync_broker.publish(TOPICS.cmd, b'second', qos=1)
    assert rig.link.queue_length == 2
    sync_broker.drop_connection(CLIENT_ID)
    items = rig.link.drain()
    assert rig.counters.dropped_queue_full == 1, 'the oldest message made room'
    assert len(items) == 2
    assert isinstance(items[0], ml.InboundMessage) and items[0].payload == b'second'
    assert isinstance(items[1], ml.DisconnectEvent)


def test_drain_returns_items_in_order_with_rx_ts_ms(rig: Rig, clock: FakeClock) -> None:
    rig.start()
    t0 = clock.now_ms()
    rig.broker.publish(TOPICS.cmd, b'{"a":1}', qos=1)
    clock.advance(1.5)
    rig.broker.publish(TOPICS.ha_heartbeat, b'{"b":2}', qos=0)
    clock.advance(0.25)
    rig.broker.publish(TOPICS.cmd, b'planted', qos=1, retain=True)
    items = rig.link.drain()
    assert items == [
        ml.InboundMessage(TOPICS.cmd, b'{"a":1}', False, t0),
        ml.InboundMessage(TOPICS.ha_heartbeat, b'{"b":2}', False, t0 + 1500),
        ml.InboundMessage(TOPICS.cmd, b'planted', True, t0 + 1750),
    ]
    assert rig.wakes == 4, 'one wake per queued item (the connect event and three messages)'
    assert rig.link.drain() == []


def test_retain_flag_live_only_under_retain_as_published(rig: Rig) -> None:
    rig.broker.publish(TOPICS.cmd, b'old command', qos=1, retain=True)
    rig.broker.publish(TOPICS.ha_heartbeat, b'old beat', qos=0, retain=True)
    replayed = messages(rig.start())
    assert {(i.topic, i.retain) for i in replayed} == {
        (TOPICS.cmd, True),
        (TOPICS.ha_heartbeat, True),
    }
    rig.broker.publish(TOPICS.ha_heartbeat, b'live beat', qos=0, retain=True)
    rig.broker.publish(TOPICS.cmd, b'live command', qos=1, retain=True)
    live = messages(rig.link.drain())
    assert [(i.payload, i.retain) for i in live] == [
        (b'live beat', False),
        (b'live command', True),
    ]


def test_inbound_after_shutdown_is_ignored(rig: Rig) -> None:
    rig.start()
    rig.link.shutdown()
    wakes = rig.wakes
    rig.broker.publish(TOPICS.cmd, b'late', qos=1)
    assert rig.link.drain() == [] and rig.wakes == wakes


def test_wake_exception_is_contained(
    sync_broker: FakeBroker, clock: FakeClock, errors: pytest.LogCaptureFixture
) -> None:
    def explode() -> None:
        raise RuntimeError('guard condition gone')

    rig = Rig(sync_broker, clock, wake=explode)
    items = rig.start()
    assert len(items) == 1 and isinstance(items[0], ml.ConnectEvent)
    sync_broker.publish(TOPICS.cmd, b'{}', qos=1)
    assert len(rig.link.drain()) == 1, 'the item was queued although the wake raised'
    assert any(r.exc_info is not None for r in records(errors, logging.ERROR))
    assert sync_broker.callback_errors == [], 'nothing escaped into the fake client'


def test_threaded_delivery_wakes_from_the_network_thread(
    broker: FakeBroker, clock: FakeClock
) -> None:
    rig = Rig(broker, clock)
    items = rig.start()
    assert len(items) == 1 and isinstance(items[0], ml.ConnectEvent)
    broker.publish(TOPICS.cmd, b'{"schema":1}', qos=1)
    assert broker.wait_idle()
    assert rig.wakes == 2
    assert rig.wake_threads[-1] != threading.get_ident(), 'the wake ran on the delivery thread'
    items = messages(rig.link.drain())
    assert len(items) == 1 and items[0].payload == b'{"schema":1}'
    rig.link.shutdown()
    assert broker.retained_payload(TOPICS.availability) == b'offline'


# --------------------------------------------------------------------------- disconnects and takeover


def test_plain_disconnect_warns_and_queues_an_event(
    rig: Rig, errors: pytest.LogCaptureFixture
) -> None:
    rig.start()
    rig.broker.drop_connection(CLIENT_ID)
    assert not rig.link.connected
    items = rig.link.drain()
    assert len(items) == 1 and isinstance(items[0], ml.DisconnectEvent)
    event = items[0]
    assert event.reason_code == REASON_UNSPECIFIED_ERROR
    assert not event.taken_over and not event.exit_requested and not event.from_server
    assert rig.counters.takeovers == 0 and not rig.link.should_exit
    assert any('paho reconnects' in r.getMessage() for r in records(errors, logging.WARNING))
    assert records(errors, logging.ERROR) == []
    assert rig.broker.retained_payload(TOPICS.availability) == b'offline', 'the will fired'


def test_session_takeover_logs_counts_and_backs_off(
    rig: Rig, errors: pytest.LogCaptureFixture
) -> None:
    rig.start()
    rig.intruder().connect(HOST)
    assert rig.counters.takeovers == 1
    assert rig.link.takeovers_in_window == 1
    assert rig.client.reconnect_delay == (2, 30), 'minimum delay doubled once'
    assert not rig.link.should_exit and rig.link.exit_reason is None
    assert not rig.link.connected
    items = rig.link.drain()
    assert len(items) == 1
    assert items[0] == ml.DisconnectEvent(
        items[0].rx_ts_ms,
        ml.REASON_SESSION_TAKEN_OVER,
        'Session taken over',
        True,
        True,
        False,
    )
    message = records(errors, logging.ERROR)[-1].getMessage()
    assert CLIENT_ID in message and 'robot.id' in message and 'Session taken over' in message
    assert 'Takeover 1 of 3' in message and 'after 2 s' in message
    assert rig.broker.retained_payload(TOPICS.availability) == b'offline', 'the will fired'


def flap(rig: Rig, intruder: FakeMqttClient, times: int) -> None:
    """Make the link's client and the intruder take the session from each other ``times`` times."""
    for index in range(times):
        if index == 0:
            intruder.connect(HOST)
        else:
            rig.client.reconnect()
            intruder.reconnect()


def test_takeover_threshold_requests_exit(rig: Rig, errors: pytest.LogCaptureFixture) -> None:
    rig.start()
    intruder = rig.intruder()
    flap(rig, intruder, 2)
    assert rig.counters.takeovers == 2 and not rig.link.should_exit
    assert rig.client.reconnect_delay == (4, 30)
    rig.client.reconnect()
    intruder.reconnect()
    assert rig.counters.takeovers == 3
    assert rig.link.should_exit
    assert rig.link.exit_reason is not None and '3 times' in rig.link.exit_reason
    assert rig.client.reconnect_delay == (8, 30)
    events = [i for i in rig.link.drain() if isinstance(i, ml.DisconnectEvent)]
    assert [e.exit_requested for e in events] == [False, False, True]
    assert all(e.taken_over for e in events)
    assert 'will now exit' in records(errors, logging.ERROR)[-1].getMessage()

    reason = rig.link.exit_reason
    rig.client.reconnect()
    intruder.reconnect()
    assert rig.counters.takeovers == 4 and rig.link.exit_reason == reason


def test_takeover_backoff_is_capped_at_reconnect_max(
    sync_broker: FakeBroker, clock: FakeClock
) -> None:
    rig = Rig(sync_broker, clock, reconnect_min_s=8, reconnect_max_s=10)
    rig.start()
    rig.intruder().connect(HOST)
    assert rig.client.reconnect_delay == (10, 10)


def test_takeovers_outside_the_window_are_forgotten(
    sync_broker: FakeBroker, clock: FakeClock
) -> None:
    rig = Rig(sync_broker, clock, takeover_exit_after=2)
    rig.start()
    intruder = rig.intruder()
    flap(rig, intruder, 1)
    assert rig.link.takeovers_in_window == 1
    clock.advance(ml.TAKEOVER_WINDOW_S + 1)
    assert rig.link.takeovers_in_window == 0
    rig.client.reconnect()
    intruder.reconnect()
    assert rig.counters.takeovers == 2, 'the lifetime counter keeps every takeover'
    assert rig.link.takeovers_in_window == 1 and not rig.link.should_exit
    rig.client.reconnect()
    intruder.reconnect()
    assert rig.link.takeovers_in_window == 2
    assert rig.link.should_exit, 'two takeovers inside the window reach the threshold'


def test_takeover_exit_can_be_disabled(sync_broker: FakeBroker, clock: FakeClock) -> None:
    rig = Rig(sync_broker, clock, takeover_exit_after=0)
    rig.start()
    intruder = rig.intruder()
    flap(rig, intruder, 5)
    assert rig.counters.takeovers == 5 and not rig.link.should_exit
    assert rig.client.reconnect_delay == (30, 30)


def test_backoff_is_restored_after_a_quiet_window(
    sync_broker: FakeBroker, clock: FakeClock, errors: pytest.LogCaptureFixture
) -> None:
    rig = Rig(sync_broker, clock)
    rig.start()
    intruder = rig.intruder()
    intruder.connect(HOST)
    assert rig.client.reconnect_delay == (2, 30)
    intruder.disconnect()
    clock.advance(ml.TAKEOVER_WINDOW_S + 1)
    rig.client.reconnect()
    assert rig.link.connected
    assert rig.client.reconnect_delay == (1, 30), 'restored at the first connect after the window'
    assert any('restored' in r.getMessage() for r in records(errors, logging.INFO))

    intruder.reconnect()
    assert rig.link.takeovers_in_window == 1 and rig.client.reconnect_delay == (2, 30)
    intruder.disconnect()
    rig.client.reconnect()
    assert rig.client.reconnect_delay == (2, 30), 'a connect inside the window keeps the backoff'
    clock.advance(ml.TAKEOVER_WINDOW_S + 1)
    sync_broker.drop_connection(CLIENT_ID)
    assert rig.client.reconnect_delay == (1, 30), 'restored at an ordinary disconnect too'


# --------------------------------------------------------------------------- connection failures


def test_connect_failure_codes_are_logged_once_each_in_plain_language(
    rig: Rig, errors: pytest.LogCaptureFixture
) -> None:
    rig.broker.connack_reason_code = REASON_BAD_CREDENTIALS
    items = rig.start()
    assert not rig.link.connected and rig.link.connack_seen
    assert len(items) == 1 and isinstance(items[0], ml.ConnectEvent) and not items[0].ok
    assert items[0].reason_code == REASON_BAD_CREDENTIALS
    rig.client.reconnect()
    rig.client.reconnect()
    bad = [r for r in records(errors, logging.ERROR) if 'user name or password' in r.getMessage()]
    assert len(bad) == 1, 'one ERROR for the code, then DEBUG'
    assert 'ROS2_HA_BRIDGE_MQTT_PASSWORD' in bad[0].getMessage()
    assert len([i for i in rig.link.drain() if isinstance(i, ml.ConnectEvent)]) == 2

    rig.broker.connack_reason_code = REASON_NOT_AUTHORIZED
    rig.client.reconnect()
    rig.client.reconnect()
    unauthorised = [
        r for r in records(errors, logging.ERROR) if 'not authorised' in r.getMessage()
    ]
    assert len(unauthorised) == 1

    rig.broker.connack_reason_code = REASON_SERVER_UNAVAILABLE
    rig.client.reconnect()
    assert len([r for r in records(errors, logging.ERROR) if 'unavailable' in r.getMessage()]) == 1

    rig.broker.connack_reason_code = 0
    rig.client.reconnect()
    assert rig.link.connected and rig.link.connects == 1
    rig.broker.drop_connection(CLIENT_ID)
    rig.broker.connack_reason_code = REASON_BAD_CREDENTIALS
    rig.client.reconnect()
    bad = [r for r in records(errors, logging.ERROR) if 'user name or password' in r.getMessage()]
    assert len(bad) == 2, 'a success resets the once-per-code memory'


def test_unknown_failure_code_falls_back_to_its_name(
    rig: Rig, errors: pytest.LogCaptureFixture
) -> None:
    rig.broker.connack_reason_code = 131
    assert 131 not in ml.CONNECT_FAILURE_TEXT
    rig.start()
    message = records(errors, logging.ERROR)[-1].getMessage()
    assert '131' in message and "'Code 131'" in message


def test_unreachable_broker_is_logged_once(rig: Rig, errors: pytest.LogCaptureFixture) -> None:
    rig.broker.unreachable = True
    assert rig.start() == [], 'no CONNACK, no event'
    assert not rig.link.connack_seen and not rig.link.connected
    rig.client.reconnect()
    unreachable = [r for r in records(errors, logging.ERROR) if 'cannot reach' in r.getMessage()]
    assert len(unreachable) == 1
    assert f'{HOST}:1883' in unreachable[0].getMessage()
    rig.broker.unreachable = False
    rig.client.reconnect()
    assert rig.link.connected
    items = rig.link.drain()
    assert len(items) == 1 and isinstance(items[0], ml.ConnectEvent) and items[0].ok


# --------------------------------------------------------------------------- shutdown


class RecordingClient(FakeMqttClient):
    """Records the order of the calls shutdown makes."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.calls: list[tuple[Any, ...]] = []

    def publish(
        self,
        topic: str,
        payload: Any = None,
        qos: int = 0,
        retain: bool = False,
        properties: Any = None,
    ) -> FakeMessageInfo:
        self.calls.append(('publish', topic, payload, qos, retain))
        return super().publish(topic, payload, qos, retain, properties)

    def disconnect(self, reasoncode: Any = None, properties: Any = None) -> int:
        self.calls.append(('disconnect',))
        return super().disconnect(reasoncode, properties)

    def loop_stop(self) -> int:
        self.calls.append(('loop_stop',))
        return super().loop_stop()


def recording_rig(broker: FakeBroker, clock: FakeClock, **overrides: Any) -> Rig:
    def factory(*args: Any, **kwargs: Any) -> FakeMqttClient:
        return RecordingClient(*args, broker=broker, synchronous=True, **kwargs)

    return Rig(broker, clock, client_factory=factory, **overrides)


def test_shutdown_publishes_offline_then_disconnects_then_stops_the_loop(
    sync_broker: FakeBroker, clock: FakeClock, errors: pytest.LogCaptureFixture
) -> None:
    rig = recording_rig(sync_broker, clock)
    rig.start()
    client = rig.client
    assert isinstance(client, RecordingClient)
    client.calls.clear()
    rig.link.shutdown()
    assert client.calls == [
        ('publish', TOPICS.availability, b'offline', 1, True),
        ('disconnect',),
        ('loop_stop',),
    ]
    assert rig.link.closed and not rig.link.connected
    assert sync_broker.retained_payload(TOPICS.availability) == b'offline'
    offline = [r for r in sync_broker.messages_on(TOPICS.availability) if r.payload == b'offline']
    assert len(offline) == 1 and offline[0].sender == CLIENT_ID, 'clean: the will did not fire'
    assert not sync_broker.is_connected(CLIENT_ID)
    assert client.loop_stop() == MQTT_ERR_INVAL, 'the loop had already been stopped'
    assert records(errors, logging.WARNING) == []

    client.calls.clear()
    rig.link.shutdown()
    assert client.calls == [], 'a second shutdown does nothing'


def test_shutdown_when_disconnected_relies_on_the_will(
    sync_broker: FakeBroker, clock: FakeClock, errors: pytest.LogCaptureFixture
) -> None:
    rig = recording_rig(sync_broker, clock)
    rig.start()
    sync_broker.drop_connection(CLIENT_ID)
    client = rig.client
    assert isinstance(client, RecordingClient)
    client.calls.clear()
    rig.link.shutdown()
    assert [c[0] for c in client.calls] == ['disconnect', 'loop_stop']
    assert sync_broker.retained_payload(TOPICS.availability) == b'offline', 'from the will'
    assert any('will' in r.getMessage() for r in records(errors, logging.INFO))


def test_shutdown_never_raises(
    sync_broker: FakeBroker, clock: FakeClock, errors: pytest.LogCaptureFixture
) -> None:
    class HostileClient(FakeMqttClient):
        """Raises from every shutdown step once; the fixture's own loop_stop at teardown then works."""

        def __init__(self, *args: Any, **kwargs: Any) -> None:
            super().__init__(*args, **kwargs)
            self.loop_stop_raised = False

        def publish(self, *args: Any, **kwargs: Any) -> FakeMessageInfo:
            raise OSError('broken pipe')

        def disconnect(self, *args: Any, **kwargs: Any) -> int:
            raise OSError('already gone')

        def loop_stop(self) -> int:
            if not self.loop_stop_raised:
                self.loop_stop_raised = True
                raise RuntimeError('thread refused to join')
            return super().loop_stop()

    def factory(*args: Any, **kwargs: Any) -> FakeMqttClient:
        return HostileClient(*args, broker=sync_broker, synchronous=True, **kwargs)

    rig = Rig(sync_broker, clock, client_factory=factory)
    rig.start()
    rig.link.shutdown()
    assert rig.link.closed
    lines = [r.getMessage() for r in records(errors, logging.WARNING)]
    assert any('broken pipe' in m for m in lines)
    assert any('already gone' in m for m in lines)
    assert any('thread refused' in m for m in lines)


def test_shutdown_warns_when_offline_is_not_acknowledged(
    sync_broker: FakeBroker, clock: FakeClock, errors: pytest.LogCaptureFixture
) -> None:
    class SilentClient(FakeMqttClient):
        def publish(self, *args: Any, **kwargs: Any) -> FakeMessageInfo:
            return FakeMessageInfo(1)  # accepted (rc 0) but never acknowledged

    def factory(*args: Any, **kwargs: Any) -> FakeMqttClient:
        return SilentClient(*args, broker=sync_broker, synchronous=True, **kwargs)

    rig = Rig(sync_broker, clock, client_factory=factory)
    rig.start()
    rig.link.shutdown(offline_wait_s=0.01)
    assert any('not acknowledged' in r.getMessage() for r in records(errors, logging.WARNING))


def test_shutdown_before_start_only_disconnects(rig: Rig) -> None:
    rig.link.shutdown()
    assert rig.link.closed and not rig.link.started
    rig.link.start()
    assert not rig.link.started, 'a closed link does not start'
