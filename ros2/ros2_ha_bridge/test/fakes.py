# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Test doubles for the rclpy-free core: fake clock, scheduler, MQTT 5 broker and paho client.

``FakeClock`` and ``FakeScheduler`` implement the ``Clock`` and ``Scheduler``
protocols of :mod:`ros2_ha_bridge.clock`. Time moves only when a test calls
``clock.advance()``, which fires due timers in order, so every timeout, grace
period and rate limit in the core is deterministic.

``FakeBroker`` is an in-memory MQTT 5 broker and ``FakeMqttClient`` the subset
of paho-mqtt 2.x that the phase 2 ``mqtt_link`` needs. Together they model:

- topic filters with ``+`` and ``#`` (including the ``$``-topic rule);
- the retained store, with an empty retained publish clearing a topic, and
  retained delivery on subscribe honouring ``retainHandling`` (0, 1, 2);
- Retain As Published per subscription: a retained message delivered live
  carries ``retain=True`` only on a subscription that asked for it, while a
  retained message replayed on subscribe always carries ``retain=True``;
- MQTT 5 Message Expiry Interval, read from a ``properties`` object's
  ``MessageExpiryInterval`` (paho ``Properties`` or :class:`FakeProperties`),
  applied to the retained store by the fake clock, and passed on to receivers
  as the remaining interval;
- ``noLocal`` subscriptions and QoS downgrade to the subscription's QoS;
- the Last Will: registered by ``will_set`` before connecting, published on
  an unclean disconnect (``drop_connection``) and on session takeover, never
  on a clean ``disconnect()``;
- session takeover: connecting a second client with the same client id
  disconnects the first with reason code 142 (``Session taken over``) as a
  server-sent DISCONNECT, publishes its will and counts ``takeovers``;
- QoS 1 duplicate injection (``broker.duplicate_qos1``) delivering every QoS
  1 message twice, the second copy with ``dup=True``, and ``redeliver()`` for a
  publisher-side resend of a specific earlier message;
- a refused CONNACK (``broker.connack_reason_code``) for the connect error
  paths, and a broker that cannot be reached at all (``broker.unreachable``),
  which produces no CONNACK and calls ``on_connect_fail`` as paho does;
- ``enable_logger`` and ``disable_logger``, recorded in ``client.logger``;
- paho's ``publish()`` validation (``ValueError`` for wildcards, an over-long
  topic, a bad QoS, ``TypeError`` for a bad payload type) and its behaviour
  while disconnected: QoS 1 messages are queued with ``rc=MQTT_ERR_NO_CONN``
  and sent with ``dup=True`` on the next successful connect, QoS 0 messages
  are dropped; ``MQTTMessageInfo.wait_for_publish`` raising ``RuntimeError``
  for a non-zero ``rc`` and otherwise waiting with a timeout;
- callbacks with the VERSION2 signatures, delivered from a real second
  thread started by ``loop_start()`` as paho does, or inline on the calling
  thread when the client (or the broker it came from) is ``synchronous``.

Not modelled, on purpose: automatic reconnection (``reconnect_delay_set`` is
recorded and the test calls ``reconnect()`` itself), keepalive and ping,
persistent sessions and offline queueing on the broker (the node connects with
``clean_start=True``), QoS 2, authentication, the ``$SYS`` tree, and expiry
of messages already queued for a connected client.
"""

from __future__ import annotations

import enum
import itertools
import logging
import queue
import threading
import time
import uuid
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import Any

LOGGER = logging.getLogger(__name__)

#: Epoch of the golden examples (2025-10-13T12:00:00Z); the default FakeClock start.
DEFAULT_EPOCH_MS = 1_760_356_800_000

# paho-mqtt 2.x constants the node uses, same names and values as paho.mqtt.client.
MQTTv311 = 4
MQTTv5 = 5
MQTT_ERR_SUCCESS = 0
MQTT_ERR_NOMEM = 1
MQTT_ERR_PROTOCOL = 2
MQTT_ERR_INVAL = 3
MQTT_ERR_NO_CONN = 4
MQTT_ERR_CONN_REFUSED = 5
MQTT_ERR_NOT_FOUND = 6
MQTT_ERR_CONN_LOST = 7
MQTT_ERR_AGAIN = 11
MQTT_ERR_QUEUE_SIZE = 15

# paho.mqtt.subscribeoptions retain-handling values.
RETAIN_SEND_ON_SUBSCRIBE = 0
RETAIN_SEND_IF_NEW_SUB = 1
RETAIN_DO_NOT_SEND = 2

# MQTT 5 reason codes the tests care about.
REASON_SUCCESS = 0
REASON_UNSPECIFIED_ERROR = 128
REASON_BAD_CREDENTIALS = 134
REASON_NOT_AUTHORIZED = 135
REASON_SERVER_UNAVAILABLE = 136
REASON_KEEP_ALIVE_TIMEOUT = 141
REASON_SESSION_TAKEN_OVER = 142

REASON_NAMES: dict[int, str] = {
    0: 'Success',
    128: 'Unspecified error',
    134: 'Bad user name or password',
    135: 'Not authorized',
    136: 'Server unavailable',
    141: 'Keep alive timeout',
    142: 'Session taken over',
}

MAX_TOPIC_BYTES = 65535


# --------------------------------------------------------------------------- paho stand-ins


class FakeCallbackAPIVersion(enum.Enum):
    """Stand-in for ``paho.mqtt.enums.CallbackAPIVersion``; accepted and recorded only."""

    VERSION1 = 1
    VERSION2 = 2


class FakeReasonCode:
    """Stand-in for paho's ``ReasonCode``: ``value``, ``getName()`` and equality with an int."""

    def __init__(self, value: int, name: str | None = None) -> None:
        self.value = int(value)
        self._name = name if name is not None else REASON_NAMES.get(self.value, f'Code {value}')

    def getName(self) -> str:
        return self._name

    @property
    def is_failure(self) -> bool:
        return self.value >= 128

    def __eq__(self, other: object) -> bool:
        if isinstance(other, FakeReasonCode):
            return self.value == other.value
        if isinstance(other, int):
            return self.value == other
        return NotImplemented

    def __hash__(self) -> int:
        return hash(self.value)

    def __int__(self) -> int:
        return self.value

    def __repr__(self) -> str:
        return f'FakeReasonCode({self.value}, {self._name!r})'

    def __str__(self) -> str:
        return self._name


class FakeProperties:
    """Stand-in for ``paho.mqtt.properties.Properties``: attributes set by name.

    Only ``MessageExpiryInterval`` is interpreted by the fake broker; it is read
    with ``getattr(properties, 'MessageExpiryInterval', None)`` so a real paho
    ``Properties`` works too.
    """

    MessageExpiryInterval: int | None = None

    def __init__(self, packet_type: Any = None, **values: Any) -> None:
        self.packetType = packet_type
        for name, value in values.items():
            setattr(self, name, value)

    def __repr__(self) -> str:
        values = {k: v for k, v in vars(self).items() if k != 'packetType' and v is not None}
        return f'FakeProperties({values})'


class FakeSubscribeOptions:
    """Stand-in for ``paho.mqtt.subscribeoptions.SubscribeOptions`` (same attribute names)."""

    def __init__(
        self,
        qos: int = 0,
        noLocal: bool = False,
        retainAsPublished: bool = False,
        retainHandling: int = RETAIN_SEND_ON_SUBSCRIBE,
    ) -> None:
        self.QoS = qos
        self.noLocal = noLocal
        self.retainAsPublished = retainAsPublished
        self.retainHandling = retainHandling

    def __repr__(self) -> str:
        return (
            f'FakeSubscribeOptions(qos={self.QoS}, noLocal={self.noLocal}, '
            f'retainAsPublished={self.retainAsPublished}, retainHandling={self.retainHandling})'
        )


@dataclass(frozen=True)
class FakeConnectFlags:
    """Stand-in for paho's ``ConnectFlags``."""

    is_session_present: bool = False


@dataclass(frozen=True)
class FakeDisconnectFlags:
    """Stand-in for paho's ``DisconnectFlags``."""

    is_disconnect_packet_from_server: bool = False


@dataclass
class FakeMQTTMessage:
    """Stand-in for ``paho.mqtt.client.MQTTMessage`` as handed to ``on_message``."""

    topic: str
    payload: bytes
    qos: int = 0
    retain: bool = False
    mid: int = 0
    dup: bool = False
    properties: FakeProperties | None = None
    timestamp: float = 0.0


class FakeMessageInfo:
    """Stand-in for ``MQTTMessageInfo``: ``rc``, ``mid``, ``is_published``, ``wait_for_publish``.

    As in paho, ``wait_for_publish`` raises ``ValueError`` for
    ``MQTT_ERR_QUEUE_SIZE`` and ``RuntimeError`` for any other non-zero ``rc``
    (including ``MQTT_ERR_NO_CONN`` for a message queued while disconnected,
    even after a reconnect has delivered it), and otherwise waits for the
    message to be published or the timeout to pass.
    """

    def __init__(self, mid: int) -> None:
        self.mid = mid
        self.rc = MQTT_ERR_SUCCESS
        self._published = threading.Event()

    def is_published(self) -> bool:
        return self._published.is_set()

    def wait_for_publish(self, timeout: float | None = None) -> None:
        if self.rc == MQTT_ERR_QUEUE_SIZE:
            raise ValueError('Message is not queued due to ERR_QUEUE_SIZE')
        if self.rc == MQTT_ERR_AGAIN:
            pass
        elif self.rc > 0:
            raise RuntimeError(f'Message publish failed: error code {self.rc}')
        self._published.wait(timeout)

    def _mark_published(self) -> None:
        self._published.set()


# --------------------------------------------------------------------------- clock and scheduler


class FakeClock:
    """A settable clock. ``advance()`` moves both clocks and fires attached schedulers' timers.

    ``now_ms`` is derived from the monotonic clock plus an offset, so advancing
    moves both together; ``set_now_ms`` steps the wall clock alone (an NTP
    correction) and ``set_monotonic`` jumps the monotonic clock without firing
    anything.
    """

    def __init__(self, now_ms: int = DEFAULT_EPOCH_MS, monotonic: float = 1000.0) -> None:
        self._monotonic = monotonic
        self._base_ms = now_ms
        self._base_monotonic = monotonic
        self._schedulers: list[FakeScheduler] = []
        self._lock = threading.RLock()

    def now_ms(self) -> int:
        with self._lock:
            return self._base_ms + round((self._monotonic - self._base_monotonic) * 1000)

    def monotonic(self) -> float:
        with self._lock:
            return self._monotonic

    def set_now_ms(self, value: int) -> None:
        """Step the wall clock to ``value`` without moving the monotonic clock."""
        with self._lock:
            self._base_ms = value
            self._base_monotonic = self._monotonic

    def set_monotonic(self, value: float) -> None:
        """Jump the monotonic clock without firing timers (the wall clock follows)."""
        with self._lock:
            if value < self._monotonic:
                raise ValueError('a monotonic clock never goes backwards')
            self._monotonic = value

    def attach(self, scheduler: FakeScheduler) -> None:
        with self._lock:
            if scheduler not in self._schedulers:
                self._schedulers.append(scheduler)

    def advance(self, seconds: float, *, max_fires: int = 100_000) -> int:
        """Move time forward by ``seconds``, firing every timer due on the way, in due order.

        Timers scheduled or cancelled by a firing callback are honoured within
        the same call, so a chain of self-rescheduling one-shots runs to the
        target time. Returns the number of callbacks fired. ``advance(0)``
        fires whatever is already due.
        """
        if seconds < 0:
            raise ValueError('cannot advance a clock by a negative amount')
        with self._lock:
            target = self._monotonic + seconds
        fired = 0
        while True:
            # The lock is never held while a callback runs: a callback may publish on a
            # FakeBroker whose delivery thread is, at that moment, reading this clock.
            with self._lock:
                due_times = [s.next_due() for s in self._schedulers]
                pending = [due for due in due_times if due is not None]
                next_due = min(pending) if pending else None
                if next_due is None or next_due > target:
                    self._monotonic = max(self._monotonic, target)
                    return fired
                self._monotonic = max(self._monotonic, next_due)
                schedulers = list(self._schedulers)
            for scheduler in schedulers:
                fired += scheduler.fire_due()
            if fired > max_fires:
                raise RuntimeError(f'more than {max_fires} timer callbacks in one advance')


class FakeTimer:
    """A :class:`ros2_ha_bridge.clock.TimerHandle` created by :class:`FakeScheduler`."""

    def __init__(self, scheduler: FakeScheduler, due: float, fn: Callable[[], None], seq: int):
        self._scheduler = scheduler
        self.due = due
        self.fn = fn
        self.seq = seq
        self.fired = False
        self.cancelled = False

    @property
    def active(self) -> bool:
        return not (self.fired or self.cancelled)

    def cancel(self) -> None:
        if self.active:
            self.cancelled = True
            self._scheduler._remove(self)

    def remaining_s(self) -> float:
        if not self.active:
            return 0.0
        return max(0.0, self.due - self._scheduler.clock.monotonic())

    def __repr__(self) -> str:
        state = 'fired' if self.fired else 'cancelled' if self.cancelled else 'pending'
        return f'FakeTimer(due={self.due}, {state})'


class FakeScheduler:
    """A :class:`ros2_ha_bridge.clock.Scheduler` driven by a :class:`FakeClock`.

    Timers fire from ``clock.advance()`` on the advancing thread, in order of
    due time (ties in creation order). A callback may schedule or cancel other
    timers, including itself, while firing.
    """

    def __init__(self, clock: FakeClock) -> None:
        self.clock = clock
        self._timers: list[FakeTimer] = []
        self._seq = itertools.count()
        self.fired_count = 0
        clock.attach(self)

    def call_later(self, delay_s: float, fn: Callable[[], None]) -> FakeTimer:
        if delay_s < 0:
            raise ValueError('delay must not be negative')
        timer = FakeTimer(self, self.clock.monotonic() + delay_s, fn, next(self._seq))
        self._timers.append(timer)
        return timer

    @property
    def pending(self) -> list[FakeTimer]:
        """Active timers, soonest first."""
        return sorted((t for t in self._timers if t.active), key=lambda t: (t.due, t.seq))

    def next_due(self) -> float | None:
        pending = self.pending
        return pending[0].due if pending else None

    def fire_due(self) -> int:
        """Fire every active timer whose due time has arrived; return how many fired."""
        fired = 0
        while True:
            now = self.clock.monotonic()
            due_now = [t for t in self.pending if t.due <= now]
            if not due_now:
                return fired
            timer = due_now[0]
            timer.fired = True
            self._remove(timer)
            fired += 1
            self.fired_count += 1
            timer.fn()

    def cancel_all(self) -> None:
        for timer in list(self._timers):
            timer.cancel()

    def _remove(self, timer: FakeTimer) -> None:
        if timer in self._timers:
            self._timers.remove(timer)


# --------------------------------------------------------------------------- topic matching


def validate_filter(topic_filter: str) -> None:
    """Raise ``ValueError('Invalid subscription filter.')`` as paho does for a bad filter."""
    if not isinstance(topic_filter, str) or not topic_filter or '\x00' in topic_filter:
        raise ValueError('Invalid subscription filter.')
    segments = topic_filter.split('/')
    for index, segment in enumerate(segments):
        if '#' in segment and (segment != '#' or index != len(segments) - 1):
            raise ValueError('Invalid subscription filter.')
        if '+' in segment and segment != '+':
            raise ValueError('Invalid subscription filter.')


def topic_matches(topic_filter: str, topic: str) -> bool:
    """MQTT topic-filter matching.

    ``+`` matches one level and ``#`` the rest; a topic starting with ``$`` is
    never matched by a filter that starts with a wildcard.
    """
    if topic_filter == topic:
        return True
    if topic.startswith('$') and topic_filter[:1] in ('+', '#'):
        return False
    filter_parts = topic_filter.split('/')
    topic_parts = topic.split('/')
    for index, part in enumerate(filter_parts):
        if part == '#':
            return True
        if index >= len(topic_parts):
            return False
        if part != '+' and part != topic_parts[index]:
            return False
    return len(filter_parts) == len(topic_parts)


# --------------------------------------------------------------------------- broker records


@dataclass(frozen=True)
class WillSpec:
    """A client's Last Will as registered by ``will_set``."""

    topic: str
    payload: bytes
    qos: int = 0
    retain: bool = False
    properties: Any = None


@dataclass(frozen=True)
class PublishRecord:
    """Every publish the broker saw, in order, for assertions."""

    topic: str
    payload: bytes
    qos: int
    retain: bool
    expiry_s: int | None
    sender: str | None
    monotonic: float
    mid: int
    dup: bool = False

    @property
    def text(self) -> str:
        return self.payload.decode('utf-8')


@dataclass
class RetainedMessage:
    """An entry of the retained store."""

    topic: str
    payload: bytes
    qos: int
    stored_at: float
    expires_at: float | None
    properties: Any = None


@dataclass
class Subscription:
    """One subscription of one session."""

    filter: str
    qos: int = 0
    no_local: bool = False
    retain_as_published: bool = False
    retain_handling: int = RETAIN_SEND_ON_SUBSCRIBE


@dataclass
class _Session:
    client: FakeMqttClient
    will: WillSpec | None
    subscriptions: list[Subscription] = field(default_factory=list)


def _expiry_from(properties: Any) -> int | None:
    value = getattr(properties, 'MessageExpiryInterval', None)
    if value is None:
        return None
    return int(value)


def _coerce_payload(payload: Any) -> bytes:
    """paho's payload rules: str, bytes, bytearray, int, float or None."""
    if payload is None:
        return b''
    if isinstance(payload, str):
        return payload.encode('utf-8')
    if isinstance(payload, (bytes, bytearray)):
        return bytes(payload)
    if isinstance(payload, (int, float)) and not isinstance(payload, bool):
        return str(payload).encode('ascii')
    raise TypeError('payload must be a string, bytearray, int, float or None.')


def _raise_for_invalid_topic(topic: Any) -> str:
    """paho's publish-topic validation, same messages."""
    if not isinstance(topic, str) or not topic:
        raise ValueError('Invalid topic.')
    if len(topic.encode('utf-8')) > MAX_TOPIC_BYTES:
        raise ValueError('Publish topic is too long.')
    if '+' in topic or '#' in topic:
        raise ValueError('Publish topic cannot contain wildcards.')
    return topic


# --------------------------------------------------------------------------- the broker


class FakeBroker:
    """An in-memory MQTT 5 broker for one test (see the module docstring for what is modelled)."""

    def __init__(self, clock: FakeClock, *, synchronous: bool = False) -> None:
        self.clock = clock
        #: Default delivery mode for clients made by :meth:`make_client`.
        self.synchronous = synchronous
        self.retained: dict[str, RetainedMessage] = {}
        self.published: list[PublishRecord] = []
        self.clients: list[FakeMqttClient] = []
        self.takeovers = 0
        #: When True every QoS 1 delivery happens twice, the second copy with ``dup=True``.
        self.duplicate_qos1 = False
        #: Non-zero makes every connect attempt fail with this CONNACK reason code.
        self.connack_reason_code = REASON_SUCCESS
        #: When True no connect attempt reaches the broker: the client's ``on_connect_fail``
        #: runs instead of ``on_connect`` (a TCP-level failure, which carries no reason code).
        self.unreachable = False
        self._sessions: dict[str, _Session] = {}
        self._lock = threading.RLock()
        self._mids = itertools.count(1)

    # ----- factories -----------------------------------------------------------------

    def make_client(self, *args: Any, **kwargs: Any) -> FakeMqttClient:
        """Create a :class:`FakeMqttClient` bound to this broker (paho ``Client`` arguments)."""
        kwargs.setdefault('synchronous', self.synchronous)
        return FakeMqttClient(*args, broker=self, **kwargs)

    @property
    def client_factory(self) -> Callable[..., FakeMqttClient]:
        """A drop-in for ``paho.mqtt.client.Client``: same positional and keyword arguments."""
        return self.make_client

    # ----- observation ---------------------------------------------------------------

    def retained_payload(self, topic: str) -> bytes | None:
        """The retained payload on ``topic`` or ``None`` (expired entries count as absent)."""
        with self._lock:
            self._purge_expired()
            entry = self.retained.get(topic)
            return None if entry is None else entry.payload

    def messages_on(self, topic_filter: str) -> list[PublishRecord]:
        """Every publish whose topic matches ``topic_filter``, oldest first."""
        with self._lock:
            return [r for r in self.published if topic_matches(topic_filter, r.topic)]

    def last_on(self, topic_filter: str) -> PublishRecord | None:
        records = self.messages_on(topic_filter)
        return records[-1] if records else None

    def connected_client_ids(self) -> list[str]:
        with self._lock:
            return sorted(self._sessions)

    def is_connected(self, client_id: str) -> bool:
        with self._lock:
            return client_id in self._sessions

    def subscriptions_of(self, client_id: str) -> list[Subscription]:
        with self._lock:
            session = self._sessions.get(client_id)
            return [] if session is None else list(session.subscriptions)

    @property
    def callback_errors(self) -> list[BaseException]:
        """Exceptions raised inside any client's callbacks (a test should assert this is empty)."""
        return [error for client in self.clients for error in client.callback_errors]

    def wait_idle(self, timeout: float = 2.0) -> bool:
        """Wait until every client's delivery thread has drained its queue."""
        deadline = time.monotonic() + timeout
        for client in list(self.clients):
            remaining = max(0.0, deadline - time.monotonic())
            if not client.wait_for_deliveries(remaining):
                return False
        return True

    # ----- actions a test drives directly ---------------------------------------------

    def publish(
        self,
        topic: str,
        payload: Any = None,
        qos: int = 0,
        retain: bool = False,
        *,
        expiry_s: int | None = None,
        properties: Any = None,
    ) -> PublishRecord:
        """Publish as an external party (Home Assistant, a planted retained message)."""
        if properties is None and expiry_s is not None:
            properties = FakeProperties(MessageExpiryInterval=expiry_s)
        return self._publish(
            None,
            _raise_for_invalid_topic(topic),
            _coerce_payload(payload),
            qos,
            retain,
            properties,
        )

    def drop_connection(self, client_id: str) -> None:
        """Cut a client's network connection: its will is published and it sees on_disconnect."""
        with self._lock:
            session = self._sessions.get(client_id)
            if session is None:
                raise KeyError(f'no connected client {client_id!r}')
            client = session.client
        self._disconnect(
            client, publish_will=True, reason_code=REASON_UNSPECIFIED_ERROR, from_server=False
        )

    def evict(self, client_id: str, *, reason_code: int = REASON_SUCCESS) -> None:
        """Broker-initiated DISCONNECT of a connected client; its will is published.

        This is how Mosquitto 2.1 removes the older holder of a duplicated client
        id: a plain broker-sent DISCONNECT with reason code 0 rather than 142.
        """
        with self._lock:
            session = self._sessions.get(client_id)
            if session is None:
                raise KeyError(f'no connected client {client_id!r}')
            client = session.client
        self._disconnect(client, publish_will=True, reason_code=reason_code, from_server=True)

    def redeliver(self, record: PublishRecord) -> None:
        """Deliver an earlier publish again with ``dup=True`` (a publisher-side QoS 1 resend)."""
        with self._lock:
            self._fan_out(record, sender=None, dup=True, force_single=True)

    def expire_retained(self) -> int:
        """Purge expired retained messages now; return how many were removed."""
        with self._lock:
            return self._purge_expired()

    # ----- client-facing operations ---------------------------------------------------

    def _connect(self, client: FakeMqttClient) -> int:
        with self._lock:
            if self.connack_reason_code != REASON_SUCCESS:
                return self.connack_reason_code
            existing = self._sessions.get(client.client_id)
            if existing is not None and existing.client is not client:
                self.takeovers += 1
                self._disconnect(
                    existing.client,
                    publish_will=True,
                    reason_code=REASON_SESSION_TAKEN_OVER,
                    from_server=True,
                )
            self._sessions[client.client_id] = _Session(client, client.will)
            return REASON_SUCCESS

    def _disconnect(
        self, client: FakeMqttClient, *, publish_will: bool, reason_code: int, from_server: bool
    ) -> None:
        with self._lock:
            session = self._sessions.get(client.client_id)
            if session is None or session.client is not client:
                return
            del self._sessions[client.client_id]
            if publish_will and session.will is not None:
                will = session.will
                self._publish(
                    None, will.topic, will.payload, will.qos, will.retain, will.properties
                )
            client._on_broker_disconnect(reason_code, from_server)

    def _subscribe(
        self, client: FakeMqttClient, subscription: Subscription
    ) -> tuple[int, list[FakeMQTTMessage]]:
        validate_filter(subscription.filter)
        with self._lock:
            session = self._sessions.get(client.client_id)
            if session is None or session.client is not client:
                raise RuntimeError('subscribe on a client that is not connected')
            existing = next(
                (s for s in session.subscriptions if s.filter == subscription.filter), None
            )
            is_new = existing is None
            if existing is not None:
                session.subscriptions.remove(existing)
            session.subscriptions.append(subscription)
            self._purge_expired()
            retained: list[FakeMQTTMessage] = []
            send_retained = subscription.retain_handling == RETAIN_SEND_ON_SUBSCRIBE or (
                subscription.retain_handling == RETAIN_SEND_IF_NEW_SUB and is_new
            )
            if send_retained:
                now = self.clock.monotonic()
                for entry in sorted(self.retained.values(), key=lambda e: e.stored_at):
                    if not topic_matches(subscription.filter, entry.topic):
                        continue
                    properties = None
                    if entry.expires_at is not None:
                        remaining = max(0, round(entry.expires_at - now))
                        properties = FakeProperties(MessageExpiryInterval=remaining)
                    retained.append(
                        FakeMQTTMessage(
                            topic=entry.topic,
                            payload=entry.payload,
                            qos=min(entry.qos, subscription.qos),
                            retain=True,
                            mid=next(self._mids),
                            properties=properties,
                            timestamp=now,
                        )
                    )
            return subscription.qos, retained

    def _unsubscribe(self, client: FakeMqttClient, topic_filter: str) -> bool:
        with self._lock:
            session = self._sessions.get(client.client_id)
            if session is None:
                return False
            before = len(session.subscriptions)
            session.subscriptions = [s for s in session.subscriptions if s.filter != topic_filter]
            return len(session.subscriptions) != before

    def _publish(
        self,
        sender: FakeMqttClient | None,
        topic: str,
        payload: bytes,
        qos: int,
        retain: bool,
        properties: Any,
        *,
        dup: bool = False,
    ) -> PublishRecord:
        with self._lock:
            now = self.clock.monotonic()
            expiry = _expiry_from(properties)
            record = PublishRecord(
                topic=topic,
                payload=payload,
                qos=qos,
                retain=retain,
                expiry_s=expiry,
                sender=None if sender is None else sender.client_id,
                monotonic=now,
                mid=next(self._mids),
                dup=dup,
            )
            self.published.append(record)
            if retain:
                if payload:
                    expires_at = None if expiry is None else now + expiry
                    self.retained[topic] = RetainedMessage(
                        topic, payload, qos, now, expires_at, properties
                    )
                else:
                    self.retained.pop(topic, None)
            self._fan_out(record, sender=sender, dup=dup)
            return record

    def _fan_out(
        self,
        record: PublishRecord,
        *,
        sender: FakeMqttClient | None,
        dup: bool,
        force_single: bool = False,
    ) -> None:
        for session in list(self._sessions.values()):
            matching = [
                s
                for s in session.subscriptions
                if topic_matches(s.filter, record.topic)
                and not (s.no_local and sender is not None and sender is session.client)
            ]
            if not matching:
                continue
            qos = min(record.qos, max(s.qos for s in matching))
            retain_flag = record.retain and any(s.retain_as_published for s in matching)
            properties = None
            if record.expiry_s is not None:
                properties = FakeProperties(MessageExpiryInterval=record.expiry_s)
            copies = 1
            if qos >= 1 and self.duplicate_qos1 and not force_single:
                copies = 2
            for index in range(copies):
                session.client._receive(
                    FakeMQTTMessage(
                        topic=record.topic,
                        payload=record.payload,
                        qos=qos,
                        retain=retain_flag,
                        mid=record.mid,
                        dup=dup or index > 0,
                        properties=properties,
                        timestamp=record.monotonic,
                    )
                )

    def _purge_expired(self) -> int:
        now = self.clock.monotonic()
        expired = [
            topic
            for topic, entry in self.retained.items()
            if entry.expires_at is not None and entry.expires_at <= now
        ]
        for topic in expired:
            del self.retained[topic]
        return len(expired)


# --------------------------------------------------------------------------- the client


_STOP = object()


class FakeMqttClient:
    """The paho-mqtt 2.x ``Client`` subset the node uses, wired to a :class:`FakeBroker`.

    Construct it through ``broker.make_client(...)`` or ``broker.client_factory``
    with the arguments the node gives paho; the ``broker`` keyword binds it.
    Callbacks (``on_connect``, ``on_disconnect``, ``on_message``,
    ``on_subscribe``, ``on_unsubscribe``, ``on_publish``) use the VERSION2
    signatures. With ``synchronous=False`` (the default) they run on a
    dedicated thread started by :meth:`loop_start`, as paho's network thread
    would; with ``synchronous=True`` they run inline on whichever thread caused
    them, which makes a test deterministic. Exceptions raised by callbacks are
    recorded in :attr:`callback_errors` and logged; they never kill the thread.
    """

    def __init__(
        self,
        callback_api_version: Any = None,
        client_id: str = '',
        clean_session: bool | None = None,
        userdata: Any = None,
        protocol: int = MQTTv311,
        transport: str = 'tcp',
        reconnect_on_failure: bool = True,
        manual_ack: bool = False,
        *,
        broker: FakeBroker,
        synchronous: bool = False,
    ) -> None:
        self.broker = broker
        self.callback_api_version = callback_api_version
        self.client_id = client_id or f'fake-{uuid.uuid4().hex[:12]}'
        self.clean_session = clean_session
        self.userdata = userdata
        self.protocol = protocol
        self.transport = transport
        self.reconnect_on_failure = reconnect_on_failure
        self.manual_ack = manual_ack
        self.synchronous = synchronous

        # Recorded configuration, for assertions.
        self.username: str | None = None
        self.password: str | None = None
        self.tls_args: dict[str, Any] | None = None
        self.tls_insecure = False
        self.will: WillSpec | None = None
        self.reconnect_delay: tuple[int, int] = (1, 120)
        self.connect_args: dict[str, Any] | None = None
        self.connect_attempts = 0
        self.subscriptions: list[Subscription] = []
        self.callback_errors: list[BaseException] = []
        #: The logger given to ``enable_logger`` (``None`` until then, or after ``disable_logger``).
        self.logger: logging.Logger | None = None

        # Callbacks, VERSION2 signatures.
        self.on_connect: Callable[..., Any] | None = None
        self.on_connect_fail: Callable[..., Any] | None = None
        self.on_disconnect: Callable[..., Any] | None = None
        self.on_message: Callable[..., Any] | None = None
        self.on_subscribe: Callable[..., Any] | None = None
        self.on_unsubscribe: Callable[..., Any] | None = None
        self.on_publish: Callable[..., Any] | None = None

        self._connected = False
        self._pending_connect = False
        self._loop_running = False
        self._thread: threading.Thread | None = None
        self._queue: queue.Queue[Any] = queue.Queue()
        self._pending_out: list[tuple[str, bytes, int, bool, Any, FakeMessageInfo]] = []
        self._mids = itertools.count(1)
        broker.clients.append(self)

    # ----- configuration -------------------------------------------------------------

    def username_pw_set(self, username: str | None, password: str | None = None) -> None:
        self.username = username
        self.password = password

    def user_data_set(self, userdata: Any) -> None:
        self.userdata = userdata

    def enable_logger(self, logger: logging.Logger | None = None) -> None:
        """Record the logger paho would route its own log lines to (nothing is logged here)."""
        self.logger = logger if logger is not None else LOGGER

    def disable_logger(self) -> None:
        self.logger = None

    def tls_set(
        self,
        ca_certs: str | None = None,
        certfile: str | None = None,
        keyfile: str | None = None,
        cert_reqs: Any = None,
        tls_version: Any = None,
        ciphers: str | None = None,
        keyfile_password: Any = None,
        alpn_protocols: Any = None,
    ) -> None:
        if self.tls_args is not None:
            raise ValueError('SSL/TLS has already been configured.')
        self.tls_args = {
            'ca_certs': ca_certs,
            'certfile': certfile,
            'keyfile': keyfile,
            'cert_reqs': cert_reqs,
            'tls_version': tls_version,
            'ciphers': ciphers,
            'keyfile_password': keyfile_password,
            'alpn_protocols': alpn_protocols,
        }

    def tls_insecure_set(self, value: bool) -> None:
        if self.tls_args is None:
            raise ValueError('Must configure SSL context before using tls_insecure_set.')
        self.tls_insecure = value

    def will_set(
        self,
        topic: str,
        payload: Any = None,
        qos: int = 0,
        retain: bool = False,
        properties: Any = None,
    ) -> None:
        if qos not in (0, 1, 2):
            raise ValueError('Invalid QoS level.')
        self.will = WillSpec(
            _raise_for_invalid_topic(topic), _coerce_payload(payload), qos, retain, properties
        )

    def will_clear(self) -> None:
        self.will = None

    def reconnect_delay_set(self, min_delay: int = 1, max_delay: int = 120) -> None:
        self.reconnect_delay = (min_delay, max_delay)

    # ----- connection ----------------------------------------------------------------

    def connect(
        self,
        host: str,
        port: int = 1883,
        keepalive: int = 60,
        bind_address: str = '',
        bind_port: int = 0,
        clean_start: Any = 3,
        properties: Any = None,
    ) -> int:
        """Connect now (the CONNACK callback still runs through the loop, as in paho)."""
        self._record_connect(
            host, port, keepalive, bind_address, bind_port, clean_start, properties
        )
        self._attempt_connect()
        return MQTT_ERR_SUCCESS

    def connect_async(
        self,
        host: str,
        port: int = 1883,
        keepalive: int = 60,
        bind_address: str = '',
        bind_port: int = 0,
        clean_start: Any = 3,
        properties: Any = None,
    ) -> None:
        """Record the connection; the attempt happens in :meth:`loop_start`."""
        self._record_connect(
            host, port, keepalive, bind_address, bind_port, clean_start, properties
        )
        self._pending_connect = True
        if self._loop_running:
            self._attempt_connect()

    def reconnect(self) -> int:
        """Attempt the recorded connection again (the test drives reconnection timing)."""
        if self.connect_args is None:
            raise ValueError('Invalid host.')
        self._attempt_connect()
        return MQTT_ERR_SUCCESS

    def disconnect(self, reasoncode: Any = None, properties: Any = None) -> int:
        """Clean disconnect: no will is published."""
        if not self._connected:
            return MQTT_ERR_NO_CONN
        self.broker._disconnect(
            self, publish_will=False, reason_code=REASON_SUCCESS, from_server=False
        )
        return MQTT_ERR_SUCCESS

    def is_connected(self) -> bool:
        return self._connected

    def simulate_network_loss(self) -> None:
        """Convenience for ``broker.drop_connection(self.client_id)``."""
        self.broker.drop_connection(self.client_id)

    def loop_start(self) -> int:
        """Start the delivery thread (or, when synchronous, just mark the loop running)."""
        if self._loop_running:
            return MQTT_ERR_INVAL
        self._loop_running = True
        if not self.synchronous:
            self._thread = threading.Thread(
                target=self._run, name=f'fake-mqtt-{self.client_id}', daemon=True
            )
            self._thread.start()
        if self._pending_connect:
            self._attempt_connect()
        return MQTT_ERR_SUCCESS

    def loop_stop(self) -> int:
        """Stop the delivery thread after it has drained what is already queued."""
        if not self._loop_running:
            return MQTT_ERR_INVAL
        self._loop_running = False
        thread = self._thread
        if thread is not None:
            self._queue.put(_STOP)
            thread.join(timeout=5.0)
            self._thread = None
        return MQTT_ERR_SUCCESS

    def wait_for_deliveries(self, timeout: float = 2.0) -> bool:
        """Block until queued callbacks have run (always True in synchronous mode)."""
        if self.synchronous:
            return True
        deadline = time.monotonic() + timeout
        while self._queue.unfinished_tasks:
            if self._thread is None or not self._thread.is_alive():
                return self._queue.unfinished_tasks == 0
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.001)
        return True

    # ----- subscribe and publish ------------------------------------------------------

    def subscribe(
        self, topic: Any, qos: int = 0, options: Any = None, properties: Any = None
    ) -> tuple[int, int | None]:
        """``subscribe(topic, qos)``, ``subscribe(topic, options=SubscribeOptions(...))``,
        ``subscribe((topic, qos_or_options))`` or ``subscribe([(topic, ...), ...])``."""
        requests = self._subscription_requests(topic, qos, options)
        if not self._connected:
            return (MQTT_ERR_NO_CONN, None)
        mid = next(self._mids)
        granted: list[int] = []
        retained: list[FakeMQTTMessage] = []
        for subscription in requests:
            self.subscriptions.append(subscription)
            granted_qos, replay = self.broker._subscribe(self, subscription)
            granted.append(granted_qos)
            retained.extend(replay)
        reason_codes = [FakeReasonCode(g, f'Granted QoS {g}') for g in granted]
        self._dispatch(
            lambda: self._call(self.on_subscribe, self, self.userdata, mid, reason_codes, None)
        )
        for message in retained:
            self._receive(message)
        return (MQTT_ERR_SUCCESS, mid)

    def unsubscribe(self, topic: Any, properties: Any = None) -> tuple[int, int | None]:
        filters = [topic] if isinstance(topic, str) else list(topic)
        for item in filters:
            validate_filter(item)
        if not self._connected:
            return (MQTT_ERR_NO_CONN, None)
        mid = next(self._mids)
        for item in filters:
            self.broker._unsubscribe(self, item)
            self.subscriptions = [s for s in self.subscriptions if s.filter != item]
        reason_codes = [FakeReasonCode(0, 'Success') for _ in filters]
        self._dispatch(
            lambda: self._call(self.on_unsubscribe, self, self.userdata, mid, reason_codes, None)
        )
        return (MQTT_ERR_SUCCESS, mid)

    def publish(
        self,
        topic: str,
        payload: Any = None,
        qos: int = 0,
        retain: bool = False,
        properties: Any = None,
    ) -> FakeMessageInfo:
        """Publish with paho's validation and its disconnected-client behaviour."""
        topic = _raise_for_invalid_topic(topic)
        if qos not in (0, 1, 2):
            raise ValueError('Invalid QoS level.')
        data = _coerce_payload(payload)
        info = FakeMessageInfo(next(self._mids))
        if not self._connected:
            info.rc = MQTT_ERR_NO_CONN
            if qos > 0:
                self._pending_out.append((topic, data, qos, retain, properties, info))
            return info
        self.broker._publish(self, topic, data, qos, retain, properties)
        info._mark_published()
        self._dispatch(
            lambda: self._call(
                self.on_publish, self, self.userdata, info.mid, FakeReasonCode(0, 'Success'), None
            )
        )
        return info

    # ----- internals -----------------------------------------------------------------

    def _record_connect(
        self,
        host: str,
        port: int,
        keepalive: int,
        bind_address: str,
        bind_port: int,
        clean_start: Any,
        properties: Any,
    ) -> None:
        if not host:
            raise ValueError('Invalid host.')
        if port <= 0:
            raise ValueError('Invalid port number.')
        if keepalive < 0:
            raise ValueError('Keepalive must be >=0.')
        self.connect_args = {
            'host': host,
            'port': port,
            'keepalive': keepalive,
            'bind_address': bind_address,
            'bind_port': bind_port,
            'clean_start': clean_start,
            'properties': properties,
        }

    def _attempt_connect(self) -> int:
        self._pending_connect = False
        self.connect_attempts += 1
        if self.broker.unreachable:
            # No CONNACK at all: paho calls on_connect_fail from its loop and retries later.
            self._dispatch(lambda: self._call(self.on_connect_fail, self, self.userdata))
            return MQTT_ERR_NO_CONN
        reason = self.broker._connect(self)
        if reason == REASON_SUCCESS:
            self._connected = True
            self._dispatch(
                lambda: self._call(
                    self.on_connect,
                    self,
                    self.userdata,
                    FakeConnectFlags(False),
                    FakeReasonCode(REASON_SUCCESS, 'Success'),
                    None,
                )
            )
            self._flush_pending_out()
        else:
            self._dispatch(
                lambda: self._call(
                    self.on_connect,
                    self,
                    self.userdata,
                    FakeConnectFlags(False),
                    FakeReasonCode(reason),
                    None,
                )
            )
        return reason

    def _flush_pending_out(self) -> None:
        pending, self._pending_out = self._pending_out, []
        for topic, data, qos, retain, properties, info in pending:
            self.broker._publish(self, topic, data, qos, retain, properties, dup=True)
            info._mark_published()
            self._dispatch(
                lambda mid=info.mid: self._call(
                    self.on_publish, self, self.userdata, mid, FakeReasonCode(0, 'Success'), None
                )
            )

    def _on_broker_disconnect(self, reason_code: int, from_server: bool) -> None:
        self._connected = False
        self._dispatch(
            lambda: self._call(
                self.on_disconnect,
                self,
                self.userdata,
                FakeDisconnectFlags(from_server),
                FakeReasonCode(reason_code),
                None,
            )
        )

    def _receive(self, message: FakeMQTTMessage) -> None:
        self._dispatch(lambda: self._call(self.on_message, self, self.userdata, message))

    def _subscription_requests(self, topic: Any, qos: int, options: Any) -> list[Subscription]:
        def from_options(topic_filter: str, opts: Any) -> Subscription:
            validate_filter(topic_filter)
            if isinstance(opts, int) and not isinstance(opts, bool):
                if opts not in (0, 1, 2):
                    raise ValueError('Invalid QoS level.')
                return Subscription(topic_filter, qos=opts)
            granted = int(getattr(opts, 'QoS', 0))
            if granted not in (0, 1, 2):
                raise ValueError('Invalid QoS level.')
            return Subscription(
                topic_filter,
                qos=granted,
                no_local=bool(getattr(opts, 'noLocal', False)),
                retain_as_published=bool(getattr(opts, 'retainAsPublished', False)),
                retain_handling=int(getattr(opts, 'retainHandling', RETAIN_SEND_ON_SUBSCRIBE)),
            )

        if isinstance(topic, str):
            return [from_options(topic, qos if options is None else options)]
        if isinstance(topic, tuple):
            if len(topic) != 2 or not isinstance(topic[0], str):
                raise ValueError('Invalid subscription filter.')
            return [from_options(topic[0], topic[1])]
        if isinstance(topic, list):
            requests: list[Subscription] = []
            for item in topic:
                if not isinstance(item, tuple) or len(item) != 2 or not isinstance(item[0], str):
                    raise ValueError('Invalid subscription filter.')
                requests.append(from_options(item[0], item[1]))
            if not requests:
                raise ValueError('Invalid subscription filter.')
            return requests
        raise ValueError('Invalid subscription filter.')

    def _dispatch(self, fn: Callable[[], None]) -> None:
        if self.synchronous:
            fn()
        else:
            self._queue.put(fn)

    def _call(self, callback: Callable[..., Any] | None, *args: Any) -> None:
        if callback is None:
            return
        try:
            callback(*args)
        except Exception as error:
            self.callback_errors.append(error)
            LOGGER.exception('callback %s raised in fake client %s', callback, self.client_id)

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is _STOP:
                    return
                item()
            finally:
                self._queue.task_done()

    def __repr__(self) -> str:
        state = 'connected' if self._connected else 'disconnected'
        return f'FakeMqttClient({self.client_id!r}, {state})'


__all__: Sequence[str] = (
    'DEFAULT_EPOCH_MS',
    'MQTT_ERR_INVAL',
    'MQTT_ERR_NO_CONN',
    'MQTT_ERR_QUEUE_SIZE',
    'MQTT_ERR_SUCCESS',
    'REASON_NOT_AUTHORIZED',
    'REASON_SESSION_TAKEN_OVER',
    'REASON_SUCCESS',
    'REASON_UNSPECIFIED_ERROR',
    'RETAIN_DO_NOT_SEND',
    'RETAIN_SEND_IF_NEW_SUB',
    'RETAIN_SEND_ON_SUBSCRIBE',
    'FakeBroker',
    'FakeCallbackAPIVersion',
    'FakeClock',
    'FakeConnectFlags',
    'FakeDisconnectFlags',
    'FakeMQTTMessage',
    'FakeMessageInfo',
    'FakeMqttClient',
    'FakeProperties',
    'FakeReasonCode',
    'FakeScheduler',
    'FakeSubscribeOptions',
    'FakeTimer',
    'MQTTv5',
    'MQTTv311',
    'PublishRecord',
    'RetainedMessage',
    'Subscription',
    'WillSpec',
    'topic_matches',
    'validate_filter',
)
