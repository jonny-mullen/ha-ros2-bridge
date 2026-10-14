# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The node's MQTT connection: paho-mqtt 2.1.0 wrapped so nothing on the wire can stop a robot.

:class:`MqttLink` owns one paho ``Client`` (MQTT 5, ``CallbackAPIVersion.VERSION2``)
and is the only module of the package that imports paho. It follows the
design's connection lifecycle exactly: the client is built with
``reconnect_on_failure=True``, credentials and optional TLS are set, the Last
Will is ``availability offline`` (QoS 1, retained), the reconnect delay is
bounded, and :meth:`MqttLink.start` calls ``connect_async(...,
clean_start=True)`` then ``loop_start()``, after which paho's own network
thread keeps the connection alive and reconnects by itself.

Threading model (docs/architecture.md): paho's network thread never touches
robot state. Every inbound delivery is reduced to an :class:`InboundMessage`
(topic, payload bytes, the RETAIN flag and the receive time in epoch
milliseconds) and appended to a bounded :class:`collections.deque`, after a
size check that drops and counts an oversize payload; a full deque drops the
new message and counts ``dropped_queue_full``. Successful connects and every
disconnect are appended to the same deque as :class:`ConnectEvent` and
:class:`DisconnectEvent`, so the main thread sees them in order with the
messages around them. After each append the injected ``wake`` callable runs
(the node passes its rclpy ``GuardCondition.trigger``); the main thread then
calls :meth:`MqttLink.drain`. Nothing a callback does may raise: paho re-raises
callback exceptions and that would end its network thread, so every callback
body here is guarded and logs instead.

Outbound traffic goes the other way only: :meth:`MqttLink.publish` is
documented as main-thread only (paho's own threaded-loop pattern). It
validates the topic with :func:`ros2_ha_bridge.protocol.validate_publish_topic`
before paho sees it, sets the MQTT 5 Message Expiry Interval when asked, and
turns every failure (``ValueError`` or ``OSError`` from paho, a non-zero error
code, anything else) into one ERROR line naming the topic, a
``publish_errors`` increment on the shared :class:`~ros2_ha_bridge.protocol.Counters`
and a ``False`` return. It never raises. One non-zero code is not a failure: a
QoS 1 publish while disconnected gets ``MQTT_ERR_NO_CONN`` from paho, which
nevertheless keeps the message and sends it after the next CONNACK, so it is
logged at DEBUG and reported as True; a QoS 0 publish while disconnected is
lost and counted.

Session takeover: a broker-sent DISCONNECT with reason code 142 ("Session
taken over") means another client connected with the same client id, which
for this node means another bridge process with the same ``robot.id``
(``mqtt.client_id`` is a function of it by design). The link logs an ERROR that
says so, counts ``takeovers``, doubles the reconnect delay per takeover so two
flapping nodes slow down rather than hammer the broker (paho resets its own
backoff after each successful CONNACK, so the minimum must be raised here;
the configured minimum is restored at the first connect or ordinary
disconnect after ten minutes without a takeover), and after
``mqtt.takeover_exit_after`` takeovers within ten minutes sets
:attr:`MqttLink.should_exit`, which the node turns into a non-zero exit so the
launch file's ``respawn_max_retries`` stops the flapping.

A refused CONNACK is retried by paho for ever, so each failure reason code is
logged at ERROR once in plain language (bad credentials, not authorised, the
broker unavailable) and then at DEBUG until a connect succeeds. A broker that
cannot be reached at all produces no CONNACK; paho reports that through
``on_connect_fail``, logged once the same way, and the node's
``mqtt.connect_timeout_s`` decides when to give up.

Clean shutdown (:meth:`MqttLink.shutdown`): inbound is stopped, ``availability
offline`` is published retained and waited for up to two seconds, then
``disconnect()`` and ``loop_stop()``. The paho client class is injectable
(``client_factory``) so the unit tests run against ``test/fakes.py``'s
``FakeMqttClient`` and ``FakeBroker`` with no broker and no network.
"""

from __future__ import annotations

import logging
import threading
from collections import deque
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass, field
from typing import Any

import paho.mqtt
import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion, MQTTProtocolVersion
from paho.mqtt.packettypes import PacketTypes
from paho.mqtt.properties import Properties
from paho.mqtt.subscribeoptions import SubscribeOptions

from ros2_ha_bridge.clock import Clock, SystemClock
from ros2_ha_bridge.protocol import (
    AVAILABILITY_OFFLINE,
    MAX_PAYLOAD_BYTES,
    Counters,
    encode_payload,
    validate_publish_topic,
)
from ros2_ha_bridge.sensor_mirror import filter_error

__all__ = [
    'CONNECT_FAILURE_TEXT',
    'DEFAULT_INBOUND_QUEUE_MAX',
    'DEFAULT_KEEPALIVE_S',
    'DEFAULT_OFFLINE_WAIT_S',
    'DEFAULT_PORT',
    'DEFAULT_RECONNECT_MAX_S',
    'DEFAULT_RECONNECT_MIN_S',
    'DEFAULT_TAKEOVER_EXIT_AFTER',
    'DEFAULT_WARN_INTERVAL_S',
    'REASON_SESSION_TAKEN_OVER',
    'TAKEOVER_WINDOW_S',
    'ConnectEvent',
    'DisconnectEvent',
    'InboundItem',
    'InboundMessage',
    'MqttLink',
    'MqttLinkConfig',
    'SubscriptionSpec',
]

LOGGER = logging.getLogger(__name__)

#: paho-mqtt below 2.0 lacks the VERSION2 callbacks and MQTT 5 subscribe options the node uses.
_REQUIRED_PAHO_MAJOR = 2
_paho_major = int(str(paho.mqtt.__version__).split('.', 1)[0] or 0)
if _paho_major < _REQUIRED_PAHO_MAJOR:
    raise ImportError(
        f'ros2_ha_bridge needs paho-mqtt {_REQUIRED_PAHO_MAJOR}.x (2.1.0 is pinned in '
        f'requirements.txt); found {paho.mqtt.__version__}. Install it with pip: the '
        'Ubuntu 24.04 apt package is 1.6.1.'
    )

#: ``mqtt.port`` default.
DEFAULT_PORT = 1883
#: ``mqtt.keepalive_s`` default; the broker publishes the will within about 1.5 times this.
DEFAULT_KEEPALIVE_S = 15
#: ``mqtt.reconnect_min_s`` and ``mqtt.reconnect_max_s`` defaults (``reconnect_delay_set``).
DEFAULT_RECONNECT_MIN_S = 1
DEFAULT_RECONNECT_MAX_S = 30
#: ``mqtt.takeover_exit_after`` default: exit after this many takeovers in the window; 0 never.
DEFAULT_TAKEOVER_EXIT_AFTER = 3
#: ``mqtt.inbound_queue_max`` default: the deque bound.
DEFAULT_INBOUND_QUEUE_MAX = 256
#: How long :meth:`MqttLink.shutdown` waits for the retained ``offline`` to be acknowledged.
DEFAULT_OFFLINE_WAIT_S = 2.0
#: Minimum spacing of the rate-limited WARN lines about dropped inbound messages.
DEFAULT_WARN_INTERVAL_S = 10.0
#: Takeovers are counted within a sliding window this long (seconds).
TAKEOVER_WINDOW_S = 600.0
#: MQTT 5 DISCONNECT reason code "Session taken over".
REASON_SESSION_TAKEN_OVER = 142
#: Key under which a TCP-level connection failure is logged once (no reason code exists).
_UNREACHABLE_KEY = -1

#: Plain-language text for the CONNACK reason codes a misconfiguration produces.
CONNECT_FAILURE_TEXT: Mapping[int, str] = {
    128: 'the broker refused the connection without giving a reason',
    129: 'the broker could not read the CONNECT packet (malformed packet)',
    130: 'the broker reported a protocol error',
    132: 'the broker does not support MQTT 5 (unsupported protocol version)',
    133: 'the broker refused the client id',
    134: 'bad user name or password (check mqtt.username and ROS2_HA_BRIDGE_MQTT_PASSWORD)',
    135: 'not authorised: this user may not connect to the broker or use its topics',
    136: 'the broker is unavailable (it answered but is not accepting connections)',
    137: 'the broker is busy',
    138: 'this client is banned by the broker',
    140: 'the broker rejected the authentication method',
    149: 'the CONNECT packet is too large for the broker',
    151: "the broker's connection quota is exceeded",
    156: 'the broker asks this client to use another server',
    157: 'the broker has moved to another server',
    159: "the broker's connection rate limit is exceeded",
}


@dataclass(frozen=True)
class MqttLinkConfig:
    """The ``mqtt.*`` node parameters the link reads (names as in ``bridge.example.yaml``).

    ``host`` and ``client_id`` have no defaults on purpose: the node supplies
    the host from its parameters and derives the client id from ``robot.id``,
    so this module never names a host. ``password`` is the value after the
    ``ROS2_HA_BRIDGE_MQTT_PASSWORD`` environment fallback the node applies.
    """

    host: str
    client_id: str
    port: int = DEFAULT_PORT
    username: str = ''
    #: Kept out of ``repr`` so a logged or raised config can never show the password.
    password: str = field(default='', repr=False)
    tls_enabled: bool = False
    tls_ca_cert: str = ''
    tls_certfile: str = ''
    tls_keyfile: str = ''
    tls_insecure: bool = False
    keepalive_s: int = DEFAULT_KEEPALIVE_S
    reconnect_min_s: int = DEFAULT_RECONNECT_MIN_S
    reconnect_max_s: int = DEFAULT_RECONNECT_MAX_S
    takeover_exit_after: int = DEFAULT_TAKEOVER_EXIT_AFTER
    max_payload_bytes: int = MAX_PAYLOAD_BYTES
    inbound_queue_max: int = DEFAULT_INBOUND_QUEUE_MAX
    warn_interval_s: float = DEFAULT_WARN_INTERVAL_S

    def __post_init__(self) -> None:
        if not isinstance(self.host, str) or not self.host.strip():
            raise ValueError('mqtt.host must be a non-empty string')
        if not isinstance(self.client_id, str) or not self.client_id:
            raise ValueError('mqtt.client_id must be a non-empty string')
        if not 1 <= self.port <= 65535:
            raise ValueError(f'mqtt.port must be between 1 and 65535, not {self.port!r}')
        if self.keepalive_s < 0:
            raise ValueError('mqtt.keepalive_s must not be negative')
        if self.reconnect_min_s < 1:
            raise ValueError('mqtt.reconnect_min_s must be at least 1')
        if self.reconnect_max_s < self.reconnect_min_s:
            raise ValueError('mqtt.reconnect_max_s must not be below mqtt.reconnect_min_s')
        if self.takeover_exit_after < 0:
            raise ValueError('mqtt.takeover_exit_after must not be negative (0 disables)')
        if self.max_payload_bytes < 1:
            raise ValueError('mqtt.max_payload_bytes must be at least 1')
        if self.inbound_queue_max < 1:
            raise ValueError('mqtt.inbound_queue_max must be at least 1')
        if self.warn_interval_s < 0:
            raise ValueError('warn_interval_s must not be negative')


@dataclass(frozen=True)
class SubscriptionSpec:
    """One subscription the link renews on every successful connect.

    ``retain_as_published`` asks for MQTT 5 Retain As Published, which the
    contract requires on the command topic so a planted retained command is
    delivered with its RETAIN flag live, not only at reconnect.
    """

    topic_filter: str
    qos: int = 0
    retain_as_published: bool = False

    def __post_init__(self) -> None:
        problem = filter_error(self.topic_filter)
        if problem is not None:
            raise ValueError(f'subscription {self.topic_filter!r}: {problem}')
        if self.qos not in (0, 1, 2):
            raise ValueError(f'subscription {self.topic_filter!r}: qos must be 0, 1 or 2')


@dataclass(frozen=True)
class InboundMessage:
    """One delivery, as queued by the network thread for the main thread."""

    topic: str
    payload: bytes
    #: The RETAIN flag as delivered (true live only under Retain As Published).
    retain: bool
    #: Node clock, epoch milliseconds, when the network thread received it.
    rx_ts_ms: int


@dataclass(frozen=True)
class ConnectEvent:
    """A CONNACK: ``ok`` when the broker accepted the connection.

    On success the subscriptions have already been renewed on the network
    thread; the node answers by publishing ``availability online``, the
    announce and the state.
    """

    rx_ts_ms: int
    reason_code: int
    reason: str
    session_present: bool = False

    @property
    def ok(self) -> bool:
        return self.reason_code == 0


@dataclass(frozen=True)
class DisconnectEvent:
    """A lost or refused connection; paho reconnects by itself unless the node exits.

    ``taken_over`` marks reason code 142 and ``exit_requested`` is true when
    this takeover crossed ``mqtt.takeover_exit_after`` (see
    :attr:`MqttLink.should_exit`).
    """

    rx_ts_ms: int
    reason_code: int
    reason: str
    from_server: bool = False
    taken_over: bool = False
    exit_requested: bool = False


#: What :meth:`MqttLink.drain` hands the main thread, in arrival order.
InboundItem = InboundMessage | ConnectEvent | DisconnectEvent


class _RateLimitedWarnings:
    """At most one WARN line per key per interval; the next line says how many were suppressed.

    Called from the network thread only, so no lock is needed.
    """

    def __init__(self, clock: Clock, interval_s: float, logger: logging.Logger) -> None:
        self._clock = clock
        self._interval_s = interval_s
        self._logger = logger
        self._last: dict[str, float] = {}
        self._suppressed: dict[str, int] = {}

    def warn(self, key: str, message: str, *args: object) -> None:
        now = self._clock.monotonic()
        last = self._last.get(key)
        if last is not None and now - last < self._interval_s:
            self._suppressed[key] = self._suppressed.get(key, 0) + 1
            return
        suppressed = self._suppressed.pop(key, 0)
        if suppressed:
            message += ' (%d similar warnings suppressed)'
            args = (*args, suppressed)
        self._logger.warning(message, *args)
        self._last[key] = now


class MqttLink:
    """One paho client, a bounded inbound deque and a guarded publish. See the module docstring.

    ``availability_topic`` is the robot's availability topic (the Last Will
    goes there); ``wake`` runs on the network thread after every append to
    the inbound deque; ``counters`` is the node's shared instance so the
    heartbeat reports what happened here; ``subscriptions`` are renewed on
    every successful connect (more can be added with :meth:`subscribe`).
    ``client_factory`` defaults to ``paho.mqtt.client.Client`` and is given
    paho's own positional and keyword arguments, so ``FakeBroker.client_factory``
    from the tests is a drop-in.

    Counters touched on the network thread (``dropped_oversize``,
    ``dropped_queue_full``, ``takeovers``) are never incremented on the main
    thread, and ``publish_errors`` only there, so no increment is ever lost
    to a race; the main thread only reads them for the heartbeat.

    Construction builds and configures the client but opens nothing; a TLS
    configuration paho refuses (a missing certificate file, for instance)
    raises here, which the node treats as a fatal start-up error.
    """

    def __init__(
        self,
        config: MqttLinkConfig,
        *,
        availability_topic: str,
        wake: Callable[[], None],
        counters: Counters | None = None,
        clock: Clock | None = None,
        subscriptions: Iterable[SubscriptionSpec] = (),
        client_factory: Callable[..., mqtt.Client] | None = None,
        logger: logging.Logger | None = None,
    ) -> None:
        self.config = config
        self.availability_topic = validate_publish_topic(availability_topic)
        self.counters = counters if counters is not None else Counters()
        self._wake_fn = wake
        self._clock: Clock = clock if clock is not None else SystemClock()
        self._logger = logger if logger is not None else LOGGER
        self._warnings = _RateLimitedWarnings(self._clock, config.warn_interval_s, self._logger)

        self._inbound: deque[InboundItem] = deque(maxlen=config.inbound_queue_max)
        self._lock = threading.Lock()
        self._subscriptions: list[SubscriptionSpec] = []
        for spec in subscriptions:
            if spec not in self._subscriptions:
                self._subscriptions.append(spec)

        self._started = False
        self._closed = False
        self._connected = False
        self._connack_seen = False
        self._connects = 0
        self._failures_logged: set[int] = set()
        self._takeover_times: deque[float] = deque()
        self._backed_off = False
        self._exit_reason: str | None = None

        factory = client_factory if client_factory is not None else mqtt.Client
        self._client = factory(
            CallbackAPIVersion.VERSION2,
            config.client_id,
            protocol=MQTTProtocolVersion.MQTTv5,
            reconnect_on_failure=True,
        )
        self._client.enable_logger(self._logger.getChild('paho'))
        if config.username:
            self._client.username_pw_set(config.username, config.password or None)
        if config.tls_enabled:
            self._client.tls_set(
                ca_certs=config.tls_ca_cert or None,
                certfile=config.tls_certfile or None,
                keyfile=config.tls_keyfile or None,
            )
            if config.tls_insecure:
                self._client.tls_insecure_set(True)
        self._client.will_set(self.availability_topic, AVAILABILITY_OFFLINE, qos=1, retain=True)
        self._client.reconnect_delay_set(config.reconnect_min_s, config.reconnect_max_s)
        self._client.on_connect = self._on_connect
        self._client.on_connect_fail = self._on_connect_fail
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message

    # ------------------------------------------------------------------ views

    @property
    def client(self) -> mqtt.Client:
        """The underlying client, for the node's diagnostics and the tests."""
        return self._client

    @property
    def client_id(self) -> str:
        return self.config.client_id

    @property
    def started(self) -> bool:
        return self._started

    @property
    def closed(self) -> bool:
        """True once :meth:`shutdown` has begun; inbound deliveries are ignored from then on."""
        return self._closed

    @property
    def connected(self) -> bool:
        """True between a successful CONNACK and the next disconnect."""
        return self._connected

    @property
    def connack_seen(self) -> bool:
        """True once any CONNACK (accepted or refused) has arrived; ``mqtt.connect_timeout_s`` uses it."""
        return self._connack_seen

    @property
    def connects(self) -> int:
        """Successful connects so far (1 after the first, more after reconnects)."""
        return self._connects

    @property
    def queue_length(self) -> int:
        return len(self._inbound)

    @property
    def takeovers_in_window(self) -> int:
        """Session takeovers seen within the last :data:`TAKEOVER_WINDOW_S` seconds."""
        self._prune_takeovers(self._clock.monotonic())
        return len(self._takeover_times)

    @property
    def should_exit(self) -> bool:
        """True once takeovers crossed ``mqtt.takeover_exit_after``; the node exits non-zero."""
        return self._exit_reason is not None

    @property
    def exit_reason(self) -> str | None:
        return self._exit_reason

    @property
    def subscriptions(self) -> tuple[SubscriptionSpec, ...]:
        with self._lock:
            return tuple(self._subscriptions)

    # ------------------------------------------------------------------ lifecycle (main thread)

    def start(self) -> None:
        """``connect_async(host, port, keepalive, clean_start=True)`` then ``loop_start()``.

        Returns at once; the CONNACK arrives as a :class:`ConnectEvent`. A
        second call does nothing.
        """
        if self._started or self._closed:
            return
        self._started = True
        self._logger.info(
            'connecting to MQTT broker %s:%d as client %r (keepalive %d s, TLS %s)',
            self.config.host,
            self.config.port,
            self.config.client_id,
            self.config.keepalive_s,
            'on' if self.config.tls_enabled else 'off',
        )
        self._client.connect_async(
            self.config.host, self.config.port, self.config.keepalive_s, clean_start=True
        )
        self._client.loop_start()

    def shutdown(self, *, offline_wait_s: float = DEFAULT_OFFLINE_WAIT_S) -> None:
        """Stop inbound, publish ``availability offline`` (retained, acknowledged), disconnect, stop the loop.

        Main thread only. Never raises: each step is guarded and logged, so a
        broker that has already gone away cannot keep the node from exiting.
        A second call does nothing.
        """
        if self._closed:
            return
        self._closed = True
        if self._client.is_connected():
            try:
                info = self._client.publish(
                    self.availability_topic, AVAILABILITY_OFFLINE, qos=1, retain=True
                )
                if info.rc != 0:
                    self._logger.warning(
                        'could not publish availability offline at shutdown: %s',
                        mqtt.error_string(info.rc),
                    )
                else:
                    info.wait_for_publish(offline_wait_s)
                    if not info.is_published():
                        self._logger.warning(
                            'availability offline was not acknowledged within %g s; '
                            'the will covers it',
                            offline_wait_s,
                        )
            except (ValueError, RuntimeError, OSError) as error:
                self._logger.warning(
                    'publishing availability offline at shutdown failed: %s', error
                )
        else:
            self._logger.info(
                'not connected at shutdown; the broker has published (or will publish) the will'
            )
        try:
            self._client.disconnect()
        except OSError as error:
            self._logger.warning('disconnect at shutdown failed: %s', error)
        if self._started:
            try:
                self._client.loop_stop()
            except RuntimeError as error:
                self._logger.warning('loop_stop at shutdown failed: %s', error)
        self._connected = False
        self._logger.info('MQTT link closed')

    # ------------------------------------------------------------------ inbound (main thread)

    def drain(self) -> list[InboundItem]:
        """Take everything queued so far, in arrival order. Main thread only.

        The deque's ``popleft`` is atomic, so no lock is needed against the
        network thread's ``append``.
        """
        items: list[InboundItem] = []
        while True:
            try:
                items.append(self._inbound.popleft())
            except IndexError:
                return items

    # ------------------------------------------------------------------ outbound (main thread)

    def publish(
        self,
        topic: str,
        payload: bytes,
        qos: int = 0,
        retain: bool = False,
        expiry_s: int | None = None,
    ) -> bool:
        """Publish ``payload`` on ``topic``; True when paho accepted it. Main thread only.

        The topic is validated first (a wildcard, an empty or over-long topic
        or a NUL character is refused here rather than inside paho's network
        code); ``expiry_s`` becomes the MQTT 5 Message Expiry Interval. A QoS 1
        publish while disconnected is accepted: paho answers
        ``MQTT_ERR_NO_CONN`` but keeps the message and sends it after the next
        CONNACK, so it is logged at DEBUG and reported as True without touching
        ``publish_errors``. Every real failure, whether a ``ValueError`` or
        ``OSError`` from paho, a non-zero error code (a QoS 0 publish while
        disconnected is lost, for instance) or anything unexpected, is logged
        once at ERROR with the topic, counted in ``publish_errors`` and
        reported as ``False``. This method never raises, so no publish can ever
        end the process.
        """
        try:
            validate_publish_topic(topic)
            properties = None if expiry_s is None else _expiry_properties(expiry_s)
            info = self._client.publish(
                topic, payload, qos=qos, retain=retain, properties=properties
            )
        except (ValueError, TypeError, OSError) as error:
            self._publish_failed(topic, f'{type(error).__name__}: {error}')
            return False
        except Exception as error:
            self._logger.exception(
                'publish to %r raised an unexpected %s', topic, type(error).__name__
            )
            self.counters.publish_errors += 1
            return False
        if info.rc != 0:
            if qos > 0 and info.rc == mqtt.MQTT_ERR_NO_CONN:
                self._logger.debug(
                    'publish to %r queued while disconnected; paho sends it after the reconnect',
                    topic,
                )
                return True
            self._publish_failed(topic, f'paho error {int(info.rc)}: {mqtt.error_string(info.rc)}')
            return False
        return True

    def publish_json(
        self,
        topic: str,
        payload: Mapping[str, Any],
        qos: int = 0,
        retain: bool = False,
        expiry_s: int | None = None,
    ) -> bool:
        """:meth:`publish` of a payload object encoded with the contract's wire encoding.

        Matches :data:`ros2_ha_bridge.task_manager.PublishFn`, so the node
        can hand this method to the task manager as is. An object the encoder
        refuses is a publish failure like any other, never an exception.
        """
        try:
            data = encode_payload(payload)
        except (TypeError, ValueError) as error:
            self._publish_failed(topic, f'payload cannot be encoded: {error}')
            return False
        return self.publish(topic, data, qos, retain, expiry_s)

    def subscribe(
        self, topic_filter: str, qos: int = 0, *, retain_as_published: bool = False
    ) -> bool:
        """Register a subscription, renewed on every connect, and subscribe now if connected.

        Main thread only. Returns False when the filter is invalid or when
        the immediate subscribe failed (the registration stands and the next
        connect retries it).
        """
        try:
            spec = SubscriptionSpec(topic_filter, qos, retain_as_published)
        except ValueError as error:
            self._logger.error('refusing subscription: %s', error)
            return False
        with self._lock:
            if spec not in self._subscriptions:
                self._subscriptions.append(spec)
        if not self._connected:
            return True
        return self._subscribe_now(spec)

    # ------------------------------------------------------------------ paho callbacks (network thread)

    def _on_connect(
        self,
        client: mqtt.Client,
        userdata: Any,
        flags: Any,
        reason_code: Any,
        properties: Any,
    ) -> None:
        try:
            code, name = _reason(reason_code)
            now = self._clock.now_ms()
            self._connack_seen = True
            if code == 0:
                self._connected = True
                self._connects += 1
                self._failures_logged.clear()
                session_present = bool(getattr(flags, 'is_session_present', False))
                self._logger.info(
                    'connected to the MQTT broker as %r (connect %d, session present: %s)',
                    self.config.client_id,
                    self._connects,
                    session_present,
                )
                for spec in self.subscriptions:
                    self._subscribe_now(spec)
                self._restore_backoff_if_quiet()
                self._enqueue_event(ConnectEvent(now, code, name, session_present))
                return
            self._connected = False
            self._log_connect_failure(code, name)
            self._enqueue_event(ConnectEvent(now, code, name))
        except Exception:
            self._logger.exception('on_connect raised; ignored so the network thread survives')

    def _on_connect_fail(self, client: mqtt.Client, userdata: Any) -> None:
        try:
            self._connected = False
            if _UNREACHABLE_KEY not in self._failures_logged:
                self._failures_logged.add(_UNREACHABLE_KEY)
                self._logger.error(
                    'cannot reach the MQTT broker at %s:%d (no TCP connection); retrying '
                    'every %d to %d s until it answers',
                    self.config.host,
                    self.config.port,
                    self.config.reconnect_min_s,
                    self.config.reconnect_max_s,
                )
            else:
                self._logger.debug('MQTT broker still unreachable; retrying')
        except Exception:
            self._logger.exception('on_connect_fail raised; ignored')

    def _on_disconnect(
        self,
        client: mqtt.Client,
        userdata: Any,
        flags: Any,
        reason_code: Any,
        properties: Any,
    ) -> None:
        try:
            self._connected = False
            code, name = _reason(reason_code)
            from_server = bool(getattr(flags, 'is_disconnect_packet_from_server', False))
            if self._closed:
                self._logger.debug('disconnected during shutdown: %s (%d)', name, code)
                return
            taken_over = code == REASON_SESSION_TAKEN_OVER
            exit_requested = self._on_takeover() if taken_over else False
            if not taken_over:
                self._restore_backoff_if_quiet()
                self._logger.warning(
                    'disconnected from the MQTT broker: %s (reason code %d%s); paho reconnects',
                    name,
                    code,
                    ', sent by the broker' if from_server else '',
                )
            self._enqueue_event(
                DisconnectEvent(
                    self._clock.now_ms(), code, name, from_server, taken_over, exit_requested
                )
            )
        except Exception:
            self._logger.exception('on_disconnect raised; ignored so the network thread survives')

    def _on_message(self, client: mqtt.Client, userdata: Any, message: Any) -> None:
        try:
            if self._closed:
                return
            topic = str(message.topic)
            payload = bytes(message.payload)
            if len(payload) > self.config.max_payload_bytes:
                self.counters.dropped_oversize += 1
                self._warnings.warn(
                    'oversize',
                    'dropped a %d-byte message on %r: over mqtt.max_payload_bytes (%d)',
                    len(payload),
                    topic,
                    self.config.max_payload_bytes,
                )
                return
            item = InboundMessage(topic, payload, bool(message.retain), self._clock.now_ms())
            if self._full():
                self.counters.dropped_queue_full += 1
                self._warnings.warn(
                    'queue_full',
                    'dropped a message on %r: the inbound queue holds %d items and the main '
                    'thread has not drained it (is the executor blocking?)',
                    topic,
                    self.config.inbound_queue_max,
                )
                return
            self._inbound.append(item)
            self._wake()
        except Exception:
            self._logger.exception('on_message raised; ignored so the network thread survives')

    # ------------------------------------------------------------------ internals

    def _full(self) -> bool:
        return len(self._inbound) >= self.config.inbound_queue_max

    def _enqueue_event(self, event: ConnectEvent | DisconnectEvent) -> None:
        """Events are never dropped: when the deque is full the oldest message makes room."""
        if self._full():
            self.counters.dropped_queue_full += 1
            self._warnings.warn(
                'queue_full',
                'inbound queue full (%d items); the oldest message was dropped to queue a '
                'connection event',
                self.config.inbound_queue_max,
            )
        self._inbound.append(event)
        self._wake()

    def _wake(self) -> None:
        try:
            self._wake_fn()
        except Exception:
            self._logger.exception('wake callable raised; the main thread may drain late')

    def _subscribe_now(self, spec: SubscriptionSpec) -> bool:
        try:
            if spec.retain_as_published:
                result, _ = self._client.subscribe(
                    spec.topic_filter,
                    options=SubscribeOptions(qos=spec.qos, retainAsPublished=True),
                )
            else:
                result, _ = self._client.subscribe(spec.topic_filter, spec.qos)
        except (ValueError, OSError) as error:
            self._logger.error('subscribe to %r failed: %s', spec.topic_filter, error)
            return False
        if result != 0:
            self._logger.error(
                'subscribe to %r failed: %s', spec.topic_filter, mqtt.error_string(result)
            )
            return False
        self._logger.debug(
            'subscribed to %r (qos %d, retain as published %s)',
            spec.topic_filter,
            spec.qos,
            spec.retain_as_published,
        )
        return True

    def _publish_failed(self, topic: str, reason: str) -> None:
        self.counters.publish_errors += 1
        self._logger.error('publish to %r failed: %s', topic, reason)

    def _log_connect_failure(self, code: int, name: str) -> None:
        text = CONNECT_FAILURE_TEXT.get(code, f'the broker answered {name!r}')
        if code in self._failures_logged:
            self._logger.debug('MQTT connect refused again: %s (%d)', name, code)
            return
        self._failures_logged.add(code)
        self._logger.error(
            'MQTT connect to %s:%d refused with reason code %d (%s): %s; paho keeps retrying '
            'every %d to %d s',
            self.config.host,
            self.config.port,
            code,
            name,
            text,
            self.config.reconnect_min_s,
            self.config.reconnect_max_s,
        )

    def _prune_takeovers(self, now: float) -> None:
        while self._takeover_times and now - self._takeover_times[0] > TAKEOVER_WINDOW_S:
            self._takeover_times.popleft()

    def _restore_backoff_if_quiet(self) -> None:
        """Put the configured reconnect minimum back once a window has passed with no takeover.

        Called on the network thread at every successful connect and every
        ordinary disconnect; while any takeover is inside the window the
        doubled minimum stands, so two flapping nodes keep slowing down.
        """
        if not self._backed_off or self.takeovers_in_window:
            return
        self._backed_off = False
        self._client.reconnect_delay_set(self.config.reconnect_min_s, self.config.reconnect_max_s)
        self._logger.info(
            'no session takeover for %g minutes; reconnect delay restored to %d to %d s',
            TAKEOVER_WINDOW_S / 60,
            self.config.reconnect_min_s,
            self.config.reconnect_max_s,
        )

    def _on_takeover(self) -> bool:
        """Count a takeover, back off, and decide whether the node must exit. Returns the decision."""
        now = self._clock.monotonic()
        self._prune_takeovers(now)
        self._takeover_times.append(now)
        count = len(self._takeover_times)
        self.counters.takeovers += 1
        delay = min(self.config.reconnect_min_s * (2**count), self.config.reconnect_max_s)
        self._client.reconnect_delay_set(delay, self.config.reconnect_max_s)
        self._backed_off = True
        limit = self.config.takeover_exit_after
        exit_now = limit > 0 and count >= limit and self._exit_reason is None
        if exit_now:
            self._exit_reason = (
                f'MQTT session for client id {self.config.client_id!r} taken over '
                f'{count} times within {TAKEOVER_WINDOW_S / 60:g} minutes'
            )
        self._logger.error(
            'MQTT session for client id %r was taken over by another connection (reason code '
            '%d, "Session taken over"): most likely a second bridge node is running with the '
            'same robot.id. Takeover %d%s within %g minutes; reconnecting after %d s%s',
            self.config.client_id,
            REASON_SESSION_TAKEN_OVER,
            count,
            f' of {limit}' if limit > 0 else '',
            TAKEOVER_WINDOW_S / 60,
            delay,
            '; this node will now exit so the launch file can stop the flapping'
            if exit_now
            else '',
        )
        return exit_now


def _expiry_properties(expiry_s: int) -> Properties:
    """PUBLISH properties carrying the MQTT 5 Message Expiry Interval; negative is refused."""
    if expiry_s < 0:
        raise ValueError(f'expiry_s must not be negative, got {expiry_s!r}')
    # paho 2.1.0 ships py.typed but leaves Properties.__init__ untyped.
    properties = Properties(PacketTypes.PUBLISH)  # type: ignore[no-untyped-call]
    properties.MessageExpiryInterval = int(expiry_s)
    return properties


def _reason(reason_code: Any) -> tuple[int, str]:
    """``(value, name)`` of a paho ``ReasonCode`` (or anything with ``value`` and ``getName``)."""
    value = getattr(reason_code, 'value', reason_code)
    try:
        code = int(value)
    except (TypeError, ValueError):
        code = -1
    get_name = getattr(reason_code, 'getName', None)
    name = str(get_name()) if callable(get_name) else str(reason_code)
    return code, name
