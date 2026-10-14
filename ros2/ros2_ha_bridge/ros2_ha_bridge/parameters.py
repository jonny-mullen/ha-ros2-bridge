# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The node's parameters: the declaration table and the validated, frozen settings.

:data:`PARAMETERS` is the single list of everything the node declares, with
the name, kind, default and one-line description of each parameter (the same
table ``config/bridge.example.yaml`` spells out). :func:`settings_from_mapping`
turns a flat mapping of parameter name to value, as the node reads it back
from rclpy, into a :class:`BridgeSettings` tree, applying the two environment
fallbacks (an empty ``mqtt.password`` reads ``ROS2_HA_BRIDGE_MQTT_PASSWORD``
and an empty ``command.secret`` reads ``ROS2_HA_BRIDGE_COMMAND_SECRET``), the
derived ``mqtt.client_id`` (``ros2_ha_bridge_{robot.id}``, a function of the
robot id by design so a duplicate id collides visibly on the broker) and every
validation rule the core modules would otherwise discover one by one. A
misconfiguration raises :class:`ConfigError` with a plain message naming the
parameter; the node treats that as a fatal start-up error.

This module imports neither rclpy nor paho, so the settings can be built and
tested on any machine; the node (``bridge_node.py``) owns the rclpy
declaration. :func:`describe_settings` renders the effective values for the
start-up log with the password and the secret masked.

Two rules the table does not express by itself:

- ``command.dedupe_window_s`` must be at least ``reply.ack_expiry_s`` (a
  redelivery within the ack's lifetime must republish the stored ack); a
  smaller value is raised to the expiry with a warning.
- ``watchdog.timeout_s`` is clamped at run time to at least 2.5 times the
  heartbeat interval Home Assistant announces (:mod:`ros2_ha_bridge.watchdog`),
  so the value here is a floor, not the effective timeout.
"""

from __future__ import annotations

import logging
import math
import os
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal

from ros2_ha_bridge.authoriser import (
    DEFAULT_RETAINED_REJECTIONS_PER_MINUTE,
    DEFAULT_TASK_TIMEOUT_CAP_S,
)
from ros2_ha_bridge.dedupe_store import DEFAULT_DEDUPE_SIZE, DEFAULT_DEDUPE_WINDOW_S
from ros2_ha_bridge.executors.base import JointUnit
from ros2_ha_bridge.executors.fake import (
    DEFAULT_FEEDBACK_HZ,
    DEFAULT_TASK_DURATION_S,
    ENTRY_POINT_NAME,
)
from ros2_ha_bridge.joints_publisher import DEFAULT_JOINTS_RATE_HZ, default_min_delta
from ros2_ha_bridge.protocol import (
    DEFAULT_ACK_EXPIRY_S,
    DEFAULT_CLOCK_SKEW_TOLERANCE_MS,
    DEFAULT_HA_STATUS_TOPIC,
    DEFAULT_MAX_AGE_CAP_MS,
    DEFAULT_PREFIX,
    DEFAULT_RESULT_EXPIRY_S,
    MAX_PAYLOAD_BYTES,
    WATCHDOG_MODES,
    WatchdogMode,
    is_valid_task_name,
    prefix_error,
    validate_robot_id,
)
from ros2_ha_bridge.sensor_mirror import (
    DEFAULT_AVAILABILITY_SUFFIX,
    DEFAULT_ROS_TOPIC,
    SensorMirror,
    filter_error,
)
from ros2_ha_bridge.task_manager import TaskManagerSettings

__all__ = [
    'DEFAULT_DATA_DIR',
    'DEFAULT_MQTT_HOST',
    'ENV_COMMAND_SECRET',
    'ENV_MQTT_PASSWORD',
    'MASK',
    'PARAMETERS',
    'PARAMETER_NAMES',
    'SECRET_PARAMETERS',
    'BridgeSettings',
    'CommandSettings',
    'ConfigError',
    'ExecutorSettings',
    'FakeSettings',
    'MirrorSettings',
    'MqttSettings',
    'ParameterKind',
    'ParameterSpec',
    'ReplySettings',
    'RobotSettings',
    'TelemetrySettings',
    'TlsSettings',
    'WatchdogSettings',
    'default_values',
    'derived_client_id',
    'describe_settings',
    'executor_parameters',
    'format_settings',
    'settings_from_mapping',
]

LOGGER = logging.getLogger(__name__)

#: Environment variables read when the matching parameter is empty.
ENV_MQTT_PASSWORD = 'ROS2_HA_BRIDGE_MQTT_PASSWORD'
ENV_COMMAND_SECRET = 'ROS2_HA_BRIDGE_COMMAND_SECRET'
#: Parameters whose values never appear in a log line or a plugin's parameter view.
SECRET_PARAMETERS = frozenset({'mqtt.password', 'command.secret'})
#: What a non-empty secret is shown as.
MASK = '***'

#: ``mqtt.host`` default: the loopback name, never a real host (the design's table).
DEFAULT_MQTT_HOST = 'localhost'
#: ``data_dir`` default; the dedupe file lives in ``{data_dir}/{robot.id}``.
DEFAULT_DATA_DIR = '~/.ros/ros2_ha_bridge'

# The ``mqtt.*`` defaults below equal the DEFAULT_* constants of ``mqtt_link``. They are
# restated here because that module imports paho and this one must stay importable on a
# machine without it; ``test_bridge_node.py`` checks that the two agree.
_DEFAULT_MQTT_PORT = 1883
_DEFAULT_KEEPALIVE_S = 15
_DEFAULT_RECONNECT_MIN_S = 1
_DEFAULT_RECONNECT_MAX_S = 30
_DEFAULT_TAKEOVER_EXIT_AFTER = 3
_DEFAULT_INBOUND_QUEUE_MAX = 256

_MANAGER_DEFAULTS = TaskManagerSettings()

ParameterKind = Literal['string', 'integer', 'double', 'bool', 'string_array']
ParameterDefault = str | int | float | bool | tuple[str, ...]


class ConfigError(Exception):
    """A parameter value the node cannot run with; the message names the parameter."""


@dataclass(frozen=True)
class ParameterSpec:
    """One row of the declaration table."""

    name: str
    kind: ParameterKind
    default: ParameterDefault
    description: str

    @property
    def default_value(self) -> Any:
        """The default as rclpy wants it (a list for an array)."""
        if isinstance(self.default, tuple):
            return list(self.default)
        return self.default


def _spec(name: str, kind: ParameterKind, default: ParameterDefault, text: str) -> ParameterSpec:
    return ParameterSpec(name, kind, default, text)


#: Every parameter the node declares, in the order ``config/bridge.example.yaml`` lists them.
PARAMETERS: tuple[ParameterSpec, ...] = (
    _spec('robot.id', 'string', '', 'Robot id: ^[a-z][a-z0-9_]{0,31}$, never ha; required.'),
    _spec('robot.name', 'string', '', 'Device name shown in Home Assistant; robot.id when empty.'),
    _spec('robot.manufacturer', 'string', '', 'Announce identity; empty means unknown.'),
    _spec('robot.model', 'string', '', 'Announce identity; empty means unknown.'),
    _spec('robot.sw_version', 'string', '', 'Announce identity; empty means unknown.'),
    _spec('robot.hw_version', 'string', '', 'Announce identity; empty means unknown.'),
    _spec('robot.serial_number', 'string', '', 'Announce identity; empty means unknown.'),
    _spec('robot.suggested_area', 'string', '', 'Area suggested for the HA device; empty omits.'),
    _spec('mqtt.host', 'string', DEFAULT_MQTT_HOST, 'MQTT broker host name or address.'),
    _spec('mqtt.port', 'integer', _DEFAULT_MQTT_PORT, 'MQTT broker port.'),
    _spec('mqtt.username', 'string', '', 'MQTT user name; empty connects anonymously.'),
    _spec(
        'mqtt.password',
        'string',
        '',
        f'MQTT password; empty reads the environment variable {ENV_MQTT_PASSWORD}.',
    ),
    _spec('mqtt.tls.enabled', 'bool', False, 'Connect over TLS.'),
    _spec('mqtt.tls.ca_cert', 'string', '', 'CA certificate file; empty uses the system store.'),
    _spec('mqtt.tls.certfile', 'string', '', 'Client certificate file for mutual TLS.'),
    _spec('mqtt.tls.keyfile', 'string', '', 'Client private key file for mutual TLS.'),
    _spec('mqtt.tls.insecure', 'bool', False, 'Skip host name verification (testing only).'),
    _spec(
        'mqtt.keepalive_s',
        'integer',
        _DEFAULT_KEEPALIVE_S,
        'MQTT keepalive; the will is published within about 1.5 times this.',
    ),
    _spec(
        'mqtt.client_id',
        'string',
        '',
        'MQTT client id; empty derives ros2_ha_bridge_{robot.id}, which must stay unique.',
    ),
    _spec(
        'mqtt.reconnect_min_s',
        'integer',
        _DEFAULT_RECONNECT_MIN_S,
        'Smallest delay between reconnect attempts.',
    ),
    _spec(
        'mqtt.reconnect_max_s',
        'integer',
        _DEFAULT_RECONNECT_MAX_S,
        'Largest delay between reconnect attempts.',
    ),
    _spec(
        'mqtt.connect_timeout_s',
        'double',
        0.0,
        'Exit non-zero without a CONNACK within this many seconds; 0 waits for ever.',
    ),
    _spec(
        'mqtt.takeover_exit_after',
        'integer',
        _DEFAULT_TAKEOVER_EXIT_AFTER,
        'Exit non-zero after this many session takeovers within ten minutes; 0 never.',
    ),
    _spec(
        'mqtt.max_payload_bytes',
        'integer',
        MAX_PAYLOAD_BYTES,
        'Inbound size limit; larger messages are dropped unparsed.',
    ),
    _spec(
        'mqtt.inbound_queue_max',
        'integer',
        _DEFAULT_INBOUND_QUEUE_MAX,
        'Bound of the inbound queue between the network thread and the node.',
    ),
    _spec(
        'topic_prefix',
        'string',
        DEFAULT_PREFIX,
        'Topic prefix shared with the HA integration; no slashes at the ends, no wildcards.',
    ),
    _spec(
        'ha_status_topic',
        'string',
        DEFAULT_HA_STATUS_TOPIC,
        "Home Assistant's birth and will topic; online triggers a re-announce.",
    ),
    _spec(
        'announce.jitter_s',
        'double',
        2.0,
        'Re-announce after HA birth within a random delay up to this many seconds.',
    ),
    _spec(
        'executor',
        'string',
        ENTRY_POINT_NAME,
        'Executor plugin: an entry-point name or package.module:Class.',
    ),
    _spec(
        'executor.call_warn_s',
        'double',
        _MANAGER_DEFAULTS.call_warn_s,
        'A plugin call longer than this marks the executor degraded.',
    ),
    _spec(
        'executor.cancel_grace_s',
        'double',
        _MANAGER_DEFAULTS.cancel_grace_s,
        'Seconds a cancel may take before the result is forced.',
    ),
    _spec(
        'executor.health_poll_s',
        'double',
        _MANAGER_DEFAULTS.health_poll_s,
        'Period of the executor health poll.',
    ),
    _spec(
        'tasks.allow',
        'string_array',
        (),
        'Task names announced to HA; empty means every task the executor offers.',
    ),
    _spec(
        'command.secret',
        'string',
        '',
        f'Shared HMAC secret; empty reads {ENV_COMMAND_SECRET}; empty after both means no HMAC.',
    ),
    _spec(
        'command.max_age_cap_ms',
        'integer',
        DEFAULT_MAX_AGE_CAP_MS,
        "Cap on a command's own max_age_ms in the freshness rule.",
    ),
    _spec(
        'command.clock_skew_tolerance_ms',
        'integer',
        DEFAULT_CLOCK_SKEW_TOLERANCE_MS,
        'Clock skew tolerated either side of the freshness window.',
    ),
    _spec(
        'command.dedupe_window_s',
        'double',
        DEFAULT_DEDUPE_WINDOW_S,
        'How long a command id is remembered; at least reply.ack_expiry_s.',
    ),
    _spec(
        'command.dedupe_size',
        'integer',
        DEFAULT_DEDUPE_SIZE,
        'How many command ids are remembered at most.',
    ),
    _spec(
        'command.task_timeout_cap_s',
        'double',
        DEFAULT_TASK_TIMEOUT_CAP_S,
        'Upper bound on any task timeout, announced or requested.',
    ),
    _spec(
        'data_dir',
        'string',
        DEFAULT_DATA_DIR,
        'Directory holding {robot.id}/dedupe.json; empty keeps the dedupe store in memory.',
    ),
    _spec(
        'reply.ack_expiry_s',
        'integer',
        DEFAULT_ACK_EXPIRY_S,
        'Message Expiry Interval of retained acks.',
    ),
    _spec(
        'reply.result_expiry_s',
        'integer',
        DEFAULT_RESULT_EXPIRY_S,
        'Message Expiry Interval of retained results.',
    ),
    _spec(
        'reply.feedback_max_rate_hz',
        'double',
        _MANAGER_DEFAULTS.feedback_max_rate_hz,
        'Per-task feedback throttle; 0 disables throttling.',
    ),
    _spec(
        'reply.retained_rejections_per_minute',
        'integer',
        DEFAULT_RETAINED_REJECTIONS_PER_MINUTE,
        'Rejected acks beyond this per minute are published non-retained.',
    ),
    _spec(
        'heartbeat.interval_s',
        'double',
        2.0,
        'Period of the robot heartbeat; 0 disables it.',
    ),
    _spec(
        'telemetry.joints_rate_hz',
        'double',
        DEFAULT_JOINTS_RATE_HZ,
        'Most joints messages per second; 0 disables joint telemetry and the announce block.',
    ),
    _spec(
        'telemetry.joints_min_delta',
        'double',
        0.0,
        'Least movement of one joint that publishes; 0 means the unit default (0.5 deg, 0.01 rad), '
        'so a tiny positive value, not 0, publishes every change.',
    ),
    _spec(
        'watchdog.mode',
        'string',
        'none',
        'none (informational), inhibit_new or stop_and_hold when the HA heartbeat stops.',
    ),
    _spec(
        'watchdog.timeout_s',
        'double',
        5.0,
        'HA heartbeat silence that counts as link down; floor of 2.5 x the HA interval.',
    ),
    _spec(
        'mirror.topics',
        'string_array',
        (),
        'MQTT topics or filters mirrored onto the ROS topic together with their availability.',
    ),
    _spec(
        'mirror.availability_suffix',
        'string',
        DEFAULT_AVAILABILITY_SUFFIX,
        "Suffix of each mirrored topic's availability sibling; empty disables tracking.",
    ),
    _spec(
        'mirror.ros_topic',
        'string',
        DEFAULT_ROS_TOPIC,
        'ROS topic (std_msgs/String envelopes) the mirrored messages are published on.',
    ),
    _spec(
        'fake.task_duration_s',
        'double',
        DEFAULT_TASK_DURATION_S,
        'Fake executor: how long reset, move_to and fail take.',
    ),
    _spec(
        'fake.feedback_hz',
        'double',
        DEFAULT_FEEDBACK_HZ,
        'Fake executor: feedback rate of a running motion task; 0 disables.',
    ),
)

#: Parameter names in declaration order.
PARAMETER_NAMES: tuple[str, ...] = tuple(spec.name for spec in PARAMETERS)
_SPECS: Mapping[str, ParameterSpec] = {spec.name: spec for spec in PARAMETERS}


def default_values() -> dict[str, Any]:
    """A fresh flat mapping of every parameter to its default."""
    return {spec.name: spec.default_value for spec in PARAMETERS}


def derived_client_id(robot_id: str) -> str:
    """The MQTT client id used when ``mqtt.client_id`` is empty."""
    return f'ros2_ha_bridge_{robot_id}'


# --------------------------------------------------------------------------- settings


@dataclass(frozen=True)
class RobotSettings:
    """``robot.*``: identity for the announce. Empty identity strings become ``None``."""

    id: str
    #: ``robot.name``, or ``robot.id`` when it was empty (the announce needs a name).
    name: str
    manufacturer: str | None = None
    model: str | None = None
    sw_version: str | None = None
    hw_version: str | None = None
    serial_number: str | None = None
    suggested_area: str | None = None


@dataclass(frozen=True)
class TlsSettings:
    """``mqtt.tls.*``; file paths are checked by paho when the link is built."""

    enabled: bool = False
    ca_cert: str = ''
    certfile: str = ''
    keyfile: str = ''
    insecure: bool = False


@dataclass(frozen=True)
class MqttSettings:
    """``mqtt.*`` after the password fallback and the client id derivation."""

    host: str
    client_id: str
    port: int = _DEFAULT_MQTT_PORT
    username: str = ''
    #: Kept out of ``repr`` so a logged or raised settings object can never show it.
    password: str = field(default='', repr=False)
    #: True when the password came from the environment variable.
    password_from_env: bool = False
    #: True when the client id was derived from ``robot.id`` (the recommended setting).
    client_id_derived: bool = True
    tls: TlsSettings = TlsSettings()
    keepalive_s: int = _DEFAULT_KEEPALIVE_S
    reconnect_min_s: int = _DEFAULT_RECONNECT_MIN_S
    reconnect_max_s: int = _DEFAULT_RECONNECT_MAX_S
    connect_timeout_s: float = 0.0
    takeover_exit_after: int = _DEFAULT_TAKEOVER_EXIT_AFTER
    max_payload_bytes: int = MAX_PAYLOAD_BYTES
    inbound_queue_max: int = _DEFAULT_INBOUND_QUEUE_MAX


@dataclass(frozen=True)
class ExecutorSettings:
    """``executor`` and ``executor.*`` plus the ``tasks.allow`` list."""

    spec: str = ENTRY_POINT_NAME
    call_warn_s: float = _MANAGER_DEFAULTS.call_warn_s
    cancel_grace_s: float = _MANAGER_DEFAULTS.cancel_grace_s
    health_poll_s: float = _MANAGER_DEFAULTS.health_poll_s
    #: ``tasks.allow``: empty means every task the executor offers.
    tasks_allow: tuple[str, ...] = ()


@dataclass(frozen=True)
class CommandSettings:
    """``command.*`` after the secret fallback and the dedupe window rule."""

    #: Kept out of ``repr`` so a logged or raised settings object can never show it.
    secret: str = field(default='', repr=False)
    #: True when the secret came from the environment variable.
    secret_from_env: bool = False
    max_age_cap_ms: int = DEFAULT_MAX_AGE_CAP_MS
    clock_skew_tolerance_ms: int = DEFAULT_CLOCK_SKEW_TOLERANCE_MS
    dedupe_window_s: float = DEFAULT_DEDUPE_WINDOW_S
    dedupe_size: int = DEFAULT_DEDUPE_SIZE
    task_timeout_cap_s: float = DEFAULT_TASK_TIMEOUT_CAP_S

    @property
    def hmac_required(self) -> bool:
        """True when a secret is configured: unsigned commands are refused."""
        return bool(self.secret)

    @property
    def keep_on_load_ms(self) -> int:
        """How old a persisted dedupe entry may be at start-up (cap plus skew tolerance)."""
        return self.max_age_cap_ms + self.clock_skew_tolerance_ms


@dataclass(frozen=True)
class ReplySettings:
    """``reply.*``."""

    ack_expiry_s: int = DEFAULT_ACK_EXPIRY_S
    result_expiry_s: int = DEFAULT_RESULT_EXPIRY_S
    feedback_max_rate_hz: float = _MANAGER_DEFAULTS.feedback_max_rate_hz
    retained_rejections_per_minute: int = DEFAULT_RETAINED_REJECTIONS_PER_MINUTE


@dataclass(frozen=True)
class TelemetrySettings:
    """``telemetry.*`` (0.2.0): the joint telemetry throttle.

    ``joints_rate_hz`` 0 disables the ``joints`` topic and the announce block.
    ``joints_min_delta`` 0 (the default) means the announced unit's default,
    0.5 for ``deg`` and 0.01 for ``rad``, resolved by :meth:`min_delta_for`
    once the executor has said which unit it reports in. Because 0 is that
    sentinel, the publisher's "any change" setting (a delta of exactly 0) is
    not reachable through the parameter; a tiny positive value (``0.001``)
    does the same job, and the node logs the effective delta at start-up.
    """

    joints_rate_hz: float = DEFAULT_JOINTS_RATE_HZ
    joints_min_delta: float = 0.0

    @property
    def joints_enabled(self) -> bool:
        """True when the node publishes joint telemetry for an executor that has joints."""
        return self.joints_rate_hz > 0

    def min_delta_for(self, unit: JointUnit) -> float:
        """The effective minimum movement in ``unit``: the parameter, or the unit default."""
        return self.joints_min_delta if self.joints_min_delta > 0 else default_min_delta(unit)


@dataclass(frozen=True)
class WatchdogSettings:
    """``watchdog.*``; ``timeout_s`` is a floor, clamped at run time to the HA interval."""

    mode: WatchdogMode = 'none'
    timeout_s: float = 5.0


@dataclass(frozen=True)
class MirrorSettings:
    """``mirror.*``."""

    topics: tuple[str, ...] = ()
    availability_suffix: str = DEFAULT_AVAILABILITY_SUFFIX
    ros_topic: str = DEFAULT_ROS_TOPIC


@dataclass(frozen=True)
class FakeSettings:
    """``fake.*``, read by the shipped fake executor through its context."""

    task_duration_s: float = DEFAULT_TASK_DURATION_S
    feedback_hz: float = DEFAULT_FEEDBACK_HZ


@dataclass(frozen=True)
class BridgeSettings:
    """Everything the node reads from its parameters, validated, as one frozen tree."""

    robot: RobotSettings
    mqtt: MqttSettings
    topic_prefix: str = DEFAULT_PREFIX
    ha_status_topic: str = DEFAULT_HA_STATUS_TOPIC
    announce_jitter_s: float = 2.0
    executor: ExecutorSettings = ExecutorSettings()
    command: CommandSettings = CommandSettings()
    #: ``data_dir`` expanded, or ``None`` when the parameter was empty (memory-only store).
    data_dir: Path | None = Path(os.path.expanduser(DEFAULT_DATA_DIR))
    reply: ReplySettings = ReplySettings()
    heartbeat_interval_s: float = 2.0
    telemetry: TelemetrySettings = TelemetrySettings()
    watchdog: WatchdogSettings = WatchdogSettings()
    mirror: MirrorSettings = MirrorSettings()
    fake: FakeSettings = FakeSettings()

    @property
    def dedupe_dir(self) -> Path | None:
        """Where ``dedupe.json`` lives: ``{data_dir}/{robot.id}``, or ``None`` for memory-only."""
        return None if self.data_dir is None else self.data_dir / self.robot.id

    def as_mapping(self) -> dict[str, Any]:
        """The effective values, flat, keyed by parameter name (secrets included, unmasked)."""
        robot, mqtt, tls = self.robot, self.mqtt, self.mqtt.tls
        executor, command, reply = self.executor, self.command, self.reply
        return {
            'robot.id': robot.id,
            'robot.name': robot.name,
            'robot.manufacturer': robot.manufacturer or '',
            'robot.model': robot.model or '',
            'robot.sw_version': robot.sw_version or '',
            'robot.hw_version': robot.hw_version or '',
            'robot.serial_number': robot.serial_number or '',
            'robot.suggested_area': robot.suggested_area or '',
            'mqtt.host': mqtt.host,
            'mqtt.port': mqtt.port,
            'mqtt.username': mqtt.username,
            'mqtt.password': mqtt.password,
            'mqtt.tls.enabled': tls.enabled,
            'mqtt.tls.ca_cert': tls.ca_cert,
            'mqtt.tls.certfile': tls.certfile,
            'mqtt.tls.keyfile': tls.keyfile,
            'mqtt.tls.insecure': tls.insecure,
            'mqtt.keepalive_s': mqtt.keepalive_s,
            'mqtt.client_id': mqtt.client_id,
            'mqtt.reconnect_min_s': mqtt.reconnect_min_s,
            'mqtt.reconnect_max_s': mqtt.reconnect_max_s,
            'mqtt.connect_timeout_s': mqtt.connect_timeout_s,
            'mqtt.takeover_exit_after': mqtt.takeover_exit_after,
            'mqtt.max_payload_bytes': mqtt.max_payload_bytes,
            'mqtt.inbound_queue_max': mqtt.inbound_queue_max,
            'topic_prefix': self.topic_prefix,
            'ha_status_topic': self.ha_status_topic,
            'announce.jitter_s': self.announce_jitter_s,
            'executor': executor.spec,
            'executor.call_warn_s': executor.call_warn_s,
            'executor.cancel_grace_s': executor.cancel_grace_s,
            'executor.health_poll_s': executor.health_poll_s,
            'tasks.allow': list(executor.tasks_allow),
            'command.secret': command.secret,
            'command.max_age_cap_ms': command.max_age_cap_ms,
            'command.clock_skew_tolerance_ms': command.clock_skew_tolerance_ms,
            'command.dedupe_window_s': command.dedupe_window_s,
            'command.dedupe_size': command.dedupe_size,
            'command.task_timeout_cap_s': command.task_timeout_cap_s,
            'data_dir': '' if self.data_dir is None else str(self.data_dir),
            'reply.ack_expiry_s': reply.ack_expiry_s,
            'reply.result_expiry_s': reply.result_expiry_s,
            'reply.feedback_max_rate_hz': reply.feedback_max_rate_hz,
            'reply.retained_rejections_per_minute': reply.retained_rejections_per_minute,
            'heartbeat.interval_s': self.heartbeat_interval_s,
            'telemetry.joints_rate_hz': self.telemetry.joints_rate_hz,
            'telemetry.joints_min_delta': self.telemetry.joints_min_delta,
            'watchdog.mode': self.watchdog.mode,
            'watchdog.timeout_s': self.watchdog.timeout_s,
            'mirror.topics': list(self.mirror.topics),
            'mirror.availability_suffix': self.mirror.availability_suffix,
            'mirror.ros_topic': self.mirror.ros_topic,
            'fake.task_duration_s': self.fake.task_duration_s,
            'fake.feedback_hz': self.fake.feedback_hz,
        }


# --------------------------------------------------------------------------- reading values


class _Reader:
    """Typed access to the flat mapping, with the table's defaults and plain error messages."""

    def __init__(self, values: Mapping[str, Any]) -> None:
        self._values = values

    def raw(self, name: str) -> Any:
        if name not in _SPECS:
            raise KeyError(f'{name!r} is not a declared parameter')
        return self._values.get(name, _SPECS[name].default_value)

    def string(self, name: str) -> str:
        value = self.raw(name)
        if not isinstance(value, str):
            raise ConfigError(f'{name} must be a string, not {_type_name(value)}')
        return value

    def boolean(self, name: str) -> bool:
        value = self.raw(name)
        if not isinstance(value, bool):
            raise ConfigError(f'{name} must be true or false, not {_type_name(value)}')
        return value

    def integer(self, name: str, *, minimum: int | None = None, maximum: int | None = None) -> int:
        value = self.raw(name)
        if isinstance(value, bool) or not isinstance(value, int):
            raise ConfigError(f'{name} must be an integer, not {_type_name(value)}')
        if minimum is not None and value < minimum:
            raise ConfigError(f'{name} must be at least {minimum}, not {value}')
        if maximum is not None and value > maximum:
            raise ConfigError(f'{name} must be at most {maximum}, not {value}')
        return value

    def number(
        self, name: str, *, minimum: float | None = None, exclusive_minimum: float | None = None
    ) -> float:
        value = self.raw(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ConfigError(f'{name} must be a number, not {_type_name(value)}')
        number = float(value)
        if not math.isfinite(number):
            raise ConfigError(f'{name} must be a finite number')
        if minimum is not None and number < minimum:
            raise ConfigError(f'{name} must be at least {minimum:g}, not {number:g}')
        if exclusive_minimum is not None and number <= exclusive_minimum:
            raise ConfigError(f'{name} must be greater than {exclusive_minimum:g}, not {number:g}')
        return number

    def strings(self, name: str) -> tuple[str, ...]:
        value = self.raw(name)
        if isinstance(value, (str, bytes)) or not isinstance(value, (list, tuple)):
            raise ConfigError(f'{name} must be a list of strings, not {_type_name(value)}')
        items: list[str] = []
        for item in value:
            if not isinstance(item, str):
                raise ConfigError(f'{name} must contain only strings; found {_type_name(item)}')
            if item not in items:
                items.append(item)
        return tuple(items)


def _type_name(value: object) -> str:
    return type(value).__name__


def _nullable(value: str) -> str | None:
    return value if value else None


def _discard(_: str) -> None:
    """Output callable for the mirror built only to validate the configuration."""


# --------------------------------------------------------------------------- building settings


def settings_from_mapping(
    values: Mapping[str, Any],
    *,
    environ: Mapping[str, str] | None = None,
    logger: logging.Logger | None = None,
) -> BridgeSettings:
    """Build :class:`BridgeSettings` from parameter values; raise :class:`ConfigError` if unusable.

    ``values`` maps parameter names (``PARAMETER_NAMES``) to the values rclpy
    returned; a missing name takes the table's default, so a test can pass
    only what it cares about. ``environ`` defaults to :data:`os.environ`.
    """
    env = os.environ if environ is None else environ
    log = logger if logger is not None else LOGGER
    read = _Reader(values)

    robot = _robot_settings(read)
    mqtt = _mqtt_settings(read, robot.id, env, log)

    prefix = read.string('topic_prefix')
    problem = prefix_error(prefix)
    if problem is not None:
        raise ConfigError(f'topic_prefix {prefix!r} is not usable: {problem}')
    status_topic = read.string('ha_status_topic')
    problem = filter_error(status_topic)
    if problem is not None:
        raise ConfigError(f'ha_status_topic {status_topic!r} is not usable: {problem}')

    reply = ReplySettings(
        ack_expiry_s=read.integer('reply.ack_expiry_s', minimum=0),
        result_expiry_s=read.integer('reply.result_expiry_s', minimum=0),
        feedback_max_rate_hz=read.number('reply.feedback_max_rate_hz', minimum=0),
        retained_rejections_per_minute=read.integer(
            'reply.retained_rejections_per_minute', minimum=0
        ),
    )
    command = _command_settings(read, reply, env, log)

    data_dir_text = read.string('data_dir')
    data_dir = None if not data_dir_text.strip() else Path(os.path.expanduser(data_dir_text))

    mode = read.string('watchdog.mode')
    if mode not in WATCHDOG_MODES:
        modes = ', '.join(WATCHDOG_MODES)
        raise ConfigError(f'watchdog.mode must be one of {modes}, not {mode!r}')
    watchdog = WatchdogSettings(
        mode=_watchdog_mode(mode),
        timeout_s=read.number('watchdog.timeout_s', exclusive_minimum=0),
    )

    return BridgeSettings(
        robot=robot,
        mqtt=mqtt,
        topic_prefix=prefix,
        ha_status_topic=status_topic,
        announce_jitter_s=read.number('announce.jitter_s', minimum=0),
        executor=_executor_settings(read),
        command=command,
        data_dir=data_dir,
        reply=reply,
        heartbeat_interval_s=read.number('heartbeat.interval_s', minimum=0),
        telemetry=TelemetrySettings(
            joints_rate_hz=read.number('telemetry.joints_rate_hz', minimum=0),
            joints_min_delta=read.number('telemetry.joints_min_delta', minimum=0),
        ),
        watchdog=watchdog,
        mirror=_mirror_settings(read),
        fake=FakeSettings(
            task_duration_s=read.number('fake.task_duration_s', exclusive_minimum=0),
            feedback_hz=read.number('fake.feedback_hz', minimum=0),
        ),
    )


def _watchdog_mode(mode: str) -> WatchdogMode:
    for candidate in WATCHDOG_MODES:
        if mode == candidate:
            return candidate
    raise ConfigError(f'watchdog.mode {mode!r} is unknown')


def _robot_settings(read: _Reader) -> RobotSettings:
    robot_id = read.string('robot.id').strip()
    if not robot_id:
        raise ConfigError(
            'robot.id is required: set it in the params file (for example robot.id: arm_left)'
        )
    try:
        validate_robot_id(robot_id)
    except ValueError as error:
        raise ConfigError(f'robot.id: {error}') from error
    name = read.string('robot.name').strip() or robot_id
    return RobotSettings(
        id=robot_id,
        name=name,
        manufacturer=_nullable(read.string('robot.manufacturer')),
        model=_nullable(read.string('robot.model')),
        sw_version=_nullable(read.string('robot.sw_version')),
        hw_version=_nullable(read.string('robot.hw_version')),
        serial_number=_nullable(read.string('robot.serial_number')),
        suggested_area=_nullable(read.string('robot.suggested_area')),
    )


def _mqtt_settings(
    read: _Reader, robot_id: str, env: Mapping[str, str], log: logging.Logger
) -> MqttSettings:
    host = read.string('mqtt.host').strip()
    if not host:
        raise ConfigError('mqtt.host must name the MQTT broker')
    password = read.string('mqtt.password')
    password_from_env = False
    if not password:
        password = env.get(ENV_MQTT_PASSWORD, '')
        password_from_env = bool(password)
    client_id = read.string('mqtt.client_id').strip()
    derived = not client_id
    if derived:
        client_id = derived_client_id(robot_id)
    else:
        log.warning(
            'mqtt.client_id is set to %r; the default ros2_ha_bridge_{robot.id} is recommended '
            'so that two nodes with the same robot.id collide visibly on the broker',
            client_id,
        )
    reconnect_min_s = read.integer('mqtt.reconnect_min_s', minimum=1)
    reconnect_max_s = read.integer('mqtt.reconnect_max_s', minimum=reconnect_min_s)
    return MqttSettings(
        host=host,
        client_id=client_id,
        port=read.integer('mqtt.port', minimum=1, maximum=65535),
        username=read.string('mqtt.username'),
        password=password,
        password_from_env=password_from_env,
        client_id_derived=derived,
        tls=TlsSettings(
            enabled=read.boolean('mqtt.tls.enabled'),
            ca_cert=read.string('mqtt.tls.ca_cert'),
            certfile=read.string('mqtt.tls.certfile'),
            keyfile=read.string('mqtt.tls.keyfile'),
            insecure=read.boolean('mqtt.tls.insecure'),
        ),
        keepalive_s=read.integer('mqtt.keepalive_s', minimum=0),
        reconnect_min_s=reconnect_min_s,
        reconnect_max_s=reconnect_max_s,
        connect_timeout_s=read.number('mqtt.connect_timeout_s', minimum=0),
        takeover_exit_after=read.integer('mqtt.takeover_exit_after', minimum=0),
        max_payload_bytes=read.integer('mqtt.max_payload_bytes', minimum=1),
        inbound_queue_max=read.integer('mqtt.inbound_queue_max', minimum=1),
    )


def _executor_settings(read: _Reader) -> ExecutorSettings:
    spec = read.string('executor').strip()
    if not spec:
        raise ConfigError('executor must name an entry point (for example fake) or module:Class')
    allow = read.strings('tasks.allow')
    for name in allow:
        if not is_valid_task_name(name):
            raise ConfigError(
                f'tasks.allow entry {name!r} is not a task name (^[a-z][a-z0-9_]{{0,31}}$)'
            )
    return ExecutorSettings(
        spec=spec,
        call_warn_s=read.number('executor.call_warn_s', minimum=0),
        cancel_grace_s=read.number('executor.cancel_grace_s', exclusive_minimum=0),
        health_poll_s=read.number('executor.health_poll_s', exclusive_minimum=0),
        tasks_allow=allow,
    )


def _command_settings(
    read: _Reader, reply: ReplySettings, env: Mapping[str, str], log: logging.Logger
) -> CommandSettings:
    secret = read.string('command.secret')
    secret_from_env = False
    if not secret:
        secret = env.get(ENV_COMMAND_SECRET, '')
        secret_from_env = bool(secret)
    window_s = read.number('command.dedupe_window_s', exclusive_minimum=0)
    if window_s < reply.ack_expiry_s:
        log.warning(
            'command.dedupe_window_s %g is shorter than reply.ack_expiry_s %d; using %d so a '
            'redelivery within the ack lifetime republishes the stored ack',
            window_s,
            reply.ack_expiry_s,
            reply.ack_expiry_s,
        )
        window_s = float(reply.ack_expiry_s)
    return CommandSettings(
        secret=secret,
        secret_from_env=secret_from_env,
        max_age_cap_ms=read.integer('command.max_age_cap_ms', minimum=1),
        clock_skew_tolerance_ms=read.integer('command.clock_skew_tolerance_ms', minimum=0),
        dedupe_window_s=window_s,
        dedupe_size=read.integer('command.dedupe_size', minimum=1),
        task_timeout_cap_s=read.number('command.task_timeout_cap_s', exclusive_minimum=0),
    )


def _mirror_settings(read: _Reader) -> MirrorSettings:
    topics = read.strings('mirror.topics')
    suffix = read.string('mirror.availability_suffix')
    ros_topic = read.string('mirror.ros_topic').strip()
    if not ros_topic:
        raise ConfigError('mirror.ros_topic must name a ROS topic (the default is ~/mqtt_in)')
    try:
        # The mirror validates its own configuration (filters and suffix); reuse that one
        # implementation rather than restate its rules here.
        SensorMirror(topics, _discard, availability_suffix=suffix)
    except (TypeError, ValueError) as error:
        raise ConfigError(f'mirror: {error}') from error
    return MirrorSettings(topics=topics, availability_suffix=suffix, ros_topic=ros_topic)


# --------------------------------------------------------------------------- presentation


def describe_settings(settings: BridgeSettings) -> dict[str, Any]:
    """The effective values for the start-up log, with the password and the secret masked."""
    shown = settings.as_mapping()
    for name in SECRET_PARAMETERS:
        shown[name] = MASK if shown[name] else ''
    if settings.mqtt.password_from_env:
        shown['mqtt.password'] = f'{MASK} (from {ENV_MQTT_PASSWORD})'
    if settings.command.secret_from_env:
        shown['command.secret'] = f'{MASK} (from {ENV_COMMAND_SECRET})'
    return shown


def format_settings(settings: BridgeSettings) -> str:
    """One line of ``name=value`` pairs (masked) for the start-up log."""
    return ' '.join(f'{name}={value!r}' for name, value in describe_settings(settings).items())


def executor_parameters(settings: BridgeSettings) -> dict[str, Any]:
    """The parameter view handed to the executor plugin: every value except the secrets."""
    return {
        name: value
        for name, value in settings.as_mapping().items()
        if name not in SECRET_PARAMETERS
    }
