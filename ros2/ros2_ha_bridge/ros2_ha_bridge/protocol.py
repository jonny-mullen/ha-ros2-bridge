# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The ha-ros2-bridge MQTT contract (schema 1) as pure Python.

This module mirrors ``contract/README.md`` and the schemas under
``contract/schemas``: the identifier patterns, the topic layout, canonical JSON
and the command signature, the payload dataclasses the node publishes, the
decoder for the payloads it receives, the params-subset validator and the
freshness rule. It imports neither rclpy nor paho and performs no I/O, so the
contract tests run on any machine.

Two rules shape the decoder. A command whose ``id`` is not a lower-case UUID4
has no safe reply topic, so :func:`decode_command` reports it as
``invalid_id`` (or ``oversize`` or ``not_json`` before that) and the node drops
it without an ack. Everything else the schema would reject is reported as
``malformed`` together with the id, so the node can answer
``rejected_malformed``. The later stages (signature, dedupe, freshness,
allowlist, params, state gating) belong to the authoriser; the pure pieces of
them (:func:`verify_sig`, :func:`check_freshness`, :func:`validate_params`)
live here so both halves can be tested against the same vectors.
"""

from __future__ import annotations

import hashlib
import hmac
import json
import math
import os
import re
import socket
import uuid
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field, fields
from typing import Any, Literal, get_args

from ros2_ha_bridge import __version__
from ros2_ha_bridge.executors.base import (
    GOAL_STATUS,
    HEALTH_LEVELS,
    JOINT_UNITS,
    MAX_JOINTS,
    RESULT_STATUSES,
    TASK_KINDS,
    HealthLevel,
    JointInfo,
    JointUnit,
    ResultStatus,
    TaskKind,
    TaskSpec,
)

__all__ = [
    'ACK_CODES',
    'AVAILABILITY_OFFLINE',
    'AVAILABILITY_ONLINE',
    'COMMAND_TYPES',
    'COUNTER_NAMES',
    'DEFAULT_ACK_EXPIRY_S',
    'DEFAULT_CLOCK_SKEW_TOLERANCE_MS',
    'DEFAULT_HA_STATUS_TOPIC',
    'DEFAULT_MAX_AGE_CAP_MS',
    'DEFAULT_PREFIX',
    'DEFAULT_RESULT_EXPIRY_S',
    'DISCOVERY_PREFIX',
    'FEEDBACK_EXPIRY_S',
    'GOAL_STATUS',
    'HA_LINK_VALUES',
    'HEALTH_LEVELS',
    'HEARTBEAT_EXPIRY_S',
    'INSTANCE_RE',
    'JOINTS_EXPIRY_S',
    'JOINT_NAME_RE',
    'JOINT_UNITS',
    'MAX_ANNOUNCE_BYTES',
    'MAX_JOINTS',
    'MAX_PAYLOAD_BYTES',
    'MAX_TOPIC_BYTES',
    'PARAM_TYPES',
    'RESERVED_ROBOT_ID',
    'RESULT_CODE_RE',
    'RESULT_STATUSES',
    'ROBOT_ID_RE',
    'SCHEMA_VERSION',
    'SESSION_RE',
    'SIG_RE',
    'STATES',
    'TASK_ID_RE',
    'TASK_KINDS',
    'TASK_NAME_RE',
    'WATCHDOG_MODES',
    'AckCode',
    'AckPayload',
    'Announce',
    'AnnounceJoints',
    'Capabilities',
    'CommandType',
    'Counters',
    'DecodeError',
    'DecodeErrorKind',
    'DecodedCommand',
    'ExecutorHealth',
    'ExecutorInfo',
    'FeedbackPayload',
    'Freshness',
    'FreshnessVerdict',
    'HaHeartbeat',
    'HaLink',
    'HealthLevel',
    'HeartbeatPayload',
    'JointInfo',
    'JointUnit',
    'JointsPayload',
    'ParamType',
    'ResultPayload',
    'ResultStatus',
    'RobotError',
    'RobotState',
    'RobotTopics',
    'StatePayload',
    'TaskKind',
    'WatchdogMode',
    'ack_topic',
    'announce_topic',
    'availability_topic',
    'canonical_json',
    'check_freshness',
    'cmd_topic',
    'compute_sig',
    'decode_command',
    'decode_ha_heartbeat',
    'encode_payload',
    'feedback_topic',
    'ha_heartbeat_topic',
    'ha_sensor_topic',
    'heartbeat_topic',
    'is_valid_instance',
    'is_valid_joint_name',
    'is_valid_prefix',
    'is_valid_result_code',
    'is_valid_robot_id',
    'is_valid_session',
    'is_valid_task_id',
    'is_valid_task_name',
    'joints_topic',
    'local_instance',
    'new_session',
    'normalise_result_code',
    'prefix_error',
    'result_topic',
    'state_topic',
    'validate_joint_info',
    'validate_params',
    'validate_params_schema',
    'validate_prefix',
    'validate_publish_topic',
    'validate_robot_id',
    'validate_task_id',
    'validate_task_spec',
    'verify_sig',
]

# --------------------------------------------------------------------------- constants

#: Every JSON payload carries ``"schema": 1``; schema 1 only ever gains optional fields.
SCHEMA_VERSION = 1

#: Default topic prefix; the HA side offers the same default.
DEFAULT_PREFIX = 'ros2_bridge'
#: Robot-id segment reserved for Home Assistant's own topics (``{prefix}/ha/...``).
RESERVED_ROBOT_ID = 'ha'
#: Home Assistant's default MQTT discovery prefix; never usable as the bridge prefix.
DISCOVERY_PREFIX = 'homeassistant'
#: HA birth and will topic; the node re-announces on ``online``.
DEFAULT_HA_STATUS_TOPIC = 'homeassistant/status'

#: Size limits on the bytes actually received (16 KiB per message, 64 KiB for announce).
MAX_PAYLOAD_BYTES = 16 * 1024
MAX_ANNOUNCE_BYTES = 64 * 1024
#: MQTT's own limit on a topic name; paho raises ``ValueError`` above it.
MAX_TOPIC_BYTES = 65535

#: Message Expiry Intervals from the topic table (seconds).
HEARTBEAT_EXPIRY_S = 5
FEEDBACK_EXPIRY_S = 10
JOINTS_EXPIRY_S = 5
DEFAULT_ACK_EXPIRY_S = 120
DEFAULT_RESULT_EXPIRY_S = 300

#: Freshness rule defaults (``command.max_age_cap_ms``, ``command.clock_skew_tolerance_ms``).
DEFAULT_MAX_AGE_CAP_MS = 30_000
DEFAULT_CLOCK_SKEW_TOLERANCE_MS = 2_000

#: The availability payload is the plain word, not JSON.
AVAILABILITY_ONLINE = b'online'
AVAILABILITY_OFFLINE = b'offline'

# --------------------------------------------------------------------------- patterns

#: ``common.schema.json#/$defs/robot_id`` (plus the reserved ``ha`` rule, see is_valid_robot_id).
ROBOT_ID_RE = re.compile(r'[a-z][a-z0-9_]{0,31}')
#: ``common.schema.json#/$defs/task_name``: same shape as a robot id, nothing reserved.
TASK_NAME_RE = re.compile(r'[a-z][a-z0-9_]{0,31}')
#: ``common.schema.json#/$defs/joint_name`` (0.2.0): same shape again, nothing reserved.
JOINT_NAME_RE = re.compile(r'[a-z][a-z0-9_]{0,31}')
#: ``common.schema.json#/$defs/task_id``: a lower-case UUID4 string.
TASK_ID_RE = re.compile(r'[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}')
#: ``common.schema.json#/$defs/session``: same shape as a task id.
SESSION_RE = TASK_ID_RE
#: ``common.schema.json#/$defs/result_code``.
RESULT_CODE_RE = re.compile(r'[a-z][a-z0-9_]{0,63}')
#: ``common.schema.json#/$defs/instance``: ``<hostname>:<pid>``.
INSTANCE_RE = re.compile(r'[^:]+:[0-9]+')
#: ``common.schema.json#/$defs/sig``: 64 lower-case hex characters.
SIG_RE = re.compile(r'[0-9a-f]{64}')
#: Segments of ``{prefix}/ha/sensors/{domain}/{object_id}`` (sensor.schema.json).
_SENSOR_SEGMENT_RE = re.compile(r'[a-z0-9_]+')

# The patterns above are applied with ``fullmatch`` so that a trailing newline, which
# ``re.match`` with ``$`` would accept, is refused exactly as the JSON Schema refuses it.

# --------------------------------------------------------------------------- closed sets

AckCode = Literal[
    'accepted',
    'rejected_malformed',
    'rejected_unknown_task',
    'rejected_invalid_params',
    'rejected_stale',
    'rejected_future',
    'rejected_unauthorised',
    'rejected_busy',
    'rejected_error',
    'rejected_inhibited',
    'rejected_executor',
    'rejected_no_such_task',
]
CommandType = Literal['task', 'cancel']
RobotState = Literal['idle', 'busy', 'error']
HaLink = Literal['up', 'down']
WatchdogMode = Literal['none', 'inhibit_new', 'stop_and_hold']
ParamType = Literal['string', 'number', 'integer', 'boolean']

ACK_CODES: tuple[AckCode, ...] = get_args(AckCode)
COMMAND_TYPES: tuple[CommandType, ...] = get_args(CommandType)
STATES: tuple[RobotState, ...] = get_args(RobotState)
HA_LINK_VALUES: tuple[HaLink, ...] = get_args(HaLink)
WATCHDOG_MODES: tuple[WatchdogMode, ...] = get_args(WatchdogMode)
PARAM_TYPES: tuple[ParamType, ...] = get_args(ParamType)

# --------------------------------------------------------------------------- identifiers


def is_valid_robot_id(value: object) -> bool:
    """True for a string matching the robot-id pattern that is not the reserved ``ha``."""
    return (
        isinstance(value, str)
        and ROBOT_ID_RE.fullmatch(value) is not None
        and value != RESERVED_ROBOT_ID
    )


def is_valid_task_name(value: object) -> bool:
    """True for a string matching the task-name pattern."""
    return isinstance(value, str) and TASK_NAME_RE.fullmatch(value) is not None


def is_valid_joint_name(value: object) -> bool:
    """True for a string matching the joint-name pattern (0.2.0)."""
    return isinstance(value, str) and JOINT_NAME_RE.fullmatch(value) is not None


def is_valid_task_id(value: object) -> bool:
    """True for a lower-case UUID4 string; the only ids that may name a reply topic."""
    return isinstance(value, str) and TASK_ID_RE.fullmatch(value) is not None


def is_valid_session(value: object) -> bool:
    """True for a lower-case UUID4 string."""
    return isinstance(value, str) and SESSION_RE.fullmatch(value) is not None


def is_valid_result_code(value: object) -> bool:
    """True for a string matching the result-code pattern."""
    return isinstance(value, str) and RESULT_CODE_RE.fullmatch(value) is not None


def is_valid_instance(value: object) -> bool:
    """True for a ``<hostname>:<pid>`` string."""
    return isinstance(value, str) and INSTANCE_RE.fullmatch(value) is not None


def validate_robot_id(robot_id: str) -> str:
    """Return ``robot_id`` or raise ``ValueError`` saying why it is unusable."""
    if robot_id == RESERVED_ROBOT_ID:
        raise ValueError(f"robot id {robot_id!r} is reserved for Home Assistant's own topics")
    if ROBOT_ID_RE.fullmatch(robot_id) is None:
        raise ValueError(
            f'robot id {robot_id!r} must match ^[a-z][a-z0-9_]{{0,31}}$ '
            '(lower-case, starts with a letter, at most 32 characters)'
        )
    return robot_id


def validate_task_id(task_id: str) -> str:
    """Return ``task_id`` or raise ``ValueError``; only UUID4 ids may name a reply topic."""
    if not is_valid_task_id(task_id):
        raise ValueError(f'task id {task_id!r} is not a lower-case UUID4 string')
    return task_id


def normalise_result_code(code: object) -> str:
    """Coerce an executor-supplied code onto the wire pattern.

    Lower-cases, replaces runs of other characters with ``_``, trims to 64
    characters and falls back to ``unknown`` when nothing usable remains, so a
    sloppy plugin can never produce a result the schema rejects.
    """
    if not isinstance(code, str):
        return 'unknown'
    if is_valid_result_code(code):
        return code
    cleaned = re.sub(r'[^a-z0-9_]+', '_', code.lower()).strip('_')
    cleaned = re.sub(r'^[^a-z]+', '', cleaned)[:64]
    return cleaned if is_valid_result_code(cleaned) else 'unknown'


def local_instance() -> str:
    """``"<hostname>:<pid>"`` for this process, matching :data:`INSTANCE_RE`."""
    host = socket.gethostname().replace(':', '_') or 'unknown'
    return f'{host}:{os.getpid()}'


def new_session() -> str:
    """A fresh lower-case UUID4 for this node process."""
    return str(uuid.uuid4())


# --------------------------------------------------------------------------- topics


def prefix_error(prefix: object) -> str | None:
    """Return why ``prefix`` is not an acceptable topic prefix, or ``None`` when it is.

    The contract's rules: no leading or trailing slash, no ``+`` or ``#``, not
    ``homeassistant`` and not starting with the segment ``ha``. Empty segments
    (``a//b``) and NUL characters are refused as well because no broker accepts
    them. The discovery prefix is refused as a first segment too, so that no
    bridge topic can ever sit under HA's discovery tree.
    """
    if not isinstance(prefix, str):
        return 'prefix must be a string'
    if not prefix:
        return 'prefix is empty'
    if '\x00' in prefix:
        return 'prefix contains a NUL character'
    if '+' in prefix or '#' in prefix:
        return "prefix contains a wildcard character ('+' or '#')"
    if prefix.startswith('/') or prefix.endswith('/'):
        return 'prefix must not start or end with a slash'
    segments = prefix.split('/')
    if '' in segments:
        return 'prefix contains an empty segment'
    if segments[0] == RESERVED_ROBOT_ID:
        return f"prefix must not start with the reserved segment '{RESERVED_ROBOT_ID}'"
    if segments[0] == DISCOVERY_PREFIX:
        return f"prefix must not be or start with the discovery prefix '{DISCOVERY_PREFIX}'"
    return None


def is_valid_prefix(prefix: object) -> bool:
    """True when :func:`prefix_error` finds nothing wrong."""
    return prefix_error(prefix) is None


def validate_prefix(prefix: str) -> str:
    """Return ``prefix`` or raise ``ValueError`` with the reason it is refused."""
    error = prefix_error(prefix)
    if error is not None:
        raise ValueError(f'invalid topic prefix {prefix!r}: {error}')
    return prefix


def validate_publish_topic(topic: str) -> str:
    """Return ``topic`` or raise ``ValueError``; the guard every publish passes first.

    A topic built by the functions below always passes; this exists so that a
    publisher can refuse a wildcard, an empty topic, a NUL character or an
    over-long topic before paho raises inside the network code.
    """
    if not isinstance(topic, str) or not topic:
        raise ValueError('publish topic must be a non-empty string')
    if '+' in topic or '#' in topic:
        raise ValueError(f'publish topic {topic!r} contains a wildcard character')
    if '\x00' in topic:
        raise ValueError('publish topic contains a NUL character')
    if len(topic.encode('utf-8')) > MAX_TOPIC_BYTES:
        raise ValueError(f'publish topic is longer than {MAX_TOPIC_BYTES} bytes')
    return topic


def _robot_base(prefix: str, robot_id: str) -> str:
    return f'{validate_prefix(prefix)}/{validate_robot_id(robot_id)}'


def announce_topic(prefix: str, robot_id: str) -> str:
    """``{prefix}/{robot_id}/announce`` (ROS to HA, QoS 1, retained, no expiry)."""
    return f'{_robot_base(prefix, robot_id)}/announce'


def availability_topic(prefix: str, robot_id: str) -> str:
    """``{prefix}/{robot_id}/availability`` (plain ``online``/``offline``, retained, the LWT)."""
    return f'{_robot_base(prefix, robot_id)}/availability'


def state_topic(prefix: str, robot_id: str) -> str:
    """``{prefix}/{robot_id}/state`` (ROS to HA, QoS 1, retained, no expiry)."""
    return f'{_robot_base(prefix, robot_id)}/state'


def heartbeat_topic(prefix: str, robot_id: str) -> str:
    """``{prefix}/{robot_id}/heartbeat`` (ROS to HA, QoS 0, not retained, expiry 5 s)."""
    return f'{_robot_base(prefix, robot_id)}/heartbeat'


def joints_topic(prefix: str, robot_id: str) -> str:
    """``{prefix}/{robot_id}/joints`` (ROS to HA, QoS 0, not retained, expiry 5 s; 0.2.0)."""
    return f'{_robot_base(prefix, robot_id)}/joints'


def cmd_topic(prefix: str, robot_id: str) -> str:
    """``{prefix}/{robot_id}/cmd`` (HA to ROS, QoS 1, never retained)."""
    return f'{_robot_base(prefix, robot_id)}/cmd'


def ack_topic(prefix: str, robot_id: str, task_id: str) -> str:
    """``{prefix}/{robot_id}/task/{task_id}/ack``.

    QoS 1, retained, Message Expiry Interval ``reply.ack_expiry_s`` (120 s).
    """
    return f'{_robot_base(prefix, robot_id)}/task/{validate_task_id(task_id)}/ack'


def feedback_topic(prefix: str, robot_id: str, task_id: str) -> str:
    """``{prefix}/{robot_id}/task/{task_id}/feedback`` (QoS 0, not retained, expiry 10 s)."""
    return f'{_robot_base(prefix, robot_id)}/task/{validate_task_id(task_id)}/feedback'


def result_topic(prefix: str, robot_id: str, task_id: str) -> str:
    """``{prefix}/{robot_id}/task/{task_id}/result``.

    QoS 1, retained, Message Expiry Interval ``reply.result_expiry_s`` (300 s).
    """
    return f'{_robot_base(prefix, robot_id)}/task/{validate_task_id(task_id)}/result'


def ha_heartbeat_topic(prefix: str) -> str:
    """``{prefix}/ha/heartbeat`` (HA to ROS, QoS 0, not retained, expiry 5 s)."""
    return f'{validate_prefix(prefix)}/{RESERVED_ROBOT_ID}/heartbeat'


def ha_sensor_topic(prefix: str, domain: str, object_id: str) -> str:
    """``{prefix}/ha/sensors/{domain}/{object_id}`` (HA to ROS, QoS 0, retained, no expiry)."""
    for label, segment in (('domain', domain), ('object_id', object_id)):
        if not isinstance(segment, str) or _SENSOR_SEGMENT_RE.fullmatch(segment) is None:
            raise ValueError(f'sensor {label} {segment!r} must match ^[a-z0-9_]+$')
    return f'{validate_prefix(prefix)}/{RESERVED_ROBOT_ID}/sensors/{domain}/{object_id}'


@dataclass(frozen=True)
class RobotTopics:
    """All topics of one robot under one prefix, validated once at construction."""

    robot_id: str
    prefix: str = DEFAULT_PREFIX

    def __post_init__(self) -> None:
        validate_prefix(self.prefix)
        validate_robot_id(self.robot_id)

    @property
    def announce(self) -> str:
        return announce_topic(self.prefix, self.robot_id)

    @property
    def availability(self) -> str:
        return availability_topic(self.prefix, self.robot_id)

    @property
    def state(self) -> str:
        return state_topic(self.prefix, self.robot_id)

    @property
    def heartbeat(self) -> str:
        return heartbeat_topic(self.prefix, self.robot_id)

    @property
    def joints(self) -> str:
        return joints_topic(self.prefix, self.robot_id)

    @property
    def cmd(self) -> str:
        return cmd_topic(self.prefix, self.robot_id)

    @property
    def ha_heartbeat(self) -> str:
        return ha_heartbeat_topic(self.prefix)

    def ack(self, task_id: str) -> str:
        return ack_topic(self.prefix, self.robot_id, task_id)

    def feedback(self, task_id: str) -> str:
        return feedback_topic(self.prefix, self.robot_id, task_id)

    def result(self, task_id: str) -> str:
        return result_topic(self.prefix, self.robot_id, task_id)


# --------------------------------------------------------------------------- JSON and HMAC


def canonical_json(payload: Mapping[str, Any]) -> bytes:
    """The bytes that are signed: ``sig`` removed, keys sorted at every level, compact, UTF-8.

    ``ensure_ascii=False`` means non-ASCII text is signed as its UTF-8 bytes,
    not as ``\\u`` escapes, and list order is preserved. Matches
    ``contract/hmac_vectors.json`` byte for byte.
    """
    unsigned = {key: value for key, value in payload.items() if key != 'sig'}
    return json.dumps(unsigned, sort_keys=True, separators=(',', ':'), ensure_ascii=False).encode(
        'utf-8'
    )


def compute_sig(secret: str, topic: str, payload: Mapping[str, Any]) -> str:
    """``hex(HMAC-SHA256(secret, topic + "\\n" + canonical))``; a present ``sig`` is ignored.

    The topic is part of the signed message so a command signed for one robot or
    one prefix cannot be replayed on another.
    """
    message = topic.encode('utf-8') + b'\n' + canonical_json(payload)
    return hmac.new(secret.encode('utf-8'), message, hashlib.sha256).hexdigest()


def verify_sig(secret: str, topic: str, payload: Mapping[str, Any], sig: object = None) -> bool:
    """Constant-time check of ``sig`` (or ``payload['sig']``) against the recomputed digest.

    Anything that is not a 64-character lower-case hex string fails without
    touching the secret, and a payload that cannot be canonicalised (an
    unpaired surrogate) fails rather than raising.
    """
    candidate = payload.get('sig') if sig is None else sig
    if not isinstance(candidate, str) or SIG_RE.fullmatch(candidate) is None:
        return False
    try:
        expected = compute_sig(secret, topic, payload)
    except UnicodeEncodeError:
        return False
    return hmac.compare_digest(expected, candidate)


def encode_payload(payload: Mapping[str, Any]) -> bytes:
    """The wire form of a payload: compact JSON, UTF-8, key order as given."""
    return json.dumps(payload, separators=(',', ':'), ensure_ascii=False).encode('utf-8')


# --------------------------------------------------------------------------- outbound payloads


@dataclass(frozen=True)
class Capabilities:
    """``announce.capabilities``: what this node and its executor support."""

    cancel: bool = True
    feedback: bool = True
    hmac_required: bool = False

    def to_payload(self) -> dict[str, Any]:
        return {
            'cancel': self.cancel,
            'feedback': self.feedback,
            'hmac_required': self.hmac_required,
        }


@dataclass(frozen=True)
class ExecutorInfo:
    """``announce.executor``: name and version of the loaded plugin."""

    name: str
    version: str

    def to_payload(self) -> dict[str, Any]:
        return {'name': self.name, 'version': self.version}


def _task_spec_payload(spec: TaskSpec) -> dict[str, Any]:
    entry: dict[str, Any] = {
        'name': spec.name,
        'label': spec.label,
        'kind': spec.kind,
        'timeout_s': spec.timeout_s,
        'params': None if spec.params is None else dict(spec.params),
    }
    if spec.description:
        entry['description'] = spec.description
    return entry


@dataclass(frozen=True)
class AnnounceJoints:
    """``announce.joints`` (0.2.0, optional): what the ``joints`` topic will carry.

    Built from the executor's :class:`~ros2_ha_bridge.executors.base.JointInfo`
    with :meth:`from_joint_info`; ``rate_hz`` is the node's
    ``telemetry.joints_rate_hz``. A node with that rate at 0 announces no
    block at all rather than a block with ``rate_hz`` 0, which the schema
    refuses.
    """

    unit: JointUnit
    names: Sequence[str]
    limits: Sequence[tuple[float, float]] | None
    rate_hz: float

    @classmethod
    def from_joint_info(cls, info: JointInfo, rate_hz: float) -> AnnounceJoints:
        limits = (
            None
            if info.limits is None
            else tuple((float(low), float(high)) for low, high in info.limits)
        )
        return cls(info.unit, tuple(info.names), limits, float(rate_hz))

    def to_payload(self) -> dict[str, Any]:
        return {
            'unit': self.unit,
            'names': list(self.names),
            'limits': None if self.limits is None else [list(pair) for pair in self.limits],
            'rate_hz': self.rate_hz,
        }


@dataclass(frozen=True, kw_only=True)
class Announce:
    """``{prefix}/{robot_id}/announce``: identity plus the task catalogue.

    ``tasks`` is the catalogue after the operator's ``tasks.allow`` intersection.
    The nullable identity strings are always emitted (as ``null`` when unknown);
    ``executor`` and ``joints`` are emitted when given.
    """

    robot_id: str
    name: str
    session: str
    instance: str
    tasks: Sequence[TaskSpec]
    ts: int
    bridge_version: str = __version__
    capabilities: Capabilities = field(default_factory=Capabilities)
    watchdog_mode: WatchdogMode = 'none'
    heartbeat_interval_s: float = 2
    executor: ExecutorInfo | None = None
    manufacturer: str | None = None
    model: str | None = None
    sw_version: str | None = None
    hw_version: str | None = None
    serial_number: str | None = None
    suggested_area: str | None = None
    joints: AnnounceJoints | None = None

    def to_payload(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            'schema': SCHEMA_VERSION,
            'robot_id': self.robot_id,
            'name': self.name,
            'manufacturer': self.manufacturer,
            'model': self.model,
            'sw_version': self.sw_version,
            'hw_version': self.hw_version,
            'serial_number': self.serial_number,
            'suggested_area': self.suggested_area,
            'bridge_version': self.bridge_version,
        }
        if self.executor is not None:
            payload['executor'] = self.executor.to_payload()
        payload.update(
            {
                'session': self.session,
                'instance': self.instance,
                'capabilities': self.capabilities.to_payload(),
                'watchdog_mode': self.watchdog_mode,
                'heartbeat_interval_s': self.heartbeat_interval_s,
                'tasks': [_task_spec_payload(spec) for spec in self.tasks],
            }
        )
        if self.joints is not None:
            payload['joints'] = self.joints.to_payload()
        payload['ts'] = self.ts
        return payload


@dataclass(frozen=True)
class ExecutorHealth:
    """``state.executor_health``: last polled health of the plugin."""

    level: HealthLevel = 'ok'
    message: str = ''

    def to_payload(self) -> dict[str, Any]:
        return {'level': self.level, 'message': self.message}


@dataclass(frozen=True)
class RobotError:
    """``state.error`` when not null: a robot-level fault, never a failed task alone."""

    code: str
    reason: str

    def to_payload(self) -> dict[str, Any]:
        return {'code': normalise_result_code(self.code), 'reason': self.reason}


@dataclass(frozen=True, kw_only=True)
class StatePayload:
    """``{prefix}/{robot_id}/state``: retained, published on change and on (re)connect.

    ``state`` is exactly ``idle``, ``busy`` or ``error``; ``paused``,
    ``inhibited`` and ``ha_link`` are separate fields. ``seq`` increments per
    publication within a session so HA can drop a regressing retained copy.
    """

    robot_id: str
    state: RobotState
    since: int
    session: str
    instance: str
    seq: int
    ts: int
    task_id: str | None = None
    task: str | None = None
    paused: bool = False
    inhibited: bool = False
    ha_link: HaLink = 'down'
    watchdog_mode: WatchdogMode = 'none'
    executor_health: ExecutorHealth = field(default_factory=ExecutorHealth)
    error: RobotError | None = None

    def to_payload(self) -> dict[str, Any]:
        return {
            'schema': SCHEMA_VERSION,
            'robot_id': self.robot_id,
            'state': self.state,
            'task_id': self.task_id,
            'task': self.task,
            'since': self.since,
            'paused': self.paused,
            'inhibited': self.inhibited,
            'ha_link': self.ha_link,
            'watchdog_mode': self.watchdog_mode,
            'executor_health': self.executor_health.to_payload(),
            'error': None if self.error is None else self.error.to_payload(),
            'session': self.session,
            'instance': self.instance,
            'seq': self.seq,
            'ts': self.ts,
        }


@dataclass
class Counters:
    """``heartbeat.counters``: lifetime counters of the node process (mutable on purpose)."""

    accepted: int = 0
    rejected: int = 0
    duplicates: int = 0
    dropped_retained: int = 0
    dropped_oversize: int = 0
    dropped_invalid_id: int = 0
    dropped_queue_full: int = 0
    publish_errors: int = 0
    takeovers: int = 0

    def to_payload(self) -> dict[str, Any]:
        return {name: getattr(self, name) for name in COUNTER_NAMES}


#: The required counter names, in schema order.
COUNTER_NAMES: tuple[str, ...] = tuple(item.name for item in fields(Counters))


@dataclass(frozen=True, kw_only=True)
class HeartbeatPayload:
    """``{prefix}/{robot_id}/heartbeat``: QoS 0, not retained, expiry 5 s."""

    robot_id: str
    session: str
    instance: str
    seq: int
    ts: int
    uptime_s: float
    counters: Counters
    ha_clock_offset_ms: int | None = None

    def to_payload(self) -> dict[str, Any]:
        return {
            'schema': SCHEMA_VERSION,
            'robot_id': self.robot_id,
            'session': self.session,
            'instance': self.instance,
            'seq': self.seq,
            'ts': self.ts,
            'uptime_s': self.uptime_s,
            'ha_clock_offset_ms': self.ha_clock_offset_ms,
            'counters': self.counters.to_payload(),
        }


@dataclass(frozen=True, kw_only=True)
class JointsPayload:
    """``{prefix}/{robot_id}/joints`` (0.2.0): QoS 0, not retained, expiry 5 s.

    ``positions`` holds one number per announced joint in the announced unit
    and order; ``velocities`` is ``None`` or one number per position. ``seq``
    increments per publication within a session so Home Assistant can drop a
    late copy that overtook a newer one.
    """

    robot_id: str
    positions: Sequence[float]
    session: str
    instance: str
    seq: int
    ts: int
    velocities: Sequence[float] | None = None

    def to_payload(self) -> dict[str, Any]:
        return {
            'schema': SCHEMA_VERSION,
            'robot_id': self.robot_id,
            'positions': [float(value) for value in self.positions],
            'velocities': (
                None if self.velocities is None else [float(value) for value in self.velocities]
            ),
            'session': self.session,
            'instance': self.instance,
            'seq': self.seq,
            'ts': self.ts,
        }


@dataclass(frozen=True, kw_only=True)
class AckPayload:
    """``{prefix}/{robot_id}/task/{id}/ack``: one per command id, retained with expiry.

    ``accepted`` is derived from ``code`` so the pair can never disagree.
    ``task`` is ``None`` for a cancel or for a command too malformed to read;
    ``clock_skew_ms`` is ``None`` when ``issued_at`` could not be read.
    """

    id: str
    robot_id: str
    type: CommandType
    code: AckCode
    received_at: int
    session: str
    ts: int
    task: str | None = None
    reason: str = ''
    clock_skew_ms: int | None = None

    @property
    def accepted(self) -> bool:
        return self.code == 'accepted'

    def to_payload(self) -> dict[str, Any]:
        return {
            'schema': SCHEMA_VERSION,
            'id': self.id,
            'robot_id': self.robot_id,
            'type': self.type,
            'task': self.task,
            'accepted': self.accepted,
            'code': self.code,
            'reason': self.reason,
            'received_at': self.received_at,
            'clock_skew_ms': self.clock_skew_ms,
            'session': self.session,
            'ts': self.ts,
        }


@dataclass(frozen=True, kw_only=True)
class FeedbackPayload:
    """``{prefix}/{robot_id}/task/{id}/feedback``: QoS 0, not retained, expiry 10 s."""

    id: str
    robot_id: str
    session: str
    ts: int
    progress: float | None = None
    stage: str = ''
    message: str = ''

    def to_payload(self) -> dict[str, Any]:
        return {
            'schema': SCHEMA_VERSION,
            'id': self.id,
            'robot_id': self.robot_id,
            'progress': self.progress,
            'stage': self.stage,
            'message': self.message,
            'session': self.session,
            'ts': self.ts,
        }


@dataclass(frozen=True, kw_only=True)
class ResultPayload:
    """``{prefix}/{robot_id}/task/{id}/result``: exactly one per accepted task command.

    ``goal_status`` is derived from ``status`` (``succeeded`` 4, ``cancelled``
    5, ``aborted`` 6) and ``code`` is normalised onto the wire pattern.
    """

    id: str
    robot_id: str
    task: str
    kind: TaskKind
    status: ResultStatus
    code: str
    started_at: int
    finished_at: int
    session: str
    ts: int
    reason: str = ''
    data: Mapping[str, Any] = field(default_factory=dict)

    @property
    def goal_status(self) -> int:
        return GOAL_STATUS[self.status]

    def to_payload(self) -> dict[str, Any]:
        return {
            'schema': SCHEMA_VERSION,
            'id': self.id,
            'robot_id': self.robot_id,
            'task': self.task,
            'kind': self.kind,
            'status': self.status,
            'goal_status': self.goal_status,
            'code': normalise_result_code(self.code),
            'reason': self.reason,
            'data': dict(self.data),
            'started_at': self.started_at,
            'finished_at': self.finished_at,
            'session': self.session,
            'ts': self.ts,
        }


# --------------------------------------------------------------------------- inbound decoding

DecodeErrorKind = Literal['oversize', 'not_json', 'invalid_id', 'malformed']


@dataclass(frozen=True)
class DecodeError:
    """Why an inbound payload was refused, and what the node may still do about it.

    ``oversize``, ``not_json`` and ``invalid_id`` have no safe reply topic: the
    node drops the message and counts it (``dropped_oversize`` for the first,
    ``dropped_invalid_id`` for the other two). ``malformed`` carries
    ``command_id`` (a verified UUID4), so the node answers ``rejected_malformed``
    on that id's ack topic, using ``ack_type`` for the ack's ``type`` field,
    ``task`` when it could be read and ``issued_at`` for ``clock_skew_ms``.
    """

    kind: DecodeErrorKind
    reason: str
    size: int = 0
    command_id: str | None = None
    command_type: CommandType | None = None
    task: str | None = None
    issued_at: int | None = None

    @property
    def replyable(self) -> bool:
        """True when a safe reply topic exists (only ``malformed``)."""
        return self.command_id is not None

    @property
    def ack_type(self) -> CommandType:
        """The ``type`` to put in a ``rejected_malformed`` ack; ``task`` when unreadable."""
        return self.command_type if self.command_type is not None else 'task'


@dataclass(frozen=True)
class DecodedCommand:
    """A structurally valid command. Authorisation (signature, freshness, allowlist) comes next.

    For ``type == 'task'``: ``task``, ``params`` and ``timeout_s`` (``None``
    means the announced value) are set and ``target_id`` is ``None``. For
    ``type == 'cancel'``: ``target_id`` is the task to cancel or ``None`` for
    the active motion task, and the task fields are ``None``. ``payload`` is
    the whole decoded object (with ``sig`` if present) for signature checks.
    """

    id: str
    type: CommandType
    robot_id: str
    issued_at: int
    max_age_ms: int
    source: Mapping[str, Any]
    payload: Mapping[str, Any]
    size: int
    task: str | None = None
    params: Mapping[str, Any] | None = None
    timeout_s: float | None = None
    target_id: str | None = None
    sig: str | None = None

    @property
    def is_task(self) -> bool:
        return self.type == 'task'

    @property
    def is_cancel(self) -> bool:
        return self.type == 'cancel'


def _is_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _as_int(value: object, *, minimum: int | None = None) -> int | None:
    """``value`` when it is a JSON integer (never a bool) at or above ``minimum``, else None."""
    if not isinstance(value, int) or isinstance(value, bool):
        return None
    if minimum is not None and value < minimum:
        return None
    return value


def _is_number(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _reject_constant(name: str) -> object:
    raise ValueError(f'{name} is not valid JSON')


@dataclass(frozen=True)
class _JsonFailure:
    reason: str


def _parse_json(raw: bytes) -> object:
    """Parse strict JSON (no NaN or Infinity); return a ``_JsonFailure`` instead of raising."""
    try:
        text = raw.decode('utf-8')
    except UnicodeDecodeError as error:
        return _JsonFailure(f'payload is not UTF-8: {error.reason}')
    try:
        parsed: object = json.loads(text, parse_constant=_reject_constant)
    except ValueError as error:
        return _JsonFailure(f'payload is not JSON: {error}')
    return parsed


def _source_error(source: object) -> str | None:
    if not isinstance(source, dict):
        return 'source must be an object'
    for key in ('origin', 'kind'):
        value = source.get(key)
        if not isinstance(value, str) or not value:
            return f'source.{key} must be a non-empty string'
    if 'instance_id' in source:
        instance_id = source['instance_id']
        if not isinstance(instance_id, str) or not instance_id:
            return 'source.instance_id must be a non-empty string when present'
    for key in ('user_id', 'context_id', 'device_id'):
        if key in source and source[key] is not None and not isinstance(source[key], str):
            return f'source.{key} must be a string or null'
    return None


def decode_command(
    raw: bytes,
    max_bytes: int | None = MAX_PAYLOAD_BYTES,
    *,
    expected_robot_id: str | None = None,
) -> DecodedCommand | DecodeError:
    """Decode the bytes received on ``{prefix}/{robot_id}/cmd``.

    The checks run in the contract's order and the first failure wins: size
    (``oversize``), UTF-8 and JSON (``not_json``), then ``id`` must be a
    lower-case UUID4 (``invalid_id``); from there a safe reply topic exists and
    every remaining schema rule (``schema`` is 1, ``type`` is ``task`` or
    ``cancel``, ``robot_id`` valid and equal to ``expected_robot_id`` when
    given, ``issued_at`` and ``max_age_ms`` integers, ``source`` well formed,
    ``sig`` 64 hex characters when present, and the per-type fields) yields
    ``malformed`` with a reason. ``max_bytes=None`` disables the size check,
    which the tests use to show what a huge id would otherwise do.
    """
    size = len(raw)
    if max_bytes is not None and size > max_bytes:
        return DecodeError('oversize', f'{size} bytes is over the limit of {max_bytes}', size)

    parsed = _parse_json(raw)
    if isinstance(parsed, _JsonFailure):
        return DecodeError('not_json', parsed.reason, size)
    if not isinstance(parsed, dict):
        return DecodeError('invalid_id', 'payload is not a JSON object, so it has no id', size)

    command_id = parsed.get('id')
    if not is_valid_task_id(command_id):
        return DecodeError('invalid_id', 'id is missing or not a lower-case UUID4 string', size)
    assert isinstance(command_id, str)

    # From here a safe reply topic exists. Collect what a rejected_malformed ack can use.
    raw_type = parsed.get('type')
    command_type: CommandType | None = None
    if raw_type == 'task' or raw_type == 'cancel':
        command_type = raw_type
    raw_task = parsed.get('task')
    task_hint = raw_task if is_valid_task_name(raw_task) else None
    issued_hint = _as_int(parsed.get('issued_at'), minimum=0)

    def malformed(reason: str) -> DecodeError:
        return DecodeError(
            'malformed',
            reason,
            size,
            command_id=command_id,
            command_type=command_type,
            task=task_hint,
            issued_at=issued_hint,
        )

    try:
        canonical_json(parsed)
    except UnicodeEncodeError:
        return malformed('payload contains an unpaired surrogate character')

    schema = parsed.get('schema')
    if not _is_int(schema) or schema != SCHEMA_VERSION:
        return malformed(f'schema must be {SCHEMA_VERSION}')
    if command_type is None:
        return malformed("type must be 'task' or 'cancel'")
    robot_id = parsed.get('robot_id')
    if not is_valid_robot_id(robot_id):
        return malformed('robot_id is missing, reserved or does not match the pattern')
    assert isinstance(robot_id, str)
    if expected_robot_id is not None and robot_id != expected_robot_id:
        return malformed(f'robot_id {robot_id!r} is not this node ({expected_robot_id!r})')
    if issued_hint is None:
        return malformed('issued_at must be a non-negative integer (epoch milliseconds)')
    max_age_ms = _as_int(parsed.get('max_age_ms'), minimum=1)
    if max_age_ms is None:
        return malformed('max_age_ms must be an integer of at least 1')
    source = parsed.get('source')
    source_problem = _source_error(source)
    if source_problem is not None:
        return malformed(source_problem)
    assert isinstance(source, dict)
    sig: str | None = None
    if 'sig' in parsed:
        raw_sig = parsed['sig']
        if not isinstance(raw_sig, str) or SIG_RE.fullmatch(raw_sig) is None:
            return malformed('sig must be 64 lower-case hex characters when present')
        sig = raw_sig

    if command_type == 'task':
        if task_hint is None:
            return malformed('task is missing or does not match the task-name pattern')
        params = parsed.get('params')
        if not isinstance(params, dict):
            return malformed('params must be an object (empty when the task takes none)')
        timeout_s: float | None = None
        if parsed.get('timeout_s') is not None:
            raw_timeout = parsed['timeout_s']
            if not _is_number(raw_timeout) or raw_timeout <= 0:
                return malformed('timeout_s must be null or a number greater than 0')
            timeout_s = float(raw_timeout)
        return DecodedCommand(
            id=command_id,
            type='task',
            robot_id=robot_id,
            issued_at=issued_hint,
            max_age_ms=max_age_ms,
            source=source,
            payload=parsed,
            size=size,
            task=task_hint,
            params=params,
            timeout_s=timeout_s,
            sig=sig,
        )

    if 'target_id' not in parsed:
        return malformed('target_id is required for a cancel (null means the active task)')
    target_id = parsed['target_id']
    if target_id is not None and not is_valid_task_id(target_id):
        return malformed('target_id must be null or a lower-case UUID4 string')
    assert target_id is None or isinstance(target_id, str)
    return DecodedCommand(
        id=command_id,
        type='cancel',
        robot_id=robot_id,
        issued_at=issued_hint,
        max_age_ms=max_age_ms,
        source=source,
        payload=parsed,
        size=size,
        target_id=target_id,
        sig=sig,
    )


@dataclass(frozen=True)
class HaHeartbeat:
    """``{prefix}/ha/heartbeat`` as received: feeds the watchdog and ``ha_clock_offset_ms``."""

    ts: int
    seq: int
    interval_ms: int
    instance_id: str


def decode_ha_heartbeat(
    raw: bytes, max_bytes: int | None = MAX_PAYLOAD_BYTES
) -> HaHeartbeat | DecodeError:
    """Decode an HA heartbeat; failures are ``oversize``, ``not_json`` or ``malformed``.

    There is no reply topic for a heartbeat, so ``malformed`` here only means
    "ignore and log"; the kinds are shared with :func:`decode_command` so the
    node can count drops the same way.
    """
    size = len(raw)
    if max_bytes is not None and size > max_bytes:
        return DecodeError('oversize', f'{size} bytes is over the limit of {max_bytes}', size)
    parsed = _parse_json(raw)
    if isinstance(parsed, _JsonFailure):
        return DecodeError('not_json', parsed.reason, size)
    if not isinstance(parsed, dict):
        return DecodeError('malformed', 'payload is not a JSON object', size)
    schema = parsed.get('schema')
    if not _is_int(schema) or schema != SCHEMA_VERSION:
        return DecodeError('malformed', f'schema must be {SCHEMA_VERSION}', size)
    ts = _as_int(parsed.get('ts'), minimum=0)
    if ts is None:
        return DecodeError('malformed', 'ts must be a non-negative integer', size)
    seq = _as_int(parsed.get('seq'), minimum=0)
    if seq is None:
        return DecodeError('malformed', 'seq must be a non-negative integer', size)
    interval_ms = _as_int(parsed.get('interval_ms'), minimum=1)
    if interval_ms is None:
        return DecodeError('malformed', 'interval_ms must be an integer of at least 1', size)
    instance_id = parsed.get('instance_id')
    if not isinstance(instance_id, str) or not instance_id:
        return DecodeError('malformed', 'instance_id must be a non-empty string', size)
    return HaHeartbeat(ts=ts, seq=seq, interval_ms=interval_ms, instance_id=instance_id)


# --------------------------------------------------------------------------- params subset


def _json_equal(left: object, right: object) -> bool:
    """Equality as JSON sees it: booleans never equal numbers, 1 equals 1.0."""
    if isinstance(left, bool) or isinstance(right, bool):
        return isinstance(left, bool) and isinstance(right, bool) and left is right
    return left == right


def _is_primitive(value: object) -> bool:
    return isinstance(value, (str, bool, int, float))


def validate_params_schema(schema: object) -> list[str]:
    """Check that ``schema`` is a well-formed params subset; return the problems found.

    The subset: ``type`` is ``object``; ``properties`` maps names to specs whose
    ``type`` is one of string, number, integer, boolean and which may carry
    ``enum`` (a non-empty list of primitives), numeric ``minimum`` and
    ``maximum``, and a string ``description``; ``required`` is a list of
    distinct strings. Other keys are ignored, as the contract says.
    """
    if not isinstance(schema, Mapping):
        return ['params schema must be an object']
    errors: list[str] = []
    if schema.get('type') != 'object':
        errors.append("params schema 'type' must be 'object'")
    properties = schema.get('properties')
    if not isinstance(properties, Mapping):
        errors.append("params schema 'properties' must be an object")
        properties = {}
    for name, spec in properties.items():
        if not isinstance(name, str):
            errors.append(f'property name {name!r} must be a string')
            continue
        if not isinstance(spec, Mapping):
            errors.append(f"property '{name}' must be an object")
            continue
        if spec.get('type') not in PARAM_TYPES:
            errors.append(f"property '{name}' type must be one of {', '.join(PARAM_TYPES)}")
        if 'enum' in spec:
            enum = spec['enum']
            if not isinstance(enum, list) or not enum or not all(_is_primitive(v) for v in enum):
                errors.append(f"property '{name}' enum must be a non-empty list of primitives")
        for bound in ('minimum', 'maximum'):
            if bound in spec and not _is_number(spec[bound]):
                errors.append(f"property '{name}' {bound} must be a number")
        if 'description' in spec and not isinstance(spec['description'], str):
            errors.append(f"property '{name}' description must be a string")
    if 'required' in schema:
        required = schema['required']
        if not isinstance(required, list) or not all(isinstance(n, str) for n in required):
            errors.append("params schema 'required' must be a list of strings")
        elif len(set(required)) != len(required):
            errors.append("params schema 'required' must not repeat a name")
    return errors


def _check_param(name: str, spec: Mapping[str, Any], value: object) -> list[str]:
    expected = spec.get('type')
    type_ok: bool
    if expected == 'string':
        type_ok = isinstance(value, str)
    elif expected == 'boolean':
        type_ok = isinstance(value, bool)
    elif expected == 'integer':
        type_ok = _is_int(value) or (
            isinstance(value, float) and math.isfinite(value) and value.is_integer()
        )
    elif expected == 'number':
        type_ok = _is_number(value)
    else:
        return [f"'{name}' has an unsupported type in the announced schema"]
    if not type_ok:
        return [f"'{name}' must be a {expected}"]
    errors: list[str] = []
    enum = spec.get('enum')
    if isinstance(enum, list) and not any(_json_equal(value, item) for item in enum):
        errors.append(f"'{name}' must be one of {enum!r}")
    if _is_number(value):
        assert isinstance(value, (int, float))
        minimum = spec.get('minimum')
        if _is_number(minimum):
            assert isinstance(minimum, (int, float))
            if value < minimum:
                errors.append(f"'{name}' must be at least {minimum}")
        maximum = spec.get('maximum')
        if _is_number(maximum):
            assert isinstance(maximum, (int, float))
            if value > maximum:
                errors.append(f"'{name}' must be at most {maximum}")
    return errors


def validate_params(schema: Mapping[str, Any] | None, params: object) -> list[str]:
    """Validate a command's ``params`` against a task's announced params subset.

    Returns the list of problems (empty means valid). ``params`` must be an
    object. With ``schema`` ``None`` (a task that takes no parameters) any
    object passes. Otherwise every name in ``required`` must be present and
    every present parameter that the schema declares must match its ``type``,
    ``enum``, ``minimum`` and ``maximum``; parameters the schema does not
    declare are not interpreted, which is JSON Schema's own meaning of
    ``properties`` without ``additionalProperties``. Written by hand on purpose,
    like the HA side, so both halves agree without a JSON Schema library.
    """
    if not isinstance(params, Mapping):
        return ['params must be an object']
    if schema is None:
        return []
    errors: list[str] = []
    properties = schema.get('properties')
    if not isinstance(properties, Mapping):
        properties = {}
    required = schema.get('required')
    if isinstance(required, list):
        errors.extend(f"'{name}' is required" for name in required if name not in params)
    for name, value in params.items():
        spec = properties.get(name)
        if isinstance(spec, Mapping) and isinstance(name, str):
            errors.extend(_check_param(name, spec, value))
    return errors


def validate_task_spec(spec: TaskSpec) -> list[str]:
    """Check an executor's :class:`TaskSpec` against the announce schema's ``task_spec``."""
    errors: list[str] = []
    if not is_valid_task_name(spec.name):
        errors.append(f'task name {spec.name!r} must match ^[a-z][a-z0-9_]{{0,31}}$')
    if not isinstance(spec.label, str) or not spec.label:
        errors.append(f'task {spec.name!r} label must be a non-empty string')
    if spec.kind not in TASK_KINDS:
        errors.append(f'task {spec.name!r} kind must be one of {", ".join(TASK_KINDS)}')
    if not _is_number(spec.timeout_s) or spec.timeout_s <= 0:
        errors.append(f'task {spec.name!r} timeout_s must be a number greater than 0')
    if spec.params is not None:
        errors.extend(f'task {spec.name!r}: {e}' for e in validate_params_schema(spec.params))
    return errors


def validate_joint_info(info: JointInfo) -> list[str]:
    """Check an executor's :class:`JointInfo` against the announce schema's ``joints`` block.

    The schema cannot compare the length of ``limits`` with the length of
    ``names``, so that rule lives here (and in Home Assistant), with the two
    the schema does express: 1 to 32 names matching the joint-name pattern,
    and each limit a pair of numbers. Names must also be distinct, and each
    pair's minimum must not exceed its maximum, so a gauge built from them is
    never inside out.
    """
    errors: list[str] = []
    # Plugin input: checked as plain objects, whatever the annotations promise.
    names: object = info.names
    limits: object = info.limits
    if isinstance(names, str) or not isinstance(names, Sequence):
        return ['joints names must be a list of strings']
    if not 1 <= len(names) <= MAX_JOINTS:
        errors.append(f'joints must name between 1 and {MAX_JOINTS} joints, not {len(names)}')
    for name in names:
        if not is_valid_joint_name(name):
            errors.append(f'joint name {name!r} must match ^[a-z][a-z0-9_]{{0,31}}$')
    if len(set(names)) != len(names):
        errors.append('joint names must be distinct')
    if info.unit not in JOINT_UNITS:
        errors.append(f'joints unit must be one of {", ".join(JOINT_UNITS)}, not {info.unit!r}')
    if limits is None:
        return errors
    if isinstance(limits, str) or not isinstance(limits, Sequence):
        errors.append('joints limits must be null or a list of [minimum, maximum] pairs')
        return errors
    if len(limits) != len(names):
        errors.append(
            f'joints limits has {len(limits)} pair(s) for {len(names)} name(s); '
            'give one pair per name or null'
        )
    for index, pair in enumerate(limits):
        if (
            isinstance(pair, str)
            or not isinstance(pair, Sequence)
            or len(pair) != 2
            or not all(_is_number(bound) for bound in pair)
        ):
            errors.append(f'joints limits[{index}] must be a pair of finite numbers')
            continue
        low, high = pair
        if low > high:
            errors.append(f'joints limits[{index}] minimum {low!r} is above maximum {high!r}')
    return errors


# --------------------------------------------------------------------------- freshness

FreshnessVerdict = Literal['fresh', 'stale', 'future']


@dataclass(frozen=True)
class Freshness:
    """Outcome of the freshness rule for one command."""

    verdict: FreshnessVerdict
    #: ``now - issued_at`` in milliseconds; goes into the ack as ``clock_skew_ms``.
    clock_skew_ms: int
    #: The acceptance window actually applied: ``min(max_age_ms, max_age_cap_ms)``.
    window_ms: int

    @property
    def ack_code(self) -> AckCode | None:
        """The rejection code for a stale or future command, ``None`` when fresh."""
        if self.verdict == 'stale':
            return 'rejected_stale'
        if self.verdict == 'future':
            return 'rejected_future'
        return None


def check_freshness(
    now_ms: int,
    issued_at: int,
    max_age_ms: int,
    *,
    max_age_cap_ms: int = DEFAULT_MAX_AGE_CAP_MS,
    skew_ms: int = DEFAULT_CLOCK_SKEW_TOLERANCE_MS,
) -> Freshness:
    """Apply ``-skew <= now - issued_at <= min(max_age_ms, cap) + skew``.

    Too old is ``stale``, issued ahead of this clock by more than the tolerance
    is ``future``. The persisted dedupe store is consulted before this rule so
    a redelivered command republishes its stored ack instead of being
    downgraded to ``rejected_stale``.
    """
    age = now_ms - issued_at
    window = min(max_age_ms, max_age_cap_ms)
    if age < -skew_ms:
        return Freshness('future', age, window)
    if age > window + skew_ms:
        return Freshness('stale', age, window)
    return Freshness('fresh', age, window)
