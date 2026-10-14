# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Constants for the ROS 2 Bridge integration.

Everything here is a plain value: option keys and defaults, timing constants,
the contract's identifier patterns, topic builders and subscription filters,
dispatcher signal and bus event names, the closed sets of the contract as
enums, and the repair issue ids. The module imports nothing from Home
Assistant on purpose, so that ``protocol.py``, ``models.py`` and
``commands.py`` can be unit-tested without a running ``hass``.
"""

from __future__ import annotations

import re
from enum import StrEnum
from typing import Any, Final

DOMAIN: Final = "ros2_bridge"
VERSION: Final = "0.1.0"
#: Every JSON payload carries ``"schema": 1``; schema 1 only ever gains optional fields.
SCHEMA_VERSION: Final = 1

# --------------------------------------------------------------------------- options

#: Config entry ``data``: the UUID4 this entry stamps into ``source.instance_id`` and heartbeats.
CONF_INSTANCE_ID: Final = "instance_id"

#: Config entry ``options`` keys (also the fields of the options flow).
CONF_TOPIC_PREFIX: Final = "topic_prefix"
CONF_SHARED_SECRET: Final = "shared_secret"
CONF_HEARTBEAT_INTERVAL_S: Final = "heartbeat_interval_s"
CONF_ACK_TIMEOUT_S: Final = "ack_timeout_s"
CONF_COMMAND_MAX_AGE_MS: Final = "command_max_age_ms"
CONF_RESULT_TIMEOUT_MARGIN_S: Final = "result_timeout_margin_s"
CONF_MAX_PENDING_PER_ROBOT: Final = "max_pending_per_robot"
CONF_HEARTBEAT_UNAVAILABLE_AFTER_S: Final = "heartbeat_unavailable_after_s"
CONF_MIRROR_ENTITIES: Final = "mirror_entities"

DEFAULT_TOPIC_PREFIX: Final = "ros2_bridge"
DEFAULT_SHARED_SECRET: Final = ""
DEFAULT_HEARTBEAT_INTERVAL_S: Final = 2
DEFAULT_ACK_TIMEOUT_S: Final = 2.0
DEFAULT_COMMAND_MAX_AGE_MS: Final = 5000
DEFAULT_RESULT_TIMEOUT_MARGIN_S: Final = 10
DEFAULT_MAX_PENDING_PER_ROBOT: Final = 16
DEFAULT_HEARTBEAT_UNAVAILABLE_AFTER_S: Final = 30
DEFAULT_MIRROR_ENTITIES: Final[tuple[str, ...]] = ()

#: Every option with its default, for the options flow and for ``HubOptions.from_mapping``.
DEFAULT_OPTIONS: Final[dict[str, Any]] = {
    CONF_TOPIC_PREFIX: DEFAULT_TOPIC_PREFIX,
    CONF_SHARED_SECRET: DEFAULT_SHARED_SECRET,
    CONF_HEARTBEAT_INTERVAL_S: DEFAULT_HEARTBEAT_INTERVAL_S,
    CONF_ACK_TIMEOUT_S: DEFAULT_ACK_TIMEOUT_S,
    CONF_COMMAND_MAX_AGE_MS: DEFAULT_COMMAND_MAX_AGE_MS,
    CONF_RESULT_TIMEOUT_MARGIN_S: DEFAULT_RESULT_TIMEOUT_MARGIN_S,
    CONF_MAX_PENDING_PER_ROBOT: DEFAULT_MAX_PENDING_PER_ROBOT,
    CONF_HEARTBEAT_UNAVAILABLE_AFTER_S: DEFAULT_HEARTBEAT_UNAVAILABLE_AFTER_S,
    CONF_MIRROR_ENTITIES: list(DEFAULT_MIRROR_ENTITIES),
}

# --------------------------------------------------------------------------- timing

#: Quiet period after ``hub.ready`` and ``mqtt_connected`` before the first reconciliation.
REPLAY_SETTLE_S: Final = 3
#: Grace after a robot's ``session`` changes before its outstanding commands are reconciled.
SESSION_CHANGE_GRACE_S: Final = 1
#: Upper bound on waiting for the ack and result subscriptions to complete.
READY_TIMEOUT_S: Final = 10
#: Period of the hub's housekeeping timer (deadlines, staleness, grace, pruning).
HOUSEKEEPING_INTERVAL_S: Final = 1
#: Feedback writes to the state sensor are limited to one per robot per this many ms.
FEEDBACK_THROTTLE_MS: Final = 500
#: A robot heartbeat is stale after this many announced intervals without one.
HEARTBEAT_STALE_INTERVALS: Final = 3
#: Message Expiry Interval of the HA heartbeat (seconds).
HA_HEARTBEAT_EXPIRY_S: Final = 5
#: Node defaults for ``reply.ack_expiry_s`` and ``reply.result_expiry_s`` (seconds).
DEFAULT_ACK_EXPIRY_S: Final = 120
DEFAULT_RESULT_EXPIRY_S: Final = 300
#: Size limits on the bytes actually received (16 KiB per message, 64 KiB for announce).
MAX_PAYLOAD_BYTES: Final = 16 * 1024
MAX_ANNOUNCE_BYTES: Final = 64 * 1024
#: ``duplicate_robot_id`` heuristics: more than this many session changes in the window.
DUPLICATE_SESSION_CHANGES: Final = 3
DUPLICATE_SESSION_WINDOW_S: Final = 300
#: ``duplicate_robot_id`` closes after this many quiet seconds.
DUPLICATE_QUIET_S: Final = 600
#: An ack whose ``clock_skew_ms`` magnitude reaches this opens the ``clock_skew`` repair.
CLOCK_SKEW_WARN_MS: Final = 1000
#: How many registry entries the diagnostics snapshot lists.
DIAGNOSTICS_RECENT_COMMANDS: Final = 50
#: Delay of the debounced Store write used for housekeeping-only changes (seconds).
STORE_DELAY_SAVE_S: Final = 5

# --------------------------------------------------------------------------- storage

STORAGE_KEY: Final = f"{DOMAIN}.commands"
STORAGE_VERSION: Final = 1

#: ``hass.data[DOMAIN][DATA_HUB]``: the loaded ``BridgeHub`` while the config entry is loaded.
DATA_HUB: Final = "hub"

# --------------------------------------------------------------------------- identifiers

#: Robot-id segment reserved for Home Assistant's own topics (``{prefix}/ha/...``).
RESERVED_ROBOT_ID: Final = "ha"
#: Home Assistant's default MQTT discovery prefix; never usable as the bridge prefix.
DISCOVERY_PREFIX: Final = "homeassistant"
#: ``source.origin`` of every command this integration publishes.
SOURCE_ORIGIN: Final = "ha"
#: ``source.kind`` values this integration uses (an open vocabulary on the wire).
SOURCE_KIND_BUTTON: Final = "button"
SOURCE_KIND_SERVICE: Final = "service"
SOURCE_KIND_INTENT: Final = "intent"

#: The availability payload is the plain word, not JSON.
AVAILABILITY_ONLINE: Final = "online"
AVAILABILITY_OFFLINE: Final = "offline"

# The patterns are applied with ``fullmatch`` so a trailing newline, which ``re.match``
# with ``$`` would accept, is refused exactly as the JSON Schema refuses it.
#: ``common.schema.json#/$defs/robot_id`` (plus the reserved ``ha`` rule, see protocol.py).
ROBOT_ID_RE: Final = re.compile(r"[a-z][a-z0-9_]{0,31}")
#: ``common.schema.json#/$defs/task_name``: same shape as a robot id, nothing reserved.
TASK_NAME_RE: Final = re.compile(r"[a-z][a-z0-9_]{0,31}")
#: ``common.schema.json#/$defs/task_id``: a lower-case UUID4 string.
TASK_ID_RE: Final = re.compile(
    r"[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}"
)
#: ``common.schema.json#/$defs/session``: same shape as a task id.
SESSION_RE: Final = TASK_ID_RE
#: ``common.schema.json#/$defs/result_code``.
RESULT_CODE_RE: Final = re.compile(r"[a-z][a-z0-9_]{0,63}")
#: ``common.schema.json#/$defs/instance``: ``<hostname>:<pid>``.
INSTANCE_RE: Final = re.compile(r"[^:]+:[0-9]+")
#: ``common.schema.json#/$defs/sig``: 64 lower-case hex characters.
SIG_RE: Final = re.compile(r"[0-9a-f]{64}")
#: Segments of ``{prefix}/ha/sensors/{domain}/{object_id}``.
SENSOR_SEGMENT_RE: Final = re.compile(r"[a-z0-9_]+")

# --------------------------------------------------------------------------- topics

# Builders are plain string formatting. Callers validate the prefix and the ids first
# (the hub always does); ``protocol.validate_publish_topic`` is the last guard.


def robot_topic(prefix: str, robot_id: str, suffix: str) -> str:
    """``{prefix}/{robot_id}/{suffix}``."""
    return f"{prefix}/{robot_id}/{suffix}"


def announce_topic(prefix: str, robot_id: str) -> str:
    """``{prefix}/{robot_id}/announce`` (ROS to HA, QoS 1, retained, no expiry)."""
    return robot_topic(prefix, robot_id, "announce")


def availability_topic(prefix: str, robot_id: str) -> str:
    """``{prefix}/{robot_id}/availability`` (plain ``online``/``offline``, retained)."""
    return robot_topic(prefix, robot_id, "availability")


def state_topic(prefix: str, robot_id: str) -> str:
    """``{prefix}/{robot_id}/state`` (ROS to HA, QoS 1, retained, no expiry)."""
    return robot_topic(prefix, robot_id, "state")


def heartbeat_topic(prefix: str, robot_id: str) -> str:
    """``{prefix}/{robot_id}/heartbeat`` (ROS to HA, QoS 0, not retained, expiry 5 s)."""
    return robot_topic(prefix, robot_id, "heartbeat")


def cmd_topic(prefix: str, robot_id: str) -> str:
    """``{prefix}/{robot_id}/cmd`` (HA to ROS, QoS 1, never retained)."""
    return robot_topic(prefix, robot_id, "cmd")


def ack_topic(prefix: str, robot_id: str, task_id: str) -> str:
    """``{prefix}/{robot_id}/task/{task_id}/ack`` (QoS 1, retained, expiry 120 s)."""
    return robot_topic(prefix, robot_id, f"task/{task_id}/ack")


def feedback_topic(prefix: str, robot_id: str, task_id: str) -> str:
    """``{prefix}/{robot_id}/task/{task_id}/feedback`` (QoS 0, not retained, expiry 10 s)."""
    return robot_topic(prefix, robot_id, f"task/{task_id}/feedback")


def result_topic(prefix: str, robot_id: str, task_id: str) -> str:
    """``{prefix}/{robot_id}/task/{task_id}/result`` (QoS 1, retained, expiry 300 s)."""
    return robot_topic(prefix, robot_id, f"task/{task_id}/result")


def ha_heartbeat_topic(prefix: str) -> str:
    """``{prefix}/ha/heartbeat`` (HA to ROS, QoS 0, not retained, expiry 5 s)."""
    return f"{prefix}/{RESERVED_ROBOT_ID}/heartbeat"


def ha_sensor_topic(prefix: str, domain: str, object_id: str) -> str:
    """``{prefix}/ha/sensors/{domain}/{object_id}`` (HA to ROS, QoS 0, retained)."""
    return f"{prefix}/{RESERVED_ROBOT_ID}/sensors/{domain}/{object_id}"


def announce_filter(prefix: str) -> str:
    """Subscription filter ``{prefix}/+/announce``."""
    return f"{prefix}/+/announce"


def availability_filter(prefix: str) -> str:
    """Subscription filter ``{prefix}/+/availability``."""
    return f"{prefix}/+/availability"


def state_filter(prefix: str) -> str:
    """Subscription filter ``{prefix}/+/state``."""
    return f"{prefix}/+/state"


def heartbeat_filter(prefix: str) -> str:
    """Subscription filter ``{prefix}/+/heartbeat``."""
    return f"{prefix}/+/heartbeat"


def ack_filter(prefix: str) -> str:
    """Subscription filter ``{prefix}/+/task/+/ack``."""
    return f"{prefix}/+/task/+/ack"


def feedback_filter(prefix: str) -> str:
    """Subscription filter ``{prefix}/+/task/+/feedback``."""
    return f"{prefix}/+/task/+/feedback"


def result_filter(prefix: str) -> str:
    """Subscription filter ``{prefix}/+/task/+/result``."""
    return f"{prefix}/+/task/+/result"


# --------------------------------------------------------------------------- signals and events

#: Dispatcher signals. All carry the ``robot_id`` as their first argument except
#: ``SIGNAL_CONNECTION_CHANGED``, which carries nothing (hub-wide availability change).
#: ``SIGNAL_TASK_EVENT`` carries ``(robot_id, event_name, data)`` with the same ``data`` as the
#: bus event of that name, so the event entity can trigger without reading the bus.
SIGNAL_ROBOT_ADDED: Final = f"{DOMAIN}_robot_added"
SIGNAL_ROBOT_UPDATED: Final = f"{DOMAIN}_robot_updated"
SIGNAL_ROBOT_REMOVED: Final = f"{DOMAIN}_robot_removed"
#: ``(robot_id, added_task_names, removed_task_names)`` when a re-announce changes the catalogue.
SIGNAL_ROBOT_TASKS_CHANGED: Final = f"{DOMAIN}_robot_tasks_changed"
SIGNAL_CONNECTION_CHANGED: Final = f"{DOMAIN}_connection_changed"
SIGNAL_TASK_EVENT: Final = f"{DOMAIN}_task_event"

#: Bus event names.
EVENT_TASK_ACK: Final = f"{DOMAIN}_task_ack"
EVENT_TASK_RESULT: Final = f"{DOMAIN}_task_result"
EVENT_TASK_LATE_RESULT: Final = f"{DOMAIN}_task_late_result"

# --------------------------------------------------------------------------- closed sets


class CommandType(StrEnum):
    """``cmd.type``."""

    TASK = "task"
    CANCEL = "cancel"


class TaskKind(StrEnum):
    """``task_spec.kind`` and ``result.kind``."""

    MOTION = "motion"
    CONTROL = "control"
    QUERY = "query"


class RobotStatus(StrEnum):
    """``state.state``: exactly idle, busy or error (Q17)."""

    IDLE = "idle"
    BUSY = "busy"
    ERROR = "error"


class HaLink(StrEnum):
    """``state.ha_link``."""

    UP = "up"
    DOWN = "down"


class WatchdogMode(StrEnum):
    """``announce.watchdog_mode`` and ``state.watchdog_mode``."""

    NONE = "none"
    INHIBIT_NEW = "inhibit_new"
    STOP_AND_HOLD = "stop_and_hold"


class HealthLevel(StrEnum):
    """``state.executor_health.level``."""

    OK = "ok"
    DEGRADED = "degraded"
    ERROR = "error"


class Availability(StrEnum):
    """The availability topic's two words."""

    ONLINE = AVAILABILITY_ONLINE
    OFFLINE = AVAILABILITY_OFFLINE


class ResultStatus(StrEnum):
    """``result.status`` on the wire, mapped onto ``action_msgs/GoalStatus``."""

    SUCCEEDED = "succeeded"
    CANCELLED = "cancelled"
    ABORTED = "aborted"


#: ``action_msgs/GoalStatus`` values for each wire status (Jazzy: 4, 5, 6).
GOAL_STATUS: Final[dict[ResultStatus, int]] = {
    ResultStatus.SUCCEEDED: 4,
    ResultStatus.CANCELLED: 5,
    ResultStatus.ABORTED: 6,
}


class TaskOutcome(StrEnum):
    """The outcome vocabulary of the HA side: event types and ``last_result`` options.

    The first three come from the wire; ``rejected``, ``timeout`` and ``lost`` are
    decided by HA from acks and its own deadlines.
    """

    SUCCEEDED = "succeeded"
    CANCELLED = "cancelled"
    ABORTED = "aborted"
    REJECTED = "rejected"
    TIMEOUT = "timeout"
    LOST = "lost"


class CommandState(StrEnum):
    """States of the command state machine in ``commands.py``.

    ``created -> sent -> (running | rejected | timeout_no_ack) -> (succeeded | cancelled |
    aborted | timeout_no_result | lost)``, plus ``timeout_no_ack -> running`` on a late
    accepted ack and ``sent -> terminal`` on a result. A cancel ends in ``done``,
    ``rejected`` or ``timeout_no_ack``.
    """

    CREATED = "created"
    SENT = "sent"
    RUNNING = "running"
    REJECTED = "rejected"
    TIMEOUT_NO_ACK = "timeout_no_ack"
    SUCCEEDED = "succeeded"
    CANCELLED = "cancelled"
    ABORTED = "aborted"
    TIMEOUT_NO_RESULT = "timeout_no_result"
    LOST = "lost"
    DONE = "done"


#: States in which a command still waits for the robot and is persisted as outstanding.
#: Every other state is terminal. ``timeout_no_ack`` is terminal but stays in memory so a
#: late accepted ack (retained up to 120 s) can re-open it as ``running``; its outcome
#: event has already fired, so the re-opened command never fires ``task_result`` again.
OUTSTANDING_STATES: Final[frozenset[CommandState]] = frozenset(
    {CommandState.CREATED, CommandState.SENT, CommandState.RUNNING}
)
#: How many terminal commands the registry keeps in memory for late acks and diagnostics.
MAX_TERMINAL_COMMANDS: Final = 500

#: Closed set of ack codes; exactly one means the command was taken on.
ACK_CODE_ACCEPTED: Final = "accepted"
ACK_CODES: Final[tuple[str, ...]] = (
    ACK_CODE_ACCEPTED,
    "rejected_malformed",
    "rejected_unknown_task",
    "rejected_invalid_params",
    "rejected_stale",
    "rejected_future",
    "rejected_unauthorised",
    "rejected_busy",
    "rejected_error",
    "rejected_inhibited",
    "rejected_executor",
    "rejected_no_such_task",
)
ACK_CODE_STALE: Final = "rejected_stale"
ACK_CODE_FUTURE: Final = "rejected_future"
ACK_CODE_UNAUTHORISED: Final = "rejected_unauthorised"

#: HA-synthesised result codes; they never travel on the wire.
CODE_NO_ACK: Final = "no_ack"
CODE_NO_RESULT: Final = "no_result"
CODE_NODE_RESTARTED: Final = "node_restarted"
CODE_HA_RESTARTED: Final = "ha_restarted"

#: Parameter types a task may declare in its params subset schema.
PARAM_TYPES: Final[tuple[str, ...]] = ("string", "number", "integer", "boolean")

#: Required counter names of the robot heartbeat, in schema order.
COUNTER_NAMES: Final[tuple[str, ...]] = (
    "accepted",
    "rejected",
    "duplicates",
    "dropped_retained",
    "dropped_oversize",
    "dropped_invalid_id",
    "dropped_queue_full",
    "publish_errors",
    "takeovers",
)

# --------------------------------------------------------------------------- repairs

#: Repair issue kinds; each is also its translation key under ``issues`` in strings.json.
ISSUE_CLOCK_SKEW: Final = "clock_skew"
ISSUE_HMAC_REJECTED: Final = "hmac_rejected"
ISSUE_INVALID_ANNOUNCE: Final = "invalid_announce"
ISSUE_MQTT_V5_REQUIRED: Final = "mqtt_v5_required"
ISSUE_DUPLICATE_ROBOT_ID: Final = "duplicate_robot_id"
ISSUE_KINDS: Final[tuple[str, ...]] = (
    ISSUE_CLOCK_SKEW,
    ISSUE_HMAC_REJECTED,
    ISSUE_INVALID_ANNOUNCE,
    ISSUE_MQTT_V5_REQUIRED,
    ISSUE_DUPLICATE_ROBOT_ID,
)


def issue_id(kind: str, robot_id: str | None = None) -> str:
    """The issue registry id for a repair: the kind alone, or ``{kind}_{robot_id}``.

    ``mqtt_v5_required`` concerns the broker and is raised once; the other four are
    raised per robot so one robot clearing never closes another robot's issue. The
    translation key is always the bare kind.
    """
    return kind if robot_id is None else f"{kind}_{robot_id}"
