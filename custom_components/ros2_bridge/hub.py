# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The bridge hub: MQTT subscriptions, the robot table and the command path.

``BridgeHub`` is a push hub without a coordinator. ``async_start`` opens the
eight wildcard subscriptions on Home Assistant's own MQTT connection with
synchronous handlers, watches the ack and result subscriptions for ``ready``
(bounded by ``READY_TIMEOUT_S``), follows the MQTT connection status, runs the
HA heartbeat and the 1 s housekeeping timer, and after a ``REPLAY_SETTLE_S``
quiet period reconciles the outstanding commands of the ``CommandRegistry``
against what the retained replay said about each robot.

Every handler first checks the topic's robot segment (``ha`` is discarded
silently, anything failing the pattern with a count), then decodes the payload
with ``protocol.py``; a bad payload is counted and logged, never raised into
the subscription callback. Commands leave through ``async_send_task`` and
``async_cancel_task`` (validation, registry entry, Store write, publish with
QoS 1 and expiry) and their replies come back by id through the registry,
which fires the bus events the hub decorates with the device id and mirrors
on the dispatcher. The sensor mirror and any other add-on registers through
``register_component`` and is started, stopped and told about connection
changes alongside the hub.
"""

from __future__ import annotations

import logging
import math
import uuid
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field, replace
from datetime import datetime, timedelta
from functools import partial
from typing import Any, Protocol

from homeassistant.components import mqtt
from homeassistant.components.mqtt.models import ReceiveMessage
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import CALLBACK_TYPE, HomeAssistant, callback
from homeassistant.exceptions import HomeAssistantError, ServiceValidationError
from homeassistant.helpers import device_registry as dr
from homeassistant.helpers import issue_registry as ir
from homeassistant.helpers.device_registry import DeviceInfo
from homeassistant.helpers.dispatcher import async_dispatcher_send
from homeassistant.helpers.event import async_call_later, async_track_time_interval
from homeassistant.util import dt as dt_util

from .commands import (
    CommandRegistry,
    RegistrySettings,
    ResultDisposition,
    RobotSnapshot,
    TooManyPendingError,
)
from .const import (
    ACK_CODE_FUTURE,
    ACK_CODE_STALE,
    ACK_CODE_UNAUTHORISED,
    CLOCK_SKEW_WARN_MS,
    CONF_ACK_TIMEOUT_S,
    CONF_COMMAND_MAX_AGE_MS,
    CONF_HEARTBEAT_INTERVAL_S,
    CONF_HEARTBEAT_UNAVAILABLE_AFTER_S,
    CONF_INSTANCE_ID,
    CONF_MAX_PENDING_PER_ROBOT,
    CONF_MIRROR_ENTITIES,
    CONF_RESULT_TIMEOUT_MARGIN_S,
    CONF_SHARED_SECRET,
    CONF_TOPIC_PREFIX,
    DEFAULT_ACK_TIMEOUT_S,
    DEFAULT_COMMAND_MAX_AGE_MS,
    DEFAULT_HEARTBEAT_INTERVAL_S,
    DEFAULT_HEARTBEAT_UNAVAILABLE_AFTER_S,
    DEFAULT_MAX_PENDING_PER_ROBOT,
    DEFAULT_RESULT_TIMEOUT_MARGIN_S,
    DEFAULT_SHARED_SECRET,
    DEFAULT_TOPIC_PREFIX,
    DIAGNOSTICS_RECENT_COMMANDS,
    DOMAIN,
    DUPLICATE_QUIET_S,
    DUPLICATE_SESSION_CHANGES,
    DUPLICATE_SESSION_WINDOW_S,
    EVENT_TASK_RESULT,
    FEEDBACK_THROTTLE_MS,
    HA_HEARTBEAT_EXPIRY_S,
    HEARTBEAT_STALE_INTERVALS,
    HOUSEKEEPING_INTERVAL_S,
    ISSUE_CLOCK_SKEW,
    ISSUE_DUPLICATE_ROBOT_ID,
    ISSUE_HMAC_REJECTED,
    ISSUE_INVALID_ANNOUNCE,
    ISSUE_MQTT_V5_REQUIRED,
    READY_TIMEOUT_S,
    REPLAY_SETTLE_S,
    RESERVED_ROBOT_ID,
    SCHEMA_VERSION,
    SESSION_CHANGE_GRACE_S,
    SIGNAL_CONNECTION_CHANGED,
    SIGNAL_ROBOT_ADDED,
    SIGNAL_ROBOT_JOINTS_CHANGED,
    SIGNAL_ROBOT_JOINTS_UPDATED,
    SIGNAL_ROBOT_REMOVED,
    SIGNAL_ROBOT_TASKS_CHANGED,
    SIGNAL_ROBOT_UPDATED,
    SIGNAL_TASK_EVENT,
    Availability,
    CommandType,
    HealthLevel,
    RobotStatus,
    TaskKind,
    TaskOutcome,
    ack_filter,
    announce_filter,
    announce_topic,
    availability_filter,
    availability_topic,
    cmd_topic,
    feedback_filter,
    ha_heartbeat_topic,
    heartbeat_filter,
    issue_id,
    joints_filter,
    result_filter,
    state_filter,
    state_topic,
)
from .models import Announce, JointInfo, LastResult, PendingCommand, Robot, TaskSpec
from .protocol import (
    CommandSource,
    ProtocolError,
    cancel_command_payload,
    decode_ack,
    decode_announce,
    decode_availability,
    decode_feedback,
    decode_heartbeat,
    decode_joints,
    decode_result,
    decode_state,
    encode_command,
    encode_payload,
    is_valid_robot_id,
    is_valid_task_id,
    split_robot_topic,
    task_command_payload,
    validate_params,
)

_LOGGER = logging.getLogger(__name__)

type Ros2BridgeConfigEntry = ConfigEntry[BridgeHub]

#: Returns the current time as epoch milliseconds.
Clock = Callable[[], int]


def default_clock() -> int:
    """Epoch milliseconds from Home Assistant's clock (so test freezers control it)."""
    return int(dt_util.utcnow().timestamp() * 1000)


@dataclass(frozen=True, slots=True)
class HubOptions:
    """The config entry's options as typed values with the defaults applied.

    ``shared_secret`` is kept out of ``repr`` so that a logged or dumped options
    object can never leak it, matching the ROS side's settings classes.
    """

    topic_prefix: str = DEFAULT_TOPIC_PREFIX
    shared_secret: str = field(default=DEFAULT_SHARED_SECRET, repr=False)
    heartbeat_interval_s: float = DEFAULT_HEARTBEAT_INTERVAL_S
    ack_timeout_s: float = DEFAULT_ACK_TIMEOUT_S
    command_max_age_ms: int = DEFAULT_COMMAND_MAX_AGE_MS
    result_timeout_margin_s: float = DEFAULT_RESULT_TIMEOUT_MARGIN_S
    max_pending_per_robot: int = DEFAULT_MAX_PENDING_PER_ROBOT
    heartbeat_unavailable_after_s: float = DEFAULT_HEARTBEAT_UNAVAILABLE_AFTER_S
    mirror_entities: tuple[str, ...] = ()

    @classmethod
    def from_mapping(cls, options: Mapping[str, Any]) -> HubOptions:
        """Build from ``entry.options``; absent keys take the defaults."""
        mirror = options.get(CONF_MIRROR_ENTITIES) or ()
        return cls(
            topic_prefix=str(options.get(CONF_TOPIC_PREFIX, DEFAULT_TOPIC_PREFIX)),
            shared_secret=str(options.get(CONF_SHARED_SECRET) or DEFAULT_SHARED_SECRET),
            heartbeat_interval_s=float(
                options.get(CONF_HEARTBEAT_INTERVAL_S, DEFAULT_HEARTBEAT_INTERVAL_S)
            ),
            ack_timeout_s=float(options.get(CONF_ACK_TIMEOUT_S, DEFAULT_ACK_TIMEOUT_S)),
            command_max_age_ms=int(
                options.get(CONF_COMMAND_MAX_AGE_MS, DEFAULT_COMMAND_MAX_AGE_MS)
            ),
            result_timeout_margin_s=float(
                options.get(CONF_RESULT_TIMEOUT_MARGIN_S, DEFAULT_RESULT_TIMEOUT_MARGIN_S)
            ),
            max_pending_per_robot=int(
                options.get(CONF_MAX_PENDING_PER_ROBOT, DEFAULT_MAX_PENDING_PER_ROBOT)
            ),
            heartbeat_unavailable_after_s=float(
                options.get(
                    CONF_HEARTBEAT_UNAVAILABLE_AFTER_S, DEFAULT_HEARTBEAT_UNAVAILABLE_AFTER_S
                )
            ),
            mirror_entities=tuple(str(entity_id) for entity_id in mirror),
        )

    @classmethod
    def from_entry(cls, entry: ConfigEntry) -> HubOptions:
        return cls.from_mapping(entry.options)

    @property
    def secret(self) -> str | None:
        """The shared secret, or ``None`` when signing is off."""
        return self.shared_secret or None

    def registry_settings(self) -> RegistrySettings:
        return RegistrySettings(
            ack_timeout_s=self.ack_timeout_s,
            result_timeout_margin_s=self.result_timeout_margin_s,
            max_pending_per_robot=self.max_pending_per_robot,
        )

    def to_dict(self, *, redact_secret: bool = True) -> dict[str, Any]:
        data: dict[str, Any] = {
            CONF_TOPIC_PREFIX: self.topic_prefix,
            CONF_HEARTBEAT_INTERVAL_S: self.heartbeat_interval_s,
            CONF_ACK_TIMEOUT_S: self.ack_timeout_s,
            CONF_COMMAND_MAX_AGE_MS: self.command_max_age_ms,
            CONF_RESULT_TIMEOUT_MARGIN_S: self.result_timeout_margin_s,
            CONF_MAX_PENDING_PER_ROBOT: self.max_pending_per_robot,
            CONF_HEARTBEAT_UNAVAILABLE_AFTER_S: self.heartbeat_unavailable_after_s,
            CONF_MIRROR_ENTITIES: list(self.mirror_entities),
        }
        if redact_secret:
            data["secret_configured"] = self.secret is not None
        else:
            data[CONF_SHARED_SECRET] = self.shared_secret
        return data


class HubComponent(Protocol):
    """Something the hub starts and stops with itself, such as the sensor mirror.

    ``async_connection_changed`` is called on every MQTT connection change after
    the hub has updated ``mqtt_connected``.
    """

    async def async_start(self) -> None: ...

    async def async_stop(self) -> None: ...

    def async_connection_changed(self, connected: bool) -> None: ...


class BridgeHub:
    """One hub per config entry; ``entry.runtime_data`` holds it while the entry is loaded."""

    def __init__(
        self,
        hass: HomeAssistant,
        entry: ConfigEntry,
        registry: CommandRegistry,
        store: Any,
        *,
        clock: Clock | None = None,
    ) -> None:
        """Wire the registry callbacks; nothing touches MQTT until ``async_start``.

        ``store`` is a ``store.CommandStore`` (typed loosely to keep the import graph
        one-directional); the registry must already be loaded from it and bound.
        """
        self.hass = hass
        self.entry = entry
        self.registry = registry
        self.store = store
        self._clock: Clock = clock or default_clock
        self.options = HubOptions.from_entry(entry)
        self.prefix = self.options.topic_prefix
        self.instance_id: str = str(entry.data.get(CONF_INSTANCE_ID) or uuid.uuid4())
        self.registry.update_settings(self.options.registry_settings())
        self.registry.set_callbacks(
            on_event=self._on_registry_event, on_persist=self._on_registry_persist
        )

        self.robots: dict[str, Robot] = {}
        self.started = False
        self.ready = False
        self.mqtt_connected = False
        self.reconciled = False
        self.subscriptions: list[tuple[str, int]] = []
        self.counters: dict[str, int] = {
            "discarded_segments": 0,
            "invalid_payloads": 0,
            "mismatched_payloads": 0,
            "seq_regressions": 0,
            "handler_errors": 0,
            "publish_errors": 0,
            "heartbeats_sent": 0,
        }
        self.heartbeat_seq = 0
        self.last_heartbeat_sent_at: int | None = None
        self._components: list[HubComponent] = []
        self._unsubscribers: list[CALLBACK_TYPE] = []
        self._timers: list[CALLBACK_TYPE] = []
        self._ready_pending: set[str] = set()
        self._ready_timeout: CALLBACK_TYPE | None = None
        self._settle_handle: CALLBACK_TYPE | None = None
        self._grace_handles: dict[str, CALLBACK_TYPE] = {}
        self._feedback_sent_at: dict[str, int] = {}
        self._feedback_pending: set[str] = set()
        self._logged_topics: set[str] = set()
        self._open_issues: set[str] = set()

    # ----------------------------------------------------------------- lifecycle

    @property
    def now(self) -> int:
        """The hub clock, epoch milliseconds."""
        return int(self._clock())

    def register_component(self, component: HubComponent) -> None:
        """Add a component; started at ``async_start`` (or at once if already started)."""
        self._components.append(component)
        if self.started:
            self.hass.async_create_task(component.async_start())

    async def async_start(self) -> None:
        """Subscribe, watch readiness and the connection, start the timers and components."""
        if self.started:
            return
        self.started = True
        self.mqtt_connected = mqtt.is_connected(self.hass)
        prefix = self.prefix
        plan: list[tuple[str, int, Callable[[ReceiveMessage], None]]] = [
            (announce_filter(prefix), 1, self._on_announce),
            (availability_filter(prefix), 1, self._on_availability),
            (state_filter(prefix), 1, self._on_state),
            (heartbeat_filter(prefix), 1, self._on_heartbeat),
            (ack_filter(prefix), 1, self._on_ack),
            (result_filter(prefix), 1, self._on_result),
            (feedback_filter(prefix), 0, self._on_feedback),
            (joints_filter(prefix), 0, self._on_joints),
        ]
        for topic, qos, handler in plan:
            self._unsubscribers.append(
                await mqtt.async_subscribe(self.hass, topic, handler, qos=qos, encoding=None)
            )
            self.subscriptions.append((topic, qos))
        self._ready_pending = {ack_filter(prefix), result_filter(prefix)}
        for topic in list(self._ready_pending):
            self._unsubscribers.append(
                mqtt.async_on_subscribe_done(
                    self.hass, topic, 1, partial(self._on_subscribe_done, topic)
                )
            )
        self._unsubscribers.append(
            mqtt.async_subscribe_connection_status(self.hass, self._on_connection_status)
        )
        self._ready_timeout = async_call_later(self.hass, READY_TIMEOUT_S, self._on_ready_timeout)
        if self.options.heartbeat_interval_s > 0:
            self._timers.append(
                async_track_time_interval(
                    self.hass,
                    self._async_heartbeat_tick,
                    timedelta(seconds=self.options.heartbeat_interval_s),
                    name=f"{DOMAIN} heartbeat",
                )
            )
        self._timers.append(
            async_track_time_interval(
                self.hass,
                self._async_housekeeping_tick,
                timedelta(seconds=HOUSEKEEPING_INTERVAL_S),
                name=f"{DOMAIN} housekeeping",
            )
        )
        for component in list(self._components):
            await component.async_start()
        self._update_settle_timer()

    async def async_stop(self) -> None:
        """Tear everything down; the registry stays loaded for the next setup."""
        if not self.started:
            return
        self.started = False
        for cancel in self._timers:
            cancel()
        self._timers.clear()
        if self._ready_timeout is not None:
            self._ready_timeout()
            self._ready_timeout = None
        self._cancel_settle_timer()
        for handle in self._grace_handles.values():
            handle()
        self._grace_handles.clear()
        for unsubscribe in self._unsubscribers:
            unsubscribe()
        self._unsubscribers.clear()
        self.subscriptions.clear()
        for component in list(self._components):
            await component.async_stop()
        self.registry.cancel_waiters()
        self.registry.set_callbacks(on_event=None, on_persist=None)
        await self.store.async_flush()

    @callback
    def async_clear_issues(self) -> None:
        """Delete every repair issue this hub opened (entry removal)."""
        for iid in list(self._open_issues):
            ir.async_delete_issue(self.hass, DOMAIN, iid)
        self._open_issues.clear()

    @property
    def open_issues(self) -> frozenset[str]:
        return frozenset(self._open_issues)

    # ----------------------------------------------------------------- readiness and connection

    @callback
    def _on_subscribe_done(self, topic: str) -> None:
        self._ready_pending.discard(topic)
        if not self._ready_pending and not self.ready:
            self._set_ready()

    @callback
    def _on_ready_timeout(self, _now: datetime) -> None:
        self._ready_timeout = None
        if not self.ready:
            _LOGGER.warning(
                "Ack and result subscriptions not confirmed within %s s; continuing",
                READY_TIMEOUT_S,
            )
            self._set_ready()

    @callback
    def _set_ready(self) -> None:
        self.ready = True
        if self._ready_timeout is not None:
            self._ready_timeout()
            self._ready_timeout = None
        self._update_settle_timer()

    @callback
    def _on_connection_status(self, connected: bool) -> None:
        if connected == self.mqtt_connected:
            return
        self.mqtt_connected = connected
        now = self.now
        if connected:
            # The replay burst is about to repeat; staleness restarts from now.
            for robot in self.robots.values():
                robot.last_heartbeat_at = now
        _LOGGER.log(
            logging.INFO if connected else logging.WARNING,
            "MQTT %s; robots are %s",
            "connected" if connected else "disconnected",
            "back" if connected else "unavailable until it returns",
        )
        self._update_settle_timer()
        async_dispatcher_send(self.hass, SIGNAL_CONNECTION_CHANGED)
        for component in self._components:
            component.async_connection_changed(connected)

    def _update_settle_timer(self) -> None:
        if self.reconciled or not self.started:
            return
        if self.ready and self.mqtt_connected:
            if self._settle_handle is None:
                self._settle_handle = async_call_later(
                    self.hass, REPLAY_SETTLE_S, self._on_settle_expired
                )
        else:
            self._cancel_settle_timer()

    def _cancel_settle_timer(self) -> None:
        if self._settle_handle is not None:
            self._settle_handle()
            self._settle_handle = None

    @callback
    def _on_settle_expired(self, _now: datetime) -> None:
        self._settle_handle = None
        if self.reconciled or not (self.ready and self.mqtt_connected):
            return
        self.reconciled = True
        snapshots = {
            robot_id: self._snapshot(robot)
            for robot_id, robot in self.robots.items()
            if robot.announce is not None
        }
        self.registry.reconcile_all(snapshots)
        _LOGGER.debug("Reconciled %d outstanding command(s)", len(self.registry.outstanding()))

    # ----------------------------------------------------------------- timers

    async def _async_heartbeat_tick(self, _now: datetime) -> None:
        if not self.mqtt_connected or not self.started:
            return
        self.heartbeat_seq += 1
        sent_at = self.now
        payload: dict[str, Any] = {
            "schema": SCHEMA_VERSION,
            "ts": sent_at,
            "seq": self.heartbeat_seq,
            "interval_ms": int(self.options.heartbeat_interval_s * 1000),
            "instance_id": self.instance_id,
        }
        try:
            await self._async_publish(
                ha_heartbeat_topic(self.prefix),
                encode_payload(payload),
                qos=0,
                retain=False,
                expiry=HA_HEARTBEAT_EXPIRY_S,
            )
        except HomeAssistantError as error:
            self.counters["publish_errors"] += 1
            _LOGGER.debug("HA heartbeat not published: %s", error)
            return
        self.counters["heartbeats_sent"] += 1
        self.last_heartbeat_sent_at = sent_at

    @callback
    def _async_housekeeping_tick(self, _now: datetime) -> None:
        self.registry.housekeeping()
        now = self.now
        for robot_id, robot in list(self.robots.items()):
            if self._check_staleness(robot, now):
                self._dispatch_updated(robot_id)
            if (
                robot.duplicate_open
                and robot.duplicate_evidence_at is not None
                and now - robot.duplicate_evidence_at >= DUPLICATE_QUIET_S * 1000
            ):
                robot.duplicate_open = False
                self._close_issue(ISSUE_DUPLICATE_ROBOT_ID, robot_id)
                self._dispatch_updated(robot_id)
            if robot_id in self._feedback_pending and (
                now - self._feedback_sent_at.get(robot_id, 0) >= FEEDBACK_THROTTLE_MS
            ):
                self._feedback_pending.discard(robot_id)
                self._feedback_sent_at[robot_id] = now
                self._dispatch_updated(robot_id)

    def _check_staleness(self, robot: Robot, now: int) -> bool:
        """Update the heartbeat flags of one robot; True when either changed."""
        interval = robot.heartbeat_interval_s
        if robot.announce is None or interval <= 0 or not self.mqtt_connected:
            changed = robot.heartbeat_stale or robot.heartbeat_unavailable
            robot.heartbeat_stale = False
            robot.heartbeat_unavailable = False
            return changed
        reference = robot.last_heartbeat_at or robot.first_seen_at or now
        age_ms = now - reference
        stale = age_ms > interval * HEARTBEAT_STALE_INTERVALS * 1000
        after_s = self.options.heartbeat_unavailable_after_s
        unavailable = after_s > 0 and age_ms > after_s * 1000
        changed = stale != robot.heartbeat_stale or unavailable != robot.heartbeat_unavailable
        if stale and not robot.heartbeat_stale:
            _LOGGER.warning("Robot %s heartbeat is stale (%.1f s)", robot.robot_id, age_ms / 1000)
        if unavailable and not robot.heartbeat_unavailable:
            _LOGGER.warning(
                "Robot %s is unavailable: no heartbeat for %.0f s", robot.robot_id, age_ms / 1000
            )
        robot.heartbeat_stale = stale
        robot.heartbeat_unavailable = unavailable
        return changed

    # ----------------------------------------------------------------- inbound routing

    def _robot_segment(self, msg: ReceiveMessage) -> tuple[str, list[str]] | None:
        """The validated robot segment and the rest of the topic, or ``None`` to discard."""
        split = split_robot_topic(self.prefix, msg.topic)
        if split is None:
            self.counters["discarded_segments"] += 1
            return None
        segment, rest = split
        if segment == RESERVED_ROBOT_ID:
            return None
        if not is_valid_robot_id(segment):
            self.counters["discarded_segments"] += 1
            self._log_once(msg.topic, "Discarding %s: robot segment fails the pattern", msg.topic)
            return None
        return segment, rest

    def _task_segment(self, msg: ReceiveMessage, rest: list[str]) -> str | None:
        if len(rest) != 3 or rest[0] != "task" or not is_valid_task_id(rest[1]):
            self.counters["discarded_segments"] += 1
            self._log_once(msg.topic, "Discarding %s: task id segment is not a UUID4", msg.topic)
            return None
        return rest[1]

    def _log_once(self, key: str, message: str, *args: Any) -> None:
        if key in self._logged_topics:
            return
        if len(self._logged_topics) < 1000:
            self._logged_topics.add(key)
        _LOGGER.debug(message, *args)

    def _robot(self, robot_id: str, *, create: bool) -> Robot | None:
        robot = self.robots.get(robot_id)
        if robot is None and create:
            robot = Robot(robot_id)
            robot.first_seen_at = self.now
            self.robots[robot_id] = robot
        return robot

    def _snapshot(self, robot: Robot) -> RobotSnapshot:
        paused: str | None = None
        state = robot.state
        if (
            state is not None
            and state.paused
            and state.task_id is not None
            and state.session == robot.session
        ):
            paused = state.task_id
        return RobotSnapshot(session=robot.session, paused_task_id=paused)

    def _guard(self, kind: str, msg: ReceiveMessage, handler: Callable[[], None]) -> None:
        """Run a handler so that no payload can break the subscription callback."""
        try:
            handler()
        except ProtocolError as error:
            self.counters["invalid_payloads"] += 1
            _LOGGER.debug("Invalid %s on %s: %s", kind, msg.topic, error.reason)
        except Exception:  # a handler bug must never kill the subscription callback
            self.counters["handler_errors"] += 1
            _LOGGER.exception("Error handling %s on %s", kind, msg.topic)

    # ----------------------------------------------------------------- announce

    @callback
    def _on_announce(self, msg: ReceiveMessage) -> None:
        self._guard("announce", msg, lambda: self._handle_announce(msg))

    def _handle_announce(self, msg: ReceiveMessage) -> None:
        split = self._robot_segment(msg)
        if split is None:
            return
        robot_id, _rest = split
        payload = _payload_bytes(msg)
        if not payload:
            self._handle_tombstone(robot_id)
            return
        try:
            announce = decode_announce(payload)
        except ProtocolError as error:
            self.counters["invalid_payloads"] += 1
            self._open_issue(
                ISSUE_INVALID_ANNOUNCE,
                robot_id,
                {"robot_id": robot_id, "topic": msg.topic, "reason": error.reason},
            )
            _LOGGER.warning("Invalid announce on %s: %s", msg.topic, error.reason)
            return
        if announce.robot_id != robot_id:
            self.counters["mismatched_payloads"] += 1
            self._open_issue(
                ISSUE_INVALID_ANNOUNCE,
                robot_id,
                {
                    "robot_id": robot_id,
                    "topic": msg.topic,
                    "reason": f"payload robot_id {announce.robot_id!r} differs from the topic",
                },
            )
            return
        self._close_issue(ISSUE_INVALID_ANNOUNCE, robot_id)
        self._apply_announce(robot_id, announce)

    def _apply_announce(self, robot_id: str, announce: Announce) -> None:
        now = self.now
        robot = self._robot(robot_id, create=True)
        assert robot is not None
        is_new = robot.announce is None
        previous = robot.announce
        robot.announce = announce
        self._ensure_device(robot)
        session_changed = self._note_identity(robot, announce.session, announce.instance, now)
        if announce.hmac_required and self.options.secret is None:
            self._open_issue(
                ISSUE_HMAC_REJECTED,
                robot_id,
                {"robot_id": robot_id, "reason": "the robot requires signed commands"},
            )
            robot.hmac_rejected_open = True
        elif robot.hmac_rejected_open and not announce.hmac_required:
            robot.hmac_rejected_open = False
            self._close_issue(ISSUE_HMAC_REJECTED, robot_id)
        if announce.joints_ignored is not None and (
            previous is None or previous.joints_ignored != announce.joints_ignored
        ):
            _LOGGER.warning(
                "Robot %s announce: %s; no joint sensors are made",
                robot_id,
                announce.joints_ignored,
            )
        if is_new:
            _LOGGER.info("Robot %s announced (%s)", robot_id, announce.name)
            async_dispatcher_send(self.hass, SIGNAL_ROBOT_ADDED, robot_id)
            if self.reconciled:
                self.registry.reconcile(robot_id, self._snapshot(robot))
        else:
            assert previous is not None
            before = set(previous.task_names)
            after = set(announce.task_names)
            if before != after:
                async_dispatcher_send(
                    self.hass,
                    SIGNAL_ROBOT_TASKS_CHANGED,
                    robot_id,
                    tuple(name for name in announce.task_names if name not in before),
                    tuple(name for name in previous.task_names if name not in after),
                )
            added, removed = _joint_diff(previous.joints, announce.joints)
            if added or removed:
                # The positions held so far belong to the old layout (or unit); the node
                # publishes a fresh set with every announce, so nothing is lost by waiting.
                robot.joints = None
                async_dispatcher_send(
                    self.hass, SIGNAL_ROBOT_JOINTS_CHANGED, robot_id, added, removed
                )
            self._dispatch_updated(robot_id)
        if session_changed:
            self._schedule_grace(robot_id)

    def _ensure_device(self, robot: Robot) -> None:
        info = self.device_info(robot.robot_id)
        registry = dr.async_get(self.hass)
        device = registry.async_get_or_create(config_entry_id=self.entry.entry_id, **info)
        robot.device_id = device.id

    def _handle_tombstone(self, robot_id: str) -> None:
        robot = self.robots.pop(robot_id, None)
        if robot is None:
            return
        _LOGGER.info("Robot %s removed (tombstone)", robot_id)
        handle = self._grace_handles.pop(robot_id, None)
        if handle is not None:
            handle()
        for kind in (
            ISSUE_CLOCK_SKEW,
            ISSUE_HMAC_REJECTED,
            ISSUE_INVALID_ANNOUNCE,
            ISSUE_DUPLICATE_ROBOT_ID,
        ):
            self._close_issue(kind, robot_id)
        async_dispatcher_send(self.hass, SIGNAL_ROBOT_REMOVED, robot_id)
        registry = dr.async_get(self.hass)
        device = registry.async_get_device_by_identifier((DOMAIN, robot_id), self.entry.entry_id)
        if device is not None:
            # This entry is the device's only owner, so removing the device outright is
            # the same as taking the entry away from it (and the deprecated
            # ``remove_config_entry_id`` form raises from Home Assistant 2027.8).
            registry.async_remove_device(device.id)

    def _note_identity(self, robot: Robot, session: str, instance: str, now: int) -> bool:
        """Track session and instance; apply the duplicate heuristics. True on a session change."""
        session_changed = False
        if robot.session != session:
            if robot.session is not None:
                session_changed = True
                robot.session_change_count += 1
                robot.session_changes.append(now)
                window = DUPLICATE_SESSION_WINDOW_S * 1000
                robot.session_changes = [t for t in robot.session_changes if now - t <= window]
                if len(robot.session_changes) > DUPLICATE_SESSION_CHANGES:
                    self._duplicate_evidence(
                        robot,
                        now,
                        f"{len(robot.session_changes)} session changes in "
                        f"{DUPLICATE_SESSION_WINDOW_S // 60} minutes",
                    )
                _LOGGER.info("Robot %s has a new session (node restarted)", robot.robot_id)
            robot.session = session
        interval_ms = int(max(robot.heartbeat_interval_s, DEFAULT_HEARTBEAT_INTERVAL_S) * 1000)
        for other, seen_at in robot.instances_seen_at.items():
            if other != instance and now - seen_at <= interval_ms:
                self._duplicate_evidence(robot, now, f"instances {other} and {instance}")
                break
        robot.instances_seen_at[instance] = now
        robot.instances_seen_at = {
            key: seen_at
            for key, seen_at in robot.instances_seen_at.items()
            if now - seen_at <= DUPLICATE_SESSION_WINDOW_S * 1000
        }
        robot.instance = instance
        return session_changed

    def _duplicate_evidence(self, robot: Robot, now: int, detail: str) -> None:
        robot.duplicate_evidence_at = now
        if not robot.duplicate_open:
            robot.duplicate_open = True
            _LOGGER.error("Two nodes may be claiming robot id %s: %s", robot.robot_id, detail)
            self._open_issue(
                ISSUE_DUPLICATE_ROBOT_ID,
                robot.robot_id,
                {"robot_id": robot.robot_id, "detail": detail},
            )
            self._dispatch_updated(robot.robot_id)

    def _schedule_grace(self, robot_id: str) -> None:
        handle = self._grace_handles.pop(robot_id, None)
        if handle is not None:
            handle()
        self._grace_handles[robot_id] = async_call_later(
            self.hass, SESSION_CHANGE_GRACE_S, partial(self._on_grace_expired, robot_id)
        )

    @callback
    def _on_grace_expired(self, robot_id: str, _now: datetime) -> None:
        self._grace_handles.pop(robot_id, None)
        robot = self.robots.get(robot_id)
        if robot is None or robot.announce is None or not self.reconciled:
            return
        self.registry.reconcile(robot_id, self._snapshot(robot))
        self._dispatch_updated(robot_id)

    # ----------------------------------------------------------------- availability, state, heartbeat

    @callback
    def _on_availability(self, msg: ReceiveMessage) -> None:
        self._guard("availability", msg, lambda: self._handle_availability(msg))

    def _handle_availability(self, msg: ReceiveMessage) -> None:
        split = self._robot_segment(msg)
        if split is None:
            return
        robot_id, _rest = split
        payload = _payload_bytes(msg)
        if not payload:
            robot = self.robots.get(robot_id)
            if robot is not None and robot.availability is not None:
                robot.availability = None
                self._dispatch_updated(robot_id)
            return
        availability = decode_availability(payload)
        robot = self._robot(robot_id, create=True)
        assert robot is not None
        previous = robot.availability
        robot.availability = availability
        if previous is not availability:
            if availability is Availability.OFFLINE and previous is not None:
                _LOGGER.warning("Robot %s went offline", robot_id)
            elif availability is Availability.ONLINE and previous is Availability.OFFLINE:
                _LOGGER.info("Robot %s is back online", robot_id)
            if availability is Availability.ONLINE:
                robot.last_heartbeat_at = self.now
            self._dispatch_updated(robot_id)

    @callback
    def _on_state(self, msg: ReceiveMessage) -> None:
        self._guard("state", msg, lambda: self._handle_state(msg))

    def _handle_state(self, msg: ReceiveMessage) -> None:
        split = self._robot_segment(msg)
        if split is None:
            return
        robot_id, _rest = split
        payload = _payload_bytes(msg)
        if not payload:
            robot = self.robots.get(robot_id)
            if robot is not None and robot.state is not None:
                robot.state = None
                self._dispatch_updated(robot_id)
            return
        state = decode_state(payload)
        if state.robot_id != robot_id:
            self.counters["mismatched_payloads"] += 1
            return
        robot = self._robot(robot_id, create=True)
        assert robot is not None
        previous = robot.state
        if previous is not None and previous.session == state.session and state.seq < previous.seq:
            self.counters["seq_regressions"] += 1
            _LOGGER.debug(
                "Dropping state seq %d for %s (have %d)", state.seq, robot_id, previous.seq
            )
            return
        robot.state = state
        if previous is None or previous.task_id != state.task_id:
            robot.feedback = None
        session_changed = self._note_identity(robot, state.session, state.instance, self.now)
        self.registry.sync_pause(robot_id, state.task_id, state.paused)
        self._dispatch_updated(robot_id)
        if session_changed and robot.announce is not None:
            self._schedule_grace(robot_id)

    @callback
    def _on_heartbeat(self, msg: ReceiveMessage) -> None:
        self._guard("heartbeat", msg, lambda: self._handle_heartbeat(msg))

    def _handle_heartbeat(self, msg: ReceiveMessage) -> None:
        split = self._robot_segment(msg)
        if split is None:
            return
        robot_id, _rest = split
        heartbeat = decode_heartbeat(_payload_bytes(msg))
        if heartbeat.robot_id != robot_id:
            self.counters["mismatched_payloads"] += 1
            return
        robot = self._robot(robot_id, create=True)
        assert robot is not None
        now = self.now
        robot.last_heartbeat = heartbeat
        robot.last_heartbeat_at = now
        robot.counters = dict(heartbeat.counters)
        robot.ha_clock_offset_ms = heartbeat.ha_clock_offset_ms
        session_changed = self._note_identity(robot, heartbeat.session, heartbeat.instance, now)
        if self._check_staleness(robot, now):
            self._dispatch_updated(robot_id)
        if session_changed and robot.announce is not None:
            self._schedule_grace(robot_id)

    # ----------------------------------------------------------------- replies

    @callback
    def _on_ack(self, msg: ReceiveMessage) -> None:
        self._guard("ack", msg, lambda: self._handle_ack(msg))

    def _handle_ack(self, msg: ReceiveMessage) -> None:
        split = self._robot_segment(msg)
        if split is None:
            return
        robot_id, rest = split
        task_id = self._task_segment(msg, rest)
        if task_id is None:
            return
        payload = _payload_bytes(msg)
        if not payload:
            return  # an expired or cleared retained ack
        ack = decode_ack(payload)
        if ack.robot_id != robot_id or ack.id != task_id:
            self.counters["mismatched_payloads"] += 1
            return
        robot = self._robot(robot_id, create=True)
        assert robot is not None
        robot.last_ack = ack
        self.registry.handle_ack(ack, retained=msg.retain)
        self._apply_ack_repairs(robot, ack)
        self._dispatch_updated(robot_id)

    def _apply_ack_repairs(self, robot: Robot, ack: Any) -> None:
        robot_id = robot.robot_id
        skew = ack.clock_skew_ms
        skewed = ack.code in (ACK_CODE_STALE, ACK_CODE_FUTURE) or (
            skew is not None and abs(skew) >= CLOCK_SKEW_WARN_MS
        )
        if skewed:
            robot.clock_skew_open = True
            self._open_issue(
                ISSUE_CLOCK_SKEW,
                robot_id,
                {"robot_id": robot_id, "skew_ms": str(skew if skew is not None else "unknown")},
            )
        elif robot.clock_skew_open and skew is not None:
            robot.clock_skew_open = False
            self._close_issue(ISSUE_CLOCK_SKEW, robot_id)
        if ack.code == ACK_CODE_UNAUTHORISED:
            reason = (
                "the robot refused the signature (secret mismatch)"
                if self.options.secret is not None
                else "the robot requires signed commands"
            )
            robot.hmac_rejected_open = True
            self._open_issue(
                ISSUE_HMAC_REJECTED, robot_id, {"robot_id": robot_id, "reason": reason}
            )
        elif ack.accepted and robot.hmac_rejected_open:
            robot.hmac_rejected_open = False
            self._close_issue(ISSUE_HMAC_REJECTED, robot_id)

    @callback
    def _on_feedback(self, msg: ReceiveMessage) -> None:
        self._guard("feedback", msg, lambda: self._handle_feedback(msg))

    def _handle_feedback(self, msg: ReceiveMessage) -> None:
        split = self._robot_segment(msg)
        if split is None:
            return
        robot_id, rest = split
        task_id = self._task_segment(msg, rest)
        if task_id is None:
            return
        feedback = decode_feedback(_payload_bytes(msg))
        if feedback.robot_id != robot_id or feedback.id != task_id:
            self.counters["mismatched_payloads"] += 1
            return
        robot = self.robots.get(robot_id)
        if robot is None:
            return
        command = self.registry.get(task_id)
        active = robot.state is not None and robot.state.task_id == task_id
        if not active and (command is None or not command.is_outstanding):
            return
        robot.feedback = feedback
        now = self.now
        if now - self._feedback_sent_at.get(robot_id, 0) >= FEEDBACK_THROTTLE_MS:
            self._feedback_sent_at[robot_id] = now
            self._feedback_pending.discard(robot_id)
            self._dispatch_updated(robot_id)
        else:
            self._feedback_pending.add(robot_id)

    @callback
    def _on_joints(self, msg: ReceiveMessage) -> None:
        self._guard("joints", msg, lambda: self._handle_joints(msg))

    def _handle_joints(self, msg: ReceiveMessage) -> None:
        """Keep the latest positions of an announced joint layout; drop what does not fit.

        A payload for a robot that has not announced a ``joints`` block (yet) is
        ignored without a count: a live QoS 0 message can overtake the retained
        announce on a fresh subscription, and the node repeats the positions.
        One whose ``positions`` length differs from the announced names, or
        whose ``velocities`` are not one per position, is counted as
        mismatched, and one whose ``seq`` regresses within the same session as
        a regression, exactly as for ``state``.
        """
        split = self._robot_segment(msg)
        if split is None:
            return
        robot_id, _rest = split
        joints = decode_joints(_payload_bytes(msg))
        if joints.robot_id != robot_id:
            self.counters["mismatched_payloads"] += 1
            return
        robot = self.robots.get(robot_id)
        info = robot.joint_info if robot is not None else None
        if robot is None or info is None:
            self._log_once(
                f"{msg.topic}#unannounced", "Ignoring %s: no joints block announced", msg.topic
            )
            return
        if len(joints.positions) != info.count:
            self.counters["mismatched_payloads"] += 1
            self._log_once(
                f"{msg.topic}#length",
                "Dropping joints for %s: %d positions for %d announced names",
                robot_id,
                len(joints.positions),
                info.count,
            )
            return
        if joints.velocities is not None and len(joints.velocities) != info.count:
            self.counters["mismatched_payloads"] += 1
            self._log_once(
                f"{msg.topic}#velocities",
                "Dropping joints for %s: %d velocities for %d positions",
                robot_id,
                len(joints.velocities),
                info.count,
            )
            return
        previous = robot.joints
        if (
            previous is not None
            and previous.session == joints.session
            and joints.seq < previous.seq
        ):
            self.counters["seq_regressions"] += 1
            _LOGGER.debug(
                "Dropping joints seq %d for %s (have %d)", joints.seq, robot_id, previous.seq
            )
            return
        robot.joints = joints
        session_changed = self._note_identity(robot, joints.session, joints.instance, self.now)
        async_dispatcher_send(self.hass, SIGNAL_ROBOT_JOINTS_UPDATED, robot_id)
        if session_changed:
            self._schedule_grace(robot_id)

    @callback
    def _on_result(self, msg: ReceiveMessage) -> None:
        self._guard("result", msg, lambda: self._handle_result(msg))

    def _handle_result(self, msg: ReceiveMessage) -> None:
        split = self._robot_segment(msg)
        if split is None:
            return
        robot_id, rest = split
        task_id = self._task_segment(msg, rest)
        if task_id is None:
            return
        payload = _payload_bytes(msg)
        if not payload:
            return  # an expired or cleared retained result
        result = decode_result(payload)
        if result.robot_id != robot_id or result.id != task_id:
            self.counters["mismatched_payloads"] += 1
            return
        robot = self._robot(robot_id, create=True)
        disposition = self.registry.handle_result(result, retained=msg.retain)
        if (
            robot is not None
            and disposition is ResultDisposition.DUPLICATE
            and msg.retain
            and str(result.kind) == str(TaskKind.MOTION)
        ):
            # A retained replay after a reload or restart: the event fired long ago and
            # must not fire again, but the last_result sensor would otherwise come back
            # as unknown. Restore it from the newest replayed motion result.
            current = robot.last_result
            if current is None or (current.finished_at or 0) < result.finished_at:
                robot.last_result = LastResult(
                    outcome=TaskOutcome(str(result.status)),
                    task_id=result.id,
                    task=result.task,
                    code=result.code,
                    reason=result.reason,
                    data=dict(result.data),
                    finished_at=result.finished_at,
                    duration_ms=result.duration_ms,
                    synthetic=False,
                )
        self._dispatch_updated(robot_id)

    # ----------------------------------------------------------------- registry callbacks

    @callback
    def _on_registry_event(self, name: str, data: dict[str, Any]) -> None:
        robot_id = data.get("robot_id")
        robot = self.robots.get(robot_id) if isinstance(robot_id, str) else None
        data["device_id"] = robot.device_id if robot is not None else None
        self.hass.bus.async_fire(name, data)
        if isinstance(robot_id, str):
            async_dispatcher_send(self.hass, SIGNAL_TASK_EVENT, robot_id, name, data)
        if (
            name == EVENT_TASK_RESULT
            and robot is not None
            and data.get("kind") == str(TaskKind.MOTION)
        ):
            robot.last_result = LastResult(
                outcome=TaskOutcome(str(data["status"])),
                task_id=str(data["task_id"]),
                task=data.get("task"),
                code=data.get("code"),
                reason=data.get("reason"),
                data=dict(data.get("data") or {}),
                finished_at=data.get("finished_at"),
                duration_ms=data.get("duration_ms"),
                synthetic=bool(data.get("synthetic")),
            )
            self._dispatch_updated(robot.robot_id)

    @callback
    def _on_registry_persist(self, urgent: bool) -> None:
        self.store.async_request_save(urgent)

    @callback
    def _dispatch_updated(self, robot_id: str) -> None:
        async_dispatcher_send(self.hass, SIGNAL_ROBOT_UPDATED, robot_id)

    # ----------------------------------------------------------------- commands out

    def check_task(self, robot_id: str, task: str, params: Mapping[str, Any] | None) -> TaskSpec:
        """Validate a task command without sending it; returns the announced task.

        Raises exactly what ``async_send_task`` raises before anything is published
        (``robot_unknown``, ``robot_unavailable``, ``not_ready``, ``task_unknown``,
        ``invalid_params``, ``too_many_pending``), so a caller with several targets
        can check every one of them before the first command goes out.
        """
        robot = self._require_robot(robot_id)
        spec = robot.task(task)
        if spec is None:
            raise ServiceValidationError(
                translation_domain=DOMAIN,
                translation_key="task_unknown",
                translation_placeholders={"robot_id": robot_id, "task": task},
            )
        problems = validate_params(spec.params, dict(params or {}))
        if problems:
            raise ServiceValidationError(
                translation_domain=DOMAIN,
                translation_key="invalid_params",
                translation_placeholders={
                    "robot_id": robot_id,
                    "task": task,
                    "problems": "; ".join(problems),
                },
            )
        self._check_pending_cap(robot_id)
        return spec

    def check_cancel(self, robot_id: str, target_id: str | None) -> None:
        """Validate a cancel without sending it; the counterpart of :meth:`check_task`."""
        self._require_robot(robot_id)
        if target_id is not None and not is_valid_task_id(target_id):
            raise ServiceValidationError(
                translation_domain=DOMAIN,
                translation_key="invalid_params",
                translation_placeholders={
                    "robot_id": robot_id,
                    "task": "cancel",
                    "problems": f"task_id {target_id!r} is not a UUID4",
                },
            )
        self._check_pending_cap(robot_id)

    async def async_send_task(
        self,
        robot_id: str,
        task: str,
        params: Mapping[str, Any] | None,
        *,
        source: CommandSource,
        timeout_s: float | None = None,
        max_age_ms: int | None = None,
    ) -> PendingCommand:
        """Validate, register, persist and publish a task command; returns the tracked command.

        Raises ``ServiceValidationError`` (``robot_unknown``, ``robot_unavailable``,
        ``task_unknown``, ``invalid_params``, ``too_many_pending``) or
        ``HomeAssistantError`` (``not_ready``, ``publish_failed``), all translated
        under this domain. Callers wait with ``registry.wait_ack`` or ``wait_result``.
        """
        spec = self.check_task(robot_id, task, params)
        robot = self.robots[robot_id]
        requested = None if timeout_s is None else float(timeout_s)
        effective_timeout = requested if requested is not None else float(spec.timeout_s)
        command, raw = self._create(
            robot=robot,
            type=CommandType.TASK,
            task=spec.name,
            kind=spec.kind,
            params=dict(params or {}),
            target_id=None,
            source=source,
            max_age_ms=max_age_ms,
            timeout_s=effective_timeout,
            requested_timeout_s=requested,
        )
        await self.async_publish_command(command, raw)
        return command

    async def async_cancel_task(
        self,
        robot_id: str,
        target_id: str | None,
        *,
        source: CommandSource,
        max_age_ms: int | None = None,
    ) -> PendingCommand:
        """Validate, register, persist and publish a cancel; ``target_id`` ``None`` is the active task."""
        self.check_cancel(robot_id, target_id)
        robot = self.robots[robot_id]
        command, raw = self._create(
            robot=robot,
            type=CommandType.CANCEL,
            task=None,
            kind=None,
            params={},
            target_id=target_id,
            source=source,
            max_age_ms=max_age_ms,
            timeout_s=None,
        )
        await self.async_publish_command(command, raw)
        return command

    def _check_pending_cap(self, robot_id: str) -> None:
        limit = self.registry.settings.max_pending_per_robot
        if limit > 0 and self.registry.pending_count(robot_id) >= limit:
            raise ServiceValidationError(
                translation_domain=DOMAIN,
                translation_key="too_many_pending",
                translation_placeholders={"robot_id": robot_id, "limit": str(limit)},
            )

    def _require_robot(self, robot_id: str) -> Robot:
        robot = self.robots.get(robot_id)
        if robot is None or robot.announce is None:
            raise ServiceValidationError(
                translation_domain=DOMAIN,
                translation_key="robot_unknown",
                translation_placeholders={"robot_id": robot_id},
            )
        if not self.robot_available(robot_id):
            raise ServiceValidationError(
                translation_domain=DOMAIN,
                translation_key="robot_unavailable",
                translation_placeholders={"robot_id": robot_id},
            )
        if not self.ready:
            raise HomeAssistantError(translation_domain=DOMAIN, translation_key="not_ready")
        return robot

    def _create(
        self,
        *,
        robot: Robot,
        type: CommandType,
        task: str | None,
        kind: TaskKind | None,
        params: dict[str, Any],
        target_id: str | None,
        source: CommandSource,
        max_age_ms: int | None,
        timeout_s: float | None,
        requested_timeout_s: float | None = None,
    ) -> tuple[PendingCommand, bytes]:
        """Build and encode the wire payload, then register the command; returns both.

        The payload is built first so that a command the contract refuses (over
        16 KiB once encoded, or a parameter value JSON cannot represent) never
        enters the registry: it is reported as ``invalid_params`` and nothing is
        tracked, persisted or counted against the pending cap.
        """
        age = int(max_age_ms) if max_age_ms is not None else self.options.command_max_age_ms
        age = max(1, age)
        if source.instance_id is None:
            source = replace(source, instance_id=self.instance_id)
        source_payload = source.to_payload()
        task_id = str(uuid.uuid4())
        issued_at = self.now
        topic = cmd_topic(self.prefix, robot.robot_id)
        try:
            if type is CommandType.TASK:
                assert task is not None
                payload = task_command_payload(
                    command_id=task_id,
                    robot_id=robot.robot_id,
                    task=task,
                    params=params,
                    issued_at=issued_at,
                    max_age_ms=age,
                    source=source_payload,
                    timeout_s=requested_timeout_s,
                )
            else:
                payload = cancel_command_payload(
                    command_id=task_id,
                    robot_id=robot.robot_id,
                    target_id=target_id,
                    issued_at=issued_at,
                    max_age_ms=age,
                    source=source_payload,
                )
            _final, raw = encode_command(payload, topic=topic, secret=self.options.secret)
        except (TypeError, ValueError) as error:  # ProtocolError is a ValueError
            raise ServiceValidationError(
                translation_domain=DOMAIN,
                translation_key="invalid_params",
                translation_placeholders={
                    "robot_id": robot.robot_id,
                    "task": task if task is not None else str(type),
                    "problems": str(error),
                },
            ) from error
        try:
            command = self.registry.create(
                task_id=task_id,
                robot_id=robot.robot_id,
                type=type,
                task=task,
                kind=kind,
                params=params,
                target_id=target_id,
                session=robot.session,
                source=source_payload,
                issued_at=issued_at,
                max_age_ms=age,
                timeout_s=timeout_s,
            )
        except TooManyPendingError as error:
            raise ServiceValidationError(
                translation_domain=DOMAIN,
                translation_key="too_many_pending",
                translation_placeholders={"robot_id": robot.robot_id, "limit": str(error.limit)},
            ) from error
        command.requested_timeout_s = requested_timeout_s
        return command, raw

    async def async_publish_command(self, command: PendingCommand, raw: bytes) -> None:
        """Save the Store, then publish ``raw`` with QoS 1, no retain and the contract's expiry.

        ``raw`` is the encoded command :meth:`_create` built for it. A
        ``ServiceValidationError`` (HA's MQTT session is not v5) opens
        ``mqtt_v5_required`` and the publish is retried once without the expiry. A
        ``HomeAssistantError`` (for example the broker is unreachable) is surfaced as
        ``publish_failed`` while the command stays ``sent`` with its normal ack
        timeout, because paho keeps the QoS 1 message and resends it on reconnect.
        """
        topic = cmd_topic(self.prefix, command.robot_id)
        await self.store.async_save()
        expiry = max(1, math.ceil(command.max_age_ms / 1000))
        failure: HomeAssistantError | None = None
        try:
            await self._async_publish(topic, raw, qos=1, retain=False, expiry=expiry)
        except HomeAssistantError as error:
            failure = error
        self.registry.mark_sent(command.task_id, publish_failed=failure is not None)
        if failure is not None:
            self.counters["publish_errors"] += 1
            _LOGGER.warning(
                "Publishing command %s to %s failed: %s", command.task_id, topic, failure
            )
            raise HomeAssistantError(
                translation_domain=DOMAIN,
                translation_key="publish_failed",
                translation_placeholders={"robot_id": command.robot_id, "error": str(failure)},
            ) from failure

    async def _async_publish(
        self, topic: str, payload: bytes, *, qos: int, retain: bool, expiry: int | None
    ) -> None:
        """Publish through HA's MQTT client, falling back to no expiry on a non-v5 session."""
        try:
            await mqtt.async_publish(
                self.hass, topic, payload, qos=qos, retain=retain, message_expiry_interval=expiry
            )
        except ServiceValidationError as error:
            if expiry is None:
                raise
            _LOGGER.warning(
                "MQTT session is not v5; publishing %s without expiry (%s)", topic, error
            )
            self._open_issue(ISSUE_MQTT_V5_REQUIRED, None, {"topic": topic})
            await mqtt.async_publish(self.hass, topic, payload, qos=qos, retain=retain)
            return
        if expiry is not None:
            self._close_issue(ISSUE_MQTT_V5_REQUIRED, None)

    async def async_forget_robot(self, robot_id: str) -> None:
        """Clear a robot's retained topics and drop it (device removal from the UI)."""
        self.counters["publish_errors"] += await async_clear_robot_topics(
            self.hass, self.prefix, robot_id
        )
        self._handle_tombstone(robot_id)

    def is_removable(self, robot_id: str) -> bool:
        """True when the robot is unknown or offline (device removal is allowed)."""
        robot = self.robots.get(robot_id)
        return robot is None or not robot.online

    # ----------------------------------------------------------------- views for entities and services

    def device_info(self, robot_id: str) -> DeviceInfo:
        """Device registry fields for a robot's device (identifiers alone before an announce)."""
        info = DeviceInfo(identifiers={(DOMAIN, robot_id)})
        robot = self.robots.get(robot_id)
        announce = robot.announce if robot is not None else None
        if announce is None:
            info["name"] = robot_id
            return info
        info["name"] = announce.name
        if announce.manufacturer:
            info["manufacturer"] = announce.manufacturer
        if announce.model:
            info["model"] = announce.model
        if announce.sw_version:
            info["sw_version"] = announce.sw_version
        if announce.hw_version:
            info["hw_version"] = announce.hw_version
        if announce.serial_number:
            info["serial_number"] = announce.serial_number
        if announce.suggested_area:
            info["suggested_area"] = announce.suggested_area
        return info

    def robot_available(self, robot_id: str) -> bool:
        """The availability rule: announced this run, online, MQTT connected, heartbeat alive."""
        robot = self.robots.get(robot_id)
        return (
            self.mqtt_connected
            and robot is not None
            and robot.announce is not None
            and robot.online
            and not robot.heartbeat_unavailable
        )

    def problem_reasons(self, robot_id: str) -> list[str]:
        """Why the ``problem`` binary sensor is on; empty when it is off."""
        robot = self.robots.get(robot_id)
        if robot is None:
            return []
        reasons: list[str] = []
        state = robot.state
        if state is not None and state.status is RobotStatus.ERROR:
            reasons.append("error")
        if state is not None and state.executor_health_level is HealthLevel.ERROR:
            reasons.append("executor_error")
        if robot.heartbeat_stale and robot.online:
            reasons.append("heartbeat_stale")
        if robot.duplicate_open:
            reasons.append(ISSUE_DUPLICATE_ROBOT_ID)
        return reasons

    def robot_summary(self, robot_id: str) -> dict[str, Any]:
        """The ``get_robots`` shape for one robot."""
        robot = self.robots.get(robot_id)
        if robot is None:
            return {"robot_id": robot_id}
        state = robot.state
        return {
            "robot_id": robot_id,
            "device_id": robot.device_id,
            "name": robot.name,
            "available": self.robot_available(robot_id),
            "state": None if state is None else str(state.status),
            "task": None if state is None else state.task,
            "task_id": None if state is None else state.task_id,
            "paused": None if state is None else state.paused,
            "ha_link": None if state is None else str(state.ha_link),
            "problem": bool(self.problem_reasons(robot_id)),
            "instance": robot.instance,
            "session": robot.session,
            "tasks": [spec.to_dict() for spec in robot.tasks],
        }

    def diagnostics(self) -> dict[str, Any]:
        """The hub's share of the config entry diagnostics (the secret is never included)."""
        now = self.now
        robots: dict[str, Any] = {}
        for robot_id, robot in self.robots.items():
            robots[robot_id] = {
                "announce": None if robot.announce is None else robot.announce.to_dict(),
                "device_id": robot.device_id,
                "availability": None if robot.availability is None else str(robot.availability),
                "available": self.robot_available(robot_id),
                "state": None if robot.state is None else robot.state.to_dict(),
                "session": robot.session,
                "instance": robot.instance,
                "session_change_count": robot.session_change_count,
                "heartbeat_age_s": (
                    None
                    if robot.last_heartbeat_at is None
                    else round((now - robot.last_heartbeat_at) / 1000, 1)
                ),
                "heartbeat_stale": robot.heartbeat_stale,
                "heartbeat_unavailable": robot.heartbeat_unavailable,
                "ha_clock_offset_ms": robot.ha_clock_offset_ms,
                "counters": dict(robot.counters),
                "last_ack": None if robot.last_ack is None else robot.last_ack.to_dict(),
                "last_result": None if robot.last_result is None else robot.last_result.to_dict(),
                "feedback": (
                    None
                    if robot.feedback is None
                    else {
                        "task_id": robot.feedback.id,
                        "progress": robot.feedback.progress,
                        "stage": robot.feedback.stage,
                        "message": robot.feedback.message,
                    }
                ),
                "joints": None if robot.joints is None else robot.joints.to_dict(),
                "pending": [c.task_id for c in self.registry.outstanding(robot_id)],
                "problem_reasons": self.problem_reasons(robot_id),
            }
        return {
            "options": self.options.to_dict(),
            "instance_id": self.instance_id,
            "mqtt_connected": self.mqtt_connected,
            "ready": self.ready,
            "reconciled": self.reconciled,
            "subscriptions": [list(item) for item in self.subscriptions],
            "heartbeat": {
                "seq": self.heartbeat_seq,
                "last_sent_at": self.last_heartbeat_sent_at,
                "interval_s": self.options.heartbeat_interval_s,
            },
            "robots": robots,
            "commands": [c.to_dict() for c in self.registry.recent(DIAGNOSTICS_RECENT_COMMANDS)],
            "counters": {**self.counters, **self.registry.counters},
            "seen_results": len(self.registry.seen_results),
            "issues": sorted(self._open_issues),
        }

    # ----------------------------------------------------------------- repairs

    def _open_issue(self, kind: str, robot_id: str | None, placeholders: dict[str, str]) -> None:
        iid = issue_id(kind, robot_id)
        ir.async_create_issue(
            self.hass,
            DOMAIN,
            iid,
            is_fixable=False,
            is_persistent=False,
            severity=ir.IssueSeverity.ERROR
            if kind in (ISSUE_HMAC_REJECTED, ISSUE_DUPLICATE_ROBOT_ID)
            else ir.IssueSeverity.WARNING,
            translation_key=kind,
            translation_placeholders=placeholders,
        )
        self._open_issues.add(iid)

    def _close_issue(self, kind: str, robot_id: str | None) -> None:
        iid = issue_id(kind, robot_id)
        if iid in self._open_issues:
            ir.async_delete_issue(self.hass, DOMAIN, iid)
            self._open_issues.discard(iid)


async def async_clear_robot_topics(hass: HomeAssistant, prefix: str, robot_id: str) -> int:
    """Publish a robot's tombstones: empty retained announce, state and availability.

    Returns how many of the three publishes failed; each failure is logged and
    the remaining topics are still cleared. Used by the hub for a device removed
    from the UI and by ``async_remove_config_entry_device`` when the entry is not
    loaded, so a deletion sticks on the broker either way.
    """
    failures = 0
    for topic in (
        announce_topic(prefix, robot_id),
        state_topic(prefix, robot_id),
        availability_topic(prefix, robot_id),
    ):
        try:
            await mqtt.async_publish(hass, topic, b"", qos=1, retain=True)
        except HomeAssistantError as error:
            failures += 1
            _LOGGER.warning("Could not clear %s: %s", topic, error)
    return failures


def _joint_diff(
    previous: JointInfo | None, current: JointInfo | None
) -> tuple[tuple[int, ...], tuple[int, ...]]:
    """Which joint sensor indices a changed announce block adds and removes.

    A joint sensor is identified by its index and configured by the name, unit
    and limits at that index, so an index whose configuration changed is listed
    in both tuples: the sensor platform removes the old entity and adds a new
    one with the new name and unit.
    """
    added: list[int] = []
    removed: list[int] = []
    before = 0 if previous is None else previous.count
    after = 0 if current is None else current.count
    for index in range(max(before, after)):
        old = previous.joint_key(index) if previous is not None and index < before else None
        new = current.joint_key(index) if current is not None and index < after else None
        if old == new:
            continue
        if old is not None:
            removed.append(index)
        if new is not None:
            added.append(index)
    return tuple(added), tuple(removed)


def _payload_bytes(msg: ReceiveMessage) -> bytes:
    """The raw payload; subscriptions use ``encoding=None`` so this is normally bytes already."""
    payload = msg.payload
    if isinstance(payload, bytes):
        return payload
    if isinstance(payload, str):
        return payload.encode("utf-8")
    return bytes(payload)
