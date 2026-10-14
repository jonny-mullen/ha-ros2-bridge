# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Dataclasses of the ROS 2 Bridge integration.

The frozen classes (``TaskSpec``, ``JointInfo``, ``Announce``, ``RobotState``,
``Heartbeat``, ``Ack``, ``Feedback``, ``Result``, ``Joints``) are what
``protocol.py`` returns for a decoded inbound payload. ``Robot`` is the hub's mutable runtime view of one
robot, ``LastResult`` what the ``last_result`` sensor shows, and
``PendingCommand`` one tracked command id with its state machine fields and
its Store form. Nothing here imports Home Assistant.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any

from .const import (
    JOINT_UNIT_SYMBOLS,
    OUTSTANDING_STATES,
    Availability,
    CommandState,
    CommandType,
    HaLink,
    HealthLevel,
    JointUnit,
    ResultStatus,
    RobotStatus,
    TaskKind,
    TaskOutcome,
    WatchdogMode,
)

# --------------------------------------------------------------------------- inbound payloads


@dataclass(frozen=True, slots=True)
class TaskSpec:
    """One entry of an announced task catalogue."""

    name: str
    label: str
    kind: TaskKind
    timeout_s: float
    params: Mapping[str, Any] | None = None
    description: str = ""

    @property
    def required_params(self) -> tuple[str, ...]:
        """Names listed in the params schema's ``required``."""
        if self.params is None:
            return ()
        required = self.params.get("required")
        if not isinstance(required, list):
            return ()
        return tuple(name for name in required if isinstance(name, str))

    @property
    def needs_params(self) -> bool:
        """True when the task cannot be started without parameters (so it gets no button)."""
        return bool(self.required_params)

    def to_dict(self) -> dict[str, Any]:
        """The task as a plain dict (service responses, diagnostics)."""
        return {
            "name": self.name,
            "label": self.label,
            "kind": str(self.kind),
            "timeout_s": self.timeout_s,
            "params": None if self.params is None else dict(self.params),
            "description": self.description,
            "needs_params": self.needs_params,
        }


@dataclass(frozen=True, slots=True)
class JointInfo:
    """The optional ``joints`` block of an announce (0.2.0): what the joints topic carries.

    ``names`` is the publication order of the positions; ``limits`` is ``None``
    when the executor knows none, otherwise one ``(minimum, maximum)`` pair per
    name in ``unit``. Both lengths agree by construction (``protocol.py``
    ignores a block whose lengths differ).
    """

    unit: JointUnit
    names: tuple[str, ...]
    limits: tuple[tuple[float, float], ...] | None
    rate_hz: float

    @property
    def count(self) -> int:
        return len(self.names)

    @property
    def unit_symbol(self) -> str:
        """The unit of measurement the joint sensors show (``°`` or ``rad``)."""
        return JOINT_UNIT_SYMBOLS[self.unit]

    def limit(self, index: int) -> tuple[float, float] | None:
        """The ``(minimum, maximum)`` of joint ``index``, or ``None`` when none was announced."""
        return None if self.limits is None else self.limits[index]

    def joint_key(self, index: int) -> tuple[str, str, tuple[float, float] | None]:
        """What identifies one joint sensor's configuration: name, unit and limits."""
        return (self.names[index], str(self.unit), self.limit(index))

    def to_dict(self) -> dict[str, Any]:
        """The block as a plain dict (diagnostics)."""
        return {
            "unit": str(self.unit),
            "names": list(self.names),
            "limits": None if self.limits is None else [list(pair) for pair in self.limits],
            "rate_hz": self.rate_hz,
        }


@dataclass(frozen=True, slots=True)
class Announce:
    """``{prefix}/{robot_id}/announce``: identity plus the task catalogue.

    ``joints`` is the decoded ``joints`` block, or ``None`` when the announce
    has none or when the block was ignored because its ``limits`` and ``names``
    lengths differ; ``joints_ignored`` then says why.
    """

    robot_id: str
    name: str
    bridge_version: str
    session: str
    instance: str
    cancel: bool
    feedback: bool
    hmac_required: bool
    watchdog_mode: WatchdogMode
    heartbeat_interval_s: float
    tasks: tuple[TaskSpec, ...]
    ts: int
    manufacturer: str | None = None
    model: str | None = None
    sw_version: str | None = None
    hw_version: str | None = None
    serial_number: str | None = None
    suggested_area: str | None = None
    executor_name: str | None = None
    executor_version: str | None = None
    joints: JointInfo | None = None
    joints_ignored: str | None = None

    def task(self, name: str) -> TaskSpec | None:
        """The announced task called ``name``, or ``None``."""
        for spec in self.tasks:
            if spec.name == name:
                return spec
        return None

    def task_by_label(self, label: str) -> TaskSpec | None:
        """The announced task whose label matches ``label`` case-insensitively, or ``None``."""
        folded = label.casefold().strip()
        for spec in self.tasks:
            if spec.label.casefold().strip() == folded:
                return spec
        return None

    @property
    def task_names(self) -> tuple[str, ...]:
        return tuple(spec.name for spec in self.tasks)

    def to_dict(self) -> dict[str, Any]:
        """The announce as a plain dict (diagnostics)."""
        return {
            "robot_id": self.robot_id,
            "name": self.name,
            "manufacturer": self.manufacturer,
            "model": self.model,
            "sw_version": self.sw_version,
            "hw_version": self.hw_version,
            "serial_number": self.serial_number,
            "suggested_area": self.suggested_area,
            "bridge_version": self.bridge_version,
            "executor": (
                None
                if self.executor_name is None
                else {"name": self.executor_name, "version": self.executor_version}
            ),
            "session": self.session,
            "instance": self.instance,
            "capabilities": {
                "cancel": self.cancel,
                "feedback": self.feedback,
                "hmac_required": self.hmac_required,
            },
            "watchdog_mode": str(self.watchdog_mode),
            "heartbeat_interval_s": self.heartbeat_interval_s,
            "tasks": [spec.to_dict() for spec in self.tasks],
            "joints": None if self.joints is None else self.joints.to_dict(),
            "joints_ignored": self.joints_ignored,
            "ts": self.ts,
        }


@dataclass(frozen=True, slots=True)
class RobotState:
    """``{prefix}/{robot_id}/state``.

    The wire field ``state`` is called ``status`` here so it does not shadow the class.
    """

    robot_id: str
    status: RobotStatus
    task_id: str | None
    task: str | None
    since: int
    paused: bool
    inhibited: bool
    ha_link: HaLink
    watchdog_mode: WatchdogMode
    executor_health_level: HealthLevel
    executor_health_message: str
    error_code: str | None
    error_reason: str | None
    session: str
    instance: str
    seq: int
    ts: int

    def to_dict(self) -> dict[str, Any]:
        """The state as a plain dict (diagnostics, attributes)."""
        return {
            "state": str(self.status),
            "task_id": self.task_id,
            "task": self.task,
            "since": self.since,
            "paused": self.paused,
            "inhibited": self.inhibited,
            "ha_link": str(self.ha_link),
            "watchdog_mode": str(self.watchdog_mode),
            "executor_health": {
                "level": str(self.executor_health_level),
                "message": self.executor_health_message,
            },
            "error_code": self.error_code,
            "error_reason": self.error_reason,
            "session": self.session,
            "instance": self.instance,
            "seq": self.seq,
            "ts": self.ts,
        }


@dataclass(frozen=True, slots=True)
class Heartbeat:
    """``{prefix}/{robot_id}/heartbeat`` (ROS to HA)."""

    robot_id: str
    session: str
    instance: str
    seq: int
    ts: int
    uptime_s: float
    counters: Mapping[str, int]
    ha_clock_offset_ms: int | None = None


@dataclass(frozen=True, slots=True)
class Ack:
    """``{prefix}/{robot_id}/task/{id}/ack``."""

    id: str
    robot_id: str
    type: CommandType
    task: str | None
    accepted: bool
    code: str
    reason: str
    received_at: int
    clock_skew_ms: int | None
    session: str
    ts: int

    def to_dict(self) -> dict[str, Any]:
        """The ack as a plain dict (diagnostics)."""
        return {
            "id": self.id,
            "type": str(self.type),
            "task": self.task,
            "accepted": self.accepted,
            "code": self.code,
            "reason": self.reason,
            "received_at": self.received_at,
            "clock_skew_ms": self.clock_skew_ms,
            "session": self.session,
            "ts": self.ts,
        }


@dataclass(frozen=True, slots=True)
class Feedback:
    """``{prefix}/{robot_id}/task/{id}/feedback``."""

    id: str
    robot_id: str
    progress: float | None
    stage: str
    message: str
    session: str
    ts: int


@dataclass(frozen=True, slots=True)
class Result:
    """``{prefix}/{robot_id}/task/{id}/result``: exactly one per accepted task command."""

    id: str
    robot_id: str
    task: str
    kind: TaskKind
    status: ResultStatus
    goal_status: int
    code: str
    reason: str
    data: Mapping[str, Any]
    started_at: int
    finished_at: int
    session: str
    ts: int

    @property
    def duration_ms(self) -> int:
        return max(0, self.finished_at - self.started_at)

    def to_dict(self) -> dict[str, Any]:
        """The result as a plain dict (diagnostics)."""
        return {
            "id": self.id,
            "task": self.task,
            "kind": str(self.kind),
            "status": str(self.status),
            "goal_status": self.goal_status,
            "code": self.code,
            "reason": self.reason,
            "data": dict(self.data),
            "started_at": self.started_at,
            "finished_at": self.finished_at,
            "duration_ms": self.duration_ms,
            "session": self.session,
            "ts": self.ts,
        }


@dataclass(frozen=True, slots=True)
class Joints:
    """``{prefix}/{robot_id}/joints`` (0.2.0): one set of joint positions.

    ``positions`` follows the order of the announced ``JointInfo.names``;
    ``velocities`` is ``None`` or one value per position.
    """

    robot_id: str
    positions: tuple[float, ...]
    velocities: tuple[float, ...] | None
    session: str
    instance: str
    seq: int
    ts: int

    def to_dict(self) -> dict[str, Any]:
        """The payload as a plain dict (diagnostics)."""
        return {
            "positions": list(self.positions),
            "velocities": None if self.velocities is None else list(self.velocities),
            "session": self.session,
            "instance": self.instance,
            "seq": self.seq,
            "ts": self.ts,
        }


# --------------------------------------------------------------------------- runtime views


@dataclass(slots=True)
class LastResult:
    """What the ``last_result`` sensor shows: the latest outcome of a motion task."""

    outcome: TaskOutcome
    task_id: str
    task: str | None
    code: str | None
    reason: str | None
    data: Mapping[str, Any]
    finished_at: int | None
    duration_ms: int | None
    synthetic: bool

    def to_dict(self) -> dict[str, Any]:
        return {
            "outcome": str(self.outcome),
            "task_id": self.task_id,
            "task": self.task,
            "code": self.code,
            "reason": self.reason,
            "data": dict(self.data),
            "finished_at": self.finished_at,
            "duration_ms": self.duration_ms,
            "synthetic": self.synthetic,
        }


@dataclass(slots=True)
class Robot:
    """The hub's runtime view of one announced robot.

    Times suffixed ``_at`` are hub clock values in epoch milliseconds; the wire
    timestamps stay inside the decoded payloads.
    """

    robot_id: str
    announce: Announce | None = None
    device_id: str | None = None
    #: ``None`` until the retained availability has arrived this run.
    availability: Availability | None = None
    state: RobotState | None = None
    session: str | None = None
    instance: str | None = None
    first_seen_at: int | None = None
    last_heartbeat: Heartbeat | None = None
    last_heartbeat_at: int | None = None
    heartbeat_stale: bool = False
    heartbeat_unavailable: bool = False
    counters: dict[str, int] = field(default_factory=dict)
    ha_clock_offset_ms: int | None = None
    feedback: Feedback | None = None
    #: The latest accepted joints payload; ``None`` until one arrives for the announced block.
    joints: Joints | None = None
    #: Per joint, whether the latest payload showed movement (``BridgeHub._handle_joints``);
    #: ``None`` with no payload. Housekeeping clears every flag once ``joints_moving_until``
    #: (a hub clock value, the ``joint_moving_hold_s`` option after the payload) has passed
    #: without a newer payload.
    joint_moving: tuple[bool, ...] | None = None
    joints_moving_until: int | None = None
    last_ack: Ack | None = None
    last_result: LastResult | None = None
    #: Hub clock values of the session changes seen in the duplicate-detection window.
    session_changes: list[int] = field(default_factory=list)
    session_change_count: int = 0
    #: Last ``instance`` seen per value, for the differing-instance heuristic.
    instances_seen_at: dict[str, int] = field(default_factory=dict)
    duplicate_evidence_at: int | None = None
    duplicate_open: bool = False
    clock_skew_open: bool = False
    hmac_rejected_open: bool = False

    @property
    def name(self) -> str:
        return self.announce.name if self.announce is not None else self.robot_id

    @property
    def online(self) -> bool:
        return self.availability is Availability.ONLINE

    @property
    def status(self) -> RobotStatus | None:
        return None if self.state is None else self.state.status

    @property
    def heartbeat_interval_s(self) -> float:
        """The announced heartbeat interval; 0 means no staleness tracking."""
        return 0.0 if self.announce is None else float(self.announce.heartbeat_interval_s)

    def task(self, name: str) -> TaskSpec | None:
        return None if self.announce is None else self.announce.task(name)

    @property
    def tasks(self) -> tuple[TaskSpec, ...]:
        return () if self.announce is None else self.announce.tasks

    @property
    def joint_info(self) -> JointInfo | None:
        """The announced joints block, or ``None`` for a robot without joint telemetry."""
        return None if self.announce is None else self.announce.joints


# --------------------------------------------------------------------------- commands


@dataclass(slots=True)
class PendingCommand:
    """One command id the registry tracks, from creation to its single terminal outcome.

    ``timeout_s`` is the effective task timeout the deadline is planned on (the
    command's own value or the announced one); it is ``None`` for a cancel. The
    deadlines and ``*_at`` fields are registry clock values in epoch milliseconds.
    """

    task_id: str
    robot_id: str
    type: CommandType
    task: str | None
    kind: TaskKind | None
    params: dict[str, Any]
    target_id: str | None
    session: str | None
    source: dict[str, Any]
    issued_at: int
    max_age_ms: int
    timeout_s: float | None
    created_at: int
    state: CommandState = CommandState.CREATED
    sent_at: int | None = None
    ack_deadline: int | None = None
    deadline: int | None = None
    paused_at: int | None = None
    ack: Ack | None = None
    result: Result | None = None
    outcome: TaskOutcome | None = None
    code: str | None = None
    reason: str | None = None
    synthetic: bool = False
    #: True once ``task_result`` (or, for a cancel, its terminal ``task_ack``) has fired.
    outcome_fired: bool = False
    finished_at: int | None = None
    #: True when the last publish attempt raised; the command still counts as sent.
    publish_failed: bool = False
    #: The caller's own ``timeout_s`` (None means the announced value travels as null).
    requested_timeout_s: float | None = None

    @property
    def is_task(self) -> bool:
        return self.type is CommandType.TASK

    @property
    def is_cancel(self) -> bool:
        return self.type is CommandType.CANCEL

    @property
    def is_outstanding(self) -> bool:
        return self.state in OUTSTANDING_STATES

    @property
    def is_terminal(self) -> bool:
        return self.state not in OUTSTANDING_STATES

    @property
    def accepted(self) -> bool | None:
        """``True``/``False`` once an ack has been seen, else ``None``."""
        return None if self.ack is None else self.ack.accepted

    def to_store(self) -> dict[str, Any]:
        """The Store form (``outstanding[<task_id>]`` in ``ros2_bridge.commands``)."""
        return {
            "robot_id": self.robot_id,
            "task": self.task,
            "kind": None if self.kind is None else str(self.kind),
            "type": str(self.type),
            "session": self.session,
            "sent_at": self.sent_at,
            "deadline": self.deadline,
            "paused_at": self.paused_at,
            "state": str(self.state),
            "source": dict(self.source),
            "target_id": self.target_id,
            "timeout_s": self.timeout_s,
            "ack_deadline": self.ack_deadline,
            "issued_at": self.issued_at,
            "max_age_ms": self.max_age_ms,
            "created_at": self.created_at,
            "outcome_fired": self.outcome_fired,
        }

    @classmethod
    def from_store(cls, task_id: str, data: Mapping[str, Any]) -> PendingCommand:
        """Rebuild a command from its Store form; missing optional keys take defaults."""
        kind_raw = data.get("kind")
        timeout_raw = data.get("timeout_s")
        return cls(
            task_id=task_id,
            robot_id=str(data["robot_id"]),
            type=CommandType(str(data.get("type", CommandType.TASK))),
            task=None if data.get("task") is None else str(data["task"]),
            kind=None if kind_raw is None else TaskKind(str(kind_raw)),
            params={},
            target_id=None if data.get("target_id") is None else str(data["target_id"]),
            session=None if data.get("session") is None else str(data["session"]),
            source=dict(data.get("source") or {}),
            issued_at=int(data.get("issued_at") or 0),
            max_age_ms=int(data.get("max_age_ms") or 0),
            timeout_s=None if timeout_raw is None else float(timeout_raw),
            created_at=int(data.get("created_at") or data.get("sent_at") or 0),
            state=CommandState(str(data.get("state", CommandState.SENT))),
            sent_at=None if data.get("sent_at") is None else int(data["sent_at"]),
            ack_deadline=(None if data.get("ack_deadline") is None else int(data["ack_deadline"])),
            deadline=None if data.get("deadline") is None else int(data["deadline"]),
            paused_at=None if data.get("paused_at") is None else int(data["paused_at"]),
            outcome_fired=bool(data.get("outcome_fired", False)),
        )

    def to_dict(self) -> dict[str, Any]:
        """A plain dict for service responses and diagnostics."""
        return {
            "task_id": self.task_id,
            "robot_id": self.robot_id,
            "type": str(self.type),
            "task": self.task,
            "kind": None if self.kind is None else str(self.kind),
            "target_id": self.target_id,
            "state": str(self.state),
            "session": self.session,
            "sent_at": self.sent_at,
            "ack_deadline": self.ack_deadline,
            "deadline": self.deadline,
            "paused_at": self.paused_at,
            "accepted": self.accepted,
            "ack_code": None if self.ack is None else self.ack.code,
            "ack_reason": None if self.ack is None else self.ack.reason,
            "clock_skew_ms": None if self.ack is None else self.ack.clock_skew_ms,
            "outcome": None if self.outcome is None else str(self.outcome),
            "code": self.code,
            "reason": self.reason,
            "synthetic": self.synthetic,
            "finished_at": self.finished_at,
            "publish_failed": self.publish_failed,
            "source": dict(self.source),
        }
