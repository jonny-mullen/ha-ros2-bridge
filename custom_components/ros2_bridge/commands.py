# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The command registry: every command id ends in exactly one named outcome.

``CommandRegistry`` owns the state machine of the contract's HA side::

    created -> sent -> (running | rejected | timeout_no_ack)
                    -> (succeeded | cancelled | aborted | timeout_no_result | lost)

plus ``timeout_no_ack -> running`` on a late accepted ack, ``sent -> terminal``
on a result that overtakes its ack, and for cancels ``sent -> done | rejected |
timeout_no_ack``. It keeps the ack and result deadlines (the result deadline is
suspended while the robot reports the task paused), the per-robot pending cap,
the age-bounded ``seen_results`` set that makes the ``task_result`` bus event
fire exactly once per id across retained replays and HA restarts, and the
reconciliation rule applied after the retained replay burst and after a robot's
``session`` changes.

Time is injected as a clock callable returning epoch milliseconds, events are
handed to a callback as ``(event_name, data)``, and persistence is requested
through a callback, so the module imports nothing from Home Assistant and is
unit-tested without ``hass``. The hub wires the callbacks to the bus, the
dispatcher and the Store.
"""

from __future__ import annotations

import asyncio
import logging
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from enum import StrEnum
from typing import Any

from .const import (
    CODE_HA_RESTARTED,
    CODE_NO_ACK,
    CODE_NO_RESULT,
    CODE_NODE_RESTARTED,
    DEFAULT_ACK_EXPIRY_S,
    DEFAULT_ACK_TIMEOUT_S,
    DEFAULT_MAX_PENDING_PER_ROBOT,
    DEFAULT_RESULT_EXPIRY_S,
    DEFAULT_RESULT_TIMEOUT_MARGIN_S,
    EVENT_TASK_ACK,
    EVENT_TASK_LATE_RESULT,
    EVENT_TASK_RESULT,
    MAX_TERMINAL_COMMANDS,
    CommandState,
    CommandType,
    TaskKind,
    TaskOutcome,
)
from .models import Ack, PendingCommand, Result

_LOGGER = logging.getLogger(__name__)

#: Returns the current time as epoch milliseconds.
Clock = Callable[[], int]
#: Receives ``(event_name, data)`` for every bus event the registry decides to fire.
EventCallback = Callable[[str, dict[str, Any]], None]
#: Receives ``urgent``: True after a transition that must reach the Store at once,
#: False for housekeeping-only changes that may be written with a delay.
PersistCallback = Callable[[bool], None]

_TERMINAL_FOR_OUTCOME: dict[TaskOutcome, CommandState] = {
    TaskOutcome.SUCCEEDED: CommandState.SUCCEEDED,
    TaskOutcome.CANCELLED: CommandState.CANCELLED,
    TaskOutcome.ABORTED: CommandState.ABORTED,
}


class TooManyPendingError(Exception):
    """Raised by :meth:`CommandRegistry.create` when a robot's pending cap is reached."""

    def __init__(self, robot_id: str, limit: int) -> None:
        super().__init__(f"{robot_id} already has {limit} commands pending")
        self.robot_id = robot_id
        self.limit = limit


class AckDisposition(StrEnum):
    """What :meth:`CommandRegistry.handle_ack` did with an ack."""

    ACCEPTED = "accepted"
    REJECTED = "rejected"
    LATE_ACCEPTED = "late_accepted"
    CANCEL_DONE = "cancel_done"
    IGNORED = "ignored"
    UNMATCHED = "unmatched"


class ResultDisposition(StrEnum):
    """What :meth:`CommandRegistry.handle_result` did with a result."""

    APPLIED = "applied"
    LATE = "late"
    DUPLICATE = "duplicate"
    UNMATCHED = "unmatched"


@dataclass(frozen=True, slots=True)
class RegistrySettings:
    """The options the registry needs; replaced whole on an options reload."""

    ack_timeout_s: float = DEFAULT_ACK_TIMEOUT_S
    result_timeout_margin_s: float = DEFAULT_RESULT_TIMEOUT_MARGIN_S
    max_pending_per_robot: int = DEFAULT_MAX_PENDING_PER_ROBOT
    #: The node's ``reply.result_expiry_s``; bounds ``seen_results`` by age together with the margin.
    result_expiry_s: float = DEFAULT_RESULT_EXPIRY_S
    #: The node's ``reply.ack_expiry_s``; how long a cancel waits for a late ack.
    ack_expiry_s: float = DEFAULT_ACK_EXPIRY_S

    @property
    def retention_ms(self) -> int:
        """How long a seen result id and a terminal command are remembered."""
        return int((self.result_expiry_s + self.result_timeout_margin_s) * 1000)


@dataclass(frozen=True, slots=True)
class RobotSnapshot:
    """What reconciliation needs to know about one announced robot."""

    session: str | None
    paused_task_id: str | None = None


def ack_event_data(
    command: PendingCommand, ack: Ack | None, *, late: bool, synthetic: bool = False
) -> dict[str, Any]:
    """The ``ros2_bridge_task_ack`` payload; ``device_id`` is filled in by the hub."""
    return {
        "robot_id": command.robot_id,
        "device_id": None,
        "task_id": command.task_id,
        "type": str(command.type),
        "task": command.task,
        "kind": None if command.kind is None else str(command.kind),
        "accepted": False if ack is None else ack.accepted,
        "code": command.code if ack is None else ack.code,
        "reason": command.reason if ack is None else ack.reason,
        "clock_skew_ms": None if ack is None else ack.clock_skew_ms,
        "late": late,
        "synthetic": synthetic,
        "source": dict(command.source),
    }


def result_event_data(
    command: PendingCommand | None,
    result: Result | None,
    *,
    outcome: TaskOutcome,
    code: str | None,
    reason: str | None,
    synthetic: bool,
    replayed: bool,
    finished_at: int | None,
    robot_id: str | None = None,
    task_id: str | None = None,
) -> dict[str, Any]:
    """The ``ros2_bridge_task_result`` and ``_late_result`` payload (``device_id`` set by the hub)."""
    started_at: int | None
    if result is not None:
        started_at = result.started_at
    elif command is not None:
        started_at = command.sent_at
    else:
        started_at = None
    duration_ms: int | None = None
    if started_at is not None and finished_at is not None:
        duration_ms = max(0, finished_at - started_at)
    kind: TaskKind | None
    task: str | None
    if result is not None:
        kind, task = result.kind, result.task
    elif command is not None:
        kind, task = command.kind, command.task
    else:
        kind, task = None, None
    return {
        "robot_id": command.robot_id if command is not None else robot_id,
        "device_id": None,
        "task_id": command.task_id if command is not None else task_id,
        "task": task,
        "kind": None if kind is None else str(kind),
        "status": str(outcome),
        "goal_status": None if result is None else result.goal_status,
        "code": code,
        "reason": reason,
        "data": {} if result is None else dict(result.data),
        "started_at": started_at,
        "finished_at": finished_at,
        "duration_ms": duration_ms,
        "synthetic": synthetic,
        "replayed": replayed,
        "source": {} if command is None else dict(command.source),
    }


class CommandRegistry:
    """The command state machine, deadlines, dedupe and reconciliation (see the module docstring).

    Public attributes: ``commands`` (every tracked command by id, outstanding and recent
    terminal ones), ``seen_results`` (result id to ``finished_at``), ``counters`` and
    ``deadlines_suspended`` (True from construction until the first ``reconcile_all``,
    so an HA restart never times everything out before the retained replay is in).
    """

    def __init__(
        self,
        *,
        clock: Clock,
        settings: RegistrySettings | None = None,
        on_event: EventCallback | None = None,
        on_persist: PersistCallback | None = None,
    ) -> None:
        self._clock = clock
        self.settings = settings or RegistrySettings()
        self._on_event = on_event
        self._on_persist = on_persist
        self.commands: dict[str, PendingCommand] = {}
        self.seen_results: dict[str, int] = {}
        self.counters: dict[str, int] = {
            "unmatched_replies": 0,
            "replayed_results_ignored": 0,
            "late_results": 0,
            "duplicate_acks": 0,
            "synthesised": 0,
        }
        self.deadlines_suspended = True
        self._ack_waiters: dict[str, list[asyncio.Future[PendingCommand]]] = {}
        self._result_waiters: dict[str, list[asyncio.Future[PendingCommand]]] = {}

    # ----------------------------------------------------------------- wiring

    @property
    def now(self) -> int:
        """The registry clock, epoch milliseconds."""
        return int(self._clock())

    def set_callbacks(
        self, *, on_event: EventCallback | None, on_persist: PersistCallback | None
    ) -> None:
        """Re-point the callbacks; the hub does this when it adopts a parked registry."""
        self._on_event = on_event
        self._on_persist = on_persist

    def update_settings(self, settings: RegistrySettings) -> None:
        """Replace the settings; only commands created afterwards use the new timings."""
        self.settings = settings

    def _emit(self, name: str, data: dict[str, Any]) -> None:
        if self._on_event is not None:
            self._on_event(name, data)

    def _persist(self, urgent: bool) -> None:
        if self._on_persist is not None:
            self._on_persist(urgent)

    # ----------------------------------------------------------------- persistence

    def export(self) -> dict[str, Any]:
        """The registry's share of the Store document: outstanding commands and seen results."""
        return {
            "outstanding": {
                task_id: command.to_store()
                for task_id, command in self.commands.items()
                if command.is_outstanding
            },
            "seen_results": [[task_id, ts] for task_id, ts in self.seen_results.items()],
        }

    def load(self, data: Mapping[str, Any] | None) -> None:
        """Replace the outstanding commands and seen results with a Store document's."""
        self.commands = {}
        self.seen_results = {}
        if not data:
            return
        outstanding = data.get("outstanding")
        if isinstance(outstanding, Mapping):
            for task_id, stored in outstanding.items():
                if not isinstance(task_id, str) or not isinstance(stored, Mapping):
                    continue
                try:
                    command = PendingCommand.from_store(task_id, stored)
                except (KeyError, ValueError, TypeError) as error:
                    _LOGGER.warning("Dropping unreadable stored command %s: %s", task_id, error)
                    continue
                if command.is_outstanding:
                    self.commands[task_id] = command
        seen = data.get("seen_results")
        if isinstance(seen, list):
            for item in seen:
                if (
                    isinstance(item, (list, tuple))
                    and len(item) == 2
                    and isinstance(item[0], str)
                    and isinstance(item[1], int)
                ):
                    self.seen_results[item[0]] = item[1]

    # ----------------------------------------------------------------- queries

    def get(self, task_id: str) -> PendingCommand | None:
        return self.commands.get(task_id)

    def outstanding(self, robot_id: str | None = None) -> list[PendingCommand]:
        """Outstanding commands, oldest first, optionally for one robot."""
        return [
            command
            for command in self.commands.values()
            if command.is_outstanding and (robot_id is None or command.robot_id == robot_id)
        ]

    def pending_count(self, robot_id: str) -> int:
        return len(self.outstanding(robot_id))

    def recent(self, limit: int = 50) -> list[PendingCommand]:
        """The most recently created commands, newest first."""
        return list(self.commands.values())[-limit:][::-1]

    def active_motion(self, robot_id: str) -> PendingCommand | None:
        """The newest running motion task of a robot, if any."""
        for command in reversed(self.commands.values()):
            if (
                command.robot_id == robot_id
                and command.is_task
                and command.kind is TaskKind.MOTION
                and command.state is CommandState.RUNNING
            ):
                return command
        return None

    # ----------------------------------------------------------------- creation and sending

    def create(
        self,
        *,
        task_id: str,
        robot_id: str,
        type: CommandType,
        task: str | None,
        kind: TaskKind | None,
        params: Mapping[str, Any] | None,
        target_id: str | None,
        session: str | None,
        source: Mapping[str, Any],
        issued_at: int,
        max_age_ms: int,
        timeout_s: float | None,
    ) -> PendingCommand:
        """Register a new command in ``created``; raises ``TooManyPendingError`` at the cap.

        The caller saves the Store before publishing and calls :meth:`mark_sent`
        when the publish returns; ``create`` itself requests no persistence.
        """
        if task_id in self.commands:
            raise ValueError(f"command id {task_id} is already tracked")
        limit = self.settings.max_pending_per_robot
        if limit > 0 and self.pending_count(robot_id) >= limit:
            raise TooManyPendingError(robot_id, limit)
        command = PendingCommand(
            task_id=task_id,
            robot_id=robot_id,
            type=type,
            task=task,
            kind=kind,
            params=dict(params or {}),
            target_id=target_id,
            session=session,
            source=dict(source),
            issued_at=issued_at,
            max_age_ms=max_age_ms,
            timeout_s=timeout_s,
            created_at=self.now,
        )
        self.commands[task_id] = command
        self._prune_terminal()
        return command

    def mark_sent(self, task_id: str, *, publish_failed: bool = False) -> PendingCommand:
        """Start the ack timer and plan the result deadline once the publish has returned.

        If a reply already overtook the publish (an ack or result arrived while
        ``async_publish`` waited for its PUBACK) the state is left as the reply set
        it; only ``sent_at`` and the deadlines are filled in.
        """
        command = self._require(task_id)
        now = self.now
        command.sent_at = now
        command.publish_failed = publish_failed
        command.ack_deadline = now + int(self.settings.ack_timeout_s * 1000)
        if command.is_task and command.timeout_s is not None:
            command.deadline = (
                now
                + int(command.timeout_s * 1000)
                + int(self.settings.result_timeout_margin_s * 1000)
            )
        else:
            command.deadline = now + int(self.settings.ack_expiry_s * 1000)
        if command.state is CommandState.CREATED:
            command.state = CommandState.SENT
        self._persist(True)
        return command

    # ----------------------------------------------------------------- inbound replies

    def handle_ack(self, ack: Ack, *, retained: bool) -> AckDisposition:
        """Apply an ack by id; see the module docstring for the transitions.

        Retained acks move the state but fire no event, except a late accepted ack
        that re-opens ``timeout_no_ack``, which always fires ``task_ack`` with ``late``.
        """
        command = self.commands.get(ack.id)
        if command is None:
            self.counters["unmatched_replies"] += 1
            return AckDisposition.UNMATCHED
        now = self.now

        if command.state in (CommandState.CREATED, CommandState.SENT):
            command.ack = ack
            if ack.session:
                command.session = ack.session
            if ack.accepted:
                if command.is_cancel:
                    self._finish(
                        command,
                        CommandState.DONE,
                        outcome=None,
                        code=ack.code,
                        reason=ack.reason,
                        synthetic=False,
                        finished_at=now,
                    )
                    if not retained:
                        self._emit(EVENT_TASK_ACK, ack_event_data(command, ack, late=False))
                    command.outcome_fired = True
                    self._resolve_waiters(command)
                    self._persist(True)
                    return AckDisposition.CANCEL_DONE
                command.state = CommandState.RUNNING
                if not retained:
                    self._emit(EVENT_TASK_ACK, ack_event_data(command, ack, late=False))
                self._resolve_waiters(command)
                self._persist(True)
                return AckDisposition.ACCEPTED
            self._finish(
                command,
                CommandState.REJECTED,
                outcome=TaskOutcome.REJECTED,
                code=ack.code,
                reason=ack.reason,
                synthetic=False,
                finished_at=now,
            )
            if not retained:
                self._emit(EVENT_TASK_ACK, ack_event_data(command, ack, late=False))
                if command.is_task and not command.outcome_fired:
                    self._emit(
                        EVENT_TASK_RESULT,
                        result_event_data(
                            command,
                            None,
                            outcome=TaskOutcome.REJECTED,
                            code=ack.code,
                            reason=ack.reason,
                            synthetic=False,
                            replayed=False,
                            finished_at=now,
                        ),
                    )
            command.outcome_fired = True
            self._resolve_waiters(command)
            self._persist(True)
            return AckDisposition.REJECTED

        if command.state is CommandState.TIMEOUT_NO_ACK:
            if not ack.accepted:
                command.ack = ack
                return AckDisposition.IGNORED
            command.ack = ack
            if ack.session:
                command.session = ack.session
            if command.is_cancel:
                command.state = CommandState.DONE
                command.code = ack.code
                command.reason = ack.reason
                command.synthetic = False
            else:
                command.state = CommandState.RUNNING
                command.finished_at = None
            self._emit(EVENT_TASK_ACK, ack_event_data(command, ack, late=True))
            self._resolve_waiters(command)
            self._persist(True)
            return AckDisposition.LATE_ACCEPTED

        self.counters["duplicate_acks"] += 1
        return AckDisposition.IGNORED

    def handle_result(self, result: Result, *, retained: bool) -> ResultDisposition:
        """Apply a result by id: dedupe by ``seen_results``, then terminate or report late."""
        if result.id in self.seen_results:
            self.counters["replayed_results_ignored"] += 1
            return ResultDisposition.DUPLICATE
        self.seen_results[result.id] = result.finished_at
        outcome = TaskOutcome(str(result.status))
        command = self.commands.get(result.id)

        if command is None:
            self.counters["unmatched_replies"] += 1
            self._emit(
                EVENT_TASK_RESULT,
                result_event_data(
                    None,
                    result,
                    outcome=outcome,
                    code=result.code,
                    reason=result.reason,
                    synthetic=False,
                    replayed=retained,
                    finished_at=result.finished_at,
                    robot_id=result.robot_id,
                    task_id=result.id,
                ),
            )
            self._persist(False)
            return ResultDisposition.UNMATCHED

        command.result = result
        if command.is_outstanding:
            self._finish(
                command,
                _TERMINAL_FOR_OUTCOME[outcome],
                outcome=outcome,
                code=result.code,
                reason=result.reason,
                synthetic=False,
                finished_at=result.finished_at,
            )
            event = EVENT_TASK_LATE_RESULT if command.outcome_fired else EVENT_TASK_RESULT
            disposition = (
                ResultDisposition.LATE if command.outcome_fired else ResultDisposition.APPLIED
            )
        else:
            event = EVENT_TASK_LATE_RESULT
            disposition = ResultDisposition.LATE
        if event == EVENT_TASK_LATE_RESULT:
            self.counters["late_results"] += 1
        self._emit(
            event,
            result_event_data(
                command,
                result,
                outcome=outcome,
                code=result.code,
                reason=result.reason,
                synthetic=False,
                replayed=retained,
                finished_at=result.finished_at,
            ),
        )
        command.outcome_fired = True
        self._resolve_waiters(command)
        self._persist(True)
        return disposition

    def sync_pause(self, robot_id: str, task_id: str | None, paused: bool) -> None:
        """Suspend the deadline of the robot's paused task and resume every other one.

        Called from every state message: when ``task_id`` matches an outstanding
        command and ``paused`` is true the clock stops for it; when it stops
        matching or ``paused`` returns to false the paused duration is added to
        its deadline.
        """
        now = self.now
        changed = False
        for command in self.outstanding(robot_id):
            if paused and task_id is not None and command.task_id == task_id:
                if command.paused_at is None:
                    command.paused_at = now
                    changed = True
            elif command.paused_at is not None:
                self._resume(command, now)
                changed = True
        if changed:
            self._persist(True)

    # ----------------------------------------------------------------- time

    def housekeeping(self) -> None:
        """Expire ack and result deadlines (unless suspended) and prune by age."""
        now = self.now
        if not self.deadlines_suspended:
            for command in self.outstanding():
                self._apply_deadlines(command, now)
        self._prune_seen(now)
        self._prune_terminal()

    def resume_deadlines(self) -> None:
        """Lift the start-up suspension without reconciling (tests and edge cases)."""
        self.deadlines_suspended = False

    def reconcile(self, robot_id: str, snapshot: RobotSnapshot | None) -> None:
        """Apply the reconciliation rule to a robot's outstanding commands.

        ``snapshot`` is ``None`` while the robot has not announced this run: its
        commands wait for the announce or their deadline. Otherwise: a command
        never published (``created``) is ``lost`` (``ha_restarted``); a stored
        session that differs from the robot's is ``lost`` (``node_restarted``); a
        passed deadline is ``timeout`` (``no_result``); anything else is re-adopted
        as ``running`` with its remaining deadline, suspended if the robot shows
        the task paused.
        """
        if snapshot is None:
            return
        now = self.now
        changed = False
        for command in self.outstanding(robot_id):
            changed = True
            if command.state is CommandState.CREATED:
                self._synthesise(
                    command,
                    CommandState.LOST,
                    TaskOutcome.LOST,
                    CODE_HA_RESTARTED,
                    "Home Assistant restarted before the command was published",
                    now,
                )
                continue
            if (
                command.session is not None
                and snapshot.session is not None
                and command.session != snapshot.session
            ):
                self._synthesise(
                    command,
                    CommandState.LOST,
                    TaskOutcome.LOST,
                    CODE_NODE_RESTARTED,
                    "The robot node restarted before the task finished",
                    now,
                )
                continue
            paused_here = snapshot.paused_task_id == command.task_id
            if command.paused_at is not None and not paused_here:
                self._resume(command, now)
            if (
                command.deadline is not None
                and command.paused_at is None
                and now >= command.deadline
            ):
                self._synthesise(
                    command,
                    CommandState.TIMEOUT_NO_RESULT,
                    TaskOutcome.TIMEOUT,
                    CODE_NO_RESULT,
                    "No result before the deadline",
                    now,
                )
                continue
            if paused_here and command.paused_at is None:
                command.paused_at = now
            # A sent task whose ack deadline has passed, or whose ack deadline was
            # cleared while it waited for this announce, is adopted as running: the
            # retained ack is consumed by the replay burst and will not come again.
            if (
                command.state is CommandState.SENT
                and command.is_task
                and (command.ack_deadline is None or now >= command.ack_deadline)
            ):
                command.state = CommandState.RUNNING
        if changed:
            self._persist(True)

    def reconcile_all(self, snapshots: Mapping[str, RobotSnapshot]) -> None:
        """Reconcile every robot with outstanding commands, then enable the deadlines.

        A robot without a snapshot has not announced within the settle period.
        The contract says its commands wait for the announce or their result
        deadline, so their ack deadlines (long past after any restart) are
        cleared instead of being applied by the next housekeeping tick as
        ``timeout/no_ack``; :meth:`reconcile` adopts them when the announce comes.
        """
        for robot_id in {command.robot_id for command in self.outstanding()}:
            snapshot = snapshots.get(robot_id)
            if snapshot is None:
                self._await_announce(robot_id)
            else:
                self.reconcile(robot_id, snapshot)
        self.deadlines_suspended = False

    def _await_announce(self, robot_id: str) -> None:
        """Leave only the result deadline on a robot's sent commands until it announces."""
        changed = False
        for command in self.outstanding(robot_id):
            if command.state is CommandState.SENT and command.ack_deadline is not None:
                command.ack_deadline = None
                changed = True
        if changed:
            self._persist(True)

    # ----------------------------------------------------------------- waiting

    def wait_ack(self, task_id: str) -> asyncio.Future[PendingCommand]:
        """A future resolved when the command leaves ``sent`` (ack, result or no-ack timeout)."""
        command = self._require(task_id)
        future: asyncio.Future[PendingCommand] = asyncio.get_running_loop().create_future()
        if command.state not in (CommandState.CREATED, CommandState.SENT):
            future.set_result(command)
        else:
            self._ack_waiters.setdefault(task_id, []).append(future)
        return future

    def wait_result(self, task_id: str) -> asyncio.Future[PendingCommand]:
        """A future resolved when the command reaches a terminal state (real or synthesised)."""
        command = self._require(task_id)
        future: asyncio.Future[PendingCommand] = asyncio.get_running_loop().create_future()
        if command.is_terminal:
            future.set_result(command)
        else:
            self._result_waiters.setdefault(task_id, []).append(future)
        return future

    def cancel_waiters(self) -> None:
        """Cancel every pending future; the commands themselves stay outstanding."""
        for waiters in (self._ack_waiters, self._result_waiters):
            for futures in waiters.values():
                for future in futures:
                    if not future.done():
                        future.cancel()
            waiters.clear()

    # ----------------------------------------------------------------- internals

    def _require(self, task_id: str) -> PendingCommand:
        command = self.commands.get(task_id)
        if command is None:
            raise KeyError(task_id)
        return command

    def _finish(
        self,
        command: PendingCommand,
        state: CommandState,
        *,
        outcome: TaskOutcome | None,
        code: str | None,
        reason: str | None,
        synthetic: bool,
        finished_at: int,
    ) -> None:
        command.state = state
        if command.outcome is None or not command.outcome_fired:
            command.outcome = outcome
            command.code = code
            command.reason = reason
            command.synthetic = synthetic
        command.finished_at = finished_at
        command.paused_at = None

    def _synthesise(
        self,
        command: PendingCommand,
        state: CommandState,
        outcome: TaskOutcome,
        code: str,
        reason: str,
        now: int,
    ) -> None:
        """Terminate a command from HA's own deadlines and fire its outcome once."""
        self._finish(
            command,
            state,
            outcome=outcome,
            code=code,
            reason=reason,
            synthetic=True,
            finished_at=now,
        )
        self.counters["synthesised"] += 1
        if not command.outcome_fired:
            if command.is_task:
                self._emit(
                    EVENT_TASK_RESULT,
                    result_event_data(
                        command,
                        None,
                        outcome=outcome,
                        code=code,
                        reason=reason,
                        synthetic=True,
                        replayed=False,
                        finished_at=now,
                    ),
                )
            else:
                self._emit(
                    EVENT_TASK_ACK, ack_event_data(command, None, late=False, synthetic=True)
                )
            command.outcome_fired = True
        self._resolve_waiters(command)

    def _apply_deadlines(self, command: PendingCommand, now: int) -> None:
        if (
            command.state is CommandState.SENT
            and command.ack_deadline is not None
            and now >= command.ack_deadline
        ):
            seconds = self.settings.ack_timeout_s
            self._synthesise(
                command,
                CommandState.TIMEOUT_NO_ACK,
                TaskOutcome.TIMEOUT,
                CODE_NO_ACK,
                f"No acknowledgement within {seconds:g} s (outcome unknown)",
                now,
            )
            self._persist(True)
            return
        if (
            command.state in (CommandState.SENT, CommandState.RUNNING)
            and command.deadline is not None
            and command.paused_at is None
            and now >= command.deadline
        ):
            self._synthesise(
                command,
                CommandState.TIMEOUT_NO_RESULT,
                TaskOutcome.TIMEOUT,
                CODE_NO_RESULT,
                "No result before the deadline",
                now,
            )
            self._persist(True)

    def _resume(self, command: PendingCommand, now: int) -> None:
        if command.paused_at is not None:
            if command.deadline is not None:
                command.deadline += max(0, now - command.paused_at)
            command.paused_at = None

    def _prune_seen(self, now: int) -> None:
        cutoff = now - self.settings.retention_ms
        stale = [task_id for task_id, ts in self.seen_results.items() if ts < cutoff]
        for task_id in stale:
            del self.seen_results[task_id]
        if stale:
            self._persist(False)

    def _prune_terminal(self) -> None:
        cutoff = self.now - self.settings.retention_ms
        terminal = [
            task_id
            for task_id, command in self.commands.items()
            if command.is_terminal
            and command.finished_at is not None
            and command.finished_at < cutoff
        ]
        for task_id in terminal:
            del self.commands[task_id]
            self._ack_waiters.pop(task_id, None)
            self._result_waiters.pop(task_id, None)
        excess = len([c for c in self.commands.values() if c.is_terminal]) - MAX_TERMINAL_COMMANDS
        if excess > 0:
            for task_id in [t for t, c in self.commands.items() if c.is_terminal][:excess]:
                del self.commands[task_id]
                self._ack_waiters.pop(task_id, None)
                self._result_waiters.pop(task_id, None)

    def _resolve_waiters(self, command: PendingCommand) -> None:
        if command.state not in (CommandState.CREATED, CommandState.SENT):
            for future in self._ack_waiters.pop(command.task_id, []):
                if not future.done():
                    future.set_result(command)
        if command.is_terminal:
            for future in self._result_waiters.pop(command.task_id, []):
                if not future.done():
                    future.set_result(command)
