# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The command state machine in ``commands.py``: the transition table, deadlines and reconciliation.

No ``hass`` here: the registry runs on the fake clock and its events and
persistence requests are recorded by the ``harness`` fixture.
"""

from __future__ import annotations

import asyncio
from typing import Any

import pytest

from custom_components.ros2_bridge.commands import (
    AckDisposition,
    CommandRegistry,
    RegistrySettings,
    ResultDisposition,
    RobotSnapshot,
    TooManyPendingError,
)
from custom_components.ros2_bridge.const import (
    CODE_HA_RESTARTED,
    CODE_NO_ACK,
    CODE_NO_RESULT,
    CODE_NODE_RESTARTED,
    DEFAULT_ACK_EXPIRY_S,
    DEFAULT_RESULT_EXPIRY_S,
    EVENT_TASK_ACK,
    EVENT_TASK_LATE_RESULT,
    EVENT_TASK_RESULT,
    CommandState,
    CommandType,
    TaskKind,
    TaskOutcome,
)
from custom_components.ros2_bridge.models import PendingCommand

from .common import (
    CANCEL_ID,
    OTHER_SESSION,
    ROBOT_ID,
    SESSION,
    TASK_ID,
    FakeClock,
    make_ack,
    make_cancel_ack,
    make_rejected_ack,
    make_result,
)
from .conftest import RegistryHarness

SOURCE = {"origin": "ha", "kind": "service", "instance_id": "x"}
TIMEOUT_S = 60.0
MARGIN_MS = 10_000
ACK_MS = 2_000


def new_task(
    harness: RegistryHarness,
    *,
    task_id: str = TASK_ID,
    robot_id: str = ROBOT_ID,
    kind: TaskKind = TaskKind.MOTION,
    timeout_s: float = TIMEOUT_S,
    session: str | None = SESSION,
    send: bool = True,
) -> PendingCommand:
    command = harness.registry.create(
        task_id=task_id,
        robot_id=robot_id,
        type=CommandType.TASK,
        task="move_to",
        kind=kind,
        params={"pose": "home"},
        target_id=None,
        session=session,
        source=SOURCE,
        issued_at=harness.clock(),
        max_age_ms=5000,
        timeout_s=timeout_s,
    )
    if send:
        harness.registry.mark_sent(task_id)
    return command


def new_cancel(harness: RegistryHarness, *, task_id: str = CANCEL_ID) -> PendingCommand:
    command = harness.registry.create(
        task_id=task_id,
        robot_id=ROBOT_ID,
        type=CommandType.CANCEL,
        task=None,
        kind=None,
        params={},
        target_id=TASK_ID,
        session=SESSION,
        source=SOURCE,
        issued_at=harness.clock(),
        max_age_ms=5000,
        timeout_s=None,
    )
    harness.registry.mark_sent(task_id)
    return command


def ack(
    harness: RegistryHarness,
    *,
    accepted: bool = True,
    retained: bool = False,
    task_id: str = TASK_ID,
    session: str = SESSION,
) -> AckDisposition:
    payload = make_ack(id=task_id, session=session) if accepted else make_rejected_ack(id=task_id)
    return harness.registry.handle_ack(payload, retained=retained)


def result(
    harness: RegistryHarness,
    *,
    status: str = "succeeded",
    retained: bool = False,
    task_id: str = TASK_ID,
    kind: str = "motion",
    finished_at: int | None = None,
) -> ResultDisposition:
    goal = {"succeeded": 4, "cancelled": 5, "aborted": 6}[status]
    payload = make_result(
        id=task_id,
        status=status,
        goal_status=goal,
        kind=kind,
        finished_at=harness.clock() if finished_at is None else finished_at,
        started_at=harness.clock() - 1000,
    )
    return harness.registry.handle_result(payload, retained=retained)


# --------------------------------------------------------------------------- creation and sending


def test_create_and_mark_sent_plan_the_deadlines(harness: RegistryHarness) -> None:
    command = new_task(harness, send=False)
    assert command.state is CommandState.CREATED
    assert command.sent_at is None and command.deadline is None
    assert harness.persists == []
    harness.clock.advance(250)
    harness.registry.mark_sent(TASK_ID)
    assert command.state is CommandState.SENT
    assert command.sent_at == harness.clock()
    assert command.ack_deadline == command.sent_at + ACK_MS
    assert command.deadline == command.sent_at + int(TIMEOUT_S * 1000) + MARGIN_MS
    assert harness.persists == [True]


def test_cancel_deadline_is_the_ack_expiry(harness: RegistryHarness) -> None:
    command = new_cancel(harness)
    assert command.deadline == command.sent_at + DEFAULT_ACK_EXPIRY_S * 1000


def test_duplicate_id_and_pending_cap(harness: RegistryHarness) -> None:
    harness.registry.update_settings(RegistrySettings(max_pending_per_robot=2))
    new_task(harness, task_id=TASK_ID)
    with pytest.raises(ValueError):
        new_task(harness, task_id=TASK_ID)
    new_task(harness, task_id=CANCEL_ID)
    with pytest.raises(TooManyPendingError) as excinfo:
        new_task(harness, task_id="7b3e9d21-5f6a-4c8b-9e0d-1f2a3b4c5d6e")
    assert excinfo.value.limit == 2
    new_task(harness, task_id="7b3e9d21-5f6a-4c8b-9e0d-1f2a3b4c5d6e", robot_id="arm_right")
    assert harness.registry.pending_count(ROBOT_ID) == 2


def test_publish_failed_keeps_the_command_sent(harness: RegistryHarness) -> None:
    command = new_task(harness, send=False)
    harness.registry.mark_sent(TASK_ID, publish_failed=True)
    assert command.state is CommandState.SENT
    assert command.publish_failed is True
    assert ack(harness) is AckDisposition.ACCEPTED
    assert command.state is CommandState.RUNNING


# --------------------------------------------------------------------------- the transition table


@pytest.mark.parametrize(
    ("setup", "action", "expected_state", "events"),
    [
        ("sent", "ack_accepted", CommandState.RUNNING, [EVENT_TASK_ACK]),
        ("sent", "ack_rejected", CommandState.REJECTED, [EVENT_TASK_ACK, EVENT_TASK_RESULT]),
        ("sent", "ack_timeout", CommandState.TIMEOUT_NO_ACK, [EVENT_TASK_RESULT]),
        ("sent", "result_succeeded", CommandState.SUCCEEDED, [EVENT_TASK_RESULT]),
        ("running", "result_succeeded", CommandState.SUCCEEDED, [EVENT_TASK_RESULT]),
        ("running", "result_cancelled", CommandState.CANCELLED, [EVENT_TASK_RESULT]),
        ("running", "result_aborted", CommandState.ABORTED, [EVENT_TASK_RESULT]),
        ("running", "deadline", CommandState.TIMEOUT_NO_RESULT, [EVENT_TASK_RESULT]),
        ("timeout_no_ack", "ack_accepted", CommandState.RUNNING, [EVENT_TASK_ACK]),
        ("timeout_no_ack", "ack_rejected", CommandState.TIMEOUT_NO_ACK, []),
        (
            "timeout_no_ack",
            "result_succeeded",
            CommandState.TIMEOUT_NO_ACK,
            [EVENT_TASK_LATE_RESULT],
        ),
        ("reopened", "result_succeeded", CommandState.SUCCEEDED, [EVENT_TASK_LATE_RESULT]),
        ("reopened", "deadline", CommandState.TIMEOUT_NO_RESULT, []),
        ("running", "ack_accepted", CommandState.RUNNING, []),
        ("succeeded", "ack_accepted", CommandState.SUCCEEDED, []),
        ("succeeded", "result_succeeded", CommandState.SUCCEEDED, []),
        ("cancel_sent", "cancel_ack_accepted", CommandState.DONE, [EVENT_TASK_ACK]),
        ("cancel_sent", "cancel_ack_rejected", CommandState.REJECTED, [EVENT_TASK_ACK]),
        ("cancel_sent", "ack_timeout", CommandState.TIMEOUT_NO_ACK, [EVENT_TASK_ACK]),
        ("cancel_timeout_no_ack", "cancel_ack_accepted", CommandState.DONE, [EVENT_TASK_ACK]),
    ],
)
def test_transition_table(
    harness: RegistryHarness,
    setup: str,
    action: str,
    expected_state: CommandState,
    events: list[str],
) -> None:
    registry = harness.registry
    if setup.startswith("cancel"):
        command = new_cancel(harness, task_id=TASK_ID)
    else:
        command = new_task(harness)
    if setup in ("running", "succeeded", "reopened"):
        assert ack(harness) is AckDisposition.ACCEPTED
    if setup == "succeeded":
        assert result(harness) is ResultDisposition.APPLIED
    if setup in ("timeout_no_ack", "cancel_timeout_no_ack", "reopened"):
        if setup == "reopened":
            command.state = CommandState.SENT  # pretend the ack never came
            command.ack = None
        harness.clock.advance(ACK_MS)
        registry.housekeeping()
        assert command.state is CommandState.TIMEOUT_NO_ACK
        assert command.outcome_fired
    if setup == "reopened":
        assert ack(harness) is AckDisposition.LATE_ACCEPTED
        assert command.state is CommandState.RUNNING
    harness.clear()

    if action == "ack_accepted":
        ack(harness)
    elif action == "ack_rejected":
        ack(harness, accepted=False)
    elif action == "ack_timeout":
        harness.clock.advance(ACK_MS)
        registry.housekeeping()
    elif action.startswith("result_"):
        result(harness, status=action.removeprefix("result_"))
    elif action == "deadline":
        harness.clock.advance(int(TIMEOUT_S * 1000) + MARGIN_MS)
        registry.housekeeping()
    elif action == "cancel_ack_accepted":
        registry.handle_ack(make_cancel_ack(id=TASK_ID), retained=False)
    elif action == "cancel_ack_rejected":
        registry.handle_ack(
            make_rejected_ack(id=TASK_ID, type="cancel", task=None, code="rejected_no_such_task"),
            retained=False,
        )
    assert command.state is expected_state
    assert [name for name, _ in harness.events] == events


def test_terminal_exactly_once_for_a_reopened_command(harness: RegistryHarness) -> None:
    """timeout_no_ack fires task_result once; the late ack and the real result never repeat it."""
    command = new_task(harness)
    harness.clock.advance(ACK_MS)
    harness.registry.housekeeping()
    first = harness.events_named(EVENT_TASK_RESULT)
    assert len(first) == 1
    assert first[0]["status"] == "timeout"
    assert first[0]["code"] == CODE_NO_ACK
    assert first[0]["synthetic"] is True
    assert ack(harness) is AckDisposition.LATE_ACCEPTED
    late_ack = harness.events_named(EVENT_TASK_ACK)[-1]
    assert late_ack["late"] is True and late_ack["accepted"] is True
    assert result(harness) is ResultDisposition.LATE
    assert len(harness.events_named(EVENT_TASK_RESULT)) == 1
    late = harness.events_named(EVENT_TASK_LATE_RESULT)
    assert len(late) == 1 and late[0]["status"] == "succeeded"
    assert command.state is CommandState.SUCCEEDED
    assert command.outcome is TaskOutcome.TIMEOUT  # the outcome that fired stays on record
    assert command.result is not None


def test_late_accepted_ack_fires_even_when_retained(harness: RegistryHarness) -> None:
    new_task(harness)
    harness.clock.advance(ACK_MS)
    harness.registry.housekeeping()
    harness.clear()
    assert ack(harness, retained=True) is AckDisposition.LATE_ACCEPTED
    assert harness.events_named(EVENT_TASK_ACK)[0]["late"] is True


def test_retained_acks_fire_no_event(harness: RegistryHarness) -> None:
    command = new_task(harness)
    assert ack(harness, retained=True) is AckDisposition.ACCEPTED
    assert command.state is CommandState.RUNNING
    assert harness.events == []


def test_rejected_ack_event_shapes(harness: RegistryHarness) -> None:
    command = new_task(harness)
    ack(harness, accepted=False)
    ack_event = harness.events_named(EVENT_TASK_ACK)[0]
    assert ack_event["accepted"] is False
    assert ack_event["code"] == "rejected_busy"
    assert ack_event["late"] is False
    assert ack_event["source"] == SOURCE
    result_event = harness.events_named(EVENT_TASK_RESULT)[0]
    assert result_event["status"] == "rejected"
    assert result_event["code"] == "rejected_busy"
    assert result_event["synthetic"] is False
    assert result_event["goal_status"] is None
    assert command.outcome is TaskOutcome.REJECTED
    assert command.reason == command.ack.reason if command.ack else False


def test_result_before_ack_terminates_sent(harness: RegistryHarness) -> None:
    command = new_task(harness)
    assert result(harness) is ResultDisposition.APPLIED
    assert command.state is CommandState.SUCCEEDED
    assert ack(harness) is AckDisposition.IGNORED
    assert harness.registry.counters["duplicate_acks"] == 1
    assert len(harness.events) == 1


def test_result_event_shape(harness: RegistryHarness) -> None:
    new_task(harness)
    ack(harness)
    harness.clock.advance(3000)
    result(harness, status="aborted")
    event = harness.events_named(EVENT_TASK_RESULT)[0]
    assert event["robot_id"] == ROBOT_ID
    assert event["task_id"] == TASK_ID
    assert event["task"] == "move_to"
    assert event["kind"] == "motion"
    assert event["status"] == "aborted"
    assert event["goal_status"] == 6
    assert event["code"] == "ok"
    assert event["duration_ms"] == 1000
    assert event["replayed"] is False
    assert event["synthetic"] is False
    assert event["source"] == SOURCE
    assert event["device_id"] is None  # the hub fills it in


# --------------------------------------------------------------------------- dedupe and replay


def test_retained_result_fires_once_and_duplicates_are_ignored(harness: RegistryHarness) -> None:
    new_task(harness)
    ack(harness)
    assert result(harness, retained=True) is ResultDisposition.APPLIED
    assert harness.events_named(EVENT_TASK_RESULT)[0]["replayed"] is True
    assert result(harness, retained=True) is ResultDisposition.DUPLICATE
    assert result(harness) is ResultDisposition.DUPLICATE
    assert len(harness.events_named(EVENT_TASK_RESULT)) == 1
    assert harness.registry.counters["replayed_results_ignored"] == 2


def test_unknown_result_fires_once(harness: RegistryHarness) -> None:
    assert result(harness, task_id=CANCEL_ID) is ResultDisposition.UNMATCHED
    event = harness.events_named(EVENT_TASK_RESULT)[0]
    assert event["task_id"] == CANCEL_ID and event["robot_id"] == ROBOT_ID
    assert event["source"] == {}
    assert result(harness, task_id=CANCEL_ID) is ResultDisposition.DUPLICATE
    assert harness.registry.counters["unmatched_replies"] == 1


def test_unknown_ack_is_counted_not_fired(harness: RegistryHarness) -> None:
    assert ack(harness) is AckDisposition.UNMATCHED
    assert harness.events == []
    assert harness.registry.counters["unmatched_replies"] == 1


def test_seen_results_are_bounded_by_age_not_count(harness: RegistryHarness) -> None:
    """250 results within 300 s all stay deduplicated; they leave only when their age passes."""
    ids = [f"{i:08x}-0000-4000-8000-{i:012x}" for i in range(250)]
    for i, task_id in enumerate(ids):
        harness.clock.advance(1000)
        assert result(harness, task_id=task_id) is ResultDisposition.UNMATCHED
        if i % 50 == 0:
            harness.registry.housekeeping()
    assert len(harness.registry.seen_results) == 250
    for task_id in ids:
        assert result(harness, task_id=task_id) is ResultDisposition.DUPLICATE
    retention = harness.registry.settings.retention_ms
    harness.clock.advance(retention - 200_000)  # cutoff reaches the oldest fifty or so
    harness.registry.housekeeping()
    assert 0 < len(harness.registry.seen_results) < 250
    harness.clock.advance(250_000)
    harness.registry.housekeeping()
    assert harness.registry.seen_results == {}
    assert False in harness.persists  # pruning is a delayed save


def test_terminal_commands_leave_memory_after_retention(harness: RegistryHarness) -> None:
    new_task(harness)
    ack(harness)
    result(harness)
    assert harness.registry.get(TASK_ID) is not None
    harness.clock.advance(int((DEFAULT_RESULT_EXPIRY_S + 10) * 1000) + 1)
    harness.registry.housekeeping()
    assert harness.registry.get(TASK_ID) is None


# --------------------------------------------------------------------------- deadlines and pause


def test_deadlines_are_suspended_until_reconciled(clock: FakeClock) -> None:
    events: list[tuple[str, dict[str, Any]]] = []
    registry = CommandRegistry(clock=clock, on_event=lambda n, d: events.append((n, d)))
    assert registry.deadlines_suspended
    registry.create(
        task_id=TASK_ID,
        robot_id=ROBOT_ID,
        type=CommandType.TASK,
        task="move_to",
        kind=TaskKind.MOTION,
        params={},
        target_id=None,
        session=SESSION,
        source=SOURCE,
        issued_at=clock(),
        max_age_ms=5000,
        timeout_s=TIMEOUT_S,
    )
    registry.mark_sent(TASK_ID)
    clock.advance(10 * 60 * 1000)
    registry.housekeeping()
    assert registry.get(TASK_ID).state is CommandState.SENT  # type: ignore[union-attr]
    registry.reconcile_all({ROBOT_ID: RobotSnapshot(session=SESSION)})
    assert not registry.deadlines_suspended
    assert registry.get(TASK_ID).state is CommandState.TIMEOUT_NO_RESULT  # type: ignore[union-attr]
    assert events[0][1]["code"] == CODE_NO_RESULT


def test_result_deadline_comes_from_timeout_plus_margin(harness: RegistryHarness) -> None:
    command = new_task(harness, timeout_s=5)
    ack(harness)
    harness.clock.advance(5000 + MARGIN_MS - 1)
    harness.registry.housekeeping()
    assert command.state is CommandState.RUNNING
    harness.clock.advance(1)
    harness.registry.housekeeping()
    assert command.state is CommandState.TIMEOUT_NO_RESULT
    event = harness.events_named(EVENT_TASK_RESULT)[0]
    assert event["code"] == CODE_NO_RESULT and event["synthetic"] is True


def test_pause_suspends_the_deadline_and_resume_extends_it(harness: RegistryHarness) -> None:
    """Paused for longer than the margin, resumed, then a real result arrives as task_result."""
    command = new_task(harness, timeout_s=5)
    ack(harness)
    original_deadline = command.deadline
    assert original_deadline is not None
    harness.clock.advance(1000)
    harness.registry.sync_pause(ROBOT_ID, TASK_ID, True)
    assert command.paused_at == harness.clock()
    harness.clock.advance(MARGIN_MS + 5000)  # longer than the margin
    harness.registry.housekeeping()
    assert command.state is CommandState.RUNNING
    harness.registry.sync_pause(ROBOT_ID, TASK_ID, False)
    assert command.paused_at is None
    assert command.deadline == original_deadline + MARGIN_MS + 5000
    harness.clock.advance(3000)
    harness.registry.housekeeping()
    assert command.state is CommandState.RUNNING
    assert result(harness) is ResultDisposition.APPLIED
    assert [name for name, _ in harness.events][-1] == EVENT_TASK_RESULT
    assert not harness.events_named(EVENT_TASK_LATE_RESULT)


def test_pause_of_another_task_resumes_this_one(harness: RegistryHarness) -> None:
    command = new_task(harness)
    ack(harness)
    harness.registry.sync_pause(ROBOT_ID, TASK_ID, True)
    harness.clock.advance(500)
    harness.registry.sync_pause(ROBOT_ID, CANCEL_ID, True)
    assert command.paused_at is None
    harness.registry.sync_pause(ROBOT_ID, None, False)
    assert command.paused_at is None


# --------------------------------------------------------------------------- reconciliation


def test_reconcile_readopts_same_session_within_deadline(harness: RegistryHarness) -> None:
    """Simulated HA restart mid-task: the stored command is adopted as running."""
    stored = harness.registry.export()
    new_task(harness)
    stored = harness.registry.export()
    clock = harness.clock
    restarted = CommandRegistry(clock=clock)
    restarted.load(stored)
    clock.advance(ACK_MS + 5000)
    restarted.reconcile_all({ROBOT_ID: RobotSnapshot(session=SESSION)})
    command = restarted.get(TASK_ID)
    assert command is not None
    assert command.state is CommandState.RUNNING
    assert command.deadline == stored["outstanding"][TASK_ID]["deadline"]


def test_reconcile_readopts_paused_task_suspended(harness: RegistryHarness) -> None:
    new_task(harness)
    ack(harness)
    stored = harness.registry.export()
    restarted = CommandRegistry(clock=harness.clock)
    restarted.load(stored)
    restarted.reconcile_all({ROBOT_ID: RobotSnapshot(session=SESSION, paused_task_id=TASK_ID)})
    command = restarted.get(TASK_ID)
    assert command is not None and command.paused_at == harness.clock()


def test_reconcile_marks_lost_on_session_change(harness: RegistryHarness) -> None:
    command = new_task(harness)
    ack(harness)
    harness.clear()
    harness.registry.reconcile(ROBOT_ID, RobotSnapshot(session=OTHER_SESSION))
    assert command.state is CommandState.LOST
    event = harness.events_named(EVENT_TASK_RESULT)[0]
    assert event["status"] == "lost" and event["code"] == CODE_NODE_RESTARTED
    assert event["synthetic"] is True


def test_reconcile_times_out_past_deadline(harness: RegistryHarness) -> None:
    command = new_task(harness, timeout_s=1)
    ack(harness)
    harness.clock.advance(1000 + MARGIN_MS)
    harness.registry.reconcile(ROBOT_ID, RobotSnapshot(session=SESSION))
    assert command.state is CommandState.TIMEOUT_NO_RESULT
    assert harness.events_named(EVENT_TASK_RESULT)[-1]["code"] == CODE_NO_RESULT


def test_reconcile_loses_never_published_commands(harness: RegistryHarness) -> None:
    command = new_task(harness, send=False)
    harness.registry.reconcile(ROBOT_ID, RobotSnapshot(session=SESSION))
    assert command.state is CommandState.LOST
    assert harness.events_named(EVENT_TASK_RESULT)[0]["code"] == CODE_HA_RESTARTED


def test_reconcile_waits_for_an_unannounced_robot(harness: RegistryHarness) -> None:
    command = new_task(harness)
    harness.registry.reconcile_all({})
    assert command.state is CommandState.SENT
    assert harness.events == []
    harness.registry.reconcile(ROBOT_ID, None)
    assert command.state is CommandState.SENT


def test_unannounced_robot_commands_wait_for_the_announce_not_the_ack_deadline(
    harness: RegistryHarness,
) -> None:
    """After a restart the stored ack deadline is long past; only the result deadline applies."""
    command = new_task(harness, timeout_s=5)
    harness.registry.reconcile_all({})
    assert command.ack_deadline is None, "the ack deadline is cleared while awaiting the announce"
    assert harness.persists[-1] is True
    harness.clock.advance(ACK_MS * 3)
    harness.registry.housekeeping()
    assert command.state is CommandState.SENT
    assert harness.events == []

    harness.registry.reconcile(ROBOT_ID, RobotSnapshot(session=SESSION))
    assert command.state is CommandState.RUNNING
    assert harness.events == []


def test_unannounced_robot_commands_still_time_out_on_the_result_deadline(
    harness: RegistryHarness,
) -> None:
    command = new_task(harness, timeout_s=5)
    harness.registry.reconcile_all({})
    harness.clock.advance(5000 + MARGIN_MS)
    harness.registry.housekeeping()
    assert command.state is CommandState.TIMEOUT_NO_RESULT
    assert harness.events_named(EVENT_TASK_RESULT)[0]["code"] == CODE_NO_RESULT


def test_reconcile_lost_cancel_fires_task_ack(harness: RegistryHarness) -> None:
    command = new_cancel(harness)
    harness.clear()
    harness.registry.reconcile(ROBOT_ID, RobotSnapshot(session=OTHER_SESSION))
    assert command.state is CommandState.LOST
    event = harness.events_named(EVENT_TASK_ACK)[0]
    assert event["accepted"] is False and event["synthetic"] is True
    assert event["code"] == CODE_NODE_RESTARTED


def test_result_arriving_after_lost_is_late(harness: RegistryHarness) -> None:
    """An old-session retained result after a new-session announce still resolves by id."""
    new_task(harness)
    ack(harness)
    harness.registry.reconcile(ROBOT_ID, RobotSnapshot(session=OTHER_SESSION))
    harness.clear()
    assert result(harness, retained=True) is ResultDisposition.LATE
    assert harness.events_named(EVENT_TASK_LATE_RESULT)[0]["replayed"] is True


# --------------------------------------------------------------------------- persistence


def test_export_and_load_round_trip(harness: RegistryHarness) -> None:
    command = new_task(harness)
    ack(harness)
    harness.registry.sync_pause(ROBOT_ID, TASK_ID, True)
    result(harness, task_id=CANCEL_ID)  # unmatched: lands only in seen_results
    exported = harness.registry.export()
    assert set(exported["outstanding"]) == {TASK_ID}
    entry = exported["outstanding"][TASK_ID]
    assert entry["state"] == "running" and entry["session"] == SESSION
    assert entry["paused_at"] == command.paused_at
    assert [CANCEL_ID, harness.clock()] in exported["seen_results"]
    loaded = CommandRegistry(clock=harness.clock)
    loaded.load(exported)
    again = loaded.get(TASK_ID)
    assert again is not None
    assert again.state is CommandState.RUNNING
    assert again.deadline == command.deadline
    assert again.kind is TaskKind.MOTION
    assert loaded.seen_results == {CANCEL_ID: harness.clock()}
    assert loaded.export() == exported


def test_load_ignores_junk(clock: FakeClock) -> None:
    registry = CommandRegistry(clock=clock)
    registry.load({"outstanding": {"x": {"nope": 1}, 5: {}}, "seen_results": [["a", 1], ["b"], 3]})
    assert registry.commands == {}
    assert registry.seen_results == {"a": 1}
    registry.load(None)
    assert registry.commands == {}


def test_persist_requests_are_urgent_on_transitions(harness: RegistryHarness) -> None:
    new_task(harness)
    ack(harness)
    result(harness)
    assert harness.persists == [True, True, True]


# --------------------------------------------------------------------------- waiting and queries


async def test_wait_ack_and_wait_result(harness: RegistryHarness) -> None:
    new_task(harness)
    ack_future = harness.registry.wait_ack(TASK_ID)
    result_future = harness.registry.wait_result(TASK_ID)
    assert not ack_future.done()
    ack(harness)
    assert ack_future.done() and ack_future.result().state is CommandState.RUNNING
    assert not result_future.done()
    result(harness)
    assert result_future.result().state is CommandState.SUCCEEDED
    assert harness.registry.wait_ack(TASK_ID).done()
    assert harness.registry.wait_result(TASK_ID).done()


async def test_wait_ack_resolves_on_no_ack_timeout(harness: RegistryHarness) -> None:
    new_task(harness)
    future = harness.registry.wait_ack(TASK_ID)
    harness.clock.advance(ACK_MS)
    harness.registry.housekeeping()
    assert future.result().state is CommandState.TIMEOUT_NO_ACK


async def test_cancel_waiters_cancels_the_futures(harness: RegistryHarness) -> None:
    new_task(harness)
    future = harness.registry.wait_result(TASK_ID)
    harness.registry.cancel_waiters()
    with pytest.raises(asyncio.CancelledError):
        await future
    assert harness.registry.get(TASK_ID).state is CommandState.SENT  # type: ignore[union-attr]


async def test_wait_result_for_a_cancel_resolves_on_done(harness: RegistryHarness) -> None:
    new_cancel(harness)
    future = harness.registry.wait_result(CANCEL_ID)
    harness.registry.handle_ack(make_cancel_ack(), retained=False)
    assert future.result().state is CommandState.DONE


def test_wait_for_unknown_command_raises(harness: RegistryHarness) -> None:
    with pytest.raises(KeyError):
        harness.registry.mark_sent("nope")


def test_queries(harness: RegistryHarness) -> None:
    new_task(harness)
    ack(harness)
    new_task(harness, task_id=CANCEL_ID, kind=TaskKind.CONTROL)
    assert harness.registry.active_motion(ROBOT_ID).task_id == TASK_ID  # type: ignore[union-attr]
    assert [c.task_id for c in harness.registry.recent(1)] == [CANCEL_ID]
    assert len(harness.registry.outstanding()) == 2
    assert harness.registry.active_motion("arm_right") is None
