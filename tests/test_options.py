# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Every hub-side option has an effect: the timings, the heuristics and the node expiries.

Each test starts a hub with one option changed from its default and shows the
behaviour follow it, so that no timing is a constant in disguise. The hubs are
built by hand on the fake clock (``test_repairs.start_hub``), so the clock-driven
rules (staleness, windows, retention) are exercised exactly, and the timers
(settle, grace, housekeeping) are fired with the frozen time.
"""

from __future__ import annotations

from collections.abc import AsyncIterator, Awaitable, Callable
from typing import Any

import pytest
from homeassistant.core import HomeAssistant
from homeassistant.helpers import issue_registry as ir
from pytest_homeassistant_custom_component.common import MockConfigEntry

from custom_components.ros2_bridge.commands import CommandRegistry
from custom_components.ros2_bridge.const import (
    CONF_ACK_EXPIRY_S,
    CONF_CLOCK_SKEW_WARN_MS,
    CONF_DUPLICATE_QUIET_S,
    CONF_DUPLICATE_SESSION_CHANGES,
    CONF_DUPLICATE_SESSION_WINDOW_S,
    CONF_FEEDBACK_THROTTLE_MS,
    CONF_HEARTBEAT_STALE_INTERVALS,
    CONF_HEARTBEAT_UNAVAILABLE_AFTER_S,
    CONF_INTENT_ACK_WAIT_S,
    CONF_JOINT_MOVING_HOLD_S,
    CONF_JOINT_MOVING_THRESHOLD,
    CONF_MIRROR_EXTRA_ATTRIBUTES,
    CONF_REPLAY_SETTLE_S,
    CONF_RESULT_EXPIRY_S,
    CONF_RESULT_TIMEOUT_MARGIN_S,
    CONF_SESSION_CHANGE_GRACE_S,
    DEFAULT_OPTIONS,
    ISSUE_CLOCK_SKEW,
    ISSUE_DUPLICATE_ROBOT_ID,
    CommandState,
    JointUnit,
    issue_id,
)
from custom_components.ros2_bridge.hub import (
    BridgeHub,
    HubOptions,
    _duration_text,
    _minutes_text,
    _moving_hold_ms,
)
from custom_components.ros2_bridge.protocol import CommandSource
from custom_components.ros2_bridge.store import CommandStore

from .common import (
    OTHER_SESSION,
    RESET_ID,
    ROBOT_ID,
    TASK_ID,
    WAIT_ID,
    FakeClock,
    ack_payload,
    feedback_payload,
    fire_robot_message,
    result_payload,
    state_payload,
)
from .test_repairs import announce_online, send, start_hub

THIRD_SESSION = "5b2d8f1e-7c3a-4e9b-8d6f-1a2b3c4d5e6f"
FOURTH_SESSION = "0a1b2c3d-4e5f-4a6b-8c7d-9e0f1a2b3c4d"


@pytest.fixture
async def started_hubs() -> AsyncIterator[list[BridgeHub]]:
    """Every hub a test starts by hand; each is stopped at teardown so no timer lingers."""
    hubs: list[BridgeHub] = []
    yield hubs
    for hub in hubs:
        await hub.async_stop()


def _issue(hass: HomeAssistant, kind: str) -> ir.IssueEntry | None:
    return ir.async_get(hass).async_get_issue("ros2_bridge", issue_id(kind, ROBOT_ID))


# --------------------------------------------------------------------------- HubOptions


def test_hub_options_read_every_key_and_round_trip() -> None:
    options = HubOptions.from_mapping(
        {
            **DEFAULT_OPTIONS,
            CONF_ACK_EXPIRY_S: 60,
            CONF_RESULT_EXPIRY_S: 900,
            CONF_REPLAY_SETTLE_S: 7,
            CONF_SESSION_CHANGE_GRACE_S: 2.5,
            CONF_FEEDBACK_THROTTLE_MS: 0,
            CONF_HEARTBEAT_STALE_INTERVALS: 5,
            CONF_CLOCK_SKEW_WARN_MS: 3000,
            CONF_DUPLICATE_SESSION_CHANGES: 1,
            CONF_DUPLICATE_SESSION_WINDOW_S: 90,
            CONF_DUPLICATE_QUIET_S: 120,
            CONF_MIRROR_EXTRA_ATTRIBUTES: ["pm2_5"],
            CONF_INTENT_ACK_WAIT_S: 3,
            CONF_JOINT_MOVING_THRESHOLD: 0.2,
            CONF_JOINT_MOVING_HOLD_S: 2.5,
        }
    )
    assert options.ack_expiry_s == 60.0
    assert options.result_expiry_s == 900.0
    assert options.replay_settle_s == 7.0
    assert options.session_change_grace_s == 2.5
    assert options.feedback_throttle_ms == 0
    assert options.heartbeat_stale_intervals == 5
    assert options.clock_skew_warn_ms == 3000
    assert options.duplicate_session_changes == 1
    assert options.duplicate_session_window_s == 90.0
    assert options.duplicate_quiet_s == 120.0
    assert options.mirror_extra_attributes == ("pm2_5",)
    assert options.intent_ack_wait_s == 3.0
    assert options.joint_moving_threshold == 0.2
    assert options.joint_moving_hold_s == 2.5
    settings = options.registry_settings()
    assert settings.ack_expiry_s == 60.0 and settings.result_expiry_s == 900.0
    assert settings.retention_ms == 910_000
    shown = options.to_dict()
    assert shown[CONF_RESULT_EXPIRY_S] == 900.0
    assert shown[CONF_MIRROR_EXTRA_ATTRIBUTES] == ["pm2_5"]
    assert shown["secret_configured"] is False
    assert options.joint_moving_threshold_for(JointUnit.DEG) == 0.2
    assert options.joint_moving_threshold_for(JointUnit.RAD) == 0.2


def test_joint_moving_threshold_zero_means_the_unit_default() -> None:
    options = HubOptions()
    assert options.joint_moving_threshold == 0.0
    assert options.joint_moving_threshold_for(JointUnit.DEG) == 0.5
    assert options.joint_moving_threshold_for(JointUnit.RAD) == 0.01


def test_text_helpers() -> None:
    assert _duration_text(300) == "5 minutes"
    assert _duration_text(60) == "1 minute"
    assert _duration_text(90) == "90 s"
    assert _duration_text(45.5) == "45.5 s"
    assert _minutes_text(600) == "10"
    assert _minutes_text(90) == "1.5"
    assert _moving_hold_ms(2.0) == 1000
    assert _moving_hold_ms(0.5) == 4000
    assert _moving_hold_ms(100.0) == 1000
    assert _moving_hold_ms(0.0) == 1000
    # The option replaces the two-interval rule; the tick stays the floor.
    assert _moving_hold_ms(2.0, 3.0) == 3000
    assert _moving_hold_ms(0.5, 2.5) == 2500
    assert _moving_hold_ms(2.0, 0.5) == 1000


# --------------------------------------------------------------------------- replay_settle_s


async def test_replay_settle_option_is_the_quiet_period(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    clock: FakeClock,
    advance: Callable[[float], Awaitable[None]],
    started_hubs: list[BridgeHub],
) -> None:
    entry = make_config_entry(options={CONF_REPLAY_SETTLE_S: 7})
    store = CommandStore(hass)
    registry = CommandRegistry(clock=clock)
    registry.load(await store.async_load())
    store.bind(registry)
    hub = BridgeHub(hass, entry, registry, store, clock=clock)
    entry.runtime_data = hub
    started_hubs.append(hub)
    await hub.async_start()
    hub._set_ready()
    if not hub.mqtt_connected:
        hub._on_connection_status(True)
    assert hub._settle_handle is not None
    await advance(6)
    assert hub.reconciled is False, "the default 3 s would have reconciled by now"
    await advance(1.5)
    assert hub.reconciled is True


# --------------------------------------------------------------------------- session_change_grace_s


async def test_session_change_grace_option_delays_the_reconciliation(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    clock: FakeClock,
    complete_hub_start: Any,
    advance: Callable[[float], Awaitable[None]],
    started_hubs: list[BridgeHub],
) -> None:
    entry = make_config_entry(options={CONF_SESSION_CHANGE_GRACE_S: 4})
    hub = await start_hub(hass, entry, clock, complete_hub_start, started_hubs)
    await announce_online(hass)
    command = await send(hub, TASK_ID)
    fire_robot_message(hass, f"task/{TASK_ID}/ack", ack_payload(id=TASK_ID, task="reset"))
    await hass.async_block_till_done()
    assert command.state is CommandState.RUNNING

    await announce_online(hass, session=OTHER_SESSION)
    await advance(3)
    assert command.state is CommandState.RUNNING, "the default 1 s grace would have expired"
    await advance(1.5)
    assert command.state is CommandState.LOST
    assert command.code == "node_restarted"


# --------------------------------------------------------------------------- feedback_throttle_ms


async def test_feedback_throttle_zero_writes_every_feedback(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    clock: FakeClock,
    complete_hub_start: Any,
    started_hubs: list[BridgeHub],
) -> None:
    entry = make_config_entry(options={CONF_FEEDBACK_THROTTLE_MS: 0})
    hub = await start_hub(hass, entry, clock, complete_hub_start, started_hubs)
    await announce_online(hass)
    fire_robot_message(hass, "state", state_payload())
    fire_robot_message(hass, f"task/{TASK_ID}/feedback", feedback_payload(progress=0.4))
    fire_robot_message(hass, f"task/{TASK_ID}/feedback", feedback_payload(progress=0.9))
    await hass.async_block_till_done()
    robot = hub.robots[ROBOT_ID]
    assert robot.feedback is not None and robot.feedback.progress == 0.9
    assert ROBOT_ID not in hub._feedback_pending, "nothing waits for the housekeeping tick"


async def test_feedback_throttle_option_sets_the_spacing(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    clock: FakeClock,
    complete_hub_start: Any,
    advance: Callable[[float], Awaitable[None]],
    started_hubs: list[BridgeHub],
) -> None:
    entry = make_config_entry(options={CONF_FEEDBACK_THROTTLE_MS: 3000})
    hub = await start_hub(hass, entry, clock, complete_hub_start, started_hubs)
    await announce_online(hass)
    fire_robot_message(hass, "state", state_payload())
    fire_robot_message(hass, f"task/{TASK_ID}/feedback", feedback_payload(progress=0.4))
    clock.advance_s(1)
    fire_robot_message(hass, f"task/{TASK_ID}/feedback", feedback_payload(progress=0.9))
    await hass.async_block_till_done()
    assert ROBOT_ID in hub._feedback_pending
    clock.advance_s(1)
    await advance(1)
    assert ROBOT_ID in hub._feedback_pending, "2 s is inside a 3 s throttle"
    clock.advance_s(1.5)
    await advance(1)
    assert ROBOT_ID not in hub._feedback_pending


# --------------------------------------------------------------------------- heartbeat_stale_intervals


async def test_heartbeat_stale_intervals_option(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    clock: FakeClock,
    complete_hub_start: Any,
    started_hubs: list[BridgeHub],
) -> None:
    entry = make_config_entry(
        options={CONF_HEARTBEAT_STALE_INTERVALS: 5, CONF_HEARTBEAT_UNAVAILABLE_AFTER_S: 0}
    )
    hub = await start_hub(hass, entry, clock, complete_hub_start, started_hubs)
    await announce_online(hass)  # the golden announce heartbeats every 2 s
    robot = hub.robots[ROBOT_ID]
    clock.advance_s(7)
    assert hub._check_staleness(robot, hub.now) is False
    assert robot.heartbeat_stale is False, "three intervals (the default) are not five"
    clock.advance_s(3.5)
    assert hub._check_staleness(robot, hub.now) is True
    assert robot.heartbeat_stale is True
    assert robot.heartbeat_unavailable is False, "unavailability is off at 0"
    assert hub.problem_reasons(ROBOT_ID) == ["heartbeat_stale"]


# --------------------------------------------------------------------------- clock_skew_warn_ms


async def test_clock_skew_warn_option_sets_the_repair_threshold(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    clock: FakeClock,
    complete_hub_start: Any,
    started_hubs: list[BridgeHub],
) -> None:
    entry = make_config_entry(options={CONF_CLOCK_SKEW_WARN_MS: 3000})
    hub = await start_hub(hass, entry, clock, complete_hub_start, started_hubs)
    await announce_online(hass)
    await send(hub, TASK_ID)
    fire_robot_message(
        hass, f"task/{TASK_ID}/ack", ack_payload(id=TASK_ID, task="reset", clock_skew_ms=1500)
    )
    await hass.async_block_till_done()
    assert _issue(hass, ISSUE_CLOCK_SKEW) is None, "1500 ms is below the 3000 ms option"
    await send(hub, RESET_ID)
    fire_robot_message(
        hass, f"task/{RESET_ID}/ack", ack_payload(id=RESET_ID, task="reset", clock_skew_ms=-3000)
    )
    await hass.async_block_till_done()
    opened = _issue(hass, ISSUE_CLOCK_SKEW)
    assert opened is not None
    assert opened.translation_placeholders == {"robot_id": ROBOT_ID, "skew_ms": "-3000"}


# --------------------------------------------------------------------------- duplicate_*


async def test_duplicate_options_shape_the_heuristic_and_the_repair_text(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    clock: FakeClock,
    complete_hub_start: Any,
    advance: Callable[[float], Awaitable[None]],
    started_hubs: list[BridgeHub],
) -> None:
    entry = make_config_entry(
        options={
            CONF_DUPLICATE_SESSION_CHANGES: 1,
            CONF_DUPLICATE_SESSION_WINDOW_S: 90,
            CONF_DUPLICATE_QUIET_S: 120,
        }
    )
    hub = await start_hub(hass, entry, clock, complete_hub_start, started_hubs)
    await announce_online(hass)
    clock.advance_s(10)
    await announce_online(hass, session=OTHER_SESSION)
    assert _issue(hass, ISSUE_DUPLICATE_ROBOT_ID) is None, "one change is not more than one"
    clock.advance_s(10)
    await announce_online(hass, session=THIRD_SESSION)
    opened = _issue(hass, ISSUE_DUPLICATE_ROBOT_ID)
    assert opened is not None
    assert opened.translation_placeholders == {
        "robot_id": ROBOT_ID,
        "detail": "2 session changes in 90 s",
        "quiet_minutes": "2",
    }
    assert hub.robots[ROBOT_ID].duplicate_open is True

    # The quiet period of the option, not the default ten minutes.
    clock.advance_s(119)
    await advance(1)
    assert _issue(hass, ISSUE_DUPLICATE_ROBOT_ID) is not None
    clock.advance_s(1)
    await advance(1)
    assert _issue(hass, ISSUE_DUPLICATE_ROBOT_ID) is None
    assert hub.robots[ROBOT_ID].duplicate_open is False

    # Changes older than the window no longer count: one fresh change opens nothing.
    await announce_online(hass, session=FOURTH_SESSION)
    assert hub.robots[ROBOT_ID].session_changes == [hub.now]
    assert _issue(hass, ISSUE_DUPLICATE_ROBOT_ID) is None


# --------------------------------------------------------------------------- ack_expiry_s and result_expiry_s


async def test_expiry_options_reach_the_registry(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    clock: FakeClock,
    complete_hub_start: Any,
    started_hubs: list[BridgeHub],
) -> None:
    entry = make_config_entry(
        options={
            CONF_ACK_EXPIRY_S: 60,
            CONF_RESULT_EXPIRY_S: 900,
            CONF_RESULT_TIMEOUT_MARGIN_S: 10,
        }
    )
    hub = await start_hub(hass, entry, clock, complete_hub_start, started_hubs)
    assert hub.registry.settings.ack_expiry_s == 60.0
    assert hub.registry.settings.result_expiry_s == 900.0
    assert hub.registry.settings.retention_ms == 910_000
    await announce_online(hass)

    # A cancel waits for its late ack for the ack expiry.
    cancel = await hub.async_cancel_task(ROBOT_ID, None, source=CommandSource(kind="service"))
    assert cancel.sent_at is not None and cancel.deadline == cancel.sent_at + 60_000

    # A result id is remembered for the result expiry plus the margin.
    fire_robot_message(
        hass,
        f"task/{WAIT_ID}/result",
        result_payload(id=WAIT_ID, finished_at=hub.now, started_at=hub.now - 1000),
    )
    await hass.async_block_till_done()
    assert WAIT_ID in hub.registry.seen_results
    clock.advance_s(905)
    hub.registry.housekeeping()
    assert WAIT_ID in hub.registry.seen_results
    clock.advance_s(10)
    hub.registry.housekeeping()
    assert WAIT_ID not in hub.registry.seen_results


# --------------------------------------------------------------------------- defaults are the old constants


@pytest.mark.parametrize(
    ("key", "value"),
    [
        (CONF_ACK_EXPIRY_S, 120),
        (CONF_RESULT_EXPIRY_S, 300),
        (CONF_REPLAY_SETTLE_S, 3),
        (CONF_SESSION_CHANGE_GRACE_S, 1),
        (CONF_FEEDBACK_THROTTLE_MS, 500),
        (CONF_HEARTBEAT_STALE_INTERVALS, 3),
        (CONF_CLOCK_SKEW_WARN_MS, 1000),
        (CONF_DUPLICATE_SESSION_CHANGES, 3),
        (CONF_DUPLICATE_SESSION_WINDOW_S, 300),
        (CONF_DUPLICATE_QUIET_S, 600),
        (CONF_INTENT_ACK_WAIT_S, 1.5),
        (CONF_JOINT_MOVING_THRESHOLD, 0.0),
        (CONF_JOINT_MOVING_HOLD_S, 0.0),
        (CONF_MIRROR_EXTRA_ATTRIBUTES, []),
    ],
)
def test_defaults_keep_the_released_behaviour(key: str, value: Any) -> None:
    """0.1.0 shipped these as constants; the options default to exactly those values."""
    assert DEFAULT_OPTIONS[key] == value
