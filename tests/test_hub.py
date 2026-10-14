# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The hub against the mocked MQTT client: robots, repairs, catalogue changes, staleness, removal.

Messages after the first on a topic are fired without the retain flag, as a
broker delivers them live: Home Assistant's client hands a subscription the
retained copy of a topic once only, so a second retained message would be
dropped by the client before the hub ever saw it.
"""

from __future__ import annotations

import logging
import sys
import types
from collections.abc import Awaitable, Callable
from typing import Any
from unittest.mock import ANY, Mock

import pytest
from homeassistant.components.mqtt import MQTT_CONNECTION_STATE
from homeassistant.core import HomeAssistant, callback
from homeassistant.exceptions import HomeAssistantError, ServiceValidationError
from homeassistant.helpers import device_registry as dr
from homeassistant.helpers import entity_registry as er
from homeassistant.helpers import issue_registry as ir
from homeassistant.helpers.dispatcher import async_dispatcher_connect, async_dispatcher_send
from pytest_homeassistant_custom_component.common import (
    MockConfigEntry,
    async_capture_events,
)
from pytest_homeassistant_custom_component.syrupy import HomeAssistantSnapshotExtension
from syrupy.assertion import SnapshotAssertion

from custom_components.ros2_bridge import async_remove_config_entry_device
from custom_components.ros2_bridge.const import (
    CODE_NODE_RESTARTED,
    DOMAIN,
    EVENT_TASK_LATE_RESULT,
    EVENT_TASK_RESULT,
    ISSUE_DUPLICATE_ROBOT_ID,
    ISSUE_INVALID_ANNOUNCE,
    SIGNAL_ROBOT_TASKS_CHANGED,
    CommandState,
    announce_topic,
    availability_topic,
    issue_id,
    state_topic,
)
from custom_components.ros2_bridge.hub import BridgeHub, HubOptions
from custom_components.ros2_bridge.protocol import CommandSource

from .common import (
    OTHER_SESSION,
    PREFIX,
    ROBOT_ID,
    SESSION,
    TASK_ID,
    ack_payload,
    announce_payload,
    feedback_payload,
    fire_robot_message,
    heartbeat_payload,
    idle_state_payload,
    result_payload,
    state_payload,
)

THIRD_SESSION = "5b2d8f1e-7c3a-4e9b-8d6f-1a2b3c4d5e6f"
BUTTON_KEYS = ("task_reset", "task_stop", "task_pause", "task_resume", "cancel")


@pytest.fixture
def snapshot(snapshot: SnapshotAssertion) -> SnapshotAssertion:
    """Syrupy with Home Assistant's serializer (ids and timestamps masked, ``tests/snapshots``)."""
    return snapshot.use_extension(HomeAssistantSnapshotExtension)


@pytest.fixture(autouse=True)
def stub_registrars(monkeypatch: pytest.MonkeyPatch) -> None:
    """Stand in the services and intent modules; only the hub is under test here."""
    for name, function in (
        ("services", "async_register_services"),
        ("intent", "async_register_intents"),
    ):
        module = types.ModuleType(f"custom_components.{DOMAIN}.{name}")
        setattr(module, function, lambda hass: None)
        monkeypatch.setitem(sys.modules, module.__name__, module)


@pytest.fixture
async def hub(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    complete_hub_start: Callable[[BridgeHub], Awaitable[None]],
) -> BridgeHub:
    """A loaded entry whose hub is ready, connected and reconciled, with no robot yet."""
    entry = make_config_entry()
    assert await hass.config_entries.async_setup(entry.entry_id)
    await hass.async_block_till_done()
    started: BridgeHub = entry.runtime_data
    await complete_hub_start(started)
    return started


async def _announce_online(hass: HomeAssistant, **overrides: Any) -> None:
    fire_robot_message(hass, "announce", announce_payload(**overrides))
    fire_robot_message(hass, "availability", "online")
    await hass.async_block_till_done()


def _device(
    hass: HomeAssistant, hub: BridgeHub, robot_id: str = ROBOT_ID
) -> dr.DeviceEntry | None:
    return dr.async_get(hass).async_get_device_by_identifier(
        (DOMAIN, robot_id), hub.entry.entry_id
    )


def _entity_id(hass: HomeAssistant, domain: str, key: str, robot_id: str = ROBOT_ID) -> str:
    entity_id = er.async_get(hass).async_get_entity_id(domain, DOMAIN, f"{robot_id}_{key}")
    assert entity_id is not None, f"{domain} {robot_id}_{key} is not registered"
    return entity_id


def _state(hass: HomeAssistant, domain: str, key: str) -> str:
    state = hass.states.get(_entity_id(hass, domain, key))
    assert state is not None
    return state.state


async def _send_and_accept(hass: HomeAssistant, hub: BridgeHub, task: str = "reset") -> str:
    command = await hub.async_send_task(ROBOT_ID, task, {}, source=CommandSource(kind="service"))
    robot = hub.robots[ROBOT_ID]
    fire_robot_message(
        hass,
        f"task/{command.task_id}/ack",
        ack_payload(id=command.task_id, task=task, session=robot.session),
    )
    await hass.async_block_till_done()
    assert command.state is CommandState.RUNNING
    return command.task_id


# --------------------------------------------------------------------------- options


def test_hub_options_repr_hides_the_secret() -> None:
    options = HubOptions(shared_secret="s3cret-value")
    assert "s3cret" not in repr(options)
    assert options.secret == "s3cret-value"
    assert options.to_dict()["secret_configured"] is True
    assert "s3cret" not in str(options.to_dict())


# --------------------------------------------------------------------------- announce


async def test_announce_creates_device_and_entities(
    hass: HomeAssistant,
    hub: BridgeHub,
    entity_registry: er.EntityRegistry,
    snapshot: SnapshotAssertion,
) -> None:
    await _announce_online(hass)
    fire_robot_message(hass, "state", idle_state_payload())
    await hass.async_block_till_done()

    device = _device(hass, hub)
    assert device is not None
    assert device == snapshot(name="device")
    assert hub.robots[ROBOT_ID].device_id == device.id
    entries = sorted(
        er.async_entries_for_device(entity_registry, device.id), key=lambda e: e.entity_id
    )
    assert [entry.entity_id for entry in entries] == snapshot(name="entity_ids")
    unique_ids = {entry.unique_id for entry in entries}
    assert unique_ids == {
        f"{ROBOT_ID}_state",
        f"{ROBOT_ID}_last_result",
        f"{ROBOT_ID}_problem",
        f"{ROBOT_ID}_task_result",
        *(f"{ROBOT_ID}_{key}" for key in BUTTON_KEYS),
    }, "move_to needs a parameter, so it gets no button"
    for entry in entries:
        assert entry == snapshot(name=f"{entry.entity_id}-entry")
        state = hass.states.get(entry.entity_id)
        assert state is not None
        assert state == snapshot(name=f"{entry.entity_id}-state")
    assert _state(hass, "sensor", "state") == "idle"


async def test_invalid_announce_opens_the_repair_and_a_valid_one_closes_it(
    hass: HomeAssistant, hub: BridgeHub, issue_registry: ir.IssueRegistry
) -> None:
    fire_robot_message(hass, "announce", b"{not json")
    await hass.async_block_till_done()
    issue = issue_registry.async_get_issue(DOMAIN, issue_id(ISSUE_INVALID_ANNOUNCE, ROBOT_ID))
    assert issue is not None
    assert issue.translation_key == ISSUE_INVALID_ANNOUNCE
    assert issue.translation_placeholders == {
        "robot_id": ROBOT_ID,
        "topic": announce_topic(PREFIX, ROBOT_ID),
        "reason": ANY,
    }
    assert _device(hass, hub) is None
    assert hub.counters["invalid_payloads"] == 1

    # A payload whose robot_id differs from the topic is refused the same way.
    fire_robot_message(hass, "announce", announce_payload(robot_id="arm_right"))
    await hass.async_block_till_done()
    assert hub.counters["mismatched_payloads"] == 1
    assert "arm_right" not in hub.robots

    await _announce_online(hass)
    assert (
        issue_registry.async_get_issue(DOMAIN, issue_id(ISSUE_INVALID_ANNOUNCE, ROBOT_ID)) is None
    )
    assert _device(hass, hub) is not None


async def test_task_diff_adds_and_removes_buttons(
    hass: HomeAssistant, hub: BridgeHub, entity_registry: er.EntityRegistry
) -> None:
    diffs: list[tuple[str, tuple[str, ...], tuple[str, ...]]] = []

    @callback
    def _record(robot_id: str, added: tuple[str, ...], removed: tuple[str, ...]) -> None:
        diffs.append((robot_id, added, removed))

    async_dispatcher_connect(hass, SIGNAL_ROBOT_TASKS_CHANGED, _record)
    await _announce_online(hass)
    for key in BUTTON_KEYS:
        assert entity_registry.async_get_entity_id("button", DOMAIN, f"{ROBOT_ID}_{key}")
    assert (
        entity_registry.async_get_entity_id("button", DOMAIN, f"{ROBOT_ID}_task_move_to") is None
    )
    stop_entity_id = _entity_id(hass, "button", "task_stop")

    tasks = announce_payload()["tasks"]
    changed = [task for task in tasks if task["name"] != "stop"]
    changed.append(
        {"name": "wave", "label": "Wave", "kind": "motion", "timeout_s": 10, "params": None}
    )
    for task in changed:
        if task["name"] == "reset":
            task["params"] = {
                "type": "object",
                "properties": {"speed": {"type": "number"}},
                "required": ["speed"],
            }
    fire_robot_message(hass, "announce", announce_payload(tasks=changed))
    await hass.async_block_till_done()
    await hass.async_block_till_done()

    assert diffs == [(ROBOT_ID, ("wave",), ("stop",))]
    assert hub.counters["handler_errors"] == 0
    assert entity_registry.async_get_entity_id("button", DOMAIN, f"{ROBOT_ID}_task_wave")
    assert hass.states.get(_entity_id(hass, "button", "task_wave")) is not None
    assert entity_registry.async_get_entity_id("button", DOMAIN, f"{ROBOT_ID}_task_stop") is None
    assert hass.states.get(stop_entity_id) is None
    assert (
        entity_registry.async_get_entity_id("button", DOMAIN, f"{ROBOT_ID}_task_reset") is None
    ), "reset now needs a parameter"
    for key in ("task_pause", "task_resume", "cancel"):
        assert entity_registry.async_get_entity_id("button", DOMAIN, f"{ROBOT_ID}_{key}")


async def test_tombstone_removes_the_device_and_a_repeated_tombstone_is_a_no_op(
    hass: HomeAssistant, hub: BridgeHub, entity_registry: er.EntityRegistry
) -> None:
    await _announce_online(hass)
    device = _device(hass, hub)
    assert device is not None
    entity_ids = [e.entity_id for e in er.async_entries_for_device(entity_registry, device.id)]
    assert len(entity_ids) == 4 + len(BUTTON_KEYS)

    fire_robot_message(hass, "announce", b"")
    await hass.async_block_till_done()
    await hass.async_block_till_done()
    assert ROBOT_ID not in hub.robots
    assert hub.counters["handler_errors"] == 0
    assert _device(hass, hub) is None
    assert (
        er.async_entries_for_device(entity_registry, device.id, include_disabled_entities=True)
        == []
    )
    for entity_id in entity_ids:
        assert hass.states.get(entity_id) is None, entity_id

    fire_robot_message(hass, "announce", b"")
    await hass.async_block_till_done()
    assert ROBOT_ID not in hub.robots
    assert hub.counters["handler_errors"] == 0

    # The robot may come back; it gets its device and entities again.
    await _announce_online(hass)
    assert _device(hass, hub) is not None
    assert _state(hass, "binary_sensor", "problem") == "off"


# --------------------------------------------------------------------------- guard and timers


async def test_guard_counts_invalid_payloads_and_handler_errors(
    hass: HomeAssistant,
    hub: BridgeHub,
    caplog: pytest.LogCaptureFixture,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A bad payload is counted and a handler bug is logged; neither reaches the subscription."""
    fire_robot_message(hass, "heartbeat", b"not json")
    await hass.async_block_till_done()
    assert hub.counters["invalid_payloads"] == 1
    assert hub.robots == {}

    monkeypatch.setattr(hub, "_handle_state", Mock(side_effect=RuntimeError("boom")))
    fire_robot_message(hass, "state", idle_state_payload())
    await hass.async_block_till_done()
    assert hub.counters["handler_errors"] == 1
    assert "Error handling state" in caplog.text


async def test_heartbeat_publish_failure_is_counted_not_raised(
    hass: HomeAssistant,
    hub: BridgeHub,
    mqtt_setup: Any,
    advance: Callable[[float], Awaitable[None]],
) -> None:
    sent_before = hub.counters["heartbeats_sent"]
    errors_before = hub.counters["publish_errors"]
    mqtt_setup.async_publish.side_effect = HomeAssistantError("broker down")
    try:
        await advance(2)
    finally:
        mqtt_setup.async_publish.side_effect = None
    assert hub.counters["publish_errors"] == errors_before + 1
    assert hub.counters["heartbeats_sent"] == sent_before
    await advance(2)
    assert hub.counters["heartbeats_sent"] == sent_before + 1


async def test_throttled_feedback_is_flushed_by_housekeeping(
    hass: HomeAssistant, hub: BridgeHub, advance: Callable[[float], Awaitable[None]]
) -> None:
    """Two feedbacks within 500 ms: the second waits for the housekeeping tick."""
    await _announce_online(hass)
    fire_robot_message(hass, "state", state_payload())
    await hass.async_block_till_done()
    entity_id = _entity_id(hass, "sensor", "state")
    fire_robot_message(hass, f"task/{TASK_ID}/feedback", feedback_payload(progress=0.4))
    fire_robot_message(hass, f"task/{TASK_ID}/feedback", feedback_payload(progress=0.9))
    await hass.async_block_till_done()
    state = hass.states.get(entity_id)
    assert state is not None and state.attributes["progress"] == 0.4
    assert ROBOT_ID in hub._feedback_pending

    await advance(1)
    assert ROBOT_ID not in hub._feedback_pending
    state = hass.states.get(entity_id)
    assert state is not None and state.attributes["progress"] == 0.9


# --------------------------------------------------------------------------- commands out


async def test_unencodable_params_are_refused_before_tracking(
    hass: HomeAssistant, hub: BridgeHub, mqtt_setup: Any
) -> None:
    """A parameter value JSON cannot encode is invalid_params and leaves no trace."""
    await _announce_online(hass)
    mqtt_setup.async_publish.reset_mock()
    with pytest.raises(ServiceValidationError) as excinfo:
        await hub.async_send_task(
            ROBOT_ID, "reset", {"when": object()}, source=CommandSource(kind="service")
        )
    assert excinfo.value.translation_domain == DOMAIN
    assert excinfo.value.translation_key == "invalid_params"
    assert excinfo.value.translation_placeholders is not None
    assert excinfo.value.translation_placeholders["task"] == "reset"
    assert hub.registry.commands == {}
    mqtt_setup.async_publish.assert_not_called()


async def test_forget_robot_counts_failed_tombstones(
    hass: HomeAssistant, hub: BridgeHub, mqtt_setup: Any
) -> None:
    await _announce_online(hass)
    fire_robot_message(hass, "availability", "offline")
    await hass.async_block_till_done()
    mqtt_setup.async_publish.side_effect = HomeAssistantError("broker down")
    try:
        await hub.async_forget_robot(ROBOT_ID)
    finally:
        mqtt_setup.async_publish.side_effect = None
    assert hub.counters["publish_errors"] == 3
    assert ROBOT_ID not in hub.robots
    assert _device(hass, hub) is None


# --------------------------------------------------------------------------- topic segments


async def test_ha_segment_is_ignored_and_bad_segments_are_discarded(
    hass: HomeAssistant, hub: BridgeHub, caplog: pytest.LogCaptureFixture
) -> None:
    caplog.set_level(logging.DEBUG, logger=f"custom_components.{DOMAIN}")
    fire_robot_message(hass, "heartbeat", heartbeat_payload(), robot_id="ha")
    fire_robot_message(hass, "announce", announce_payload(), robot_id="ha")
    fire_robot_message(hass, "state", state_payload(), robot_id="ha")
    await hass.async_block_till_done()
    assert "ha" not in hub.robots
    assert hub.robots == {}
    assert hub.counters["discarded_segments"] == 0
    assert hub.counters["invalid_payloads"] == 0
    ours = [r for r in caplog.records if r.name.startswith(f"custom_components.{DOMAIN}")]
    assert not [r for r in ours if f"{PREFIX}/ha/" in r.getMessage()]

    fire_robot_message(hass, "state", state_payload(), robot_id="Arm-Left")
    await hass.async_block_till_done()
    assert hub.robots == {}
    assert hub.counters["discarded_segments"] == 1


# --------------------------------------------------------------------------- availability and state


async def test_availability_offline_makes_the_robot_unavailable(
    hass: HomeAssistant, hub: BridgeHub
) -> None:
    await _announce_online(hass)
    fire_robot_message(hass, "state", idle_state_payload())
    await hass.async_block_till_done()
    assert _state(hass, "sensor", "state") == "idle"
    assert hub.robot_summary(ROBOT_ID)["available"] is True

    fire_robot_message(hass, "availability", "offline")
    await hass.async_block_till_done()
    assert hub.robot_available(ROBOT_ID) is False
    assert _state(hass, "sensor", "state") == "unavailable"
    assert _state(hass, "button", "cancel") == "unavailable"
    with pytest.raises(Exception) as excinfo:
        await hub.async_send_task(ROBOT_ID, "reset", {}, source=CommandSource(kind="service"))
    assert getattr(excinfo.value, "translation_key", None) == "robot_unavailable"

    fire_robot_message(hass, "availability", "online")
    await hass.async_block_till_done()
    assert _state(hass, "sensor", "state") == "idle"


async def test_state_seq_regression_is_dropped(hass: HomeAssistant, hub: BridgeHub) -> None:
    await _announce_online(hass)
    fire_robot_message(hass, "state", state_payload(seq=42))
    await hass.async_block_till_done()
    assert _state(hass, "sensor", "state") == "busy"

    fire_robot_message(hass, "state", idle_state_payload(seq=41))
    await hass.async_block_till_done()
    assert _state(hass, "sensor", "state") == "busy"
    assert hub.counters["seq_regressions"] == 1

    fire_robot_message(hass, "state", idle_state_payload(seq=43))
    await hass.async_block_till_done()
    assert _state(hass, "sensor", "state") == "idle"

    # A new session restarts the sequence, so a lower seq is accepted again.
    fire_robot_message(hass, "state", state_payload(seq=1, session=OTHER_SESSION))
    await hass.async_block_till_done()
    assert _state(hass, "sensor", "state") == "busy"
    assert hub.counters["seq_regressions"] == 1


# --------------------------------------------------------------------------- sessions and commands


async def test_session_change_marks_commands_lost_after_grace(
    hass: HomeAssistant, hub: BridgeHub, advance: Callable[[float], Awaitable[None]]
) -> None:
    results = async_capture_events(hass, EVENT_TASK_RESULT)
    await _announce_online(hass)
    task_id = await _send_and_accept(hass, hub)

    # The handler runs inline, so the state can be checked before the loop turns.
    fire_robot_message(hass, "announce", announce_payload(session=OTHER_SESSION))
    command = hub.registry.get(task_id)
    assert command is not None
    assert command.state is CommandState.RUNNING, "the grace period has not passed"
    assert ROBOT_ID in hub._grace_handles
    await advance(1.5)
    assert command.state is CommandState.LOST
    assert ROBOT_ID not in hub._grace_handles
    assert len(results) == 1
    assert results[0].data["status"] == "lost"
    assert results[0].data["code"] == CODE_NODE_RESTARTED
    assert results[0].data["synthetic"] is True
    assert hub.robots[ROBOT_ID].session == OTHER_SESSION
    assert hub.robots[ROBOT_ID].session_change_count == 1


async def test_session_change_does_not_lose_a_command_whose_result_arrived(
    hass: HomeAssistant, hub: BridgeHub, advance: Callable[[float], Awaitable[None]]
) -> None:
    results = async_capture_events(hass, EVENT_TASK_RESULT)
    late = async_capture_events(hass, EVENT_TASK_LATE_RESULT)
    await _announce_online(hass)
    task_id = await _send_and_accept(hass, hub)

    fire_robot_message(hass, "announce", announce_payload(session=OTHER_SESSION))
    now = hub.now
    fire_robot_message(
        hass,
        f"task/{task_id}/result",
        result_payload(id=task_id, task="reset", started_at=now - 1500, finished_at=now),
        retain=True,
    )
    await hass.async_block_till_done()
    await advance(2)
    command = hub.registry.get(task_id)
    assert command is not None
    assert command.state is CommandState.SUCCEEDED
    assert [event.data["status"] for event in results] == ["succeeded"]
    assert late == []
    assert hub.robots[ROBOT_ID].last_result is not None
    assert str(hub.robots[ROBOT_ID].last_result.outcome) == "succeeded"


async def test_retained_result_replay_restores_last_result_without_an_event(
    hass: HomeAssistant,
    hub: BridgeHub,
    complete_hub_start: Callable[[BridgeHub], Awaitable[None]],
) -> None:
    """After a reload the retained result comes back: no second event, but last_result is restored."""
    results = async_capture_events(hass, EVENT_TASK_RESULT)
    await _announce_online(hass)
    task_id = await _send_and_accept(hass, hub)
    now = hub.now
    payload = result_payload(id=task_id, task="reset", started_at=now - 1500, finished_at=now)
    fire_robot_message(hass, f"task/{task_id}/result", payload, retain=True)
    await hass.async_block_till_done()
    assert len(results) == 1
    assert hub.robots[ROBOT_ID].last_result is not None

    # An options reload: a fresh hub with fresh subscriptions, so the broker replays the
    # retained announce, availability and result (HA's client only delivers a retained
    # message once per subscription, which is why the reload is needed here too).
    entry = hub.entry
    assert await hass.config_entries.async_reload(entry.entry_id)
    await hass.async_block_till_done()
    reloaded: BridgeHub = entry.runtime_data
    assert reloaded is not hub
    await complete_hub_start(reloaded)
    await _announce_online(hass)
    assert reloaded.robots[ROBOT_ID].last_result is None, "the sensor would read unknown"
    fire_robot_message(hass, f"task/{task_id}/result", payload, retain=True)
    await hass.async_block_till_done()
    assert len(results) == 1, "a replayed result never fires a second event"
    restored = reloaded.robots[ROBOT_ID].last_result
    assert restored is not None and restored.task_id == task_id
    assert str(restored.outcome) == "succeeded" and restored.synthetic is False
    assert reloaded.registry.counters["replayed_results_ignored"] >= 1


# --------------------------------------------------------------------------- heartbeat staleness


async def test_heartbeat_staleness_turns_problem_on_then_unavailable(
    hass: HomeAssistant, hub: BridgeHub, advance: Callable[[float], Awaitable[None]]
) -> None:
    await _announce_online(hass)
    fire_robot_message(hass, "state", idle_state_payload())
    await hass.async_block_till_done()
    assert _state(hass, "binary_sensor", "problem") == "off"

    await advance(7)  # more than three 2 s intervals without a heartbeat
    assert hub.robots[ROBOT_ID].heartbeat_stale is True
    assert hub.problem_reasons(ROBOT_ID) == ["heartbeat_stale"]
    assert _state(hass, "binary_sensor", "problem") == "on"
    assert _state(hass, "sensor", "state") == "idle", "stale alone keeps the robot available"

    await advance(24)  # past heartbeat_unavailable_after_s (30)
    assert hub.robots[ROBOT_ID].heartbeat_unavailable is True
    assert hub.robot_available(ROBOT_ID) is False
    assert _state(hass, "sensor", "state") == "unavailable"
    assert _state(hass, "binary_sensor", "problem") == "unavailable"

    fire_robot_message(hass, "heartbeat", heartbeat_payload())
    await hass.async_block_till_done()
    assert hub.robots[ROBOT_ID].heartbeat_stale is False
    assert hub.robots[ROBOT_ID].heartbeat_unavailable is False
    assert _state(hass, "sensor", "state") == "idle"
    assert _state(hass, "binary_sensor", "problem") == "off"


async def test_heartbeat_interval_zero_disables_staleness(
    hass: HomeAssistant, hub: BridgeHub, advance: Callable[[float], Awaitable[None]]
) -> None:
    await _announce_online(hass, heartbeat_interval_s=0)
    fire_robot_message(hass, "state", idle_state_payload())
    await hass.async_block_till_done()
    await advance(120)
    assert hub.robots[ROBOT_ID].heartbeat_stale is False
    assert hub.robots[ROBOT_ID].heartbeat_unavailable is False
    assert _state(hass, "sensor", "state") == "idle"
    assert _state(hass, "binary_sensor", "problem") == "off"


# --------------------------------------------------------------------------- connection


async def test_mqtt_disconnect_makes_everything_unavailable(
    hass: HomeAssistant, hub: BridgeHub
) -> None:
    await _announce_online(hass)
    fire_robot_message(hass, "state", idle_state_payload())
    await hass.async_block_till_done()
    assert _state(hass, "sensor", "state") == "idle"

    async_dispatcher_send(hass, MQTT_CONNECTION_STATE, False)
    await hass.async_block_till_done()
    assert hub.mqtt_connected is False
    for domain, key in (
        ("sensor", "state"),
        ("sensor", "last_result"),
        ("binary_sensor", "problem"),
        ("event", "task_result"),
        ("button", "task_reset"),
        ("button", "cancel"),
    ):
        assert _state(hass, domain, key) == "unavailable", (domain, key)

    async_dispatcher_send(hass, MQTT_CONNECTION_STATE, True)
    await hass.async_block_till_done()
    assert _state(hass, "sensor", "state") == "idle"
    assert _state(hass, "event", "task_result") == "unknown"


# --------------------------------------------------------------------------- device removal


async def test_device_removal_rules(hass: HomeAssistant, hub: BridgeHub, mqtt_setup: Any) -> None:
    await _announce_online(hass)
    device = _device(hass, hub)
    assert device is not None
    mqtt_setup.async_publish.reset_mock()

    assert await async_remove_config_entry_device(hass, hub.entry, device) is False
    mqtt_setup.async_publish.assert_not_called()
    assert ROBOT_ID in hub.robots

    fire_robot_message(hass, "availability", "offline")
    await hass.async_block_till_done()
    assert hub.is_removable(ROBOT_ID)
    assert await async_remove_config_entry_device(hass, hub.entry, device) is True
    await hass.async_block_till_done()
    for topic in (
        announce_topic(PREFIX, ROBOT_ID),
        state_topic(PREFIX, ROBOT_ID),
        availability_topic(PREFIX, ROBOT_ID),
    ):
        mqtt_setup.async_publish.assert_any_call(topic, b"", 1, True, message_expiry_interval=None)
    assert ROBOT_ID not in hub.robots
    assert _device(hass, hub) is None

    # An unknown robot (a stale device from before a restart) is removable too.
    ghost = dr.async_get(hass).async_get_or_create(
        config_entry_id=hub.entry.entry_id, identifiers={(DOMAIN, "ghost")}, name="ghost"
    )
    assert await async_remove_config_entry_device(hass, hub.entry, ghost) is True
    mqtt_setup.async_publish.assert_any_call(
        announce_topic(PREFIX, "ghost"), b"", 1, True, message_expiry_interval=None
    )


# --------------------------------------------------------------------------- duplicate robot id


async def test_duplicate_robot_id_from_a_differing_instance(
    hass: HomeAssistant,
    hub: BridgeHub,
    issue_registry: ir.IssueRegistry,
    advance: Callable[[float], Awaitable[None]],
) -> None:
    await _announce_online(hass)
    assert (
        issue_registry.async_get_issue(DOMAIN, issue_id(ISSUE_DUPLICATE_ROBOT_ID, ROBOT_ID))
        is None
    )

    fire_robot_message(hass, "heartbeat", heartbeat_payload(instance="otherpc:77"))
    await hass.async_block_till_done()
    issue = issue_registry.async_get_issue(DOMAIN, issue_id(ISSUE_DUPLICATE_ROBOT_ID, ROBOT_ID))
    assert issue is not None
    assert issue.severity is ir.IssueSeverity.ERROR
    assert issue.translation_placeholders == {
        "robot_id": ROBOT_ID,
        "detail": "instances robotpc:4123 and otherpc:77",
        "quiet_minutes": "10",
    }
    assert "duplicate_robot_id" in hub.problem_reasons(ROBOT_ID)
    assert _state(hass, "binary_sensor", "problem") == "on"

    await advance(601)  # ten quiet minutes close it
    assert (
        issue_registry.async_get_issue(DOMAIN, issue_id(ISSUE_DUPLICATE_ROBOT_ID, ROBOT_ID))
        is None
    )
    assert "duplicate_robot_id" not in hub.problem_reasons(ROBOT_ID)


async def test_duplicate_robot_id_from_session_flapping(
    hass: HomeAssistant, hub: BridgeHub, issue_registry: ir.IssueRegistry
) -> None:
    sessions = [
        SESSION,
        OTHER_SESSION,
        THIRD_SESSION,
        "0a1b2c3d-4e5f-4a6b-8c7d-9e0f1a2b3c4d",
    ]
    for session in sessions:
        await _announce_online(hass, session=session)
    assert hub.robots[ROBOT_ID].session_change_count == 3
    assert (
        issue_registry.async_get_issue(DOMAIN, issue_id(ISSUE_DUPLICATE_ROBOT_ID, ROBOT_ID))
        is None
    )

    await _announce_online(hass, session="1b2c3d4e-5f6a-4b7c-9d8e-0f1a2b3c4d5e")
    assert hub.robots[ROBOT_ID].session_change_count == 4
    issue = issue_registry.async_get_issue(DOMAIN, issue_id(ISSUE_DUPLICATE_ROBOT_ID, ROBOT_ID))
    assert issue is not None
    assert issue.translation_placeholders == {
        "robot_id": ROBOT_ID,
        "detail": "4 session changes in 5 minutes",
        "quiet_minutes": "10",
    }
    assert hub.robots[ROBOT_ID].duplicate_open is True
