# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The entity platforms: enum sensors, the problem sensor, the task_result event and the buttons.

Messages after the first on a topic are fired without the retain flag, as a
broker delivers them live: Home Assistant's client hands a subscription the
retained copy of a topic once only.
"""

from __future__ import annotations

import asyncio
import json
import sys
import types
from collections.abc import Awaitable, Callable
from typing import Any
from unittest.mock import ANY

import pytest
from homeassistant.core import HomeAssistant, is_callback
from homeassistant.exceptions import HomeAssistantError
from homeassistant.helpers import entity_registry as er
from homeassistant.helpers.dispatcher import async_dispatcher_send
from pytest_homeassistant_custom_component.common import (
    MockConfigEntry,
    async_capture_events,
)

from custom_components.ros2_bridge import binary_sensor, button, event, sensor
from custom_components.ros2_bridge.binary_sensor import ProblemSensor
from custom_components.ros2_bridge.button import CancelButton, TaskButton
from custom_components.ros2_bridge.const import (
    CODE_NO_ACK,
    DOMAIN,
    EVENT_TASK_LATE_RESULT,
    EVENT_TASK_RESULT,
    SIGNAL_ROBOT_ADDED,
    CommandState,
    cmd_topic,
)
from custom_components.ros2_bridge.entity import Ros2BridgeEntity
from custom_components.ros2_bridge.event import TaskResultEvent
from custom_components.ros2_bridge.hub import BridgeHub
from custom_components.ros2_bridge.protocol import CommandSource
from custom_components.ros2_bridge.sensor import (
    STATE_UNRECORDED_ATTRIBUTES,
    LastResultSensor,
    StateSensor,
)

from .common import (
    PREFIX,
    ROBOT_ID,
    ack_payload,
    announce_payload,
    cancel_ack_payload,
    feedback_payload,
    fire_robot_message,
    idle_state_payload,
    rejected_ack_payload,
    result_payload,
    state_payload,
)

OUTCOMES = ["succeeded", "cancelled", "aborted", "rejected", "timeout", "lost"]


@pytest.fixture(autouse=True)
def stub_registrars(monkeypatch: pytest.MonkeyPatch) -> None:
    """Stand in the services and intent modules; only the platforms are under test here."""
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
    """A ready hub with the golden robot announced, online and idle."""
    entry = make_config_entry()
    assert await hass.config_entries.async_setup(entry.entry_id)
    await hass.async_block_till_done()
    started: BridgeHub = entry.runtime_data
    await complete_hub_start(started)
    fire_robot_message(hass, "announce", announce_payload())
    fire_robot_message(hass, "availability", "online")
    fire_robot_message(hass, "state", idle_state_payload())
    await hass.async_block_till_done()
    return started


def _entity_id(hass: HomeAssistant, domain: str, key: str) -> str:
    entity_id = er.async_get(hass).async_get_entity_id(domain, DOMAIN, f"{ROBOT_ID}_{key}")
    assert entity_id is not None, f"{domain} {ROBOT_ID}_{key} is not registered"
    return entity_id


async def _until(predicate: Callable[[], bool], attempts: int = 500) -> None:
    """Yield to the loop until ``predicate`` holds (everything here is in-loop, no real I/O)."""
    for _ in range(attempts):
        if predicate():
            return
        await asyncio.sleep(0)
    raise AssertionError("the condition was not reached")


async def _sent_command(hub: BridgeHub) -> Any:
    """Wait until the button's command has been published and marked sent, then return it."""

    def _one_sent() -> bool:
        return any(c.state is CommandState.SENT for c in hub.registry.outstanding(ROBOT_ID))

    await _until(_one_sent)
    return next(c for c in hub.registry.outstanding(ROBOT_ID) if c.state is CommandState.SENT)


async def _result(
    hass: HomeAssistant, hub: BridgeHub, task_id: str, *, task: str, kind: str, status: str
) -> None:
    goal = {"succeeded": 4, "cancelled": 5, "aborted": 6}[status]
    now = hub.now
    fire_robot_message(
        hass,
        f"task/{task_id}/result",
        result_payload(
            id=task_id,
            task=task,
            kind=kind,
            status=status,
            goal_status=goal,
            started_at=now - 1000,
            finished_at=now,
        ),
        retain=True,
    )
    await hass.async_block_till_done()


# --------------------------------------------------------------------------- sensors


async def test_state_sensor_options_attributes_and_unrecorded_attributes(
    hass: HomeAssistant, hub: BridgeHub
) -> None:
    entity_id = _entity_id(hass, "sensor", "state")
    state = hass.states.get(entity_id)
    assert state is not None
    assert state.state == "idle"
    assert state.attributes["options"] == ["idle", "busy", "error"]
    assert state.attributes["device_class"] == "enum"
    assert state.attributes["task_id"] is None
    assert state.attributes["progress"] is None

    fire_robot_message(hass, "state", state_payload(seq=44))
    fire_robot_message(
        hass, "task/6a0c2b58-1d7e-4a1f-9b2a-3c4d5e6f7a8b/feedback", feedback_payload()
    )
    await hass.async_block_till_done()
    state = hass.states.get(entity_id)
    assert state is not None
    assert state.state == "busy"
    assert state.attributes["task_id"] == "6a0c2b58-1d7e-4a1f-9b2a-3c4d5e6f7a8b"
    assert state.attributes["task"] == "move_to"
    assert state.attributes["paused"] is False
    assert state.attributes["inhibited"] is False
    assert state.attributes["ha_link"] == "up"
    assert state.attributes["watchdog_mode"] == "none"
    assert state.attributes["executor_health"] == {"level": "ok", "message": ""}
    assert state.attributes["error_code"] is None
    assert state.attributes["session"] == state_payload()["session"]
    assert state.attributes["instance"] == "robotpc:4123"
    assert state.attributes["progress"] == 0.4
    assert state.attributes["stage"] == "executing"
    assert state.attributes["message"] == "waypoint 2 of 5"
    assert state.state_info is not None
    assert STATE_UNRECORDED_ATTRIBUTES <= state.state_info["unrecorded_attributes"]
    assert StateSensor._unrecorded_attributes == frozenset(
        {"progress", "stage", "message", "since", "session"}
    )

    # Feedback is cleared when the active task changes.
    fire_robot_message(hass, "state", idle_state_payload(seq=50))
    await hass.async_block_till_done()
    state = hass.states.get(entity_id)
    assert state is not None
    assert state.state == "idle"
    assert state.attributes["progress"] is None
    assert state.attributes["stage"] is None


async def test_last_result_sensor_follows_motion_tasks_only(
    hass: HomeAssistant, hub: BridgeHub
) -> None:
    entity_id = _entity_id(hass, "sensor", "last_result")
    state = hass.states.get(entity_id)
    assert state is not None
    assert state.state == "unknown"
    assert state.attributes["options"] == OUTCOMES
    assert state.attributes["device_class"] == "enum"

    motion = await hub.async_send_task(ROBOT_ID, "reset", {}, source=CommandSource(kind="service"))
    fire_robot_message(
        hass, f"task/{motion.task_id}/ack", ack_payload(id=motion.task_id, task="reset")
    )
    await hass.async_block_till_done()
    await _result(hass, hub, motion.task_id, task="reset", kind="motion", status="aborted")
    state = hass.states.get(entity_id)
    assert state is not None
    assert state.state == "aborted"
    assert state.attributes["task_id"] == motion.task_id
    assert state.attributes["task"] == "reset"
    assert state.attributes["code"] == "ok"
    assert state.attributes["duration_ms"] == 1000
    assert state.attributes["synthetic"] is False
    assert state.attributes["data"] == {}

    control = await hub.async_send_task(ROBOT_ID, "stop", {}, source=CommandSource(kind="service"))
    fire_robot_message(
        hass, f"task/{control.task_id}/ack", ack_payload(id=control.task_id, task="stop")
    )
    await hass.async_block_till_done()
    await _result(hass, hub, control.task_id, task="stop", kind="control", status="succeeded")
    state = hass.states.get(entity_id)
    assert state is not None
    assert state.state == "aborted", "a control result never touches last_result"
    assert state.attributes["task_id"] == motion.task_id


async def test_problem_binary_sensor(hass: HomeAssistant, hub: BridgeHub) -> None:
    entity_id = _entity_id(hass, "binary_sensor", "problem")
    state = hass.states.get(entity_id)
    assert state is not None
    assert state.state == "off"
    assert state.attributes["device_class"] == "problem"
    assert state.attributes["reasons"] == []

    fire_robot_message(
        hass,
        "state",
        idle_state_payload(
            state="error",
            seq=60,
            error={"code": "timeout", "reason": "forced finish"},
            executor_health={"level": "error", "message": "stuck"},
        ),
    )
    await hass.async_block_till_done()
    state = hass.states.get(entity_id)
    assert state is not None
    assert state.state == "on"
    assert state.attributes["reasons"] == ["error", "executor_error"]
    assert hass.states.get(_entity_id(hass, "sensor", "state")).state == "error"  # type: ignore[union-attr]

    fire_robot_message(hass, "state", idle_state_payload(seq=61))
    await hass.async_block_till_done()
    state = hass.states.get(entity_id)
    assert state is not None
    assert state.state == "off"


# --------------------------------------------------------------------------- lifecycle hooks


@pytest.mark.parametrize(
    "entity_class", [StateSensor, LastResultSensor, ProblemSensor, TaskResultEvent, CancelButton]
)
def test_async_entity_id_changed_is_present_and_a_callback(
    entity_class: type[Ros2BridgeEntity],
) -> None:
    hook = entity_class.async_entity_id_changed
    assert callable(hook)
    assert is_callback(hook)
    assert hook is Ros2BridgeEntity.async_entity_id_changed, "one implementation for every entity"


async def test_async_entity_id_changed_runs_on_a_live_entity(
    hass: HomeAssistant, hub: BridgeHub
) -> None:
    entity = StateSensor(hub, ROBOT_ID)
    entity.hass = hass
    # No parent hook exists at 2026.10.1, so this must simply not raise.
    entity.async_entity_id_changed("sensor.old_name")
    assert entity.unique_id == f"{ROBOT_ID}_state"
    assert entity.device_info == hub.device_info(ROBOT_ID)
    assert entity.available is True
    for button_class in (TaskButton, CancelButton):
        assert button_class.async_entity_id_changed is Ros2BridgeEntity.async_entity_id_changed


# --------------------------------------------------------------------------- event entity


async def test_event_entity_types_and_trigger(hass: HomeAssistant, hub: BridgeHub) -> None:
    entity_id = _entity_id(hass, "event", "task_result")
    state = hass.states.get(entity_id)
    assert state is not None
    assert state.state == "unknown"
    assert state.attributes["event_types"] == OUTCOMES

    command = await hub.async_send_task(
        ROBOT_ID, "reset", {}, source=CommandSource(kind="service")
    )
    fire_robot_message(
        hass, f"task/{command.task_id}/ack", ack_payload(id=command.task_id, task="reset")
    )
    await hass.async_block_till_done()
    await _result(hass, hub, command.task_id, task="reset", kind="motion", status="succeeded")
    state = hass.states.get(entity_id)
    assert state is not None
    assert state.state != "unknown"
    assert state.attributes["event_type"] == "succeeded"
    assert state.attributes["task_id"] == command.task_id
    assert state.attributes["task"] == "reset"
    assert state.attributes["kind"] == "motion"
    assert state.attributes["code"] == "ok"
    assert state.attributes["goal_status"] == 4
    assert state.attributes["replayed"] is True
    assert state.attributes["synthetic"] is False

    # A rejected ack is an outcome too.
    rejected = await hub.async_send_task(
        ROBOT_ID, "reset", {}, source=CommandSource(kind="service")
    )
    fire_robot_message(
        hass,
        f"task/{rejected.task_id}/ack",
        rejected_ack_payload(id=rejected.task_id, task="reset"),
    )
    await hass.async_block_till_done()
    state = hass.states.get(entity_id)
    assert state is not None
    assert state.attributes["event_type"] == "rejected"
    assert state.attributes["code"] == "rejected_busy"


async def test_event_entity_ignores_late_results(
    hass: HomeAssistant, hub: BridgeHub, advance: Callable[[float], Awaitable[None]]
) -> None:
    """A synthesised no-ack timeout is the one outcome; the real result arrives as late_result."""
    entity_id = _entity_id(hass, "event", "task_result")
    results = async_capture_events(hass, EVENT_TASK_RESULT)
    late = async_capture_events(hass, EVENT_TASK_LATE_RESULT)
    command = await hub.async_send_task(
        ROBOT_ID, "reset", {}, source=CommandSource(kind="service")
    )
    assert command.state is CommandState.SENT
    await advance(3)  # past ack_timeout_s (2)
    assert command.state is CommandState.TIMEOUT_NO_ACK
    assert [event.data["status"] for event in results] == ["timeout"]
    state = hass.states.get(entity_id)
    assert state is not None
    assert state.attributes["event_type"] == "timeout"
    assert state.attributes["code"] == CODE_NO_ACK
    assert state.attributes["synthetic"] is True
    triggered_at = state.state

    await _result(hass, hub, command.task_id, task="reset", kind="motion", status="succeeded")
    assert command.state is CommandState.TIMEOUT_NO_ACK, "a late result corrects, never re-opens"
    assert command.result is not None
    assert len(results) == 1 and len(late) == 1
    assert late[0].data["status"] == "succeeded"
    state = hass.states.get(entity_id)
    assert state is not None
    assert state.attributes["event_type"] == "timeout"
    assert state.state == triggered_at


# --------------------------------------------------------------------------- buttons


async def _press(hass: HomeAssistant, entity_id: str) -> asyncio.Task[None]:
    return hass.async_create_task(
        hass.services.async_call("button", "press", {"entity_id": entity_id}, blocking=True)
    )


async def test_button_press_sends_the_task_and_waits_for_the_ack(
    hass: HomeAssistant, hub: BridgeHub, mqtt_setup: Any
) -> None:
    entity_id = _entity_id(hass, "button", "task_reset")
    state = hass.states.get(entity_id)
    assert state is not None
    assert state.attributes["friendly_name"] == "Left arm Reset to home"
    mqtt_setup.async_publish.reset_mock()

    press = await _press(hass, entity_id)
    command = await _sent_command(hub)
    mqtt_setup.async_publish.assert_called_once_with(
        cmd_topic(PREFIX, ROBOT_ID), ANY, 1, False, message_expiry_interval=5
    )
    payload = json.loads(mqtt_setup.async_publish.call_args.args[1])
    assert payload["id"] == command.task_id
    assert payload["type"] == "task"
    assert payload["task"] == "reset"
    assert payload["params"] == {}
    assert payload["timeout_s"] is None
    assert payload["source"]["kind"] == "button"
    assert payload["source"]["origin"] == "ha"
    assert payload["source"]["instance_id"] == hub.instance_id
    assert "sig" not in payload
    assert not press.done()

    fire_robot_message(
        hass, f"task/{command.task_id}/ack", ack_payload(id=command.task_id, task="reset")
    )
    await press
    assert command.state is CommandState.RUNNING
    state = hass.states.get(entity_id)
    assert state is not None
    assert state.state != "unknown", "a successful press records its timestamp"


async def test_button_press_rejection_raises_task_rejected(
    hass: HomeAssistant, hub: BridgeHub, mqtt_setup: Any
) -> None:
    entity_id = _entity_id(hass, "button", "task_reset")
    press = await _press(hass, entity_id)
    command = await _sent_command(hub)
    fire_robot_message(
        hass,
        f"task/{command.task_id}/ack",
        rejected_ack_payload(id=command.task_id, task="reset"),
    )
    with pytest.raises(HomeAssistantError) as excinfo:
        await press
    assert excinfo.value.translation_domain == DOMAIN
    assert excinfo.value.translation_key == "task_rejected"
    assert excinfo.value.translation_placeholders == {
        "robot_id": ROBOT_ID,
        "task": "reset",
        "code": "rejected_busy",
        "reason": ANY,
    }
    assert command.state is CommandState.REJECTED


async def test_button_press_without_ack_raises_no_ack(
    hass: HomeAssistant,
    hub: BridgeHub,
    advance: Callable[[float], Awaitable[None]],
) -> None:
    entity_id = _entity_id(hass, "button", "task_reset")
    press = await _press(hass, entity_id)
    command = await _sent_command(hub)
    await advance(3)  # the registry's no-ack timer resolves the wait
    assert command.state is CommandState.TIMEOUT_NO_ACK
    with pytest.raises(HomeAssistantError) as excinfo:
        await press
    assert excinfo.value.translation_key == "no_ack"
    assert excinfo.value.translation_placeholders == {"robot_id": ROBOT_ID, "task": "reset"}


async def test_button_press_while_the_hub_stops_raises_not_loaded(
    hass: HomeAssistant, hub: BridgeHub
) -> None:
    """An options save mid-press cancels the wait; the press fails with not_loaded, not silence."""
    entity_id = _entity_id(hass, "button", "task_reset")
    press = await _press(hass, entity_id)
    command = await _sent_command(hub)
    await _until(lambda: bool(hub.registry._ack_waiters.get(command.task_id)))
    await hub.async_stop()
    with pytest.raises(HomeAssistantError) as excinfo:
        await press
    assert excinfo.value.translation_domain == DOMAIN
    assert excinfo.value.translation_key == "not_loaded"
    assert command.state is CommandState.SENT, "the command itself stays tracked"


async def test_cancel_button_sends_a_cancel_and_completes_on_its_ack(
    hass: HomeAssistant, hub: BridgeHub, mqtt_setup: Any
) -> None:
    entity_id = _entity_id(hass, "button", "cancel")
    state = hass.states.get(entity_id)
    assert state is not None
    assert state.attributes["friendly_name"] == "Left arm Cancel task"
    mqtt_setup.async_publish.reset_mock()
    press = await _press(hass, entity_id)
    command = await _sent_command(hub)
    mqtt_setup.async_publish.assert_called_once_with(
        cmd_topic(PREFIX, ROBOT_ID), ANY, 1, False, message_expiry_interval=5
    )
    payload = json.loads(mqtt_setup.async_publish.call_args.args[1])
    assert payload["id"] == command.task_id
    assert payload["type"] == "cancel"
    assert payload["target_id"] is None
    assert payload["source"]["kind"] == "button"
    fire_robot_message(hass, f"task/{command.task_id}/ack", cancel_ack_payload(id=command.task_id))
    await press
    assert command.state is CommandState.DONE


async def test_button_press_while_unavailable_sends_nothing(
    hass: HomeAssistant, hub: BridgeHub, mqtt_setup: Any
) -> None:
    """Home Assistant skips unavailable entities in entity services, so nothing is published."""
    entity_id = _entity_id(hass, "button", "task_reset")
    fire_robot_message(hass, "availability", "offline")
    await hass.async_block_till_done()
    assert hass.states.get(entity_id).state == "unavailable"  # type: ignore[union-attr]
    mqtt_setup.async_publish.reset_mock()
    await hass.services.async_call("button", "press", {"entity_id": entity_id}, blocking=True)
    mqtt_setup.async_publish.assert_not_called()
    assert hub.registry.outstanding(ROBOT_ID) == []


# --------------------------------------------------------------------------- platform setup


@pytest.mark.parametrize(
    ("platform", "expected"),
    [(sensor, 2), (binary_sensor, 1), (event, 1), (button, 5)],
    ids=["sensor", "binary_sensor", "event", "button"],
)
async def test_platform_picks_up_robots_announced_before_it_loaded(
    hass: HomeAssistant, hub: BridgeHub, platform: Any, expected: int
) -> None:
    """A platform set up after an announce creates the entities at once, and only once."""
    added: list[list[str]] = []

    def _add_entities(entities: Any, update_before_add: bool = False, **_: Any) -> None:
        added.append([entity.unique_id for entity in entities])

    await platform.async_setup_entry(hass, hub.entry, _add_entities)
    assert len(added) == 1
    assert len(added[0]) == expected
    assert all(unique_id.startswith(f"{ROBOT_ID}_") for unique_id in added[0])

    async_dispatcher_send(hass, SIGNAL_ROBOT_ADDED, ROBOT_ID)
    await hass.async_block_till_done()
    assert len(added) == 1, "a repeated robot_added signal adds nothing"
