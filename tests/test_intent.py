# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The ``Ros2BridgeSendTask`` and ``Ros2BridgeCancelTask`` Assist intents."""

from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
from datetime import timedelta
from typing import Any

import pytest
from freezegun.api import FrozenDateTimeFactory
from homeassistant.core import Context, HomeAssistant
from homeassistant.exceptions import HomeAssistantError
from homeassistant.helpers import area_registry as ar
from homeassistant.helpers import device_registry as dr
from homeassistant.helpers import intent
from homeassistant.util import dt as dt_util
from pytest_homeassistant_custom_component.common import async_fire_time_changed

from custom_components.ros2_bridge.commands import CommandRegistry
from custom_components.ros2_bridge.const import (
    CONF_ACK_TIMEOUT_S,
    CONF_INTENT_ACK_WAIT_S,
    DEFAULT_INTENT_ACK_WAIT_S,
    DOMAIN,
)
from custom_components.ros2_bridge.hub import BridgeHub
from custom_components.ros2_bridge.intent import (
    INTENT_CANCEL_TASK,
    INTENT_SEND_TASK,
    SPEECH,
    _speech_for_error,
    async_register_intents,
    async_setup_intents,
    say,
)
from custom_components.ros2_bridge.models import Robot
from custom_components.ros2_bridge.services import DATA_HUB

from .common import ROBOT_ID, fire_robot_message, make_announce, robot_topic
from .test_services import CMD_TOPIC, OTHER_ROBOT, HubRig, default_rig_options, hub_rig  # noqa: F401

SATELLITE_DEVICE_ID = "voice_pe_kitchen"


def speech(response: intent.IntentResponse) -> str:
    return str(response.speech["plain"]["speech"])


async def handle(
    hass: HomeAssistant,
    intent_type: str = INTENT_SEND_TASK,
    *,
    task: str | None = "reset",
    name: str | None = None,
    area: str | None = None,
    device_id: str | None = None,
) -> intent.IntentResponse:
    """Run an intent the way the conversation agent would."""
    slots: dict[str, Any] = {}
    if task is not None and intent_type == INTENT_SEND_TASK:
        slots["task"] = {"value": task}
    if name is not None:
        slots["robot"] = {"value": name}
    if area is not None:
        slots["area"] = {"value": area}
    return await intent.async_handle(
        hass,
        "test",
        intent_type,
        slots,
        context=Context(user_id="voice-user"),
        device_id=device_id,
    )


def start_handle(hass: HomeAssistant, **kwargs: Any) -> asyncio.Task[intent.IntentResponse]:
    return hass.async_create_task(handle(hass, **kwargs))


async def satellite_in(rig: HubRig, area_id: str) -> str:
    """A satellite device placed in an area; returns its device id."""
    registry = dr.async_get(rig.hass)
    device = registry.async_get_or_create(
        config_entry_id=rig.entry.entry_id, identifiers={("assist_satellite", SATELLITE_DEVICE_ID)}
    )
    registry.async_update_device(device.id, area_id=area_id)
    return device.id


def place_robot(rig: HubRig, robot_id: str, area_name: str) -> str:
    """Put a robot's device in an area (created on demand); returns the area id."""
    area = ar.async_get(rig.hass).async_get_area_by_name(area_name) or ar.async_get(
        rig.hass
    ).async_create(area_name)
    dr.async_get(rig.hass).async_update_device(rig.device_id(robot_id), area_id=area.id)
    return area.id


# --------------------------------------------------------------------------- registration


async def test_intents_register_once(hass: HomeAssistant) -> None:
    async_register_intents(hass)
    async_register_intents(hass)
    handlers = {handler.intent_type: handler for handler in intent.async_get(hass)}
    assert set(handlers) >= {INTENT_SEND_TASK, INTENT_CANCEL_TASK}
    assert len([h for h in intent.async_get(hass) if h.intent_type == INTENT_SEND_TASK]) == 1
    send = handlers[INTENT_SEND_TASK]
    assert send.slot_schema is not None and send.async_validate_slots({"task": {"value": "x"}})


async def test_async_setup_intents_is_the_platform_hook(hass: HomeAssistant) -> None:
    """Home Assistant's intent component calls this name for an intent.py platform."""
    await async_setup_intents(hass)
    await async_setup_intents(hass)
    async_register_intents(hass)
    assert len([h for h in intent.async_get(hass) if h.intent_type == INTENT_SEND_TASK]) == 1
    assert len([h for h in intent.async_get(hass) if h.intent_type == INTENT_CANCEL_TASK]) == 1


async def test_not_loaded_speech(hass: HomeAssistant) -> None:
    async_register_intents(hass)
    response = await handle(hass)
    assert response.response_type is intent.IntentResponseType.ERROR
    assert response.error_code is intent.IntentResponseErrorCode.FAILED_TO_HANDLE
    assert speech(response) == SPEECH["not_loaded"]
    response = await handle(hass, INTENT_CANCEL_TASK)
    assert speech(response) == SPEECH["not_loaded"]


# --------------------------------------------------------------------------- the three outcomes


async def test_accepted_speech_and_source(rig: HubRig) -> None:
    await rig.announce()
    pending = start_handle(rig.hass, task="reset", device_id=SATELLITE_DEVICE_ID)
    await rig.until_published(CMD_TOPIC)
    command = rig.commands()[0]
    assert command["task"] == "reset" and command["params"] == {}
    assert command["source"]["kind"] == "intent"
    assert command["source"]["device_id"] == SATELLITE_DEVICE_ID
    assert command["source"]["user_id"] == "voice-user"
    rig.ack(command["id"])
    response = await pending
    assert response.response_type is intent.IntentResponseType.ACTION_DONE
    assert speech(response) == "Starting Reset to home on Left arm"


async def test_refused_speech(rig: HubRig) -> None:
    await rig.announce()
    pending = start_handle(rig.hass, task="reset")
    await rig.until_published(CMD_TOPIC)
    rig.ack(rig.commands()[0]["id"], accepted=False)
    response = await pending
    assert response.response_type is intent.IntentResponseType.ERROR
    assert speech(response) == (
        "Left arm refused: Task 'reset' (id 51f0a2b3-c4d5-4e6f-a7b8-c9d0e1f2a3b4) is running"
    )


async def test_report_back_speech_without_waiting_for_the_result(
    rig: HubRig, advance: Callable[[float], Awaitable[None]]
) -> None:
    await rig.announce()
    pending = start_handle(rig.hass, task="reset")
    await rig.until_published(CMD_TOPIC)
    await advance(DEFAULT_INTENT_ACK_WAIT_S + 0.1)
    response = await pending
    assert response.response_type is intent.IntentResponseType.ACTION_DONE
    assert speech(response) == "Sent Reset to home to Left arm, I will report back"
    command = rig.hub.registry.get(rig.commands()[0]["id"])
    assert command is not None and command.state.value == "sent"  # still tracked, no no_ack yet


@pytest.mark.parametrize(
    ("key", "expected"),
    [
        ("robot_unavailable", "Left arm is not available right now"),
        ("too_many_pending", "Left arm has too many commands waiting"),
        ("invalid_params", "Reset to home needs details I cannot take by voice"),
        ("not_ready", "The robot bridge is not ready yet"),
        ("publish_failed", "I could not reach Left arm"),
        ("task_unknown", "Left arm has no task called Reset to home"),
        ("robot_unknown", "I do not know Left arm any more"),
    ],
)
def test_speech_for_hub_errors(key: str, expected: str) -> None:
    robot = Robot(ROBOT_ID, announce=make_announce())
    error = HomeAssistantError(translation_domain=DOMAIN, translation_key=key)
    assert _speech_for_error(error, robot, "Reset to home") == expected


def test_speech_for_an_untranslated_error_is_its_text() -> None:
    robot = Robot(ROBOT_ID)
    error = HomeAssistantError("plain failure")
    assert _speech_for_error(error, robot, "Reset to home") == "plain failure"


async def test_hub_stop_while_waiting_for_the_ack_still_answers(rig: HubRig) -> None:
    """An options save during the brief wait: the intent still speaks, the command stays."""
    await rig.announce()
    pending = start_handle(rig.hass, task="reset")
    await rig.until_published(CMD_TOPIC)
    command_id = rig.commands()[0]["id"]
    for _ in range(100):
        if rig.hub.registry._ack_waiters.get(command_id):
            break
        await asyncio.sleep(0)
    await rig.hub.async_stop()
    response = await pending
    assert response.response_type is intent.IntentResponseType.ACTION_DONE
    assert speech(response) == "Sent Reset to home to Left arm, I will report back"
    assert [c.state.value for c in rig.hub.registry.outstanding(ROBOT_ID)] == ["sent"]


async def test_task_needing_parameters_is_refused_by_voice(rig: HubRig) -> None:
    """move_to needs a pose; the hub refuses before anything is published and the speech says so."""
    await rig.announce()
    response = await handle(rig.hass, task="move to named pose")
    assert response.response_type is intent.IntentResponseType.ERROR
    assert response.error_code is intent.IntentResponseErrorCode.FAILED_TO_HANDLE
    assert speech(response) == "Move to named pose needs details I cannot take by voice"
    assert rig.commands() == []


# --------------------------------------------------------------------------- task resolution


async def test_task_by_label_and_unknown_task(rig: HubRig) -> None:
    await rig.announce()
    pending = start_handle(rig.hass, task="Reset To Home")
    await rig.until_published(CMD_TOPIC)
    assert rig.commands()[0]["task"] == "reset"
    rig.ack(rig.commands()[0]["id"])
    assert speech(await pending).startswith("Starting Reset to home")

    response = await handle(rig.hass, task="dance")
    assert response.response_type is intent.IntentResponseType.ERROR
    assert speech(response) == "Left arm has no task called dance"


# --------------------------------------------------------------------------- robot resolution


@pytest.mark.parametrize(
    "name",
    ["arm_left", "arm left", "Arm Left", "Left arm", "left arm", "Kitchen helper"],
)
async def test_resolution_by_id_and_names(rig: HubRig, name: str) -> None:
    await rig.announce()
    await rig.announce(OTHER_ROBOT)
    dr.async_get(rig.hass).async_update_device(
        rig.device_id(ROBOT_ID), name_by_user="Kitchen helper"
    )
    pending = start_handle(rig.hass, task="reset", name=name)
    await rig.until_published(CMD_TOPIC)
    assert rig.commands(ROBOT_ID) and not rig.commands(OTHER_ROBOT)
    rig.ack(rig.commands()[0]["id"])
    assert speech(await pending) == "Starting Reset to home on Left arm"


async def test_unknown_name(rig: HubRig) -> None:
    await rig.announce()
    response = await handle(rig.hass, name="toaster")
    assert response.response_type is intent.IntentResponseType.ERROR
    assert response.error_code is intent.IntentResponseErrorCode.NO_VALID_TARGETS
    assert speech(response) == "I do not know a robot called toaster"
    assert rig.commands() == []


async def test_resolution_by_area_slot(rig: HubRig) -> None:
    await rig.announce()
    await rig.announce(OTHER_ROBOT)
    place_robot(rig, ROBOT_ID, "Kitchen")
    place_robot(rig, OTHER_ROBOT, "Workshop")
    pending = start_handle(rig.hass, task="reset", area="kitchen")
    await rig.until_published(CMD_TOPIC)
    assert rig.commands(ROBOT_ID) and not rig.commands(OTHER_ROBOT)
    rig.ack(rig.commands()[0]["id"])
    assert speech(await pending) == "Starting Reset to home on Left arm"

    response = await handle(rig.hass, area="Garden")
    assert speech(response) == "I do not know an area called Garden"
    ar.async_get(rig.hass).async_create("Garden")
    response = await handle(rig.hass, area="Garden")
    assert speech(response) == "There is no robot in Garden"


async def test_resolution_by_satellite_area(rig: HubRig) -> None:
    await rig.announce()
    await rig.announce(OTHER_ROBOT)
    place_robot(rig, ROBOT_ID, "Kitchen")
    workshop = place_robot(rig, OTHER_ROBOT, "Workshop")
    satellite = await satellite_in(rig, workshop)
    pending = start_handle(rig.hass, task="reset", device_id=satellite)
    await rig.until_published(robot_topic("cmd", robot_id=OTHER_ROBOT))
    assert rig.commands(OTHER_ROBOT) and not rig.commands(ROBOT_ID)
    assert rig.commands(OTHER_ROBOT)[0]["source"]["device_id"] == satellite
    rig.ack(rig.commands(OTHER_ROBOT)[0]["id"], robot_id=OTHER_ROBOT)
    assert speech(await pending) == "Starting Reset to home on Arm right"


async def test_only_robot_and_ambiguity(rig: HubRig) -> None:
    await rig.announce()
    pending = start_handle(rig.hass, task="reset")
    await rig.until_published(CMD_TOPIC)
    rig.ack(rig.commands()[0]["id"])
    assert speech(await pending) == "Starting Reset to home on Left arm"

    await rig.announce(OTHER_ROBOT)
    response = await handle(rig.hass, task="reset")
    assert response.response_type is intent.IntentResponseType.ERROR
    assert speech(response) == SPEECH["which_robot"]
    # Two robots in one area are just as ambiguous.
    place_robot(rig, ROBOT_ID, "Kitchen")
    kitchen = place_robot(rig, OTHER_ROBOT, "Kitchen")
    assert speech(await handle(rig.hass, area="Kitchen")) == SPEECH["which_robot"]
    satellite = await satellite_in(rig, kitchen)
    assert speech(await handle(rig.hass, device_id=satellite)) == SPEECH["which_robot"]
    assert len(rig.commands()) == 1


async def test_no_robots_at_all(rig: HubRig) -> None:
    response = await handle(rig.hass)
    assert response.response_type is intent.IntentResponseType.ERROR
    assert speech(response) == "No robot has announced itself yet"


async def test_unavailable_robot_speech(rig: HubRig) -> None:
    await rig.announce()
    fire_robot_message(rig.hass, "availability", "offline")
    await rig.hass.async_block_till_done()
    response = await handle(rig.hass)
    assert response.response_type is intent.IntentResponseType.ERROR
    assert speech(response) == "Left arm is not available right now"


# --------------------------------------------------------------------------- cancel


async def test_cancel_intent_outcomes(
    rig: HubRig, advance: Callable[[float], Awaitable[None]]
) -> None:
    await rig.announce()
    pending = start_handle(rig.hass, intent_type=INTENT_CANCEL_TASK, name="left arm")
    await rig.until_published(CMD_TOPIC)
    cancel = rig.commands()[0]
    assert cancel["type"] == "cancel" and cancel["target_id"] is None
    assert cancel["source"]["kind"] == "intent"
    rig.ack(cancel["id"], command_type="cancel")
    assert speech(await pending) == "Cancelling the current task on Left arm"

    pending = start_handle(rig.hass, intent_type=INTENT_CANCEL_TASK)
    await rig.until_published(CMD_TOPIC, 2)
    rig.ack(rig.commands()[1]["id"], command_type="cancel", accepted=False)
    response = await pending
    assert response.response_type is intent.IntentResponseType.ERROR
    assert speech(response) == "Left arm refused: Nothing running"

    pending = start_handle(rig.hass, intent_type=INTENT_CANCEL_TASK)
    await rig.until_published(CMD_TOPIC, 3)
    await advance(DEFAULT_INTENT_ACK_WAIT_S + 0.1)
    assert speech(await pending) == "Sent cancel to Left arm, I will report back"


async def test_cancel_intent_speech_for_an_unavailable_robot(rig: HubRig) -> None:
    await rig.announce()
    fire_robot_message(rig.hass, "availability", "offline")
    await rig.hass.async_block_till_done()
    response = await handle(rig.hass, INTENT_CANCEL_TASK)
    assert response.response_type is intent.IntentResponseType.ERROR
    assert response.error_code is intent.IntentResponseErrorCode.FAILED_TO_HANDLE
    assert speech(response) == "Left arm is not available right now"
    assert rig.commands() == []


# --------------------------------------------------------------------------- lifecycle


async def test_still_handled_after_an_options_reload_and_after_unload(
    rig: HubRig, complete_hub_start: Callable[[BridgeHub], Awaitable[None]]
) -> None:
    await rig.announce()
    # An options save unloads the entry and sets it up again with the parked registry.
    await rig.hub.async_stop()
    rig.unload()
    assert speech(await handle(rig.hass)) == SPEECH["not_loaded"]
    registry: CommandRegistry = rig.hub.registry
    reloaded = BridgeHub(rig.hass, rig.entry, registry, rig.store)
    await reloaded.async_start()
    await complete_hub_start(reloaded)
    rig.hass.data[DOMAIN][DATA_HUB] = reloaded
    rig.hub = reloaded
    await rig.announce()
    rig.mqtt.async_publish.reset_mock()
    pending = start_handle(rig.hass, task="reset")
    await rig.until_published(CMD_TOPIC)
    rig.ack(rig.commands()[0]["id"])
    assert speech(await pending) == "Starting Reset to home on Left arm"

    # An unload leaves the handlers registered; they answer with the error speech.
    await reloaded.async_stop()
    rig.unload()
    assert {h.intent_type for h in intent.async_get(rig.hass)} >= {
        INTENT_SEND_TASK,
        INTENT_CANCEL_TASK,
    }
    response = await handle(rig.hass)
    assert response.response_type is intent.IntentResponseType.ERROR
    assert speech(response) == SPEECH["not_loaded"]


# --------------------------------------------------------------------------- intent_ack_wait_s


@pytest.mark.parametrize("rig_options", [{CONF_INTENT_ACK_WAIT_S: 3.0, CONF_ACK_TIMEOUT_S: 5.0}])
async def test_intent_ack_wait_option_lengthens_the_wait(
    rig: HubRig, freezer: FrozenDateTimeFactory
) -> None:
    await rig.announce()
    pending = start_handle(rig.hass, task="reset")
    await rig.until_published(CMD_TOPIC)
    freezer.tick(timedelta(seconds=DEFAULT_INTENT_ACK_WAIT_S + 0.5))
    async_fire_time_changed(rig.hass, dt_util.utcnow())
    for _ in range(20):
        await asyncio.sleep(0)
    assert not pending.done(), "the released 1.5 s cap no longer applies"
    rig.ack(rig.commands()[0]["id"])
    response = await pending
    assert speech(response) == "Starting Reset to home on Left arm"


@pytest.mark.parametrize("rig_options", [{CONF_INTENT_ACK_WAIT_S: 5.0, CONF_ACK_TIMEOUT_S: 1.0}])
async def test_intent_ack_wait_is_capped_by_the_ack_timeout(
    rig: HubRig, advance: Callable[[float], Awaitable[None]]
) -> None:
    await rig.announce()
    pending = start_handle(rig.hass, task="reset")
    await rig.until_published(CMD_TOPIC)
    await advance(1.2)
    response = await pending
    assert speech(response) == "Sent Reset to home to Left arm, I will report back"


def test_speech_table_is_generic_and_complete() -> None:
    """Every reply is one table entry and none presumes the robot is an arm."""
    assert all("arm" not in text.casefold().split() for text in SPEECH.values())
    assert SPEECH["which_robot"] == "Which robot do you mean?"
    assert say("refused", robot="Left arm", reason="busy") == "Left arm refused: busy"
