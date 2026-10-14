# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Assist intents of the ROS 2 Bridge integration.

``Ros2BridgeSendTask`` starts a named task on a robot and ``Ros2BridgeCancelTask``
cancels its running task. Both are registered once from ``async_setup`` and
never removed (``intent.async_remove`` pops a handler globally, which would
silence voice after the first options save); while no hub is loaded they
answer with the error speech "The robot bridge is not loaded".

The robot is resolved in this order: the ``name`` slot as a robot id, then as
a folded robot or device name; the ``area`` slot; the area of the satellite
that heard the command (``intent_obj.device_id``); and finally the only robot
there is. Anything ambiguous asks "Which robot do you mean?". The task is
matched by name and then by label. The handler waits for the acknowledgement
for at most ``min(ack_timeout_s, intent_ack_wait_s)`` seconds (both options)
and speaks "Starting {label} on {robot}", "{robot} refused: {reason}" or
"Sent {label} to {robot}, I will report back"; it never waits for the result
(Q10: the outcome is reported later by an automation on
``ros2_bridge_task_result``).

Every spoken reply lives in :data:`SPEECH`, one table, so an adopter whose
robot is not an arm, or a translator, changes the wording in one place
without touching the handlers. The replies are deliberately generic ("robot",
never "arm").
"""

from __future__ import annotations

import asyncio
import logging
from collections.abc import Mapping
from typing import Any, Final

import probatio
from homeassistant.core import HomeAssistant, callback
from homeassistant.exceptions import HomeAssistantError
from homeassistant.helpers import area_registry as ar
from homeassistant.helpers import config_validation as cv
from homeassistant.helpers import device_registry as dr
from homeassistant.helpers import intent
from homeassistant.helpers.typing import VolDictType

from .const import SOURCE_KIND_INTENT
from .hub import BridgeHub
from .models import PendingCommand, Robot, TaskSpec
from .protocol import CommandSource
from .services import async_get_hub

_LOGGER = logging.getLogger(__name__)

INTENT_SEND_TASK: Final = "Ros2BridgeSendTask"
INTENT_CANCEL_TASK: Final = "Ros2BridgeCancelTask"

#: Every spoken reply. Entries with braces are ``str.format`` templates taking ``robot``
#: (the robot's announced name), ``task`` (the task's announced label), ``name`` or
#: ``area`` (the spoken words) and ``reason`` (the robot's ack reason or code).
SPEECH: Final[dict[str, str]] = {
    "not_loaded": "The robot bridge is not loaded",
    "not_ready": "The robot bridge is not ready yet",
    "which_robot": "Which robot do you mean?",
    "no_robots": "No robot has announced itself yet",
    "unknown_robot": "I do not know a robot called {name}",
    "unknown_area": "I do not know an area called {area}",
    "no_robot_in_area": "There is no robot in {area}",
    "unknown_task": "{robot} has no task called {task}",
    "unavailable": "{robot} is not available right now",
    "too_many_pending": "{robot} has too many commands waiting",
    "needs_params": "{task} needs details I cannot take by voice",
    "unreachable": "I could not reach {robot}",
    "forgotten": "I do not know {robot} any more",
    "refused": "{robot} refused: {reason}",
    "starting": "Starting {task} on {robot}",
    "sent": "Sent {task} to {robot}, I will report back",
    "cancelling": "Cancelling the current task on {robot}",
    "cancel_sent": "Sent cancel to {robot}, I will report back",
}

SLOT_TASK: Final = "task"
SLOT_NAME: Final = "robot"  # not "name": Home Assistant resolves a "name" slot as an entity name
SLOT_AREA: Final = "area"


def say(key: str, **values: Any) -> str:
    """The reply ``key`` from :data:`SPEECH` with its placeholders filled in."""
    return SPEECH[key].format(**values)


# --------------------------------------------------------------------------- resolution


def _fold(text: str) -> str:
    """Case-fold and collapse whitespace for matching spoken words."""
    return " ".join(text.casefold().split())


def _as_robot_id(text: str) -> str:
    """Spoken words as a robot id or task name (``left arm`` becomes ``left_arm``)."""
    return _fold(text).replace("-", " ").replace(" ", "_")


def _robot_names(robot: Robot, devices: dr.DeviceRegistry) -> set[str]:
    """Every folded name a robot answers to: announce name, id and device names."""
    names = {_fold(robot.name), _fold(robot.robot_id.replace("_", " "))}
    device = devices.async_get(robot.device_id) if robot.device_id else None
    if device is not None:
        if device.name_by_user:
            names.add(_fold(device.name_by_user))
        if device.name:
            names.add(_fold(device.name))
    return names


def _robots_in_area(robots: list[Robot], devices: dr.DeviceRegistry, area_id: str) -> list[Robot]:
    matches: list[Robot] = []
    for robot in robots:
        device = devices.async_get(robot.device_id) if robot.device_id else None
        if device is not None and device.area_id == area_id:
            matches.append(robot)
    return matches


def _find_area(hass: HomeAssistant, text: str) -> ar.AreaEntry | None:
    """An area by name, alias or id."""
    areas = ar.async_get(hass)
    area = areas.async_get_area_by_name(text)
    if area is not None:
        return area
    by_alias = areas.async_get_areas_by_alias(text)
    if len(by_alias) == 1:
        return by_alias[0]
    return areas.async_get_area(text)


def resolve_robot(
    hass: HomeAssistant,
    hub: BridgeHub,
    *,
    name: str | None,
    area: str | None,
    device_id: str | None,
) -> tuple[Robot | None, str]:
    """Pick the robot a spoken command means; returns ``(robot, speech)``.

    ``speech`` is what to say when ``robot`` is ``None`` (unknown name or area,
    an ambiguous choice, or no robot at all).
    """
    robots = [robot for robot in hub.robots.values() if robot.announce is not None]
    devices = dr.async_get(hass)
    if name:
        wanted = _as_robot_id(name)
        matches = [robot for robot in robots if robot.robot_id == wanted]
        if not matches:
            folded = _fold(name)
            matches = [robot for robot in robots if folded in _robot_names(robot, devices)]
        if len(matches) == 1:
            return matches[0], ""
        if matches:
            return None, say("which_robot")
        return None, say("unknown_robot", name=name)
    if area:
        entry = _find_area(hass, area)
        if entry is None:
            return None, say("unknown_area", area=area)
        matches = _robots_in_area(robots, devices, entry.id)
        if len(matches) == 1:
            return matches[0], ""
        if matches:
            return None, say("which_robot")
        return None, say("no_robot_in_area", area=entry.name)
    if device_id:
        satellite = devices.async_get(device_id)
        if satellite is not None and satellite.area_id:
            matches = _robots_in_area(robots, devices, satellite.area_id)
            if len(matches) == 1:
                return matches[0], ""
            if matches:
                return None, say("which_robot")
    if len(robots) == 1:
        return robots[0], ""
    if not robots:
        return None, say("no_robots")
    return None, say("which_robot")


def resolve_task(robot: Robot, text: str) -> TaskSpec | None:
    """The announced task the words name: by task name first, then by label."""
    spec = robot.task(_as_robot_id(text))
    if spec is None and robot.announce is not None:
        spec = robot.announce.task_by_label(text)
    return spec


def _slot(slots: Mapping[str, Any], key: str) -> str | None:
    """The text of an optional slot, or ``None``."""
    slot = slots.get(key)
    if not isinstance(slot, Mapping):
        return None
    value = slot.get("value")
    if value is None:
        return None
    text = str(value).strip()
    return text or None


#: Hub error translation keys and the :data:`SPEECH` entry each is spoken as.
_SPEECH_FOR_ERROR: Final[dict[str, str]] = {
    "robot_unavailable": "unavailable",
    "too_many_pending": "too_many_pending",
    "invalid_params": "needs_params",
    "not_ready": "not_ready",
    "publish_failed": "unreachable",
    "task_unknown": "unknown_task",
    "robot_unknown": "forgotten",
}


def _speech_for_error(error: HomeAssistantError, robot: Robot, label: str) -> str:
    """Plain speech for a hub error, keyed on its translation key (no translations needed)."""
    key = _SPEECH_FOR_ERROR.get(error.translation_key or "")
    if key is None:
        return str(error)
    return say(key, robot=robot.name, task=label)


async def _async_wait_ack_briefly(hub: BridgeHub, command: PendingCommand) -> None:
    """Wait for the ack for at most ``min(ack_timeout_s, intent_ack_wait_s)`` seconds."""
    try:
        future = hub.registry.wait_ack(command.task_id)
    except KeyError:
        return
    timeout = min(hub.options.ack_timeout_s, hub.options.intent_ack_wait_s)
    try:
        await asyncio.wait_for(future, timeout)
    except TimeoutError:
        return
    except asyncio.CancelledError:
        task = asyncio.current_task()
        if task is not None and task.cancelling():
            raise
        # The hub was unloaded while we waited; the command is still tracked.


# --------------------------------------------------------------------------- handlers


class _RobotIntentHandler(intent.IntentHandler):
    """Shared plumbing: hub lookup, robot resolution and the error response."""

    @staticmethod
    def _error(
        intent_obj: intent.Intent,
        speech: str,
        code: intent.IntentResponseErrorCode = intent.IntentResponseErrorCode.NO_VALID_TARGETS,
    ) -> intent.IntentResponse:
        response = intent_obj.create_response()
        response.async_set_error(code, speech)
        return response

    @staticmethod
    def _source(intent_obj: intent.Intent) -> CommandSource:
        return CommandSource(
            kind=SOURCE_KIND_INTENT,
            user_id=intent_obj.context.user_id,
            context_id=intent_obj.context.id,
            device_id=intent_obj.device_id,
        )

    def _target(
        self, intent_obj: intent.Intent, slots: Mapping[str, Any]
    ) -> tuple[BridgeHub, Robot] | intent.IntentResponse:
        """The hub and robot a command addresses, or the error response to give instead."""
        hub = async_get_hub(intent_obj.hass)
        if hub is None:
            return self._error(
                intent_obj, say("not_loaded"), intent.IntentResponseErrorCode.FAILED_TO_HANDLE
            )
        robot, speech = resolve_robot(
            intent_obj.hass,
            hub,
            name=_slot(slots, SLOT_NAME),
            area=_slot(slots, SLOT_AREA),
            device_id=intent_obj.device_id,
        )
        if robot is None:
            return self._error(intent_obj, speech)
        return hub, robot

    def _refused(self, intent_obj: intent.Intent, robot: Robot, command: PendingCommand) -> Any:
        ack = command.ack
        reason = (ack.reason or ack.code) if ack is not None else "unknown"
        return self._error(
            intent_obj,
            say("refused", robot=robot.name, reason=reason),
            intent.IntentResponseErrorCode.FAILED_TO_HANDLE,
        )


SEND_TASK_SLOT_SCHEMA: VolDictType = {
    probatio.Required(SLOT_TASK): cv.string,
    probatio.Optional(SLOT_NAME): cv.string,
    probatio.Optional(SLOT_AREA): cv.string,
}
CANCEL_TASK_SLOT_SCHEMA: VolDictType = {
    probatio.Optional(SLOT_NAME): cv.string,
    probatio.Optional(SLOT_AREA): cv.string,
}


class Ros2BridgeSendTaskHandler(_RobotIntentHandler):
    """``Ros2BridgeSendTask``: start the named task on the resolved robot."""

    intent_type = INTENT_SEND_TASK
    description = "Start a named task on a ROS 2 robot"

    @property
    def slot_schema(self) -> VolDictType:
        """``task`` is required; ``name`` and ``area`` pick the robot."""
        return SEND_TASK_SLOT_SCHEMA

    async def async_handle(self, intent_obj: intent.Intent) -> intent.IntentResponse:
        """Resolve the robot and task, send, wait briefly for the ack and speak the outcome."""
        slots = self.async_validate_slots(intent_obj.slots)
        target = self._target(intent_obj, slots)
        if isinstance(target, intent.IntentResponse):
            return target
        hub, robot = target
        task_text = _slot(slots, SLOT_TASK) or ""
        spec = resolve_task(robot, task_text)
        if spec is None:
            return self._error(intent_obj, say("unknown_task", robot=robot.name, task=task_text))
        try:
            command = await hub.async_send_task(
                robot.robot_id, spec.name, {}, source=self._source(intent_obj)
            )
        except HomeAssistantError as error:
            return self._error(
                intent_obj,
                _speech_for_error(error, robot, spec.label),
                intent.IntentResponseErrorCode.FAILED_TO_HANDLE,
            )
        await _async_wait_ack_briefly(hub, command)
        if command.accepted is False:
            response: intent.IntentResponse = self._refused(intent_obj, robot, command)
            return response
        response = intent_obj.create_response()
        if command.accepted is True:
            response.async_set_speech(say("starting", task=spec.label, robot=robot.name))
        else:
            response.async_set_speech(say("sent", task=spec.label, robot=robot.name))
        return response


class Ros2BridgeCancelTaskHandler(_RobotIntentHandler):
    """``Ros2BridgeCancelTask``: cancel the active task on the resolved robot."""

    intent_type = INTENT_CANCEL_TASK
    description = "Cancel the running task of a ROS 2 robot"

    @property
    def slot_schema(self) -> VolDictType:
        """``name`` and ``area`` pick the robot; both are optional."""
        return CANCEL_TASK_SLOT_SCHEMA

    async def async_handle(self, intent_obj: intent.Intent) -> intent.IntentResponse:
        """Resolve the robot, send a cancel, wait briefly for its ack and speak the outcome."""
        slots = self.async_validate_slots(intent_obj.slots)
        target = self._target(intent_obj, slots)
        if isinstance(target, intent.IntentResponse):
            return target
        hub, robot = target
        try:
            command = await hub.async_cancel_task(
                robot.robot_id, None, source=self._source(intent_obj)
            )
        except HomeAssistantError as error:
            return self._error(
                intent_obj,
                _speech_for_error(error, robot, "cancel"),
                intent.IntentResponseErrorCode.FAILED_TO_HANDLE,
            )
        await _async_wait_ack_briefly(hub, command)
        if command.accepted is False:
            response: intent.IntentResponse = self._refused(intent_obj, robot, command)
            return response
        response = intent_obj.create_response()
        if command.accepted is True:
            response.async_set_speech(say("cancelling", robot=robot.name))
        else:
            response.async_set_speech(say("cancel_sent", robot=robot.name))
        return response


@callback
def async_register_intents(hass: HomeAssistant) -> None:
    """Register both intent handlers once; they are never removed."""
    registered = {handler.intent_type for handler in intent.async_get(hass)}
    if INTENT_SEND_TASK not in registered:
        intent.async_register(hass, Ros2BridgeSendTaskHandler())
    if INTENT_CANCEL_TASK not in registered:
        intent.async_register(hass, Ros2BridgeCancelTaskHandler())


async def async_setup_intents(hass: HomeAssistant) -> None:
    """The hook Home Assistant's intent component calls for an ``intent.py`` platform.

    Home Assistant processes every loaded integration's ``intent`` platform and
    expects this name; without it the intent component logs an error at setup.
    Registration is idempotent, so being called here and from ``async_setup``
    registers each handler exactly once.
    """
    async_register_intents(hass)
