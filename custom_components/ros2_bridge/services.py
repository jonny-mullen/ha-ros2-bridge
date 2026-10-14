# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Services of the ROS 2 Bridge integration: ``send_task``, ``cancel_task`` and ``get_robots``.

The three services are registered once from ``async_setup`` so they exist for
the whole Home Assistant run; a call made while no hub is loaded raises
``not_loaded``. Handlers find the loaded hub through :func:`async_get_hub`,
which looks at ``hass.data[DOMAIN]["hub"]`` first and then at the loaded
config entry's ``runtime_data``.

``send_task`` and ``cancel_task`` support an optional response. With a
response requested the outcome is reported in the response and nothing short
of a validation or publish problem raises; without one, a refusal raises
``task_rejected``, a missing acknowledgement raises ``no_ack`` and (for
``wait: result``) a missing result raises ``no_result``, so a script stops at
the step that did not happen. With several targets every one is validated
before the first command is published, so a validation error never leaves a
command already on its way to another robot. Every error is a translated
``HomeAssistantError`` or ``ServiceValidationError`` under this domain; the
placeholders each key carries are listed next to :func:`_raise_for_command`.

Schemas are probatio objects (``import probatio``, never voluptuous) typed as
``VolSchemaType``.
"""

from __future__ import annotations

import asyncio
import logging
from collections.abc import Mapping
from typing import Any, Final

import probatio
from homeassistant.core import (
    HomeAssistant,
    ServiceCall,
    ServiceResponse,
    SupportsResponse,
    callback,
)
from homeassistant.exceptions import HomeAssistantError, ServiceValidationError
from homeassistant.helpers import config_validation as cv
from homeassistant.helpers import device_registry as dr
from homeassistant.helpers.typing import VolDictType, VolSchemaType
from homeassistant.util.json import JsonObjectType, JsonValueType

from .const import DATA_HUB, DOMAIN, SOURCE_KIND_SERVICE, CommandState
from .hub import BridgeHub
from .models import PendingCommand
from .protocol import CommandSource

_LOGGER = logging.getLogger(__name__)

SERVICE_SEND_TASK: Final = "send_task"
SERVICE_CANCEL_TASK: Final = "cancel_task"
SERVICE_GET_ROBOTS: Final = "get_robots"

ATTR_DEVICE_ID: Final = "device_id"
ATTR_ROBOT_ID: Final = "robot_id"
ATTR_TASK: Final = "task"
ATTR_TASK_ID: Final = "task_id"
ATTR_PARAMS: Final = "params"
ATTR_WAIT: Final = "wait"
ATTR_TIMEOUT_S: Final = "timeout_s"
ATTR_MAX_AGE_MS: Final = "max_age_ms"

WAIT_ACK: Final = "ack"
WAIT_RESULT: Final = "result"
WAIT_NONE: Final = "none"
WAIT_MODES: Final[tuple[str, ...]] = (WAIT_ACK, WAIT_RESULT, WAIT_NONE)

#: Seconds added to the registry's own deadline before a waiting caller gives up on it.
WAIT_SLACK_S: Final = 2.0

_TARGET_FIELDS: VolDictType = {
    probatio.Optional(ATTR_DEVICE_ID): probatio.All(cv.ensure_list, [cv.string]),
    probatio.Optional(ATTR_ROBOT_ID): cv.string,
}
_WAIT_FIELD: VolDictType = {
    probatio.Optional(ATTR_WAIT, default=WAIT_ACK): probatio.In(WAIT_MODES),
}
#: The smallest ``max_age_ms`` the actions accept, the same floor as the options form; there
#: is no ceiling here because the node clamps to its own ``command.max_age_cap_ms``.
MIN_MAX_AGE_MS: Final = 1000

_MAX_AGE_FIELD: VolDictType = {
    probatio.Optional(ATTR_MAX_AGE_MS): probatio.All(
        probatio.Coerce(int), probatio.Range(min=MIN_MAX_AGE_MS)
    ),
}

SEND_TASK_SCHEMA: VolSchemaType = probatio.All(
    probatio.Schema(
        {
            **_TARGET_FIELDS,
            probatio.Required(ATTR_TASK): cv.string,
            probatio.Optional(ATTR_PARAMS): dict,
            **_WAIT_FIELD,
            probatio.Optional(ATTR_TIMEOUT_S): probatio.All(
                probatio.Coerce(float), probatio.Range(min=0, min_included=False)
            ),
            **_MAX_AGE_FIELD,
        }
    ),
    probatio.AtLeastOne(ATTR_DEVICE_ID, ATTR_ROBOT_ID),
)

CANCEL_TASK_SCHEMA: VolSchemaType = probatio.All(
    probatio.Schema(
        {
            **_TARGET_FIELDS,
            probatio.Optional(ATTR_TASK_ID): cv.string,
            **_WAIT_FIELD,
            **_MAX_AGE_FIELD,
        }
    ),
    probatio.AtLeastOne(ATTR_DEVICE_ID, ATTR_ROBOT_ID),
)

GET_ROBOTS_SCHEMA: VolSchemaType = probatio.Schema({})


# --------------------------------------------------------------------------- hub lookup


@callback
def async_get_hub(hass: HomeAssistant) -> BridgeHub | None:
    """The started hub, or ``None`` when the config entry is not loaded.

    ``hass.data[DOMAIN][DATA_HUB]`` is where ``async_setup_entry`` leaves it
    (and its unload teardown removes it); the loaded config entry's
    ``runtime_data`` is the fallback so the services and intents do not depend
    on that key alone.
    """
    container = hass.data.get(DOMAIN)
    if isinstance(container, dict):
        hub = container.get(DATA_HUB)
        if isinstance(hub, BridgeHub) and hub.started:
            return hub
    for entry in hass.config_entries.async_loaded_entries(DOMAIN):
        candidate = getattr(entry, "runtime_data", None)
        if isinstance(candidate, BridgeHub) and candidate.started:
            return candidate
    return None


def _require_hub(hass: HomeAssistant) -> BridgeHub:
    hub = async_get_hub(hass)
    if hub is None:
        raise HomeAssistantError(translation_domain=DOMAIN, translation_key="not_loaded")
    return hub


def _target_robot_ids(hass: HomeAssistant, data: Mapping[str, Any]) -> list[str]:
    """The robots a call addresses: ``robot_id`` plus the robot behind each ``device_id``."""
    robot_ids: list[str] = []
    robot_id = data.get(ATTR_ROBOT_ID)
    if isinstance(robot_id, str):
        robot_ids.append(robot_id)
    registry = dr.async_get(hass)
    for device_id in data.get(ATTR_DEVICE_ID) or ():
        device = registry.async_get(device_id)
        found: str | None = None
        if device is not None:
            for domain, identifier in device.identifiers:
                if domain == DOMAIN:
                    found = identifier
                    break
        if found is None:
            raise ServiceValidationError(
                translation_domain=DOMAIN,
                translation_key="robot_unknown",
                translation_placeholders={"robot_id": str(device_id)},
            )
        if found not in robot_ids:
            robot_ids.append(found)
    return robot_ids


def _call_source(call: ServiceCall) -> CommandSource:
    """``cmd.source`` for a service call; the hub fills in ``instance_id``."""
    return CommandSource(
        kind=SOURCE_KIND_SERVICE, user_id=call.context.user_id, context_id=call.context.id
    )


# --------------------------------------------------------------------------- waiting


async def _async_await(future: asyncio.Future[PendingCommand], timeout: float) -> bool:
    """Await a registry future for at most ``timeout`` seconds; True when it resolved.

    A future the registry cancelled (the hub was unloaded while the caller
    waited) is reported as ``not_loaded`` rather than cancelling the service
    call; a cancellation of the calling task itself is left alone.
    """
    try:
        await asyncio.wait_for(future, timeout)
    except TimeoutError:
        return False
    except asyncio.CancelledError:
        task = asyncio.current_task()
        if task is not None and task.cancelling():
            raise
        raise HomeAssistantError(translation_domain=DOMAIN, translation_key="not_loaded") from None
    return True


async def _async_wait_ack(hub: BridgeHub, command: PendingCommand) -> None:
    """Wait until the command leaves ``sent`` (ack, result or the no-ack deadline)."""
    try:
        future = hub.registry.wait_ack(command.task_id)
    except KeyError:
        return
    await _async_await(future, hub.options.ack_timeout_s + WAIT_SLACK_S)


async def _async_wait_result(hub: BridgeHub, command: PendingCommand) -> None:
    """Wait until the command is terminal; the registry's deadline decides, with slack.

    The deadline moves while the robot reports the task paused, so the wait is
    re-armed whenever the deadline changed or the task is still paused.
    """
    registry = hub.registry
    while command.is_outstanding:
        deadline = command.deadline
        remaining = 0.0 if deadline is None else max(0.0, (deadline - hub.now) / 1000)
        try:
            future = registry.wait_result(command.task_id)
        except KeyError:
            return
        if await _async_await(future, remaining + WAIT_SLACK_S):
            return
        if command.deadline == deadline and command.paused_at is None:
            return


async def _async_wait(
    hub: BridgeHub, command: PendingCommand, wait: str, *, result_of: PendingCommand | None
) -> None:
    """Apply a ``wait`` mode to a command; ``result_of`` is whose result ``result`` means."""
    if wait == WAIT_NONE:
        return
    if wait == WAIT_ACK or result_of is None:
        await _async_wait_ack(hub, command)
        return
    if result_of is not command:
        await _async_wait_ack(hub, command)
        if command.accepted is not True:
            return
    await _async_wait_result(hub, result_of)


# --------------------------------------------------------------------------- responses


def _outcome(command: PendingCommand) -> JsonObjectType | None:
    """The result view of a terminal command (``None`` while it is still outstanding)."""
    if command.is_outstanding:
        return None
    result = command.result
    status: str | None
    code: str | None
    reason: str | None
    data: JsonValueType
    started_at: int | None
    finished_at: int | None
    if result is not None:
        status = str(result.status)
        code, reason = result.code, result.reason
        data = dict(result.data)
        started_at, finished_at = result.started_at, result.finished_at
    else:
        status = None if command.outcome is None else str(command.outcome)
        code, reason = command.code, command.reason
        data = {}
        started_at, finished_at = command.sent_at, command.finished_at
    duration_ms = (
        None if started_at is None or finished_at is None else max(0, finished_at - started_at)
    )
    return {
        "task_id": command.task_id,
        "task": command.task,
        "status": status,
        "goal_status": None if result is None else result.goal_status,
        "code": code,
        "reason": reason,
        "data": data,
        "started_at": started_at,
        "finished_at": finished_at,
        "duration_ms": duration_ms,
        "synthetic": command.synthetic,
    }


def _task_entry(command: PendingCommand, *, result_of: PendingCommand | None) -> JsonObjectType:
    """One entry of the ``tasks`` response list (the ack view plus, when waited for, the result)."""
    ack = command.ack
    entry: JsonObjectType = {
        "robot_id": command.robot_id,
        "task_id": command.task_id,
        "task": command.task,
        "kind": None if command.kind is None else str(command.kind),
        "state": str(command.state),
        "accepted": command.accepted,
        "code": command.code if ack is None else ack.code,
        "reason": command.reason if ack is None else ack.reason,
        "clock_skew_ms": None if ack is None else ack.clock_skew_ms,
        "result": None if result_of is None else _outcome(result_of),
    }
    if command.is_cancel:
        entry["target_id"] = command.target_id if result_of is None else result_of.task_id
    return entry


def _raise_for_command(
    command: PendingCommand, wait: str, *, result_of: PendingCommand | None
) -> None:
    """Raise when a call without a response ended short of what ``wait`` asked for.

    Translation keys and placeholders: ``task_rejected`` {robot_id, task, code,
    reason}; ``no_ack`` {robot_id, task}; ``no_result`` {robot_id, task}. The
    hub's own keys are ``robot_unknown`` {robot_id}, ``robot_unavailable``
    {robot_id}, ``task_unknown`` {robot_id, task}, ``invalid_params`` {robot_id,
    task, problems}, ``too_many_pending`` {robot_id, limit}, ``not_ready`` and
    ``publish_failed`` {robot_id, error}; ``not_loaded`` has none.
    """
    task = command.task or str(command.type)
    if command.state is CommandState.REJECTED:
        ack = command.ack
        raise HomeAssistantError(
            translation_domain=DOMAIN,
            translation_key="task_rejected",
            translation_placeholders={
                "robot_id": command.robot_id,
                "task": task,
                "code": str(command.code if ack is None else ack.code),
                "reason": str(command.reason if ack is None else ack.reason),
            },
        )
    if wait == WAIT_NONE:
        return
    if command.state in (CommandState.CREATED, CommandState.SENT, CommandState.TIMEOUT_NO_ACK):
        raise HomeAssistantError(
            translation_domain=DOMAIN,
            translation_key="no_ack",
            translation_placeholders={"robot_id": command.robot_id, "task": task},
        )
    if wait != WAIT_RESULT or result_of is None:
        return
    if result_of.is_outstanding or result_of.state in (
        CommandState.TIMEOUT_NO_RESULT,
        CommandState.TIMEOUT_NO_ACK,
        CommandState.LOST,
    ):
        raise HomeAssistantError(
            translation_domain=DOMAIN,
            translation_key="no_result",
            translation_placeholders={
                "robot_id": result_of.robot_id,
                "task": result_of.task or task,
            },
        )


def _respond(
    call: ServiceCall, pairs: list[tuple[PendingCommand, PendingCommand | None]], wait: str
) -> ServiceResponse:
    if call.return_response:
        tasks: list[JsonValueType] = [
            _task_entry(command, result_of=target if wait == WAIT_RESULT else None)
            for command, target in pairs
        ]
        return {"tasks": tasks}
    for command, target in pairs:
        _raise_for_command(command, wait, result_of=target)
    return None


# --------------------------------------------------------------------------- handlers


async def _async_send_task(call: ServiceCall) -> ServiceResponse:
    """``send_task``: one command per targeted robot, then the requested wait."""
    hub = _require_hub(call.hass)
    data = call.data
    wait: str = data[ATTR_WAIT]
    source = _call_source(call)
    params = dict(data.get(ATTR_PARAMS) or {})
    robot_ids = _target_robot_ids(call.hass, data)
    for robot_id in robot_ids:
        hub.check_task(robot_id, data[ATTR_TASK], params)
    pairs: list[tuple[PendingCommand, PendingCommand | None]] = []
    for robot_id in robot_ids:
        command = await hub.async_send_task(
            robot_id,
            data[ATTR_TASK],
            params,
            source=source,
            timeout_s=data.get(ATTR_TIMEOUT_S),
            max_age_ms=data.get(ATTR_MAX_AGE_MS),
        )
        pairs.append((command, command))
    await asyncio.gather(
        *(_async_wait(hub, command, wait, result_of=target) for command, target in pairs)
    )
    return _respond(call, pairs, wait)


def _cancel_target(hub: BridgeHub, robot_id: str, target_id: str | None) -> PendingCommand | None:
    """The tracked command a cancel aims at, if the registry knows it."""
    if target_id is not None:
        return hub.registry.get(target_id)
    active = hub.registry.active_motion(robot_id)
    if active is not None:
        return active
    robot = hub.robots.get(robot_id)
    if robot is not None and robot.state is not None and robot.state.task_id is not None:
        return hub.registry.get(robot.state.task_id)
    return None


async def _async_cancel_task(call: ServiceCall) -> ServiceResponse:
    """``cancel_task``: ``wait: ack`` awaits the cancel's ack, ``wait: result`` the target's result."""
    hub = _require_hub(call.hass)
    data = call.data
    wait: str = data[ATTR_WAIT]
    source = _call_source(call)
    target_id = data.get(ATTR_TASK_ID)
    robot_ids = _target_robot_ids(call.hass, data)
    for robot_id in robot_ids:
        hub.check_cancel(robot_id, target_id)
    pairs: list[tuple[PendingCommand, PendingCommand | None]] = []
    for robot_id in robot_ids:
        target = _cancel_target(hub, robot_id, target_id)
        command = await hub.async_cancel_task(
            robot_id, target_id, source=source, max_age_ms=data.get(ATTR_MAX_AGE_MS)
        )
        pairs.append((command, target))
    await asyncio.gather(
        *(_async_wait(hub, command, wait, result_of=target) for command, target in pairs)
    )
    return _respond(call, pairs, wait)


async def _async_get_robots(call: ServiceCall) -> ServiceResponse:
    """``get_robots``: every announced robot in the ``robot_summary`` shape."""
    hub = _require_hub(call.hass)
    robots: list[JsonValueType] = [
        hub.robot_summary(robot_id)
        for robot_id, robot in sorted(hub.robots.items())
        if robot.announce is not None
    ]
    return {"robots": robots}


@callback
def async_register_services(hass: HomeAssistant) -> None:
    """Register the three services once; ``async_setup`` calls this."""
    if hass.services.has_service(DOMAIN, SERVICE_SEND_TASK):
        return
    hass.services.async_register(
        DOMAIN,
        SERVICE_SEND_TASK,
        _async_send_task,
        schema=SEND_TASK_SCHEMA,
        supports_response=SupportsResponse.OPTIONAL,
    )
    hass.services.async_register(
        DOMAIN,
        SERVICE_CANCEL_TASK,
        _async_cancel_task,
        schema=CANCEL_TASK_SCHEMA,
        supports_response=SupportsResponse.OPTIONAL,
    )
    hass.services.async_register(
        DOMAIN,
        SERVICE_GET_ROBOTS,
        _async_get_robots,
        schema=GET_ROBOTS_SCHEMA,
        supports_response=SupportsResponse.ONLY,
    )
