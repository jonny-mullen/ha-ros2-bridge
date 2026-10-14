# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Button platform of the ROS 2 Bridge integration.

One button per announced task that needs no parameters (a task with required
parameters is reachable only through the ``send_task`` service) plus a
``cancel`` button per robot. A press publishes the command with
``source.kind`` ``button`` and waits for the acknowledgement: a rejected ack
raises ``task_rejected`` and a missing one ``no_ack`` (outcome unknown), so
the UI shows why nothing moved. The catalogue is dynamic: when a re-announce
adds or removes tasks, or changes which tasks need parameters, the buttons
follow; when a robot's device is removed its buttons go too.
"""

from __future__ import annotations

import asyncio
import logging

from homeassistant.components.button import ButtonEntity
from homeassistant.core import HomeAssistant, callback
from homeassistant.exceptions import HomeAssistantError
from homeassistant.helpers import entity_registry as er
from homeassistant.helpers.dispatcher import async_dispatcher_connect
from homeassistant.helpers.entity_platform import AddConfigEntryEntitiesCallback

from .const import (
    DOMAIN,
    SIGNAL_ROBOT_ADDED,
    SIGNAL_ROBOT_REMOVED,
    SIGNAL_ROBOT_TASKS_CHANGED,
    SIGNAL_ROBOT_UPDATED,
    SOURCE_KIND_BUTTON,
    CommandState,
)
from .entity import Ros2BridgeEntity
from .hub import BridgeHub, Ros2BridgeConfigEntry
from .models import PendingCommand, TaskSpec
from .protocol import CommandSource

_LOGGER = logging.getLogger(__name__)

#: Push entities; the hub writes at most a few states per second.
PARALLEL_UPDATES = 0

#: Slack added to the configured ack timeout so the registry's own no-ack timer fires first.
ACK_WAIT_SLACK_S = 1.0


async def async_setup_entry(
    hass: HomeAssistant,
    entry: Ros2BridgeConfigEntry,
    async_add_entities: AddConfigEntryEntitiesCallback,
) -> None:
    """Keep one button per parameterless task and a cancel button per announced robot."""
    hub = entry.runtime_data
    manager = _ButtonManager(hass, hub, async_add_entities)
    entry.async_on_unload(
        async_dispatcher_connect(hass, SIGNAL_ROBOT_ADDED, manager.async_sync_robot)
    )
    entry.async_on_unload(
        async_dispatcher_connect(hass, SIGNAL_ROBOT_TASKS_CHANGED, manager.async_tasks_changed)
    )
    entry.async_on_unload(
        async_dispatcher_connect(hass, SIGNAL_ROBOT_UPDATED, manager.async_sync_robot)
    )
    entry.async_on_unload(
        async_dispatcher_connect(hass, SIGNAL_ROBOT_REMOVED, manager.async_remove_robot)
    )
    for robot_id, robot in list(hub.robots.items()):
        if robot.announce is not None:
            manager.async_sync_robot(robot_id)


class _ButtonManager:
    """Reconciles the buttons of every robot with its announced catalogue."""

    def __init__(
        self,
        hass: HomeAssistant,
        hub: BridgeHub,
        async_add_entities: AddConfigEntryEntitiesCallback,
    ) -> None:
        self.hass = hass
        self.hub = hub
        self._async_add_entities = async_add_entities
        #: robot id to the task buttons by task name.
        self._task_buttons: dict[str, dict[str, TaskButton]] = {}
        #: robot id to its cancel button.
        self._cancel_buttons: dict[str, CancelButton] = {}

    @callback
    def async_tasks_changed(
        self, robot_id: str, _added: tuple[str, ...], _removed: tuple[str, ...]
    ) -> None:
        """A re-announce changed the catalogue; the diff is recomputed from the robot itself."""
        self.async_sync_robot(robot_id)

    @callback
    def async_sync_robot(self, robot_id: str) -> None:
        """Add the buttons the catalogue calls for and remove the ones it no longer does."""
        robot = self.hub.robots.get(robot_id)
        if robot is None or robot.announce is None:
            return
        wanted: dict[str, TaskSpec] = {
            spec.name: spec for spec in robot.tasks if not spec.needs_params
        }
        current = self._task_buttons.setdefault(robot_id, {})
        new_entities: list[Ros2BridgeEntity] = []
        for name, spec in wanted.items():
            if name not in current:
                button = TaskButton(self.hub, robot_id, spec)
                current[name] = button
                new_entities.append(button)
        if robot_id not in self._cancel_buttons:
            cancel = CancelButton(self.hub, robot_id)
            self._cancel_buttons[robot_id] = cancel
            new_entities.append(cancel)
        for name in [name for name in current if name not in wanted]:
            self._async_schedule_removal(current.pop(name))
        if new_entities:
            self._async_add_entities(new_entities)

    @callback
    def async_remove_robot(self, robot_id: str) -> None:
        """A tombstone: drop every button of the robot."""
        for button in self._task_buttons.pop(robot_id, {}).values():
            self._async_schedule_removal(button)
        cancel = self._cancel_buttons.pop(robot_id, None)
        if cancel is not None:
            self._async_schedule_removal(cancel)

    @callback
    def _async_schedule_removal(self, entity: Ros2BridgeEntity) -> None:
        self.hass.async_create_task(
            self._async_remove_entity(entity), f"{DOMAIN} remove {entity.unique_id}"
        )

    async def _async_remove_entity(self, entity: Ros2BridgeEntity) -> None:
        """Remove the entity and its registry entry so a renamed task leaves nothing behind."""
        registry = er.async_get(self.hass)
        if entity.unique_id is not None and (
            entity_id := registry.async_get_entity_id("button", DOMAIN, entity.unique_id)
        ):
            registry.async_remove(entity_id)
        if entity.hass is not None:
            await entity.async_remove(force_remove=True)


class _CommandButton(Ros2BridgeEntity, ButtonEntity):
    """Shared press handling: publish, wait for the ack, raise on rejection or silence."""

    def _source(self) -> CommandSource:
        context = self._context
        return CommandSource(
            kind=SOURCE_KIND_BUTTON,
            user_id=None if context is None else context.user_id,
            context_id=None if context is None else context.id,
        )

    async def _async_wait_ack(self, command: PendingCommand, task: str) -> None:
        """Await the ack; a rejection raises ``task_rejected`` and silence raises ``no_ack``.

        A wait the registry cancels (the entry was unloaded or reloaded while the
        press was pending) raises ``not_loaded`` rather than cancelling the
        ``button.press`` call; a cancellation of the pressing task itself is left alone.
        """
        timeout = self.hub.options.ack_timeout_s + ACK_WAIT_SLACK_S
        try:
            acked = await asyncio.wait_for(
                self.hub.registry.wait_ack(command.task_id), timeout=timeout
            )
        except TimeoutError:
            acked = command
        except asyncio.CancelledError:
            current = asyncio.current_task()
            if current is not None and current.cancelling():
                raise
            raise HomeAssistantError(
                translation_domain=DOMAIN, translation_key="not_loaded"
            ) from None
        if acked.state is CommandState.REJECTED:
            raise HomeAssistantError(
                translation_domain=DOMAIN,
                translation_key="task_rejected",
                translation_placeholders={
                    "robot_id": self.robot_id,
                    "task": task,
                    "code": acked.code or "",
                    "reason": acked.reason or "",
                },
            )
        if acked.state in (CommandState.TIMEOUT_NO_ACK, CommandState.SENT):
            raise HomeAssistantError(
                translation_domain=DOMAIN,
                translation_key="no_ack",
                translation_placeholders={"robot_id": self.robot_id, "task": task},
            )


class TaskButton(_CommandButton):
    """``button.<robot>_<label>``: start one announced task that takes no parameters.

    The name is the announced label through the ``task`` translation (``{label}``);
    a task relabelled by a later announce keeps its button name until the entry
    is reloaded, because the entity registry's original name is fixed on creation.
    """

    _attr_translation_key = "task"

    def __init__(self, hub: BridgeHub, robot_id: str, spec: TaskSpec) -> None:
        super().__init__(hub, robot_id, f"task_{spec.name}")
        self.task_name = spec.name
        self.label = spec.label
        self._attr_translation_placeholders = {"label": spec.label}

    async def async_press(self) -> None:
        """Send the task with ``source.kind`` ``button`` and wait for its acknowledgement."""
        command = await self.hub.async_send_task(
            self.robot_id, self.task_name, {}, source=self._source()
        )
        await self._async_wait_ack(command, self.task_name)


class CancelButton(_CommandButton):
    """``button.<robot>_cancel``: cancel the robot's active motion task."""

    _attr_translation_key = "cancel"

    def __init__(self, hub: BridgeHub, robot_id: str) -> None:
        super().__init__(hub, robot_id, "cancel")

    async def async_press(self) -> None:
        """Send a cancel for the active task and wait for its acknowledgement."""
        command = await self.hub.async_cancel_task(self.robot_id, None, source=self._source())
        await self._async_wait_ack(command, "cancel")
