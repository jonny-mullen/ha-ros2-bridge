# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Event platform of the ROS 2 Bridge integration.

One ``task_result`` event entity per robot. Its event types are the HA-side
outcome vocabulary (``succeeded``, ``cancelled``, ``aborted``, ``rejected``,
``timeout``, ``lost``) and it triggers exactly when the hub fires the
``ros2_bridge_task_result`` bus event for that robot, carrying the same data
(task, kind, code, reason and so on) as attributes. A late result
(``ros2_bridge_task_late_result``) is a correction to an outcome that has
already been reported, so it does not re-trigger the entity.
"""

from __future__ import annotations

from typing import Any

from homeassistant.components.event import EventEntity
from homeassistant.core import HomeAssistant, callback
from homeassistant.helpers.dispatcher import async_dispatcher_connect
from homeassistant.helpers.entity_platform import AddConfigEntryEntitiesCallback

from .const import (
    EVENT_TASK_RESULT,
    SIGNAL_ROBOT_ADDED,
    SIGNAL_ROBOT_REMOVED,
    SIGNAL_TASK_EVENT,
    TaskOutcome,
)
from .entity import Ros2BridgeEntity
from .hub import BridgeHub, Ros2BridgeConfigEntry

#: Push entities; the hub writes at most a few states per second.
PARALLEL_UPDATES = 0

#: The bus event fields shown as attributes of the triggered event.
EVENT_ATTRIBUTE_KEYS: tuple[str, ...] = (
    "task_id",
    "task",
    "kind",
    "code",
    "reason",
    "data",
    "goal_status",
    "started_at",
    "finished_at",
    "duration_ms",
    "synthetic",
    "replayed",
)


async def async_setup_entry(
    hass: HomeAssistant,
    entry: Ros2BridgeConfigEntry,
    async_add_entities: AddConfigEntryEntitiesCallback,
) -> None:
    """Create the task_result event entity for every announced robot, now and as robots announce."""
    hub = entry.runtime_data
    added: set[str] = set()

    @callback
    def _async_add_robot(robot_id: str) -> None:
        if robot_id in added:
            return
        added.add(robot_id)
        async_add_entities([TaskResultEvent(hub, robot_id)])

    @callback
    def _async_forget_robot(robot_id: str) -> None:
        added.discard(robot_id)

    entry.async_on_unload(async_dispatcher_connect(hass, SIGNAL_ROBOT_ADDED, _async_add_robot))
    entry.async_on_unload(
        async_dispatcher_connect(hass, SIGNAL_ROBOT_REMOVED, _async_forget_robot)
    )
    for robot_id, robot in list(hub.robots.items()):
        if robot.announce is not None:
            _async_add_robot(robot_id)


class TaskResultEvent(Ros2BridgeEntity, EventEntity):
    """``event.<robot>_task_result``: one trigger per task outcome, typed by the outcome."""

    _attr_translation_key = "task_result"

    def __init__(self, hub: BridgeHub, robot_id: str) -> None:
        super().__init__(hub, robot_id, "task_result")
        self._attr_event_types = [str(outcome) for outcome in TaskOutcome]

    async def async_added_to_hass(self) -> None:
        """Listen for the hub's task events as well as the robot signals."""
        await super().async_added_to_hass()
        self.async_on_remove(
            async_dispatcher_connect(self.hass, SIGNAL_TASK_EVENT, self._async_task_event)
        )

    @callback
    def _async_task_event(self, robot_id: str, event_name: str, data: dict[str, Any]) -> None:
        if robot_id != self.robot_id or event_name != EVENT_TASK_RESULT:
            return
        status = data.get("status")
        if not isinstance(status, str) or status not in self.event_types:
            return
        attributes = {key: data.get(key) for key in EVENT_ATTRIBUTE_KEYS}
        self._trigger_event(status, attributes)
        self.async_write_ha_state()
