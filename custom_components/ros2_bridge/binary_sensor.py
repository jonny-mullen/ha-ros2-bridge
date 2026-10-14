# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Binary sensor platform of the ROS 2 Bridge integration.

One ``problem`` sensor per robot. It is on when the robot reports ``error``,
when its executor health is ``error``, when its heartbeat is stale while the
retained availability still says ``online``, or when the ``duplicate_robot_id``
repair is open; the ``reasons`` attribute lists which. The hub decides
(``problem_reasons``); the entity only shows it.
"""

from __future__ import annotations

from homeassistant.components.binary_sensor import BinarySensorEntity
from homeassistant.components.binary_sensor.const import BinarySensorDeviceClass
from homeassistant.core import HomeAssistant, callback
from homeassistant.helpers.dispatcher import async_dispatcher_connect
from homeassistant.helpers.entity_platform import AddConfigEntryEntitiesCallback

from .const import SIGNAL_ROBOT_ADDED, SIGNAL_ROBOT_REMOVED
from .entity import Ros2BridgeEntity
from .hub import BridgeHub, Ros2BridgeConfigEntry

#: Push entities; the hub writes at most a few states per second.
PARALLEL_UPDATES = 0


async def async_setup_entry(
    hass: HomeAssistant,
    entry: Ros2BridgeConfigEntry,
    async_add_entities: AddConfigEntryEntitiesCallback,
) -> None:
    """Create the problem sensor for every announced robot, now and as robots announce."""
    hub = entry.runtime_data
    added: set[str] = set()

    @callback
    def _async_add_robot(robot_id: str) -> None:
        if robot_id in added:
            return
        added.add(robot_id)
        async_add_entities([ProblemSensor(hub, robot_id)])

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


class ProblemSensor(Ros2BridgeEntity, BinarySensorEntity):
    """``binary_sensor.<robot>_problem``: on while the hub lists at least one problem reason."""

    _attr_device_class = BinarySensorDeviceClass.PROBLEM
    _attr_translation_key = "problem"

    def __init__(self, hub: BridgeHub, robot_id: str) -> None:
        super().__init__(hub, robot_id, "problem")

    @callback
    def _handle_robot_update(self) -> None:
        reasons = self.hub.problem_reasons(self.robot_id)
        self._attr_is_on = bool(reasons)
        self._attr_extra_state_attributes = {"reasons": reasons}
