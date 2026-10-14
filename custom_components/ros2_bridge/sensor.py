# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Sensor platform of the ROS 2 Bridge integration.

Two enum sensors per robot: ``state`` (the main feature: ``idle``, ``busy`` or
``error`` with the retained state's fields and the latest feedback as
attributes) and ``last_result`` (the outcome of the most recent motion task;
control and query results never touch it). Both are created when a robot
announces and follow the hub's dispatcher signals; the device registry cascade
removes them when the robot's device goes with a tombstone.
"""

from __future__ import annotations

from typing import Any

from homeassistant.components.sensor import SensorDeviceClass, SensorEntity
from homeassistant.core import HomeAssistant, callback
from homeassistant.helpers.dispatcher import async_dispatcher_connect
from homeassistant.helpers.entity_platform import AddConfigEntryEntitiesCallback

from .const import SIGNAL_ROBOT_ADDED, SIGNAL_ROBOT_REMOVED, RobotStatus, TaskOutcome
from .entity import Ros2BridgeEntity
from .hub import BridgeHub, Ros2BridgeConfigEntry

#: Push entities; the hub writes at most a few states per second.
PARALLEL_UPDATES = 0

#: Attributes that change often or carry no history value; never written to the recorder.
STATE_UNRECORDED_ATTRIBUTES = frozenset({"progress", "stage", "message", "since", "session"})


async def async_setup_entry(
    hass: HomeAssistant,
    entry: Ros2BridgeConfigEntry,
    async_add_entities: AddConfigEntryEntitiesCallback,
) -> None:
    """Create the two sensors for every announced robot, now and as robots announce."""
    hub = entry.runtime_data
    added: set[str] = set()

    @callback
    def _async_add_robot(robot_id: str) -> None:
        if robot_id in added:
            return
        added.add(robot_id)
        async_add_entities([StateSensor(hub, robot_id), LastResultSensor(hub, robot_id)])

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


class StateSensor(Ros2BridgeEntity, SensorEntity):
    """``sensor.<robot>_state``: the robot's discrete state with the retained fields as attributes."""

    _attr_device_class = SensorDeviceClass.ENUM
    _attr_translation_key = "state"
    _unrecorded_attributes = STATE_UNRECORDED_ATTRIBUTES

    def __init__(self, hub: BridgeHub, robot_id: str) -> None:
        super().__init__(hub, robot_id, "state")
        self._attr_options = [str(status) for status in RobotStatus]

    @callback
    def _handle_robot_update(self) -> None:
        robot = self.robot
        state = robot.state if robot is not None else None
        feedback = robot.feedback if robot is not None else None
        self._attr_native_value = None if state is None else str(state.status)
        attributes: dict[str, Any] = {
            "task_id": None if state is None else state.task_id,
            "task": None if state is None else state.task,
            "since": None if state is None else state.since,
            "paused": None if state is None else state.paused,
            "inhibited": None if state is None else state.inhibited,
            "ha_link": None if state is None else str(state.ha_link),
            "watchdog_mode": None if state is None else str(state.watchdog_mode),
            "executor_health": (
                None
                if state is None
                else {
                    "level": str(state.executor_health_level),
                    "message": state.executor_health_message,
                }
            ),
            "error_code": None if state is None else state.error_code,
            "error_reason": None if state is None else state.error_reason,
            "session": None if state is None else state.session,
            "instance": None if state is None else state.instance,
            "progress": None if feedback is None else feedback.progress,
            "stage": None if feedback is None else feedback.stage,
            "message": None if feedback is None else feedback.message,
        }
        self._attr_extra_state_attributes = attributes


class LastResultSensor(Ros2BridgeEntity, SensorEntity):
    """``sensor.<robot>_last_result``: the outcome of the latest motion task."""

    _attr_device_class = SensorDeviceClass.ENUM
    _attr_translation_key = "last_result"

    def __init__(self, hub: BridgeHub, robot_id: str) -> None:
        super().__init__(hub, robot_id, "last_result")
        self._attr_options = [str(outcome) for outcome in TaskOutcome]

    @callback
    def _handle_robot_update(self) -> None:
        robot = self.robot
        last = robot.last_result if robot is not None else None
        self._attr_native_value = None if last is None else str(last.outcome)
        self._attr_extra_state_attributes = {
            "task_id": None if last is None else last.task_id,
            "task": None if last is None else last.task,
            "code": None if last is None else last.code,
            "reason": None if last is None else last.reason,
            "data": {} if last is None else dict(last.data),
            "finished_at": None if last is None else last.finished_at,
            "duration_ms": None if last is None else last.duration_ms,
            "synthetic": False if last is None else last.synthetic,
        }
