# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Sensor platform of the ROS 2 Bridge integration.

Two enum sensors per robot: ``state`` (the main feature: ``idle``, ``busy`` or
``error`` with the retained state's fields and the latest feedback as
attributes) and ``last_result`` (the outcome of the most recent motion task;
control and query results never touch it). Both are created when a robot
announces and follow the hub's dispatcher signals; the device registry cascade
removes them when the robot's device goes with a tombstone.

Since 0.2.0 a robot whose announce carries a ``joints`` block also gets one
numeric sensor per joint (``joint``), fed by the ``joints`` topic at the
node's rate. The joint sensors are dynamic like the task buttons: a
re-announce that adds, drops or reconfigures a joint (name, unit or limits at
an index) adds and removes the matching sensors, and an announce without the
block removes them all. Registry entries of joints dropped while Home
Assistant was stopped are swept on the first announce after a start, so no
restored, unavailable joint lingers on the device.
"""

from __future__ import annotations

import logging
from typing import Any

from homeassistant.components.sensor import SensorDeviceClass, SensorEntity
from homeassistant.core import HomeAssistant, callback
from homeassistant.helpers import entity_registry as er
from homeassistant.helpers.dispatcher import async_dispatcher_connect
from homeassistant.helpers.entity_platform import AddConfigEntryEntitiesCallback

from .const import (
    DOMAIN,
    SIGNAL_ROBOT_ADDED,
    SIGNAL_ROBOT_JOINTS_CHANGED,
    SIGNAL_ROBOT_JOINTS_UPDATED,
    SIGNAL_ROBOT_REMOVED,
    RobotStatus,
    TaskOutcome,
)
from .entity import Ros2BridgeEntity
from .hub import BridgeHub, Ros2BridgeConfigEntry
from .models import JointInfo

_LOGGER = logging.getLogger(__name__)

#: Push entities; the hub writes at most a few states per second per robot, plus the
#: joint sensors at the node's announced rate (2 per second by default).
PARALLEL_UPDATES = 0

#: Attributes that change often or carry no history value; never written to the recorder.
STATE_UNRECORDED_ATTRIBUTES = frozenset({"progress", "stage", "message", "since", "session"})


async def async_setup_entry(
    hass: HomeAssistant,
    entry: Ros2BridgeConfigEntry,
    async_add_entities: AddConfigEntryEntitiesCallback,
) -> None:
    """Create the sensors for every announced robot, now and as robots announce.

    The two enum sensors are made once per robot; the joint sensors are kept in
    step with the announced ``joints`` block by :class:`_JointSensorManager`.
    """
    hub = entry.runtime_data
    added: set[str] = set()
    joints = _JointSensorManager(hass, hub, async_add_entities)

    @callback
    def _async_add_robot(robot_id: str) -> None:
        if robot_id in added:
            return
        added.add(robot_id)
        entities: list[Ros2BridgeEntity] = [
            StateSensor(hub, robot_id),
            LastResultSensor(hub, robot_id),
            *joints.async_sync_robot(robot_id, add=False),
        ]
        async_add_entities(entities)

    @callback
    def _async_forget_robot(robot_id: str) -> None:
        added.discard(robot_id)
        joints.async_remove_robot(robot_id)

    entry.async_on_unload(async_dispatcher_connect(hass, SIGNAL_ROBOT_ADDED, _async_add_robot))
    entry.async_on_unload(
        async_dispatcher_connect(hass, SIGNAL_ROBOT_JOINTS_CHANGED, joints.async_joints_changed)
    )
    entry.async_on_unload(
        async_dispatcher_connect(hass, SIGNAL_ROBOT_REMOVED, _async_forget_robot)
    )
    for robot_id, robot in list(hub.robots.items()):
        if robot.announce is not None:
            _async_add_robot(robot_id)


class _JointSensorManager:
    """Reconciles the joint sensors of every robot with its announced ``joints`` block."""

    def __init__(
        self,
        hass: HomeAssistant,
        hub: BridgeHub,
        async_add_entities: AddConfigEntryEntitiesCallback,
    ) -> None:
        self.hass = hass
        self.hub = hub
        self._async_add_entities = async_add_entities
        #: robot id to its joint sensors by index.
        self._sensors: dict[str, dict[int, JointSensor]] = {}

    @callback
    def async_joints_changed(
        self, robot_id: str, _added: tuple[int, ...], _removed: tuple[int, ...]
    ) -> None:
        """A re-announce changed the block; the diff is recomputed from the robot itself."""
        self.async_sync_robot(robot_id)

    @callback
    def async_sync_robot(self, robot_id: str, *, add: bool = True) -> list[JointSensor]:
        """Add the sensors the block calls for and remove the ones it no longer describes.

        A sensor whose index is still announced but with another name, unit or
        limits is removed and made afresh, so its name and unit follow the robot.
        With ``add`` false the new sensors are returned for the caller to add in
        one batch with the robot's other entities.
        """
        robot = self.hub.robots.get(robot_id)
        if robot is None or robot.announce is None:
            return []
        info = robot.announce.joints
        current = self._sensors.setdefault(robot_id, {})
        for index in [
            index
            for index, sensor in current.items()
            if info is None or index >= info.count or sensor.key != info.joint_key(index)
        ]:
            self._async_schedule_removal(current.pop(index))
        self._async_remove_stale_registry_entries(robot_id, 0 if info is None else info.count)
        new_sensors: list[JointSensor] = []
        if info is not None:
            for index in range(info.count):
                if index not in current:
                    sensor = JointSensor(self.hub, robot_id, index, info)
                    current[index] = sensor
                    new_sensors.append(sensor)
        if add and new_sensors:
            self._async_add_entities(new_sensors)
        return new_sensors

    @callback
    def _async_remove_stale_registry_entries(self, robot_id: str, count: int) -> None:
        """Drop registry entries of joints the robot no longer announces and this run never made.

        A joint dropped while Home Assistant was stopped (the node restarted
        with ``telemetry.joints_rate_hz`` 0, or with another executor) is not
        in :attr:`_sensors`, so the removal above never sees it; its registry
        entry would otherwise come back as a restored, unavailable entity on
        the device until deleted by hand.
        """
        registry = er.async_get(self.hass)
        prefix = f"{robot_id}_joint_"
        live = self._sensors.get(robot_id, {})
        for entry in er.async_entries_for_config_entry(registry, self.hub.entry.entry_id):
            if entry.domain != "sensor" or entry.platform != DOMAIN:
                continue
            if not isinstance(entry.unique_id, str) or not entry.unique_id.startswith(prefix):
                continue
            suffix = entry.unique_id.removeprefix(prefix)
            if not suffix.isdigit() or (index := int(suffix)) in live or index < count:
                continue
            _LOGGER.debug("Removing %s: joint %d is no longer announced", entry.entity_id, index)
            registry.async_remove(entry.entity_id)

    @callback
    def async_remove_robot(self, robot_id: str) -> None:
        """A tombstone: drop every joint sensor of the robot."""
        for sensor in self._sensors.pop(robot_id, {}).values():
            self._async_schedule_removal(sensor)

    @callback
    def _async_schedule_removal(self, entity: Ros2BridgeEntity) -> None:
        self.hass.async_create_task(
            self._async_remove_entity(entity), f"{DOMAIN} remove {entity.unique_id}"
        )

    async def _async_remove_entity(self, entity: Ros2BridgeEntity) -> None:
        """Remove the entity and its registry entry so a reconfigured joint leaves nothing behind."""
        registry = er.async_get(self.hass)
        if entity.unique_id is not None and (
            entity_id := registry.async_get_entity_id("sensor", DOMAIN, entity.unique_id)
        ):
            registry.async_remove(entity_id)
        if entity.hass is not None:
            await entity.async_remove(force_remove=True)


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


class JointSensor(Ros2BridgeEntity, SensorEntity):
    """``sensor.<robot>_joint_<n>_<name>``: the position of one joint in the announced unit.

    The unique id is ``{robot_id}_joint_{index}`` with ``index`` counted from 0,
    the position's place in the ``joints`` payload; the name shows the index
    from 1 ("Joint 1 j1"). The attributes ``index`` (from 0), ``joint_name``,
    ``limit_min`` and ``limit_max`` are fixed for the life of the entity, because
    a change in any of them makes a new entity. The value is the latest
    accepted position; it becomes unknown when a re-announce changes the layout
    and unavailable with the robot, never on a quiet joints stream alone.
    """

    _attr_translation_key = "joint"
    _attr_suggested_display_precision = 1

    def __init__(self, hub: BridgeHub, robot_id: str, index: int, info: JointInfo) -> None:
        super().__init__(hub, robot_id, f"joint_{index}")
        self.index = index
        self.joint_name = info.names[index]
        #: The configuration this sensor was made for; the manager replaces it when it changes.
        self.key = info.joint_key(index)
        limit = info.limit(index)
        self._attr_native_unit_of_measurement = info.unit_symbol
        self._attr_translation_placeholders = {
            "index": str(index + 1),
            "joint_name": self.joint_name,
        }
        self._attr_extra_state_attributes = {
            "index": index,
            "joint_name": self.joint_name,
            "limit_min": None if limit is None else limit[0],
            "limit_max": None if limit is None else limit[1],
        }

    async def async_added_to_hass(self) -> None:
        """Listen for the joints stream as well as the robot signals."""
        await super().async_added_to_hass()
        self.async_on_remove(
            async_dispatcher_connect(
                self.hass, SIGNAL_ROBOT_JOINTS_UPDATED, self._async_robot_updated
            )
        )

    @callback
    def _handle_robot_update(self) -> None:
        robot = self.robot
        joints = robot.joints if robot is not None else None
        if joints is None or self.index >= len(joints.positions):
            self._attr_native_value = None
        else:
            self._attr_native_value = joints.positions[self.index]
