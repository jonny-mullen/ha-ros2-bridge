# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Base entity of the ROS 2 Bridge integration.

Every entity belongs to one robot's device, carries ``_attr_has_entity_name``,
never polls, derives its availability from the hub's rule (robot online,
announced this run, MQTT connected, heartbeat not stale beyond
``heartbeat_unavailable_after_s``) and refreshes on the hub's dispatcher
signals. Subclasses override ``_handle_robot_update`` to pull new values from
``self.robot`` before the state is written.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from homeassistant.core import callback
from homeassistant.helpers.dispatcher import async_dispatcher_connect
from homeassistant.helpers.entity import Entity

from .const import SIGNAL_CONNECTION_CHANGED, SIGNAL_ROBOT_UPDATED
from .models import Robot

if TYPE_CHECKING:
    from .hub import BridgeHub

#: Push entities: no polling, and the hub writes at most a few states per second.
PARALLEL_UPDATES = 0


class Ros2BridgeEntity(Entity):
    """Common wiring for every entity of one robot."""

    _attr_has_entity_name = True
    _attr_should_poll = False

    def __init__(self, hub: BridgeHub, robot_id: str, key: str) -> None:
        """Bind to ``robot_id``; ``key`` completes the unique id ``{robot_id}_{key}``."""
        self.hub = hub
        self.robot_id = robot_id
        self._attr_unique_id = f"{robot_id}_{key}"
        self._attr_device_info = hub.device_info(robot_id)

    @property
    def robot(self) -> Robot | None:
        """The hub's current view of this entity's robot (``None`` after a tombstone)."""
        return self.hub.robots.get(self.robot_id)

    @property
    def available(self) -> bool:
        """The hub's availability rule for this robot."""
        return self.hub.robot_available(self.robot_id)

    async def async_added_to_hass(self) -> None:
        """Subscribe to the hub's signals and take the current values."""
        await super().async_added_to_hass()
        self.async_on_remove(
            async_dispatcher_connect(self.hass, SIGNAL_ROBOT_UPDATED, self._async_robot_updated)
        )
        self.async_on_remove(
            async_dispatcher_connect(
                self.hass, SIGNAL_CONNECTION_CHANGED, self._async_connection_changed
            )
        )
        self._handle_robot_update()

    async def async_will_remove_from_hass(self) -> None:
        """Nothing beyond the base class; the dispatcher connections are removed by HA."""
        await super().async_will_remove_from_hass()

    @callback
    def async_entity_id_changed(self, old_entity_id: str) -> None:
        """Opt in to in-place entity_id changes (Home Assistant 2026.11 and later).

        The hook was announced for 2026.10 (developer blog, 2026-10-12) but
        ``homeassistant.helpers.entity.Entity`` at 2026.10.1 does not define it,
        so this is a plain method here. ``super()`` is called when the base class
        gains the hook, as the blog asks, and nothing of ours depends on the
        entity id, so there is nothing else to do.
        """
        parent = getattr(super(), "async_entity_id_changed", None)
        if callable(parent):
            parent(old_entity_id)

    @callback
    def _handle_robot_update(self) -> None:
        """Pull new values from ``self.robot``; subclasses override this."""

    @callback
    def _async_robot_updated(self, robot_id: str) -> None:
        if robot_id != self.robot_id:
            return
        self._handle_robot_update()
        self.async_write_ha_state()

    @callback
    def _async_connection_changed(self) -> None:
        self.async_write_ha_state()
