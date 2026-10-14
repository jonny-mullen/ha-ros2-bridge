# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Diagnostics of the ROS 2 Bridge integration.

The config entry download carries the entry itself (data and options), the
hub's own snapshot (``BridgeHub.diagnostics``: connection and readiness flags,
subscriptions, heartbeat, every robot, the last 50 commands, counters and open
issues) and the parked command Store. The device download narrows that to one
robot. The shared secret never appears (the hub reports only whether one is
configured) and user ids in command sources are redacted.
"""

from __future__ import annotations

from typing import Any

from homeassistant.components.diagnostics import async_redact_data
from homeassistant.core import HomeAssistant
from homeassistant.helpers.device_registry import DeviceEntry

from .const import CONF_SHARED_SECRET, DOMAIN, ISSUE_KINDS, issue_id
from .hub import BridgeHub, Ros2BridgeConfigEntry

#: Keys redacted wherever they appear in the download.
TO_REDACT: frozenset[str] = frozenset({CONF_SHARED_SECRET, "user_id"})


def _entry_section(entry: Ros2BridgeConfigEntry) -> dict[str, Any]:
    return {
        "title": entry.title,
        "version": entry.version,
        "minor_version": entry.minor_version,
        "data": dict(entry.data),
        "options": dict(entry.options),
    }


def robot_id_of(device: DeviceEntry) -> str | None:
    """The robot id behind a device entry, from its ``(ros2_bridge, robot_id)`` identifier."""
    for domain, identifier in device.identifiers:
        if domain == DOMAIN:
            return identifier
    return None


async def async_get_config_entry_diagnostics(
    hass: HomeAssistant, entry: Ros2BridgeConfigEntry
) -> dict[str, Any]:
    """Diagnostics for the config entry: the entry, the hub snapshot and the Store document."""
    hub: BridgeHub = entry.runtime_data
    data: dict[str, Any] = {
        "entry": _entry_section(entry),
        "hub": hub.diagnostics(),
        "store": hub.store.snapshot(),
    }
    return async_redact_data(data, TO_REDACT)


async def async_get_device_diagnostics(
    hass: HomeAssistant, entry: Ros2BridgeConfigEntry, device: DeviceEntry
) -> dict[str, Any]:
    """Diagnostics for one robot's device: its hub record, its commands and its open issues."""
    hub: BridgeHub = entry.runtime_data
    robot_id = robot_id_of(device)
    snapshot = hub.diagnostics()
    commands = [c for c in snapshot["commands"] if c.get("robot_id") == robot_id]
    open_issues = set(snapshot["issues"])
    issues = (
        []
        if robot_id is None
        else [
            issue_id(kind, robot_id)
            for kind in ISSUE_KINDS
            if issue_id(kind, robot_id) in open_issues
        ]
    )
    data: dict[str, Any] = {
        "robot_id": robot_id,
        "robot": snapshot["robots"].get(robot_id) if robot_id is not None else None,
        "commands": commands,
        "issues": issues,
        "mqtt_connected": snapshot["mqtt_connected"],
        "ready": snapshot["ready"],
        "reconciled": snapshot["reconciled"],
    }
    return async_redact_data(data, TO_REDACT)
