# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The ROS 2 Bridge integration.

A thin MQTT-only integration that rides Home Assistant's own MQTT connection.
``async_setup`` runs once per Home Assistant start: it registers the services
and the Assist intents (neither is ever removed, so an options reload cannot
silence voice) and creates the container in ``hass.data`` that keeps a config
entry's ``CommandRegistry`` alive across reloads. ``async_setup_entry`` builds
the ``BridgeHub`` on top of the Store-backed registry, attaches the sensor
mirror, publishes the hub at ``hass.data[DOMAIN]["hub"]``, registers the hub's
teardown with ``entry.async_on_unload`` (so a setup that fails part-way leaves
no subscription or timer behind), starts it and forwards the four entity
platforms; ``async_unload_entry`` stops the hub and parks the registry;
``async_remove_entry`` clears what the integration left on the broker and on
disk. Device removal from the UI is allowed only for robots that are offline
or unknown, and publishes the contract's tombstones whether or not the entry
is loaded.
"""

from __future__ import annotations

import importlib
import inspect
import logging
from typing import Any, Final

from homeassistant.components import mqtt
from homeassistant.config_entries import ConfigEntry
from homeassistant.const import Platform
from homeassistant.core import HomeAssistant
from homeassistant.exceptions import ConfigEntryNotReady
from homeassistant.helpers import device_registry as dr
from homeassistant.helpers import issue_registry as ir
from homeassistant.helpers.typing import ConfigType

from .commands import CommandRegistry
from .const import DATA_HUB, DOMAIN
from .hub import (
    BridgeHub,
    HubOptions,
    Ros2BridgeConfigEntry,
    async_clear_robot_topics,
    default_clock,
)
from .sensors_out import async_clear_published_topics, async_setup_sensor_mirror
from .store import CommandStore

_LOGGER = logging.getLogger(__name__)

PLATFORMS: Final = [Platform.SENSOR, Platform.BINARY_SENSOR, Platform.BUTTON, Platform.EVENT]

#: ``hass.data[DOMAIN][DATA_REGISTRIES]``: entry id to the ``CommandRegistry`` parked at unload.
DATA_REGISTRIES: Final = "registries"


def _registries(hass: HomeAssistant) -> dict[str, CommandRegistry]:
    """The parked-registry container, created on first use so it outlives entry reloads."""
    domain_data: dict[str, Any] = hass.data.setdefault(DOMAIN, {})
    registries: dict[str, CommandRegistry] = domain_data.setdefault(DATA_REGISTRIES, {})
    return registries


async def _async_call_registrar(hass: HomeAssistant, module_name: str, function: str) -> None:
    """Import ``module_name`` by name and call its registrar, awaiting it if it is a coroutine.

    The services and intent modules are imported lazily by name so that this
    module stays importable (and hassfest-checkable) on its own, and so that a
    test can stand in a stub through ``sys.modules`` without touching the files.
    """
    # Importing inside the event loop is a blocking call that Home Assistant flags;
    # the import executor is the sanctioned place for it.
    module = await hass.async_add_import_executor_job(
        importlib.import_module, module_name, __package__
    )
    result = getattr(module, function)(hass)
    if inspect.isawaitable(result):
        await result


async def async_setup(hass: HomeAssistant, config: ConfigType) -> bool:
    """Register the services and intents once per start and create the shared container.

    Intent handlers are never removed: ``intent.async_remove`` pops a handler
    globally, which would silence voice after the first options save.
    """
    _registries(hass)
    await _async_call_registrar(hass, ".services", "async_register_services")
    await _async_call_registrar(hass, ".intent", "async_register_intents")
    return True


async def async_setup_entry(hass: HomeAssistant, entry: Ros2BridgeConfigEntry) -> bool:
    """Build the hub on a Store-backed registry, start it and forward the platforms.

    The registry is taken out of the parked container when an unload left it
    there (an options reload), otherwise created and loaded from the Store. A
    parked registry has its deadlines suspended again until the first
    reconciliation after the retained replay, exactly as a fresh one.
    """
    if not await mqtt.async_wait_for_mqtt_client(hass):
        raise ConfigEntryNotReady("The MQTT integration is not available yet")
    registries = _registries(hass)
    store = CommandStore(hass)
    data = await store.async_load()
    registry = registries.pop(entry.entry_id, None)
    if registry is None:
        registry = CommandRegistry(clock=default_clock)
        registry.load(data)
    else:
        registry.deadlines_suspended = True
    store.bind(registry)
    hub = BridgeHub(hass, entry, registry, store)
    entry.runtime_data = hub
    # The mirror registers itself as a hub component: the hub starts it with the
    # subscriptions and stops it with them.
    async_setup_sensor_mirror(hub)
    domain_data: dict[str, Any] = hass.data[DOMAIN]
    domain_data[DATA_HUB] = hub

    async def _async_teardown() -> None:
        """Forget and stop the hub; runs at unload and when this setup fails part-way.

        Home Assistant runs the on-unload callbacks when ``async_setup_entry``
        raises as well as on a normal unload, and ``async_stop`` is idempotent,
        so the explicit stop in ``async_unload_entry`` and this one never clash.
        """
        if domain_data.get(DATA_HUB) is hub:
            domain_data.pop(DATA_HUB)
        await hub.async_stop()

    entry.async_on_unload(_async_teardown)
    try:
        await hub.async_start()
    except Exception:
        # A failure after the first subscription must not leave the rest running.
        await hub.async_stop()
        raise
    await hass.config_entries.async_forward_entry_setups(entry, PLATFORMS)
    return True


async def async_unload_entry(hass: HomeAssistant, entry: Ros2BridgeConfigEntry) -> bool:
    """Unload the platforms, stop the hub and park the registry for the next setup.

    Stopping the hub flushes the Store and cancels every pending ``wait_ack``
    and ``wait_result`` future; the commands themselves stay outstanding and
    are reconciled by the next setup.
    """
    unload_ok = await hass.config_entries.async_unload_platforms(entry, PLATFORMS)
    # An entry marked loaded by a patched setup (tests) has no hub; there is nothing to stop.
    hub: BridgeHub | None = getattr(entry, "runtime_data", None)
    if unload_ok and hub is not None:
        await hub.async_stop()
        _registries(hass)[entry.entry_id] = hub.registry
    return unload_ok


async def async_remove_entry(hass: HomeAssistant, entry: ConfigEntry) -> None:
    """Clear the recorded sensor topics, drop the parked registry, the Store and the repairs.

    The entry is already unloaded here, so the Store is opened afresh to read
    the topics the sensor mirror published; each is cleared with an empty
    retained publish before the document is deleted.
    """
    _registries(hass).pop(entry.entry_id, None)
    store = CommandStore(hass)
    await store.async_load()
    await async_clear_published_topics(hass, store)
    await store.async_remove()
    issue_registry = ir.async_get(hass)
    for domain, issue_id in list(issue_registry.issues):
        if domain == DOMAIN:
            ir.async_delete_issue(hass, DOMAIN, issue_id)


async def async_remove_config_entry_device(
    hass: HomeAssistant, entry: Ros2BridgeConfigEntry, device_entry: dr.DeviceEntry
) -> bool:
    """Allow a device to be removed from the UI only while its robot is offline or unknown.

    Removal publishes empty retained payloads to the robot's announce, state
    and availability topics so the broker forgets it too; the node's own echo
    of the tombstone is a no-op. While the entry is not loaded there is no hub
    to ask, so the robot counts as unknown and the tombstones are published
    straight through the MQTT helpers under the entry's configured prefix;
    otherwise the retained announce would recreate the device at the next setup.
    """
    robot_ids = [robot_id for domain, robot_id in device_entry.identifiers if domain == DOMAIN]
    if not robot_ids:
        return True
    hub: BridgeHub | None = getattr(entry, "runtime_data", None)
    if hub is None:
        prefix = HubOptions.from_entry(entry).topic_prefix
        for robot_id in robot_ids:
            await async_clear_robot_topics(hass, prefix, robot_id)
        return True
    for robot_id in robot_ids:
        if not hub.is_removable(robot_id):
            _LOGGER.info("Robot %s is online; take it offline before removing it", robot_id)
            return False
    for robot_id in robot_ids:
        await hub.async_forget_robot(robot_id)
    return True


async def async_migrate_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Migrate an entry to the current version; version 1 is the first and only one so far."""
    _LOGGER.debug(
        "Config entry %s is at version %s.%s", entry.entry_id, entry.version, entry.minor_version
    )
    return True
