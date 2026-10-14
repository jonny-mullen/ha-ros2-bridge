# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Config entry and device diagnostics: a snapshot, with the secret and user ids redacted.

The hub is built by hand on the fake clock (so every timestamp in the snapshot
is fixed) and the download goes through the diagnostics HTTP view, which also
proves the document is JSON-serialisable and that the platform registers.
"""

from __future__ import annotations

import uuid
from collections.abc import AsyncIterator
from typing import Any
from unittest.mock import patch

import pytest
from homeassistant.components.diagnostics import REDACTED
from homeassistant.core import HomeAssistant
from homeassistant.helpers import device_registry as dr
from homeassistant.setup import async_setup_component
from pytest_homeassistant_custom_component.common import MockConfigEntry
from pytest_homeassistant_custom_component.components.diagnostics import (
    get_diagnostics_for_config_entry,
    get_diagnostics_for_device,
)
from pytest_homeassistant_custom_component.syrupy import HomeAssistantSnapshotExtension
from pytest_homeassistant_custom_component.typing import ClientSessionGenerator
from syrupy.assertion import SnapshotAssertion
from syrupy.filters import props

from custom_components.ros2_bridge.commands import CommandRegistry
from custom_components.ros2_bridge.const import (
    CONF_INSTANCE_ID,
    CONF_SHARED_SECRET,
    DEFAULT_OPTIONS,
    DOMAIN,
    ISSUE_CLOCK_SKEW,
    issue_id,
)
from custom_components.ros2_bridge.diagnostics import (
    async_get_device_diagnostics,
    robot_id_of,
)
from custom_components.ros2_bridge.hub import BridgeHub
from custom_components.ros2_bridge.protocol import CommandSource
from custom_components.ros2_bridge.store import CommandStore

from .common import (
    INSTANCE_ID,
    RESET_ID,
    ROBOT_ID,
    TASK_ID,
    FakeClock,
    ack_payload,
    announce_payload,
    fire_robot_message,
    heartbeat_payload,
    rejected_ack_payload,
    state_payload,
)

#: A placeholder for tests only; it is not a credential.
PLACEHOLDER_SECRET = "placeholder-shared-secret"
ENTRY_ID = "ros2_bridge_diagnostics"


@pytest.fixture
def ha_snapshot(snapshot: SnapshotAssertion) -> SnapshotAssertion:
    """Syrupy with Home Assistant's extension bound explicitly.

    The harness plugin overrides syrupy's ``snapshot`` fixture with this extension,
    but which of the two plugins registers last is not fixed, so the extension is
    applied here by hand: the file then always lives in ``tests/snapshots``.
    """
    return snapshot.use_extension(HomeAssistantSnapshotExtension)


@pytest.fixture
async def started_hubs() -> AsyncIterator[list[BridgeHub]]:
    """Every hub a test starts by hand; each is stopped at teardown so no timer lingers."""
    hubs: list[BridgeHub] = []
    yield hubs
    for hub in hubs:
        await hub.async_stop()


async def start_hub(
    hass: HomeAssistant,
    entry: MockConfigEntry,
    clock: FakeClock,
    complete_hub_start: Any,
    started_hubs: list[BridgeHub],
) -> BridgeHub:
    """The wiring ``__init__.py`` performs, on the fake clock, driven to reconciled."""
    store = CommandStore(hass)
    data = await store.async_load()
    registry = CommandRegistry(clock=clock)
    registry.load(data)
    store.bind(registry)
    hub = BridgeHub(hass, entry, registry, store, clock=clock)
    entry.runtime_data = hub
    started_hubs.append(hub)
    await hub.async_start()
    await complete_hub_start(hub)
    return hub


async def build_scenario(
    hass: HomeAssistant,
    clock: FakeClock,
    complete_hub_start: Any,
    started_hubs: list[BridgeHub],
) -> tuple[MockConfigEntry, BridgeHub]:
    """One announced robot, a running task and a rejected one that opened a clock-skew issue."""
    # The component is set up before the entry exists so nothing tries to set the entry up;
    # the diagnostics platform is only found for a component that is loaded.
    assert await async_setup_component(hass, DOMAIN, {})
    entry = MockConfigEntry(
        domain=DOMAIN,
        title="ROS 2 Bridge",
        unique_id=DOMAIN,
        entry_id=ENTRY_ID,
        data={CONF_INSTANCE_ID: INSTANCE_ID},
        options={**DEFAULT_OPTIONS, CONF_SHARED_SECRET: PLACEHOLDER_SECRET},
    )
    entry.add_to_hass(hass)
    hub = await start_hub(hass, entry, clock, complete_hub_start, started_hubs)

    fire_robot_message(hass, "announce", announce_payload(), retain=True)
    fire_robot_message(hass, "availability", "online", retain=True)
    fire_robot_message(hass, "state", state_payload(), retain=True)
    fire_robot_message(hass, "heartbeat", heartbeat_payload())
    await hass.async_block_till_done()

    source = CommandSource(kind="service", user_id="user-1234", context_id="ctx-1")
    with patch("custom_components.ros2_bridge.hub.uuid.uuid4", return_value=uuid.UUID(TASK_ID)):
        await hub.async_send_task(ROBOT_ID, "move_to", {"pose": "home"}, source=source)
    fire_robot_message(hass, f"task/{TASK_ID}/ack", ack_payload())
    with patch("custom_components.ros2_bridge.hub.uuid.uuid4", return_value=uuid.UUID(RESET_ID)):
        await hub.async_send_task(ROBOT_ID, "reset", {}, source=source)
    fire_robot_message(
        hass,
        f"task/{RESET_ID}/ack",
        rejected_ack_payload(id=RESET_ID, task="reset", code="rejected_stale", clock_skew_ms=5000),
    )
    await hass.async_block_till_done()
    assert issue_id(ISSUE_CLOCK_SKEW, ROBOT_ID) in hub.open_issues
    return entry, hub


async def test_config_entry_diagnostics(
    hass: HomeAssistant,
    hass_client: ClientSessionGenerator,
    mqtt_setup: Any,
    clock: FakeClock,
    complete_hub_start: Any,
    ha_snapshot: SnapshotAssertion,
    started_hubs: list[BridgeHub],
) -> None:
    entry, _hub = await build_scenario(hass, clock, complete_hub_start, started_hubs)
    data = await get_diagnostics_for_config_entry(hass, hass_client, entry)
    assert data == ha_snapshot(exclude=props("device_id"))

    assert data["entry"]["options"][CONF_SHARED_SECRET] == REDACTED
    assert data["entry"]["data"][CONF_INSTANCE_ID] == INSTANCE_ID
    assert CONF_SHARED_SECRET not in data["hub"]["options"]
    assert data["hub"]["options"]["secret_configured"] is True
    commands = {c["task_id"]: c for c in data["hub"]["commands"]}
    assert commands[TASK_ID]["state"] == "running"
    assert commands[TASK_ID]["source"]["user_id"] == REDACTED
    assert commands[TASK_ID]["source"]["context_id"] == "ctx-1"
    assert commands[RESET_ID]["state"] == "rejected"
    assert data["store"]["outstanding"][TASK_ID]["source"]["user_id"] == REDACTED
    assert data["hub"]["issues"] == [issue_id(ISSUE_CLOCK_SKEW, ROBOT_ID)]


async def test_device_diagnostics(
    hass: HomeAssistant,
    hass_client: ClientSessionGenerator,
    mqtt_setup: Any,
    clock: FakeClock,
    complete_hub_start: Any,
    ha_snapshot: SnapshotAssertion,
    started_hubs: list[BridgeHub],
) -> None:
    entry, _hub = await build_scenario(hass, clock, complete_hub_start, started_hubs)
    device = dr.async_get(hass).async_get_device_by_identifier((DOMAIN, ROBOT_ID), entry.entry_id)
    assert device is not None
    assert robot_id_of(device) == ROBOT_ID
    data = await get_diagnostics_for_device(hass, hass_client, entry, device)
    assert data == ha_snapshot(exclude=props("device_id"))

    assert data["robot_id"] == ROBOT_ID
    assert data["robot"]["announce"]["name"] == "Left arm"
    assert {c["task_id"] for c in data["commands"]} == {TASK_ID, RESET_ID}
    assert all(c["source"]["user_id"] == REDACTED for c in data["commands"])
    assert data["issues"] == [issue_id(ISSUE_CLOCK_SKEW, ROBOT_ID)]


async def test_device_diagnostics_for_a_foreign_device(
    hass: HomeAssistant,
    mqtt_setup: Any,
    clock: FakeClock,
    complete_hub_start: Any,
    started_hubs: list[BridgeHub],
) -> None:
    """A device without a ros2_bridge identifier yields an empty record, not an error."""
    entry, _hub = await build_scenario(hass, clock, complete_hub_start, started_hubs)
    device = dr.async_get(hass).async_get_or_create(
        config_entry_id=entry.entry_id, identifiers={("other_domain", "thing")}
    )
    assert robot_id_of(device) is None
    data = await async_get_device_diagnostics(hass, entry, device)
    assert data["robot_id"] is None
    assert data["robot"] is None
    assert data["commands"] == []
    assert data["issues"] == []
    assert data["mqtt_connected"] is True
