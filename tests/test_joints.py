# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Joint telemetry (0.2.0): the announce ``joints`` block, the joint sensors and the joints topic.

Messages after the first on a topic are fired without the retain flag, as a
broker delivers them live: Home Assistant's client hands a subscription the
retained copy of a topic once only.
"""

from __future__ import annotations

import logging
import sys
import types
from collections.abc import Awaitable, Callable
from typing import Any

import pytest
from homeassistant.components.mqtt import MQTT_CONNECTION_STATE
from homeassistant.core import HomeAssistant, callback
from homeassistant.helpers import device_registry as dr
from homeassistant.helpers import entity_registry as er
from homeassistant.helpers.dispatcher import async_dispatcher_connect, async_dispatcher_send
from pytest_homeassistant_custom_component.common import MockConfigEntry
from pytest_homeassistant_custom_component.syrupy import HomeAssistantSnapshotExtension
from syrupy.assertion import SnapshotAssertion

from custom_components.ros2_bridge import sensor
from custom_components.ros2_bridge.const import (
    DOMAIN,
    SIGNAL_ROBOT_JOINTS_CHANGED,
    joints_filter,
)
from custom_components.ros2_bridge.diagnostics import async_get_device_diagnostics
from custom_components.ros2_bridge.hub import BridgeHub
from custom_components.ros2_bridge.sensor import JointSensor

from .common import (
    OTHER_SESSION,
    PREFIX,
    ROBOT_ID,
    announce_payload,
    announce_with_joints_payload,
    fire_robot_message,
    idle_state_payload,
    joints_block,
    joints_payload,
)

JOINT_COUNT = 7
POSITIONS = [12.5, -34.0, 90.25, -45.5, 0.0, 60.75, -10.0]


@pytest.fixture
def snapshot(snapshot: SnapshotAssertion) -> SnapshotAssertion:
    """Syrupy with Home Assistant's serializer (ids and timestamps masked, ``tests/snapshots``)."""
    return snapshot.use_extension(HomeAssistantSnapshotExtension)


@pytest.fixture(autouse=True)
def stub_registrars(monkeypatch: pytest.MonkeyPatch) -> None:
    """Stand in the services and intent modules; only the hub and the sensors are under test."""
    for name, function in (
        ("services", "async_register_services"),
        ("intent", "async_register_intents"),
    ):
        module = types.ModuleType(f"custom_components.{DOMAIN}.{name}")
        setattr(module, function, lambda hass: None)
        monkeypatch.setitem(sys.modules, module.__name__, module)


@pytest.fixture
async def hub(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    complete_hub_start: Callable[[BridgeHub], Awaitable[None]],
) -> BridgeHub:
    """A ready hub with the golden robot announced with seven joints, online and idle."""
    entry = make_config_entry()
    assert await hass.config_entries.async_setup(entry.entry_id)
    await hass.async_block_till_done()
    started: BridgeHub = entry.runtime_data
    await complete_hub_start(started)
    fire_robot_message(hass, "announce", announce_with_joints_payload())
    fire_robot_message(hass, "availability", "online")
    fire_robot_message(hass, "state", idle_state_payload())
    await hass.async_block_till_done()
    return started


@pytest.fixture
def joint_diffs(hass: HomeAssistant) -> list[tuple[str, tuple[int, ...], tuple[int, ...]]]:
    """Every ``SIGNAL_ROBOT_JOINTS_CHANGED`` the hub sends, in order."""
    diffs: list[tuple[str, tuple[int, ...], tuple[int, ...]]] = []

    @callback
    def _record(robot_id: str, added: tuple[int, ...], removed: tuple[int, ...]) -> None:
        diffs.append((robot_id, added, removed))

    async_dispatcher_connect(hass, SIGNAL_ROBOT_JOINTS_CHANGED, _record)
    return diffs


def _joint_entity_ids(hass: HomeAssistant, robot_id: str = ROBOT_ID) -> list[str | None]:
    registry = er.async_get(hass)
    return [
        registry.async_get_entity_id("sensor", DOMAIN, f"{robot_id}_joint_{index}")
        for index in range(JOINT_COUNT)
    ]


def _joint_states(hass: HomeAssistant) -> list[str]:
    states: list[str] = []
    for entity_id in _joint_entity_ids(hass):
        assert entity_id is not None
        state = hass.states.get(entity_id)
        assert state is not None, entity_id
        states.append(state.state)
    return states


async def _fire_joints(hass: HomeAssistant, **overrides: Any) -> None:
    fire_robot_message(hass, "joints", joints_payload(**overrides))
    await hass.async_block_till_done()


# --------------------------------------------------------------------------- creation


async def test_announce_with_joints_creates_seven_sensors(
    hass: HomeAssistant,
    hub: BridgeHub,
    entity_registry: er.EntityRegistry,
    snapshot: SnapshotAssertion,
) -> None:
    assert (joints_filter(PREFIX), 0) in hub.subscriptions
    info = hub.robots[ROBOT_ID].joint_info
    assert info is not None and info.count == JOINT_COUNT
    entity_ids = _joint_entity_ids(hass)
    assert all(entity_ids)
    assert entity_ids == snapshot(name="entity_ids")
    for index, entity_id in enumerate(entity_ids):
        assert entity_id is not None
        entry = entity_registry.async_get(entity_id)
        assert entry is not None
        assert entry.unique_id == f"{ROBOT_ID}_joint_{index}"
        assert entry.disabled_by is None, "the joint sensors are enabled by default"
        assert entry == snapshot(name=f"{entity_id}-entry")
        state = hass.states.get(entity_id)
        assert state is not None
        assert state == snapshot(name=f"{entity_id}-state")
        assert state.state == "unknown", "no joints payload has arrived yet"
        assert state.attributes["friendly_name"] == f"Left arm Joint {index + 1} j{index + 1}"
        assert state.attributes["unit_of_measurement"] == "°"
        assert state.attributes["index"] == index
        assert state.attributes["joint_name"] == f"j{index + 1}"
        assert "device_class" not in state.attributes
        assert "state_class" not in state.attributes
    first = hass.states.get(entity_ids[0])
    fourth = hass.states.get(entity_ids[3])
    assert first is not None and fourth is not None
    assert (first.attributes["limit_min"], first.attributes["limit_max"]) == (-170.0, 170.0)
    assert (fourth.attributes["limit_min"], fourth.attributes["limit_max"]) == (-120.0, 120.0)
    # The other entities of the robot are untouched by the block.
    device = dr.async_get(hass).async_get_device_by_identifier(
        (DOMAIN, ROBOT_ID), hub.entry.entry_id
    )
    assert device is not None
    entries = er.async_entries_for_device(entity_registry, device.id)
    assert len(entries) == 4 + 5 + JOINT_COUNT


async def test_joint_sensor_without_limits_in_radians(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    complete_hub_start: Callable[[BridgeHub], Awaitable[None]],
) -> None:
    entry = make_config_entry()
    assert await hass.config_entries.async_setup(entry.entry_id)
    await hass.async_block_till_done()
    await complete_hub_start(entry.runtime_data)
    block = {**joints_block(), "unit": "rad", "limits": None, "names": ["base", "elbow"]}
    fire_robot_message(hass, "announce", announce_with_joints_payload(joints=block))
    fire_robot_message(hass, "availability", "online")
    await hass.async_block_till_done()
    entity_ids = _joint_entity_ids(hass)
    assert [bool(entity_id) for entity_id in entity_ids] == [True, True] + [False] * 5
    assert entity_ids[1] is not None
    state = hass.states.get(entity_ids[1])
    assert state is not None
    assert state.attributes["friendly_name"] == "Left arm Joint 2 elbow"
    assert state.attributes["unit_of_measurement"] == "rad"
    assert state.attributes["limit_min"] is None
    assert state.attributes["limit_max"] is None
    await _fire_joints(hass, positions=[0.5, -1.25])
    assert _joint_states_present(hass, 2) == ["0.5", "-1.25"]


def _joint_states_present(hass: HomeAssistant, count: int) -> list[str]:
    states: list[str] = []
    for entity_id in _joint_entity_ids(hass)[:count]:
        assert entity_id is not None
        state = hass.states.get(entity_id)
        assert state is not None
        states.append(state.state)
    return states


async def test_sensor_platform_picks_up_joints_announced_before_it_loaded(
    hass: HomeAssistant, hub: BridgeHub
) -> None:
    """A platform set up after the announce adds the enum and joint sensors in one batch."""
    added: list[list[str]] = []

    def _add_entities(entities: Any, update_before_add: bool = False, **_: Any) -> None:
        added.append([entity.unique_id for entity in entities])

    await sensor.async_setup_entry(hass, hub.entry, _add_entities)
    assert len(added) == 1
    assert added[0] == [
        f"{ROBOT_ID}_state",
        f"{ROBOT_ID}_last_result",
        *(f"{ROBOT_ID}_joint_{index}" for index in range(JOINT_COUNT)),
    ]


# --------------------------------------------------------------------------- the joints topic


async def test_joints_payload_updates_the_sensors(hass: HomeAssistant, hub: BridgeHub) -> None:
    await _fire_joints(hass)
    assert _joint_states(hass) == [str(value) for value in POSITIONS]
    robot = hub.robots[ROBOT_ID]
    assert robot.joints is not None
    assert robot.joints.seq == 7
    assert robot.joints.velocities is None
    assert robot.instance == "robotpc:4123"

    moved = [value + 1.5 for value in POSITIONS]
    await _fire_joints(hass, positions=moved, velocities=[0.1] * JOINT_COUNT, seq=8)
    assert _joint_states(hass) == [str(value) for value in moved]
    assert robot.joints is not None and robot.joints.seq == 8
    assert robot.joints.velocities == (0.1,) * JOINT_COUNT
    assert hub.counters["seq_regressions"] == 0
    assert hub.counters["mismatched_payloads"] == 0
    assert hub.counters["invalid_payloads"] == 0


async def test_joints_seq_regression_is_dropped(hass: HomeAssistant, hub: BridgeHub) -> None:
    await _fire_joints(hass, seq=7)
    await _fire_joints(hass, positions=[0.0] * JOINT_COUNT, seq=6)
    assert _joint_states(hass) == [str(value) for value in POSITIONS]
    assert hub.counters["seq_regressions"] == 1

    await _fire_joints(hass, positions=[1.0] * JOINT_COUNT, seq=8)
    assert _joint_states(hass) == ["1.0"] * JOINT_COUNT

    # A new session restarts the sequence, so a lower seq is accepted again.
    await _fire_joints(hass, positions=[2.0] * JOINT_COUNT, seq=0, session=OTHER_SESSION)
    assert _joint_states(hass) == ["2.0"] * JOINT_COUNT
    assert hub.counters["seq_regressions"] == 1
    assert hub.robots[ROBOT_ID].session == OTHER_SESSION
    assert hub.robots[ROBOT_ID].session_change_count == 1


async def test_joints_that_do_not_fit_are_dropped(
    hass: HomeAssistant, hub: BridgeHub, caplog: pytest.LogCaptureFixture
) -> None:
    caplog.set_level(logging.DEBUG, logger=f"custom_components.{DOMAIN}")
    await _fire_joints(hass)

    # Six positions for seven announced names.
    await _fire_joints(hass, positions=POSITIONS[:-1], seq=8)
    assert _joint_states(hass) == [str(value) for value in POSITIONS]
    assert hub.counters["mismatched_payloads"] == 1
    assert "7 announced names" in caplog.text

    # Seven positions but three velocities: the contract wants one per position.
    await _fire_joints(hass, positions=[1.0] * JOINT_COUNT, velocities=[0.1] * 3, seq=9)
    assert _joint_states(hass) == [str(value) for value in POSITIONS]
    assert hub.counters["mismatched_payloads"] == 2
    assert "3 velocities for 7 positions" in caplog.text

    # A payload whose robot_id differs from the topic.
    await _fire_joints(hass, robot_id="arm_right", seq=9)
    assert hub.counters["mismatched_payloads"] == 3

    # Not JSON, and the reserved segment, and a robot nobody announced.
    fire_robot_message(hass, "joints", b"{not json")
    fire_robot_message(hass, "joints", joints_payload(), robot_id="ha")
    fire_robot_message(hass, "joints", joints_payload(robot_id="arm_right"), robot_id="arm_right")
    await hass.async_block_till_done()
    assert hub.counters["invalid_payloads"] == 1
    assert "arm_right" not in hub.robots
    assert "no joints block announced" in caplog.text
    assert hub.counters["mismatched_payloads"] == 3
    assert _joint_states(hass) == [str(value) for value in POSITIONS]
    assert hub.counters["handler_errors"] == 0


async def test_joints_for_a_robot_without_the_block_are_ignored(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    complete_hub_start: Callable[[BridgeHub], Awaitable[None]],
) -> None:
    entry = make_config_entry()
    assert await hass.config_entries.async_setup(entry.entry_id)
    await hass.async_block_till_done()
    hub: BridgeHub = entry.runtime_data
    await complete_hub_start(hub)
    fire_robot_message(hass, "announce", announce_payload())
    fire_robot_message(hass, "availability", "online")
    await hass.async_block_till_done()
    await _fire_joints(hass)
    assert hub.robots[ROBOT_ID].joints is None
    assert hub.robots[ROBOT_ID].joint_info is None
    assert _joint_entity_ids(hass) == [None] * JOINT_COUNT
    assert hub.counters["mismatched_payloads"] == 0
    assert hub.counters["invalid_payloads"] == 0


# --------------------------------------------------------------------------- announce changes


async def test_announce_without_joints_removes_the_sensors(
    hass: HomeAssistant,
    hub: BridgeHub,
    entity_registry: er.EntityRegistry,
    joint_diffs: list[tuple[str, tuple[int, ...], tuple[int, ...]]],
) -> None:
    await _fire_joints(hass)
    entity_ids = _joint_entity_ids(hass)
    assert all(entity_ids)

    fire_robot_message(hass, "announce", announce_payload())
    await hass.async_block_till_done()
    await hass.async_block_till_done()
    assert joint_diffs == [(ROBOT_ID, (), tuple(range(JOINT_COUNT)))]
    assert hub.robots[ROBOT_ID].joints is None
    assert _joint_entity_ids(hass) == [None] * JOINT_COUNT
    for entity_id in entity_ids:
        assert entity_id is not None
        assert hass.states.get(entity_id) is None, entity_id
    assert hub.counters["handler_errors"] == 0
    # A later payload has nothing to land on and is ignored without a count.
    await _fire_joints(hass, seq=8)
    assert hub.robots[ROBOT_ID].joints is None
    assert hub.counters["mismatched_payloads"] == 0

    # The block may come back; the sensors are made again and wait for the next payload.
    fire_robot_message(hass, "announce", announce_with_joints_payload())
    await hass.async_block_till_done()
    assert joint_diffs[-1] == (ROBOT_ID, tuple(range(JOINT_COUNT)), ())
    assert all(_joint_entity_ids(hass))
    assert _joint_states(hass) == ["unknown"] * JOINT_COUNT
    await _fire_joints(hass, seq=9)
    assert _joint_states(hass) == [str(value) for value in POSITIONS]


async def test_announce_that_reconfigures_a_joint_replaces_its_sensor(
    hass: HomeAssistant,
    hub: BridgeHub,
    entity_registry: er.EntityRegistry,
    joint_diffs: list[tuple[str, tuple[int, ...], tuple[int, ...]]],
) -> None:
    await _fire_joints(hass)
    before = _joint_entity_ids(hass)
    assert all(before)

    # Index 1 is renamed and gets new limits; the other six are untouched.
    block = joints_block()
    block["names"][1] = "shoulder"
    block["limits"][1] = [-90, 90]
    fire_robot_message(hass, "announce", announce_with_joints_payload(joints=block))
    await hass.async_block_till_done()
    await hass.async_block_till_done()
    assert joint_diffs == [(ROBOT_ID, (1,), (1,))]
    after = _joint_entity_ids(hass)
    assert all(after)
    assert after[0] == before[0] and after[6] == before[6]
    assert after[1] is not None
    entry = entity_registry.async_get(after[1])
    assert entry is not None
    assert entry.original_name == "Joint 2 shoulder"
    state = hass.states.get(after[1])
    assert state is not None
    assert state.attributes["joint_name"] == "shoulder"
    assert (state.attributes["limit_min"], state.attributes["limit_max"]) == (-90.0, 90.0)
    assert _joint_states(hass) == ["unknown"] * JOINT_COUNT, "the old positions are discarded"
    await _fire_joints(hass, seq=8)
    assert _joint_states(hass) == [str(value) for value in POSITIONS]

    # A rate change alone is not a layout change: nothing is replaced or cleared.
    block["rate_hz"] = 5.0
    fire_robot_message(hass, "announce", announce_with_joints_payload(joints=block))
    await hass.async_block_till_done()
    assert len(joint_diffs) == 1
    assert _joint_states(hass) == [str(value) for value in POSITIONS]

    # Five joints: the last two sensors go.
    block["names"] = block["names"][:5]
    block["limits"] = block["limits"][:5]
    fire_robot_message(hass, "announce", announce_with_joints_payload(joints=block))
    await hass.async_block_till_done()
    await hass.async_block_till_done()
    assert joint_diffs[-1] == (ROBOT_ID, (), (5, 6))
    assert _joint_entity_ids(hass)[5:] == [None, None]
    assert all(_joint_entity_ids(hass)[:5])
    # Positions of the old length no longer fit.
    await _fire_joints(hass, seq=9)
    assert hub.counters["mismatched_payloads"] == 1
    await _fire_joints(hass, positions=POSITIONS[:5], seq=10)
    assert _joint_states_present(hass, 5) == [str(value) for value in POSITIONS[:5]]
    assert hub.counters["handler_errors"] == 0


async def test_announce_with_mismatched_limits_announces_without_joints(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    complete_hub_start: Callable[[BridgeHub], Awaitable[None]],
    caplog: pytest.LogCaptureFixture,
) -> None:
    """Six limits for seven names: the robot is announced, the block is ignored, a warning once."""
    entry = make_config_entry()
    assert await hass.config_entries.async_setup(entry.entry_id)
    await hass.async_block_till_done()
    hub: BridgeHub = entry.runtime_data
    await complete_hub_start(hub)
    block = joints_block()
    block["limits"] = block["limits"][:-1]
    fire_robot_message(hass, "announce", announce_with_joints_payload(joints=block))
    fire_robot_message(hass, "availability", "online")
    await hass.async_block_till_done()
    robot = hub.robots[ROBOT_ID]
    assert robot.announce is not None
    assert robot.joint_info is None
    assert robot.announce.joints_ignored is not None
    assert _joint_entity_ids(hass) == [None] * JOINT_COUNT
    assert hub.counters["invalid_payloads"] == 0
    assert caplog.text.count("6 entries for 7 names") == 1
    fire_robot_message(hass, "announce", announce_with_joints_payload(joints=block))
    await hass.async_block_till_done()
    assert caplog.text.count("6 entries for 7 names") == 1, "repeated announces do not nag"
    assert hub.diagnostics()["robots"][ROBOT_ID]["announce"]["joints_ignored"] == (
        "joints.limits has 6 entries for 7 names; the block is ignored"
    )


async def test_joints_dropped_while_ha_was_stopped_leave_no_registry_entries(
    hass: HomeAssistant,
    hub: BridgeHub,
    entity_registry: er.EntityRegistry,
    complete_hub_start: Callable[[BridgeHub], Awaitable[None]],
) -> None:
    """Seven joints announced, HA stopped, the node restarted without joints, HA started.

    The registry remembers the seven entries across the stop; the retained
    announce that replays after the start has no block, and nothing in the
    new run ever made those sensors, so the sweep must find them by unique id.
    """
    await _fire_joints(hass)
    entry = hub.entry
    before = _joint_entity_ids(hass)
    assert all(before)

    assert await hass.config_entries.async_unload(entry.entry_id)
    await hass.async_block_till_done()
    assert _joint_entity_ids(hass) == before, "an unload keeps the registry entries"

    assert await hass.config_entries.async_setup(entry.entry_id)
    await hass.async_block_till_done()
    restarted: BridgeHub = entry.runtime_data
    await complete_hub_start(restarted)
    fire_robot_message(hass, "announce", announce_payload(), retain=True)
    fire_robot_message(hass, "availability", "online", retain=True)
    await hass.async_block_till_done()
    await hass.async_block_till_done()
    assert _joint_entity_ids(hass) == [None] * JOINT_COUNT
    for entity_id in before:
        assert entity_id is not None
        assert hass.states.get(entity_id) is None, entity_id
    state_id = entity_registry.async_get_entity_id("sensor", DOMAIN, f"{ROBOT_ID}_state")
    assert state_id is not None and hass.states.get(state_id) is not None, "the robot is back"
    assert restarted.counters["handler_errors"] == 0

    # Fewer joints after a restart: only the entries beyond the new count go.
    assert await hass.config_entries.async_unload(entry.entry_id)
    await hass.async_block_till_done()
    assert await hass.config_entries.async_setup(entry.entry_id)
    await hass.async_block_till_done()
    await complete_hub_start(entry.runtime_data)
    fire_robot_message(hass, "announce", announce_with_joints_payload(), retain=True)
    await hass.async_block_till_done()
    assert all(_joint_entity_ids(hass))
    assert await hass.config_entries.async_unload(entry.entry_id)
    await hass.async_block_till_done()
    assert await hass.config_entries.async_setup(entry.entry_id)
    await hass.async_block_till_done()
    await complete_hub_start(entry.runtime_data)
    block = joints_block()
    block["names"] = block["names"][:5]
    block["limits"] = block["limits"][:5]
    fire_robot_message(hass, "announce", announce_with_joints_payload(joints=block), retain=True)
    await hass.async_block_till_done()
    await hass.async_block_till_done()
    assert [bool(entity_id) for entity_id in _joint_entity_ids(hass)] == [True] * 5 + [False] * 2


async def test_tombstone_removes_the_joint_sensors(
    hass: HomeAssistant, hub: BridgeHub, entity_registry: er.EntityRegistry
) -> None:
    await _fire_joints(hass)
    entity_ids = _joint_entity_ids(hass)
    fire_robot_message(hass, "announce", b"")
    await hass.async_block_till_done()
    await hass.async_block_till_done()
    assert ROBOT_ID not in hub.robots
    assert _joint_entity_ids(hass) == [None] * JOINT_COUNT
    for entity_id in entity_ids:
        assert entity_id is not None
        assert hass.states.get(entity_id) is None
    assert hub.counters["handler_errors"] == 0


# --------------------------------------------------------------------------- availability


async def test_joint_sensors_are_unavailable_with_the_robot(
    hass: HomeAssistant, hub: BridgeHub
) -> None:
    await _fire_joints(hass)
    assert _joint_states(hass) == [str(value) for value in POSITIONS]

    fire_robot_message(hass, "availability", "offline")
    await hass.async_block_till_done()
    assert _joint_states(hass) == ["unavailable"] * JOINT_COUNT
    # A payload while offline is still kept, so the value is right when the robot returns.
    await _fire_joints(hass, positions=[3.0] * JOINT_COUNT, seq=8)
    assert _joint_states(hass) == ["unavailable"] * JOINT_COUNT

    fire_robot_message(hass, "availability", "online")
    await hass.async_block_till_done()
    assert _joint_states(hass) == ["3.0"] * JOINT_COUNT, "the last value is kept, not reset"

    async_dispatcher_send(hass, MQTT_CONNECTION_STATE, False)
    await hass.async_block_till_done()
    assert _joint_states(hass) == ["unavailable"] * JOINT_COUNT
    async_dispatcher_send(hass, MQTT_CONNECTION_STATE, True)
    await hass.async_block_till_done()
    assert _joint_states(hass) == ["3.0"] * JOINT_COUNT


async def test_joint_sensor_entity_hooks(hass: HomeAssistant, hub: BridgeHub) -> None:
    info = hub.robots[ROBOT_ID].joint_info
    assert info is not None
    entity = JointSensor(hub, ROBOT_ID, 3, info)
    entity.hass = hass
    assert entity.unique_id == f"{ROBOT_ID}_joint_3"
    assert entity.key == ("j4", "deg", (-120.0, 120.0))
    assert entity.translation_placeholders == {"index": "4", "joint_name": "j4"}
    assert entity.native_unit_of_measurement == "°"
    assert entity.suggested_display_precision == 1
    assert entity.device_info == hub.device_info(ROBOT_ID)
    entity.async_entity_id_changed("sensor.old_name")
    # A position for an index the latest payload does not carry reads unknown, never raises.
    entity._handle_robot_update()
    assert entity.native_value is None
    await _fire_joints(hass)
    entity._handle_robot_update()
    assert entity.native_value == -45.5


# --------------------------------------------------------------------------- diagnostics


async def test_diagnostics_show_the_positions_and_seq(hass: HomeAssistant, hub: BridgeHub) -> None:
    robot = hub.diagnostics()["robots"][ROBOT_ID]
    assert robot["joints"] is None
    assert robot["announce"]["joints"] == joints_block()
    assert robot["announce"]["joints_ignored"] is None

    await _fire_joints(hass)
    robot = hub.diagnostics()["robots"][ROBOT_ID]
    assert robot["joints"] == {
        "positions": POSITIONS,
        "velocities": None,
        "session": joints_payload()["session"],
        "instance": "robotpc:4123",
        "seq": 7,
        "ts": joints_payload()["ts"],
    }
    device = dr.async_get(hass).async_get_device_by_identifier(
        (DOMAIN, ROBOT_ID), hub.entry.entry_id
    )
    assert device is not None
    data = await async_get_device_diagnostics(hass, hub.entry, device)
    assert data["robot"]["joints"]["seq"] == 7
    assert data["robot"]["joints"]["positions"] == POSITIONS
    assert data["robot"]["announce"]["joints"]["names"] == joints_block()["names"]
