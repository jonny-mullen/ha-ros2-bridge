# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The ``ros2_bridge.commands`` Store: round trip across two simulated restarts."""

from __future__ import annotations

from typing import Any

from homeassistant.core import HomeAssistant

from custom_components.ros2_bridge.commands import CommandRegistry
from custom_components.ros2_bridge.const import STORAGE_KEY, CommandState, CommandType, TaskKind
from custom_components.ros2_bridge.store import CommandStore, normalise

from .common import ROBOT_ID, SESSION, TASK_ID, FakeClock, make_ack

TOPIC = "ros2_bridge/ha/sensors/binary_sensor/kitchen_motion"


def _registry_with_command(clock: FakeClock) -> CommandRegistry:
    registry = CommandRegistry(clock=clock)
    registry.create(
        task_id=TASK_ID,
        robot_id=ROBOT_ID,
        type=CommandType.TASK,
        task="move_to",
        kind=TaskKind.MOTION,
        params={"pose": "home"},
        target_id=None,
        session=SESSION,
        source={"origin": "ha", "kind": "service"},
        issued_at=clock(),
        max_age_ms=5000,
        timeout_s=60,
    )
    registry.mark_sent(TASK_ID)
    registry.handle_ack(make_ack(), retained=False)
    return registry


async def test_round_trip_across_two_restarts(
    hass: HomeAssistant, hass_storage: dict[str, Any], clock: FakeClock
) -> None:
    store = CommandStore(hass)
    data = await store.async_load()
    assert data == {"outstanding": {}, "seen_results": [], "published_sensor_topics": []}
    registry = _registry_with_command(clock)
    store.bind(registry)
    store.published_sensor_topics.append(TOPIC)
    await store.async_save()
    assert hass_storage[STORAGE_KEY]["version"] == 1
    saved = hass_storage[STORAGE_KEY]["data"]
    assert saved["outstanding"][TASK_ID]["state"] == "running"
    assert saved["published_sensor_topics"] == [TOPIC]

    # First restart: a fresh store and registry rebuild the command with its session and deadline.
    first = CommandStore(hass)
    loaded = await first.async_load()
    assert first.published_sensor_topics == [TOPIC]
    registry_1 = CommandRegistry(clock=clock)
    registry_1.load(loaded)
    command = registry_1.get(TASK_ID)
    assert command is not None
    assert command.state is CommandState.RUNNING
    assert command.session == SESSION
    assert command.deadline == registry.get(TASK_ID).deadline  # type: ignore[union-attr]
    first.bind(registry_1)
    await first.async_save()

    # Second restart: still there, byte for byte the same registry share.
    second = CommandStore(hass)
    loaded_2 = await second.async_load()
    registry_2 = CommandRegistry(clock=clock)
    registry_2.load(loaded_2)
    assert registry_2.export() == registry_1.export() == registry.export()
    assert second.published_sensor_topics == [TOPIC]


async def test_request_save_coalesces_and_flushes(
    hass: HomeAssistant, hass_storage: dict[str, Any], clock: FakeClock
) -> None:
    store = CommandStore(hass)
    await store.async_load()
    registry = _registry_with_command(clock)
    store.bind(registry)
    store.async_request_save(True)
    store.async_request_save(True)
    store.published_sensor_topics.append(TOPIC)
    await hass.async_block_till_done()
    assert hass_storage[STORAGE_KEY]["data"]["published_sensor_topics"] == [TOPIC]
    store.published_sensor_topics.clear()
    store.async_request_save(False)
    await store.async_flush()
    assert hass_storage[STORAGE_KEY]["data"]["published_sensor_topics"] == []


async def test_remove_deletes_the_document(
    hass: HomeAssistant, hass_storage: dict[str, Any], clock: FakeClock
) -> None:
    store = CommandStore(hass)
    await store.async_load()
    store.bind(_registry_with_command(clock))
    store.published_sensor_topics.append(TOPIC)
    await store.async_save()
    assert STORAGE_KEY in hass_storage
    await store.async_remove()
    assert STORAGE_KEY not in hass_storage
    assert store.published_sensor_topics == []


def test_normalise_drops_junk() -> None:
    assert normalise(None) == {
        "outstanding": {},
        "seen_results": [],
        "published_sensor_topics": [],
    }
    data = normalise(
        {
            "outstanding": {TASK_ID: {"robot_id": ROBOT_ID}, "bad": "x"},
            "seen_results": [[TASK_ID, 5], ["x"], [1, 2], [TASK_ID, True]],
            "published_sensor_topics": [TOPIC, 3],
            "extra": 1,
        }
    )
    assert data == {
        "outstanding": {TASK_ID: {"robot_id": ROBOT_ID}},
        "seen_results": [[TASK_ID, 5]],
        "published_sensor_topics": [TOPIC],
    }
