# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The canonical sensor republish in ``sensors_out.py``."""

from __future__ import annotations

import json
from datetime import datetime
from typing import Any
from unittest.mock import ANY

import pytest
from homeassistant.core import HomeAssistant, State
from homeassistant.exceptions import HomeAssistantError

from custom_components.ros2_bridge.const import CONF_MIRROR_ENTITIES, CONF_TOPIC_PREFIX
from custom_components.ros2_bridge.sensors_out import (
    SENSOR_ATTRIBUTE_ALLOWLIST,
    SensorMirror,
    async_clear_published_topics,
    async_setup_sensor_mirror,
    sensor_payload,
    sensor_topic,
)

from .common import PREFIX, validate
from .test_services import HubRig, default_rig_options, hub_rig  # noqa: F401

MOTION = "binary_sensor.kitchen_motion"
TEMPERATURE = "sensor.kitchen_temperature"
LIGHT = "light.kitchen_ceiling"
MOTION_TOPIC = f"{PREFIX}/ha/sensors/binary_sensor/kitchen_motion"
TEMPERATURE_TOPIC = f"{PREFIX}/ha/sensors/sensor/kitchen_temperature"
MIRROR_TWO = {CONF_MIRROR_ENTITIES: [MOTION, TEMPERATURE]}


def payload_of(rig: HubRig, topic: str, index: int = -1) -> dict[str, Any]:
    raw = rig.publishes(topic)[index].args[1]
    assert isinstance(raw, bytes)
    return json.loads(raw)


async def start_mirror(rig: HubRig) -> SensorMirror:
    mirror = async_setup_sensor_mirror(rig.hub)
    await rig.hass.async_block_till_done()
    assert mirror.started
    return mirror


# --------------------------------------------------------------------------- pure pieces


def test_sensor_payload_shape_and_allowlist() -> None:
    changed = datetime.fromisoformat("2025-10-13T12:00:00.123456+00:00")
    state = State(
        MOTION,
        "on",
        {
            "friendly_name": "Kitchen motion",
            "device_class": "motion",
            "secret_token": "never",
            "latitude": 51.5,
            "options": ("a", "b"),
            "last_seen": changed,
        },
        last_changed=changed,
    )
    payload = sensor_payload(MOTION, state, 1760356800123)
    assert validate("sensor", payload) == []
    assert payload == {
        "schema": 1,
        "entity_id": MOTION,
        "domain": "binary_sensor",
        "object_id": "kitchen_motion",
        "state": "on",
        "available": True,
        "device_class": "motion",
        "unit": None,
        "attributes": {
            "friendly_name": "Kitchen motion",
            "device_class": "motion",
            "options": ["a", "b"],
            "last_seen": "2025-10-13 12:00:00.123456+00:00",
        },
        "last_changed": "2025-10-13T12:00:00.123456+00:00",
        "ts": 1760356800123,
    }
    assert "secret_token" not in SENSOR_ATTRIBUTE_ALLOWLIST
    assert "latitude" not in SENSOR_ATTRIBUTE_ALLOWLIST


@pytest.mark.parametrize("value", ["unavailable", "unknown"])
def test_unavailable_and_unknown_map_to_null_state(value: str) -> None:
    state = State(TEMPERATURE, value, {"unit_of_measurement": "°C", "device_class": "temperature"})
    payload = sensor_payload(TEMPERATURE, state, 1)
    assert validate("sensor", payload) == []
    assert payload["state"] is None and payload["available"] is False
    assert payload["unit"] == "°C" and payload["device_class"] == "temperature"


def test_missing_entity_is_unavailable() -> None:
    payload = sensor_payload(LIGHT, None, 5)
    assert validate("sensor", payload) == []
    assert payload["state"] is None and payload["available"] is False
    assert payload["attributes"] == {} and payload["last_changed"] is None


def test_sensor_topic_validates_segments() -> None:
    assert sensor_topic(PREFIX, MOTION) == MOTION_TOPIC
    assert sensor_topic("lab/ha2", LIGHT) == "lab/ha2/ha/sensors/light/kitchen_ceiling"
    assert sensor_topic(PREFIX, "sensor.Bad-Name") is None
    assert sensor_topic(PREFIX, "nodot") is None


# --------------------------------------------------------------------------- publishing


@pytest.mark.parametrize("rig_options", [MIRROR_TWO])
async def test_publishes_once_at_setup_then_on_change(rig: HubRig) -> None:
    hass = rig.hass
    hass.states.async_set(
        MOTION, "off", {"friendly_name": "Kitchen motion", "device_class": "motion"}
    )
    await start_mirror(rig)
    rig.mqtt.async_publish.assert_any_call(
        MOTION_TOPIC, ANY, 0, True, message_expiry_interval=None
    )
    rig.mqtt.async_publish.assert_any_call(
        TEMPERATURE_TOPIC, ANY, 0, True, message_expiry_interval=None
    )
    assert rig.mqtt.async_publish.call_count == 2
    motion = payload_of(rig, MOTION_TOPIC)
    assert validate("sensor", motion) == []
    assert motion["state"] == "off" and motion["available"] is True
    assert motion["attributes"] == {"friendly_name": "Kitchen motion", "device_class": "motion"}
    assert motion["ts"] == rig.hub.now
    temperature = payload_of(rig, TEMPERATURE_TOPIC)
    assert temperature["state"] is None and temperature["available"] is False  # no such entity yet
    assert sorted(rig.store.published_sensor_topics) == [MOTION_TOPIC, TEMPERATURE_TOPIC]

    hass.states.async_set(
        MOTION, "on", {"friendly_name": "Kitchen motion", "device_class": "motion"}
    )
    await hass.async_block_till_done()
    assert len(rig.publishes(MOTION_TOPIC)) == 2
    assert payload_of(rig, MOTION_TOPIC)["state"] == "on"

    hass.states.async_set(TEMPERATURE, "21.5", {"unit_of_measurement": "°C"})
    await hass.async_block_till_done()
    temperature = payload_of(rig, TEMPERATURE_TOPIC)
    assert temperature["state"] == "21.5" and temperature["unit"] == "°C"
    assert temperature["available"] is True

    hass.states.async_set(TEMPERATURE, "unavailable", {"unit_of_measurement": "°C"})
    await hass.async_block_till_done()
    temperature = payload_of(rig, TEMPERATURE_TOPIC)
    assert temperature["state"] is None and temperature["available"] is False

    hass.states.async_remove(TEMPERATURE)
    await hass.async_block_till_done()
    assert payload_of(rig, TEMPERATURE_TOPIC)["available"] is False
    assert rig.hub.counters["publish_errors"] == 0


@pytest.mark.parametrize("rig_options", [{CONF_MIRROR_ENTITIES: [MOTION]}])
async def test_removed_entity_topic_is_cleared_at_start(rig: HubRig) -> None:
    rig.store.published_sensor_topics.extend([TEMPERATURE_TOPIC, MOTION_TOPIC])
    await start_mirror(rig)
    rig.mqtt.async_publish.assert_any_call(
        TEMPERATURE_TOPIC, b"", 0, True, message_expiry_interval=None
    )
    assert rig.store.published_sensor_topics == [MOTION_TOPIC]
    assert (
        len(rig.publishes(MOTION_TOPIC)) == 1
        and payload_of(rig, MOTION_TOPIC)["entity_id"] == MOTION
    )


@pytest.mark.parametrize(
    "rig_options", [{CONF_MIRROR_ENTITIES: [MOTION], CONF_TOPIC_PREFIX: "lab/ros"}]
)
async def test_prefix_change_clears_the_old_prefix_topics(rig: HubRig) -> None:
    rig.store.published_sensor_topics.append(MOTION_TOPIC)  # published under the default prefix
    await start_mirror(rig)
    new_topic = "lab/ros/ha/sensors/binary_sensor/kitchen_motion"
    rig.mqtt.async_publish.assert_any_call(
        MOTION_TOPIC, b"", 0, True, message_expiry_interval=None
    )
    rig.mqtt.async_publish.assert_any_call(new_topic, ANY, 0, True, message_expiry_interval=None)
    assert rig.store.published_sensor_topics == [new_topic]


@pytest.mark.parametrize("rig_options", [{CONF_MIRROR_ENTITIES: [MOTION]}])
async def test_waits_for_the_connection_and_counts_failures(rig: HubRig) -> None:
    hass = rig.hass
    rig.hub._on_connection_status(False)
    await start_mirror(rig)
    rig.mqtt.async_publish.assert_not_called()
    hass.states.async_set(MOTION, "on")
    await hass.async_block_till_done()
    rig.mqtt.async_publish.assert_not_called()

    rig.hub._on_connection_status(True)
    await hass.async_block_till_done()
    assert len(rig.publishes(MOTION_TOPIC)) == 1
    assert payload_of(rig, MOTION_TOPIC)["state"] == "on"

    rig.mqtt.async_publish.side_effect = HomeAssistantError("no broker")
    hass.states.async_set(MOTION, "off")
    await hass.async_block_till_done()
    assert rig.hub.counters["publish_errors"] == 1
    rig.mqtt.async_publish.side_effect = None
    rig.hub._on_connection_status(False)
    rig.hub._on_connection_status(True)
    await hass.async_block_till_done()
    assert payload_of(rig, MOTION_TOPIC)["state"] == "off"


@pytest.mark.parametrize("rig_options", [{CONF_MIRROR_ENTITIES: [MOTION, "sensor.Bad-Name"]}])
async def test_invalid_entity_ids_are_skipped(rig: HubRig) -> None:
    mirror = await start_mirror(rig)
    assert list(mirror.topics) == [MOTION]
    assert rig.mqtt.async_publish.call_count == 1


@pytest.mark.parametrize("rig_options", [MIRROR_TWO])
async def test_stop_and_clear_all(rig: HubRig) -> None:
    mirror = await start_mirror(rig)
    await mirror.async_stop()
    rig.mqtt.async_publish.reset_mock()
    rig.hass.states.async_set(MOTION, "on")
    await rig.hass.async_block_till_done()
    rig.mqtt.async_publish.assert_not_called()  # no longer tracking
    assert sorted(rig.store.published_sensor_topics) == [MOTION_TOPIC, TEMPERATURE_TOPIC]
    await async_clear_published_topics(rig.hass, rig.store)
    assert rig.store.published_sensor_topics == []
    assert sorted(c.args[0] for c in rig.mqtt.async_publish.call_args_list) == [
        MOTION_TOPIC,
        TEMPERATURE_TOPIC,
    ]
    for c in rig.mqtt.async_publish.call_args_list:
        assert c.args[1] == b"" and c.args[2] == 0 and c.args[3] is True


async def test_mirror_without_entities_is_quiet(rig: HubRig, hass: HomeAssistant) -> None:
    mirror = await start_mirror(rig)
    assert mirror.topics == {}
    hass.states.async_set(MOTION, "on")
    await hass.async_block_till_done()
    rig.mqtt.async_publish.assert_not_called()
    await mirror.async_stop()
