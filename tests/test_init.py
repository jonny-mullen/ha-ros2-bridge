# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Config entry lifecycle: setup, readiness, heartbeat, unload, removal and the Store across restarts.

The services and intent modules are stood in through ``sys.modules`` so this
file tests only the wiring in ``__init__.py``; their own suites cover them.
Messages after the first on a topic are fired without the retain flag, as a
broker delivers them live: Home Assistant's client hands a subscription the
retained copy of a topic once only.
"""

from __future__ import annotations

import json
import sys
import types
from collections.abc import Awaitable, Callable
from typing import Any
from unittest.mock import ANY, patch

import pytest
from homeassistant.components import mqtt
from homeassistant.components.mqtt import MQTT_CONNECTION_STATE
from homeassistant.config_entries import ConfigEntry, ConfigEntryState
from homeassistant.core import HomeAssistant
from homeassistant.exceptions import HomeAssistantError
from homeassistant.helpers import device_registry as dr
from homeassistant.helpers import intent
from homeassistant.helpers import issue_registry as ir
from homeassistant.helpers.dispatcher import async_dispatcher_send
from pytest_homeassistant_custom_component.common import (
    MockConfigEntry,
    async_capture_events,
)

from custom_components.ros2_bridge import DATA_REGISTRIES, async_remove_config_entry_device
from custom_components.ros2_bridge.const import (
    CONF_MIRROR_ENTITIES,
    CONF_TOPIC_PREFIX,
    DATA_HUB,
    DOMAIN,
    EVENT_TASK_LATE_RESULT,
    EVENT_TASK_RESULT,
    HA_HEARTBEAT_EXPIRY_S,
    STORAGE_KEY,
    STORE_DELAY_SAVE_S,
    CommandState,
    ack_filter,
    announce_filter,
    announce_topic,
    availability_filter,
    availability_topic,
    feedback_filter,
    ha_heartbeat_topic,
    heartbeat_filter,
    joints_filter,
    result_filter,
    state_filter,
    state_topic,
)
from custom_components.ros2_bridge.hub import BridgeHub
from custom_components.ros2_bridge.protocol import CommandSource

from .common import (
    INSTANCE_ID,
    PREFIX,
    ROBOT_ID,
    ack_payload,
    announce_payload,
    fire_robot_message,
    result_payload,
)

SENSOR_TOPICS = [
    f"{PREFIX}/ha/sensors/binary_sensor/kitchen_motion",
    f"{PREFIX}/ha/sensors/sensor/kitchen_temperature",
]
MIRRORED = "binary_sensor.kitchen_motion"
MIRRORED_TOPIC = SENSOR_TOPICS[0]
MIRRORED_ATTRIBUTES = {"friendly_name": "Kitchen motion", "device_class": "motion"}


class _StubIntentHandler(intent.IntentHandler):
    """A real intent handler the stub registers, so unload can be checked against the registry."""

    intent_type = "Ros2BridgeSendTask"

    async def async_handle(self, intent_obj: intent.Intent) -> intent.IntentResponse:
        return intent_obj.create_response()


@pytest.fixture(autouse=True)
def stub_registrars(monkeypatch: pytest.MonkeyPatch) -> dict[str, Any]:
    """Stand in the services and intent modules; the intent stub registers a real handler."""
    record: dict[str, Any] = {"services": 0, "intents": 0, "handler": None}
    services = types.ModuleType(f"custom_components.{DOMAIN}.services")

    def register_services(hass: HomeAssistant) -> None:
        record["services"] += 1

    services.async_register_services = register_services  # type: ignore[attr-defined]
    intents = types.ModuleType(f"custom_components.{DOMAIN}.intent")

    def register_intents(hass: HomeAssistant) -> None:
        record["intents"] += 1
        handler = _StubIntentHandler()
        record["handler"] = handler
        intent.async_register(hass, handler)

    intents.async_register_intents = register_intents  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, services.__name__, services)
    monkeypatch.setitem(sys.modules, intents.__name__, intents)
    return record


async def _setup(hass: HomeAssistant, entry: MockConfigEntry) -> BridgeHub:
    assert await hass.config_entries.async_setup(entry.entry_id)
    await hass.async_block_till_done()
    assert entry.state is ConfigEntryState.LOADED
    hub: BridgeHub = entry.runtime_data
    return hub


def _announce_online(hass: HomeAssistant, **overrides: Any) -> None:
    fire_robot_message(hass, "announce", announce_payload(**overrides))
    fire_robot_message(hass, "availability", "online")


# --------------------------------------------------------------------------- setup


async def test_setup_entry_not_ready_without_mqtt(
    hass: HomeAssistant, make_config_entry: Callable[..., MockConfigEntry]
) -> None:
    """No MQTT config entry: the entry goes to retry instead of loading."""
    entry = make_config_entry()
    assert not await hass.config_entries.async_setup(entry.entry_id)
    await hass.async_block_till_done()
    assert entry.state is ConfigEntryState.SETUP_RETRY


async def test_setup_registers_services_intents_and_the_container(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    stub_registrars: dict[str, Any],
) -> None:
    entry = make_config_entry()
    hub = await _setup(hass, entry)
    assert stub_registrars["services"] == 1
    assert stub_registrars["intents"] == 1
    assert hass.data[DOMAIN][DATA_REGISTRIES] == {}
    assert hass.data[DOMAIN][DATA_HUB] is hub, "the services' primary lookup finds the hub"


async def test_setup_subscribes_eight_filters_and_becomes_ready(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    complete_hub_start: Callable[[BridgeHub], Awaitable[None]],
) -> None:
    entry = make_config_entry()
    hub = await _setup(hass, entry)
    expected = [
        (announce_filter(PREFIX), 1),
        (availability_filter(PREFIX), 1),
        (state_filter(PREFIX), 1),
        (heartbeat_filter(PREFIX), 1),
        (ack_filter(PREFIX), 1),
        (result_filter(PREFIX), 1),
        (feedback_filter(PREFIX), 0),
        (joints_filter(PREFIX), 0),
    ]
    assert hub.subscriptions == expected
    # The MQTT integration subscribes to its own discovery topics as well (including the
    # manifest's ``ros2_bridge/+/announce`` trigger at QoS 0); ours must all be among them.
    subscribed = {
        (call.args[0], call.args[2]) for call in mqtt_setup.async_subscribe.call_args_list
    }
    assert set(expected) <= subscribed
    assert hub.mqtt_connected is True
    assert hub.ready is False, "the subscribe debouncer has not run yet"
    assert hub.reconciled is False
    await complete_hub_start(hub)
    assert hub.ready is True
    assert hub.reconciled is True


async def test_not_ready_before_subscriptions_confirmed(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
) -> None:
    """A command sent before the ack and result subscriptions are confirmed is refused."""
    entry = make_config_entry()
    hub = await _setup(hass, entry)
    _announce_online(hass)
    await hass.async_block_till_done()
    assert hub.robot_available(ROBOT_ID)
    assert not hub.ready
    with pytest.raises(HomeAssistantError) as excinfo:
        await hub.async_send_task(ROBOT_ID, "reset", {}, source=CommandSource(kind="service"))
    assert type(excinfo.value) is HomeAssistantError
    assert excinfo.value.translation_key == "not_ready"
    assert excinfo.value.translation_domain == DOMAIN


# --------------------------------------------------------------------------- heartbeat


async def test_heartbeat_publish_arguments_and_suppression_while_disconnected(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    complete_hub_start: Callable[[BridgeHub], Awaitable[None]],
    advance: Callable[[float], Awaitable[None]],
) -> None:
    entry = make_config_entry()
    hub = await _setup(hass, entry)
    await complete_hub_start(hub)
    mqtt_setup.async_publish.reset_mock()
    await advance(2)
    mqtt_setup.async_publish.assert_called_once_with(
        ha_heartbeat_topic(PREFIX), ANY, 0, False, message_expiry_interval=HA_HEARTBEAT_EXPIRY_S
    )
    payload = json.loads(mqtt_setup.async_publish.call_args.args[1])
    assert payload["schema"] == 1
    assert payload["interval_ms"] == 2000
    assert payload["instance_id"] == INSTANCE_ID
    assert payload["seq"] == hub.heartbeat_seq
    assert payload["ts"] == hub.last_heartbeat_sent_at
    sent_before = hub.counters["heartbeats_sent"]

    async_dispatcher_send(hass, MQTT_CONNECTION_STATE, False)
    await hass.async_block_till_done()
    assert hub.mqtt_connected is False
    mqtt_setup.async_publish.reset_mock()
    await advance(2)
    await advance(2)
    mqtt_setup.async_publish.assert_not_called()
    assert hub.counters["heartbeats_sent"] == sent_before

    async_dispatcher_send(hass, MQTT_CONNECTION_STATE, True)
    await hass.async_block_till_done()
    await advance(2)
    mqtt_setup.async_publish.assert_called_once()
    assert hub.counters["heartbeats_sent"] == sent_before + 1


async def test_heartbeat_disabled_at_interval_zero(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    complete_hub_start: Callable[[BridgeHub], Awaitable[None]],
    advance: Callable[[float], Awaitable[None]],
) -> None:
    entry = make_config_entry(options={"heartbeat_interval_s": 0})
    hub = await _setup(hass, entry)
    await complete_hub_start(hub)
    mqtt_setup.async_publish.reset_mock()
    await advance(10)
    mqtt_setup.async_publish.assert_not_called()


# --------------------------------------------------------------------------- unload and reload


async def test_unload_keeps_intents_registered_and_parks_the_registry(
    hass: HomeAssistant,
    hass_storage: dict[str, Any],
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    complete_hub_start: Callable[[BridgeHub], Awaitable[None]],
    stub_registrars: dict[str, Any],
) -> None:
    entry = make_config_entry()
    hub = await _setup(hass, entry)
    await complete_hub_start(hub)
    _announce_online(hass)
    await hass.async_block_till_done()
    command = await hub.async_send_task(
        ROBOT_ID, "reset", {}, source=CommandSource(kind="service")
    )
    future = hub.registry.wait_result(command.task_id)
    handler = stub_registrars["handler"]
    assert hass.data[intent.DATA_KEY][_StubIntentHandler.intent_type] is handler

    assert await hass.config_entries.async_unload(entry.entry_id)
    await hass.async_block_till_done()
    assert entry.state is ConfigEntryState.NOT_LOADED
    assert hub.started is False
    assert DATA_HUB not in hass.data[DOMAIN]
    assert hass.data[intent.DATA_KEY][_StubIntentHandler.intent_type] is handler
    assert hass.data[DOMAIN][DATA_REGISTRIES][entry.entry_id] is hub.registry
    assert future.cancelled()
    parked = hub.registry.get(command.task_id)
    assert parked is not None and parked.state is CommandState.SENT
    assert command.task_id in hass_storage[STORAGE_KEY]["data"]["outstanding"]

    # Reload: the parked registry is adopted, with its deadlines suspended until reconciled.
    assert stub_registrars["intents"] == 1
    hub_2 = await _setup(hass, entry)
    assert hub_2 is not hub
    assert hub_2.registry is hub.registry
    assert hub_2.registry.deadlines_suspended is True
    assert stub_registrars["intents"] == 1, "async_setup runs once per start"
    assert hass.data[DOMAIN][DATA_REGISTRIES] == {}
    assert hass.data[DOMAIN][DATA_HUB] is hub_2


# --------------------------------------------------------------------------- setup failures


async def test_failed_platform_setup_stops_the_hub(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    advance: Callable[[float], Awaitable[None]],
) -> None:
    """A platform error after the hub started leaves no subscription, timer or heartbeat."""
    entry = make_config_entry()
    hubs: list[BridgeHub] = []

    async def _boom(forwarded: ConfigEntry, platforms: Any) -> None:
        hubs.append(forwarded.runtime_data)
        raise RuntimeError("platform setup failed")

    with patch.object(hass.config_entries, "async_forward_entry_setups", side_effect=_boom):
        assert not await hass.config_entries.async_setup(entry.entry_id)
        await hass.async_block_till_done()
    assert entry.state is ConfigEntryState.SETUP_ERROR
    (hub,) = hubs
    assert hub.started is False
    assert hub.subscriptions == []
    assert hub._timers == [] and hub._ready_timeout is None and hub._settle_handle is None
    assert DATA_HUB not in hass.data[DOMAIN]
    mqtt_setup.async_publish.reset_mock()
    await advance(2)
    await advance(2)
    mqtt_setup.async_publish.assert_not_called()


async def test_failed_hub_start_tears_down_the_partial_subscriptions(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
) -> None:
    """An exception inside ``hub.async_start`` undoes the subscriptions made before it."""
    entry = make_config_entry()
    real_subscribe = mqtt.async_subscribe
    calls = 0

    async def _subscribe(*args: Any, **kwargs: Any) -> Any:
        nonlocal calls
        calls += 1
        if calls == 3:
            raise RuntimeError("subscribe failed")
        return await real_subscribe(*args, **kwargs)

    with patch("custom_components.ros2_bridge.hub.mqtt.async_subscribe", side_effect=_subscribe):
        assert not await hass.config_entries.async_setup(entry.entry_id)
        await hass.async_block_till_done()
    assert calls == 3
    assert entry.state is ConfigEntryState.SETUP_ERROR
    hub: BridgeHub = entry.runtime_data
    assert hub.started is False
    assert hub.subscriptions == [] and hub._unsubscribers == []
    assert hub._timers == [] and hub._ready_timeout is None
    assert DATA_HUB not in hass.data[DOMAIN]


# --------------------------------------------------------------------------- sensor mirror


def _publishes(mqtt_setup: Any, topic: str) -> list[Any]:
    return [c for c in mqtt_setup.async_publish.call_args_list if c.args[0] == topic]


async def test_sensor_mirror_publishes_mirrored_entities(
    hass: HomeAssistant,
    hass_storage: dict[str, Any],
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    complete_hub_start: Callable[[BridgeHub], Awaitable[None]],
    advance: Callable[[float], Awaitable[None]],
) -> None:
    """The mirror is wired at setup: one retained publish per entity, recorded in the Store."""
    hass.states.async_set(MIRRORED, "on", MIRRORED_ATTRIBUTES)
    entry = make_config_entry(options={CONF_MIRROR_ENTITIES: [MIRRORED]})
    hub = await _setup(hass, entry)
    await complete_hub_start(hub)
    publishes = _publishes(mqtt_setup, MIRRORED_TOPIC)
    assert len(publishes) == 1
    mqtt_setup.async_publish.assert_any_call(
        MIRRORED_TOPIC, ANY, 0, True, message_expiry_interval=None
    )
    payload = json.loads(publishes[0].args[1])
    assert payload["schema"] == 1
    assert payload["entity_id"] == MIRRORED
    assert payload["state"] == "on" and payload["available"] is True
    assert payload["device_class"] == "motion"
    assert payload["attributes"] == MIRRORED_ATTRIBUTES
    assert hub.store.published_sensor_topics == [MIRRORED_TOPIC]
    await advance(STORE_DELAY_SAVE_S + 1)
    assert hass_storage[STORAGE_KEY]["data"]["published_sensor_topics"] == [MIRRORED_TOPIC]

    hass.states.async_set(MIRRORED, "off", MIRRORED_ATTRIBUTES)
    await hass.async_block_till_done()
    publishes = _publishes(mqtt_setup, MIRRORED_TOPIC)
    assert len(publishes) == 2
    assert json.loads(publishes[-1].args[1])["state"] == "off"


async def test_prefix_change_on_reload_clears_the_old_sensor_topic(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    complete_hub_start: Callable[[BridgeHub], Awaitable[None]],
) -> None:
    """An options reload with a new prefix clears the old topic and publishes the new one."""
    entry = make_config_entry(options={CONF_MIRROR_ENTITIES: [MIRRORED]})
    hub = await _setup(hass, entry)
    await complete_hub_start(hub)
    assert hub.store.published_sensor_topics == [MIRRORED_TOPIC]
    mqtt_setup.async_publish.reset_mock()

    hass.config_entries.async_update_entry(
        entry, options={**entry.options, CONF_TOPIC_PREFIX: "lab"}
    )
    assert await hass.config_entries.async_reload(entry.entry_id)
    await hass.async_block_till_done()
    assert entry.state is ConfigEntryState.LOADED
    new_hub: BridgeHub = entry.runtime_data
    assert new_hub is not hub and new_hub.prefix == "lab"
    new_topic = "lab/ha/sensors/binary_sensor/kitchen_motion"
    mqtt_setup.async_publish.assert_any_call(
        MIRRORED_TOPIC, b"", 0, True, message_expiry_interval=None
    )
    mqtt_setup.async_publish.assert_any_call(new_topic, ANY, 0, True, message_expiry_interval=None)
    assert new_hub.store.published_sensor_topics == [new_topic]
    assert hass.data[DOMAIN][DATA_HUB] is new_hub


# --------------------------------------------------------------------------- removal


async def test_remove_entry_clears_recorded_topics_registry_store_and_issues(
    hass: HomeAssistant,
    hass_storage: dict[str, Any],
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    complete_hub_start: Callable[[BridgeHub], Awaitable[None]],
    issue_registry: ir.IssueRegistry,
) -> None:
    entry = make_config_entry()
    hub = await _setup(hass, entry)
    await complete_hub_start(hub)
    hub.store.published_sensor_topics.extend(SENSOR_TOPICS)
    await hub.store.async_save()
    assert hass_storage[STORAGE_KEY]["data"]["published_sensor_topics"] == SENSOR_TOPICS
    ir.async_create_issue(
        hass,
        DOMAIN,
        "clock_skew_arm_left",
        is_fixable=False,
        severity=ir.IssueSeverity.WARNING,
        translation_key="clock_skew",
    )
    ir.async_create_issue(
        hass,
        "other_domain",
        "unrelated",
        is_fixable=False,
        severity=ir.IssueSeverity.WARNING,
        translation_key="unrelated",
    )

    mqtt_setup.async_publish.reset_mock()
    await hass.config_entries.async_remove(entry.entry_id)
    await hass.async_block_till_done()

    cleared = [call.args[0] for call in mqtt_setup.async_publish.call_args_list]
    for topic in SENSOR_TOPICS:
        assert topic in cleared
        mqtt_setup.async_publish.assert_any_call(topic, b"", 0, True, message_expiry_interval=None)
    assert STORAGE_KEY not in hass_storage
    assert entry.entry_id not in hass.data[DOMAIN][DATA_REGISTRIES]
    assert issue_registry.async_get_issue(DOMAIN, "clock_skew_arm_left") is None
    assert issue_registry.async_get_issue("other_domain", "unrelated") is not None


async def test_remove_entry_without_recorded_topics_is_quiet(
    hass: HomeAssistant,
    hass_storage: dict[str, Any],
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
) -> None:
    entry = make_config_entry()
    await _setup(hass, entry)
    mqtt_setup.async_publish.reset_mock()
    await hass.config_entries.async_remove(entry.entry_id)
    await hass.async_block_till_done()
    mqtt_setup.async_publish.assert_not_called()
    assert STORAGE_KEY not in hass_storage


# --------------------------------------------------------------------------- the Store across restarts


async def test_store_round_trip_across_two_simulated_restarts(
    hass: HomeAssistant,
    hass_storage: dict[str, Any],
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    complete_hub_start: Callable[[BridgeHub], Awaitable[None]],
) -> None:
    """A running command survives two restarts and its real result fires task_result once."""
    results = async_capture_events(hass, EVENT_TASK_RESULT)
    late = async_capture_events(hass, EVENT_TASK_LATE_RESULT)
    entry = make_config_entry()
    hub = await _setup(hass, entry)
    await complete_hub_start(hub)
    _announce_online(hass)
    await hass.async_block_till_done()
    command = await hub.async_send_task(
        ROBOT_ID, "reset", {}, source=CommandSource(kind="service")
    )
    task_id = command.task_id
    fire_robot_message(hass, f"task/{task_id}/ack", ack_payload(id=task_id, task="reset"))
    await hass.async_block_till_done()
    assert command.state is CommandState.RUNNING
    stored = hass_storage[STORAGE_KEY]["data"]["outstanding"][task_id]
    assert stored["state"] == "running"
    assert stored["session"] == announce_payload()["session"]
    exported = hub.registry.export()

    for restart in (1, 2):
        previous_registry = hub.registry
        assert await hass.config_entries.async_unload(entry.entry_id)
        await hass.async_block_till_done()
        # A restart loses the parked registry; the next setup must rebuild it from the Store.
        hass.data[DOMAIN][DATA_REGISTRIES].clear()
        hub = await _setup(hass, entry)
        assert hub.registry is not previous_registry, "a fresh registry after a restart"
        _announce_online(hass)  # the retained replay
        await hass.async_block_till_done()
        rebuilt = hub.registry.get(task_id)
        assert rebuilt is not None, f"restart {restart} lost the command"
        assert rebuilt.state is CommandState.RUNNING
        assert hub.registry.deadlines_suspended is True
        await complete_hub_start(hub)
        assert hub.registry.deadlines_suspended is False
        assert rebuilt.state is CommandState.RUNNING, f"restart {restart} did not re-adopt"
        assert hub.registry.export() == exported
        assert results == []

    now = hub.now
    fire_robot_message(
        hass,
        f"task/{task_id}/result",
        result_payload(id=task_id, task="reset", started_at=now - 2000, finished_at=now),
        retain=True,
    )
    await hass.async_block_till_done()
    assert hub.registry.get(task_id).state is CommandState.SUCCEEDED  # type: ignore[union-attr]
    assert len(results) == 1
    assert results[0].data["status"] == "succeeded"
    assert results[0].data["replayed"] is True
    assert results[0].data["device_id"] is not None
    assert late == []
    assert task_id not in hass_storage[STORAGE_KEY]["data"]["outstanding"]
    assert [task_id, now] in hass_storage[STORAGE_KEY]["data"]["seen_results"]


async def test_migrate_entry_is_a_no_op(
    hass: HomeAssistant, make_config_entry: Callable[..., MockConfigEntry]
) -> None:
    from custom_components.ros2_bridge import async_migrate_entry

    entry = make_config_entry()
    assert await async_migrate_entry(hass, entry) is True


# --------------------------------------------------------------------------- edges


async def test_setup_awaits_an_async_registrar(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A registrar written as a coroutine function is awaited, not left dangling."""
    calls: list[str] = []
    services = types.ModuleType(f"custom_components.{DOMAIN}.services")

    async def register_services(hass: HomeAssistant) -> None:
        calls.append("services")

    services.async_register_services = register_services  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, services.__name__, services)
    entry = make_config_entry()
    await _setup(hass, entry)
    assert calls == ["services"]


async def test_remove_entry_survives_a_failed_clear(
    hass: HomeAssistant,
    hass_storage: dict[str, Any],
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
    caplog: pytest.LogCaptureFixture,
) -> None:
    entry = make_config_entry()
    hub = await _setup(hass, entry)
    hub.store.published_sensor_topics.extend(SENSOR_TOPICS)
    await hub.store.async_save()
    assert await hass.config_entries.async_unload(entry.entry_id)
    mqtt_setup.async_publish.side_effect = HomeAssistantError("broker unreachable")
    await hass.config_entries.async_remove(entry.entry_id)
    await hass.async_block_till_done()
    assert STORAGE_KEY not in hass_storage
    assert "Could not clear" in caplog.text


async def test_remove_device_for_foreign_or_unloaded_entries(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Callable[..., MockConfigEntry],
) -> None:
    """A foreign device is let go quietly; an unloaded entry still publishes the tombstones."""
    entry = make_config_entry(options={CONF_TOPIC_PREFIX: "lab"})
    hub = await _setup(hass, entry)
    registry = dr.async_get(hass)
    foreign = registry.async_get_or_create(
        config_entry_id=entry.entry_id, identifiers={("other_domain", "x")}, name="foreign"
    )
    mqtt_setup.async_publish.reset_mock()
    assert await async_remove_config_entry_device(hass, entry, foreign) is True
    mqtt_setup.async_publish.assert_not_called()

    fire_robot_message(hass, "announce", announce_payload(), prefix="lab")
    fire_robot_message(hass, "availability", "online", prefix="lab")
    await hass.async_block_till_done()
    device = registry.async_get_device_by_identifier((DOMAIN, ROBOT_ID), entry.entry_id)
    assert device is not None
    assert hub.robots[ROBOT_ID].online
    assert await hass.config_entries.async_unload(entry.entry_id)
    await hass.async_block_till_done()
    assert not hasattr(entry, "runtime_data")
    mqtt_setup.async_publish.reset_mock()
    assert await async_remove_config_entry_device(hass, entry, device) is True
    # No hub to ask, so the robot counts as unknown and the entry's own prefix is used.
    for topic in (
        announce_topic("lab", ROBOT_ID),
        state_topic("lab", ROBOT_ID),
        availability_topic("lab", ROBOT_ID),
    ):
        mqtt_setup.async_publish.assert_any_call(topic, b"", 1, True, message_expiry_interval=None)
    assert mqtt_setup.async_publish.call_count == 3
