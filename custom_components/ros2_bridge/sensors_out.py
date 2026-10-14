# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The canonical sensor republish: ``{prefix}/ha/sensors/{domain}/{object_id}``.

``SensorMirror`` is a hub component (``hub.register_component``). It publishes
every entity named in the ``mirror_entities`` option once at start and again
on every state change, retained at QoS 0 with no expiry, in the contract's
sensor shape: ``unavailable`` and ``unknown`` become ``state: null`` with
``available: false``, and ``attributes`` is filtered to
:data:`SENSOR_ATTRIBUTE_ALLOWLIST` so large or private attributes never leave
Home Assistant. Topics it has published are recorded in the Store's
``published_sensor_topics``; at start any recorded topic the current options
no longer produce (an entity left the option, or the prefix changed) is
cleared with an empty retained publish and dropped from the list. Nothing is
published while HA's MQTT client is disconnected; the backlog is flushed when
it reconnects. ``async_clear_published_topics`` clears everything on entry
removal.
"""

from __future__ import annotations

import asyncio
import logging
from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, Final

from homeassistant.components import mqtt
from homeassistant.const import (
    ATTR_DEVICE_CLASS,
    ATTR_UNIT_OF_MEASUREMENT,
    STATE_UNAVAILABLE,
    STATE_UNKNOWN,
)
from homeassistant.core import (
    CALLBACK_TYPE,
    Event,
    EventStateChangedData,
    HomeAssistant,
    State,
    callback,
)
from homeassistant.exceptions import HomeAssistantError
from homeassistant.helpers.event import async_track_state_change_event

from .const import SCHEMA_VERSION, ha_sensor_topic
from .protocol import ProtocolError, encode_payload, validate_sensor_segment
from .store import CommandStore

if TYPE_CHECKING:
    from .hub import BridgeHub

_LOGGER = logging.getLogger(__name__)

#: The attributes a mirrored entity may carry onto the bus; everything else is dropped.
SENSOR_ATTRIBUTE_ALLOWLIST: Final[frozenset[str]] = frozenset(
    {
        "friendly_name",
        "device_class",
        "state_class",
        "unit_of_measurement",
        "icon",
        "battery",
        "battery_level",
        "linkquality",
        "last_seen",
        "occupancy",
        "motion",
        "contact",
        "tamper",
        "illuminance",
        "illuminance_lux",
        "temperature",
        "humidity",
        "pressure",
        "co2",
        "voc",
        "voltage",
        "current",
        "power",
        "energy",
        "brightness",
        "color_temp_kelvin",
        "rgb_color",
        "color_mode",
        "effect",
        "current_position",
        "current_tilt_position",
        "position",
        "percentage",
        "preset_mode",
        "speed",
        "hvac_action",
        "hvac_mode",
        "current_temperature",
        "target_temperature",
        "fan_mode",
        "options",
        "min",
        "max",
        "step",
        "mode",
    }
)

#: Publish settings of the sensor topic: QoS 0, retained, no expiry.
SENSOR_QOS: Final = 0
SENSOR_RETAIN: Final = True


def _jsonable(value: Any) -> Any:
    """A JSON-encodable copy of an attribute value (unknown types become their ``str``)."""
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set, frozenset)):
        return [_jsonable(item) for item in value]
    return str(value)


def _nullable_str(value: Any) -> str | None:
    return None if value is None else str(value)


def sensor_payload(entity_id: str, state: State | None, ts: int) -> dict[str, Any]:
    """The contract's sensor payload for an entity's current ``State`` (or ``None``)."""
    domain, _, object_id = entity_id.partition(".")
    attributes: Mapping[str, Any] = {} if state is None else state.attributes
    value: str | None = None
    if state is not None and state.state not in (STATE_UNAVAILABLE, STATE_UNKNOWN):
        value = str(state.state)
    return {
        "schema": SCHEMA_VERSION,
        "entity_id": entity_id,
        "domain": domain,
        "object_id": object_id,
        "state": value,
        "available": value is not None,
        "device_class": _nullable_str(attributes.get(ATTR_DEVICE_CLASS)),
        "unit": _nullable_str(attributes.get(ATTR_UNIT_OF_MEASUREMENT)),
        "attributes": {
            key: _jsonable(value)
            for key, value in attributes.items()
            if key in SENSOR_ATTRIBUTE_ALLOWLIST
        },
        "last_changed": None if state is None else state.last_changed.isoformat(),
        "ts": ts,
    }


def sensor_topic(prefix: str, entity_id: str) -> str | None:
    """The topic an entity is mirrored on, or ``None`` when its id cannot form one."""
    domain, _, object_id = entity_id.partition(".")
    try:
        validate_sensor_segment("domain", domain)
        validate_sensor_segment("object_id", object_id)
    except ProtocolError as error:
        _LOGGER.warning("Not mirroring %s: %s", entity_id, error.reason)
        return None
    return ha_sensor_topic(prefix, domain, object_id)


class SensorMirror:
    """Republishes ``mirror_entities`` and keeps the Store's topic list true."""

    def __init__(self, hass: HomeAssistant, hub: BridgeHub) -> None:
        self.hass = hass
        self.hub = hub
        self.started = False
        #: Entity id to topic for every mirrored entity whose id forms a valid topic.
        self.topics: dict[str, str] = {}
        self._pending: set[str] = set()
        self._pending_clear: set[str] = set()
        self._unsubscribe: CALLBACK_TYPE | None = None
        self._lock = asyncio.Lock()

    @property
    def store(self) -> CommandStore:
        store: CommandStore = self.hub.store
        return store

    # ----------------------------------------------------------------- HubComponent

    async def async_start(self) -> None:
        """Track the mirrored entities, queue a publish of each and clear stale topics."""
        if self.started:
            return
        self.started = True
        self.topics = {}
        for entity_id in self.hub.options.mirror_entities:
            topic = sensor_topic(self.hub.prefix, entity_id)
            if topic is not None:
                self.topics[entity_id] = topic
        if self.topics:
            self._unsubscribe = async_track_state_change_event(
                self.hass, list(self.topics), self._on_state_changed
            )
        produced = set(self.topics.values())
        self._pending_clear.update(
            topic for topic in self.store.published_sensor_topics if topic not in produced
        )
        self._pending.update(self.topics)
        await self._async_flush()

    async def async_stop(self) -> None:
        """Stop tracking; retained topics stay on the broker for the next start."""
        if self._unsubscribe is not None:
            self._unsubscribe()
            self._unsubscribe = None
        self.started = False

    @callback
    def async_connection_changed(self, connected: bool) -> None:
        """Republish everything (and clear the backlog) once MQTT is back."""
        if not connected or not self.started:
            return
        self._pending.update(self.topics)
        self.hass.async_create_task(self._async_flush())

    # ----------------------------------------------------------------- internals

    @callback
    def _on_state_changed(self, event: Event[EventStateChangedData]) -> None:
        self._pending.add(event.data["entity_id"])
        self.hass.async_create_task(self._async_flush())

    async def _async_publish(self, topic: str, payload: bytes) -> bool:
        """Publish retained at QoS 0; False (and a counted error) when the client refused."""
        try:
            await mqtt.async_publish(
                self.hass, topic, payload, qos=SENSOR_QOS, retain=SENSOR_RETAIN
            )
        except HomeAssistantError as error:
            self.hub.counters["publish_errors"] += 1
            _LOGGER.warning("Could not publish %s: %s", topic, error)
            return False
        return True

    async def _async_flush(self) -> None:
        """Clear stale topics, then publish every pending entity, one writer at a time."""
        async with self._lock:
            if not self.hub.mqtt_connected or not self.started:
                return
            recorded = self.store.published_sensor_topics
            changed = False
            while self._pending_clear:
                topic = self._pending_clear.pop()
                if not await self._async_publish(topic, b""):
                    self._pending_clear.add(topic)
                    break
                while topic in recorded:
                    recorded.remove(topic)
                changed = True
            while self._pending:
                entity_id = self._pending.pop()
                target = self.topics.get(entity_id)
                if target is None:
                    continue
                payload = sensor_payload(entity_id, self.hass.states.get(entity_id), self.hub.now)
                if not await self._async_publish(target, encode_payload(payload)):
                    self._pending.add(entity_id)
                    break
                if target not in recorded:
                    recorded.append(target)
                    changed = True
            if changed:
                self.store.async_request_save(False)


def async_setup_sensor_mirror(hub: BridgeHub) -> SensorMirror:
    """Create the mirror for a hub and register it as a component."""
    mirror = SensorMirror(hub.hass, hub)
    hub.register_component(mirror)
    return mirror


async def async_clear_published_topics(hass: HomeAssistant, store: CommandStore) -> None:
    """Clear every recorded sensor topic with an empty retained publish (entry removal)."""
    for topic in list(store.published_sensor_topics):
        try:
            await mqtt.async_publish(hass, topic, b"", qos=SENSOR_QOS, retain=SENSOR_RETAIN)
        except HomeAssistantError as error:
            _LOGGER.warning("Could not clear %s: %s", topic, error)
            continue
        store.published_sensor_topics.remove(topic)
