# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Config flow and options flow of the ROS 2 Bridge integration.

One config entry per Home Assistant instance (``single_config_entry`` in the
manifest). The user step asks only for the topic prefix and the optional
shared secret because the broker itself comes from Home Assistant's own MQTT
integration; it aborts when MQTT is not configured or its client never
becomes available. The MQTT discovery step is driven by the manifest's
``ros2_bridge/+/announce`` subscription: a retained announce from a robot
offers to create the entry after a confirmation. The options flow edits the
nine options and reloads the entry when it finishes
(``OptionsFlowWithReload``), so the hub picks the new values up without an
update listener.
"""

from __future__ import annotations

import uuid
from typing import Any

import probatio
from homeassistant.components import mqtt
from homeassistant.config_entries import (
    ConfigEntry,
    ConfigFlow,
    ConfigFlowResult,
    OptionsFlowWithReload,
)
from homeassistant.core import DOMAIN as HOMEASSISTANT_DOMAIN
from homeassistant.core import callback
from homeassistant.helpers.selector import (
    EntitySelector,
    EntitySelectorConfig,
    NumberSelector,
    NumberSelectorConfig,
    NumberSelectorMode,
    TextSelector,
    TextSelectorConfig,
    TextSelectorType,
)
from homeassistant.helpers.service_info.mqtt import MqttServiceInfo

from .const import (
    CONF_ACK_TIMEOUT_S,
    CONF_COMMAND_MAX_AGE_MS,
    CONF_HEARTBEAT_INTERVAL_S,
    CONF_HEARTBEAT_UNAVAILABLE_AFTER_S,
    CONF_INSTANCE_ID,
    CONF_MAX_PENDING_PER_ROBOT,
    CONF_MIRROR_ENTITIES,
    CONF_RESULT_TIMEOUT_MARGIN_S,
    CONF_SHARED_SECRET,
    CONF_TOPIC_PREFIX,
    DEFAULT_MIRROR_ENTITIES,
    DEFAULT_OPTIONS,
    DEFAULT_SHARED_SECRET,
    DEFAULT_TOPIC_PREFIX,
    DOMAIN,
)
from .models import Announce
from .protocol import (
    ProtocolError,
    decode_announce,
    is_valid_robot_id,
    prefix_error,
    split_robot_topic,
)

#: The config entry title; there is only ever one entry.
TITLE = "ROS 2 Bridge"
#: What the manifest's discovery filter ends with; the prefix is what precedes it.
ANNOUNCE_SUFFIX = "/+/announce"
#: The one error key the flows raise, on the ``topic_prefix`` field.
ERROR_INVALID_PREFIX = "invalid_prefix"

#: Abort reasons (translation keys under ``config.abort``).
ABORT_MQTT_NOT_CONFIGURED = "mqtt_not_configured"
ABORT_MQTT_NOT_AVAILABLE = "mqtt_not_available"
ABORT_INVALID_DISCOVERY_INFO = "invalid_discovery_info"
ABORT_SINGLE_INSTANCE = "single_instance_allowed"

_SECRET_SELECTOR = TextSelector(TextSelectorConfig(type=TextSelectorType.PASSWORD))


def _number(
    minimum: float, maximum: float, step: float, unit: str | None = None
) -> NumberSelector:
    config = NumberSelectorConfig(min=minimum, max=maximum, step=step, mode=NumberSelectorMode.BOX)
    if unit is not None:
        config["unit_of_measurement"] = unit
    return NumberSelector(config)


USER_SCHEMA: probatio.Schema = probatio.Schema(
    {
        probatio.Required(CONF_TOPIC_PREFIX, default=DEFAULT_TOPIC_PREFIX): TextSelector(),
        probatio.Optional(CONF_SHARED_SECRET, default=DEFAULT_SHARED_SECRET): _SECRET_SELECTOR,
    }
)

CONFIRM_SCHEMA: probatio.Schema = probatio.Schema(
    {probatio.Optional(CONF_SHARED_SECRET, default=DEFAULT_SHARED_SECRET): _SECRET_SELECTOR}
)

#: The nine options with the design's ranges; defaults are suggested from the entry.
OPTIONS_SCHEMA: probatio.Schema = probatio.Schema(
    {
        probatio.Required(CONF_TOPIC_PREFIX): TextSelector(),
        probatio.Optional(CONF_SHARED_SECRET, default=DEFAULT_SHARED_SECRET): _SECRET_SELECTOR,
        probatio.Required(CONF_HEARTBEAT_INTERVAL_S): _number(0, 10, 0.5, "s"),
        probatio.Required(CONF_ACK_TIMEOUT_S): _number(0.5, 10, 0.1, "s"),
        probatio.Required(CONF_COMMAND_MAX_AGE_MS): _number(1000, 30000, 100, "ms"),
        probatio.Required(CONF_RESULT_TIMEOUT_MARGIN_S): _number(0, 300, 1, "s"),
        probatio.Required(CONF_MAX_PENDING_PER_ROBOT): _number(1, 100, 1),
        probatio.Required(CONF_HEARTBEAT_UNAVAILABLE_AFTER_S): _number(0, 600, 1, "s"),
        probatio.Optional(CONF_MIRROR_ENTITIES, default=list(DEFAULT_MIRROR_ENTITIES)): (
            EntitySelector(EntitySelectorConfig(multiple=True))
        ),
    }
)


def validate_prefix(prefix: str) -> str | None:
    """Return the error key for an unusable topic prefix, or ``None`` when it is fine.

    Home Assistant's own publish-topic rule comes first (it refuses ``+`` and
    ``#``, control characters and an empty string), then the contract's prefix
    rules from ``protocol.prefix_error`` (no leading or trailing slash, no empty
    segment, not ``homeassistant``, not starting with the ``ha`` segment).
    """
    try:
        mqtt.valid_publish_topic(prefix)
    except probatio.Invalid:
        return ERROR_INVALID_PREFIX
    if prefix_error(prefix) is not None:
        return ERROR_INVALID_PREFIX
    return None


def normalise_options(user_input: dict[str, Any]) -> dict[str, Any]:
    """The options document from a submitted options form.

    Number selectors return floats; the counts and millisecond values are stored
    as integers so the Store and the diagnostics show what the user typed.
    """
    return {
        CONF_TOPIC_PREFIX: str(user_input[CONF_TOPIC_PREFIX]).strip(),
        CONF_SHARED_SECRET: str(user_input.get(CONF_SHARED_SECRET) or DEFAULT_SHARED_SECRET),
        CONF_HEARTBEAT_INTERVAL_S: float(user_input[CONF_HEARTBEAT_INTERVAL_S]),
        CONF_ACK_TIMEOUT_S: float(user_input[CONF_ACK_TIMEOUT_S]),
        CONF_COMMAND_MAX_AGE_MS: int(user_input[CONF_COMMAND_MAX_AGE_MS]),
        CONF_RESULT_TIMEOUT_MARGIN_S: int(user_input[CONF_RESULT_TIMEOUT_MARGIN_S]),
        CONF_MAX_PENDING_PER_ROBOT: int(user_input[CONF_MAX_PENDING_PER_ROBOT]),
        CONF_HEARTBEAT_UNAVAILABLE_AFTER_S: int(user_input[CONF_HEARTBEAT_UNAVAILABLE_AFTER_S]),
        CONF_MIRROR_ENTITIES: [
            str(entity_id) for entity_id in user_input.get(CONF_MIRROR_ENTITIES) or []
        ],
    }


def parse_discovery(discovery_info: MqttServiceInfo) -> tuple[str, Announce] | None:
    """The prefix and the decoded announce behind a discovery message, or ``None``.

    The prefix is the subscribed filter minus ``/+/announce`` and must pass the
    contract's prefix rules; the topic's robot segment must match the robot id
    pattern; the payload must decode as a schema 1 announce whose ``robot_id``
    equals the segment. Anything else (an empty tombstone included) is not a
    robot the flow can offer.
    """
    subscribed = discovery_info.subscribed_topic
    if not subscribed.endswith(ANNOUNCE_SUFFIX):
        return None
    prefix = subscribed.removesuffix(ANNOUNCE_SUFFIX)
    if prefix_error(prefix) is not None:
        return None
    split = split_robot_topic(prefix, discovery_info.topic)
    if split is None or split[1] != ["announce"] or not is_valid_robot_id(split[0]):
        return None
    try:
        announce = decode_announce(discovery_info.payload)
    except ProtocolError:
        return None
    if announce.robot_id != split[0]:
        return None
    return prefix, announce


class Ros2BridgeConfigFlow(ConfigFlow, domain=DOMAIN):
    """The user and MQTT discovery steps."""

    VERSION = 1
    MINOR_VERSION = 1

    def __init__(self) -> None:
        """Start with the default prefix; discovery replaces it."""
        self._prefix = DEFAULT_TOPIC_PREFIX
        self._robot_name = ""

    @staticmethod
    @callback
    def async_get_options_flow(config_entry: ConfigEntry) -> Ros2BridgeOptionsFlow:
        """The options flow for the one entry."""
        return Ros2BridgeOptionsFlow()

    async def async_step_user(self, user_input: dict[str, Any] | None = None) -> ConfigFlowResult:
        """Ask for the topic prefix and the optional shared secret."""
        if not mqtt.mqtt_config_entry_enabled(self.hass):
            return self.async_abort(reason=ABORT_MQTT_NOT_CONFIGURED)
        if not await mqtt.async_wait_for_mqtt_client(self.hass):
            return self.async_abort(reason=ABORT_MQTT_NOT_AVAILABLE)
        await self.async_set_unique_id(DOMAIN)
        self._abort_if_unique_id_configured()
        errors: dict[str, str] = {}
        if user_input is not None:
            prefix = str(user_input[CONF_TOPIC_PREFIX]).strip()
            error = validate_prefix(prefix)
            if error is None:
                return self._create_entry(prefix, user_input.get(CONF_SHARED_SECRET))
            errors[CONF_TOPIC_PREFIX] = error
        return self.async_show_form(
            step_id="user",
            data_schema=self.add_suggested_values_to_schema(USER_SCHEMA, user_input),
            errors=errors,
        )

    async def async_step_mqtt(self, discovery_info: MqttServiceInfo) -> ConfigFlowResult:
        """Offer the entry when a robot's retained announce is seen on the broker."""
        if self._async_in_progress() or self._async_current_entries():
            return self.async_abort(
                reason=ABORT_SINGLE_INSTANCE, translation_domain=HOMEASSISTANT_DOMAIN
            )
        parsed = parse_discovery(discovery_info)
        if parsed is None:
            return self.async_abort(reason=ABORT_INVALID_DISCOVERY_INFO)
        self._prefix, announce = parsed
        self._robot_name = announce.name
        await self.async_set_unique_id(DOMAIN)
        self.context["title_placeholders"] = {"name": announce.name}
        return await self.async_step_confirm()

    async def async_step_confirm(
        self, user_input: dict[str, Any] | None = None
    ) -> ConfigFlowResult:
        """Confirm the discovered robot and take the optional shared secret."""
        if user_input is not None:
            return self._create_entry(self._prefix, user_input.get(CONF_SHARED_SECRET))
        return self.async_show_form(
            step_id="confirm",
            data_schema=CONFIRM_SCHEMA,
            description_placeholders={
                "robot_name": self._robot_name,
                "topic_prefix": self._prefix,
            },
        )

    def _create_entry(self, prefix: str, secret: object) -> ConfigFlowResult:
        """Create the entry: a fresh instance id in ``data``, every option in ``options``."""
        options: dict[str, Any] = {
            **DEFAULT_OPTIONS,
            CONF_TOPIC_PREFIX: prefix,
            CONF_SHARED_SECRET: str(secret or DEFAULT_SHARED_SECRET),
            CONF_MIRROR_ENTITIES: list(DEFAULT_MIRROR_ENTITIES),
        }
        return self.async_create_entry(
            title=TITLE,
            data={CONF_INSTANCE_ID: str(uuid.uuid4())},
            options=options,
        )


class Ros2BridgeOptionsFlow(OptionsFlowWithReload):
    """Edit the nine options; saving reloads the entry."""

    async def async_step_init(self, user_input: dict[str, Any] | None = None) -> ConfigFlowResult:
        """The one options form."""
        errors: dict[str, str] = {}
        if user_input is not None:
            options = normalise_options(user_input)
            error = validate_prefix(options[CONF_TOPIC_PREFIX])
            if error is None:
                return self.async_create_entry(data=options)
            errors[CONF_TOPIC_PREFIX] = error
        current: dict[str, Any] = {
            **DEFAULT_OPTIONS,
            **self.config_entry.options,
            **(user_input or {}),
        }
        return self.async_show_form(
            step_id="init",
            data_schema=self.add_suggested_values_to_schema(OPTIONS_SCHEMA, current),
            errors=errors,
        )
