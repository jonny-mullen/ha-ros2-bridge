# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The config flow and the options flow of the ROS 2 Bridge integration.

User happy path, the two MQTT aborts, every invalid prefix, the single
instance rule (Home Assistant's own pre-emption and the flow's in-progress
check), MQTT discovery through the confirm step to an entry, every junk
discovery, and the options flow with its reload. ``async_setup_entry`` is
patched in while the entry is created, as Home Assistant's own flow tests do,
so these tests do not depend on the hub wiring in ``__init__.py``. The options
form carries a collapsed advanced section; the stored document stays flat.
"""

from __future__ import annotations

import json
from typing import Any
from unittest.mock import AsyncMock, patch

import probatio
import pytest
from homeassistant.config_entries import SOURCE_MQTT, SOURCE_USER, ConfigEntryState
from homeassistant.core import HomeAssistant
from homeassistant.data_entry_flow import FlowResultType, InvalidData, section
from homeassistant.helpers.service_info.mqtt import MqttServiceInfo
from pytest_homeassistant_custom_component.common import MockConfigEntry

from custom_components.ros2_bridge.config_flow import (
    ABORT_INVALID_DISCOVERY_INFO,
    ABORT_MQTT_NOT_AVAILABLE,
    ABORT_MQTT_NOT_CONFIGURED,
    ABORT_SINGLE_INSTANCE,
    ERROR_INVALID_PREFIX,
    TITLE,
    flatten_options,
    nest_options,
    normalise_options,
    parse_discovery,
    validate_prefix,
)
from custom_components.ros2_bridge.const import (
    ADVANCED_OPTIONS,
    CONF_ACK_EXPIRY_S,
    CONF_ACK_TIMEOUT_S,
    CONF_CLOCK_SKEW_WARN_MS,
    CONF_COMMAND_MAX_AGE_MS,
    CONF_DUPLICATE_QUIET_S,
    CONF_DUPLICATE_SESSION_CHANGES,
    CONF_DUPLICATE_SESSION_WINDOW_S,
    CONF_FEEDBACK_THROTTLE_MS,
    CONF_HEARTBEAT_INTERVAL_S,
    CONF_HEARTBEAT_STALE_INTERVALS,
    CONF_HEARTBEAT_UNAVAILABLE_AFTER_S,
    CONF_INSTANCE_ID,
    CONF_INTENT_ACK_WAIT_S,
    CONF_JOINT_MOVING_HOLD_S,
    CONF_JOINT_MOVING_THRESHOLD,
    CONF_MAX_PENDING_PER_ROBOT,
    CONF_MIRROR_ENTITIES,
    CONF_MIRROR_EXTRA_ATTRIBUTES,
    CONF_REPLAY_SETTLE_S,
    CONF_RESULT_EXPIRY_S,
    CONF_RESULT_TIMEOUT_MARGIN_S,
    CONF_SESSION_CHANGE_GRACE_S,
    CONF_SHARED_SECRET,
    CONF_TOPIC_PREFIX,
    DEFAULT_OPTIONS,
    DEFAULT_TOPIC_PREFIX,
    DOMAIN,
    ISSUE_KINDS,
    SECTION_ADVANCED,
    TASK_ID_RE,
)
from custom_components.ros2_bridge.hub import BridgeHub, HubOptions
from custom_components.ros2_bridge.protocol import CommandSource

from .common import REPO_ROOT, ROBOT_ID, announce_payload, encode, fire_robot_message

INTEGRATION_DIR = REPO_ROOT / "custom_components" / "ros2_bridge"
#: A placeholder for tests only; it is not a credential.
PLACEHOLDER_SECRET = "placeholder-shared-secret"
INVALID_PREFIXES = ["+", "a/+", "#", "homeassistant", "/x", "x/", "ha/x", "", "a//b", "lab/#"]
EXCEPTION_KEYS = (
    "not_loaded",
    "not_ready",
    "robot_unknown",
    "robot_unavailable",
    "task_unknown",
    "invalid_params",
    "too_many_pending",
    "publish_failed",
    "no_ack",
    "no_result",
    "task_rejected",
)


def discovery(
    topic: str = f"ros2_bridge/{ROBOT_ID}/announce",
    payload: Any = None,
    subscribed_topic: str = "ros2_bridge/+/announce",
) -> MqttServiceInfo:
    """What the mqtt integration hands the flow for a retained announce."""
    if payload is None:
        payload = encode(announce_payload())
    return MqttServiceInfo(
        topic=topic,
        payload=payload,
        qos=1,
        retain=True,
        subscribed_topic=subscribed_topic,
        timestamp=0.0,
    )


class patch_setup_entry:
    """Stand in for the entry setup and unload while an entry is created.

    Home Assistant's own flow tests patch ``async_setup_entry``; the unload is
    patched too because the test harness unloads every entry at teardown and a
    mocked setup leaves no ``runtime_data`` for the real unload to find.
    """

    def __init__(self) -> None:
        self.setup = AsyncMock(return_value=True)
        self._patches = [
            patch(f"custom_components.ros2_bridge.{name}", new=mock, create=True)
            for name, mock in (
                ("async_setup_entry", self.setup),
                ("async_unload_entry", AsyncMock(return_value=True)),
            )
        ]

    def __enter__(self) -> AsyncMock:
        for patcher in self._patches:
            patcher.start()
        return self.setup

    def __exit__(self, *exc: object) -> None:
        for patcher in reversed(self._patches):
            patcher.stop()


def options_input(**overrides: Any) -> dict[str, Any]:
    """An options form submission as the frontend sends it.

    Numbers arrive as floats and the advanced options nested under the section;
    ``overrides`` are given flat and nested here.
    """
    data: dict[str, Any] = {
        key: float(value) if isinstance(value, (int, float)) else list(value)
        for key, value in DEFAULT_OPTIONS.items()
        if not isinstance(value, str)
    }
    data[CONF_TOPIC_PREFIX] = DEFAULT_TOPIC_PREFIX
    data[CONF_SHARED_SECRET] = ""
    data.update(overrides)
    return nest_options(data)


# --------------------------------------------------------------------------- user step


async def test_user_flow_creates_the_entry(hass: HomeAssistant, mqtt_setup: Any) -> None:
    result = await hass.config_entries.flow.async_init(DOMAIN, context={"source": SOURCE_USER})
    assert result["type"] is FlowResultType.FORM
    assert result["step_id"] == "user"
    assert result["errors"] == {}
    with patch_setup_entry() as mock_setup_entry:
        result = await hass.config_entries.flow.async_configure(
            result["flow_id"],
            {CONF_TOPIC_PREFIX: " lab/arms ", CONF_SHARED_SECRET: PLACEHOLDER_SECRET},
        )
        await hass.async_block_till_done()
    assert result["type"] is FlowResultType.CREATE_ENTRY
    assert result["title"] == TITLE
    entry = result["result"]
    assert entry.unique_id == DOMAIN
    assert entry.version == 1 and entry.minor_version == 1
    assert TASK_ID_RE.fullmatch(entry.data[CONF_INSTANCE_ID])
    assert entry.options == {
        **DEFAULT_OPTIONS,
        CONF_TOPIC_PREFIX: "lab/arms",
        CONF_SHARED_SECRET: PLACEHOLDER_SECRET,
    }
    assert entry.options[CONF_MIRROR_ENTITIES] is not DEFAULT_OPTIONS[CONF_MIRROR_ENTITIES]
    assert mock_setup_entry.await_count == 1


async def test_user_flow_without_a_secret(hass: HomeAssistant, mqtt_setup: Any) -> None:
    result = await hass.config_entries.flow.async_init(DOMAIN, context={"source": SOURCE_USER})
    with patch_setup_entry():
        result = await hass.config_entries.flow.async_configure(
            result["flow_id"], {CONF_TOPIC_PREFIX: DEFAULT_TOPIC_PREFIX}
        )
        await hass.async_block_till_done()
    assert result["type"] is FlowResultType.CREATE_ENTRY
    assert result["result"].options == DEFAULT_OPTIONS


async def test_user_flow_aborts_when_mqtt_is_not_configured(hass: HomeAssistant) -> None:
    result = await hass.config_entries.flow.async_init(DOMAIN, context={"source": SOURCE_USER})
    assert result["type"] is FlowResultType.ABORT
    assert result["reason"] == ABORT_MQTT_NOT_CONFIGURED


async def test_user_flow_aborts_when_mqtt_is_not_available(
    hass: HomeAssistant, mqtt_setup: Any
) -> None:
    with patch(
        "homeassistant.components.mqtt.async_wait_for_mqtt_client", return_value=False
    ) as wait:
        result = await hass.config_entries.flow.async_init(DOMAIN, context={"source": SOURCE_USER})
    assert wait.await_count == 1
    assert result["type"] is FlowResultType.ABORT
    assert result["reason"] == ABORT_MQTT_NOT_AVAILABLE


@pytest.mark.parametrize("prefix", INVALID_PREFIXES)
async def test_user_flow_rejects_invalid_prefixes(
    hass: HomeAssistant, mqtt_setup: Any, prefix: str
) -> None:
    result = await hass.config_entries.flow.async_init(DOMAIN, context={"source": SOURCE_USER})
    result = await hass.config_entries.flow.async_configure(
        result["flow_id"], {CONF_TOPIC_PREFIX: prefix}
    )
    assert result["type"] is FlowResultType.FORM
    assert result["step_id"] == "user"
    assert result["errors"] == {CONF_TOPIC_PREFIX: ERROR_INVALID_PREFIX}


async def test_user_flow_recovers_from_an_invalid_prefix(
    hass: HomeAssistant, mqtt_setup: Any
) -> None:
    result = await hass.config_entries.flow.async_init(DOMAIN, context={"source": SOURCE_USER})
    result = await hass.config_entries.flow.async_configure(
        result["flow_id"], {CONF_TOPIC_PREFIX: "ha/x"}
    )
    assert result["errors"] == {CONF_TOPIC_PREFIX: ERROR_INVALID_PREFIX}
    with patch_setup_entry():
        result = await hass.config_entries.flow.async_configure(
            result["flow_id"], {CONF_TOPIC_PREFIX: "lab"}
        )
        await hass.async_block_till_done()
    assert result["type"] is FlowResultType.CREATE_ENTRY
    assert result["result"].options[CONF_TOPIC_PREFIX] == "lab"


@pytest.mark.parametrize("prefix", INVALID_PREFIXES)
def test_validate_prefix_refuses(prefix: str) -> None:
    assert validate_prefix(prefix) == ERROR_INVALID_PREFIX


@pytest.mark.parametrize("prefix", ["ros2_bridge", "lab/arms", "home_assistant", "hab/x"])
def test_validate_prefix_accepts(prefix: str) -> None:
    assert validate_prefix(prefix) is None


# --------------------------------------------------------------------------- single instance


async def test_second_entry_is_refused(
    hass: HomeAssistant, mqtt_setup: Any, make_config_entry: Any
) -> None:
    """With an entry present Home Assistant itself aborts both sources (single_config_entry)."""
    make_config_entry()
    result = await hass.config_entries.flow.async_init(DOMAIN, context={"source": SOURCE_USER})
    assert result["type"] is FlowResultType.ABORT
    assert result["reason"] == ABORT_SINGLE_INSTANCE
    result = await hass.config_entries.flow.async_init(
        DOMAIN, context={"source": SOURCE_MQTT}, data=discovery()
    )
    assert result["type"] is FlowResultType.ABORT
    assert result["reason"] == ABORT_SINGLE_INSTANCE


async def test_discovery_aborts_while_a_user_flow_is_in_progress(
    hass: HomeAssistant, mqtt_setup: Any
) -> None:
    """The flow's own check: a discovery arriving during a user flow does not start a second."""
    first = await hass.config_entries.flow.async_init(DOMAIN, context={"source": SOURCE_USER})
    assert first["type"] is FlowResultType.FORM
    result = await hass.config_entries.flow.async_init(
        DOMAIN, context={"source": SOURCE_MQTT}, data=discovery()
    )
    assert result["type"] is FlowResultType.ABORT
    assert result["reason"] == ABORT_SINGLE_INSTANCE
    assert hass.config_entries.flow.async_get(first["flow_id"])["step_id"] == "user"


# --------------------------------------------------------------------------- MQTT discovery


async def test_discovery_confirms_and_creates_the_entry(
    hass: HomeAssistant, mqtt_setup: Any
) -> None:
    result = await hass.config_entries.flow.async_init(
        DOMAIN,
        context={"source": SOURCE_MQTT},
        data=discovery(topic=f"lab/{ROBOT_ID}/announce", subscribed_topic="lab/+/announce"),
    )
    assert result["type"] is FlowResultType.FORM
    assert result["step_id"] == "confirm"
    assert result["description_placeholders"] == {"robot_name": "Left arm", "topic_prefix": "lab"}
    progress = hass.config_entries.flow.async_get(result["flow_id"])
    assert progress["context"]["title_placeholders"] == {"name": "Left arm"}
    assert progress["context"]["unique_id"] == DOMAIN
    with patch_setup_entry() as mock_setup_entry:
        result = await hass.config_entries.flow.async_configure(
            result["flow_id"], {CONF_SHARED_SECRET: PLACEHOLDER_SECRET}
        )
        await hass.async_block_till_done()
    assert result["type"] is FlowResultType.CREATE_ENTRY
    assert result["title"] == TITLE
    entry = result["result"]
    assert entry.unique_id == DOMAIN
    assert TASK_ID_RE.fullmatch(entry.data[CONF_INSTANCE_ID])
    assert entry.options == {
        **DEFAULT_OPTIONS,
        CONF_TOPIC_PREFIX: "lab",
        CONF_SHARED_SECRET: PLACEHOLDER_SECRET,
    }
    assert mock_setup_entry.await_count == 1


async def test_discovery_confirm_without_a_secret(hass: HomeAssistant, mqtt_setup: Any) -> None:
    result = await hass.config_entries.flow.async_init(
        DOMAIN, context={"source": SOURCE_MQTT}, data=discovery()
    )
    assert result["step_id"] == "confirm"
    with patch_setup_entry():
        result = await hass.config_entries.flow.async_configure(result["flow_id"], {})
        await hass.async_block_till_done()
    assert result["type"] is FlowResultType.CREATE_ENTRY
    assert result["result"].options == DEFAULT_OPTIONS


@pytest.mark.parametrize(
    "info",
    [
        discovery(payload=b"not json"),
        discovery(payload=""),
        discovery(payload=encode(announce_payload(robot_id="arm left"))),
        discovery(topic="ros2_bridge/arm left/announce"),
        discovery(topic="ros2_bridge/ha/announce"),
        discovery(topic=f"ros2_bridge/{ROBOT_ID}/state"),
        discovery(topic=f"other/{ROBOT_ID}/announce"),
        discovery(subscribed_topic="ros2_bridge/+/state"),
        discovery(
            topic=f"homeassistant/{ROBOT_ID}/announce",
            subscribed_topic="homeassistant/+/announce",
        ),
        discovery(payload=encode(announce_payload(robot_id="arm_right"))),
    ],
    ids=[
        "junk_payload",
        "tombstone",
        "robot_id_with_space_in_payload",
        "robot_id_with_space_in_topic",
        "reserved_ha_segment",
        "not_an_announce_topic",
        "topic_outside_the_prefix",
        "filter_without_announce_suffix",
        "discovery_prefix",
        "payload_robot_id_differs",
    ],
)
async def test_junk_discovery_aborts(
    hass: HomeAssistant, mqtt_setup: Any, info: MqttServiceInfo
) -> None:
    assert parse_discovery(info) is None
    result = await hass.config_entries.flow.async_init(
        DOMAIN, context={"source": SOURCE_MQTT}, data=info
    )
    assert result["type"] is FlowResultType.ABORT
    assert result["reason"] == ABORT_INVALID_DISCOVERY_INFO


def test_parse_discovery_returns_prefix_and_announce() -> None:
    parsed = parse_discovery(
        discovery(topic=f"lab/arms/{ROBOT_ID}/announce", subscribed_topic="lab/arms/+/announce")
    )
    assert parsed is not None
    prefix, announce = parsed
    assert prefix == "lab/arms"
    assert announce.robot_id == ROBOT_ID
    assert announce.name == "Left arm"


# --------------------------------------------------------------------------- options flow


def _suggested(schema: probatio.Schema) -> dict[str, Any]:
    """The suggested values a shown form carries, keyed by field name."""
    return {
        key.schema: key.description["suggested_value"]
        for key in schema.schema
        if isinstance(key, probatio.Marker) and key.description
    }


async def test_options_flow_saves_and_schedules_a_reload(
    hass: HomeAssistant, mqtt_setup: Any, make_config_entry: Any
) -> None:
    entry = make_config_entry()
    entry.mock_state(hass, ConfigEntryState.LOADED)
    result = await hass.config_entries.options.async_init(entry.entry_id)
    assert result["type"] is FlowResultType.FORM
    assert result["step_id"] == "init"
    assert result["errors"] == {}
    # The form shows the stored (flat) options, the advanced ones nested under the section.
    schema = result["data_schema"]
    assert schema is not None
    assert _suggested(schema)[CONF_TOPIC_PREFIX] == DEFAULT_TOPIC_PREFIX
    advanced = next(
        value
        for key, value in schema.schema.items()
        if isinstance(key, probatio.Marker) and key.schema == SECTION_ADVANCED
    )
    assert isinstance(advanced, section)
    assert advanced.options == {"collapsed": True}
    assert _suggested(advanced.schema) == {key: DEFAULT_OPTIONS[key] for key in ADVANCED_OPTIONS}

    result = await hass.config_entries.options.async_configure(
        result["flow_id"], options_input(topic_prefix="ha/x")
    )
    assert result["type"] is FlowResultType.FORM
    assert result["errors"] == {CONF_TOPIC_PREFIX: ERROR_INVALID_PREFIX}
    with patch.object(hass.config_entries, "async_schedule_reload") as schedule_reload:
        result = await hass.config_entries.options.async_configure(
            result["flow_id"],
            options_input(
                topic_prefix=" lab ",
                shared_secret=PLACEHOLDER_SECRET,
                ack_timeout_s=3.5,
                command_max_age_ms=8000.0,
                mirror_entities=["binary_sensor.kitchen_motion"],
                mirror_extra_attributes=["pm2_5", " moisture "],
                result_expiry_s=3600.0,
                duplicate_quiet_s=900.0,
                session_change_grace_s=2.5,
            ),
        )
        await hass.async_block_till_done()
    assert result["type"] is FlowResultType.CREATE_ENTRY
    assert entry.options == {
        **DEFAULT_OPTIONS,
        CONF_TOPIC_PREFIX: "lab",
        CONF_SHARED_SECRET: PLACEHOLDER_SECRET,
        CONF_ACK_TIMEOUT_S: 3.5,
        CONF_COMMAND_MAX_AGE_MS: 8000,
        CONF_MIRROR_ENTITIES: ["binary_sensor.kitchen_motion"],
        CONF_MIRROR_EXTRA_ATTRIBUTES: ["pm2_5", "moisture"],
        CONF_RESULT_EXPIRY_S: 3600,
        CONF_DUPLICATE_QUIET_S: 900,
        CONF_SESSION_CHANGE_GRACE_S: 2.5,
    }
    assert SECTION_ADVANCED not in entry.options, "the stored document is flat"
    for key in (CONF_COMMAND_MAX_AGE_MS, CONF_RESULT_EXPIRY_S, CONF_DUPLICATE_QUIET_S):
        assert isinstance(entry.options[key], int)
    schedule_reload.assert_called_once_with(entry.entry_id)


async def test_options_flow_reshows_the_submitted_values_after_an_error(
    hass: HomeAssistant, mqtt_setup: Any, make_config_entry: Any
) -> None:
    """A rejected prefix keeps what was typed, in the section too, instead of the stored values."""
    entry = make_config_entry()
    entry.mock_state(hass, ConfigEntryState.LOADED)
    result = await hass.config_entries.options.async_init(entry.entry_id)
    result = await hass.config_entries.options.async_configure(
        result["flow_id"], options_input(topic_prefix="/bad", ack_expiry_s=240.0)
    )
    assert result["errors"] == {CONF_TOPIC_PREFIX: ERROR_INVALID_PREFIX}
    schema = result["data_schema"]
    assert schema is not None
    assert _suggested(schema)[CONF_TOPIC_PREFIX] == "/bad"
    advanced = next(
        value
        for key, value in schema.schema.items()
        if isinstance(key, probatio.Marker) and key.schema == SECTION_ADVANCED
    )
    assert isinstance(advanced, section)
    assert _suggested(advanced.schema)[CONF_ACK_EXPIRY_S] == 240.0


async def test_options_flow_enforces_the_number_ranges(
    hass: HomeAssistant, mqtt_setup: Any, make_config_entry: Any
) -> None:
    entry = make_config_entry()
    entry.mock_state(hass, ConfigEntryState.LOADED)
    result = await hass.config_entries.options.async_init(entry.entry_id)
    for bad in (
        {CONF_ACK_TIMEOUT_S: 0.0},
        {CONF_HEARTBEAT_UNAVAILABLE_AFTER_S: 601.0},
        {CONF_COMMAND_MAX_AGE_MS: 600100.0},
        {CONF_INTENT_ACK_WAIT_S: 0.4},
        {CONF_JOINT_MOVING_THRESHOLD: -0.5},
        {CONF_JOINT_MOVING_HOLD_S: 30.5},
        {CONF_ACK_EXPIRY_S: 5.0},
        {CONF_RESULT_EXPIRY_S: 86401.0},
        {CONF_REPLAY_SETTLE_S: 0.0},
        {CONF_SESSION_CHANGE_GRACE_S: 10.5},
        {CONF_FEEDBACK_THROTTLE_MS: 5050.0},
        {CONF_HEARTBEAT_STALE_INTERVALS: 1.0},
        {CONF_CLOCK_SKEW_WARN_MS: 50.0},
        {CONF_DUPLICATE_SESSION_CHANGES: 0.0},
        {CONF_DUPLICATE_SESSION_WINDOW_S: 30.0},
        {CONF_DUPLICATE_QUIET_S: 3630.0},
    ):
        with pytest.raises(InvalidData):
            await hass.config_entries.options.async_configure(
                result["flow_id"], options_input(**bad)
            )
    # The widened maximum age is accepted, as is every other bound.
    with patch.object(hass.config_entries, "async_schedule_reload"):
        result = await hass.config_entries.options.async_configure(
            result["flow_id"],
            options_input(
                command_max_age_ms=600000.0,
                feedback_throttle_ms=0.0,
                session_change_grace_s=0.0,
                heartbeat_stale_intervals=20.0,
                joint_moving_threshold=0.0,
            ),
        )
    assert result["type"] is FlowResultType.CREATE_ENTRY
    assert entry.options[CONF_COMMAND_MAX_AGE_MS] == 600000
    assert entry.options[CONF_FEEDBACK_THROTTLE_MS] == 0
    assert entry.options[CONF_SESSION_CHANGE_GRACE_S] == 0.0


def test_normalise_options_casts_types() -> None:
    options = normalise_options(
        options_input(
            topic_prefix="lab ",
            shared_secret=None,
            mirror_entities=("sensor.a",),
            mirror_extra_attributes=[" pm2_5 ", "", "pm2_5", "moisture"],
        )
    )
    assert set(options) == set(DEFAULT_OPTIONS)
    assert options[CONF_TOPIC_PREFIX] == "lab"
    assert options[CONF_SHARED_SECRET] == ""
    assert options[CONF_MIRROR_ENTITIES] == ["sensor.a"]
    assert options[CONF_MIRROR_EXTRA_ATTRIBUTES] == ["pm2_5", "moisture"]
    for key in (
        CONF_COMMAND_MAX_AGE_MS,
        CONF_RESULT_TIMEOUT_MARGIN_S,
        CONF_MAX_PENDING_PER_ROBOT,
        CONF_HEARTBEAT_UNAVAILABLE_AFTER_S,
        CONF_ACK_EXPIRY_S,
        CONF_RESULT_EXPIRY_S,
        CONF_REPLAY_SETTLE_S,
        CONF_FEEDBACK_THROTTLE_MS,
        CONF_HEARTBEAT_STALE_INTERVALS,
        CONF_CLOCK_SKEW_WARN_MS,
        CONF_DUPLICATE_SESSION_CHANGES,
        CONF_DUPLICATE_SESSION_WINDOW_S,
        CONF_DUPLICATE_QUIET_S,
    ):
        assert type(options[key]) is int, key
    for key in (
        CONF_ACK_TIMEOUT_S,
        CONF_HEARTBEAT_INTERVAL_S,
        CONF_INTENT_ACK_WAIT_S,
        CONF_JOINT_MOVING_THRESHOLD,
        CONF_JOINT_MOVING_HOLD_S,
        CONF_SESSION_CHANGE_GRACE_S,
    ):
        assert type(options[key]) is float, key
    # A flat submission (no section) normalises the same way, and a single text is a list.
    flat = flatten_options(options_input(mirror_extra_attributes="pm2_5"))
    assert SECTION_ADVANCED not in flat
    assert normalise_options(flat)[CONF_MIRROR_EXTRA_ATTRIBUTES] == ["pm2_5"]
    assert flatten_options(nest_options(DEFAULT_OPTIONS)) == DEFAULT_OPTIONS


def test_every_default_is_what_the_hub_reads_and_the_section_lists_each_key_once() -> None:
    assert HubOptions.from_mapping({}) == HubOptions.from_mapping(DEFAULT_OPTIONS)
    assert set(ADVANCED_OPTIONS) < set(DEFAULT_OPTIONS)
    assert len(set(ADVANCED_OPTIONS)) == len(ADVANCED_OPTIONS)
    assert set(HubOptions.from_mapping({}).to_dict()) == (
        set(DEFAULT_OPTIONS) - {CONF_SHARED_SECRET}
    ) | {"secret_configured"}


async def test_options_reload_keeps_the_registry(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Any,
    complete_hub_start: Any,
) -> None:
    """A command outstanding before an options save is still tracked after the reload."""
    entry: MockConfigEntry = make_config_entry()
    assert await hass.config_entries.async_setup(entry.entry_id)
    await hass.async_block_till_done()
    hub: BridgeHub = entry.runtime_data
    await complete_hub_start(hub)
    fire_robot_message(hass, "announce", announce_payload(), retain=True)
    fire_robot_message(hass, "availability", "online", retain=True)
    await hass.async_block_till_done()
    command = await hub.async_send_task(
        ROBOT_ID, "reset", {}, source=CommandSource(kind="service")
    )
    assert hub.registry.get(command.task_id) is not None

    result = await hass.config_entries.options.async_init(entry.entry_id)
    result = await hass.config_entries.options.async_configure(
        result["flow_id"], options_input(ack_timeout_s=3.0, result_expiry_s=900.0)
    )
    assert result["type"] is FlowResultType.CREATE_ENTRY
    await hass.async_block_till_done()
    assert entry.state is ConfigEntryState.LOADED
    new_hub: BridgeHub = entry.runtime_data
    assert new_hub is not hub
    assert new_hub.options.ack_timeout_s == 3.0
    assert new_hub.options.result_expiry_s == 900.0
    assert new_hub.registry.settings.result_expiry_s == 900.0, "the parked registry is retuned"
    survivor = new_hub.registry.get(command.task_id)
    assert survivor is not None
    assert survivor.is_outstanding


# --------------------------------------------------------------------------- translations


def test_translations_mirror_strings_and_cover_the_flow_keys() -> None:
    strings = json.loads((INTEGRATION_DIR / "strings.json").read_text(encoding="utf-8"))
    english = json.loads(
        (INTEGRATION_DIR / "translations" / "en.json").read_text(encoding="utf-8")
    )
    assert strings == english
    assert "[%key:" not in json.dumps(strings)
    aborts = strings["config"]["abort"]
    for reason in (
        ABORT_MQTT_NOT_CONFIGURED,
        ABORT_MQTT_NOT_AVAILABLE,
        ABORT_INVALID_DISCOVERY_INFO,
        ABORT_SINGLE_INSTANCE,
        "already_configured",
    ):
        assert reason in aborts
    assert ERROR_INVALID_PREFIX in strings["config"]["error"]
    assert ERROR_INVALID_PREFIX in strings["options"]["error"]
    assert set(strings["config"]["step"]["user"]["data"]) == {
        CONF_TOPIC_PREFIX,
        CONF_SHARED_SECRET,
    }
    assert set(strings["config"]["step"]["confirm"]["data"]) == {CONF_SHARED_SECRET}
    main_keys = set(DEFAULT_OPTIONS) - set(ADVANCED_OPTIONS)
    init = strings["options"]["step"]["init"]
    assert set(init["data"]) == main_keys
    assert set(init["data_description"]) == main_keys
    advanced = init["sections"][SECTION_ADVANCED]
    assert set(advanced["data"]) == set(ADVANCED_OPTIONS)
    assert set(advanced["data_description"]) == set(ADVANCED_OPTIONS)
    assert "{quiet_minutes}" in strings["issues"]["duplicate_robot_id"]["description"]
    assert set(strings["exceptions"]) == set(EXCEPTION_KEYS)
    assert set(strings["issues"]) == set(ISSUE_KINDS)
    assert set(strings["selector"]["wait"]["options"]) == {"ack", "result", "none"}
    assert set(strings["services"]) == {"send_task", "cancel_task", "get_robots"}
