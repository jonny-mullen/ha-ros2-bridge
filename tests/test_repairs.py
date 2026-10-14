# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The five repair issues the hub raises: opened on evidence, closed when clear, translated.

``clock_skew``, ``hmac_rejected``, ``invalid_announce`` and
``duplicate_robot_id`` are raised per robot (issue id ``{kind}_{robot_id}``),
``mqtt_v5_required`` once for the broker. None is fixable; each is deleted the
moment its cause is gone, and every placeholder the hub passes is one the
strings file uses.
"""

from __future__ import annotations

import json
import re
import uuid
from collections.abc import AsyncIterator
from typing import Any
from unittest.mock import ANY, AsyncMock, patch

import pytest
from homeassistant.core import HomeAssistant
from homeassistant.exceptions import ServiceValidationError
from homeassistant.helpers import issue_registry as ir
from pytest_homeassistant_custom_component.common import MockConfigEntry

from custom_components.ros2_bridge.commands import CommandRegistry
from custom_components.ros2_bridge.const import (
    CONF_SHARED_SECRET,
    DEFAULT_DUPLICATE_QUIET_S,
    DOMAIN,
    ISSUE_CLOCK_SKEW,
    ISSUE_DUPLICATE_ROBOT_ID,
    ISSUE_HMAC_REJECTED,
    ISSUE_INVALID_ANNOUNCE,
    ISSUE_KINDS,
    ISSUE_MQTT_V5_REQUIRED,
    CommandState,
    cmd_topic,
    issue_id,
)
from custom_components.ros2_bridge.hub import BridgeHub
from custom_components.ros2_bridge.models import PendingCommand
from custom_components.ros2_bridge.protocol import CommandSource
from custom_components.ros2_bridge.store import CommandStore

from .common import (
    OTHER_SESSION,
    PREFIX,
    REPO_ROOT,
    RESET_ID,
    ROBOT_ID,
    TASK_ID,
    WAIT_ID,
    FakeClock,
    ack_payload,
    announce_payload,
    fire_robot_message,
    heartbeat_payload,
    rejected_ack_payload,
)

STRINGS_PATH = REPO_ROOT / "custom_components" / "ros2_bridge" / "strings.json"
#: A placeholder for tests only; it is not a credential.
PLACEHOLDER_SECRET = "placeholder-shared-secret"
#: The placeholders the hub supplies per issue kind (const.py documents them).
EXPECTED_PLACEHOLDERS: dict[str, set[str]] = {
    ISSUE_CLOCK_SKEW: {"robot_id", "skew_ms"},
    ISSUE_HMAC_REJECTED: {"robot_id", "reason"},
    ISSUE_INVALID_ANNOUNCE: {"robot_id", "topic", "reason"},
    ISSUE_DUPLICATE_ROBOT_ID: {"robot_id", "detail", "quiet_minutes"},
    ISSUE_MQTT_V5_REQUIRED: {"topic"},
}
PLACEHOLDER_RE = re.compile(r"\{([a-z_]+)\}")
COMMAND_IDS = [TASK_ID, RESET_ID, WAIT_ID]


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


async def announce_online(hass: HomeAssistant, **overrides: Any) -> None:
    """Deliver an announce and an online availability as live (non-retained) messages.

    Home Assistant hands a subscription a retained message only once per topic, so
    every message in this file is delivered live; the hub treats both alike.
    """
    fire_robot_message(hass, "announce", announce_payload(**overrides))
    fire_robot_message(hass, "availability", "online")
    await hass.async_block_till_done()


async def send(hub: BridgeHub, task_id: str, task: str = "reset") -> PendingCommand:
    """Send a parameterless task with a fixed command id."""
    with patch("custom_components.ros2_bridge.hub.uuid.uuid4", return_value=uuid.UUID(task_id)):
        return await hub.async_send_task(ROBOT_ID, task, {}, source=CommandSource(kind="service"))


def issue(hass: HomeAssistant, kind: str, robot_id: str | None = ROBOT_ID) -> ir.IssueEntry | None:
    return ir.async_get(hass).async_get_issue(DOMAIN, issue_id(kind, robot_id))


def assert_open(
    hass: HomeAssistant,
    hub: BridgeHub,
    kind: str,
    robot_id: str | None = ROBOT_ID,
    *,
    severity: ir.IssueSeverity,
) -> ir.IssueEntry:
    entry = issue(hass, kind, robot_id)
    assert entry is not None, f"{kind} is not open"
    assert issue_id(kind, robot_id) in hub.open_issues
    assert entry.translation_key == kind
    assert entry.is_fixable is False
    assert entry.is_persistent is False
    assert entry.severity is severity
    assert entry.translation_placeholders is not None
    assert set(entry.translation_placeholders) == EXPECTED_PLACEHOLDERS[kind]
    return entry


def assert_closed(
    hass: HomeAssistant, hub: BridgeHub, kind: str, robot_id: str | None = ROBOT_ID
) -> None:
    assert issue(hass, kind, robot_id) is None
    assert issue_id(kind, robot_id) not in hub.open_issues


# --------------------------------------------------------------------------- invalid_announce


async def test_invalid_announce_opens_and_a_valid_one_closes(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Any,
    clock: FakeClock,
    complete_hub_start: Any,
    started_hubs: list[BridgeHub],
) -> None:
    hub = await start_hub(hass, make_config_entry(), clock, complete_hub_start, started_hubs)
    fire_robot_message(hass, "announce", b"not json")
    await hass.async_block_till_done()
    entry = assert_open(hass, hub, ISSUE_INVALID_ANNOUNCE, severity=ir.IssueSeverity.WARNING)
    assert entry.translation_placeholders == {
        "robot_id": ROBOT_ID,
        "topic": f"{PREFIX}/{ROBOT_ID}/announce",
        "reason": entry.translation_placeholders["reason"],
    }
    assert "JSON" in entry.translation_placeholders["reason"]
    assert ROBOT_ID not in hub.robots

    # A payload whose robot_id differs from the topic is invalid too, with its own reason.
    fire_robot_message(hass, "announce", announce_payload(robot_id="arm_right"))
    await hass.async_block_till_done()
    entry = assert_open(hass, hub, ISSUE_INVALID_ANNOUNCE, severity=ir.IssueSeverity.WARNING)
    assert "differs" in entry.translation_placeholders["reason"]

    await announce_online(hass)
    assert_closed(hass, hub, ISSUE_INVALID_ANNOUNCE)
    assert ROBOT_ID in hub.robots


# --------------------------------------------------------------------------- hmac_rejected


async def test_hmac_required_without_a_secret_opens_and_closes(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Any,
    clock: FakeClock,
    complete_hub_start: Any,
    started_hubs: list[BridgeHub],
) -> None:
    hub = await start_hub(hass, make_config_entry(), clock, complete_hub_start, started_hubs)
    await announce_online(
        hass, capabilities={"cancel": True, "feedback": True, "hmac_required": True}
    )
    entry = assert_open(hass, hub, ISSUE_HMAC_REJECTED, severity=ir.IssueSeverity.ERROR)
    assert entry.translation_placeholders == {
        "robot_id": ROBOT_ID,
        "reason": "the robot requires signed commands",
    }
    await announce_online(hass)
    assert_closed(hass, hub, ISSUE_HMAC_REJECTED)


async def test_unauthorised_ack_with_a_secret_opens_and_an_accepted_ack_closes(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Any,
    clock: FakeClock,
    complete_hub_start: Any,
    started_hubs: list[BridgeHub],
) -> None:
    entry = make_config_entry(options={CONF_SHARED_SECRET: PLACEHOLDER_SECRET})
    hub = await start_hub(hass, entry, clock, complete_hub_start, started_hubs)
    await announce_online(hass)
    await send(hub, TASK_ID)
    fire_robot_message(
        hass,
        f"task/{TASK_ID}/ack",
        rejected_ack_payload(task="reset", code="rejected_unauthorised"),
    )
    await hass.async_block_till_done()
    opened = assert_open(hass, hub, ISSUE_HMAC_REJECTED, severity=ir.IssueSeverity.ERROR)
    assert opened.translation_placeholders == {
        "robot_id": ROBOT_ID,
        "reason": "the robot refused the signature (secret mismatch)",
    }
    assert hub.registry.get(TASK_ID).state is CommandState.REJECTED  # type: ignore[union-attr]

    await send(hub, RESET_ID)
    fire_robot_message(hass, f"task/{RESET_ID}/ack", ack_payload(id=RESET_ID, task="reset"))
    await hass.async_block_till_done()
    assert_closed(hass, hub, ISSUE_HMAC_REJECTED)


# --------------------------------------------------------------------------- clock_skew


async def test_clock_skew_opens_on_stale_and_closes_on_an_aligned_ack(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Any,
    clock: FakeClock,
    complete_hub_start: Any,
    started_hubs: list[BridgeHub],
) -> None:
    hub = await start_hub(hass, make_config_entry(), clock, complete_hub_start, started_hubs)
    await announce_online(hass)
    await send(hub, TASK_ID)
    fire_robot_message(
        hass,
        f"task/{TASK_ID}/ack",
        rejected_ack_payload(task="reset", code="rejected_stale", clock_skew_ms=5000),
    )
    await hass.async_block_till_done()
    opened = assert_open(hass, hub, ISSUE_CLOCK_SKEW, severity=ir.IssueSeverity.WARNING)
    assert opened.translation_placeholders == {"robot_id": ROBOT_ID, "skew_ms": "5000"}

    # An accepted ack with a large skew keeps it open; an aligned one closes it.
    await send(hub, RESET_ID)
    fire_robot_message(
        hass, f"task/{RESET_ID}/ack", ack_payload(id=RESET_ID, task="reset", clock_skew_ms=-1500)
    )
    await hass.async_block_till_done()
    opened = assert_open(hass, hub, ISSUE_CLOCK_SKEW, severity=ir.IssueSeverity.WARNING)
    assert opened.translation_placeholders["skew_ms"] == "-1500"

    await send(hub, WAIT_ID)
    fire_robot_message(
        hass, f"task/{WAIT_ID}/ack", ack_payload(id=WAIT_ID, task="reset", clock_skew_ms=12)
    )
    await hass.async_block_till_done()
    assert_closed(hass, hub, ISSUE_CLOCK_SKEW)


async def test_clock_skew_with_an_unknown_skew(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Any,
    clock: FakeClock,
    complete_hub_start: Any,
    started_hubs: list[BridgeHub],
) -> None:
    hub = await start_hub(hass, make_config_entry(), clock, complete_hub_start, started_hubs)
    await announce_online(hass)
    await send(hub, TASK_ID)
    fire_robot_message(
        hass,
        f"task/{TASK_ID}/ack",
        rejected_ack_payload(task="reset", code="rejected_future", clock_skew_ms=None),
    )
    await hass.async_block_till_done()
    opened = assert_open(hass, hub, ISSUE_CLOCK_SKEW, severity=ir.IssueSeverity.WARNING)
    assert opened.translation_placeholders["skew_ms"] == "unknown"


# --------------------------------------------------------------------------- duplicate_robot_id


async def test_duplicate_robot_id_from_a_second_instance(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Any,
    clock: FakeClock,
    complete_hub_start: Any,
    advance: Any,
    started_hubs: list[BridgeHub],
) -> None:
    hub = await start_hub(hass, make_config_entry(), clock, complete_hub_start, started_hubs)
    await announce_online(hass)
    assert_closed(hass, hub, ISSUE_DUPLICATE_ROBOT_ID)
    fire_robot_message(hass, "heartbeat", heartbeat_payload(instance="otherpc:99"))
    await hass.async_block_till_done()
    opened = assert_open(hass, hub, ISSUE_DUPLICATE_ROBOT_ID, severity=ir.IssueSeverity.ERROR)
    assert opened.translation_placeholders == {
        "robot_id": ROBOT_ID,
        "detail": "instances robotpc:4123 and otherpc:99",
        "quiet_minutes": "10",
    }
    assert ISSUE_DUPLICATE_ROBOT_ID in hub.problem_reasons(ROBOT_ID)

    # Ten quiet minutes close it from the housekeeping timer.
    clock.advance_s(DEFAULT_DUPLICATE_QUIET_S - 1)
    await advance(1)
    assert_open(hass, hub, ISSUE_DUPLICATE_ROBOT_ID, severity=ir.IssueSeverity.ERROR)
    clock.advance_s(1)
    await advance(1)
    assert_closed(hass, hub, ISSUE_DUPLICATE_ROBOT_ID)
    assert ISSUE_DUPLICATE_ROBOT_ID not in hub.problem_reasons(ROBOT_ID)


async def test_duplicate_robot_id_from_session_flapping(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Any,
    clock: FakeClock,
    complete_hub_start: Any,
    started_hubs: list[BridgeHub],
) -> None:
    hub = await start_hub(hass, make_config_entry(), clock, complete_hub_start, started_hubs)
    await announce_online(hass)
    sessions = [OTHER_SESSION, TASK_ID, RESET_ID, WAIT_ID]
    for session in sessions[:3]:
        clock.advance_s(10)
        fire_robot_message(hass, "announce", announce_payload(session=session))
        await hass.async_block_till_done()
    assert_closed(hass, hub, ISSUE_DUPLICATE_ROBOT_ID)
    clock.advance_s(10)
    fire_robot_message(hass, "announce", announce_payload(session=sessions[3]))
    await hass.async_block_till_done()
    opened = assert_open(hass, hub, ISSUE_DUPLICATE_ROBOT_ID, severity=ir.IssueSeverity.ERROR)
    assert opened.translation_placeholders["detail"] == "4 session changes in 5 minutes"


# --------------------------------------------------------------------------- mqtt_v5_required


async def test_mqtt_v5_required_opens_on_refused_expiry_and_closes_on_success(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Any,
    clock: FakeClock,
    complete_hub_start: Any,
    started_hubs: list[BridgeHub],
) -> None:
    hub = await start_hub(hass, make_config_entry(), clock, complete_hub_start, started_hubs)
    await announce_online(hass)

    async def refuse_expiry(*args: Any, **kwargs: Any) -> None:
        if kwargs.get("message_expiry_interval") is not None:
            raise ServiceValidationError("message expiry needs MQTT 5")

    with patch(
        "custom_components.ros2_bridge.hub.mqtt.async_publish",
        new=AsyncMock(side_effect=refuse_expiry),
    ) as publish:
        command = await send(hub, TASK_ID)
    assert publish.await_count == 2
    first, second = publish.await_args_list
    assert first.kwargs["message_expiry_interval"] == 5
    assert "message_expiry_interval" not in second.kwargs
    assert command.state is CommandState.SENT
    assert command.publish_failed is False
    opened = assert_open(
        hass, hub, ISSUE_MQTT_V5_REQUIRED, None, severity=ir.IssueSeverity.WARNING
    )
    assert opened.translation_placeholders == {"topic": cmd_topic(PREFIX, ROBOT_ID)}

    await send(hub, RESET_ID)
    mqtt_setup.async_publish.assert_called_with(
        cmd_topic(PREFIX, ROBOT_ID), ANY, 1, False, message_expiry_interval=5
    )
    assert_closed(hass, hub, ISSUE_MQTT_V5_REQUIRED, None)


# --------------------------------------------------------------------------- lifecycle


async def test_tombstone_closes_the_robot_issues_and_clear_removes_all(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Any,
    clock: FakeClock,
    complete_hub_start: Any,
    started_hubs: list[BridgeHub],
) -> None:
    hub = await start_hub(hass, make_config_entry(), clock, complete_hub_start, started_hubs)
    await announce_online(hass)
    fire_robot_message(hass, "heartbeat", heartbeat_payload(instance="otherpc:99"))
    await hass.async_block_till_done()
    assert_open(hass, hub, ISSUE_DUPLICATE_ROBOT_ID, severity=ir.IssueSeverity.ERROR)
    hub._open_issue(ISSUE_MQTT_V5_REQUIRED, None, {"topic": cmd_topic(PREFIX, ROBOT_ID)})

    fire_robot_message(hass, "announce", b"")
    await hass.async_block_till_done()
    assert_closed(hass, hub, ISSUE_DUPLICATE_ROBOT_ID)
    assert ROBOT_ID not in hub.robots
    assert hub.open_issues == {issue_id(ISSUE_MQTT_V5_REQUIRED)}

    hub.async_clear_issues()
    assert hub.open_issues == frozenset()
    assert issue(hass, ISSUE_MQTT_V5_REQUIRED, None) is None


async def test_async_forget_robot_closes_its_issues(
    hass: HomeAssistant,
    mqtt_setup: Any,
    make_config_entry: Any,
    clock: FakeClock,
    complete_hub_start: Any,
    started_hubs: list[BridgeHub],
) -> None:
    hub = await start_hub(hass, make_config_entry(), clock, complete_hub_start, started_hubs)
    await announce_online(
        hass, capabilities={"cancel": True, "feedback": True, "hmac_required": True}
    )
    assert_open(hass, hub, ISSUE_HMAC_REJECTED, severity=ir.IssueSeverity.ERROR)
    fire_robot_message(hass, "availability", "offline")
    await hass.async_block_till_done()
    assert hub.is_removable(ROBOT_ID)
    await hub.async_forget_robot(ROBOT_ID)
    assert_closed(hass, hub, ISSUE_HMAC_REJECTED)
    assert hub.open_issues == frozenset()


# --------------------------------------------------------------------------- translations


@pytest.mark.parametrize("kind", ISSUE_KINDS)
def test_issue_strings_use_only_the_supplied_placeholders(kind: str) -> None:
    strings = json.loads(STRINGS_PATH.read_text(encoding="utf-8"))
    text = strings["issues"][kind]
    used = set(PLACEHOLDER_RE.findall(text["title"] + " " + text["description"]))
    assert used <= EXPECTED_PLACEHOLDERS[kind], f"{kind} uses placeholders the hub does not pass"
    assert "robot_id" in used or kind == ISSUE_MQTT_V5_REQUIRED
    assert "closes by itself" in text["description"]


def test_issue_ids() -> None:
    assert issue_id(ISSUE_MQTT_V5_REQUIRED) == "mqtt_v5_required"
    assert issue_id(ISSUE_CLOCK_SKEW, ROBOT_ID) == f"clock_skew_{ROBOT_ID}"
    assert len(ISSUE_KINDS) == 5
