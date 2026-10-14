# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The ``send_task``, ``cancel_task`` and ``get_robots`` services.

Also home of :class:`HubRig` and the ``rig`` fixture, a started hub on the
mocked MQTT client with the services and intents registered, which
``test_intent.py`` and ``test_sensors_out.py`` import.
"""

from __future__ import annotations

import ast
import asyncio
import inspect
import json
from collections.abc import AsyncIterator, Awaitable, Callable
from dataclasses import dataclass
from types import ModuleType
from typing import Any
from unittest.mock import ANY, DEFAULT, MagicMock, call

import probatio
import pytest
from homeassistant.config_entries import ConfigEntryState
from homeassistant.core import Context, HomeAssistant
from homeassistant.exceptions import HomeAssistantError, ServiceValidationError
from homeassistant.helpers import device_registry as dr
from pytest_homeassistant_custom_component.common import MockConfigEntry

from custom_components.ros2_bridge import protocol, services
from custom_components.ros2_bridge.commands import CommandRegistry
from custom_components.ros2_bridge.const import (
    CONF_HEARTBEAT_INTERVAL_S,
    CONF_MAX_PENDING_PER_ROBOT,
    CONF_SHARED_SECRET,
    DOMAIN,
    ISSUE_MQTT_V5_REQUIRED,
    issue_id,
)
from custom_components.ros2_bridge.hub import BridgeHub, default_clock
from custom_components.ros2_bridge.intent import async_register_intents
from custom_components.ros2_bridge.services import (
    ATTR_DEVICE_ID,
    ATTR_MAX_AGE_MS,
    ATTR_PARAMS,
    ATTR_ROBOT_ID,
    ATTR_TASK,
    ATTR_TASK_ID,
    ATTR_TIMEOUT_S,
    ATTR_WAIT,
    DATA_HUB,
    SERVICE_CANCEL_TASK,
    SERVICE_GET_ROBOTS,
    SERVICE_SEND_TASK,
    WAIT_ACK,
    WAIT_NONE,
    WAIT_RESULT,
    async_get_hub,
    async_register_services,
)
from custom_components.ros2_bridge.store import CommandStore

from .common import (
    OTHER_SESSION,
    PREFIX,
    ROBOT_ID,
    VECTOR_SECRET,
    announce_payload,
    cancel_ack_payload,
    fire_robot_message,
    idle_state_payload,
    rejected_ack_payload,
    result_payload,
    robot_topic,
    validate,
)
from .common import ack_payload as golden_ack_payload

OTHER_ROBOT = "arm_right"
CMD_TOPIC = robot_topic("cmd")


# --------------------------------------------------------------------------- the rig


@dataclass
class HubRig:
    """A started hub with its entry, store and the MQTT mock, plus message helpers."""

    hass: HomeAssistant
    entry: MockConfigEntry
    hub: BridgeHub
    store: CommandStore
    mqtt: MagicMock

    async def announce(self, robot_id: str = ROBOT_ID, **overrides: Any) -> None:
        """Deliver an announce, ``online`` and an idle state for a robot.

        They are delivered as live messages: HA's MQTT client drops a second
        retained message on a topic it has already delivered one for, which
        a real broker never does for a live publish.
        """
        extra: dict[str, Any] = {"robot_id": robot_id}
        if robot_id != ROBOT_ID:
            extra.update(
                {
                    "name": robot_id.replace("_", " ").capitalize(),
                    "session": OTHER_SESSION,
                    "instance": "robotpc:4124",
                }
            )
        extra.update(overrides)
        fire_robot_message(self.hass, "announce", announce_payload(**extra), robot_id=robot_id)
        fire_robot_message(self.hass, "availability", "online", robot_id=robot_id)
        identity = {key: extra[key] for key in ("robot_id", "session", "instance") if key in extra}
        fire_robot_message(self.hass, "state", idle_state_payload(**identity), robot_id=robot_id)
        await self.hass.async_block_till_done()

    def publishes(self, topic: str) -> list[Any]:
        """The ``async_publish`` calls made on ``topic``."""
        return [c for c in self.mqtt.async_publish.call_args_list if c.args[0] == topic]

    def commands(self, robot_id: str = ROBOT_ID) -> list[dict[str, Any]]:
        """Every command published for a robot, decoded, oldest first."""
        return [
            json.loads(c.args[1]) for c in self.publishes(robot_topic("cmd", robot_id=robot_id))
        ]

    async def until_published(self, topic: str, count: int = 1) -> None:
        """Yield to the loop until ``count`` commands on ``topic`` are published and marked sent.

        The publish call is recorded before HA's client sees the PUBACK, and the
        registry marks the command ``sent`` (arming its deadlines) only after that,
        so both are awaited before a test advances time or answers.
        """
        for _ in range(500):
            calls = self.publishes(topic)
            if len(calls) >= count and all(self._marked_sent(c.args[1]) for c in calls):
                return
            await asyncio.sleep(0)
        raise AssertionError(f"{count} publish(es) on {topic} did not happen")

    def _marked_sent(self, raw: Any) -> bool:
        command = self.hub.registry.get(json.loads(raw)["id"])
        return command is None or command.sent_at is not None

    @staticmethod
    def _identity(robot_id: str) -> dict[str, Any]:
        """The ``robot_id`` and ``session`` a reply from that robot carries."""
        if robot_id == ROBOT_ID:
            return {"robot_id": robot_id}
        return {"robot_id": robot_id, "session": OTHER_SESSION}

    def ack(
        self,
        command_id: str,
        *,
        accepted: bool = True,
        task: str | None = "reset",
        command_type: str = "task",
        retained: bool = False,
        robot_id: str = ROBOT_ID,
        **overrides: Any,
    ) -> None:
        """Deliver an ack for a command id (accepted, ``rejected_busy`` or a cancel's)."""
        fields = {**self._identity(robot_id), **overrides}
        if command_type == "cancel":
            payload = cancel_ack_payload(id=command_id, **fields)
            if not accepted:
                payload.update(
                    {
                        "accepted": False,
                        "code": "rejected_no_such_task",
                        "reason": "Nothing running",
                    }
                )
        elif accepted:
            payload = golden_ack_payload(id=command_id, task=task, **fields)
        else:
            payload = rejected_ack_payload(id=command_id, task=task, **fields)
        fire_robot_message(
            self.hass, f"task/{command_id}/ack", payload, robot_id=robot_id, retain=retained
        )

    def result(
        self,
        command_id: str,
        *,
        status: str = "succeeded",
        task: str = "reset",
        retained: bool = False,
        robot_id: str = ROBOT_ID,
        **overrides: Any,
    ) -> None:
        """Deliver a result for a command id."""
        goal = {"succeeded": 4, "cancelled": 5, "aborted": 6}[status]
        payload = result_payload(
            id=command_id,
            task=task,
            status=status,
            goal_status=goal,
            started_at=self.hub.now - 1500,
            finished_at=self.hub.now,
            **self._identity(robot_id),
            **overrides,
        )
        fire_robot_message(
            self.hass, f"task/{command_id}/result", payload, robot_id=robot_id, retain=retained
        )

    def device_id(self, robot_id: str = ROBOT_ID) -> str:
        """The device registry id of a robot's device."""
        device = dr.async_get(self.hass).async_get_device_by_identifier(
            (DOMAIN, robot_id), self.entry.entry_id
        )
        assert device is not None, robot_id
        return device.id

    def unload(self) -> None:
        """Pretend the entry was unloaded: no hub is findable any more."""
        self.hass.data[DOMAIN].pop(DATA_HUB, None)


@pytest.fixture(name="rig_options")
def default_rig_options() -> dict[str, Any]:
    """Options merged into the rig's config entry; override with ``parametrize``."""
    return {}


@pytest.fixture(name="rig")
async def hub_rig(
    hass: HomeAssistant,
    mqtt_setup: MagicMock,
    make_config_entry: Callable[..., MockConfigEntry],
    complete_hub_start: Callable[[BridgeHub], Awaitable[None]],
    rig_options: dict[str, Any],
) -> AsyncIterator[HubRig]:
    """A started, ready and reconciled hub with the services and intents registered."""
    entry = make_config_entry(options={CONF_HEARTBEAT_INTERVAL_S: 0, **rig_options})
    store = CommandStore(hass)
    registry = CommandRegistry(clock=default_clock)
    registry.load(await store.async_load())
    store.bind(registry)
    hub = BridgeHub(hass, entry, registry, store)
    await hub.async_start()
    await complete_hub_start(hub)
    hass.data.setdefault(DOMAIN, {})[DATA_HUB] = hub
    async_register_services(hass)
    async_register_intents(hass)
    mqtt_setup.async_publish.reset_mock()
    yield HubRig(hass, entry, hub, store, mqtt_setup)
    await hub.async_stop()
    hass.data[DOMAIN].pop(DATA_HUB, None)


async def call_service(
    hass: HomeAssistant, service: str, data: dict[str, Any], *, response: bool = True
) -> Any:
    """Call one of the integration's services, blocking, with or without a response."""
    return await hass.services.async_call(
        DOMAIN,
        service,
        data,
        blocking=True,
        return_response=response,
        context=Context(user_id="user-1"),
    )


def start_call(
    hass: HomeAssistant, service: str, data: dict[str, Any], *, response: bool = True
) -> asyncio.Task[Any]:
    """Start a service call that will block until the robot answers."""
    return hass.async_create_task(call_service(hass, service, data, response=response))


# --------------------------------------------------------------------------- schemas


def test_schemas_are_probatio_objects_typed_as_vol_schema_type() -> None:
    annotations = inspect.get_annotations(services)
    for name in ("SEND_TASK_SCHEMA", "CANCEL_TASK_SCHEMA", "GET_ROBOTS_SCHEMA"):
        assert annotations[name] == "VolSchemaType"
        schema = getattr(services, name)
        assert isinstance(schema, (probatio.Schema, probatio.All, probatio.Any))
    source = inspect.getsource(services)
    tree = ast.parse(source)
    imported = {
        alias.name.partition(".")[0]
        for node in ast.walk(tree)
        if isinstance(node, ast.Import)
        for alias in node.names
    } | {
        (node.module or "").partition(".")[0]
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom)
    }
    assert "voluptuous" not in imported
    assert "probatio as vol" not in source


def test_send_task_schema_rules() -> None:
    schema = services.SEND_TASK_SCHEMA
    with pytest.raises(probatio.Invalid):
        schema({ATTR_TASK: "reset"})  # neither device_id nor robot_id
    with pytest.raises(probatio.Invalid):
        schema({ATTR_ROBOT_ID: ROBOT_ID})  # task is required
    with pytest.raises(probatio.Invalid):
        schema({ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "reset", ATTR_WAIT: "later"})
    with pytest.raises(probatio.Invalid):
        schema({ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "reset", ATTR_TIMEOUT_S: 0})
    data = schema({ATTR_DEVICE_ID: "abc", ATTR_TASK: "reset", ATTR_TIMEOUT_S: "5"})
    assert data[ATTR_DEVICE_ID] == ["abc"]
    assert data[ATTR_WAIT] == WAIT_ACK
    assert data[ATTR_TIMEOUT_S] == 5.0
    cancel = services.CANCEL_TASK_SCHEMA({ATTR_ROBOT_ID: ROBOT_ID})
    assert cancel[ATTR_WAIT] == WAIT_ACK
    with pytest.raises(probatio.Invalid):
        services.CANCEL_TASK_SCHEMA({ATTR_TASK_ID: "x"})
    assert services.GET_ROBOTS_SCHEMA({}) == {}


async def test_services_register_once(hass: HomeAssistant) -> None:
    async_register_services(hass)
    async_register_services(hass)
    for service in (SERVICE_SEND_TASK, SERVICE_CANCEL_TASK, SERVICE_GET_ROBOTS):
        assert hass.services.has_service(DOMAIN, service)
    assert hass.services.supports_response(DOMAIN, SERVICE_GET_ROBOTS).value == "only"
    assert hass.services.supports_response(DOMAIN, SERVICE_SEND_TASK).value == "optional"


# --------------------------------------------------------------------------- send_task


@pytest.mark.parametrize("rig_options", [{CONF_SHARED_SECRET: VECTOR_SECRET}])
async def test_send_task_publishes_a_valid_signed_command(
    rig: HubRig, ros_protocol: ModuleType
) -> None:
    await rig.announce()
    response = await call_service(
        rig.hass,
        SERVICE_SEND_TASK,
        {
            ATTR_ROBOT_ID: ROBOT_ID,
            ATTR_TASK: "move_to",
            ATTR_PARAMS: {"pose": "home"},
            ATTR_WAIT: WAIT_NONE,
        },
    )
    rig.mqtt.async_publish.assert_called_once_with(
        CMD_TOPIC, ANY, 1, False, message_expiry_interval=5
    )
    payload = rig.commands()[0]
    assert validate("command", payload) == []
    assert payload["type"] == "task" and payload["task"] == "move_to"
    assert payload["params"] == {"pose": "home"}
    assert payload["timeout_s"] is None
    assert payload["source"]["kind"] == "service"
    assert payload["source"]["user_id"] == "user-1"
    assert payload["source"]["context_id"]
    assert payload["source"]["instance_id"] == rig.hub.instance_id
    assert payload["sig"] == protocol.compute_sig(VECTOR_SECRET, CMD_TOPIC, payload)
    assert ros_protocol.verify_sig(VECTOR_SECRET, CMD_TOPIC, payload)
    assert not ros_protocol.verify_sig(
        VECTOR_SECRET, robot_topic("cmd", robot_id=OTHER_ROBOT), payload
    )
    (entry,) = response["tasks"]
    assert entry["task_id"] == payload["id"]
    assert entry["state"] == "sent" and entry["accepted"] is None and entry["result"] is None


async def test_send_task_timeout_and_max_age_travel(rig: HubRig) -> None:
    await rig.announce()
    await call_service(
        rig.hass,
        SERVICE_SEND_TASK,
        {
            ATTR_ROBOT_ID: ROBOT_ID,
            ATTR_TASK: "reset",
            ATTR_WAIT: WAIT_NONE,
            ATTR_TIMEOUT_S: 7.5,
            ATTR_MAX_AGE_MS: 12000,
        },
        response=False,
    )
    rig.mqtt.async_publish.assert_called_once_with(
        CMD_TOPIC, ANY, 1, False, message_expiry_interval=12
    )
    payload = rig.commands()[0]
    assert payload["timeout_s"] == 7.5 and payload["max_age_ms"] == 12000
    assert "sig" not in payload
    command = rig.hub.registry.get(payload["id"])
    assert command is not None and command.timeout_s == 7.5


async def test_wait_ack_reports_the_accepted_ack(rig: HubRig) -> None:
    await rig.announce()
    pending = start_call(
        rig.hass, SERVICE_SEND_TASK, {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "reset"}
    )
    await rig.until_published(CMD_TOPIC)
    command_id = rig.commands()[0]["id"]
    rig.ack(command_id)
    response = await pending
    (entry,) = response["tasks"]
    assert entry == {
        "robot_id": ROBOT_ID,
        "task_id": command_id,
        "task": "reset",
        "kind": "motion",
        "state": "running",
        "accepted": True,
        "code": "accepted",
        "reason": "",
        "clock_skew_ms": 12,
        "result": None,
    }


async def test_wait_result_reports_the_result(rig: HubRig) -> None:
    await rig.announce()
    pending = start_call(
        rig.hass,
        SERVICE_SEND_TASK,
        {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "reset", ATTR_WAIT: WAIT_RESULT},
    )
    await rig.until_published(CMD_TOPIC)
    command_id = rig.commands()[0]["id"]
    rig.ack(command_id)
    await asyncio.sleep(0)
    assert not pending.done()
    rig.result(command_id, data={"moveit_error_code": 1})
    response = await pending
    (entry,) = response["tasks"]
    assert entry["state"] == "succeeded" and entry["accepted"] is True
    result = entry["result"]
    assert result["status"] == "succeeded" and result["goal_status"] == 4
    assert result["code"] == "ok" and result["data"] == {"moveit_error_code": 1}
    assert result["duration_ms"] == 1500 and result["synthetic"] is False
    assert result["task_id"] == command_id and result["task"] == "reset"


async def test_wait_result_without_response_does_not_raise_on_abort(rig: HubRig) -> None:
    await rig.announce()
    pending = start_call(
        rig.hass,
        SERVICE_SEND_TASK,
        {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "reset", ATTR_WAIT: WAIT_RESULT},
        response=False,
    )
    await rig.until_published(CMD_TOPIC)
    command_id = rig.commands()[0]["id"]
    rig.ack(command_id)
    rig.result(command_id, status="aborted", code="collision")
    assert await pending is None


async def test_rejection_raises_without_a_response_and_reports_with_one(rig: HubRig) -> None:
    await rig.announce()
    pending = start_call(
        rig.hass, SERVICE_SEND_TASK, {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "reset"}
    )
    await rig.until_published(CMD_TOPIC)
    rig.ack(rig.commands()[0]["id"], accepted=False)
    (entry,) = (await pending)["tasks"]
    assert entry["state"] == "rejected" and entry["accepted"] is False
    assert entry["code"] == "rejected_busy" and entry["reason"].startswith("Task 'reset'")

    pending = start_call(
        rig.hass, SERVICE_SEND_TASK, {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "reset"}, response=False
    )
    await rig.until_published(CMD_TOPIC, 2)
    rig.ack(rig.commands()[1]["id"], accepted=False)
    with pytest.raises(HomeAssistantError) as excinfo:
        await pending
    assert excinfo.value.translation_key == "task_rejected"
    assert excinfo.value.translation_domain == DOMAIN
    assert excinfo.value.translation_placeholders == {
        "robot_id": ROBOT_ID,
        "task": "reset",
        "code": "rejected_busy",
        "reason": "Task 'reset' (id 51f0a2b3-c4d5-4e6f-a7b8-c9d0e1f2a3b4) is running",
    }


async def test_no_ack(rig: HubRig, advance: Callable[[float], Awaitable[None]]) -> None:
    await rig.announce()
    pending = start_call(
        rig.hass, SERVICE_SEND_TASK, {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "reset"}, response=False
    )
    await rig.until_published(CMD_TOPIC)
    await advance(3)
    with pytest.raises(HomeAssistantError) as excinfo:
        await pending
    assert excinfo.value.translation_key == "no_ack"
    assert excinfo.value.translation_placeholders == {"robot_id": ROBOT_ID, "task": "reset"}

    pending = start_call(
        rig.hass, SERVICE_SEND_TASK, {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "reset"}
    )
    await rig.until_published(CMD_TOPIC, 2)
    await advance(3)
    (entry,) = (await pending)["tasks"]
    assert entry["state"] == "timeout_no_ack" and entry["accepted"] is None
    assert entry["code"] == "no_ack"


async def test_no_result(rig: HubRig, advance: Callable[[float], Awaitable[None]]) -> None:
    await rig.announce()
    pending = start_call(
        rig.hass,
        SERVICE_SEND_TASK,
        {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "stop", ATTR_WAIT: WAIT_RESULT},
        response=False,
    )
    await rig.until_published(CMD_TOPIC)
    rig.ack(rig.commands()[0]["id"], task="stop")
    await advance(5 + 10 + 1)  # announced timeout plus the margin
    with pytest.raises(HomeAssistantError) as excinfo:
        await pending
    assert excinfo.value.translation_key == "no_result"
    assert excinfo.value.translation_placeholders == {"robot_id": ROBOT_ID, "task": "stop"}


@pytest.mark.parametrize(
    ("data", "key", "placeholders"),
    [
        (
            {ATTR_ROBOT_ID: "arm_ghost", ATTR_TASK: "reset"},
            "robot_unknown",
            {"robot_id": "arm_ghost"},
        ),
        ({ATTR_DEVICE_ID: "not-a-device"}, "robot_unknown", {"robot_id": "not-a-device"}),
        (
            {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "dance"},
            "task_unknown",
            {"robot_id": ROBOT_ID, "task": "dance"},
        ),
        (
            {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "move_to"},
            "invalid_params",
            {"robot_id": ROBOT_ID, "task": "move_to", "problems": "'pose' is required"},
        ),
    ],
)
async def test_validation_errors(
    rig: HubRig, data: dict[str, Any], key: str, placeholders: dict[str, str]
) -> None:
    await rig.announce()
    data = {ATTR_TASK: "reset", **data}
    with pytest.raises(ServiceValidationError) as excinfo:
        await call_service(rig.hass, SERVICE_SEND_TASK, data)
    assert excinfo.value.translation_domain == DOMAIN
    assert excinfo.value.translation_key == key
    assert excinfo.value.translation_placeholders == placeholders
    rig.mqtt.async_publish.assert_not_called()


async def test_robot_unavailable(rig: HubRig) -> None:
    await rig.announce()
    fire_robot_message(rig.hass, "availability", "offline")
    await rig.hass.async_block_till_done()
    with pytest.raises(ServiceValidationError) as excinfo:
        await call_service(
            rig.hass, SERVICE_SEND_TASK, {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "reset"}
        )
    assert excinfo.value.translation_key == "robot_unavailable"
    assert excinfo.value.translation_placeholders == {"robot_id": ROBOT_ID}


async def test_oversize_params_are_refused_before_anything_is_tracked(rig: HubRig) -> None:
    """A command over 16 KiB once encoded never enters the registry or the broker."""
    await rig.announce()
    with pytest.raises(ServiceValidationError) as excinfo:
        await call_service(
            rig.hass,
            SERVICE_SEND_TASK,
            {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "reset", ATTR_PARAMS: {"blob": "x" * 16384}},
        )
    assert excinfo.value.translation_key == "invalid_params"
    assert excinfo.value.translation_placeholders["robot_id"] == ROBOT_ID
    assert excinfo.value.translation_placeholders["task"] == "reset"
    assert "over the limit" in excinfo.value.translation_placeholders["problems"]
    assert rig.hub.registry.outstanding(ROBOT_ID) == []
    assert rig.hub.registry.commands == {}
    rig.mqtt.async_publish.assert_not_called()
    # The pending cap is untouched: the next command goes out.
    await call_service(
        rig.hass,
        SERVICE_SEND_TASK,
        {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "reset", ATTR_WAIT: WAIT_NONE},
    )
    assert len(rig.commands()) == 1


@pytest.mark.parametrize("rig_options", [{CONF_MAX_PENDING_PER_ROBOT: 1}])
async def test_too_many_pending(rig: HubRig) -> None:
    await rig.announce()
    await call_service(
        rig.hass,
        SERVICE_SEND_TASK,
        {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "reset", ATTR_WAIT: WAIT_NONE},
    )
    with pytest.raises(ServiceValidationError) as excinfo:
        await call_service(
            rig.hass, SERVICE_SEND_TASK, {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "stop"}
        )
    assert excinfo.value.translation_key == "too_many_pending"
    assert excinfo.value.translation_placeholders == {"robot_id": ROBOT_ID, "limit": "1"}


async def test_not_ready(rig: HubRig) -> None:
    await rig.announce()
    rig.hub.ready = False
    with pytest.raises(HomeAssistantError) as excinfo:
        await call_service(
            rig.hass, SERVICE_SEND_TASK, {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "reset"}
        )
    assert excinfo.value.translation_key == "not_ready"
    assert not isinstance(excinfo.value, ServiceValidationError)


async def test_publish_failed_keeps_the_command_sent(rig: HubRig) -> None:
    await rig.announce()
    rig.mqtt.async_publish.side_effect = HomeAssistantError("broker gone")
    with pytest.raises(HomeAssistantError) as excinfo:
        await call_service(
            rig.hass,
            SERVICE_SEND_TASK,
            {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "reset", ATTR_WAIT: WAIT_NONE},
        )
    assert excinfo.value.translation_key == "publish_failed"
    assert excinfo.value.translation_placeholders == {"robot_id": ROBOT_ID, "error": "broker gone"}
    (command,) = rig.hub.registry.outstanding(ROBOT_ID)
    assert command.state.value == "sent" and command.publish_failed is True
    rig.mqtt.async_publish.side_effect = None


async def test_not_loaded_without_a_hub(hass: HomeAssistant) -> None:
    async_register_services(hass)
    for service, data in (
        (SERVICE_SEND_TASK, {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "reset"}),
        (SERVICE_CANCEL_TASK, {ATTR_ROBOT_ID: ROBOT_ID}),
        (SERVICE_GET_ROBOTS, {}),
    ):
        with pytest.raises(HomeAssistantError) as excinfo:
            await call_service(hass, service, data)
        assert excinfo.value.translation_key == "not_loaded"
        assert excinfo.value.translation_domain == DOMAIN
    assert async_get_hub(hass) is None


async def test_hub_lookup_falls_back_to_the_loaded_entry(rig: HubRig) -> None:
    await rig.announce()
    rig.unload()
    assert async_get_hub(rig.hass) is None
    rig.entry.runtime_data = rig.hub
    rig.entry.mock_state(rig.hass, ConfigEntryState.LOADED)
    try:
        assert async_get_hub(rig.hass) is rig.hub
        response = await call_service(rig.hass, SERVICE_GET_ROBOTS, {})
        assert [robot["robot_id"] for robot in response["robots"]] == [ROBOT_ID]
    finally:
        rig.entry.mock_state(rig.hass, ConfigEntryState.NOT_LOADED)
        del rig.entry.runtime_data


async def test_unload_while_waiting_raises_not_loaded(rig: HubRig) -> None:
    """Stopping the hub cancels the registry's waiters; the caller learns, the command stays."""
    await rig.announce()
    pending = start_call(
        rig.hass, SERVICE_SEND_TASK, {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "reset"}
    )
    await rig.until_published(CMD_TOPIC)
    for _ in range(5):  # let the gathered wait reach the registry before the hub stops
        await asyncio.sleep(0)
    await rig.hub.async_stop()
    with pytest.raises(HomeAssistantError) as excinfo:
        await pending
    assert excinfo.value.translation_key == "not_loaded"
    assert [c.state.value for c in rig.hub.registry.outstanding(ROBOT_ID)] == ["sent"]


async def test_stopped_hub_is_not_loaded(rig: HubRig) -> None:
    await rig.hub.async_stop()
    with pytest.raises(HomeAssistantError) as excinfo:
        await call_service(rig.hass, SERVICE_GET_ROBOTS, {})
    assert excinfo.value.translation_key == "not_loaded"


# --------------------------------------------------------------------------- targets


async def test_multi_device_target(rig: HubRig) -> None:
    await rig.announce()
    await rig.announce(OTHER_ROBOT)
    response = await call_service(
        rig.hass,
        SERVICE_SEND_TASK,
        {
            ATTR_DEVICE_ID: [rig.device_id(ROBOT_ID), rig.device_id(OTHER_ROBOT)],
            ATTR_ROBOT_ID: ROBOT_ID,  # a duplicate of the first device: sent once
            ATTR_TASK: "reset",
            ATTR_WAIT: WAIT_NONE,
        },
    )
    assert [entry["robot_id"] for entry in response["tasks"]] == [ROBOT_ID, OTHER_ROBOT]
    assert len(rig.commands(ROBOT_ID)) == 1 and len(rig.commands(OTHER_ROBOT)) == 1
    assert rig.commands(OTHER_ROBOT)[0]["robot_id"] == OTHER_ROBOT
    assert rig.mqtt.async_publish.call_args_list == [
        call(robot_topic("cmd", robot_id=ROBOT_ID), ANY, 1, False, message_expiry_interval=5),
        call(robot_topic("cmd", robot_id=OTHER_ROBOT), ANY, 1, False, message_expiry_interval=5),
    ]


async def test_multi_target_validation_happens_before_any_publish(rig: HubRig) -> None:
    """A validation failure on the second robot leaves nothing sent to the first."""
    await rig.announce()
    without_reset = [task for task in announce_payload()["tasks"] if task["name"] != "reset"]
    await rig.announce(OTHER_ROBOT, tasks=without_reset)
    with pytest.raises(ServiceValidationError) as excinfo:
        await call_service(
            rig.hass,
            SERVICE_SEND_TASK,
            {
                ATTR_DEVICE_ID: [rig.device_id(ROBOT_ID), rig.device_id(OTHER_ROBOT)],
                ATTR_TASK: "reset",
                ATTR_WAIT: WAIT_NONE,
            },
        )
    assert excinfo.value.translation_key == "task_unknown"
    assert excinfo.value.translation_placeholders == {"robot_id": OTHER_ROBOT, "task": "reset"}
    rig.mqtt.async_publish.assert_not_called()
    assert rig.hub.registry.outstanding() == []

    fire_robot_message(rig.hass, "availability", "offline", robot_id=OTHER_ROBOT)
    await rig.hass.async_block_till_done()
    with pytest.raises(ServiceValidationError) as excinfo:
        await call_service(
            rig.hass,
            SERVICE_CANCEL_TASK,
            {
                ATTR_DEVICE_ID: [rig.device_id(ROBOT_ID), rig.device_id(OTHER_ROBOT)],
                ATTR_WAIT: WAIT_NONE,
            },
        )
    assert excinfo.value.translation_key == "robot_unavailable"
    assert excinfo.value.translation_placeholders == {"robot_id": OTHER_ROBOT}
    rig.mqtt.async_publish.assert_not_called()


async def test_multi_target_wait_ack_waits_for_every_robot(rig: HubRig) -> None:
    await rig.announce()
    await rig.announce(OTHER_ROBOT)
    pending = start_call(
        rig.hass,
        SERVICE_SEND_TASK,
        {
            ATTR_DEVICE_ID: [rig.device_id(ROBOT_ID), rig.device_id(OTHER_ROBOT)],
            ATTR_TASK: "reset",
        },
        response=False,
    )
    await rig.until_published(robot_topic("cmd", robot_id=OTHER_ROBOT))
    rig.ack(rig.commands(ROBOT_ID)[0]["id"])
    await asyncio.sleep(0)
    assert not pending.done()
    rig.ack(rig.commands(OTHER_ROBOT)[0]["id"], robot_id=OTHER_ROBOT)
    assert await pending is None


# --------------------------------------------------------------------------- cancel_task


async def test_cancel_task_wait_result_awaits_the_target(rig: HubRig) -> None:
    await rig.announce()
    sent = await call_service(
        rig.hass,
        SERVICE_SEND_TASK,
        {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "reset", ATTR_WAIT: WAIT_NONE},
    )
    task_id = sent["tasks"][0]["task_id"]
    rig.ack(task_id)
    await rig.hass.async_block_till_done()

    pending = start_call(
        rig.hass, SERVICE_CANCEL_TASK, {ATTR_ROBOT_ID: ROBOT_ID, ATTR_WAIT: WAIT_RESULT}
    )
    await rig.until_published(CMD_TOPIC, 2)
    cancel = rig.commands()[1]
    assert validate("command", cancel) == []
    assert cancel["type"] == "cancel" and cancel["target_id"] is None
    rig.mqtt.async_publish.assert_called_with(CMD_TOPIC, ANY, 1, False, message_expiry_interval=5)
    rig.ack(cancel["id"], command_type="cancel")
    await asyncio.sleep(0)
    assert not pending.done()  # the cancel is done, the target's result is still to come
    rig.result(task_id, status="cancelled", code="preempted")
    (entry,) = (await pending)["tasks"]
    assert entry["task_id"] == cancel["id"] and entry["target_id"] == task_id
    assert entry["state"] == "done" and entry["accepted"] is True and entry["task"] is None
    assert entry["result"]["status"] == "cancelled" and entry["result"]["task_id"] == task_id
    assert entry["result"]["code"] == "preempted"


async def test_cancel_task_wait_ack_with_an_explicit_target(rig: HubRig) -> None:
    await rig.announce()
    sent = await call_service(
        rig.hass,
        SERVICE_SEND_TASK,
        {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "reset", ATTR_WAIT: WAIT_NONE},
    )
    task_id = sent["tasks"][0]["task_id"]
    pending = start_call(
        rig.hass, SERVICE_CANCEL_TASK, {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK_ID: task_id}
    )
    await rig.until_published(CMD_TOPIC, 2)
    cancel = rig.commands()[1]
    assert cancel["target_id"] == task_id
    rig.ack(cancel["id"], command_type="cancel")
    (entry,) = (await pending)["tasks"]
    assert entry["state"] == "done" and entry["target_id"] == task_id and entry["result"] is None


async def test_cancel_rejected_raises_without_a_response(rig: HubRig) -> None:
    await rig.announce()
    pending = start_call(rig.hass, SERVICE_CANCEL_TASK, {ATTR_ROBOT_ID: ROBOT_ID}, response=False)
    await rig.until_published(CMD_TOPIC)
    rig.ack(rig.commands()[0]["id"], command_type="cancel", accepted=False)
    with pytest.raises(HomeAssistantError) as excinfo:
        await pending
    assert excinfo.value.translation_key == "task_rejected"
    assert excinfo.value.translation_placeholders == {
        "robot_id": ROBOT_ID,
        "task": "cancel",
        "code": "rejected_no_such_task",
        "reason": "Nothing running",
    }


async def test_cancel_with_a_bad_task_id_is_invalid_params(rig: HubRig) -> None:
    await rig.announce()
    with pytest.raises(ServiceValidationError) as excinfo:
        await call_service(
            rig.hass, SERVICE_CANCEL_TASK, {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK_ID: "nope"}
        )
    assert excinfo.value.translation_key == "invalid_params"


# --------------------------------------------------------------------------- get_robots


async def test_get_robots(rig: HubRig) -> None:
    await rig.announce()
    await rig.announce(OTHER_ROBOT)
    fire_robot_message(rig.hass, "availability", "online", robot_id="arm_quiet")
    await rig.hass.async_block_till_done()
    response = await call_service(rig.hass, SERVICE_GET_ROBOTS, {})
    robots = response["robots"]
    assert [robot["robot_id"] for robot in robots] == [
        ROBOT_ID,
        OTHER_ROBOT,
    ]  # no unannounced robot
    left = robots[0]
    assert left["name"] == "Left arm" and left["available"] is True
    assert left["device_id"] == rig.device_id(ROBOT_ID)
    assert left["state"] == "idle" and left["task"] is None and left["problem"] is False
    assert left["ha_link"] == "up" and left["paused"] is False
    assert [task["name"] for task in left["tasks"]] == [
        "reset",
        "stop",
        "pause",
        "resume",
        "move_to",
    ]
    assert left["tasks"][4]["needs_params"] is True
    assert set(left) == {
        "robot_id", "device_id", "name", "available", "state", "task", "task_id", "paused",
        "ha_link", "problem", "instance", "session", "tasks",
    }  # fmt: skip
    with pytest.raises(ServiceValidationError):
        await call_service(rig.hass, SERVICE_GET_ROBOTS, {}, response=False)


# --------------------------------------------------------------------------- MQTT v5 fallback


async def test_mqtt_v5_required_fallback(rig: HubRig) -> None:
    await rig.announce()
    rig.mqtt.async_publish.side_effect = [
        ServiceValidationError(
            translation_domain="mqtt", translation_key="mqtt_message_expiry_interval_not_supported"
        ),
        DEFAULT,
    ]
    response = await call_service(
        rig.hass,
        SERVICE_SEND_TASK,
        {ATTR_ROBOT_ID: ROBOT_ID, ATTR_TASK: "reset", ATTR_WAIT: WAIT_NONE},
    )
    rig.mqtt.async_publish.side_effect = None
    assert rig.mqtt.async_publish.call_args_list == [
        call(CMD_TOPIC, ANY, 1, False, message_expiry_interval=5),
        call(CMD_TOPIC, ANY, 1, False, message_expiry_interval=None),
    ]
    assert issue_id(ISSUE_MQTT_V5_REQUIRED) in rig.hub.open_issues
    assert response["tasks"][0]["state"] == "sent"
    assert rig.hub.prefix == PREFIX
