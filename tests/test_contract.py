# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Contract conformance of the HA side against the schemas and the ROS 2 protocol module.

Every command the HA builders can produce validates against
``contract/schemas/command.schema.json``; canonical JSON and signatures are
identical to ``ros2_ha_bridge.protocol`` for every vector; every command HA
encodes is accepted by the node's decoder and its signature verifies there;
and no must-reject command example can be produced by the builders.
"""

from __future__ import annotations

from types import ModuleType
from typing import Any

import pytest

from custom_components.ros2_bridge import protocol
from custom_components.ros2_bridge.protocol import CommandSource, ProtocolError

from .common import (
    INSTANCE_ID,
    ROBOT_ID,
    TASK_ID,
    VECTOR_SECRET,
    GoldenExample,
    load_examples,
    validate,
)

CANCEL_ID = "0c1d4e6f-2a3b-4c5d-8e9f-0a1b2c3d4e5f"
TOPIC = f"ros2_bridge/{ROBOT_ID}/cmd"


def _source(kind: str = "service") -> CommandSource:
    return CommandSource(
        kind=kind, instance_id=INSTANCE_ID, user_id=None, context_id="01J", device_id=None
    )


def _spread() -> dict[str, dict[str, Any]]:
    """A spread of commands the integration may build, keyed by a test id."""
    return {
        "task_no_params": protocol.task_command_payload(
            command_id=TASK_ID,
            robot_id=ROBOT_ID,
            task="reset",
            params={},
            issued_at=1760356800100,
            max_age_ms=5000,
            source=_source("button"),
        ),
        "task_with_timeout": protocol.task_command_payload(
            command_id=TASK_ID,
            robot_id="arm_right",
            task="move_to",
            params={"pose": "home", "speed": 0.5, "retries": 2, "gentle": True},
            issued_at=1760356800100,
            max_age_ms=30000,
            source=_source("intent"),
            timeout_s=45.5,
        ),
        "task_nested_non_ascii": protocol.task_command_payload(
            command_id=TASK_ID,
            robot_id=ROBOT_ID,
            task="grip",
            params={"zeta": 1, "options": {"tags": ["b", "a"], "limit": None}, "pose": "Küche"},
            issued_at=1760356800100,
            max_age_ms=1,
            source={"origin": "ha", "kind": "automation"},
        ),
        "cancel_active": protocol.cancel_command_payload(
            command_id=CANCEL_ID,
            robot_id=ROBOT_ID,
            target_id=None,
            issued_at=1760356802000,
            max_age_ms=5000,
            source=_source("button"),
        ),
        "cancel_target": protocol.cancel_command_payload(
            command_id=CANCEL_ID,
            robot_id=ROBOT_ID,
            target_id=TASK_ID,
            issued_at=1760356802000,
            max_age_ms=5000,
            source=_source("service"),
        ),
    }


@pytest.mark.parametrize("name", list(_spread()))
@pytest.mark.parametrize("secret", [None, VECTOR_SECRET], ids=["unsigned", "signed"])
def test_encoded_commands_validate_against_the_schema(name: str, secret: str | None) -> None:
    payload = _spread()[name]
    final, raw = protocol.encode_command(payload, topic=TOPIC, secret=secret)
    assert validate("command", final) == []
    assert ("sig" in final) is (secret is not None)
    assert len(raw) <= 16 * 1024


def test_golden_commands_validate(valid_command_example: GoldenExample) -> None:
    assert validate("command", valid_command_example.payload) == []


def test_invalid_commands_fail_the_schema_or_size(invalid_command_example: GoldenExample) -> None:
    payload = invalid_command_example.payload
    rejected = bool(validate("command", payload)) or len(invalid_command_example.raw) > 16 * 1024
    assert rejected, invalid_command_example.name


# --------------------------------------------------------------------------- against the ROS module


def test_vectors_agree_with_the_ros_protocol(
    hmac_vector: dict[str, Any], ros_protocol: ModuleType
) -> None:
    payload = hmac_vector["payload"]
    assert protocol.canonical_json(payload) == ros_protocol.canonical_json(payload)
    assert protocol.compute_sig(
        hmac_vector["secret"], hmac_vector["topic"], payload
    ) == ros_protocol.compute_sig(hmac_vector["secret"], hmac_vector["topic"], payload)
    assert (
        protocol.compute_sig(hmac_vector["secret"], hmac_vector["topic"], payload)
        == (hmac_vector["sig"])
    )


@pytest.mark.parametrize("name", list(_spread()))
def test_ros_decoder_accepts_and_verifies_ha_commands(name: str, ros_protocol: ModuleType) -> None:
    payload = _spread()[name]
    topic = f"ros2_bridge/{payload['robot_id']}/cmd"
    _final, raw = protocol.encode_command(payload, topic=topic, secret=VECTOR_SECRET)
    decoded = ros_protocol.decode_command(raw, expected_robot_id=payload["robot_id"])
    assert isinstance(decoded, ros_protocol.DecodedCommand), decoded
    assert decoded.id == payload["id"]
    assert decoded.type == payload["type"]
    assert ros_protocol.verify_sig(VECTOR_SECRET, topic, decoded.payload)
    assert not ros_protocol.verify_sig("another-secret", topic, decoded.payload)
    assert not ros_protocol.verify_sig(VECTOR_SECRET, "lab/arm_left/cmd", decoded.payload)


def test_golden_commands_round_trip_through_the_ros_decoder(
    valid_command_example: GoldenExample, ros_protocol: ModuleType
) -> None:
    decoded = ros_protocol.decode_command(valid_command_example.raw)
    assert isinstance(decoded, ros_protocol.DecodedCommand)
    assert decoded.robot_id == valid_command_example.payload["robot_id"]


def _rebuild_from_fields(payload: dict[str, Any]) -> dict[str, Any]:
    """Rebuild a payload from whatever fields it has, so the builders see the bad values."""
    source = payload.get("source", {"origin": "ha", "kind": "test"})
    common = {
        "command_id": payload.get("id", TASK_ID),
        "robot_id": payload.get("robot_id", ROBOT_ID),
        "issued_at": payload.get("issued_at", 1),
        "max_age_ms": payload.get("max_age_ms", 1),
        "source": source,
    }
    if payload.get("type") == "cancel":
        return protocol.cancel_command_payload(target_id=payload.get("target_id"), **common)
    return protocol.task_command_payload(
        task=payload.get("task", "reset"),
        params=payload.get("params", {}),
        timeout_s=payload.get("timeout_s"),
        **common,
    )


def test_builders_cannot_produce_a_must_reject_command(
    invalid_command_example: GoldenExample,
) -> None:
    """Either the builder refuses the bad field, or what it builds is valid and differs from the file."""
    payload = invalid_command_example.payload
    try:
        rebuilt = _rebuild_from_fields(payload)
        final, _raw = protocol.encode_command(rebuilt, topic=TOPIC, secret=None)
    except ProtocolError:
        return
    assert validate("command", final) == []
    assert final != payload


def test_every_kind_has_examples() -> None:
    for kind in ("command", "announce", "ack", "result", "state", "feedback", "heartbeat_robot"):
        assert len(load_examples(kind, valid=True)) >= 2
        assert len(load_examples(kind, valid=False)) >= 1
