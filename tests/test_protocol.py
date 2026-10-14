# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""``protocol.py`` against the HMAC vectors, the golden examples and the must-reject set."""

from __future__ import annotations

import json
from typing import Any

import pytest

from custom_components.ros2_bridge import protocol
from custom_components.ros2_bridge.const import (
    Availability,
    CommandType,
    HaLink,
    HealthLevel,
    ResultStatus,
    RobotStatus,
    TaskKind,
    WatchdogMode,
)
from custom_components.ros2_bridge.protocol import CommandSource, ProtocolError

from .common import (
    INBOUND_DECODERS,
    ROBOT_ID,
    TASK_ID,
    VECTOR_SECRET,
    GoldenExample,
    announce_payload,
    encode,
    load_examples,
    make_announce,
    make_result,
    state_payload,
)

INBOUND_KINDS = tuple(INBOUND_DECODERS)

# --------------------------------------------------------------------------- vectors


def test_vector_canonical_json_and_signature(hmac_vector: dict[str, Any]) -> None:
    """Canonical bytes and the digest match the vector file byte for byte."""
    assert (
        protocol.canonical_json(hmac_vector["payload"]).decode("utf-8") == hmac_vector["canonical"]
    )
    assert (
        protocol.compute_sig(hmac_vector["secret"], hmac_vector["topic"], hmac_vector["payload"])
        == hmac_vector["sig"]
    )


def test_canonical_json_ignores_sig_and_key_order() -> None:
    payload = {"b": 1, "a": {"z": True, "y": [2, 1]}, "sig": "ff" * 32}
    assert protocol.canonical_json(payload) == b'{"a":{"y":[2,1],"z":true},"b":1}'


def test_sign_command_binds_the_topic() -> None:
    payload = {"schema": 1, "id": TASK_ID}
    here = protocol.sign_command(payload, topic="ros2_bridge/arm_left/cmd", secret="s")
    there = protocol.sign_command(payload, topic="lab/arm_left/cmd", secret="s")
    assert here["sig"] != there["sig"]
    assert "sig" not in payload


# --------------------------------------------------------------------------- golden examples


@pytest.mark.parametrize("kind", INBOUND_KINDS)
def test_every_valid_example_decodes(kind: str) -> None:
    examples = load_examples(kind, valid=True)
    assert examples, kind
    for example in examples:
        decoded = INBOUND_DECODERS[kind](example.wire)
        assert decoded is not None, example.name


@pytest.mark.parametrize("kind", INBOUND_KINDS)
def test_every_invalid_example_is_refused(kind: str) -> None:
    examples = load_examples(kind, valid=False)
    assert examples, kind
    for example in examples:
        with pytest.raises(ProtocolError):
            INBOUND_DECODERS[kind](example.wire)


def test_announce_full_catalogue_fields(valid_announce_example: GoldenExample) -> None:
    announce = protocol.decode_announce(valid_announce_example.wire)
    payload = valid_announce_example.payload
    assert announce.robot_id == payload["robot_id"]
    assert announce.name == payload["name"]
    assert announce.session == payload["session"]
    assert announce.instance == payload["instance"]
    assert announce.hmac_required is payload["capabilities"]["hmac_required"]
    assert announce.watchdog_mode == WatchdogMode(payload["watchdog_mode"])
    assert announce.heartbeat_interval_s == payload["heartbeat_interval_s"]
    assert announce.task_names == tuple(t["name"] for t in payload["tasks"])
    for spec, raw in zip(announce.tasks, payload["tasks"], strict=True):
        assert spec.kind == TaskKind(raw["kind"])
        assert spec.timeout_s == raw["timeout_s"]
        assert (spec.params is None) == (raw["params"] is None)
        assert spec.description == raw.get("description", "")
    assert announce.manufacturer == payload.get("manufacturer")
    executor = payload.get("executor")
    assert announce.executor_name == (None if executor is None else executor["name"])


def test_task_spec_button_rule() -> None:
    announce = make_announce()
    reset = announce.task("reset")
    move_to = announce.task("move_to")
    assert reset is not None and move_to is not None
    assert reset.needs_params is False
    assert move_to.needs_params is True
    assert move_to.required_params == ("pose",)
    assert announce.task("nope") is None
    assert announce.task_by_label("move TO named pose ") is move_to


def test_announce_refuses_duplicate_task_names() -> None:
    payload = announce_payload()
    payload["tasks"].append(dict(payload["tasks"][0]))
    with pytest.raises(ProtocolError, match="twice"):
        protocol.decode_announce(encode(payload))


def test_state_fields(valid_state_example: GoldenExample) -> None:
    state = protocol.decode_state(valid_state_example.wire)
    payload = valid_state_example.payload
    assert state.status == RobotStatus(payload["state"])
    assert state.task_id == payload["task_id"]
    assert state.paused is payload["paused"]
    assert state.ha_link == HaLink(payload["ha_link"])
    assert state.executor_health_level == HealthLevel(payload["executor_health"]["level"])
    error = payload["error"]
    assert state.error_code == (None if error is None else error["code"])
    assert state.seq == payload["seq"]
    assert state.to_dict()["state"] == payload["state"]


def test_heartbeat_fields(valid_heartbeat_robot_example: GoldenExample) -> None:
    heartbeat = protocol.decode_heartbeat(valid_heartbeat_robot_example.wire)
    payload = valid_heartbeat_robot_example.payload
    assert dict(heartbeat.counters) == payload["counters"]
    assert heartbeat.ha_clock_offset_ms == payload["ha_clock_offset_ms"]
    assert heartbeat.uptime_s == payload["uptime_s"]


def test_ack_fields(valid_ack_example: GoldenExample) -> None:
    ack = protocol.decode_ack(valid_ack_example.wire)
    payload = valid_ack_example.payload
    assert ack.id == payload["id"]
    assert ack.type == CommandType(payload["type"])
    assert ack.accepted is payload["accepted"]
    assert ack.code == payload["code"]
    assert ack.clock_skew_ms == payload["clock_skew_ms"]
    assert ack.task == payload["task"]


def test_result_fields(valid_result_example: GoldenExample) -> None:
    result = protocol.decode_result(valid_result_example.wire)
    payload = valid_result_example.payload
    assert result.status == ResultStatus(payload["status"])
    assert result.goal_status == payload["goal_status"]
    assert result.kind == TaskKind(payload["kind"])
    assert dict(result.data) == payload["data"]
    assert result.duration_ms == payload["finished_at"] - payload["started_at"]


def test_feedback_fields(valid_feedback_example: GoldenExample) -> None:
    feedback = protocol.decode_feedback(valid_feedback_example.wire)
    assert feedback.progress == valid_feedback_example.payload["progress"]


def test_availability_is_exact() -> None:
    assert protocol.decode_availability(b"online") is Availability.ONLINE
    assert protocol.decode_availability("offline") is Availability.OFFLINE
    for bad in (b"online ", b"ONLINE", b'"online"', b"", b"{}"):
        with pytest.raises(ProtocolError):
            protocol.decode_availability(bad)


# --------------------------------------------------------------------------- decoder guards


@pytest.mark.parametrize(
    ("raw", "fragment"),
    [
        (b"\xff\xfe", "UTF-8"),
        (b"{not json", "not JSON"),
        (b"[1, 2]", "not a JSON object"),
        (b'{"schema": 2}', "schema must be 1"),
        (b'{"schema": true}', "schema must be 1"),
        (b'{"schema": 1, "robot_id": "ha"}', "robot_id"),
        (b'{"schema": 1, "robot_id": "Arm"}', "robot_id"),
    ],
)
def test_state_structural_errors(raw: bytes, fragment: str) -> None:
    with pytest.raises(ProtocolError, match=fragment):
        protocol.decode_state(raw)


def test_nan_is_refused() -> None:
    raw = encode(state_payload()).replace(b'"seq":42', b'"seq":NaN')
    with pytest.raises(ProtocolError, match="not JSON"):
        protocol.decode_state(raw)


def test_size_limits_apply_to_the_bytes_received() -> None:
    state = state_payload()
    state["padding"] = "x" * (16 * 1024)
    with pytest.raises(ProtocolError, match="over the limit"):
        protocol.decode_state(encode(state))
    announce = announce_payload()
    announce["padding"] = "x" * (16 * 1024)
    assert protocol.decode_announce(encode(announce)).robot_id == ROBOT_ID
    announce["padding"] = "x" * (64 * 1024)
    with pytest.raises(ProtocolError, match="over the limit"):
        protocol.decode_announce(encode(announce))


def test_result_goal_status_must_agree() -> None:
    with pytest.raises(ProtocolError, match="goal_status"):
        make_result(status="succeeded", goal_status=5)
    assert make_result(status="cancelled", goal_status=5).status is ResultStatus.CANCELLED


def test_ack_accepted_must_agree_with_code() -> None:
    from .common import ack_payload

    with pytest.raises(ProtocolError, match="accepted"):
        protocol.decode_ack(encode(ack_payload(accepted=False)))
    with pytest.raises(ProtocolError, match="clock_skew_ms"):
        protocol.decode_ack(encode(ack_payload(clock_skew_ms=...)))


def test_unknown_fields_are_ignored() -> None:
    payload = state_payload(extra={"future": "field"})
    assert protocol.decode_state(encode(payload)).robot_id == ROBOT_ID


# --------------------------------------------------------------------------- identifiers and topics


@pytest.mark.parametrize(
    ("value", "valid"),
    [
        ("arm_left", True),
        ("a", True),
        ("a" * 32, True),
        ("a" * 33, False),
        ("ha", False),
        ("Arm", False),
        ("arm left", False),
        ("1arm", False),
        ("", False),
        (None, False),
    ],
)
def test_robot_id_pattern(value: object, valid: bool) -> None:
    assert protocol.is_valid_robot_id(value) is valid
    if valid:
        assert protocol.validate_robot_id(value) == value
    else:
        with pytest.raises(ProtocolError):
            protocol.validate_robot_id(value)


@pytest.mark.parametrize(
    ("value", "valid"),
    [
        (TASK_ID, True),
        (TASK_ID.upper(), False),
        ("6a0c2b58-1d7e-1a1f-9b2a-3c4d5e6f7a8b", False),
        ("6a0c2b58-1d7e-4a1f-7b2a-3c4d5e6f7a8b", False),
        ("x/#", False),
        ("a+b", False),
        (TASK_ID + "\n", False),
        (42, False),
    ],
)
def test_task_id_pattern(value: object, valid: bool) -> None:
    assert protocol.is_valid_task_id(value) is valid
    if not valid:
        with pytest.raises(ProtocolError):
            protocol.validate_task_id(value)


@pytest.mark.parametrize(
    ("prefix", "ok"),
    [
        ("ros2_bridge", True),
        ("lab/arms", True),
        ("+", False),
        ("a/+", False),
        ("#", False),
        ("homeassistant", False),
        ("homeassistant/x", False),
        ("/x", False),
        ("x/", False),
        ("ha/x", False),
        ("ha", False),
        ("", False),
        ("a//b", False),
        ("a\x00b", False),
        (None, False),
    ],
)
def test_prefix_rules(prefix: object, ok: bool) -> None:
    assert protocol.is_valid_prefix(prefix) is ok
    if ok:
        assert isinstance(prefix, str)
        assert protocol.validate_prefix(prefix) == prefix
    else:
        assert protocol.prefix_error(prefix)
        if isinstance(prefix, str):
            with pytest.raises(ProtocolError):
                protocol.validate_prefix(prefix)


def test_split_robot_topic() -> None:
    assert protocol.split_robot_topic("ros2_bridge", "ros2_bridge/arm_left/announce") == (
        "arm_left",
        ["announce"],
    )
    assert protocol.split_robot_topic(
        "ros2_bridge", f"ros2_bridge/arm_left/task/{TASK_ID}/ack"
    ) == (
        "arm_left",
        ["task", TASK_ID, "ack"],
    )
    assert protocol.split_robot_topic("ros2_bridge", "ros2_bridge/ha/heartbeat") == (
        "ha",
        ["heartbeat"],
    )
    assert protocol.split_robot_topic("ros2_bridge", "other/arm_left/announce") is None
    assert protocol.split_robot_topic("ros2_bridge", "ros2_bridge/arm_left") is None
    assert protocol.split_robot_topic("ros2_bridge", "ros2_bridge//announce") is None


@pytest.mark.parametrize(
    "topic", ["", "a/+/b", "a/#", "a\x00", pytest.param("t" * 70000, id="longer_than_65535_bytes")]
)
def test_validate_publish_topic_refuses(topic: str) -> None:
    with pytest.raises(ProtocolError):
        protocol.validate_publish_topic(topic)


def test_validate_sensor_segment() -> None:
    assert protocol.validate_sensor_segment("domain", "binary_sensor") == "binary_sensor"
    with pytest.raises(ProtocolError):
        protocol.validate_sensor_segment("object_id", "Kitchen Motion")


# --------------------------------------------------------------------------- outbound commands


def _rebuild(payload: dict[str, Any]) -> dict[str, Any]:
    if payload["type"] == "task":
        return protocol.task_command_payload(
            command_id=payload["id"],
            robot_id=payload["robot_id"],
            task=payload["task"],
            params=payload["params"],
            issued_at=payload["issued_at"],
            max_age_ms=payload["max_age_ms"],
            source=payload["source"],
            timeout_s=payload.get("timeout_s"),
        )
    return protocol.cancel_command_payload(
        command_id=payload["id"],
        robot_id=payload["robot_id"],
        target_id=payload["target_id"],
        issued_at=payload["issued_at"],
        max_age_ms=payload["max_age_ms"],
        source=payload["source"],
    )


def test_encode_command_reproduces_golden(valid_command_example: GoldenExample) -> None:
    """Rebuilding a golden command from its fields gives the same payload, signature included."""
    payload = valid_command_example.payload
    topic = f"ros2_bridge/{payload['robot_id']}/cmd"
    final, raw = protocol.encode_command(
        _rebuild(payload), topic=topic, secret=VECTOR_SECRET if "sig" in payload else None
    )
    expected = dict(payload)
    if payload["type"] == "task":
        expected.setdefault("timeout_s", None)
    assert final == expected
    assert json.loads(raw.decode("utf-8")) == expected
    assert list(final)[:4] == ["schema", "id", "type", "robot_id"]


def test_encode_command_unsigned_without_secret() -> None:
    payload = protocol.task_command_payload(
        command_id=TASK_ID,
        robot_id=ROBOT_ID,
        task="reset",
        params={},
        issued_at=1,
        max_age_ms=5000,
        source=CommandSource(kind="button"),
    )
    final, raw = protocol.encode_command(payload, topic="ros2_bridge/arm_left/cmd", secret="")
    assert "sig" not in final
    assert b'"sig"' not in raw
    assert "instance_id" not in final["source"]


def test_command_source_payload_order_and_optional_instance() -> None:
    source = CommandSource(
        kind="intent", instance_id="x", user_id="u", context_id="c", device_id="d"
    )
    assert list(source.to_payload()) == [
        "origin",
        "instance_id",
        "kind",
        "user_id",
        "context_id",
        "device_id",
    ]
    assert source.to_payload()["origin"] == "ha"


@pytest.mark.parametrize(
    "overrides",
    [
        {"command_id": TASK_ID.upper()},
        {"command_id": "x/#"},
        {"robot_id": "ha"},
        {"robot_id": "arm left"},
        {"task": "move to"},
        {"params": ["not", "an", "object"]},
        {"issued_at": -1},
        {"issued_at": 1.5},
        {"max_age_ms": 0},
        {"timeout_s": 0},
        {"timeout_s": -3},
        {"source": {"origin": "ha"}},
        {"source": {"origin": "", "kind": "x"}},
        {"source": {"origin": "ha", "kind": "x", "instance_id": ""}},
        {"source": {"origin": "ha", "kind": "x", "user_id": 5}},
    ],
)
def test_task_command_payload_refuses_bad_fields(overrides: dict[str, Any]) -> None:
    fields: dict[str, Any] = {
        "command_id": TASK_ID,
        "robot_id": ROBOT_ID,
        "task": "move_to",
        "params": {"pose": "home"},
        "issued_at": 1,
        "max_age_ms": 5000,
        "source": {"origin": "ha", "kind": "service"},
    }
    fields.update(overrides)
    with pytest.raises(ProtocolError):
        protocol.task_command_payload(**fields)


def test_cancel_command_payload_refuses_bad_target() -> None:
    with pytest.raises(ProtocolError):
        protocol.cancel_command_payload(
            command_id=TASK_ID,
            robot_id=ROBOT_ID,
            target_id="not-a-uuid",
            issued_at=1,
            max_age_ms=1,
            source={"origin": "ha", "kind": "button"},
        )


def test_encode_command_refuses_oversize_and_bad_topic() -> None:
    payload = protocol.task_command_payload(
        command_id=TASK_ID,
        robot_id=ROBOT_ID,
        task="reset",
        params={"blob": "x" * (17 * 1024)},
        issued_at=1,
        max_age_ms=1,
        source={"origin": "ha", "kind": "service"},
    )
    with pytest.raises(ProtocolError, match="over the limit"):
        protocol.encode_command(payload, topic="ros2_bridge/arm_left/cmd")
    with pytest.raises(ProtocolError, match="wildcard"):
        protocol.encode_command({}, topic="ros2_bridge/+/cmd")


# --------------------------------------------------------------------------- params subset

MOVE_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "pose": {"type": "string", "enum": ["home", "rest"]},
        "speed": {"type": "number", "minimum": 0, "maximum": 1},
        "retries": {"type": "integer", "minimum": 0, "maximum": 3},
        "gentle": {"type": "boolean"},
    },
    "required": ["pose"],
}


@pytest.mark.parametrize(
    ("params", "problems"),
    [
        ({"pose": "home"}, 0),
        ({"pose": "home", "speed": 0.5, "retries": 2.0, "gentle": True, "extra": 1}, 0),
        ({}, 1),
        ({"pose": "kitchen"}, 1),
        ({"pose": 1}, 1),
        ({"pose": "home", "speed": 2}, 1),
        ({"pose": "home", "speed": True}, 1),
        ({"pose": "home", "retries": 1.5}, 1),
        ({"pose": "home", "retries": -1}, 1),
        ({"pose": "home", "gentle": "yes"}, 1),
        ("not an object", 1),
    ],
)
def test_validate_params(params: Any, problems: int) -> None:
    assert len(protocol.validate_params(MOVE_SCHEMA, params)) == problems


def test_validate_params_without_schema_accepts_any_object() -> None:
    assert protocol.validate_params(None, {"anything": 1}) == []
    assert protocol.validate_params(None, "x") == ["params must be an object"]


@pytest.mark.parametrize(
    "schema",
    [
        "x",
        {"type": "array", "properties": {}},
        {"type": "object"},
        {"type": "object", "properties": {"a": {"type": "array"}}},
        {"type": "object", "properties": {"a": {"type": "string", "enum": []}}},
        {"type": "object", "properties": {"a": {"type": "number", "minimum": "0"}}},
        {"type": "object", "properties": {"a": {"type": "string"}}, "required": ["a", "a"]},
        {"type": "object", "properties": {"a": {"type": "string"}}, "required": "a"},
        {"type": "object", "properties": {"a": "string"}},
        {"type": "object", "properties": {"a": {"type": "string", "description": 1}}},
    ],
)
def test_validate_params_schema_refuses(schema: Any) -> None:
    assert protocol.validate_params_schema(schema)


def test_validate_params_schema_accepts_the_contract_subset() -> None:
    assert protocol.validate_params_schema(MOVE_SCHEMA) == []
