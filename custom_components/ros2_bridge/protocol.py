# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The ha-ros2-bridge MQTT contract (schema 1) as pure Python, Home Assistant side.

This module mirrors ``ros2/ros2_ha_bridge/ros2_ha_bridge/protocol.py`` byte for
byte where the two halves must agree: canonical JSON, the command signature,
the identifier patterns and the params-subset validator. On top of that it
builds and encodes the two outbound command payloads and decodes every inbound
kind (announce, availability, state, heartbeat, ack, feedback, result, joints)
into the dataclasses of ``models.py``, raising :class:`ProtocolError` with a reason on
anything the schemas would refuse. It imports nothing from Home Assistant and
performs no I/O, so it is tested against ``contract/`` on any machine.
"""

from __future__ import annotations

import hashlib
import hmac
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from .const import (
    ACK_CODE_ACCEPTED,
    ACK_CODES,
    AVAILABILITY_OFFLINE,
    AVAILABILITY_ONLINE,
    COUNTER_NAMES,
    DISCOVERY_PREFIX,
    GOAL_STATUS,
    INSTANCE_RE,
    JOINT_NAME_RE,
    MAX_ANNOUNCE_BYTES,
    MAX_JOINTS,
    MAX_PAYLOAD_BYTES,
    PARAM_TYPES,
    RESERVED_ROBOT_ID,
    RESULT_CODE_RE,
    ROBOT_ID_RE,
    SCHEMA_VERSION,
    SENSOR_SEGMENT_RE,
    SESSION_RE,
    SOURCE_ORIGIN,
    TASK_ID_RE,
    TASK_NAME_RE,
    Availability,
    CommandType,
    HaLink,
    HealthLevel,
    JointUnit,
    ResultStatus,
    RobotStatus,
    TaskKind,
    WatchdogMode,
)
from .models import (
    Ack,
    Announce,
    Feedback,
    Heartbeat,
    JointInfo,
    Joints,
    Result,
    RobotState,
    TaskSpec,
)

#: MQTT's own limit on a topic name.
MAX_TOPIC_BYTES = 65535

RawPayload = bytes | bytearray | memoryview | str


class ProtocolError(ValueError):
    """An inbound payload or an outbound command field that the contract refuses.

    ``reason`` says what is wrong in one line; ``kind`` names the payload kind
    when known. The hub logs the reason, counts the drop and never answers.
    """

    def __init__(self, reason: str, *, kind: str | None = None) -> None:
        super().__init__(reason)
        self.reason = reason
        self.kind = kind


# --------------------------------------------------------------------------- identifiers


def is_valid_robot_id(value: object) -> bool:
    """True for a string matching the robot-id pattern that is not the reserved ``ha``."""
    return (
        isinstance(value, str)
        and ROBOT_ID_RE.fullmatch(value) is not None
        and value != RESERVED_ROBOT_ID
    )


def is_valid_task_name(value: object) -> bool:
    """True for a string matching the task-name pattern."""
    return isinstance(value, str) and TASK_NAME_RE.fullmatch(value) is not None


def is_valid_task_id(value: object) -> bool:
    """True for a lower-case UUID4 string; the only ids that may name a reply topic."""
    return isinstance(value, str) and TASK_ID_RE.fullmatch(value) is not None


def is_valid_session(value: object) -> bool:
    """True for a lower-case UUID4 string."""
    return isinstance(value, str) and SESSION_RE.fullmatch(value) is not None


def is_valid_result_code(value: object) -> bool:
    """True for a string matching the result-code pattern."""
    return isinstance(value, str) and RESULT_CODE_RE.fullmatch(value) is not None


def is_valid_joint_name(value: object) -> bool:
    """True for a string matching the joint-name pattern (0.2.0)."""
    return isinstance(value, str) and JOINT_NAME_RE.fullmatch(value) is not None


def is_valid_instance(value: object) -> bool:
    """True for a ``<hostname>:<pid>`` string."""
    return isinstance(value, str) and INSTANCE_RE.fullmatch(value) is not None


def validate_robot_id(robot_id: object) -> str:
    """Return ``robot_id`` or raise :class:`ProtocolError` saying why it is unusable."""
    if not isinstance(robot_id, str):
        raise ProtocolError("robot id must be a string")
    if robot_id == RESERVED_ROBOT_ID:
        raise ProtocolError(f"robot id {robot_id!r} is reserved for Home Assistant's own topics")
    if ROBOT_ID_RE.fullmatch(robot_id) is None:
        raise ProtocolError(
            f"robot id {robot_id!r} must match ^[a-z][a-z0-9_]{{0,31}}$ "
            "(lower-case, starts with a letter, at most 32 characters)"
        )
    return robot_id


def validate_task_id(task_id: object) -> str:
    """Return ``task_id`` or raise :class:`ProtocolError`; only UUID4 ids name reply topics."""
    if not is_valid_task_id(task_id):
        raise ProtocolError(f"task id {task_id!r} is not a lower-case UUID4 string")
    assert isinstance(task_id, str)
    return task_id


def validate_task_name(task: object) -> str:
    """Return ``task`` or raise :class:`ProtocolError`."""
    if not is_valid_task_name(task):
        raise ProtocolError(f"task name {task!r} must match ^[a-z][a-z0-9_]{{0,31}}$")
    assert isinstance(task, str)
    return task


# --------------------------------------------------------------------------- prefix and topics


def prefix_error(prefix: object) -> str | None:
    """Return why ``prefix`` is not an acceptable topic prefix, or ``None`` when it is.

    The contract's rules: no leading or trailing slash, no ``+`` or ``#``, not
    ``homeassistant`` and not starting with the segment ``ha``. Empty segments
    and NUL characters are refused as well because no broker accepts them.
    """
    if not isinstance(prefix, str):
        return "prefix must be a string"
    if not prefix:
        return "prefix is empty"
    if "\x00" in prefix:
        return "prefix contains a NUL character"
    if "+" in prefix or "#" in prefix:
        return "prefix contains a wildcard character ('+' or '#')"
    if prefix.startswith("/") or prefix.endswith("/"):
        return "prefix must not start or end with a slash"
    segments = prefix.split("/")
    if "" in segments:
        return "prefix contains an empty segment"
    if segments[0] == RESERVED_ROBOT_ID:
        return f"prefix must not start with the reserved segment '{RESERVED_ROBOT_ID}'"
    if segments[0] == DISCOVERY_PREFIX:
        return f"prefix must not be or start with the discovery prefix '{DISCOVERY_PREFIX}'"
    return None


def is_valid_prefix(prefix: object) -> bool:
    """True when :func:`prefix_error` finds nothing wrong."""
    return prefix_error(prefix) is None


def validate_prefix(prefix: str) -> str:
    """Return ``prefix`` or raise :class:`ProtocolError` with the reason it is refused."""
    error = prefix_error(prefix)
    if error is not None:
        raise ProtocolError(f"invalid topic prefix {prefix!r}: {error}")
    return prefix


def validate_publish_topic(topic: str) -> str:
    """Return ``topic`` or raise :class:`ProtocolError`; the guard every publish passes first."""
    if not isinstance(topic, str) or not topic:
        raise ProtocolError("publish topic must be a non-empty string")
    if "+" in topic or "#" in topic:
        raise ProtocolError(f"publish topic {topic!r} contains a wildcard character")
    if "\x00" in topic:
        raise ProtocolError("publish topic contains a NUL character")
    if len(topic.encode("utf-8")) > MAX_TOPIC_BYTES:
        raise ProtocolError(f"publish topic is longer than {MAX_TOPIC_BYTES} bytes")
    return topic


def validate_sensor_segment(label: str, segment: object) -> str:
    """Return a ``domain`` or ``object_id`` segment of a sensor topic or raise."""
    if not isinstance(segment, str) or SENSOR_SEGMENT_RE.fullmatch(segment) is None:
        raise ProtocolError(f"sensor {label} {segment!r} must match ^[a-z0-9_]+$")
    return segment


def split_robot_topic(prefix: str, topic: str) -> tuple[str, list[str]] | None:
    """Split ``{prefix}/{segment}/rest...`` into ``(segment, [rest...])``.

    Returns ``None`` when the topic does not sit under the prefix or has no
    segment after it. The segment is returned unchecked; the caller discards
    ``ha`` silently and anything failing :func:`is_valid_robot_id` with a count.
    """
    head = f"{prefix}/"
    if not topic.startswith(head):
        return None
    parts = topic[len(head) :].split("/")
    if len(parts) < 2 or not parts[0]:
        return None
    return parts[0], parts[1:]


# --------------------------------------------------------------------------- JSON and HMAC


def canonical_json(payload: Mapping[str, Any]) -> bytes:
    """The bytes that are signed: ``sig`` removed, keys sorted at every level, compact, UTF-8.

    ``ensure_ascii=False`` means non-ASCII text is signed as its UTF-8 bytes,
    not as ``\\u`` escapes, and list order is preserved. Matches
    ``contract/hmac_vectors.json`` byte for byte.
    """
    unsigned = {key: value for key, value in payload.items() if key != "sig"}
    return json.dumps(unsigned, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode(
        "utf-8"
    )


def compute_sig(secret: str, topic: str, payload: Mapping[str, Any]) -> str:
    """``hex(HMAC-SHA256(secret, topic + "\\n" + canonical))``; a present ``sig`` is ignored."""
    message = topic.encode("utf-8") + b"\n" + canonical_json(payload)
    return hmac.new(secret.encode("utf-8"), message, hashlib.sha256).hexdigest()


def encode_payload(payload: Mapping[str, Any]) -> bytes:
    """The wire form of a payload: compact JSON, UTF-8, key order as given."""
    return json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


# --------------------------------------------------------------------------- outbound commands


@dataclass(frozen=True, slots=True)
class CommandSource:
    """``cmd.source``: who raised the command. ``origin`` is always ``ha``."""

    kind: str
    instance_id: str | None = None
    user_id: str | None = None
    context_id: str | None = None
    device_id: str | None = None
    origin: str = SOURCE_ORIGIN

    def to_payload(self) -> dict[str, Any]:
        """The wire form; ``instance_id`` is left out when unknown (optional in the schema)."""
        payload: dict[str, Any] = {"origin": self.origin}
        if self.instance_id is not None:
            payload["instance_id"] = self.instance_id
        payload["kind"] = self.kind
        payload["user_id"] = self.user_id
        payload["context_id"] = self.context_id
        payload["device_id"] = self.device_id
        return payload


def _source_payload(source: CommandSource | Mapping[str, Any]) -> dict[str, Any]:
    payload = source.to_payload() if isinstance(source, CommandSource) else dict(source)
    for key in ("origin", "kind"):
        if not isinstance(payload.get(key), str) or not payload[key]:
            raise ProtocolError(f"source.{key} must be a non-empty string")
    if "instance_id" in payload and (
        not isinstance(payload["instance_id"], str) or not payload["instance_id"]
    ):
        raise ProtocolError("source.instance_id must be a non-empty string when present")
    for key in ("user_id", "context_id", "device_id"):
        if key in payload and payload[key] is not None and not isinstance(payload[key], str):
            raise ProtocolError(f"source.{key} must be a string or null")
    return payload


def _validate_common(
    command_id: object, robot_id: object, issued_at: object, max_age_ms: object
) -> None:
    validate_task_id(command_id)
    validate_robot_id(robot_id)
    if _as_int(issued_at, minimum=0) is None:
        raise ProtocolError("issued_at must be a non-negative integer (epoch milliseconds)")
    if _as_int(max_age_ms, minimum=1) is None:
        raise ProtocolError("max_age_ms must be an integer of at least 1")


def task_command_payload(
    *,
    command_id: str,
    robot_id: str,
    task: str,
    params: Mapping[str, Any],
    issued_at: int,
    max_age_ms: int,
    source: CommandSource | Mapping[str, Any],
    timeout_s: float | None = None,
) -> dict[str, Any]:
    """Build an unsigned ``type: task`` command in the contract's key order.

    Every field is validated so a bad command can never be built; the result is
    what :func:`encode_command` signs and encodes.
    """
    _validate_common(command_id, robot_id, issued_at, max_age_ms)
    validate_task_name(task)
    if not isinstance(params, Mapping):
        raise ProtocolError("params must be an object (empty when the task takes none)")
    if timeout_s is not None and (not _is_number(timeout_s) or timeout_s <= 0):
        raise ProtocolError("timeout_s must be null or a number greater than 0")
    return {
        "schema": SCHEMA_VERSION,
        "id": command_id,
        "type": str(CommandType.TASK),
        "robot_id": robot_id,
        "task": task,
        "params": dict(params),
        "issued_at": issued_at,
        "max_age_ms": max_age_ms,
        "timeout_s": timeout_s,
        "source": _source_payload(source),
    }


def cancel_command_payload(
    *,
    command_id: str,
    robot_id: str,
    target_id: str | None,
    issued_at: int,
    max_age_ms: int,
    source: CommandSource | Mapping[str, Any],
) -> dict[str, Any]:
    """Build an unsigned ``type: cancel`` command; ``target_id`` ``None`` means the active task."""
    _validate_common(command_id, robot_id, issued_at, max_age_ms)
    if target_id is not None:
        validate_task_id(target_id)
    return {
        "schema": SCHEMA_VERSION,
        "id": command_id,
        "type": str(CommandType.CANCEL),
        "robot_id": robot_id,
        "target_id": target_id,
        "issued_at": issued_at,
        "max_age_ms": max_age_ms,
        "source": _source_payload(source),
    }


def sign_command(payload: Mapping[str, Any], *, topic: str, secret: str) -> dict[str, Any]:
    """A copy of ``payload`` with ``sig`` computed over ``topic`` and the canonical form."""
    signed = {key: value for key, value in payload.items() if key != "sig"}
    signed["sig"] = compute_sig(secret, topic, signed)
    return signed


def encode_command(
    payload: Mapping[str, Any], *, topic: str, secret: str | None = None
) -> tuple[dict[str, Any], bytes]:
    """Sign (when ``secret`` is set and not empty) and encode a command for ``topic``.

    Returns the final payload (with ``sig`` when signed) and its wire bytes. The
    topic is validated first because it is part of the signed message.
    """
    validate_publish_topic(topic)
    final = sign_command(payload, topic=topic, secret=secret) if secret else dict(payload)
    raw = encode_payload(final)
    if len(raw) > MAX_PAYLOAD_BYTES:
        raise ProtocolError(f"command is {len(raw)} bytes, over the limit of {MAX_PAYLOAD_BYTES}")
    return final, raw


# --------------------------------------------------------------------------- decoding helpers


def _is_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _as_int(value: object, *, minimum: int | None = None) -> int | None:
    """``value`` when it is a JSON integer (never a bool) at or above ``minimum``, else None."""
    if not isinstance(value, int) or isinstance(value, bool):
        return None
    if minimum is not None and value < minimum:
        return None
    return value


def _is_number(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _reject_constant(name: str) -> object:
    raise ValueError(f"{name} is not valid JSON")


def _to_bytes(raw: RawPayload) -> bytes:
    if isinstance(raw, str):
        return raw.encode("utf-8")
    return bytes(raw)


def _parse_object(raw: RawPayload, *, kind: str, max_bytes: int) -> dict[str, Any]:
    """Size check, UTF-8, strict JSON and object check, in that order."""
    data = _to_bytes(raw)
    if len(data) > max_bytes:
        raise ProtocolError(f"{len(data)} bytes is over the limit of {max_bytes}", kind=kind)
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as error:
        raise ProtocolError(f"payload is not UTF-8: {error.reason}", kind=kind) from None
    try:
        parsed: object = json.loads(text, parse_constant=_reject_constant)
    except ValueError as error:
        raise ProtocolError(f"payload is not JSON: {error}", kind=kind) from None
    if not isinstance(parsed, dict):
        raise ProtocolError("payload is not a JSON object", kind=kind)
    schema = parsed.get("schema")
    if not _is_int(schema) or schema != SCHEMA_VERSION:
        raise ProtocolError(f"schema must be {SCHEMA_VERSION}", kind=kind)
    return parsed


class _Fields:
    """Typed field access over a decoded object; every failure is a ProtocolError."""

    __slots__ = ("data", "kind")

    def __init__(self, data: Mapping[str, Any], kind: str) -> None:
        self.data = data
        self.kind = kind

    def fail(self, message: str) -> ProtocolError:
        return ProtocolError(message, kind=self.kind)

    def present(self, name: str) -> object:
        if name not in self.data:
            raise self.fail(f"{name} is missing")
        return self.data[name]

    def string(self, name: str, *, non_empty: bool = False) -> str:
        value = self.present(name)
        if not isinstance(value, str) or (non_empty and not value):
            raise self.fail(f"{name} must be a {'non-empty ' if non_empty else ''}string")
        return value

    def optional_string(self, name: str) -> str | None:
        value = self.data.get(name)
        if value is not None and not isinstance(value, str):
            raise self.fail(f"{name} must be a string or null")
        return value

    def boolean(self, name: str) -> bool:
        value = self.present(name)
        if not isinstance(value, bool):
            raise self.fail(f"{name} must be a boolean")
        return value

    def integer(self, name: str, *, minimum: int | None = None) -> int:
        value = _as_int(self.present(name), minimum=minimum)
        if value is None:
            bound = "" if minimum is None else f" of at least {minimum}"
            raise self.fail(f"{name} must be an integer{bound}")
        return value

    def optional_integer(self, name: str) -> int | None:
        value = self.data.get(name)
        if value is None:
            return None
        parsed = _as_int(value)
        if parsed is None:
            raise self.fail(f"{name} must be an integer or null")
        return parsed

    def number(self, name: str, *, minimum: float | None = None, exclusive: bool = False) -> float:
        value = self.present(name)
        if not _is_number(value):
            raise self.fail(f"{name} must be a number")
        assert isinstance(value, (int, float))
        if minimum is not None and (value < minimum or (exclusive and value == minimum)):
            raise self.fail(
                f"{name} must be {'greater than' if exclusive else 'at least'} {minimum}"
            )
        return float(value)

    def epoch(self, name: str) -> int:
        return self.integer(name, minimum=0)

    def obj(self, name: str) -> dict[str, Any]:
        value = self.present(name)
        if not isinstance(value, dict):
            raise self.fail(f"{name} must be an object")
        return value

    def robot_id(self) -> str:
        value = self.present("robot_id")
        if not is_valid_robot_id(value):
            raise self.fail("robot_id is missing, reserved or does not match the pattern")
        assert isinstance(value, str)
        return value

    def task_id(self, name: str = "id") -> str:
        value = self.present(name)
        if not is_valid_task_id(value):
            raise self.fail(f"{name} must be a lower-case UUID4 string")
        assert isinstance(value, str)
        return value

    def optional_task_id(self, name: str) -> str | None:
        value = self.present(name)
        if value is None:
            return None
        if not is_valid_task_id(value):
            raise self.fail(f"{name} must be null or a lower-case UUID4 string")
        assert isinstance(value, str)
        return value

    def session(self) -> str:
        value = self.present("session")
        if not is_valid_session(value):
            raise self.fail("session must be a lower-case UUID4 string")
        assert isinstance(value, str)
        return value

    def instance(self) -> str:
        value = self.present("instance")
        if not is_valid_instance(value):
            raise self.fail("instance must be '<hostname>:<pid>'")
        assert isinstance(value, str)
        return value

    def task_name(self, name: str = "task") -> str:
        value = self.present(name)
        if not is_valid_task_name(value):
            raise self.fail(f"{name} must match ^[a-z][a-z0-9_]{{0,31}}$")
        assert isinstance(value, str)
        return value

    def optional_task_name(self, name: str = "task") -> str | None:
        value = self.present(name)
        if value is None:
            return None
        if not is_valid_task_name(value):
            raise self.fail(f"{name} must be null or match ^[a-z][a-z0-9_]{{0,31}}$")
        assert isinstance(value, str)
        return value

    def enum(self, name: str, allowed: tuple[str, ...]) -> str:
        value = self.present(name)
        if not isinstance(value, str) or value not in allowed:
            raise self.fail(f"{name} must be one of {', '.join(allowed)}")
        return value

    def numbers(self, name: str, *, min_items: int, max_items: int) -> tuple[float, ...]:
        """A list of ``min_items`` to ``max_items`` finite numbers (never booleans)."""
        return self._numbers(name, self.present(name), min_items=min_items, max_items=max_items)

    def optional_numbers(
        self, name: str, *, min_items: int, max_items: int
    ) -> tuple[float, ...] | None:
        """``None`` for a null value, otherwise as :meth:`numbers`; the key must be present."""
        value = self.present(name)
        if value is None:
            return None
        return self._numbers(name, value, min_items=min_items, max_items=max_items)

    def _numbers(
        self, name: str, value: object, *, min_items: int, max_items: int
    ) -> tuple[float, ...]:
        if not isinstance(value, list):
            raise self.fail(f"{name} must be a list of numbers")
        if not min_items <= len(value) <= max_items:
            raise self.fail(f"{name} must hold {min_items} to {max_items} numbers")
        if not all(_is_number(item) for item in value):
            raise self.fail(f"{name} must hold numbers only")
        return tuple(float(item) for item in value)


def _enum_values(enum_type: type[Any]) -> tuple[str, ...]:
    return tuple(str(member.value) for member in enum_type)


# --------------------------------------------------------------------------- params subset


def _json_equal(left: object, right: object) -> bool:
    """Equality as JSON sees it: booleans never equal numbers, 1 equals 1.0."""
    if isinstance(left, bool) or isinstance(right, bool):
        return isinstance(left, bool) and isinstance(right, bool) and left is right
    return left == right


def _is_primitive(value: object) -> bool:
    return isinstance(value, (str, bool, int, float))


def validate_params_schema(schema: object) -> list[str]:
    """Check that ``schema`` is a well-formed params subset; return the problems found.

    The subset: ``type`` is ``object``; ``properties`` maps names to specs whose
    ``type`` is one of string, number, integer, boolean and which may carry
    ``enum`` (a non-empty list of primitives), numeric ``minimum`` and
    ``maximum``, and a string ``description``; ``required`` is a list of
    distinct strings. Other keys are ignored, as the contract says.
    """
    if not isinstance(schema, Mapping):
        return ["params schema must be an object"]
    errors: list[str] = []
    if schema.get("type") != "object":
        errors.append("params schema 'type' must be 'object'")
    properties = schema.get("properties")
    if not isinstance(properties, Mapping):
        errors.append("params schema 'properties' must be an object")
        properties = {}
    for name, spec in properties.items():
        if not isinstance(name, str):
            errors.append(f"property name {name!r} must be a string")
            continue
        if not isinstance(spec, Mapping):
            errors.append(f"property '{name}' must be an object")
            continue
        if spec.get("type") not in PARAM_TYPES:
            errors.append(f"property '{name}' type must be one of {', '.join(PARAM_TYPES)}")
        if "enum" in spec:
            enum = spec["enum"]
            if not isinstance(enum, list) or not enum or not all(_is_primitive(v) for v in enum):
                errors.append(f"property '{name}' enum must be a non-empty list of primitives")
        for bound in ("minimum", "maximum"):
            if bound in spec and not _is_number(spec[bound]):
                errors.append(f"property '{name}' {bound} must be a number")
        if "description" in spec and not isinstance(spec["description"], str):
            errors.append(f"property '{name}' description must be a string")
    if "required" in schema:
        required = schema["required"]
        if not isinstance(required, list) or not all(isinstance(n, str) for n in required):
            errors.append("params schema 'required' must be a list of strings")
        elif len(set(required)) != len(required):
            errors.append("params schema 'required' must not repeat a name")
    return errors


def _check_param(name: str, spec: Mapping[str, Any], value: object) -> list[str]:
    expected = spec.get("type")
    type_ok: bool
    if expected == "string":
        type_ok = isinstance(value, str)
    elif expected == "boolean":
        type_ok = isinstance(value, bool)
    elif expected == "integer":
        type_ok = _is_int(value) or (
            isinstance(value, float) and math.isfinite(value) and value.is_integer()
        )
    elif expected == "number":
        type_ok = _is_number(value)
    else:
        return [f"'{name}' has an unsupported type in the announced schema"]
    if not type_ok:
        return [f"'{name}' must be a {expected}"]
    errors: list[str] = []
    enum = spec.get("enum")
    if isinstance(enum, list) and not any(_json_equal(value, item) for item in enum):
        errors.append(f"'{name}' must be one of {enum!r}")
    if _is_number(value):
        assert isinstance(value, (int, float))
        minimum = spec.get("minimum")
        if _is_number(minimum):
            assert isinstance(minimum, (int, float))
            if value < minimum:
                errors.append(f"'{name}' must be at least {minimum}")
        maximum = spec.get("maximum")
        if _is_number(maximum):
            assert isinstance(maximum, (int, float))
            if value > maximum:
                errors.append(f"'{name}' must be at most {maximum}")
    return errors


def validate_params(schema: Mapping[str, Any] | None, params: object) -> list[str]:
    """Validate a command's ``params`` against a task's announced params subset.

    Returns the list of problems (empty means valid). ``params`` must be an
    object. With ``schema`` ``None`` (a task that takes no parameters) any
    object passes. Otherwise every name in ``required`` must be present and
    every present parameter that the schema declares must match its ``type``,
    ``enum``, ``minimum`` and ``maximum``; parameters the schema does not
    declare are not interpreted. Hand-written on purpose, like the ROS side,
    so both halves agree without a JSON Schema library.
    """
    if not isinstance(params, Mapping):
        return ["params must be an object"]
    if schema is None:
        return []
    errors: list[str] = []
    properties = schema.get("properties")
    if not isinstance(properties, Mapping):
        properties = {}
    required = schema.get("required")
    if isinstance(required, list):
        errors.extend(f"'{name}' is required" for name in required if name not in params)
    for name, value in params.items():
        spec = properties.get(name)
        if isinstance(spec, Mapping) and isinstance(name, str):
            errors.extend(_check_param(name, spec, value))
    return errors


# --------------------------------------------------------------------------- inbound decoding


def decode_availability(raw: RawPayload) -> Availability:
    """The plain word ``online`` or ``offline``; anything else (including JSON) is refused."""
    text = _to_bytes(raw)
    if text == AVAILABILITY_ONLINE.encode("utf-8"):
        return Availability.ONLINE
    if text == AVAILABILITY_OFFLINE.encode("utf-8"):
        return Availability.OFFLINE
    raise ProtocolError("availability must be exactly 'online' or 'offline'", kind="availability")


def _decode_task_spec(index: int, raw_spec: object) -> TaskSpec:
    if not isinstance(raw_spec, dict):
        raise ProtocolError(f"tasks[{index}] must be an object", kind="announce")
    fields = _Fields(raw_spec, "announce")
    name = fields.task_name("name")
    label = fields.string("label", non_empty=True)
    kind = TaskKind(fields.enum("kind", _enum_values(TaskKind)))
    timeout_s = fields.number("timeout_s", minimum=0, exclusive=True)
    params_raw = fields.present("params")
    params: dict[str, Any] | None = None
    if params_raw is not None:
        problems = validate_params_schema(params_raw)
        if problems:
            raise ProtocolError(f"task '{name}' params: {problems[0]}", kind="announce")
        assert isinstance(params_raw, dict)
        params = params_raw
    description = raw_spec.get("description", "")
    if not isinstance(description, str):
        raise ProtocolError(f"task '{name}' description must be a string", kind="announce")
    return TaskSpec(
        name=name,
        label=label,
        kind=kind,
        timeout_s=timeout_s,
        params=params,
        description=description,
    )


def _decode_joint_info(raw_block: object) -> tuple[JointInfo | None, str | None]:
    """Decode an announce ``joints`` block; returns ``(info, None)`` or ``(None, reason)``.

    Anything the schema refuses (a bad unit, no names, a name failing the
    pattern, more than 32 names, a limits entry that is not a pair of numbers,
    a rate of 0) raises :class:`ProtocolError` and the whole announce is
    refused. A ``limits`` list whose length differs from ``names`` is not a
    schema failure; the contract says HA ignores such a block, so it comes
    back as ``(None, reason)`` and the announce stands without joints.
    """
    if not isinstance(raw_block, dict):
        raise ProtocolError("joints must be an object", kind="announce")
    fields = _Fields(raw_block, "announce")
    unit = JointUnit(fields.enum("unit", _enum_values(JointUnit)))
    names_raw = fields.present("names")
    if not isinstance(names_raw, list):
        raise fields.fail("joints.names must be a list")
    if not 1 <= len(names_raw) <= MAX_JOINTS:
        raise fields.fail(f"joints.names must hold 1 to {MAX_JOINTS} names")
    names: list[str] = []
    for index, name in enumerate(names_raw):
        if not is_valid_joint_name(name):
            raise fields.fail(f"joints.names[{index}] must match ^[a-z][a-z0-9_]{{0,31}}$")
        assert isinstance(name, str)
        names.append(name)
    limits_raw = fields.present("limits")
    limits: tuple[tuple[float, float], ...] | None = None
    if limits_raw is not None:
        if not isinstance(limits_raw, list):
            raise fields.fail("joints.limits must be null or a list of [minimum, maximum] pairs")
        pairs: list[tuple[float, float]] = []
        for index, pair in enumerate(limits_raw):
            if not (isinstance(pair, list) and len(pair) == 2 and all(map(_is_number, pair))):
                raise fields.fail(f"joints.limits[{index}] must be a pair of numbers")
            pairs.append((float(pair[0]), float(pair[1])))
        limits = tuple(pairs)
    rate_hz = fields.number("rate_hz", minimum=0, exclusive=True)
    if limits is not None and len(limits) != len(names):
        return None, (
            f"joints.limits has {len(limits)} entries for {len(names)} names; the block is ignored"
        )
    return JointInfo(unit=unit, names=tuple(names), limits=limits, rate_hz=rate_hz), None


def decode_announce(raw: RawPayload) -> Announce:
    """Decode the announce payload (64 KiB limit). An empty payload is a tombstone, not an announce."""
    data = _parse_object(raw, kind="announce", max_bytes=MAX_ANNOUNCE_BYTES)
    fields = _Fields(data, "announce")
    robot_id = fields.robot_id()
    name = fields.string("name", non_empty=True)
    bridge_version = fields.string("bridge_version", non_empty=True)
    session = fields.session()
    instance = fields.instance()
    capabilities = _Fields(fields.obj("capabilities"), "announce")
    cancel = capabilities.boolean("cancel")
    feedback = capabilities.boolean("feedback")
    hmac_required = capabilities.boolean("hmac_required")
    watchdog_mode = WatchdogMode(fields.enum("watchdog_mode", _enum_values(WatchdogMode)))
    heartbeat_interval_s = fields.number("heartbeat_interval_s", minimum=0)
    tasks_raw = fields.present("tasks")
    if not isinstance(tasks_raw, list):
        raise fields.fail("tasks must be a list")
    tasks = tuple(_decode_task_spec(index, spec) for index, spec in enumerate(tasks_raw))
    seen: set[str] = set()
    for spec in tasks:
        if spec.name in seen:
            raise fields.fail(f"tasks names a task twice: '{spec.name}'")
        seen.add(spec.name)
    ts = fields.epoch("ts")
    executor_name: str | None = None
    executor_version: str | None = None
    if data.get("executor") is not None:
        executor = _Fields(fields.obj("executor"), "announce")
        executor_name = executor.string("name", non_empty=True)
        executor_version = executor.string("version")
    joints: JointInfo | None = None
    joints_ignored: str | None = None
    if "joints" in data:
        joints, joints_ignored = _decode_joint_info(data["joints"])
    return Announce(
        robot_id=robot_id,
        name=name,
        bridge_version=bridge_version,
        session=session,
        instance=instance,
        cancel=cancel,
        feedback=feedback,
        hmac_required=hmac_required,
        watchdog_mode=watchdog_mode,
        heartbeat_interval_s=heartbeat_interval_s,
        tasks=tasks,
        ts=ts,
        manufacturer=fields.optional_string("manufacturer"),
        model=fields.optional_string("model"),
        sw_version=fields.optional_string("sw_version"),
        hw_version=fields.optional_string("hw_version"),
        serial_number=fields.optional_string("serial_number"),
        suggested_area=fields.optional_string("suggested_area"),
        executor_name=executor_name,
        executor_version=executor_version,
        joints=joints,
        joints_ignored=joints_ignored,
    )


def decode_state(raw: RawPayload) -> RobotState:
    """Decode the retained state payload."""
    data = _parse_object(raw, kind="state", max_bytes=MAX_PAYLOAD_BYTES)
    fields = _Fields(data, "state")
    robot_id = fields.robot_id()
    status = RobotStatus(fields.enum("state", _enum_values(RobotStatus)))
    task_id = fields.optional_task_id("task_id")
    task = fields.optional_task_name("task")
    since = fields.epoch("since")
    paused = fields.boolean("paused")
    inhibited = fields.boolean("inhibited")
    ha_link = HaLink(fields.enum("ha_link", _enum_values(HaLink)))
    watchdog_mode = WatchdogMode(fields.enum("watchdog_mode", _enum_values(WatchdogMode)))
    health = _Fields(fields.obj("executor_health"), "state")
    health_level = HealthLevel(health.enum("level", _enum_values(HealthLevel)))
    health_message = health.string("message")
    error_raw = fields.present("error")
    error_code: str | None = None
    error_reason: str | None = None
    if error_raw is not None:
        if not isinstance(error_raw, dict):
            raise fields.fail("error must be null or an object")
        error = _Fields(error_raw, "state")
        code = error.string("code")
        if not is_valid_result_code(code):
            raise fields.fail("error.code must match ^[a-z][a-z0-9_]{0,63}$")
        error_code = code
        error_reason = error.string("reason")
    session = fields.session()
    instance = fields.instance()
    seq = fields.integer("seq", minimum=0)
    ts = fields.epoch("ts")
    return RobotState(
        robot_id=robot_id,
        status=status,
        task_id=task_id,
        task=task,
        since=since,
        paused=paused,
        inhibited=inhibited,
        ha_link=ha_link,
        watchdog_mode=watchdog_mode,
        executor_health_level=health_level,
        executor_health_message=health_message,
        error_code=error_code,
        error_reason=error_reason,
        session=session,
        instance=instance,
        seq=seq,
        ts=ts,
    )


def decode_heartbeat(raw: RawPayload) -> Heartbeat:
    """Decode the robot heartbeat payload."""
    data = _parse_object(raw, kind="heartbeat", max_bytes=MAX_PAYLOAD_BYTES)
    fields = _Fields(data, "heartbeat")
    robot_id = fields.robot_id()
    session = fields.session()
    instance = fields.instance()
    seq = fields.integer("seq", minimum=0)
    ts = fields.epoch("ts")
    uptime_s = fields.number("uptime_s", minimum=0)
    offset = fields.optional_integer("ha_clock_offset_ms")
    counters_raw = fields.obj("counters")
    counters: dict[str, int] = {}
    for name in COUNTER_NAMES:
        if name not in counters_raw:
            raise fields.fail(f"counters.{name} is missing")
        value = _as_int(counters_raw[name], minimum=0)
        if value is None:
            raise fields.fail(f"counters.{name} must be a non-negative integer")
        counters[name] = value
    for name, value in counters_raw.items():
        if (
            name not in counters
            and isinstance(name, str)
            and _as_int(value, minimum=0) is not None
        ):
            counters[name] = value
    return Heartbeat(
        robot_id=robot_id,
        session=session,
        instance=instance,
        seq=seq,
        ts=ts,
        uptime_s=uptime_s,
        counters=counters,
        ha_clock_offset_ms=offset,
    )


def decode_ack(raw: RawPayload) -> Ack:
    """Decode an ack; ``accepted`` must agree with ``code`` as the schema requires."""
    data = _parse_object(raw, kind="ack", max_bytes=MAX_PAYLOAD_BYTES)
    fields = _Fields(data, "ack")
    command_id = fields.task_id()
    robot_id = fields.robot_id()
    command_type = CommandType(fields.enum("type", _enum_values(CommandType)))
    task = fields.optional_task_name("task")
    accepted = fields.boolean("accepted")
    code = fields.enum("code", ACK_CODES)
    if accepted != (code == ACK_CODE_ACCEPTED):
        raise fields.fail("accepted must be true exactly when code is 'accepted'")
    reason = fields.string("reason")
    received_at = fields.epoch("received_at")
    if "clock_skew_ms" not in data:
        raise fields.fail("clock_skew_ms is missing")
    clock_skew_ms = fields.optional_integer("clock_skew_ms")
    session = fields.session()
    ts = fields.epoch("ts")
    return Ack(
        id=command_id,
        robot_id=robot_id,
        type=command_type,
        task=task,
        accepted=accepted,
        code=code,
        reason=reason,
        received_at=received_at,
        clock_skew_ms=clock_skew_ms,
        session=session,
        ts=ts,
    )


def decode_feedback(raw: RawPayload) -> Feedback:
    """Decode a feedback payload; ``progress`` is 0 to 1 or null."""
    data = _parse_object(raw, kind="feedback", max_bytes=MAX_PAYLOAD_BYTES)
    fields = _Fields(data, "feedback")
    command_id = fields.task_id()
    robot_id = fields.robot_id()
    progress_raw = fields.present("progress")
    progress: float | None = None
    if progress_raw is not None:
        if not _is_number(progress_raw):
            raise fields.fail("progress must be a number or null")
        assert isinstance(progress_raw, (int, float))
        if progress_raw < 0 or progress_raw > 1:
            raise fields.fail("progress must be between 0 and 1")
        progress = float(progress_raw)
    stage = fields.string("stage")
    message = fields.string("message")
    session = fields.session()
    ts = fields.epoch("ts")
    return Feedback(
        id=command_id,
        robot_id=robot_id,
        progress=progress,
        stage=stage,
        message=message,
        session=session,
        ts=ts,
    )


def decode_result(raw: RawPayload) -> Result:
    """Decode a result; ``goal_status`` must agree with ``status`` as the schema requires."""
    data = _parse_object(raw, kind="result", max_bytes=MAX_PAYLOAD_BYTES)
    fields = _Fields(data, "result")
    command_id = fields.task_id()
    robot_id = fields.robot_id()
    task = fields.task_name("task")
    kind = TaskKind(fields.enum("kind", _enum_values(TaskKind)))
    status = ResultStatus(fields.enum("status", _enum_values(ResultStatus)))
    goal_status = fields.integer("goal_status")
    if goal_status != GOAL_STATUS[status]:
        raise fields.fail(f"goal_status must be {GOAL_STATUS[status]} for status '{status}'")
    code = fields.string("code")
    if not is_valid_result_code(code):
        raise fields.fail("code must match ^[a-z][a-z0-9_]{0,63}$")
    reason = fields.string("reason")
    result_data = fields.obj("data")
    started_at = fields.epoch("started_at")
    finished_at = fields.epoch("finished_at")
    session = fields.session()
    ts = fields.epoch("ts")
    return Result(
        id=command_id,
        robot_id=robot_id,
        task=task,
        kind=kind,
        status=status,
        goal_status=goal_status,
        code=code,
        reason=reason,
        data=result_data,
        started_at=started_at,
        finished_at=finished_at,
        session=session,
        ts=ts,
    )


def decode_joints(raw: RawPayload) -> Joints:
    """Decode a joints payload (0.2.0): 1 to 32 positions, velocities null or a list of numbers.

    Whether the positions match the announced names in number is the hub's
    check, because it needs the announce; the schema cannot express it.
    """
    data = _parse_object(raw, kind="joints", max_bytes=MAX_PAYLOAD_BYTES)
    fields = _Fields(data, "joints")
    robot_id = fields.robot_id()
    positions = fields.numbers("positions", min_items=1, max_items=MAX_JOINTS)
    velocities = fields.optional_numbers("velocities", min_items=1, max_items=MAX_JOINTS)
    session = fields.session()
    instance = fields.instance()
    seq = fields.integer("seq", minimum=0)
    ts = fields.epoch("ts")
    return Joints(
        robot_id=robot_id,
        positions=positions,
        velocities=velocities,
        session=session,
        instance=instance,
        seq=seq,
        ts=ts,
    )
