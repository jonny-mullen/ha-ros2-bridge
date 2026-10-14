# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Shared helpers for the Home Assistant integration tests.

Plain functions and classes only; the fixtures live in ``conftest.py``. The
golden examples and HMAC vectors under ``contract/`` are the only hand-written
fixtures the suite uses, so every payload builder here starts from one of them.
"""

from __future__ import annotations

import json
from collections.abc import Callable, Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from homeassistant.core import HomeAssistant
from jsonschema import Draft202012Validator
from jsonschema.exceptions import best_match
from pytest_homeassistant_custom_component.common import async_fire_mqtt_message
from referencing import Registry
from referencing.jsonschema import DRAFT202012

from custom_components.ros2_bridge import protocol
from custom_components.ros2_bridge.const import DOMAIN
from custom_components.ros2_bridge.models import (
    Ack,
    Announce,
    Feedback,
    Heartbeat,
    Result,
    RobotState,
)

REPO_ROOT = Path(__file__).resolve().parent.parent
CONTRACT_DIR = REPO_ROOT / "contract"
SCHEMAS_DIR = CONTRACT_DIR / "schemas"
EXAMPLES_DIR = CONTRACT_DIR / "examples"
VECTORS_PATH = CONTRACT_DIR / "hmac_vectors.json"
ROS_PACKAGE_DIR = REPO_ROOT / "ros2" / "ros2_ha_bridge"

SCHEMA_ID_BASE = "https://ha-ros2-bridge.invalid/contract/1/"
KINDS: tuple[str, ...] = (
    "announce",
    "availability",
    "state",
    "heartbeat_robot",
    "heartbeat_ha",
    "command",
    "ack",
    "feedback",
    "result",
    "sensor",
)
#: Kinds the HA side decodes, with the protocol function that decodes each.
INBOUND_DECODERS: dict[str, Callable[[bytes], Any]] = {
    "announce": protocol.decode_announce,
    "availability": protocol.decode_availability,
    "state": protocol.decode_state,
    "heartbeat_robot": protocol.decode_heartbeat,
    "ack": protocol.decode_ack,
    "feedback": protocol.decode_feedback,
    "result": protocol.decode_result,
}

#: The placeholder secret of ``contract/hmac_vectors.json`` and the signed golden commands.
VECTOR_SECRET = "example-shared-secret-change-me"
#: The golden examples' epoch (2025-10-13T12:00:00Z) in milliseconds.
EPOCH_MS = 1760356800000
PREFIX = "ros2_bridge"
ROBOT_ID = "arm_left"
SESSION = "9f1c7a0e-4a1b-4b2c-8d1e-0f1a2b3c4d5e"
OTHER_SESSION = "3e7a1c9b-5d2f-4b8e-a1c3-6f0d2e4a8b7c"
INSTANCE = "robotpc:4123"
TASK_ID = "6a0c2b58-1d7e-4a1f-9b2a-3c4d5e6f7a8b"
RESET_ID = "51f0a2b3-c4d5-4e6f-a7b8-c9d0e1f2a3b4"
CANCEL_ID = "0c1d4e6f-2a3b-4c5d-8e9f-0a1b2c3d4e5f"
WAIT_ID = "7b3e9d21-5f6a-4c8b-9e0d-1f2a3b4c5d6e"
INSTANCE_ID = "c2a4f6e8-1b3d-4f5a-9c7e-2d4f6a8b0c1e"


# --------------------------------------------------------------------------- golden loaders


@dataclass(frozen=True)
class GoldenExample:
    """One file from ``contract/examples``."""

    kind: str
    name: str
    path: Path
    payload: Any
    #: The file's bytes exactly as stored (pretty-printed JSON).
    raw: bytes
    valid: bool

    @property
    def wire(self) -> bytes:
        """What the broker would deliver: the raw file, or the bare word for availability."""
        if self.kind == "availability" and isinstance(self.payload, str):
            return self.payload.encode("utf-8")
        return self.raw


def load_examples(kind: str, valid: bool) -> list[GoldenExample]:
    """Every example of ``kind`` from the valid or invalid folder, sorted by name."""
    folder = EXAMPLES_DIR / ("valid" if valid else "invalid") / kind
    examples: list[GoldenExample] = []
    for path in sorted(folder.glob("*.json")):
        raw = path.read_bytes()
        examples.append(GoldenExample(kind, path.stem, path, json.loads(raw), raw, valid))
    return examples


def load_example(kind: str, name: str, *, valid: bool = True) -> GoldenExample:
    """One named example."""
    for example in load_examples(kind, valid):
        if example.name == name:
            return example
    raise FileNotFoundError(f"{kind}/{name}")


def golden(kind: str, name: str) -> dict[str, Any]:
    """A deep copy of a valid example's payload, ready to be edited."""
    payload = load_example(kind, name).payload
    assert isinstance(payload, dict)
    return json.loads(json.dumps(payload))


def load_vectors() -> list[dict[str, Any]]:
    """The HMAC vectors, each with secret, topic, payload, canonical and sig."""
    with VECTORS_PATH.open(encoding="utf-8") as handle:
        document = json.load(handle)
    vectors: list[dict[str, Any]] = list(document["vectors"])
    return vectors


# --------------------------------------------------------------------------- schema validation


def _registry() -> Registry:
    resources = []
    for path in sorted(SCHEMAS_DIR.glob("*.schema.json")):
        schema = json.loads(path.read_text(encoding="utf-8"))
        resources.append((schema["$id"], DRAFT202012.create_resource(schema)))
    return Registry().with_resources(resources)


_REGISTRY = _registry()
_VALIDATORS: dict[str, Draft202012Validator] = {}


def schema_validator(kind: str) -> Draft202012Validator:
    """A draft 2020-12 validator for one payload kind, with the shared definitions resolving."""
    if kind not in _VALIDATORS:
        schema = json.loads((SCHEMAS_DIR / f"{kind}.schema.json").read_text(encoding="utf-8"))
        _VALIDATORS[kind] = Draft202012Validator(schema, registry=_REGISTRY)
    return _VALIDATORS[kind]


def validate(kind: str, payload: Any) -> list[str]:
    """Schema problems for ``payload`` as ``kind``; empty means valid."""
    errors = list(schema_validator(kind).iter_errors(payload))
    if not errors:
        return []
    best = best_match(errors)
    return [f"{'/'.join(str(p) for p in best.absolute_path) or '<root>'}: {best.message}"]


# --------------------------------------------------------------------------- clock


class FakeClock:
    """A clock callable in epoch milliseconds that moves only when told to."""

    def __init__(self, start_ms: int = EPOCH_MS) -> None:
        self.now_ms = start_ms

    def __call__(self) -> int:
        return self.now_ms

    def advance(self, ms: int) -> int:
        self.now_ms += ms
        return self.now_ms

    def advance_s(self, seconds: float) -> int:
        return self.advance(int(seconds * 1000))


# --------------------------------------------------------------------------- payload builders


def _with(base: dict[str, Any], overrides: dict[str, Any]) -> dict[str, Any]:
    payload = dict(base)
    for key, value in overrides.items():
        if value is ... and key in payload:
            del payload[key]
        else:
            payload[key] = value
    return payload


def announce_payload(**overrides: Any) -> dict[str, Any]:
    """The full catalogue announce with fields overridden (``...`` removes a key)."""
    return _with(golden("announce", "full_catalogue"), overrides)


def state_payload(**overrides: Any) -> dict[str, Any]:
    return _with(golden("state", "busy_move_to"), overrides)


def idle_state_payload(**overrides: Any) -> dict[str, Any]:
    return _with(golden("state", "idle"), overrides)


def heartbeat_payload(**overrides: Any) -> dict[str, Any]:
    return _with(golden("heartbeat_robot", "steady"), overrides)


def ack_payload(**overrides: Any) -> dict[str, Any]:
    return _with(golden("ack", "accepted_task"), overrides)


def rejected_ack_payload(**overrides: Any) -> dict[str, Any]:
    return _with(golden("ack", "rejected_busy"), overrides)


def cancel_ack_payload(**overrides: Any) -> dict[str, Any]:
    return _with(golden("ack", "accepted_cancel"), overrides)


def feedback_payload(**overrides: Any) -> dict[str, Any]:
    return _with(golden("feedback", "executing_waypoint"), overrides)


def result_payload(**overrides: Any) -> dict[str, Any]:
    """The succeeded ``reset`` result, re-pointed at ``TASK_ID``/``move_to`` unless overridden."""
    base = golden("result", "succeeded_reset")
    base.update({"id": TASK_ID, "task": "move_to"})
    return _with(base, overrides)


def encode(payload: Any) -> bytes:
    """Compact UTF-8 JSON, the way the node publishes."""
    return json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def make_announce(**overrides: Any) -> Announce:
    return protocol.decode_announce(encode(announce_payload(**overrides)))


def make_state(**overrides: Any) -> RobotState:
    return protocol.decode_state(encode(state_payload(**overrides)))


def make_heartbeat(**overrides: Any) -> Heartbeat:
    return protocol.decode_heartbeat(encode(heartbeat_payload(**overrides)))


def make_ack(**overrides: Any) -> Ack:
    return protocol.decode_ack(encode(ack_payload(**overrides)))


def make_rejected_ack(**overrides: Any) -> Ack:
    return protocol.decode_ack(encode(rejected_ack_payload(**overrides)))


def make_cancel_ack(**overrides: Any) -> Ack:
    return protocol.decode_ack(encode(cancel_ack_payload(**overrides)))


def make_feedback(**overrides: Any) -> Feedback:
    return protocol.decode_feedback(encode(feedback_payload(**overrides)))


def make_result(**overrides: Any) -> Result:
    return protocol.decode_result(encode(result_payload(**overrides)))


# --------------------------------------------------------------------------- MQTT helpers


def robot_topic(suffix: str, robot_id: str = ROBOT_ID, prefix: str = PREFIX) -> str:
    return f"{prefix}/{robot_id}/{suffix}"


def fire_robot_message(
    hass: HomeAssistant,
    suffix: str,
    payload: Any,
    *,
    robot_id: str = ROBOT_ID,
    prefix: str = PREFIX,
    retain: bool = False,
    qos: int = 0,
) -> None:
    """Deliver a message on ``{prefix}/{robot_id}/{suffix}`` through the mocked MQTT client.

    ``payload`` may be bytes, a string (sent as UTF-8) or a dict (compact JSON).
    """
    if isinstance(payload, (dict, list)):
        raw: bytes | str = encode(payload)
    else:
        raw = payload
    async_fire_mqtt_message(hass, robot_topic(suffix, robot_id, prefix), raw, qos, retain)


def device_identifiers(robot_id: str = ROBOT_ID) -> set[tuple[str, str]]:
    return {(DOMAIN, robot_id)}


def iter_kinds() -> Iterator[str]:
    yield from KINDS
