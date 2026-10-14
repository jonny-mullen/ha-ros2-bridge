#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Measure command-to-ack latency and heartbeat jitter against a running bridge node.

The probe needs only paho-mqtt 2.1.0 and a broker the node is connected to; no
ROS, no Home Assistant and no integration code. It makes two measurements and
prints min, median, p95 and max for each, then one JSON summary.

1. Command to ack. ``--count`` task commands are published to
   ``{prefix}/{robot}/cmd`` exactly as Home Assistant would publish them (a
   fresh UUID4 ``id`` and a current ``issued_at`` per command, QoS 1, never
   retained, Message Expiry Interval ``ceil(max_age_ms / 1000)``). The probe
   subscribes to ``{prefix}/{robot}/task/+/ack`` and waits for the SUBACK
   before the first publish, so the retained ack cannot be missed, and
   measures the time from the publish call to the arrival of the ack that
   carries the command's id. By default the task is a name the node cannot
   know (``latency_probe_unknown``), which the node answers
   ``rejected_unknown_task`` at contract step 8, before the executor is ever
   consulted, so nothing runs on the robot. With ``--task`` an announced task
   is sent instead and the node will run it: use that only against the fake
   executor, or with a task you intend to run. The time to the broker's
   PUBACK is recorded alongside as the command's own round trip.
2. Heartbeat period jitter. ``--beats`` consecutive messages on
   ``{prefix}/{robot}/heartbeat`` are timed on arrival. The nominal period is
   read from the robot's retained announce (``heartbeat_interval_s``) when
   one is present, else taken as the median observed period. Reported are the
   period as received, the deviation from the nominal period (jitter), the
   deviation of the received period from the node's own send period (the
   ``ts`` deltas, which isolates the broker and network from the node's
   timer) and any gaps in ``seq``.

Side effects to know about: every command is recorded in the node's dedupe
store and counted in its heartbeat counters (``rejected`` for the default
task); a Home Assistant integration sharing the prefix sees each ack as an
unmatched reply in its diagnostics (``unmatched_replies``) and would open its
``clock_skew`` repair if an ack reported a skew of 1000 ms or more. Rejected
acks beyond the node's ``reply.retained_rejections_per_minute`` (30) are
published without the retain flag; the probe receives them live all the same,
so the measurement is unaffected, but keep ``--count`` under that budget when
the retained acks matter to someone else.

The password is never a command-line argument: it is read from the
environment variable named by ``--password-env`` (default
``ROS2_HA_BRIDGE_MQTT_PASSWORD``). With ``--sign`` every command carries the
contract's HMAC-SHA256 ``sig`` over the topic and the canonical payload, with
the secret read from ``--secret-env`` (default ``ROS2_HA_BRIDGE_COMMAND_SECRET``),
which is the same variable the node reads. The other defaults come from
``MQTT_HOST``, ``MQTT_PORT``, ``MQTT_USERNAME`` and ``TOPIC_PREFIX`` so the
probe runs from ``tools/.env`` with few flags. No host, user or secret is
written into this file.

``--dry-run`` validates every argument, prints the plan (topics, counts,
signing) and exits without opening a connection.

Exit status: 0 when every command was acknowledged and every requested beat
arrived, 1 when the measurement is incomplete (a command without an ack, or
fewer beats than asked for), 2 for a bad argument or a broker that could not
be reached or refused the client.
"""

from __future__ import annotations

import argparse
import hashlib
import hmac
import itertools
import json
import math
import os
import queue
import re
import statistics
import sys
import threading
import time
import uuid
from dataclasses import dataclass, field
from datetime import UTC, datetime
from typing import Any

import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion, MQTTProtocolVersion
from paho.mqtt.packettypes import PacketTypes
from paho.mqtt.properties import Properties

EXIT_OK = 0
EXIT_INCOMPLETE = 1
EXIT_BROKER = 2

DEFAULT_PREFIX = "ros2_bridge"
DEFAULT_PASSWORD_ENV = "ROS2_HA_BRIDGE_MQTT_PASSWORD"
DEFAULT_SECRET_ENV = "ROS2_HA_BRIDGE_COMMAND_SECRET"
#: A task name the node cannot know: rejected at step 8, the executor is never consulted.
DEFAULT_UNKNOWN_TASK = "latency_probe_unknown"
DEFAULT_COUNT = 20
DEFAULT_BEATS = 15
DEFAULT_MAX_AGE_MS = 5000
DEFAULT_INTERVAL_S = 0.25
DEFAULT_ACK_TIMEOUT_S = 5.0
#: Fallback nominal heartbeat period when no announce is retained (the node's default).
DEFAULT_HEARTBEAT_INTERVAL_S = 2.0
#: ``source`` of every command the probe publishes; the node records it and never interprets it.
SOURCE: dict[str, str] = {"origin": "tools", "kind": "latency_probe"}

#: The contract's identifier patterns (``contract/schemas/common.schema.json``).
ROBOT_ID_RE = re.compile(r"[a-z][a-z0-9_]{0,31}")
TASK_NAME_RE = re.compile(r"[a-z][a-z0-9_]{0,31}")
PREFIX_SEGMENT_RE = re.compile(r"[^/+#\x00]+")


# --------------------------------------------------------------------------- helpers


def now_ms() -> int:
    return time.time_ns() // 1_000_000


def prefix_error(prefix: str) -> str | None:
    """Why ``prefix`` is unusable as a topic prefix, or ``None`` when it is fine."""
    if not prefix:
        return "prefix is empty"
    if prefix.startswith("/") or prefix.endswith("/"):
        return "prefix must not start or end with a slash"
    segments = prefix.split("/")
    for segment in segments:
        if PREFIX_SEGMENT_RE.fullmatch(segment) is None:
            return "prefix has an empty segment or a wildcard character"
    if segments[0] in ("ha", "homeassistant"):
        return f"prefix must not start with the reserved segment {segments[0]!r}"
    return None


def canonical_json(payload: dict[str, Any]) -> bytes:
    """The contract's canonical form: sorted keys, compact separators, UTF-8."""
    return json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode(
        "utf-8"
    )


def sign(secret: str, topic: str, payload: dict[str, Any]) -> str:
    """``sig`` for ``payload`` (without a ``sig`` key) published on ``topic``."""
    message = topic.encode("utf-8") + b"\n" + canonical_json(payload)
    return hmac.new(secret.encode("utf-8"), message, hashlib.sha256).hexdigest()


def build_command(
    robot_id: str, task: str, params: dict[str, Any], max_age_ms: int, command_id: str
) -> dict[str, Any]:
    """A schema 1 task command, in the field order Home Assistant uses."""
    return {
        "schema": 1,
        "id": command_id,
        "type": "task",
        "robot_id": robot_id,
        "task": task,
        "params": params,
        "issued_at": now_ms(),
        "max_age_ms": max_age_ms,
        "timeout_s": None,
        "source": dict(SOURCE),
    }


def publish_properties(expiry_s: int) -> Properties:
    # paho 2.1.0 ships py.typed but leaves Properties.__init__ untyped.
    properties = Properties(PacketTypes.PUBLISH)  # type: ignore[no-untyped-call]
    properties.MessageExpiryInterval = expiry_s
    return properties


def percentile(sorted_values: list[float], fraction: float) -> float:
    """Nearest-rank percentile of an already sorted, non-empty list."""
    rank = max(1, math.ceil(fraction * len(sorted_values)))
    return sorted_values[rank - 1]


def summarise(values: list[float]) -> dict[str, float] | None:
    """min, median, p95 and max (rounded to 0.1 ms), or ``None`` for no values."""
    if not values:
        return None
    ordered = sorted(values)
    return {
        "count": len(ordered),
        "min": round(ordered[0], 1),
        "median": round(statistics.median(ordered), 1),
        "p95": round(percentile(ordered, 0.95), 1),
        "max": round(ordered[-1], 1),
    }


def format_stats(label: str, stats: dict[str, float] | None, unit: str = "ms") -> str:
    if stats is None:
        return f"{label}: no samples"
    return (
        f"{label} over {int(stats['count'])} sample(s): min {stats['min']:g} {unit}, "
        f"median {stats['median']:g} {unit}, p95 {stats['p95']:g} {unit}, "
        f"max {stats['max']:g} {unit}"
    )


# --------------------------------------------------------------------------- MQTT side


@dataclass(frozen=True)
class AckSeen:
    """One ack as received: the command id it answers and its fields of interest."""

    command_id: str
    code: str
    clock_skew_ms: int | None
    rx_ns: int


@dataclass(frozen=True)
class BeatSeen:
    """One robot heartbeat as received, with the node's own ``ts`` and ``seq``."""

    ts_ms: int
    seq: int
    session: str
    rx_ns: int


@dataclass
class Observations:
    """Everything the callbacks collected, consumed on the main thread."""

    acks: dict[str, AckSeen] = field(default_factory=dict)
    beats: list[BeatSeen] = field(default_factory=list)
    pubacks: dict[int, int] = field(default_factory=dict)
    foreign_acks: int = 0
    announced_interval_s: float | None = None


class Probe:
    """The paho callbacks and the queue they feed; the main thread does the measuring."""

    def __init__(self, prefix: str, robot_id: str) -> None:
        self.cmd_topic = f"{prefix}/{robot_id}/cmd"
        self.ack_filter = f"{prefix}/{robot_id}/task/+/ack"
        self.heartbeat_topic = f"{prefix}/{robot_id}/heartbeat"
        self.announce_topic = f"{prefix}/{robot_id}/announce"
        self.events: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.connected = threading.Event()
        self.subscribed = threading.Event()
        self.connect_reason: str | None = None
        self.connect_ok = False

    def on_connect(
        self, client: mqtt.Client, userdata: Any, flags: Any, reason_code: Any, properties: Any
    ) -> None:
        code = int(getattr(reason_code, "value", reason_code))
        self.connect_ok = code == 0
        self.connect_reason = f"{reason_code} ({code})"
        self.connected.set()

    def on_connect_fail(self, client: mqtt.Client, userdata: Any) -> None:
        self.connect_reason = "no TCP connection to the broker"
        self.connected.set()

    def on_subscribe(
        self, client: mqtt.Client, userdata: Any, mid: int, reason_codes: Any, properties: Any
    ) -> None:
        self.subscribed.set()

    def on_publish(
        self, client: mqtt.Client, userdata: Any, mid: int, reason_code: Any, properties: Any
    ) -> None:
        self.events.put(("puback", (mid, time.perf_counter_ns())))

    def on_message(self, client: mqtt.Client, userdata: Any, message: Any) -> None:
        rx_ns = time.perf_counter_ns()
        topic = str(message.topic)
        payload = bytes(message.payload)
        if topic == self.heartbeat_topic:
            kind = "beat"
        elif topic == self.announce_topic:
            kind = "announce"
        else:
            kind = "ack"
        self.events.put((kind, (topic, payload, rx_ns)))


def parse_json_object(payload: bytes) -> dict[str, Any] | None:
    try:
        document = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, ValueError):
        return None
    return document if isinstance(document, dict) else None


def _int_field(document: dict[str, Any], key: str) -> int | None:
    value = document.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        return None
    return value


def apply_event(kind: str, data: Any, seen: Observations) -> AckSeen | None:
    """Fold one queue event into ``seen``; returns the ack when the event was one."""
    if kind == "puback":
        mid, t_ns = data
        seen.pubacks[int(mid)] = int(t_ns)
        return None
    _topic, payload, rx_ns = data
    if not payload:
        return None  # an expired or cleared retained message
    document = parse_json_object(payload)
    if document is None:
        return None
    if kind == "announce":
        interval = document.get("heartbeat_interval_s")
        if isinstance(interval, (int, float)) and not isinstance(interval, bool):
            seen.announced_interval_s = float(interval)
        return None
    if kind == "beat":
        ts = _int_field(document, "ts")
        seq = _int_field(document, "seq")
        session = document.get("session")
        if ts is None or seq is None or not isinstance(session, str):
            return None
        seen.beats.append(BeatSeen(ts, seq, session, rx_ns))
        return None
    command_id = document.get("id")
    code = document.get("code")
    if not isinstance(command_id, str) or not isinstance(code, str):
        return None
    ack = AckSeen(command_id, code, _int_field(document, "clock_skew_ms"), rx_ns)
    return ack


def build_client(args: argparse.Namespace, probe: Probe, client_id: str) -> mqtt.Client:
    client = mqtt.Client(
        CallbackAPIVersion.VERSION2, client_id, protocol=MQTTProtocolVersion.MQTTv5
    )
    if args.username:
        client.username_pw_set(args.username, os.environ.get(args.password_env) or None)
    if args.tls:
        client.tls_set(ca_certs=args.tls_ca_cert or None)
        if args.tls_insecure:
            client.tls_insecure_set(True)
    client.on_connect = probe.on_connect
    client.on_connect_fail = probe.on_connect_fail
    client.on_subscribe = probe.on_subscribe
    client.on_publish = probe.on_publish
    client.on_message = probe.on_message
    return client


# --------------------------------------------------------------------------- arguments


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Measure command-to-ack latency and heartbeat jitter against a running "
            "ha-ros2-bridge node."
        )
    )
    parser.add_argument(
        "--host", default=os.environ.get("MQTT_HOST", ""), help="broker host (default: $MQTT_HOST)"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("MQTT_PORT", "1883") or "1883"),
        help="broker port (default: $MQTT_PORT or 1883)",
    )
    parser.add_argument(
        "--username",
        default=os.environ.get("MQTT_USERNAME", ""),
        help="broker user name; empty means anonymous (default: $MQTT_USERNAME)",
    )
    parser.add_argument(
        "--password-env",
        default=DEFAULT_PASSWORD_ENV,
        help=f"environment variable holding the password (default: {DEFAULT_PASSWORD_ENV})",
    )
    parser.add_argument(
        "--prefix",
        default=os.environ.get("TOPIC_PREFIX", DEFAULT_PREFIX) or DEFAULT_PREFIX,
        help=f"topic prefix (default: $TOPIC_PREFIX or {DEFAULT_PREFIX})",
    )
    parser.add_argument(
        "--robot",
        required=True,
        help="robot id of the node under test, for example fake_arm_1",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=DEFAULT_COUNT,
        help=f"commands to send (default: {DEFAULT_COUNT}; 0 skips the command measurement)",
    )
    parser.add_argument(
        "--interval-s",
        type=float,
        default=DEFAULT_INTERVAL_S,
        help=f"pause between commands in seconds (default: {DEFAULT_INTERVAL_S})",
    )
    parser.add_argument(
        "--task",
        default=DEFAULT_UNKNOWN_TASK,
        help=(
            "task name to send; the default is unknown to every node and is rejected before "
            "the executor is consulted. An announced task WILL be run by the node."
        ),
    )
    parser.add_argument(
        "--params",
        default="{}",
        help="JSON object of task parameters for --task (default: {})",
    )
    parser.add_argument(
        "--max-age-ms",
        type=int,
        default=DEFAULT_MAX_AGE_MS,
        help=f"max_age_ms of each command (default: {DEFAULT_MAX_AGE_MS})",
    )
    parser.add_argument(
        "--ack-timeout-s",
        type=float,
        default=DEFAULT_ACK_TIMEOUT_S,
        help=f"how long to wait for each ack (default: {DEFAULT_ACK_TIMEOUT_S})",
    )
    parser.add_argument(
        "--sign",
        action="store_true",
        help="sign every command with the secret in --secret-env (HMAC-SHA256, contract rule)",
    )
    parser.add_argument(
        "--secret-env",
        default=DEFAULT_SECRET_ENV,
        help=f"environment variable holding the shared secret (default: {DEFAULT_SECRET_ENV})",
    )
    parser.add_argument(
        "--beats",
        type=int,
        default=DEFAULT_BEATS,
        help=f"heartbeats to time (default: {DEFAULT_BEATS}; 0 skips the heartbeat measurement)",
    )
    parser.add_argument(
        "--heartbeat-timeout-s",
        type=float,
        default=0.0,
        help="give up waiting for beats after this long (default: (beats + 3) x the period)",
    )
    parser.add_argument("--tls", action="store_true", help="connect with TLS")
    parser.add_argument("--tls-ca-cert", default="", help="CA certificate file for --tls")
    parser.add_argument(
        "--tls-insecure", action="store_true", help="skip hostname verification (testing only)"
    )
    parser.add_argument(
        "--connect-timeout-s", type=float, default=10.0, help="CONNACK wait (default: 10)"
    )
    parser.add_argument("--json", action="store_true", help="print only the JSON summary")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate the arguments, print the plan and exit without connecting",
    )
    return parser.parse_args(argv)


def validate(args: argparse.Namespace) -> tuple[dict[str, Any] | None, str | None]:
    """The parsed task parameters, or the reason the arguments are unusable."""
    if not args.host:
        return None, "give --host or set MQTT_HOST"
    if not 1 <= args.port <= 65535:
        return None, f"--port {args.port} is not a TCP port"
    problem = prefix_error(args.prefix)
    if problem is not None:
        return None, f"{problem}: {args.prefix!r}"
    if ROBOT_ID_RE.fullmatch(args.robot) is None or args.robot == "ha":
        return None, f"--robot {args.robot!r} must match ^[a-z][a-z0-9_]{{0,31}}$ and not be 'ha'"
    if TASK_NAME_RE.fullmatch(args.task) is None:
        return None, f"--task {args.task!r} must match ^[a-z][a-z0-9_]{{0,31}}$"
    try:
        params = json.loads(args.params)
    except ValueError as error:
        return None, f"--params is not valid JSON: {error}"
    if not isinstance(params, dict):
        return None, "--params must be a JSON object"
    if args.count < 0 or args.beats < 0:
        return None, "--count and --beats must not be negative"
    if args.count == 0 and args.beats == 0:
        return None, "nothing to measure: --count and --beats are both 0"
    if args.count > 0:
        if args.interval_s < 0:
            return None, "--interval-s must not be negative"
        if args.max_age_ms < 1:
            return None, "--max-age-ms must be at least 1"
        if args.ack_timeout_s <= 0:
            return None, "--ack-timeout-s must be greater than 0"
    if args.heartbeat_timeout_s < 0:
        return None, "--heartbeat-timeout-s must not be negative"
    if args.sign and not os.environ.get(args.secret_env):
        return (
            None,
            f"--sign needs the shared secret in the environment variable {args.secret_env}",
        )
    if args.username and not os.environ.get(args.password_env):
        return None, (
            f"--username is set but the environment variable {args.password_env} is empty; "
            "the password is never a command-line argument"
        )
    return params, None


def plan_of(args: argparse.Namespace, probe: Probe, params: dict[str, Any]) -> dict[str, Any]:
    return {
        "broker": f"{args.host}:{args.port}",
        "username": args.username or None,
        "tls": bool(args.tls),
        "prefix": args.prefix,
        "robot_id": args.robot,
        "cmd_topic": probe.cmd_topic,
        "ack_filter": probe.ack_filter,
        "heartbeat_topic": probe.heartbeat_topic,
        "announce_topic": probe.announce_topic,
        "commands": {
            "count": args.count,
            "task": args.task,
            "params": params,
            "executor_touched": args.task != DEFAULT_UNKNOWN_TASK,
            "max_age_ms": args.max_age_ms,
            "expiry_s": math.ceil(args.max_age_ms / 1000),
            "interval_s": args.interval_s,
            "ack_timeout_s": args.ack_timeout_s,
            "signed": bool(args.sign),
            "secret_env": args.secret_env if args.sign else None,
        },
        "heartbeat": {"beats": args.beats, "timeout_s": args.heartbeat_timeout_s or None},
    }


def print_plan(plan: dict[str, Any]) -> None:
    commands = plan["commands"]
    heartbeat = plan["heartbeat"]
    print(f"latency_probe: plan for robot {plan['robot_id']!r} via {plan['broker']}")
    print(f"  user {plan['username'] or '(anonymous)'}, TLS {'on' if plan['tls'] else 'off'}")
    print(f"  publish {commands['count']} command(s) to {plan['cmd_topic']}")
    if commands["count"]:
        touched = (
            "the node WILL run this task"
            if commands["executor_touched"]
            else "unknown to the node, rejected before the executor"
        )
        print(f"  task {commands['task']!r} with params {commands['params']}: {touched}")
        print(
            f"  max_age_ms {commands['max_age_ms']} (expiry {commands['expiry_s']} s), "
            f"{commands['interval_s']:g} s between commands, ack wait {commands['ack_timeout_s']:g} s"
        )
        signed = (
            f"signed with the secret from {commands['secret_env']}"
            if commands["signed"]
            else "unsigned"
        )
        print(f"  commands {signed}; acks awaited on {plan['ack_filter']}")
    print(f"  time {heartbeat['beats']} heartbeat(s) on {plan['heartbeat_topic']}")
    print(f"  nominal period from the retained announce on {plan['announce_topic']} when present")
    print("  dry run: no connection was opened")


# --------------------------------------------------------------------------- measuring


def run(args: argparse.Namespace) -> int:
    params, problem = validate(args)
    if problem is not None or params is None:
        print(f"latency_probe: {problem}", file=sys.stderr)
        return EXIT_BROKER
    probe = Probe(args.prefix, args.robot)
    plan = plan_of(args, probe, params)
    if args.dry_run:
        if args.json:
            print(json.dumps({"dry_run": True, "plan": plan}, indent=2))
        else:
            print_plan(plan)
            print("plan JSON: " + json.dumps(plan, separators=(",", ":")))
        return EXIT_OK

    client_id = f"ros2_ha_bridge_latency_probe_{uuid.uuid4().hex[:12]}"
    client = build_client(args, probe, client_id)

    def say(text: str) -> None:
        """Progress lines go to stdout unless only the JSON summary was asked for."""
        if not args.json:
            print(text)

    say(f"latency_probe: connecting to {args.host}:{args.port} as {client_id}")
    try:
        client.connect(args.host, args.port, keepalive=15, clean_start=True)
    except OSError as error:
        print(f"latency_probe: cannot connect: {error}", file=sys.stderr)
        return EXIT_BROKER
    client.loop_start()
    try:
        return measure(args, client, probe, params, plan, say)
    finally:
        client.disconnect()
        client.loop_stop()


def measure(
    args: argparse.Namespace,
    client: mqtt.Client,
    probe: Probe,
    params: dict[str, Any],
    plan: dict[str, Any],
    say: Any,
) -> int:
    if not probe.connected.wait(args.connect_timeout_s) or not probe.connect_ok:
        reason = probe.connect_reason or f"no CONNACK within {args.connect_timeout_s:g} s"
        print(f"latency_probe: broker refused or did not answer: {reason}", file=sys.stderr)
        return EXIT_BROKER
    result, _mid = client.subscribe(
        [(probe.ack_filter, 1), (probe.heartbeat_topic, 0), (probe.announce_topic, 1)]
    )
    if result != mqtt.MQTT_ERR_SUCCESS or not probe.subscribed.wait(args.connect_timeout_s):
        print("latency_probe: subscribe failed or was not acknowledged", file=sys.stderr)
        return EXIT_BROKER

    seen = Observations()
    secret = os.environ.get(args.secret_env, "") if args.sign else ""

    def drain_until(deadline: float, wanted_id: str | None) -> AckSeen | None:
        """Consume events until the wanted ack arrives or the deadline passes."""
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            try:
                kind, data = probe.events.get(timeout=remaining)
            except queue.Empty:
                return None
            ack = apply_event(kind, data, seen)
            if ack is None:
                continue
            if wanted_id is not None and ack.command_id == wanted_id:
                return ack
            seen.foreign_acks += 1

    # Phase A: commands. Beats that arrive meanwhile are kept for phase B.
    latencies: list[float] = []
    puback_latencies: list[float] = []
    skews: list[float] = []
    codes: dict[str, int] = {}
    missing: list[str] = []
    expiry = math.ceil(args.max_age_ms / 1000)
    for index in range(args.count):
        command_id = str(uuid.uuid4())
        payload = build_command(args.robot, args.task, params, args.max_age_ms, command_id)
        if secret:
            payload["sig"] = sign(secret, probe.cmd_topic, payload)
        wire = json.dumps(payload).encode("utf-8")
        t0 = time.perf_counter_ns()
        info = client.publish(
            probe.cmd_topic, wire, qos=1, retain=False, properties=publish_properties(expiry)
        )
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            print(f"latency_probe: publish failed: {mqtt.error_string(info.rc)}", file=sys.stderr)
            return EXIT_BROKER
        ack = drain_until(time.monotonic() + args.ack_timeout_s, command_id)
        if ack is None:
            missing.append(command_id)
            say(f"latency_probe: command {index + 1}/{args.count} {command_id}: no ack")
        else:
            seen.acks[command_id] = ack
            latency_ms = (ack.rx_ns - t0) / 1e6
            latencies.append(latency_ms)
            codes[ack.code] = codes.get(ack.code, 0) + 1
            if ack.clock_skew_ms is not None:
                skews.append(float(ack.clock_skew_ms))
            puback_ns = seen.pubacks.get(int(info.mid))
            if puback_ns is not None:
                puback_latencies.append((puback_ns - t0) / 1e6)
            say(
                f"latency_probe: command {index + 1}/{args.count}: {ack.code} after "
                f"{latency_ms:.1f} ms"
            )
        if args.interval_s > 0 and index + 1 < args.count:
            drain_until(time.monotonic() + args.interval_s, None)

    # Phase B: heartbeats.
    nominal_s = seen.announced_interval_s or DEFAULT_HEARTBEAT_INTERVAL_S
    if args.beats > 0:
        timeout_s = args.heartbeat_timeout_s or max(10.0, (args.beats + 3) * nominal_s)
        deadline = time.monotonic() + timeout_s
        say(
            f"latency_probe: waiting for {args.beats} heartbeat(s) on {probe.heartbeat_topic} "
            f"(up to {timeout_s:g} s)"
        )
        while len(seen.beats) < args.beats and time.monotonic() < deadline:
            drain_until(min(deadline, time.monotonic() + 1.0), None)

    return report(args, seen, plan, latencies, puback_latencies, skews, codes, missing, say)


def heartbeat_summary(beats: list[BeatSeen], nominal_s: float | None) -> dict[str, Any]:
    """Period, jitter against the nominal period, transport jitter and seq gaps."""
    periods: list[float] = []
    send_periods: list[float] = []
    transport: list[float] = []
    gaps = 0
    sessions = {beat.session for beat in beats}
    for previous, current in itertools.pairwise(beats):
        if current.session != previous.session:
            continue
        period = (current.rx_ns - previous.rx_ns) / 1e6
        periods.append(period)
        sent = float(current.ts_ms - previous.ts_ms)
        send_periods.append(sent)
        transport.append(period - sent)
        if current.seq - previous.seq != 1:
            gaps += 1
    if nominal_s is None:
        nominal_ms = round(statistics.median(periods)) if periods else None
        nominal_source = "median observed period" if periods else None
    else:
        nominal_ms = round(nominal_s * 1000)
        nominal_source = "announce.heartbeat_interval_s"
    jitter = [period - nominal_ms for period in periods] if nominal_ms is not None else []
    return {
        "beats": len(beats),
        "sessions_seen": len(sessions),
        "nominal_ms": nominal_ms,
        "nominal_source": nominal_source,
        "period_ms": summarise(periods),
        "jitter_abs_ms": summarise([abs(value) for value in jitter]),
        "jitter_signed_ms": (
            {"min": round(min(jitter), 1), "max": round(max(jitter), 1)} if jitter else None
        ),
        "send_period_ms": summarise(send_periods),
        "transport_jitter_abs_ms": summarise([abs(value) for value in transport]),
        "seq_gaps": gaps,
    }


def report(
    args: argparse.Namespace,
    seen: Observations,
    plan: dict[str, Any],
    latencies: list[float],
    puback_latencies: list[float],
    skews: list[float],
    codes: dict[str, int],
    missing: list[str],
    say: Any,
) -> int:
    commands = {
        "task": args.task,
        "signed": bool(args.sign),
        "sent": args.count,
        "acked": len(latencies),
        "missing": len(missing),
        "codes": codes,
        "command_to_ack_ms": summarise(latencies),
        "publish_to_puback_ms": summarise(puback_latencies),
        "node_clock_skew_ms": summarise(skews),
        "foreign_acks_seen": seen.foreign_acks,
    }
    heartbeat = heartbeat_summary(seen.beats, seen.announced_interval_s)
    incomplete = bool(missing) or len(seen.beats) < args.beats
    summary = {
        "probe": "latency_probe",
        "summary_version": 1,
        "measured_at": datetime.now(UTC).isoformat(timespec="seconds"),
        "broker": plan["broker"],
        "prefix": args.prefix,
        "robot_id": args.robot,
        "commands": commands,
        "heartbeat": heartbeat,
        "exit_code": EXIT_INCOMPLETE if incomplete else EXIT_OK,
    }
    if args.count:
        say(format_stats("latency_probe: command to ack", commands["command_to_ack_ms"]))
        say(format_stats("latency_probe: publish to PUBACK", commands["publish_to_puback_ms"]))
        say(
            format_stats(
                "latency_probe: clock skew reported by the node", commands["node_clock_skew_ms"]
            )
        )
        say(f"latency_probe: ack codes {codes or '{}'}; {len(missing)} command(s) without an ack")
    if args.beats:
        nominal = heartbeat["nominal_ms"]
        source = heartbeat["nominal_source"]
        say(
            f"latency_probe: {heartbeat['beats']} heartbeat(s), nominal period "
            f"{nominal if nominal is not None else 'unknown'} ms ({source or 'no sample'})"
        )
        say(format_stats("latency_probe: heartbeat period as received", heartbeat["period_ms"]))
        say(format_stats("latency_probe: heartbeat jitter (abs)", heartbeat["jitter_abs_ms"]))
        say(
            format_stats(
                "latency_probe: transport jitter (received minus sent period, abs)",
                heartbeat["transport_jitter_abs_ms"],
            )
        )
        say(
            f"latency_probe: seq gaps {heartbeat['seq_gaps']}, sessions seen "
            f"{heartbeat['sessions_seen']}"
        )
    if incomplete:
        print("latency_probe: INCOMPLETE: see missing acks or beats above", file=sys.stderr)
    if args.json:
        print(json.dumps(summary, indent=2))
    else:
        print("JSON summary: " + json.dumps(summary, separators=(",", ":")))
    return EXIT_INCOMPLETE if incomplete else EXIT_OK


def main(argv: list[str] | None = None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
