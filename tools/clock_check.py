#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Check this machine's clock against Home Assistant's through the MQTT broker.

Commands carry an issue time and a maximum age, and the node accepts them only within
``command.clock_skew_tolerance_ms`` (2 s by default) of its own clock, so the robot
machine and the Home Assistant machine must agree on the time. This probe needs only
paho-mqtt 2.x and a broker; no ROS, no integration code. It makes two measurements:

1. Round trip. A few timestamped probes are published to
   ``{prefix}/ha/probe/{client_id}`` and read straight back. The time between
   sending and receiving is the broker round-trip time: it bounds how precisely
   the offset can be measured, and it proves that the broker accepts this client's
   publishes and subscriptions. The topic sits under the reserved ``ha`` segment
   where neither the node nor the integration subscribes, so the probe disturbs
   nothing.
2. Offset. Once the Home Assistant integration is installed it publishes
   ``{prefix}/ha/heartbeat`` every two seconds with its own ``ts`` (epoch
   milliseconds). The offset is ``local receive time - ts - rtt / 2``: positive
   means this machine's clock is ahead of Home Assistant's.

Exit status: 0 when the offset is within the threshold (or no heartbeat was seen and
none was required), 1 when the offset exceeds the threshold, 2 when the broker could
not be reached or refused the client, 3 when ``--require-heartbeat`` was given and no
heartbeat arrived.

The password is never a command-line argument: it is read from the environment
variable named by ``--password-env`` (default ``ROS2_HA_BRIDGE_MQTT_PASSWORD``). The
other defaults come from ``MQTT_HOST``, ``MQTT_PORT``, ``MQTT_USERNAME`` and
``TOPIC_PREFIX`` so ``docker compose run --rm clock-check`` works with no arguments.
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import re
import statistics
import sys
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Any

import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion, MQTTProtocolVersion
from paho.mqtt.packettypes import PacketTypes
from paho.mqtt.properties import Properties

EXIT_OK = 0
EXIT_OFFSET = 1
EXIT_BROKER = 2
EXIT_NO_HEARTBEAT = 3

DEFAULT_PREFIX = "ros2_bridge"
DEFAULT_PASSWORD_ENV = "ROS2_HA_BRIDGE_MQTT_PASSWORD"
PROBE_EXPIRY_S = 10
#: The contract's prefix rules, enough of them for a probe: no slash at either end, no
#: wildcard, no empty segment, not the discovery prefix and not the reserved segment.
PREFIX_SEGMENT_RE = re.compile(r"[^/+#\x00]+")


@dataclass(frozen=True)
class ProbeEcho:
    """One probe that came back: its sequence number and the measured round trip."""

    seq: int
    rtt_ms: int


@dataclass(frozen=True)
class Heartbeat:
    """One Home Assistant heartbeat as received, with the local receive time."""

    ts_ms: int
    seq: int
    interval_ms: int
    instance_id: str
    rx_ms: int


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


class Probe:
    """The paho callbacks and the queue they feed; the main thread does the measuring."""

    def __init__(self, prefix: str, client_id: str) -> None:
        self.probe_topic = f"{prefix}/ha/probe/{client_id}"
        self.heartbeat_topic = f"{prefix}/ha/heartbeat"
        self.events: queue.Queue[tuple[str, bytes, int]] = queue.Queue()
        self.connected = threading.Event()
        self.subscribed = threading.Event()
        self.connect_reason: str | None = None
        self.connect_ok = False
        self.disconnect_reason: str | None = None

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

    def on_disconnect(
        self, client: mqtt.Client, userdata: Any, flags: Any, reason_code: Any, properties: Any
    ) -> None:
        self.disconnect_reason = str(reason_code)

    def on_subscribe(
        self, client: mqtt.Client, userdata: Any, mid: int, reason_codes: Any, properties: Any
    ) -> None:
        self.subscribed.set()

    def on_message(self, client: mqtt.Client, userdata: Any, message: Any) -> None:
        self.events.put((str(message.topic), bytes(message.payload), now_ms()))


def parse_heartbeat(payload: bytes, rx_ms: int) -> Heartbeat | None:
    """The HA heartbeat fields the probe needs, or ``None`` when the payload is not one."""
    try:
        document = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, ValueError):
        return None
    if not isinstance(document, dict):
        return None
    ts = document.get("ts")
    seq = document.get("seq")
    interval = document.get("interval_ms")
    instance_id = document.get("instance_id")
    if not isinstance(ts, int) or not isinstance(seq, int) or not isinstance(interval, int):
        return None
    if isinstance(ts, bool) or isinstance(seq, bool) or isinstance(interval, bool):
        return None
    if not isinstance(instance_id, str):
        return None
    return Heartbeat(ts, seq, interval, instance_id, rx_ms)


def parse_probe(payload: bytes, rx_ms: int, client_id: str) -> ProbeEcho | None:
    """Our own probe coming back, or ``None`` for anything else on the probe topic."""
    try:
        document = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, ValueError):
        return None
    if not isinstance(document, dict) or document.get("probe") != client_id:
        return None
    seq = document.get("seq")
    sent = document.get("sent_ms")
    if not isinstance(seq, int) or not isinstance(sent, int):
        return None
    return ProbeEcho(seq, rx_ms - sent)


def publish_properties(expiry_s: int) -> Properties:
    # paho 2.1.0 ships py.typed but leaves Properties.__init__ untyped.
    properties = Properties(PacketTypes.PUBLISH)  # type: ignore[no-untyped-call]
    properties.MessageExpiryInterval = expiry_s
    return properties


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
    client.on_disconnect = probe.on_disconnect
    client.on_subscribe = probe.on_subscribe
    client.on_message = probe.on_message
    return client


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Measure the MQTT round trip and the clock offset against Home Assistant."
    )
    parser.add_argument(
        "--host",
        default=os.environ.get("MQTT_HOST", ""),
        help="broker host (default: $MQTT_HOST)",
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
    parser.add_argument("--tls", action="store_true", help="connect with TLS")
    parser.add_argument("--tls-ca-cert", default="", help="CA certificate file for --tls")
    parser.add_argument(
        "--tls-insecure", action="store_true", help="skip hostname verification (testing only)"
    )
    parser.add_argument("--probes", type=int, default=5, help="round-trip probes to send")
    parser.add_argument(
        "--heartbeat-wait-s",
        type=float,
        default=6.0,
        help="how long to wait for a Home Assistant heartbeat (default: 6, three intervals)",
    )
    parser.add_argument(
        "--threshold-s",
        type=float,
        default=1.0,
        help="largest acceptable absolute offset in seconds (default: 1.0)",
    )
    parser.add_argument(
        "--require-heartbeat",
        action="store_true",
        help="exit 3 when no heartbeat arrives instead of reporting that none was seen",
    )
    parser.add_argument(
        "--connect-timeout-s", type=float, default=10.0, help="CONNACK wait (default: 10)"
    )
    return parser.parse_args(argv)


def run(args: argparse.Namespace) -> int:
    if not args.host:
        print("clock_check: give --host or set MQTT_HOST", file=sys.stderr)
        return EXIT_BROKER
    problem = prefix_error(args.prefix)
    if problem is not None:
        print(f"clock_check: {problem}: {args.prefix!r}", file=sys.stderr)
        return EXIT_BROKER
    if args.probes < 1:
        print("clock_check: --probes must be at least 1", file=sys.stderr)
        return EXIT_BROKER

    client_id = f"ros2_ha_bridge_clock_check_{uuid.uuid4().hex[:12]}"
    probe = Probe(args.prefix, client_id)
    client = build_client(args, probe, client_id)
    print(f"clock_check: connecting to {args.host}:{args.port} as {client_id}")
    try:
        client.connect(args.host, args.port, keepalive=15, clean_start=True)
    except OSError as error:
        print(f"clock_check: cannot connect: {error}", file=sys.stderr)
        return EXIT_BROKER
    client.loop_start()
    try:
        return measure(args, client, probe, client_id)
    finally:
        client.disconnect()
        client.loop_stop()


def measure(args: argparse.Namespace, client: mqtt.Client, probe: Probe, client_id: str) -> int:
    if not probe.connected.wait(args.connect_timeout_s) or not probe.connect_ok:
        reason = probe.connect_reason or f"no CONNACK within {args.connect_timeout_s:g} s"
        print(f"clock_check: broker refused or did not answer: {reason}", file=sys.stderr)
        return EXIT_BROKER
    result, _ = client.subscribe([(probe.probe_topic, 1), (probe.heartbeat_topic, 0)])
    if result != mqtt.MQTT_ERR_SUCCESS or not probe.subscribed.wait(args.connect_timeout_s):
        print("clock_check: subscribe failed or was not acknowledged", file=sys.stderr)
        return EXIT_BROKER

    echoes: dict[int, ProbeEcho] = {}
    heartbeats: list[Heartbeat] = []

    def drain_until(deadline: float, wanted_seq: int | None) -> bool:
        """Consume events until ``wanted_seq`` echoes (True) or the deadline passes (False)."""
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            try:
                topic, payload, rx_ms = probe.events.get(timeout=remaining)
            except queue.Empty:
                return False
            if topic == probe.heartbeat_topic:
                beat = parse_heartbeat(payload, rx_ms)
                if beat is not None:
                    heartbeats.append(beat)
                continue
            echo = parse_probe(payload, rx_ms, client_id)
            if echo is not None:
                echoes[echo.seq] = echo
                if wanted_seq is not None and echo.seq == wanted_seq:
                    return True

    for seq in range(args.probes):
        payload = json.dumps({"probe": client_id, "seq": seq, "sent_ms": now_ms()})
        info = client.publish(
            probe.probe_topic, payload, qos=1, properties=publish_properties(PROBE_EXPIRY_S)
        )
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            print(f"clock_check: publish failed: {mqtt.error_string(info.rc)}", file=sys.stderr)
            return EXIT_BROKER
        if not drain_until(time.monotonic() + 5.0, seq):
            print(f"clock_check: probe {seq} did not come back within 5 s", file=sys.stderr)
            return EXIT_BROKER

    rtts = sorted(echo.rtt_ms for echo in echoes.values())
    rtt_median = statistics.median(rtts)
    print(
        f"clock_check: round trip through the broker over {len(rtts)} probe(s): "
        f"min {rtts[0]} ms, median {rtt_median:g} ms, max {rtts[-1]} ms"
    )

    if not heartbeats:
        print(
            f"clock_check: waiting up to {args.heartbeat_wait_s:g} s for a Home Assistant "
            f"heartbeat on {probe.heartbeat_topic}"
        )
        drain_until(time.monotonic() + args.heartbeat_wait_s, None)
    if not heartbeats:
        message = (
            "clock_check: no Home Assistant heartbeat seen; the integration is not installed, "
            "not connected, or uses another prefix. Only the round trip could be measured."
        )
        if args.require_heartbeat:
            print(message, file=sys.stderr)
            return EXIT_NO_HEARTBEAT
        print(message)
        return EXIT_OK

    beat = heartbeats[-1]
    offset_ms = beat.rx_ms - beat.ts_ms - round(rtt_median / 2)
    ahead_or_behind = "ahead of" if offset_ms >= 0 else "behind"
    print(
        f"clock_check: Home Assistant heartbeat seq {beat.seq} from instance "
        f"{beat.instance_id} (interval {beat.interval_ms} ms)"
    )
    print(
        f"clock_check: this machine's clock is {abs(offset_ms)} ms {ahead_or_behind} "
        f"Home Assistant's (offset {offset_ms:+d} ms, half the median round trip removed)"
    )
    threshold_ms = round(args.threshold_s * 1000)
    if abs(offset_ms) > threshold_ms:
        print(
            f"clock_check: FAIL: the offset exceeds {threshold_ms} ms; check NTP on both "
            "machines before relying on command freshness",
            file=sys.stderr,
        )
        return EXIT_OFFSET
    print(f"clock_check: OK: within the {threshold_ms} ms threshold")
    return EXIT_OK


def main(argv: list[str] | None = None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
