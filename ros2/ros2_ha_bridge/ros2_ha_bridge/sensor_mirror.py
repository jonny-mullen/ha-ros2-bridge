# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Fast sensor path: mirror chosen MQTT topics onto a ROS topic as JSON envelopes.

The node subscribes to every entry of the ``mirror.topics`` parameter (a plain
topic such as ``zigbee2mqtt/kitchen_motion``, or a filter such as
``zigbee2mqtt/+`` or ``ros2_bridge/ha/sensors/#``) and to each entry's
availability sibling (the entry plus ``mirror.availability_suffix``, default
``/availability``). :class:`SensorMirror` turns every message that arrives on
those subscriptions into one JSON envelope and hands it to an output callable;
in the node (phase 2) that callable publishes a ``std_msgs/String`` on
``mirror.ros_topic`` (``~/mqtt_in``). The payload itself travels unchanged, so
a Zigbee2MQTT event reaches ROS 2 at the same time as it reaches Home
Assistant, with nothing reformatted in between (docs/architecture.md, fast
sensor path).

Envelope (one JSON object per message, keys always present in this order)::

    {"topic": "zigbee2mqtt/kitchen_motion",
     "payload": "{\\"occupancy\\":true}",
     "encoding": "utf-8",
     "retain": false,
     "source_available": true,
     "rx_ts_ms": 1760356800123}

``payload`` is the message decoded as UTF-8 and ``encoding`` is ``utf-8``;
when the bytes are not UTF-8, ``payload`` is their base64 form and
``encoding`` is ``base64``. ``source_available`` is the last availability seen
on the topic's sibling (Zigbee2MQTT publishes ``{"state":"online"}`` or the
plain word ``online``; both forms and both ``offline`` forms are understood) or
``null`` when no sibling message has been seen. ``rx_ts_ms`` is the epoch
millisecond the node received the message. An availability message is
mirrored too, so a consumer learns that a sensor went offline without waiting
for a data message that will not come.

Nothing here raises on an inbound message: a bad topic, a bad payload type, an
oversize message, an unparseable availability word or a failing output
callable is logged and counted in :attr:`SensorMirror.counters`. Only the
configuration (the topic list and the suffix) is validated eagerly, with
``TypeError`` for a wrong type and ``ValueError`` for a bad value, because a
bad filter is an operator error that should stop the node at start-up rather
than silence a sensor path. The module imports neither
rclpy nor paho; the node calls :meth:`SensorMirror.handle` on its main thread
with what the paho thread queued, so no locking is needed here.
"""

from __future__ import annotations

import base64
import json
import logging
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass
from typing import Any

from ros2_ha_bridge.clock import Clock, SystemClock
from ros2_ha_bridge.protocol import MAX_PAYLOAD_BYTES, MAX_TOPIC_BYTES

__all__ = [
    'DEFAULT_AVAILABILITY_SUFFIX',
    'DEFAULT_ROS_QUEUE_DEPTH',
    'DEFAULT_ROS_TOPIC',
    'ENCODING_BASE64',
    'ENCODING_UTF8',
    'ENVELOPE_KEYS',
    'MIRROR_QOS',
    'MirrorCounters',
    'SensorMirror',
    'build_envelope',
    'encode_envelope',
    'filter_error',
    'is_valid_filter',
    'parse_availability',
    'topic_matches',
    'validate_filter',
]

LOGGER = logging.getLogger(__name__)

#: ``mirror.availability_suffix`` default: Zigbee2MQTT's ``zigbee2mqtt/{name}/availability``.
DEFAULT_AVAILABILITY_SUFFIX = '/availability'
#: ``mirror.ros_topic`` default; the node resolves ``~`` to its own namespace.
DEFAULT_ROS_TOPIC = '~/mqtt_in'
#: ``mirror.ros_queue_depth`` default: keep-last history depth of the ROS publisher, which is
#: the only buffer between a chatty ``#`` filter and a slow ROS subscriber (DDS drops beyond it).
DEFAULT_ROS_QUEUE_DEPTH = 10
#: Mirror subscriptions are QoS 0 (the topic table: "as published", nothing to acknowledge).
MIRROR_QOS = 0
#: Values of the envelope's ``encoding`` key.
ENCODING_UTF8 = 'utf-8'
ENCODING_BASE64 = 'base64'
#: Envelope keys, in output order. Every envelope carries all of them.
ENVELOPE_KEYS: tuple[str, ...] = (
    'topic',
    'payload',
    'encoding',
    'retain',
    'source_available',
    'rx_ts_ms',
)

#: Bound on the per-topic "warn once" memory, so a ``#`` filter over a noisy tree cannot
#: grow it without limit. Beyond it, repeats are logged at DEBUG only.
_WARN_ONCE_LIMIT = 1000

_AVAILABILITY_WORDS: Mapping[str, bool] = {'online': True, 'offline': False}


# --------------------------------------------------------------------------- topic filters


def filter_error(topic_filter: object) -> str | None:
    """Return why ``topic_filter`` is not a usable MQTT subscription filter, or ``None``.

    The MQTT rules paho enforces: a non-empty string without NUL, at most
    65535 bytes, ``#`` only as the whole last segment and ``+`` only as a
    whole segment.
    """
    if not isinstance(topic_filter, str):
        return 'topic filter must be a string'
    if not topic_filter:
        return 'topic filter is empty'
    if '\x00' in topic_filter:
        return 'topic filter contains a NUL character'
    if len(topic_filter.encode('utf-8')) > MAX_TOPIC_BYTES:
        return f'topic filter is longer than {MAX_TOPIC_BYTES} bytes'
    segments = topic_filter.split('/')
    last = len(segments) - 1
    for index, segment in enumerate(segments):
        if '#' in segment and (segment != '#' or index != last):
            return "'#' may only be the whole last segment of a topic filter"
        if '+' in segment and segment != '+':
            return "'+' may only be a whole segment of a topic filter"
    return None


def is_valid_filter(topic_filter: object) -> bool:
    """True when :func:`filter_error` finds nothing wrong."""
    return filter_error(topic_filter) is None


def validate_filter(topic_filter: str) -> str:
    """Return ``topic_filter`` or raise ``ValueError`` with the reason it is refused."""
    error = filter_error(topic_filter)
    if error is not None:
        raise ValueError(f'invalid mirror topic {topic_filter!r}: {error}')
    return topic_filter


def topic_matches(topic_filter: str, topic: str) -> bool:
    """MQTT topic-filter matching: ``+`` one level, ``#`` the rest, ``$`` topics excluded.

    Same rules as a broker applies, so the mirror classifies an inbound topic
    exactly as the subscription that delivered it.
    """
    if topic_filter == topic:
        return True
    if topic.startswith('$') and topic_filter[:1] in ('+', '#'):
        return False
    filter_parts = topic_filter.split('/')
    topic_parts = topic.split('/')
    for index, part in enumerate(filter_parts):
        if part == '#':
            return True
        if index >= len(topic_parts):
            return False
        if part != '+' and part != topic_parts[index]:
            return False
    return len(filter_parts) == len(topic_parts)


# --------------------------------------------------------------------------- availability


def parse_availability(payload: bytes | str) -> bool | None:
    """Read an availability payload; ``True`` online, ``False`` offline, ``None`` unreadable.

    Accepts the plain words ``online`` and ``offline`` (any case, surrounding
    whitespace ignored), Zigbee2MQTT's JSON form ``{"state": "online"}`` and a
    JSON string ``"online"``. Anything else, including bytes that are not
    UTF-8, is ``None`` and leaves the tracked state unchanged.
    """
    if isinstance(payload, str):
        text = payload
    else:
        try:
            text = payload.decode('utf-8')
        except UnicodeDecodeError:
            return None
    word = text.strip()
    verdict = _AVAILABILITY_WORDS.get(word.lower())
    if verdict is not None:
        return verdict
    if not word.startswith(('{', '"')):
        return None
    try:
        parsed: object = json.loads(word)
    except ValueError:
        return None
    if isinstance(parsed, dict):
        parsed = parsed.get('state')
    if isinstance(parsed, str):
        return _AVAILABILITY_WORDS.get(parsed.strip().lower())
    return None


# --------------------------------------------------------------------------- envelopes


def build_envelope(
    topic: str,
    payload: bytes,
    *,
    retain: bool,
    source_available: bool | None,
    rx_ts_ms: int,
) -> dict[str, Any]:
    """The envelope dict for one message (see the module docstring for the keys)."""
    try:
        text = payload.decode('utf-8')
        encoding = ENCODING_UTF8
    except UnicodeDecodeError:
        text = base64.b64encode(payload).decode('ascii')
        encoding = ENCODING_BASE64
    return {
        'topic': topic,
        'payload': text,
        'encoding': encoding,
        'retain': bool(retain),
        'source_available': source_available,
        'rx_ts_ms': rx_ts_ms,
    }


def encode_envelope(envelope: Mapping[str, Any]) -> str:
    """The envelope as the compact JSON text that goes into the ``std_msgs/String``."""
    return json.dumps(envelope, separators=(',', ':'), ensure_ascii=False)


# --------------------------------------------------------------------------- the mirror


@dataclass
class MirrorCounters:
    """Lifetime counters of one :class:`SensorMirror`, for logs and diagnostics.

    They are not part of the heartbeat (its counter set is closed by the
    contract); the node may log them or expose them as it sees fit.
    """

    #: Envelopes handed to the output callable without error.
    emitted: int = 0
    #: Tracked availability changes (unknown to known counts, as does a clear).
    availability_updates: int = 0
    #: Messages whose payload was not UTF-8 and went out base64 encoded.
    binary_payloads: int = 0
    #: Messages dropped for being over ``max_payload_bytes``.
    dropped_oversize: int = 0
    #: Messages dropped for a topic or payload of an unusable type.
    dropped_invalid: int = 0
    #: Availability messages whose payload was neither form (state left unchanged).
    unparsed_availability: int = 0
    #: Messages on topics no configured filter matches (not ours; nothing emitted).
    ignored: int = 0
    #: Envelopes the output callable refused by raising.
    output_errors: int = 0
    #: Anything else that went wrong inside :meth:`SensorMirror.handle`.
    errors: int = 0


class SensorMirror:
    """Mirror configured MQTT topics, with their source availability, onto one output.

    ``topics`` are the ``mirror.topics`` entries (validated as MQTT filters;
    duplicates dropped, order kept). ``output`` receives each envelope as a
    JSON string; in the node it publishes a ``std_msgs/String``.
    ``availability_suffix`` names each entry's sibling (``''`` disables
    availability tracking altogether). ``max_payload_bytes`` drops larger
    messages (``None`` disables the check). ``clock`` supplies ``rx_ts_ms``
    when the caller does not pass one. Construction raises ``TypeError`` for
    a topic list or suffix of the wrong type and ``ValueError`` for a filter
    or suffix that MQTT would refuse.
    """

    def __init__(
        self,
        topics: Iterable[str],
        output: Callable[[str], object],
        *,
        clock: Clock | None = None,
        availability_suffix: str = DEFAULT_AVAILABILITY_SUFFIX,
        max_payload_bytes: int | None = MAX_PAYLOAD_BYTES,
        logger: logging.Logger | None = None,
    ) -> None:
        self._output = output
        self._clock: Clock = clock if clock is not None else SystemClock()
        self.logger = logger if logger is not None else LOGGER
        self.availability_suffix = _validate_suffix(availability_suffix)
        self.topics: tuple[str, ...] = _validate_topics(topics)
        self.max_payload_bytes = max_payload_bytes
        self.counters = MirrorCounters()
        self._available: dict[str, bool] = {}
        self._warned: set[str] = set()

    # ----- configuration -------------------------------------------------------------

    def availability_filter(self, topic_filter: str) -> str | None:
        """The sibling filter carrying ``topic_filter``'s availability, or ``None``.

        ``None`` when tracking is disabled, when the filter ends in ``#``
        (which already covers every sibling beneath it) or when the filter is
        itself a sibling.
        """
        suffix = self.availability_suffix
        if not suffix or topic_filter.endswith(('#', suffix)):
            return None
        return topic_filter + suffix

    def subscriptions(self) -> list[tuple[str, int]]:
        """``(filter, qos)`` pairs the node must subscribe to: each topic and its sibling."""
        filters: list[str] = []
        for topic in self.topics:
            if topic not in filters:
                filters.append(topic)
            sibling = self.availability_filter(topic)
            if sibling is not None and sibling not in filters:
                filters.append(sibling)
        return [(topic_filter, MIRROR_QOS) for topic_filter in filters]

    # ----- queries --------------------------------------------------------------------

    def matches(self, topic: object) -> bool:
        """True when ``topic`` is a mirrored topic or the availability sibling of one."""
        if not isinstance(topic, str) or not topic:
            return False
        return self._matches_data(topic) or self._base_of(topic) is not None

    def source_available(self, topic: str) -> bool | None:
        """Last availability seen for the data topic ``topic``; ``None`` when never seen."""
        return self._available.get(topic)

    @property
    def availability(self) -> dict[str, bool]:
        """A copy of the tracked availability, data topic to state."""
        return dict(self._available)

    # ----- inbound ----------------------------------------------------------------------

    def handle(
        self,
        topic: object,
        payload: object,
        *,
        retain: bool = False,
        rx_ts_ms: int | None = None,
    ) -> bool:
        """Process one inbound message; return True when an envelope was emitted.

        Never raises: every failure is logged, counted and answered with
        ``False``. ``payload`` may be ``bytes``, ``bytearray``, ``memoryview``,
        ``str`` or ``None`` (empty); ``rx_ts_ms`` defaults to the clock's
        ``now_ms()``.
        """
        try:
            return self._handle(topic, payload, retain, rx_ts_ms)
        except Exception:
            self.counters.errors += 1
            self.logger.exception('sensor mirror: unexpected error handling topic %r', topic)
            return False

    def _handle(self, topic: object, payload: object, retain: bool, rx_ts_ms: int | None) -> bool:
        if not isinstance(topic, str) or not topic:
            self.counters.dropped_invalid += 1
            self._warn_once(
                f'topic:{topic!r}', 'sensor mirror: dropped message with topic %r', topic
            )
            return False
        data = _coerce_payload(payload)
        if data is None:
            self.counters.dropped_invalid += 1
            self._warn_once(
                f'payload:{topic}',
                'sensor mirror: dropped message on %r with payload of type %s',
                topic,
                type(payload).__name__,
            )
            return False

        base = self._base_of(topic)
        if base is None and not self._matches_data(topic):
            self.counters.ignored += 1
            self.logger.debug('sensor mirror: ignoring %r (no configured filter matches)', topic)
            return False

        if self.max_payload_bytes is not None and len(data) > self.max_payload_bytes:
            self.counters.dropped_oversize += 1
            self._warn_once(
                f'oversize:{topic}',
                'sensor mirror: dropped %d-byte message on %r (limit %d)',
                len(data),
                topic,
                self.max_payload_bytes,
            )
            return False

        if base is not None:
            source_available = self._update_availability(base, topic, data)
        else:
            source_available = self._available.get(topic)

        when = self._clock.now_ms() if rx_ts_ms is None else int(rx_ts_ms)
        envelope = build_envelope(
            topic, data, retain=retain, source_available=source_available, rx_ts_ms=when
        )
        if envelope['encoding'] == ENCODING_BASE64:
            self.counters.binary_payloads += 1
            self.logger.debug('sensor mirror: %r carried %d non-UTF-8 bytes', topic, len(data))
        text = encode_envelope(envelope)
        try:
            self._output(text)
        except Exception:
            self.counters.output_errors += 1
            self.logger.exception('sensor mirror: output failed for %r', topic)
            return False
        self.counters.emitted += 1
        return True

    # ----- internals -----------------------------------------------------------------

    def _matches_data(self, topic: str) -> bool:
        return any(topic_matches(topic_filter, topic) for topic_filter in self.topics)

    def _base_of(self, topic: str) -> str | None:
        """The data topic whose availability sibling ``topic`` is, or ``None``."""
        suffix = self.availability_suffix
        if not suffix or len(topic) <= len(suffix) or not topic.endswith(suffix):
            return None
        base = topic[: -len(suffix)]
        return base if self._matches_data(base) else None

    def _update_availability(self, base: str, topic: str, data: bytes) -> bool | None:
        if not data.strip():
            # An empty retained publish clears the sibling: back to unknown.
            if self._available.pop(base, None) is not None:
                self.counters.availability_updates += 1
                self.logger.info('sensor mirror: availability of %r cleared', base)
            return None
        verdict = parse_availability(data)
        if verdict is None:
            self.counters.unparsed_availability += 1
            self._warn_once(
                f'availability:{topic}',
                'sensor mirror: availability payload on %r not understood: %r',
                topic,
                data[:64],
            )
            return self._available.get(base)
        if self._available.get(base) != verdict:
            self.counters.availability_updates += 1
            self.logger.info('sensor mirror: %r is %s', base, 'online' if verdict else 'offline')
        self._available[base] = verdict
        return verdict

    def _warn_once(self, key: str, message: str, *args: object) -> None:
        if key in self._warned or len(self._warned) >= _WARN_ONCE_LIMIT:
            self.logger.debug(message, *args)
            return
        self._warned.add(key)
        self.logger.warning(message, *args)


def _validate_suffix(suffix: object) -> str:
    if not isinstance(suffix, str):
        raise TypeError('mirror availability suffix must be a string')
    if not suffix:
        return ''
    if '+' in suffix or '#' in suffix or '\x00' in suffix:
        raise ValueError(f'mirror availability suffix {suffix!r} must not contain +, # or NUL')
    if not suffix.startswith('/') or suffix.endswith('/') or '//' in suffix:
        raise ValueError(
            f'mirror availability suffix {suffix!r} must start with a slash, '
            'not end with one and have no empty segment'
        )
    return suffix


def _validate_topics(topics: Iterable[str]) -> tuple[str, ...]:
    if isinstance(topics, str):
        raise TypeError('mirror topics must be a list of topic filters, not one string')
    seen: list[str] = []
    for topic in topics:
        if not isinstance(topic, str):
            raise TypeError(f'mirror topic {topic!r} must be a string')
        validate_filter(topic)
        if topic not in seen:
            seen.append(topic)
    return tuple(seen)


def _coerce_payload(payload: object) -> bytes | None:
    """The message bytes, or ``None`` for a type no MQTT client would hand over."""
    if payload is None:
        return b''
    if isinstance(payload, bytes):
        return payload
    if isinstance(payload, (bytearray, memoryview)):
        return bytes(payload)
    if isinstance(payload, str):
        try:
            return payload.encode('utf-8')
        except UnicodeEncodeError:
            return None
    return None
