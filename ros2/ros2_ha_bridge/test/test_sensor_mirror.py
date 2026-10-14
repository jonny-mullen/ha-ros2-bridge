# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Tests for :mod:`ros2_ha_bridge.sensor_mirror`.

Envelope shape, availability tracking in both Zigbee2MQTT forms (JSON
``{"state": ...}`` and the plain word), binary payloads, the subscription list
with availability siblings, configuration validation, and the rule that bad
input is logged and counted but never raised.
"""

from __future__ import annotations

import base64
import json
import logging
import sys
from typing import Any

import pytest

from ros2_ha_bridge import sensor_mirror
from ros2_ha_bridge.sensor_mirror import (
    DEFAULT_AVAILABILITY_SUFFIX,
    ENCODING_BASE64,
    ENCODING_UTF8,
    ENVELOPE_KEYS,
    MIRROR_QOS,
    MirrorCounters,
    SensorMirror,
)

from .fakes import DEFAULT_EPOCH_MS, FakeClock
from .fakes import topic_matches as broker_topic_matches

MOTION = 'zigbee2mqtt/kitchen_motion'
MOTION_AVAILABILITY = 'zigbee2mqtt/kitchen_motion/availability'
SENSORS = 'ros2_bridge/ha/sensors/#'
KITCHEN_MOTION_SENSOR = 'ros2_bridge/ha/sensors/binary_sensor/kitchen_motion'
LOGGER_NAME = 'ros2_ha_bridge.sensor_mirror'


class Recorder:
    """Output callable that keeps every envelope text; ``fail`` makes it raise."""

    def __init__(self) -> None:
        self.texts: list[str] = []
        self.fail = False

    def __call__(self, text: str) -> None:
        if self.fail:
            raise RuntimeError('publisher is down')
        self.texts.append(text)

    @property
    def envelopes(self) -> list[dict[str, Any]]:
        return [json.loads(text) for text in self.texts]

    @property
    def last(self) -> dict[str, Any]:
        assert self.texts, 'nothing was emitted'
        return json.loads(self.texts[-1])


@pytest.fixture
def output() -> Recorder:
    return Recorder()


@pytest.fixture
def mirror(clock: FakeClock, output: Recorder) -> SensorMirror:
    return SensorMirror([MOTION, SENSORS], output, clock=clock)


# --------------------------------------------------------------------------- hygiene


def test_module_imports_neither_rclpy_nor_paho() -> None:
    assert sensor_mirror.__name__ in sys.modules
    loaded = {name.split('.')[0] for name in sys.modules}
    assert 'rclpy' not in loaded
    assert 'paho' not in loaded


def test_counters_start_at_zero() -> None:
    counters = MirrorCounters()
    assert all(value == 0 for value in vars(counters).values())
    assert set(vars(counters)) == {
        'emitted',
        'availability_updates',
        'binary_payloads',
        'dropped_oversize',
        'dropped_invalid',
        'unparsed_availability',
        'ignored',
        'output_errors',
        'errors',
    }


# --------------------------------------------------------------------------- envelope shape


def test_envelope_shape(mirror: SensorMirror, output: Recorder, clock: FakeClock) -> None:
    assert mirror.handle(MOTION, b'{"occupancy":true}') is True
    assert len(output.texts) == 1
    envelope = output.last
    assert list(envelope) == list(ENVELOPE_KEYS)
    assert envelope == {
        'topic': MOTION,
        'payload': '{"occupancy":true}',
        'encoding': ENCODING_UTF8,
        'retain': False,
        'source_available': None,
        'rx_ts_ms': DEFAULT_EPOCH_MS,
    }
    # The text is compact JSON that encodes to UTF-8 for the std_msgs/String.
    assert '\n' not in output.texts[0] and ': ' not in output.texts[0]
    output.texts[0].encode('utf-8')

    clock.advance(1.5)
    assert mirror.handle(MOTION, b'{"occupancy":false}', retain=True)
    assert output.last['rx_ts_ms'] == DEFAULT_EPOCH_MS + 1500
    assert output.last['retain'] is True

    assert mirror.handle(MOTION, b'x', rx_ts_ms=42)
    assert output.last['rx_ts_ms'] == 42
    assert mirror.counters.emitted == 3


def test_payload_is_mirrored_unchanged(mirror: SensorMirror, output: Recorder) -> None:
    text = '{\n  "temperature": 21.5,\n  "unit": "°C",\n  "room": "Küche"\n}\n'
    assert mirror.handle(MOTION, text.encode('utf-8'))
    assert output.last['payload'] == text
    assert output.last['encoding'] == ENCODING_UTF8
    # Non-ASCII is kept as is, never escaped.
    assert '°C' in output.texts[-1] and '\\u00b0' not in output.texts[-1]


@pytest.mark.parametrize(
    'payload',
    [
        b'"\\\x00\n\t\x1f"',
        b'\x00',
        b'',
        b'   ',
        b'plain words with "quotes" and \\ backslashes',
    ],
)
def test_envelope_is_valid_json_for_awkward_payloads(
    mirror: SensorMirror, output: Recorder, payload: bytes
) -> None:
    assert mirror.handle(MOTION, payload)
    envelope = json.loads(output.texts[-1])
    assert envelope['payload'] == payload.decode('utf-8')
    assert envelope['encoding'] == ENCODING_UTF8


# --------------------------------------------------------------------------- availability


@pytest.mark.parametrize(
    ('payload', 'expected'),
    [
        (b'{"state":"online"}', True),
        (b'{"state": "offline"}', False),
        (b'{"state":"Online","other":1}', True),
        (b'online', True),
        (b'offline', False),
        (b'OFFLINE', False),
        (b' online\n', True),
        (b'"online"', True),
        (b'"offline"', False),
        (b'{"state":"weird"}', None),
        (b'{"state":5}', None),
        (b'{"status":"online"}', None),
        (b'[1]', None),
        (b'maybe', None),
        (b'', None),
        (b'\xff\xfe', None),
        (b'{"state":', None),
        ('online', True),
        ('{"state": "offline"}', False),
    ],
)
def test_parse_availability(payload: bytes | str, expected: bool | None) -> None:
    assert sensor_mirror.parse_availability(payload) is expected


@pytest.mark.parametrize(
    ('online', 'offline'),
    [
        (b'{"state":"online"}', b'{"state":"offline"}'),
        (b'online', b'offline'),
        (b'{"state":"online"}', b'offline'),
    ],
    ids=['json', 'plain', 'mixed'],
)
def test_availability_is_tracked_from_the_sibling(
    mirror: SensorMirror, output: Recorder, online: bytes, offline: bytes
) -> None:
    assert mirror.source_available(MOTION) is None
    assert mirror.handle(MOTION, b'{"occupancy":true}')
    assert output.last['source_available'] is None

    # The availability message is itself mirrored, carrying the new state.
    assert mirror.handle(MOTION_AVAILABILITY, online, retain=True)
    assert output.last['topic'] == MOTION_AVAILABILITY
    assert output.last['payload'] == online.decode()
    assert output.last['source_available'] is True
    assert mirror.source_available(MOTION) is True
    assert mirror.availability == {MOTION: True}

    assert mirror.handle(MOTION, b'{"occupancy":false}')
    assert output.last['source_available'] is True

    assert mirror.handle(MOTION_AVAILABILITY, offline)
    assert output.last['source_available'] is False
    assert mirror.handle(MOTION, b'{"occupancy":true}')
    assert output.last['source_available'] is False
    assert mirror.counters.availability_updates == 2
    assert mirror.counters.emitted == 5


def test_repeated_availability_counts_only_changes(mirror: SensorMirror) -> None:
    for payload in (b'online', b'online', b'{"state":"online"}', b'offline', b'offline'):
        assert mirror.handle(MOTION_AVAILABILITY, payload)
    assert mirror.counters.availability_updates == 2
    assert mirror.source_available(MOTION) is False


def test_unparseable_availability_keeps_the_last_state(
    mirror: SensorMirror, output: Recorder, caplog: pytest.LogCaptureFixture
) -> None:
    assert mirror.handle(MOTION_AVAILABILITY, b'online')
    junk = [b'{"state":"weird"}', b'maybe', b'\xff', b'{"state":5}', b'[1]']
    with caplog.at_level(logging.DEBUG, logger=LOGGER_NAME):
        for payload in junk:
            # Still mirrored (the consumer may understand it), state untouched.
            assert mirror.handle(MOTION_AVAILABILITY, payload)
            assert output.last['source_available'] is True
    assert mirror.source_available(MOTION) is True
    assert mirror.counters.unparsed_availability == len(junk)
    warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
    assert len(warnings) == 1, 'the unparseable-availability warning is logged once per topic'
    assert MOTION_AVAILABILITY in warnings[0].getMessage()
    assert mirror.counters.binary_payloads == 1


def test_empty_availability_payload_clears_the_state(
    mirror: SensorMirror, output: Recorder
) -> None:
    # A clear when nothing is known changes nothing and counts nothing.
    assert mirror.handle(MOTION_AVAILABILITY, b'')
    assert output.last['source_available'] is None
    assert mirror.counters.availability_updates == 0

    assert mirror.handle(MOTION_AVAILABILITY, b'online')
    assert mirror.handle(MOTION_AVAILABILITY, b'', retain=True)
    assert output.last['source_available'] is None
    assert mirror.source_available(MOTION) is None
    assert mirror.availability == {}
    assert mirror.counters.availability_updates == 2


def test_wildcard_filter_tracks_each_device_separately(clock: FakeClock, output: Recorder) -> None:
    mirror = SensorMirror(['zigbee2mqtt/+'], output, clock=clock)
    assert mirror.subscriptions() == [
        ('zigbee2mqtt/+', MIRROR_QOS),
        ('zigbee2mqtt/+/availability', MIRROR_QOS),
    ]
    assert mirror.handle('zigbee2mqtt/a/availability', b'{"state":"online"}')
    assert mirror.handle('zigbee2mqtt/b/availability', b'offline')
    assert mirror.handle('zigbee2mqtt/a', b'1')
    assert output.last['source_available'] is True
    assert mirror.handle('zigbee2mqtt/b', b'2')
    assert output.last['source_available'] is False
    assert mirror.handle('zigbee2mqtt/c', b'3')
    assert output.last['source_available'] is None
    assert mirror.availability == {'zigbee2mqtt/a': True, 'zigbee2mqtt/b': False}
    # Two levels down is not matched by '+' and is not a sibling of anything matched.
    assert not mirror.handle('zigbee2mqtt/a/b', b'4')
    assert mirror.counters.ignored == 1


def test_hash_filter_covers_its_own_siblings(clock: FakeClock, output: Recorder) -> None:
    mirror = SensorMirror([SENSORS], output, clock=clock)
    assert mirror.subscriptions() == [(SENSORS, MIRROR_QOS)], (
        "a filter ending in '#' already receives every availability sibling beneath it"
    )
    assert mirror.availability_filter(SENSORS) is None
    assert mirror.handle(KITCHEN_MOTION_SENSOR, b'{"schema":1}', retain=True)
    assert output.last['source_available'] is None
    assert output.last['retain'] is True
    # Should anything publish a sibling under the tree, it is tracked like any other.
    assert mirror.handle(KITCHEN_MOTION_SENSOR + '/availability', b'online')
    assert output.last['source_available'] is True
    assert mirror.handle(KITCHEN_MOTION_SENSOR, b'{"schema":1}')
    assert output.last['source_available'] is True
    # The parent of the tree matches '#' too.
    assert mirror.matches('ros2_bridge/ha/sensors')
    assert not mirror.matches('ros2_bridge/ha/heartbeat')


def test_availability_sibling_listed_as_a_data_topic_is_mirrored_as_data(
    clock: FakeClock, output: Recorder
) -> None:
    mirror = SensorMirror([MOTION_AVAILABILITY], output, clock=clock)
    assert mirror.subscriptions() == [(MOTION_AVAILABILITY, MIRROR_QOS)]
    assert mirror.handle(MOTION_AVAILABILITY, b'online')
    assert output.last['source_available'] is None, 'its base topic is not mirrored'
    assert mirror.availability == {}
    assert not mirror.handle(MOTION, b'x')


# --------------------------------------------------------------------------- binary payloads


def test_binary_payload_is_base64(mirror: SensorMirror, output: Recorder) -> None:
    raw = b'\xff\x00\x01\x80 not utf-8'
    assert mirror.handle(MOTION, raw)
    envelope = output.last
    assert envelope['encoding'] == ENCODING_BASE64
    assert base64.b64decode(envelope['payload']) == raw
    assert envelope['payload'].isascii()
    assert mirror.counters.binary_payloads == 1
    assert mirror.handle(MOTION, 'utf-8 text é'.encode())
    assert output.last['encoding'] == ENCODING_UTF8
    assert mirror.counters.binary_payloads == 1


def test_build_envelope_and_encode_envelope() -> None:
    envelope = sensor_mirror.build_envelope(
        MOTION, b'\xc3\x28', retain=True, source_available=False, rx_ts_ms=7
    )
    assert envelope == {
        'topic': MOTION,
        'payload': base64.b64encode(b'\xc3\x28').decode('ascii'),
        'encoding': ENCODING_BASE64,
        'retain': True,
        'source_available': False,
        'rx_ts_ms': 7,
    }
    text = sensor_mirror.encode_envelope(envelope)
    assert json.loads(text) == envelope
    assert text.startswith('{"topic":')


# --------------------------------------------------------------------------- subscriptions


def test_subscriptions_list_topics_and_siblings_once_in_order(
    clock: FakeClock, output: Recorder
) -> None:
    mirror = SensorMirror(
        [MOTION, 'zigbee2mqtt/hall_motion', MOTION, MOTION_AVAILABILITY, SENSORS, 'a/+/b'],
        output,
        clock=clock,
    )
    assert mirror.topics == (
        MOTION,
        'zigbee2mqtt/hall_motion',
        MOTION_AVAILABILITY,
        SENSORS,
        'a/+/b',
    )
    assert mirror.subscriptions() == [
        (MOTION, 0),
        (MOTION_AVAILABILITY, 0),
        ('zigbee2mqtt/hall_motion', 0),
        ('zigbee2mqtt/hall_motion/availability', 0),
        (SENSORS, 0),
        ('a/+/b', 0),
        ('a/+/b/availability', 0),
    ]
    assert all(qos == MIRROR_QOS == 0 for _, qos in mirror.subscriptions())
    assert mirror.availability_suffix == DEFAULT_AVAILABILITY_SUFFIX


def test_custom_availability_suffix(clock: FakeClock, output: Recorder) -> None:
    mirror = SensorMirror([MOTION], output, clock=clock, availability_suffix='/status/link')
    assert mirror.subscriptions() == [(MOTION, 0), (MOTION + '/status/link', 0)]
    assert mirror.handle(MOTION + '/status/link', b'offline')
    assert mirror.source_available(MOTION) is False
    # The default suffix is now an ordinary unmatched topic.
    assert not mirror.handle(MOTION_AVAILABILITY, b'online')
    assert mirror.counters.ignored == 1


def test_empty_suffix_disables_availability_tracking(clock: FakeClock, output: Recorder) -> None:
    mirror = SensorMirror([MOTION, 'zigbee2mqtt/+'], output, clock=clock, availability_suffix='')
    assert mirror.subscriptions() == [(MOTION, 0), ('zigbee2mqtt/+', 0)]
    assert mirror.availability_filter(MOTION) is None
    # The sibling is not a configured filter here, so it is not ours.
    assert not mirror.handle(MOTION_AVAILABILITY, b'online')
    assert mirror.handle(MOTION, b'x')
    assert output.last['source_available'] is None
    assert mirror.availability == {}


def test_empty_topic_list_means_nothing_to_mirror(clock: FakeClock, output: Recorder) -> None:
    mirror = SensorMirror([], output, clock=clock)
    assert mirror.topics == ()
    assert mirror.subscriptions() == []
    assert not mirror.handle(MOTION, b'x')
    assert mirror.counters.ignored == 1
    assert output.texts == []


# --------------------------------------------------------------------------- configuration


@pytest.mark.parametrize(
    'topic_filter',
    [
        '',
        'a/#/b',
        '#a',
        'a#',
        'a+b',
        '+a',
        'a/+b/c',
        'a\x00b',
        pytest.param('x' * 65536, id='over-65535-bytes'),
    ],
)
def test_invalid_mirror_topics_are_refused_at_construction(
    topic_filter: str, output: Recorder
) -> None:
    assert not sensor_mirror.is_valid_filter(topic_filter)
    assert sensor_mirror.filter_error(topic_filter)
    with pytest.raises(ValueError):
        sensor_mirror.validate_filter(topic_filter)
    with pytest.raises(ValueError):
        SensorMirror([MOTION, topic_filter], output)


@pytest.mark.parametrize('topic_filter', [None, 5, b'a/b', ['a/b']])
def test_non_string_mirror_topics_are_a_type_error(topic_filter: Any, output: Recorder) -> None:
    assert not sensor_mirror.is_valid_filter(topic_filter)
    assert sensor_mirror.filter_error(topic_filter) == 'topic filter must be a string'
    with pytest.raises(ValueError):
        sensor_mirror.validate_filter(topic_filter)
    with pytest.raises(TypeError):
        SensorMirror([MOTION, topic_filter], output)


@pytest.mark.parametrize(
    'topic_filter',
    [
        '#',
        '+',
        'a',
        'a/b',
        'a/+/b',
        'a/#',
        '+/+',
        '$SYS/#',
        'a/b/',
        '/a',
        pytest.param('x' * 65535, id='65535-bytes'),
    ],
)
def test_valid_mirror_topics_are_accepted(topic_filter: str, output: Recorder) -> None:
    assert sensor_mirror.is_valid_filter(topic_filter)
    assert sensor_mirror.filter_error(topic_filter) is None
    assert sensor_mirror.validate_filter(topic_filter) == topic_filter
    assert SensorMirror([topic_filter], output).topics == (topic_filter,)


def test_topics_given_as_one_string_are_refused(output: Recorder) -> None:
    with pytest.raises(TypeError):
        SensorMirror(MOTION, output)  # type: ignore[arg-type]


@pytest.mark.parametrize(
    ('suffix', 'error'),
    [
        ('availability', ValueError),
        ('/availability/', ValueError),
        ('/a//b', ValueError),
        ('/+', ValueError),
        ('/#', ValueError),
        ('/a\x00', ValueError),
        (5, TypeError),
        (None, TypeError),
        (b'/availability', TypeError),
    ],
)
def test_invalid_availability_suffixes_are_refused(
    suffix: Any, error: type[Exception], output: Recorder
) -> None:
    with pytest.raises(error):
        SensorMirror([MOTION], output, availability_suffix=suffix)


@pytest.mark.parametrize(
    ('topic_filter', 'topic', 'expected'),
    [
        ('a/b', 'a/b', True),
        ('a/b', 'a/b/c', False),
        ('a/+', 'a/b', True),
        ('a/+', 'a', False),
        ('a/+', 'a/b/c', False),
        ('a/+/c', 'a/b/c', True),
        ('a/#', 'a', True),
        ('a/#', 'a/b/c/d', True),
        ('#', 'anything/at/all', True),
        ('#', '$SYS/broker', False),
        ('+/x', '$SYS/x', False),
        ('$SYS/#', '$SYS/broker', True),
        ('a/b', 'a/B', False),
        ('a/b/', 'a/b', False),
        ('a//b', 'a//b', True),
    ],
)
def test_topic_matches_agrees_with_the_fake_broker(
    topic_filter: str, topic: str, expected: bool
) -> None:
    assert sensor_mirror.topic_matches(topic_filter, topic) is expected
    assert broker_topic_matches(topic_filter, topic) is expected


def test_matches_covers_topics_and_siblings(mirror: SensorMirror) -> None:
    assert mirror.matches(MOTION)
    assert mirror.matches(MOTION_AVAILABILITY)
    assert mirror.matches(KITCHEN_MOTION_SENSOR)
    assert not mirror.matches('zigbee2mqtt/hall_motion')
    assert not mirror.matches('zigbee2mqtt/hall_motion/availability')
    assert not mirror.matches(MOTION + '/extra')
    assert not mirror.matches('')
    assert not mirror.matches(None)
    assert not mirror.matches(7)


# --------------------------------------------------------------------------- bad input never raises


def test_bad_topics_and_payload_types_are_dropped_and_counted(
    mirror: SensorMirror, output: Recorder, caplog: pytest.LogCaptureFixture
) -> None:
    bad: list[tuple[Any, Any]] = [
        (None, b'x'),
        (5, b'x'),
        ('', b'x'),
        (b'zigbee2mqtt/kitchen_motion', b'x'),
        (MOTION, 5),
        (MOTION, 1.5),
        (MOTION, object()),
        (MOTION, ['x']),
        (MOTION, '\ud800'),
    ]
    with caplog.at_level(logging.DEBUG, logger=LOGGER_NAME):
        for topic, payload in bad:
            assert mirror.handle(topic, payload) is False
    assert mirror.counters.dropped_invalid == len(bad)
    assert mirror.counters.emitted == 0
    assert output.texts == []
    assert any(r.levelno == logging.WARNING for r in caplog.records)


def test_accepted_payload_types(mirror: SensorMirror, output: Recorder) -> None:
    assert mirror.handle(MOTION, None)
    assert output.last['payload'] == '' and output.last['encoding'] == ENCODING_UTF8
    assert mirror.handle(MOTION, bytearray(b'ab'))
    assert output.last['payload'] == 'ab'
    assert mirror.handle(MOTION, memoryview(b'cd'))
    assert output.last['payload'] == 'cd'
    assert mirror.handle(MOTION, 'héllo')
    assert output.last['payload'] == 'héllo'
    assert mirror.counters.emitted == 4
    assert mirror.counters.dropped_invalid == 0


def test_oversize_messages_are_dropped(
    clock: FakeClock, output: Recorder, caplog: pytest.LogCaptureFixture
) -> None:
    mirror = SensorMirror([MOTION], output, clock=clock, max_payload_bytes=8)
    with caplog.at_level(logging.DEBUG, logger=LOGGER_NAME):
        assert not mirror.handle(MOTION, b'x' * 9)
        assert not mirror.handle(MOTION, b'x' * 9)
        assert not mirror.handle(MOTION_AVAILABILITY, b'{"state":"online"}')
    assert mirror.counters.dropped_oversize == 3
    assert mirror.source_available(MOTION) is None, 'an oversize sibling never updates state'
    assert output.texts == []
    warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
    assert len(warnings) == 2, 'one warning per topic, repeats at DEBUG'
    assert mirror.handle(MOTION, b'x' * 8)
    assert mirror.counters.emitted == 1

    unlimited = SensorMirror([MOTION], output, clock=clock, max_payload_bytes=None)
    assert unlimited.handle(MOTION, b'x' * 100_000)
    # Oversize on a topic that is not ours is just ignored, not counted as oversize.
    assert not mirror.handle('other/topic', b'x' * 9)
    assert mirror.counters.ignored == 1
    assert mirror.counters.dropped_oversize == 3


def test_unmatched_topics_are_ignored(mirror: SensorMirror, output: Recorder) -> None:
    for topic in ('other/topic', 'zigbee2mqtt', MOTION + '/extra', 'ros2_bridge/arm_left/cmd'):
        assert mirror.handle(topic, b'x') is False
    assert mirror.counters.ignored == 4
    assert output.texts == []
    assert mirror.counters.dropped_invalid == 0


def test_output_failure_is_caught_and_counted(
    mirror: SensorMirror, output: Recorder, caplog: pytest.LogCaptureFixture
) -> None:
    output.fail = True
    with caplog.at_level(logging.ERROR, logger=LOGGER_NAME):
        assert mirror.handle(MOTION, b'x') is False
        assert mirror.handle(MOTION_AVAILABILITY, b'online') is False
    assert mirror.counters.output_errors == 2
    assert mirror.counters.emitted == 0
    assert mirror.source_available(MOTION) is True, 'tracking happens before the output'
    errors = [r for r in caplog.records if r.levelno == logging.ERROR]
    assert len(errors) == 2 and errors[0].exc_info is not None
    output.fail = False
    assert mirror.handle(MOTION, b'x') is True
    assert output.last['source_available'] is True


def test_unexpected_internal_errors_are_caught_and_counted(
    mirror: SensorMirror, output: Recorder, caplog: pytest.LogCaptureFixture
) -> None:
    with caplog.at_level(logging.ERROR, logger=LOGGER_NAME):
        assert mirror.handle(MOTION, b'x', rx_ts_ms='soon') is False  # type: ignore[arg-type]
    assert mirror.counters.errors == 1
    assert output.texts == []
    assert any(r.exc_info is not None for r in caplog.records)
    assert mirror.handle(MOTION, b'x') is True


def test_warn_once_memory_is_bounded(
    mirror: SensorMirror, caplog: pytest.LogCaptureFixture
) -> None:
    limit = sensor_mirror._WARN_ONCE_LIMIT
    with caplog.at_level(logging.DEBUG, logger=LOGGER_NAME):
        for index in range(limit + 50):
            assert mirror.handle(f'ros2_bridge/ha/sensors/x{index}/availability', b'?')
    warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
    assert len(warnings) == limit
    assert mirror.counters.unparsed_availability == limit + 50
