# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Tests for :mod:`ros2_ha_bridge.joints_publisher`, the throttle behind the ``joints`` topic.

The three contract rules (rate, minimum delta, once per connect and announce)
against the fake clock and scheduler, the trailing publication that carries
the final pose, ``seq`` per successful publication, and what a bad report or
a failed publish does.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from typing import Any

import pytest

from ros2_ha_bridge import protocol
from ros2_ha_bridge.joints_publisher import (
    DEFAULT_JOINTS_RATE_HZ,
    DEFAULT_MIN_DELTA_DEG,
    DEFAULT_MIN_DELTA_RAD,
    JointsPublisher,
    default_min_delta,
)

from .conftest import assert_imports_without
from .fakes import FakeClock, FakeScheduler

ROBOT = 'fake_arm_1'
SESSION = '9f1c7a0e-4a1b-4b2c-8d1e-0f1a2b3c4d5e'
INSTANCE = 'robotpc:4123'
HOME = (0.0,) * 7
PICK = (35.0, -40.0, 20.0, -95.0, 10.0, 60.0, 45.0)


@dataclass(frozen=True)
class Published:
    topic: str
    payload: dict[str, Any]
    qos: int
    retain: bool
    expiry_s: int | None


class Sink:
    """The node's publish callable: records, encodes as the link would, and can be made to fail."""

    def __init__(self) -> None:
        self.records: list[Published] = []
        self.ok = True

    def __call__(
        self, topic: str, payload: Any, qos: int, retain: bool, expiry_s: int | None
    ) -> bool:
        wire = protocol.encode_payload(payload)
        if not self.ok:
            return False
        self.records.append(Published(topic, json.loads(wire), qos, retain, expiry_s))
        return True

    @property
    def positions(self) -> list[list[float]]:
        return [r.payload['positions'] for r in self.records]

    @property
    def seqs(self) -> list[int]:
        return [r.payload['seq'] for r in self.records]


def make(
    clock: FakeClock, scheduler: FakeScheduler | None, **overrides: Any
) -> tuple[JointsPublisher, Sink]:
    sink = Sink()
    settings: dict[str, Any] = {
        'robot_id': ROBOT,
        'session': SESSION,
        'instance': INSTANCE,
        'clock': clock,
        'scheduler': scheduler,
        'publish': sink,
        'joint_count': 7,
        'rate_hz': 2.0,
        'min_delta': 0.5,
    }
    settings.update(overrides)
    return JointsPublisher(**settings), sink


def step(positions: tuple[float, ...], delta: float) -> tuple[float, ...]:
    """``positions`` with every joint moved by ``delta``."""
    return tuple(value + delta for value in positions)


# --------------------------------------------------------------------------- hygiene and defaults


def test_module_imports_neither_rclpy_nor_paho() -> None:
    assert_imports_without('ros2_ha_bridge.joints_publisher', {'rclpy', 'paho'})


def test_defaults_match_the_contract() -> None:
    assert DEFAULT_JOINTS_RATE_HZ == 2.0
    assert default_min_delta('deg') == DEFAULT_MIN_DELTA_DEG == 0.5
    assert default_min_delta('rad') == DEFAULT_MIN_DELTA_RAD == 0.01


@pytest.mark.parametrize(
    'overrides',
    [
        {'joint_count': 0},
        {'joint_count': 33},
        {'rate_hz': 0},
        {'rate_hz': -1},
        {'rate_hz': float('nan')},
        {'min_delta': -0.1},
        {'min_delta': float('inf')},
    ],
)
def test_constructor_refuses_bad_settings(
    clock: FakeClock, scheduler: FakeScheduler, overrides: dict[str, Any]
) -> None:
    with pytest.raises(ValueError):
        make(clock, scheduler, **overrides)


# --------------------------------------------------------------------------- the first report


def test_first_report_publishes_at_once_with_the_topic_table_settings(
    clock: FakeClock, scheduler: FakeScheduler, schema_validator: Any
) -> None:
    publisher, sink = make(clock, scheduler)
    assert publisher.seq == 0 and publisher.latest is None
    assert publisher.report(HOME) is True
    (record,) = sink.records
    assert record.topic == f'{protocol.DEFAULT_PREFIX}/{ROBOT}/joints'
    assert record.qos == 0 and record.retain is False and record.expiry_s == 5
    schema_validator.assert_valid('joints', record.payload)
    assert record.payload == {
        'schema': 1,
        'robot_id': ROBOT,
        'positions': list(HOME),
        'velocities': None,
        'session': SESSION,
        'instance': INSTANCE,
        'seq': 1,
        'ts': clock.now_ms(),
    }
    assert publisher.seq == 1 and publisher.published == 1
    assert publisher.latest is not None and publisher.latest.positions == HOME
    assert publisher.last_published is publisher.latest


def test_velocities_travel_with_the_positions(
    clock: FakeClock, scheduler: FakeScheduler, schema_validator: Any
) -> None:
    publisher, sink = make(clock, scheduler, joint_count=3, min_delta=0.01)
    assert publisher.report([0.0, -1.5708, 1.5708], [0.0, 0.1, -0.05])
    payload = sink.records[0].payload
    schema_validator.assert_valid('joints', payload)
    assert payload['velocities'] == [0.0, 0.1, -0.05]
    clock.advance(1)
    assert publisher.report([0.1, -1.5708, 1.5708])
    assert sink.records[1].payload['velocities'] is None


# --------------------------------------------------------------------------- rate


def test_reports_inside_the_interval_are_deferred_to_one_trailing_publication(
    clock: FakeClock, scheduler: FakeScheduler
) -> None:
    publisher, sink = make(clock, scheduler)
    assert publisher.report(HOME)
    clock.advance(0.1)
    assert publisher.report(step(HOME, 1.0)) is False, 'inside the 0.5 s interval'
    assert publisher.timer_pending and len(scheduler.pending) == 1
    clock.advance(0.1)
    assert publisher.report(step(HOME, 2.0)) is False
    assert len(scheduler.pending) == 1, 'one trailing timer, however many reports'
    assert publisher.deferred == 2
    clock.advance(0.29)
    assert len(sink.records) == 1
    clock.advance(0.01)
    assert len(sink.records) == 2, 'published exactly one interval after the previous one'
    assert sink.positions[1] == list(step(HOME, 2.0)), 'the latest sample, not the first held'
    assert sink.seqs == [1, 2] and not publisher.timer_pending
    assert scheduler.pending == []


def test_a_report_exactly_one_interval_later_publishes_at_once(
    clock: FakeClock, scheduler: FakeScheduler
) -> None:
    publisher, sink = make(clock, scheduler)
    assert publisher.report(HOME)
    clock.advance(0.5)
    assert publisher.report(step(HOME, 1.0)) is True
    clock.advance(0.5)
    assert publisher.report(step(HOME, 2.0)) is True
    assert sink.seqs == [1, 2, 3] and publisher.deferred == 0


def test_rate_is_honoured_over_a_fast_stream(clock: FakeClock, scheduler: FakeScheduler) -> None:
    publisher, sink = make(clock, scheduler, rate_hz=4.0, min_delta=0.0)
    for index in range(100):
        publisher.report(step(HOME, float(index + 1)))
        clock.advance(0.01)
    assert len(sink.records) == 4, 'one per 0.25 s over a second'
    clock.advance(0.25)
    assert len(sink.records) == 5, 'the trailing publication carries the last position'
    assert sink.positions[-1] == list(step(HOME, 100.0))
    assert sink.seqs == list(range(1, 6))


def test_trailing_timer_does_nothing_when_the_held_sample_no_longer_differs(
    clock: FakeClock, scheduler: FakeScheduler
) -> None:
    publisher, sink = make(clock, scheduler)
    assert publisher.report(HOME)
    clock.advance(0.1)
    assert publisher.report(step(HOME, 1.0)) is False
    clock.advance(0.1)
    assert publisher.report(HOME) is False, 'back where it was published'
    clock.advance(1.0)
    assert len(sink.records) == 1 and scheduler.pending == []


def test_without_a_scheduler_a_deferred_report_waits_for_the_next_one(clock: FakeClock) -> None:
    publisher, sink = make(clock, None)
    assert publisher.report(HOME)
    clock.advance(0.1)
    assert publisher.report(step(HOME, 1.0)) is False
    assert not publisher.timer_pending and publisher.deferred == 1
    clock.advance(1.0)
    assert len(sink.records) == 1, 'nothing fires by itself'
    assert publisher.report(step(HOME, 1.0)) is True
    assert sink.seqs == [1, 2]


# --------------------------------------------------------------------------- minimum delta


def test_movement_at_or_below_the_minimum_delta_is_not_published(
    clock: FakeClock, scheduler: FakeScheduler
) -> None:
    publisher, sink = make(clock, scheduler, min_delta=0.5)
    assert publisher.report(HOME)
    clock.advance(1)
    assert publisher.report(step(HOME, 0.5)) is False, 'moved by exactly the delta: not more'
    clock.advance(1)
    assert publisher.report(step(HOME, 0.4)) is False
    clock.advance(1)
    assert publisher.report(step(HOME, 0.51)) is True, 'more than the delta since the last publish'
    assert sink.positions == [list(HOME), list(step(HOME, 0.51))]
    clock.advance(1)
    one_joint = list(step(HOME, 0.51))
    one_joint[3] -= 0.6
    assert publisher.report(one_joint) is True, 'any single joint moving is enough'
    assert publisher.latest is not None and publisher.latest.positions == tuple(one_joint)


def test_delta_is_measured_against_the_last_publication_not_the_last_report(
    clock: FakeClock, scheduler: FakeScheduler
) -> None:
    publisher, sink = make(clock, scheduler, min_delta=0.5)
    assert publisher.report(HOME)
    for _ in range(10):
        clock.advance(1)
        publisher.report(step(publisher.latest.positions, 0.25))  # type: ignore[union-attr]
    # Ten creeping reports of 0.25 each: published whenever the drift since the last publish
    # passes 0.5, so at 0.75, 1.5 and 2.25 (0.5 and 1.25 are not more than the delta).
    assert sink.positions == [
        list(HOME),
        list(step(HOME, 0.75)),
        list(step(HOME, 1.5)),
        list(step(HOME, 2.25)),
    ]
    assert sink.seqs == [1, 2, 3, 4]


def test_min_delta_zero_publishes_any_change_but_not_a_repeat(
    clock: FakeClock, scheduler: FakeScheduler
) -> None:
    publisher, sink = make(clock, scheduler, min_delta=0.0)
    assert publisher.report(HOME)
    clock.advance(1)
    assert publisher.report(HOME) is False, 'identical: nothing moved'
    clock.advance(1)
    assert publisher.report(step(HOME, 1e-6)) is True
    assert len(sink.records) == 2


# --------------------------------------------------------------------------- connect and announce


def test_republish_sends_the_latest_whatever_the_delta_and_the_rate(
    clock: FakeClock, scheduler: FakeScheduler
) -> None:
    publisher, sink = make(clock, scheduler)
    assert publisher.republish() is False, 'nothing seen yet; the next report publishes'
    assert publisher.report(HOME) is True
    assert publisher.republish() is True, 'at once, inside the interval, nothing moved'
    assert sink.seqs == [1, 2] and sink.positions == [list(HOME), list(HOME)]
    clock.advance(0.1)
    assert publisher.report(step(HOME, 0.1)) is False, 'back to the ordinary rules'
    assert publisher.republish() is True
    assert sink.positions[-1] == list(step(HOME, 0.1)), 'the latest report, published or not'
    assert sink.seqs == [1, 2, 3] and not publisher.timer_pending


def test_republish_arms_the_next_report_when_the_first_one_was_held(
    clock: FakeClock, scheduler: FakeScheduler
) -> None:
    publisher, sink = make(clock, scheduler)
    assert publisher.report(HOME)
    clock.advance(0.1)
    publisher.report(step(HOME, 1.0))
    publisher.close()
    assert scheduler.pending == [], 'close cancels the trailing timer'
    assert publisher.report(step(HOME, 2.0)) is False and publisher.republish() is False
    assert len(sink.records) == 1, 'nothing after close'


# --------------------------------------------------------------------------- seq and failures


def test_failed_publish_consumes_no_seq_and_is_retried_by_the_next_report(
    clock: FakeClock, scheduler: FakeScheduler
) -> None:
    publisher, sink = make(clock, scheduler)
    sink.ok = False
    assert publisher.report(HOME) is False
    assert publisher.seq == 0 and publisher.failed == 1 and publisher.published == 0
    assert publisher.latest is not None and publisher.last_published is None
    sink.ok = True
    clock.advance(0.1)
    assert publisher.report(step(HOME, 0.1)) is True, 'still forced: the first publish never went'
    assert sink.seqs == [1] and publisher.seq == 1
    sink.ok = False
    clock.advance(1)
    assert publisher.report(step(HOME, 5.0)) is False
    sink.ok = True
    clock.advance(0.01)
    assert publisher.report(step(HOME, 5.0)) is True, 'no interval runs from a failed publish'
    assert sink.seqs == [1, 2]


def test_publish_that_raises_is_logged_and_counted(
    clock: FakeClock, scheduler: FakeScheduler, caplog: pytest.LogCaptureFixture
) -> None:
    def broken(topic: str, payload: Any, qos: int, retain: bool, expiry_s: int | None) -> bool:
        raise RuntimeError('boom')

    publisher = JointsPublisher(
        robot_id=ROBOT,
        session=SESSION,
        instance=INSTANCE,
        clock=clock,
        scheduler=scheduler,
        publish=broken,
        joint_count=7,
    )
    with caplog.at_level(logging.ERROR, logger='ros2_ha_bridge.joints_publisher'):
        assert publisher.report(HOME) is False
    assert publisher.failed == 1 and publisher.seq == 0
    assert any('raised' in r.getMessage() for r in caplog.records)


@pytest.mark.parametrize(
    ('positions', 'velocities'),
    [
        (HOME[:6], None),
        (HOME + (0.0,), None),
        ((), None),
        ('0,0,0,0,0,0,0', None),
        ((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'x'), None),
        ((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, True), None),
        ((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float('nan')), None),
        ((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float('inf')), None),
        (HOME, HOME[:6]),
        (HOME, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 'fast')),
        (HOME, 'none'),
    ],
)
def test_bad_reports_are_dropped_and_warned_about_once(
    clock: FakeClock,
    scheduler: FakeScheduler,
    caplog: pytest.LogCaptureFixture,
    positions: Any,
    velocities: Any,
) -> None:
    publisher, sink = make(clock, scheduler)
    with caplog.at_level(logging.WARNING, logger='ros2_ha_bridge.joints_publisher'):
        assert publisher.report(positions, velocities) is False
        assert publisher.report(positions, velocities) is False
    assert publisher.dropped == 2 and sink.records == [] and publisher.latest is None
    warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
    assert len(warnings) == 1, 'one warning per reason'
    assert publisher.report(HOME) is True, 'a good report still goes through'


def test_integers_and_mixed_sequences_are_accepted_as_numbers(
    clock: FakeClock, scheduler: FakeScheduler
) -> None:
    publisher, sink = make(clock, scheduler, joint_count=3)
    assert publisher.report([1, 2.5, 3], (0, 0, 1)) is True
    assert sink.records[0].payload['positions'] == [1.0, 2.5, 3.0]
    assert sink.records[0].payload['velocities'] == [0.0, 0.0, 1.0]
    assert all(isinstance(v, float) for v in sink.records[0].payload['positions'])
