# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Joint telemetry throttle: what ``report_joints`` delivers becomes ``joints`` publications.

:class:`JointsPublisher` sits between the executor's
:meth:`~ros2_ha_bridge.executors.base.BridgeContext.report_joints` and the
``{prefix}/{robot_id}/joints`` topic (QoS 0, never retained, Message Expiry
Interval 5 s). The contract's three rules live here and nowhere else:

- at most ``rate_hz`` publications per second (``telemetry.joints_rate_hz``,
  default 2);
- only when some position moved by more than ``min_delta`` since the last
  publication (``telemetry.joints_min_delta``, default 0.5 for ``deg`` and
  0.01 for ``rad``);
- plus once on every (re)connect and announce, through :meth:`republish`.

A report that arrives inside the rate interval is not thrown away: the latest
sample is kept and a one-shot through the injected scheduler publishes it when
the interval has passed, if it still differs from the last publication. That
trailing publication is what lets the final resting pose of a motion reach
Home Assistant even when the executor's last report lands a few milliseconds
after the previous publication; without it a dashboard would stop one step
short of the real pose.

``seq`` increments per successful publication within the session. A report
whose length differs from the announced joint count, or that carries a value
that is not a finite number, is dropped and counted, never published, because
Home Assistant would drop it too.

This module imports neither rclpy nor paho: it takes a
:class:`~ros2_ha_bridge.clock.Clock`, a
:class:`~ros2_ha_bridge.clock.Scheduler` and the node's publish callable, so
the tests drive it with the fakes. Every method is main-thread only; the
context's ``report_joints`` posts onto that thread before calling
:meth:`report`.
"""

from __future__ import annotations

import logging
import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any

from ros2_ha_bridge.clock import Clock, Scheduler, TimerHandle
from ros2_ha_bridge.executors.base import MAX_JOINTS, JointUnit
from ros2_ha_bridge.protocol import (
    DEFAULT_PREFIX,
    JOINTS_EXPIRY_S,
    JointsPayload,
    joints_topic,
)

__all__ = [
    'DEFAULT_JOINTS_RATE_HZ',
    'DEFAULT_MIN_DELTA_BY_UNIT',
    'DEFAULT_MIN_DELTA_DEG',
    'DEFAULT_MIN_DELTA_RAD',
    'JointSample',
    'JointsPublishFn',
    'JointsPublisher',
    'default_min_delta',
]

#: ``telemetry.joints_rate_hz`` default: the most ``joints`` messages per second.
DEFAULT_JOINTS_RATE_HZ = 2.0
#: ``telemetry.joints_min_delta`` defaults per unit: a position must move by more than this.
DEFAULT_MIN_DELTA_DEG = 0.5
DEFAULT_MIN_DELTA_RAD = 0.01
DEFAULT_MIN_DELTA_BY_UNIT: Mapping[JointUnit, float] = {
    'deg': DEFAULT_MIN_DELTA_DEG,
    'rad': DEFAULT_MIN_DELTA_RAD,
}

#: ``publish(topic, payload, qos, retain, expiry_s) -> ok``: the node's guarded JSON publish.
JointsPublishFn = Callable[[str, Mapping[str, Any], int, bool, int | None], bool]

_LOGGER_NAME = 'ros2_ha_bridge.joints_publisher'
#: Slack on the rate comparison so a report exactly one interval later is not deferred.
_RATE_SLACK_S = 1e-9


def default_min_delta(unit: JointUnit) -> float:
    """The ``telemetry.joints_min_delta`` default for ``unit`` (0.5 deg, 0.01 rad)."""
    return DEFAULT_MIN_DELTA_BY_UNIT[unit]


@dataclass(frozen=True)
class JointSample:
    """One accepted report: positions, optional velocities and when it arrived."""

    positions: tuple[float, ...]
    velocities: tuple[float, ...] | None
    monotonic: float


class JointsPublisher:
    """Throttle and publish joint reports for one robot. See the module docstring.

    ``joint_count`` is the length every report must have (the announced
    ``names``); ``rate_hz`` must be greater than 0 (a node with the rate at 0
    builds no publisher and announces no joints); ``min_delta`` is in the
    announced unit. ``scheduler`` may be ``None``, in which case a report
    inside the rate interval is held until the next report instead of being
    published by a timer.
    """

    def __init__(
        self,
        *,
        robot_id: str,
        session: str,
        instance: str,
        clock: Clock,
        publish: JointsPublishFn,
        joint_count: int,
        rate_hz: float = DEFAULT_JOINTS_RATE_HZ,
        min_delta: float = DEFAULT_MIN_DELTA_DEG,
        scheduler: Scheduler | None = None,
        prefix: str = DEFAULT_PREFIX,
        logger: logging.Logger | None = None,
    ) -> None:
        if not 1 <= joint_count <= MAX_JOINTS:
            raise ValueError(f'joint_count must be between 1 and {MAX_JOINTS}, not {joint_count}')
        if not (isinstance(rate_hz, (int, float)) and math.isfinite(rate_hz) and rate_hz > 0):
            raise ValueError(f'rate_hz must be a finite number greater than 0, not {rate_hz!r}')
        if not (isinstance(min_delta, (int, float)) and math.isfinite(min_delta)) or min_delta < 0:
            raise ValueError(f'min_delta must be a finite number of at least 0, not {min_delta!r}')
        self.topic = joints_topic(prefix, robot_id)
        self.robot_id = robot_id
        self.session = session
        self.instance = instance
        self.joint_count = joint_count
        self.rate_hz = float(rate_hz)
        self.min_delta = float(min_delta)
        self._interval_s = 1.0 / self.rate_hz
        self._clock = clock
        self._scheduler = scheduler
        self._publish_fn = publish
        self._logger = logger if logger is not None else logging.getLogger(_LOGGER_NAME)
        self._latest: JointSample | None = None
        self._last_published: JointSample | None = None
        self._last_publish_monotonic: float | None = None
        self._seq = 0
        #: True until the first publication and after every :meth:`republish`.
        self._force = True
        self._timer: TimerHandle | None = None
        self._closed = False
        self._warned: set[str] = set()
        #: Lifetime counts, for logs and tests.
        self.published = 0
        self.failed = 0
        self.dropped = 0
        self.deferred = 0

    # ------------------------------------------------------------------ views

    @property
    def seq(self) -> int:
        """Sequence number of the last publication (0 before the first)."""
        return self._seq

    @property
    def latest(self) -> JointSample | None:
        """The most recent accepted report, published or not."""
        return self._latest

    @property
    def last_published(self) -> JointSample | None:
        """The report behind the last publication; the delta is measured against it."""
        return self._last_published

    @property
    def interval_s(self) -> float:
        """The least time between two publications, ``1 / rate_hz``."""
        return self._interval_s

    @property
    def timer_pending(self) -> bool:
        """True while a trailing publication is armed."""
        return self._timer is not None

    # ------------------------------------------------------------------ inputs (main thread)

    def report(
        self, positions: Sequence[float], velocities: Sequence[float] | None = None
    ) -> bool:
        """Take one report; return True when it was published at once.

        A report the contract would not accept (wrong length, a value that is
        not a finite number) is dropped with one warning per reason and
        counted in :attr:`dropped`. Otherwise it becomes :attr:`latest` and is
        published now, deferred to the trailing timer, or held because
        nothing moved.
        """
        if self._closed:
            return False
        sample = self._sample(positions, velocities)
        if sample is None:
            self.dropped += 1
            return False
        self._latest = sample
        return self._consider()

    def republish(self) -> bool:
        """Connect or announce: publish the latest report now, whatever the delta and the rate.

        With no report seen yet, the next one is published as soon as it
        arrives. Returns True when a message was published by this call.
        """
        if self._closed:
            return False
        self._force = True
        if self._latest is None:
            return False
        return self._consider()

    def close(self) -> None:
        """Cancel the trailing timer; later reports are ignored."""
        self._closed = True
        self._cancel_timer()

    # ------------------------------------------------------------------ the rules

    def _consider(self) -> bool:
        latest = self._latest
        assert latest is not None
        if not self._force:
            if not self._moved(latest):
                return False
            last = self._last_publish_monotonic
            if last is not None:
                elapsed = self._clock.monotonic() - last
                if elapsed < self._interval_s - _RATE_SLACK_S:
                    self._defer(self._interval_s - elapsed)
                    return False
        return self._publish(latest)

    def _moved(self, sample: JointSample) -> bool:
        last = self._last_published
        if last is None:
            return True
        return any(
            abs(new - old) > self.min_delta
            for new, old in zip(sample.positions, last.positions, strict=True)
        )

    def _defer(self, delay_s: float) -> None:
        self.deferred += 1
        if self._timer is not None or self._scheduler is None:
            return
        self._timer = self._scheduler.call_later(max(0.0, delay_s), self._on_timer)

    def _on_timer(self) -> None:
        self._timer = None
        if self._closed or self._latest is None:
            return
        self._consider()

    def _cancel_timer(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    def _publish(self, sample: JointSample) -> bool:
        self._cancel_timer()
        payload = JointsPayload(
            robot_id=self.robot_id,
            positions=sample.positions,
            velocities=sample.velocities,
            session=self.session,
            instance=self.instance,
            seq=self._seq + 1,
            ts=self._clock.now_ms(),
        ).to_payload()
        try:
            ok = self._publish_fn(self.topic, payload, 0, False, JOINTS_EXPIRY_S)
        except Exception:
            self._logger.exception(
                'publish to %s raised; the link should have caught this', self.topic
            )
            ok = False
        if not ok:
            # Not connected, or the link refused: the sample stays unpublished and the next
            # report (or the connect's republish) tries again. No seq is consumed.
            self.failed += 1
            self._logger.debug('joints publish to %s did not go out', self.topic)
            return False
        self._seq += 1
        self.published += 1
        self._force = False
        self._last_published = sample
        self._last_publish_monotonic = self._clock.monotonic()
        return True

    # ------------------------------------------------------------------ validation

    def _sample(
        self, positions: Sequence[float], velocities: Sequence[float] | None
    ) -> JointSample | None:
        values = _numbers(positions)
        if values is None:
            self._warn('positions_type', 'joint positions must be a sequence of finite numbers')
            return None
        if len(values) != self.joint_count:
            self._warn(
                'positions_length',
                f'{len(values)} joint position(s) reported for {self.joint_count} announced '
                'joint(s); the report is dropped',
            )
            return None
        speeds: tuple[float, ...] | None = None
        if velocities is not None:
            speeds = _numbers(velocities)
            if speeds is None or len(speeds) != len(values):
                self._warn(
                    'velocities',
                    'joint velocities must be null or one finite number per position; '
                    'the report is dropped',
                )
                return None
        return JointSample(values, speeds, self._clock.monotonic())

    def _warn(self, key: str, message: str) -> None:
        if key in self._warned:
            self._logger.debug('%s (repeated)', message)
            return
        self._warned.add(key)
        self._logger.warning(message)


def _numbers(values: object) -> tuple[float, ...] | None:
    """``values`` as a tuple of finite floats, or ``None`` when it is not such a sequence."""
    if isinstance(values, (str, bytes)) or not isinstance(values, Sequence):
        return None
    result: list[float] = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None
        number = float(value)
        if not math.isfinite(number):
            return None
        result.append(number)
    return tuple(result)
