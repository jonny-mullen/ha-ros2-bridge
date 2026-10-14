# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Tests for :mod:`ros2_ha_bridge.watchdog`: the three modes, the clamp rule and the offset."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import pytest

from ros2_ha_bridge import protocol
from ros2_ha_bridge.authoriser import resolve_request
from ros2_ha_bridge.executors.base import (
    BridgeContext,
    TaskExecutor,
    TaskHandle,
    TaskRequest,
    TaskSpec,
)
from ros2_ha_bridge.protocol import DecodedCommand, HaHeartbeat
from ros2_ha_bridge.task_manager import TaskManager
from ros2_ha_bridge.watchdog import MIN_TIMEOUT_FACTOR, TICK_S, Watchdog

from .conftest import assert_imports_without
from .fakes import FakeClock, FakeScheduler

ROBOT = 'arm_left'
SESSION = '9f1c7a0e-4a1b-4b2c-8d1e-0f1a2b3c4d5e'
WAIT = TaskSpec('wait', 'Wait', 'motion', 60)


class Target:
    """Records what the watchdog asks of the task manager."""

    def __init__(self) -> None:
        self.links: list[tuple[bool, bool | None]] = []
        self.holds: list[str] = []

    def set_ha_link(self, up: bool, *, inhibited: bool | None = None) -> None:
        self.links.append((up, inhibited))

    def stop_and_hold(self, reason: str) -> None:
        self.holds.append(reason)


class Rig:
    def __init__(self, clock: FakeClock, mode: Any = 'none', timeout_s: float = 5.0) -> None:
        self.clock = clock
        self.target = Target()
        self.changes: list[bool] = []
        self.watchdog = Watchdog(
            clock=clock,
            mode=mode,
            timeout_s=timeout_s,
            target=self.target,
            on_link_change=self.changes.append,
        )

    def beat(self, interval_ms: int = 2000, *, seq: int = 1, offset_ms: int = 0) -> None:
        self.watchdog.ha_heartbeat(self.clock.now_ms() - offset_ms, seq, interval_ms)


def test_module_imports_neither_rclpy_nor_paho() -> None:
    assert_imports_without('ros2_ha_bridge.watchdog', {'rclpy', 'paho'})


def test_construction_validates_mode_and_timeout(clock: FakeClock) -> None:
    with pytest.raises(ValueError):
        Watchdog(clock=clock, mode='panic')  # type: ignore[arg-type]
    with pytest.raises(ValueError):
        Watchdog(clock=clock, timeout_s=0)
    watchdog = Watchdog(clock=clock)
    assert watchdog.mode == 'none' and watchdog.timeout_s == 5.0
    assert not watchdog.link_up and watchdog.ha_link == 'down'
    assert watchdog.ha_clock_offset_ms is None and watchdog.announced_interval_ms is None
    assert watchdog.last_seen_monotonic is None and watchdog.last_seq is None
    assert watchdog.losses == 0 and watchdog.heartbeats == 0


# --------------------------------------------------------------------------- modes


def test_mode_none_only_records(clock: FakeClock) -> None:
    rig = Rig(clock, 'none')
    watchdog = rig.watchdog
    assert watchdog.tick() is False
    assert rig.target.links == [(False, None)] and rig.changes == [False]
    assert watchdog.losses == 0, 'starting down is not a loss'

    rig.beat()
    assert watchdog.link_up and watchdog.ha_link_ok()
    assert rig.target.links[-1] == (True, None) and rig.changes == [False, True]
    assert watchdog.heartbeats == 1

    clock.advance(5.0)
    assert watchdog.tick() is True
    clock.advance(0.01)
    assert watchdog.tick() is False
    assert rig.target.links[-1] == (False, None) and rig.changes == [False, True, False]
    assert watchdog.losses == 1
    assert rig.target.holds == [], 'mode none never inhibits or stops'
    assert all(inhibited is None for _, inhibited in rig.target.links)


def test_inhibit_new_inhibits_while_down(clock: FakeClock) -> None:
    rig = Rig(clock, 'inhibit_new')
    rig.watchdog.tick()
    assert rig.target.links == [(False, True)], 'down from start-up until HA appears'
    rig.beat()
    assert rig.target.links[-1] == (True, False)
    clock.advance(5.01)
    rig.watchdog.tick()
    assert rig.target.links[-1] == (False, True)
    for _ in range(5):
        clock.advance(1)
        rig.watchdog.tick()
    assert len(rig.target.links) == 3, 'no repeat while the link stays down'
    assert rig.target.holds == []
    rig.beat()
    assert rig.target.links[-1] == (True, False)


def test_stop_and_hold_fires_once_per_loss(clock: FakeClock) -> None:
    rig = Rig(clock, 'stop_and_hold')
    rig.watchdog.tick()
    assert rig.target.holds == [], 'never at start-up'
    rig.beat()
    clock.advance(5.01)
    rig.watchdog.tick()
    assert rig.target.holds == ['Home Assistant heartbeat lost for more than 5 s']
    assert rig.target.links[-1] == (False, True)
    for _ in range(10):
        clock.advance(0.5)
        rig.watchdog.tick()
    assert len(rig.target.holds) == 1, 'once per loss, not once per tick'
    rig.beat(seq=2)
    assert rig.watchdog.link_up
    clock.advance(6)
    rig.watchdog.tick()
    assert len(rig.target.holds) == 2 and rig.watchdog.losses == 2
    assert rig.changes == [False, True, False, True, False]


def test_heartbeat_brings_the_link_up_at_once(clock: FakeClock) -> None:
    rig = Rig(clock, 'inhibit_new')
    rig.watchdog.tick()
    clock.advance(30)
    rig.beat()
    assert rig.watchdog.link_up, 'no tick needed'
    assert rig.target.links[-1] == (True, False)


# --------------------------------------------------------------------------- clamp rule


def test_timeout_is_clamped_to_two_and_a_half_announced_intervals(clock: FakeClock) -> None:
    rig = Rig(clock, 'none', timeout_s=5.0)
    watchdog = rig.watchdog
    assert watchdog.effective_timeout_s == 5.0, 'the configured value before any heartbeat'
    rig.beat(interval_ms=4000)
    assert MIN_TIMEOUT_FACTOR == 2.5
    assert watchdog.effective_timeout_s == 10.0
    clock.advance(6)
    assert watchdog.tick() is True, 'a 5 s timeout cannot trip a 4 s heartbeat'
    clock.advance(4)
    assert watchdog.tick() is True
    clock.advance(0.01)
    assert watchdog.tick() is False

    rig.beat(interval_ms=1000, seq=2)
    assert watchdog.effective_timeout_s == 5.0, 'the configured value when it is the larger'
    clock.advance(5.01)
    assert watchdog.tick() is False


# --------------------------------------------------------------------------- offset


def test_clock_offset_is_node_clock_minus_ha_clock(clock: FakeClock) -> None:
    rig = Rig(clock)
    rig.beat(offset_ms=12)
    assert rig.watchdog.ha_clock_offset_ms == 12
    rig.beat(offset_ms=-250, seq=2)
    assert rig.watchdog.ha_clock_offset_ms == -250
    assert rig.watchdog.last_seq == 2 and rig.watchdog.announced_interval_ms == 2000
    assert rig.watchdog.last_seen_monotonic == clock.monotonic()
    clock.set_now_ms(clock.now_ms() + 1_000)
    rig.beat(offset_ms=0, seq=3)
    assert rig.watchdog.ha_clock_offset_ms == 0


def test_feed_accepts_a_decoded_heartbeat(clock: FakeClock, valid_examples: Any) -> None:
    golden = valid_examples('heartbeat_ha')['steady']
    beat = protocol.decode_ha_heartbeat(golden.raw)
    assert isinstance(beat, HaHeartbeat)
    watchdog = Watchdog(clock=clock)
    watchdog.feed(beat)
    assert watchdog.link_up
    assert watchdog.ha_clock_offset_ms == clock.now_ms() - beat.ts
    assert watchdog.announced_interval_ms == beat.interval_ms
    assert watchdog.last_seq == beat.seq


# --------------------------------------------------------------------------- scheduler chain


def test_scheduler_chain_ticks_every_half_second(
    clock: FakeClock, scheduler: FakeScheduler
) -> None:
    rig = Rig(clock, 'none')
    rig.watchdog.start(scheduler)
    rig.watchdog.start(scheduler)
    assert len(scheduler.pending) == 1 and TICK_S == 0.5
    assert rig.target.links == [(False, None)], 'start evaluates at once'
    rig.beat()
    clock.advance(5.0)
    assert rig.watchdog.link_up
    clock.advance(0.5)
    assert not rig.watchdog.link_up, 'decided by the tick after the timeout passed'
    assert scheduler.fired_count == 11
    rig.watchdog.stop()
    assert scheduler.pending == []
    rig.beat(seq=2)
    clock.advance(60)
    assert rig.watchdog.link_up, 'nothing ticks after stop'


# --------------------------------------------------------------------------- with the task manager


class SilentExecutor(TaskExecutor):
    name = 'silent'
    version = '0'

    def __init__(self) -> None:
        self.cancelled: list[str] = []
        self.holds: list[str] = []

    def setup(self, ctx: BridgeContext) -> None:
        pass

    def tasks(self) -> Sequence[TaskSpec]:
        return [WAIT]

    def start(self, req: TaskRequest, handle: TaskHandle) -> None:
        pass

    def cancel(self, task_id: str) -> None:
        self.cancelled.append(task_id)

    def stop_and_hold(self, reason: str) -> None:
        self.holds.append(reason)


def test_stop_and_hold_mode_drives_a_real_task_manager(
    clock: FakeClock, scheduler: FakeScheduler, schema_validator: Any
) -> None:
    published: list[tuple[str, dict[str, Any]]] = []
    executor = SilentExecutor()
    manager = TaskManager(
        robot_id=ROBOT,
        executor=executor,
        publish=lambda topic, payload, qos, retain, expiry: (
            published.append((topic, dict(payload))) or True
        ),
        post=lambda fn: fn(),
        clock=clock,
        scheduler=scheduler,
        session=SESSION,
        instance='robotpc:1',
        watchdog_mode='stop_and_hold',
    )
    watchdog = Watchdog(clock=clock, mode='stop_and_hold', timeout_s=5.0, target=manager)
    watchdog.start(scheduler)
    assert manager.inhibited and manager.ha_link == 'down'
    assert manager.motion_rejection() == (
        'rejected_inhibited',
        'New motion is inhibited while the HA link is down',
    )

    watchdog.ha_heartbeat(clock.now_ms(), 1, 2000)
    assert not manager.inhibited and manager.ha_link == 'up'
    command = protocol.decode_command(
        protocol.encode_payload(
            {
                'schema': 1,
                'id': '6a0c2b58-1d7e-4a1f-9b2a-3c4d5e6f7a8b',
                'type': 'task',
                'robot_id': ROBOT,
                'task': 'wait',
                'params': {},
                'issued_at': clock.now_ms(),
                'max_age_ms': 5000,
                'source': {'origin': 'ha', 'kind': 'test'},
            }
        )
    )
    assert isinstance(command, DecodedCommand)
    assert manager.start(resolve_request(command, WAIT), WAIT).code == 'accepted'

    clock.advance(5.5)
    assert not watchdog.link_up
    assert executor.holds == ['Home Assistant heartbeat lost for more than 5 s']
    assert executor.cancelled == [command.id]
    results = [p for t, p in published if t.endswith('/result')]
    assert len(results) == 1
    schema_validator.assert_valid('result', results[0])
    assert results[0]['status'] == 'cancelled' and results[0]['code'] == 'inhibited'
    state = [p for t, p in published if t.endswith('/state')][-1]
    schema_validator.assert_valid('state', state)
    assert state['state'] == 'idle' and state['inhibited'] is True
    assert state['ha_link'] == 'down' and state['watchdog_mode'] == 'stop_and_hold'
    assert state['error'] is None

    clock.advance(10)
    assert len(executor.holds) == 1
    watchdog.ha_heartbeat(clock.now_ms(), 2, 2000)
    assert not manager.inhibited and manager.motion_rejection() is None
    watchdog.stop()
