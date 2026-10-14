# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Tests for :mod:`ros2_ha_bridge.dedupe_store`.

The file round trip, the once-per-second rewrite rule (with and without a
scheduler), age pruning at load, the memory window and size bounds, an
unwritable directory falling back to memory, a write failure after start-up,
corrupt and foreign files, and a publisher-side QoS 1 resend across a
simulated node restart finding the stored ack.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any

import pytest

from ros2_ha_bridge import protocol
from ros2_ha_bridge.dedupe_store import (
    DEFAULT_KEEP_ON_LOAD_MS,
    FILE_NAME,
    FILE_VERSION,
    DedupeEntry,
    DedupeStore,
)

from .conftest import assert_imports_without
from .fakes import DEFAULT_EPOCH_MS, FakeClock, FakeScheduler

ROBOT = 'arm_left'
SESSION = '9f1c7a0e-4a1b-4b2c-8d1e-0f1a2b3c4d5e'
ID_A = '6a0c2b58-1d7e-4a1f-9b2a-3c4d5e6f7a8b'
ID_B = '51f0a2b3-c4d5-4e6f-a7b8-c9d0e1f2a3b4'
ID_C = '0c1d4e6f-2a3b-4c5d-8e9f-0a1b2c3d4e5f'
ID_D = '7b3e9d21-5f6a-4c8b-9e0d-1f2a3b4c5d6e'
LOGGER_NAME = 'ros2_ha_bridge.dedupe_store'


def _ack_bytes(
    command_id: str, code: Any = 'accepted', received_at: int = DEFAULT_EPOCH_MS
) -> bytes:
    """Wire bytes of a plausible ack for ``command_id``, as the authoriser would store them."""
    payload = protocol.AckPayload(
        id=command_id,
        robot_id=ROBOT,
        type='task',
        task='reset',
        code=code,
        reason='' if code == 'accepted' else 'why',
        received_at=received_at,
        clock_skew_ms=0,
        session=SESSION,
        ts=received_at,
    ).to_payload()
    return protocol.encode_payload(payload)


def _file_entries(path: Path) -> list[dict[str, Any]]:
    document = json.loads(path.read_text(encoding='utf-8'))
    assert document['version'] == FILE_VERSION
    entries: list[dict[str, Any]] = document['entries']
    return entries


def _write_file(data_dir: Path, entries: list[Any], version: int = FILE_VERSION) -> Path:
    data_dir.mkdir(parents=True, exist_ok=True)
    path = data_dir / FILE_NAME
    path.write_text(json.dumps({'version': version, 'entries': entries}), encoding='utf-8')
    return path


def _warnings(caplog: pytest.LogCaptureFixture) -> list[str]:
    return [r.getMessage() for r in caplog.records if r.levelno >= logging.WARNING]


# --------------------------------------------------------------------------- hygiene


def test_module_imports_neither_rclpy_nor_paho() -> None:
    assert_imports_without('ros2_ha_bridge.dedupe_store', {'rclpy', 'paho'})


def test_constructor_validates_arguments(clock: FakeClock) -> None:
    with pytest.raises(ValueError):
        DedupeStore(clock=clock, max_size=0)
    with pytest.raises(ValueError):
        DedupeStore(clock=clock, window_s=0)
    with pytest.raises(ValueError):
        DedupeStore(clock=clock, keep_on_load_ms=-1)
    with pytest.raises(ValueError):
        DedupeStore(clock=clock, min_write_interval_s=-0.5)
    assert DEFAULT_KEEP_ON_LOAD_MS == (
        protocol.DEFAULT_MAX_AGE_CAP_MS + protocol.DEFAULT_CLOCK_SKEW_TOLERANCE_MS
    )


# --------------------------------------------------------------------------- memory


def test_memory_only_store_remembers_and_looks_up(clock: FakeClock) -> None:
    store = DedupeStore(clock=clock, data_dir=None)
    assert store.path is None
    assert not store.persistent
    assert store.lookup(ID_A) is None
    ack = _ack_bytes(ID_A)
    store.remember(ID_A, clock.now_ms(), ack, accepted=True)
    entry = store.lookup(ID_A)
    assert entry == DedupeEntry(clock.now_ms(), ack, True)
    assert ID_A in store and ID_B not in store
    assert len(store) == 1
    assert not store.pending_write
    assert store.flush() is False
    assert store.writes == 0
    store.remember(ID_B, clock.now_ms(), _ack_bytes(ID_B, 'rejected_busy'), accepted=False)
    rejected = store.lookup(ID_B)
    assert rejected is not None and not rejected.accepted
    assert list(store.entries) == [ID_A, ID_B]


def test_entries_view_is_read_only(clock: FakeClock) -> None:
    store = DedupeStore(clock=clock)
    store.remember(ID_A, clock.now_ms(), _ack_bytes(ID_A), accepted=True)
    with pytest.raises(TypeError):
        store.entries[ID_B] = DedupeEntry(0, b'', True)  # type: ignore[index]


def test_window_prunes_old_entries_on_lookup(clock: FakeClock) -> None:
    store = DedupeStore(clock=clock, window_s=10)
    store.remember(ID_A, clock.now_ms(), _ack_bytes(ID_A), accepted=True)
    clock.advance(10)
    assert store.lookup(ID_A) is not None, 'an entry exactly at the window edge is kept'
    clock.advance(0.001)
    assert store.lookup(ID_A) is None
    assert len(store) == 0


def test_size_bound_evicts_the_oldest(clock: FakeClock) -> None:
    store = DedupeStore(clock=clock, max_size=2)
    for command_id in (ID_A, ID_B, ID_C):
        store.remember(command_id, clock.now_ms(), _ack_bytes(command_id), accepted=True)
    assert list(store.entries) == [ID_B, ID_C]
    assert store.lookup(ID_A) is None


def test_remember_twice_replaces_and_moves_to_the_end(clock: FakeClock) -> None:
    store = DedupeStore(clock=clock)
    first = _ack_bytes(ID_A)
    store.remember(ID_A, clock.now_ms(), first, accepted=True)
    store.remember(ID_B, clock.now_ms(), _ack_bytes(ID_B), accepted=True)
    second = _ack_bytes(ID_A, 'rejected_executor')
    store.remember(ID_A, clock.now_ms() + 1, second, accepted=False)
    assert list(store.entries) == [ID_B, ID_A]
    entry = store.lookup(ID_A)
    assert entry is not None and entry.ack_bytes == second and not entry.accepted


# --------------------------------------------------------------------------- file


def test_file_round_trip_keeps_accepted_entries_only(clock: FakeClock, tmp_path: Path) -> None:
    first = DedupeStore(clock=clock, data_dir=tmp_path)
    assert first.path == tmp_path / FILE_NAME
    assert first.persistent and first.loaded == 0 and first.writes == 0
    accepted = _ack_bytes(ID_A)
    received_at = clock.now_ms()
    first.remember(ID_A, received_at, accepted, accepted=True)
    assert first.writes == 1 and not first.pending_write
    assert first.path.is_file()
    assert not (tmp_path / (FILE_NAME + '.tmp')).exists(), 'the temporary file was replaced'
    document = json.loads(first.path.read_text(encoding='utf-8'))
    assert document['version'] == FILE_VERSION
    assert document['written_at_ms'] == clock.now_ms()
    assert document['entries'] == [
        {'id': ID_A, 'received_at_ms': received_at, 'ack': accepted.decode('utf-8')}
    ]

    first.remember(ID_B, clock.now_ms(), _ack_bytes(ID_B, 'rejected_busy'), accepted=False)
    assert first.writes == 1, 'a rejection is memory-only'
    assert [e['id'] for e in _file_entries(first.path)] == [ID_A]

    second = DedupeStore(clock=clock, data_dir=tmp_path)
    assert second.loaded == 1
    entry = second.lookup(ID_A)
    assert entry == DedupeEntry(received_at, accepted, True)
    assert entry is not None and entry.ack_bytes == accepted
    assert ID_B not in second


def test_non_ascii_reason_survives_the_file(clock: FakeClock, tmp_path: Path) -> None:
    payload = json.loads(_ack_bytes(ID_A))
    payload['reason'] = 'Küche 厨房'
    ack = protocol.encode_payload(payload)
    DedupeStore(clock=clock, data_dir=tmp_path).remember(ID_A, clock.now_ms(), ack, accepted=True)
    reloaded = DedupeStore(clock=clock, data_dir=tmp_path).lookup(ID_A)
    assert reloaded is not None and reloaded.ack_bytes == ack


def test_once_per_second_coalesces_writes_with_a_scheduler(
    clock: FakeClock, scheduler: FakeScheduler, tmp_path: Path
) -> None:
    store = DedupeStore(clock=clock, data_dir=tmp_path, scheduler=scheduler)
    store.remember(ID_A, clock.now_ms(), _ack_bytes(ID_A), accepted=True)
    assert store.writes == 1
    store.remember(ID_B, clock.now_ms(), _ack_bytes(ID_B), accepted=True)
    assert (store.writes, store.pending_write) == (1, True)
    assert len(scheduler.pending) == 1
    clock.advance(0.5)
    assert store.writes == 1
    store.remember(ID_C, clock.now_ms(), _ack_bytes(ID_C), accepted=True)
    assert len(scheduler.pending) == 1, 'one timer covers every deferred write'
    clock.advance(0.5)
    assert (store.writes, store.pending_write) == (2, False)
    assert store.path is not None
    assert [e['id'] for e in _file_entries(store.path)] == [ID_A, ID_B, ID_C]
    clock.advance(0.2)
    store.remember(ID_D, clock.now_ms(), _ack_bytes(ID_D), accepted=True)
    assert store.pending_write
    assert store.flush() is True
    assert store.writes == 3 and scheduler.pending == []
    assert store.flush() is False


def test_zero_write_interval_writes_at_every_accepted_ack(
    clock: FakeClock, scheduler: FakeScheduler, tmp_path: Path
) -> None:
    """``command.dedupe_write_interval_s`` 0: no coalescing, so no crash window."""
    store = DedupeStore(
        clock=clock, data_dir=tmp_path, scheduler=scheduler, min_write_interval_s=0
    )
    store.remember(ID_A, clock.now_ms(), _ack_bytes(ID_A), accepted=True)
    store.remember(ID_B, clock.now_ms(), _ack_bytes(ID_B), accepted=True)
    store.remember(ID_C, clock.now_ms(), _ack_bytes(ID_C), accepted=True)
    assert (store.writes, store.pending_write) == (3, False)
    assert scheduler.pending == []
    assert store.path is not None
    assert [e['id'] for e in _file_entries(store.path)] == [ID_A, ID_B, ID_C]
    slow = DedupeStore(
        clock=clock, data_dir=tmp_path / 'slow', scheduler=scheduler, min_write_interval_s=30
    )
    slow.remember(ID_A, clock.now_ms(), _ack_bytes(ID_A), accepted=True)
    slow.remember(ID_B, clock.now_ms(), _ack_bytes(ID_B), accepted=True)
    assert (slow.writes, slow.pending_write) == (1, True)
    clock.advance(29)
    assert slow.writes == 1
    clock.advance(1)
    assert (slow.writes, slow.pending_write) == (2, False)


def test_once_per_second_without_a_scheduler_writes_at_the_next_chance(
    clock: FakeClock, tmp_path: Path
) -> None:
    store = DedupeStore(clock=clock, data_dir=tmp_path)
    store.remember(ID_A, clock.now_ms(), _ack_bytes(ID_A), accepted=True)
    store.remember(ID_B, clock.now_ms(), _ack_bytes(ID_B), accepted=True)
    assert (store.writes, store.pending_write) == (1, True)
    clock.advance(2)
    assert store.writes == 1, 'nothing fires without a scheduler'
    store.remember(ID_C, clock.now_ms(), _ack_bytes(ID_C), accepted=True)
    assert (store.writes, store.pending_write) == (2, False)
    assert store.path is not None
    assert [e['id'] for e in _file_entries(store.path)] == [ID_A, ID_B, ID_C]
    assert store.flush() is False


def test_age_pruning_at_load(clock: FakeClock, tmp_path: Path) -> None:
    now = clock.now_ms()
    cutoff = now - DEFAULT_KEEP_ON_LOAD_MS
    _write_file(
        tmp_path,
        [
            {'id': ID_A, 'received_at_ms': cutoff - 1, 'ack': _ack_bytes(ID_A).decode()},
            {'id': ID_B, 'received_at_ms': cutoff, 'ack': _ack_bytes(ID_B).decode()},
            {'id': ID_C, 'received_at_ms': now, 'ack': _ack_bytes(ID_C).decode()},
        ],
    )
    store = DedupeStore(clock=clock, data_dir=tmp_path)
    assert store.loaded == 2
    assert ID_A not in store, 'older than max_age_cap_ms plus skew: no resend can pass freshness'
    assert ID_B in store and ID_C in store
    strict = DedupeStore(clock=clock, data_dir=tmp_path, keep_on_load_ms=0)
    assert list(strict.entries) == [ID_C]


def test_load_skips_unusable_entries_and_warns(
    clock: FakeClock, tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    good = _ack_bytes(ID_A).decode()
    _write_file(
        tmp_path,
        [
            'not an object',
            {'id': 'cmd-123', 'received_at_ms': clock.now_ms(), 'ack': good},
            {'id': ID_B, 'received_at_ms': clock.now_ms(), 'ack': good},
            {'id': ID_C, 'received_at_ms': clock.now_ms(), 'ack': '{not json'},
            {'id': ID_D, 'received_at_ms': 1.5, 'ack': _ack_bytes(ID_D).decode()},
            {'id': ID_A, 'received_at_ms': clock.now_ms(), 'ack': good},
        ],
    )
    with caplog.at_level(logging.WARNING, logger=LOGGER_NAME):
        store = DedupeStore(clock=clock, data_dir=tmp_path)
    assert store.loaded == 1 and list(store.entries) == [ID_A]
    assert any('skipped 5 unusable' in line for line in _warnings(caplog))


@pytest.mark.parametrize(
    ('text', 'fragment'),
    [
        ('{not json', 'corrupt'),
        ('[]', 'not a version'),
        ('{"version": 2, "entries": []}', 'not a version'),
        ('{"version": 1, "entries": {}}', 'no entries list'),
    ],
)
def test_unreadable_file_is_ignored_and_then_overwritten(
    clock: FakeClock, tmp_path: Path, caplog: pytest.LogCaptureFixture, text: str, fragment: str
) -> None:
    path = tmp_path / FILE_NAME
    path.write_text(text, encoding='utf-8')
    with caplog.at_level(logging.WARNING, logger=LOGGER_NAME):
        store = DedupeStore(clock=clock, data_dir=tmp_path)
    assert store.loaded == 0 and store.persistent
    assert any(fragment in line for line in _warnings(caplog))
    store.remember(ID_A, clock.now_ms(), _ack_bytes(ID_A), accepted=True)
    assert [e['id'] for e in _file_entries(path)] == [ID_A]


def test_missing_directory_is_created(clock: FakeClock, tmp_path: Path) -> None:
    data_dir = tmp_path / 'deep' / 'er'
    store = DedupeStore(clock=clock, data_dir=data_dir)
    assert data_dir.is_dir() and store.persistent
    store.remember(ID_A, clock.now_ms(), _ack_bytes(ID_A), accepted=True)
    assert (data_dir / FILE_NAME).is_file()


def test_unwritable_directory_falls_back_to_memory(
    clock: FakeClock, tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    blocker = tmp_path / 'blocker'
    blocker.write_text('a file where a directory is needed', encoding='utf-8')
    with caplog.at_level(logging.WARNING, logger=LOGGER_NAME):
        store = DedupeStore(clock=clock, data_dir=blocker / 'sub')
    assert not store.persistent
    warnings = _warnings(caplog)
    assert len(warnings) == 1 and 'memory-only' in warnings[0]
    store.remember(ID_A, clock.now_ms(), _ack_bytes(ID_A), accepted=True)
    assert store.lookup(ID_A) is not None
    assert store.writes == 0 and not store.pending_write
    assert store.flush() is False
    assert not (blocker / 'sub').exists()
    assert len(_warnings(caplog)) == 1, 'the fallback is reported once, not per ack'


def test_write_failure_after_start_disables_persistence_once(
    clock: FakeClock, tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    store = DedupeStore(clock=clock, data_dir=tmp_path)
    assert store.path is not None
    store.path.mkdir()  # a directory where the file goes makes os.replace fail everywhere
    with caplog.at_level(logging.WARNING, logger=LOGGER_NAME):
        store.remember(ID_A, clock.now_ms(), _ack_bytes(ID_A), accepted=True)
        store.remember(ID_B, clock.now_ms(), _ack_bytes(ID_B), accepted=True)
    assert not store.persistent and store.writes == 0
    warnings = _warnings(caplog)
    assert len(warnings) == 1 and 'cannot write' in warnings[0]
    assert not (tmp_path / (FILE_NAME + '.tmp')).exists(), 'the temporary file is cleaned up'
    assert store.lookup(ID_A) is not None and store.lookup(ID_B) is not None
    assert store.flush() is False


# --------------------------------------------------------------------------- restart


def test_qos1_resend_across_a_restart_finds_the_stored_ack(
    clock: FakeClock, tmp_path: Path
) -> None:
    """A publisher-side resend after a node restart must meet the ack the old process stored."""
    old_process = DedupeStore(clock=clock, data_dir=tmp_path)
    stale = _ack_bytes(ID_B, received_at=clock.now_ms() - 40_000)
    old_process.remember(ID_B, clock.now_ms() - 40_000, stale, accepted=True)
    ack = _ack_bytes(ID_A)
    old_process.remember(ID_A, clock.now_ms(), ack, accepted=True)
    assert old_process.pending_write, 'within a second of the first write: deferred'
    assert old_process.flush() is True, 'a clean shutdown flushes the deferred write'
    del old_process

    clock.advance(3)  # the restart takes a moment; the resend arrives within the freshness window
    new_process = DedupeStore(clock=clock, data_dir=tmp_path)
    assert new_process.loaded == 1
    entry = new_process.lookup(ID_A)
    assert entry is not None and entry.ack_bytes == ack and entry.accepted
    assert new_process.lookup(ID_B) is None, 'too old for any resend to pass freshness'


def test_pending_write_must_be_flushed_before_a_restart(
    clock: FakeClock, scheduler: FakeScheduler, tmp_path: Path
) -> None:
    """Documents the one-second window: an unflushed accept is lost by a crash, kept by flush()."""
    store = DedupeStore(clock=clock, data_dir=tmp_path, scheduler=scheduler)
    store.remember(ID_A, clock.now_ms(), _ack_bytes(ID_A), accepted=True)
    store.remember(ID_B, clock.now_ms(), _ack_bytes(ID_B), accepted=True)
    assert store.pending_write
    crashed = DedupeStore(clock=clock, data_dir=tmp_path)
    assert list(crashed.entries) == [ID_A]
    assert store.flush() is True
    clean = DedupeStore(clock=clock, data_dir=tmp_path)
    assert list(clean.entries) == [ID_A, ID_B]
