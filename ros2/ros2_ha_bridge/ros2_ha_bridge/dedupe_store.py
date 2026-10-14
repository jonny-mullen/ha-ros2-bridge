# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Persisted command-id to ack-bytes store behind the dedupe step of command handling.

The contract promises one distinct ack per command id: a redelivered command
(a QoS 1 retransmission within one process, or a publisher-side resend across
a node restart) republishes the stored ack bytes instead of being processed
again, and the dedupe lookup runs before the freshness rule so that a
redelivery can never turn an ``accepted`` ack into ``rejected_stale``.

:class:`DedupeStore` keeps an ordered mapping ``id -> (received_at_ms,
ack_bytes)`` bounded by ``command.dedupe_size`` entries and the
``command.dedupe_window_s`` age window. Every ack, accepted or rejected, is
written to memory; only accepted acks are persisted to ``data_dir/dedupe.json``,
which is rewritten atomically (temporary file plus :func:`os.replace`) at most
once per ``min_write_interval_s`` (1 s). When a write is deferred by that rule
it happens on the injected :class:`~ros2_ha_bridge.clock.Scheduler` once the
interval has passed, or, without a scheduler, at the next accepted ack or at
:meth:`DedupeStore.flush`. At construction the file is loaded and only ids
younger than ``keep_on_load_ms`` (``command.max_age_cap_ms`` plus
``command.clock_skew_tolerance_ms``) are kept, which is exactly the window in
which a publisher-side QoS 1 resend could still pass the freshness check. A
data directory that cannot be created or written logs one warning and the
store runs memory-only for the rest of the process.

Everything here runs on the bridge's main thread; the clock and the scheduler
are injected so tests drive time themselves. This module imports neither rclpy
nor paho and does no I/O beyond the one JSON file.
"""

from __future__ import annotations

import json
import logging
import os
from collections import OrderedDict
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any

from ros2_ha_bridge.clock import Clock, Scheduler, TimerHandle
from ros2_ha_bridge.protocol import (
    DEFAULT_CLOCK_SKEW_TOLERANCE_MS,
    DEFAULT_MAX_AGE_CAP_MS,
    is_valid_task_id,
)

__all__ = [
    'DEFAULT_DEDUPE_SIZE',
    'DEFAULT_DEDUPE_WINDOW_S',
    'DEFAULT_KEEP_ON_LOAD_MS',
    'DEFAULT_MIN_WRITE_INTERVAL_S',
    'FILE_NAME',
    'FILE_VERSION',
    'DedupeEntry',
    'DedupeStore',
]

LOGGER = logging.getLogger(__name__)

#: File name under ``data_dir``.
FILE_NAME = 'dedupe.json'
#: Format version written into the file; a different version is ignored on load.
FILE_VERSION = 1
#: ``command.dedupe_size`` default: how many ids the memory store keeps at most.
DEFAULT_DEDUPE_SIZE = 1024
#: ``command.dedupe_window_s`` default: how long an id is remembered (at least ``reply.ack_expiry_s``).
DEFAULT_DEDUPE_WINDOW_S = 300.0
#: How old a persisted id may be at start-up and still be kept (cap plus skew tolerance).
DEFAULT_KEEP_ON_LOAD_MS = DEFAULT_MAX_AGE_CAP_MS + DEFAULT_CLOCK_SKEW_TOLERANCE_MS
#: Minimum spacing between two rewrites of the file.
DEFAULT_MIN_WRITE_INTERVAL_S = 1.0


@dataclass(frozen=True)
class DedupeEntry:
    """What the store remembers about one command id."""

    #: Node clock (epoch milliseconds) when the command was received.
    received_at_ms: int
    #: The exact bytes of the ack that was published, republished unchanged on a redelivery.
    ack_bytes: bytes
    #: True when the stored ack is ``accepted``; only these entries are persisted.
    accepted: bool
    #: Whether the ack was published retained. A rejection published non-retained because
    #: ``reply.retained_rejections_per_minute`` was exhausted stays non-retained when it is
    #: republished for a redelivery, so a replay cannot defeat the budget. Accepted acks are
    #: always retained, which is why the file needs no column for it.
    retain: bool = True


def _is_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _parse_file_entry(item: object) -> tuple[str, int, bytes] | None:
    """One entry of the file as ``(id, received_at_ms, ack_bytes)``, or ``None`` when unusable."""
    if not isinstance(item, dict):
        return None
    command_id = item.get('id')
    received_at_ms = item.get('received_at_ms')
    ack_text = item.get('ack')
    if not is_valid_task_id(command_id) or not _is_int(received_at_ms):
        return None
    if not isinstance(ack_text, str) or not ack_text:
        return None
    try:
        ack = json.loads(ack_text)
    except ValueError:
        return None
    if not isinstance(ack, dict) or ack.get('id') != command_id:
        return None
    assert isinstance(command_id, str) and isinstance(received_at_ms, int)
    return command_id, received_at_ms, ack_text.encode('utf-8')


class DedupeStore:
    """Ordered ``id -> DedupeEntry`` store with a persisted copy of the accepted entries.

    ``data_dir`` ``None`` means memory-only on purpose (tests, or an operator who
    wants no file). Construction loads the file so that a later write can never
    clobber entries that were not read first.
    """

    def __init__(
        self,
        *,
        clock: Clock,
        data_dir: str | os.PathLike[str] | None = None,
        max_size: int = DEFAULT_DEDUPE_SIZE,
        window_s: float = DEFAULT_DEDUPE_WINDOW_S,
        keep_on_load_ms: int = DEFAULT_KEEP_ON_LOAD_MS,
        min_write_interval_s: float = DEFAULT_MIN_WRITE_INTERVAL_S,
        scheduler: Scheduler | None = None,
        logger: logging.Logger | None = None,
    ) -> None:
        if max_size < 1:
            raise ValueError('max_size must be at least 1')
        if window_s <= 0:
            raise ValueError('window_s must be greater than 0')
        if keep_on_load_ms < 0:
            raise ValueError('keep_on_load_ms must not be negative')
        if min_write_interval_s < 0:
            raise ValueError('min_write_interval_s must not be negative')
        self.clock = clock
        self.scheduler = scheduler
        self.logger = logger if logger is not None else LOGGER
        self.max_size = max_size
        self.window_s = window_s
        self.keep_on_load_ms = keep_on_load_ms
        self.min_write_interval_s = min_write_interval_s
        self.path: Path | None = None if data_dir is None else Path(data_dir) / FILE_NAME
        #: How many entries the file contributed at construction.
        self.loaded = 0
        #: How many times the file has been rewritten by this instance.
        self.writes = 0
        self._entries: OrderedDict[str, DedupeEntry] = OrderedDict()
        self._persistent = self.path is not None
        self._dirty = False
        self._last_write_monotonic: float | None = None
        self._timer: TimerHandle | None = None
        if self.path is not None:
            self.loaded = self._load()

    # ----- observation ---------------------------------------------------------------

    @property
    def persistent(self) -> bool:
        """True while accepted entries are being written to the file."""
        return self._persistent

    @property
    def pending_write(self) -> bool:
        """True when an accepted entry is waiting for the once-per-second rewrite."""
        return self._dirty and self._persistent

    @property
    def entries(self) -> Mapping[str, DedupeEntry]:
        """Read-only view of the memory store, oldest first (no pruning is applied)."""
        return MappingProxyType(self._entries)

    def __len__(self) -> int:
        return len(self._entries)

    def __contains__(self, command_id: object) -> bool:
        return command_id in self._entries

    # ----- the dedupe step ------------------------------------------------------------

    def lookup(self, command_id: str) -> DedupeEntry | None:
        """The stored entry for ``command_id``, or ``None`` when unknown or past the window."""
        self._prune()
        return self._entries.get(command_id)

    def remember(
        self,
        command_id: str,
        received_at_ms: int,
        ack_bytes: bytes,
        *,
        accepted: bool,
        retain: bool = True,
    ) -> None:
        """Record the ack published for ``command_id``.

        Every ack goes into memory; an accepted one also marks the file dirty,
        which writes it now or, within ``min_write_interval_s`` of the last
        write, defers it. ``retain`` records how the ack was published so a
        redelivery republishes it the same way. Remembering an id twice
        replaces the earlier entry; the authoriser only does that to replace an
        entry whose bytes turned out to be unreadable, because the dedupe lookup
        runs before anything that could produce a second ack.
        """
        if command_id in self._entries:
            self.logger.debug('dedupe: replacing the stored ack for %s', command_id)
            del self._entries[command_id]
        self._entries[command_id] = DedupeEntry(
            received_at_ms, bytes(ack_bytes), accepted, retain or accepted
        )
        self._prune()
        if accepted and self._persistent:
            self._dirty = True
            self._write_now_or_later()

    def flush(self) -> bool:
        """Write a pending rewrite now (for shutdown); True when the file was written."""
        if not self.pending_write:
            return False
        return self._write()

    # ----- internals -----------------------------------------------------------------

    def _prune(self) -> None:
        cutoff = self.clock.now_ms() - round(self.window_s * 1000)
        expired = [cid for cid, entry in self._entries.items() if entry.received_at_ms < cutoff]
        for command_id in expired:
            del self._entries[command_id]
        while len(self._entries) > self.max_size:
            self._entries.popitem(last=False)

    def _load(self) -> int:
        assert self.path is not None
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
        except OSError as error:
            self._disable_persistence(f'cannot create {self.path.parent}: {error}')
            return 0
        try:
            if not self.path.is_file():
                return 0
            text = self.path.read_text(encoding='utf-8')
        except OSError as error:
            self.logger.warning(
                'dedupe: cannot read %s (%s); starting with an empty store', self.path, error
            )
            return 0
        try:
            document = json.loads(text)
        except ValueError as error:
            self.logger.warning('dedupe: ignoring corrupt %s (%s)', self.path, error)
            return 0
        if not isinstance(document, dict) or document.get('version') != FILE_VERSION:
            self.logger.warning(
                'dedupe: ignoring %s: not a version %d file', self.path, FILE_VERSION
            )
            return 0
        items = document.get('entries')
        if not isinstance(items, list):
            self.logger.warning('dedupe: ignoring %s: no entries list', self.path)
            return 0
        cutoff = self.clock.now_ms() - self.keep_on_load_ms
        skipped = 0
        for item in items:
            parsed = _parse_file_entry(item)
            if parsed is None:
                skipped += 1
                continue
            command_id, received_at_ms, ack_bytes = parsed
            if received_at_ms < cutoff:
                continue
            self._entries[command_id] = DedupeEntry(received_at_ms, ack_bytes, True)
        if skipped:
            self.logger.warning('dedupe: skipped %d unusable entries in %s', skipped, self.path)
        self._prune()
        return len(self._entries)

    def _write_now_or_later(self) -> None:
        now = self.clock.monotonic()
        last = self._last_write_monotonic
        if last is None or now - last >= self.min_write_interval_s:
            self._write()
            return
        if self.scheduler is not None and self._timer is None:
            delay = max(0.0, last + self.min_write_interval_s - now)
            self._timer = self.scheduler.call_later(delay, self._on_timer)

    def _on_timer(self) -> None:
        self._timer = None
        if self.pending_write:
            self._write()

    def _write(self) -> bool:
        assert self.path is not None
        self._cancel_timer()
        entries: list[dict[str, Any]] = []
        for command_id, entry in self._entries.items():
            if not entry.accepted:
                continue
            try:
                ack_text = entry.ack_bytes.decode('utf-8')
            except UnicodeDecodeError:
                self.logger.warning('dedupe: ack for %s is not UTF-8; not persisted', command_id)
                continue
            entries.append(
                {'id': command_id, 'received_at_ms': entry.received_at_ms, 'ack': ack_text}
            )
        document = {
            'version': FILE_VERSION,
            'written_at_ms': self.clock.now_ms(),
            'entries': entries,
        }
        temporary = self.path.with_name(self.path.name + '.tmp')
        try:
            with temporary.open('w', encoding='utf-8') as handle:
                json.dump(document, handle, separators=(',', ':'))
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, self.path)
        except OSError as error:
            self._disable_persistence(f'cannot write {self.path}: {error}')
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                self.logger.debug('dedupe: could not remove %s', temporary)
            return False
        self._dirty = False
        self._last_write_monotonic = self.clock.monotonic()
        self.writes += 1
        return True

    def _disable_persistence(self, reason: str) -> None:
        if not self._persistent:
            return
        self._persistent = False
        self._dirty = False
        self._cancel_timer()
        self.logger.warning('dedupe: running memory-only, %s', reason)

    def _cancel_timer(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
