# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The ``ros2_bridge.commands`` Store (version 1).

One document per config entry::

    {"outstanding": {"<task_id>": {...}},          # CommandRegistry.export()
     "seen_results": [["<task_id>", finished_at]],  # CommandRegistry.export()
     "published_sensor_topics": ["<topic>", ...]}   # owned by sensors_out

``CommandStore`` wraps ``homeassistant.helpers.storage.Store``: it loads and
normalises the document, holds ``published_sensor_topics`` for the sensor
mirror, pulls the registry's share through :meth:`bind`, writes at once for
``async_save`` (before a command is published and on terminal transitions)
and with a delay for housekeeping-only changes, and removes the file on entry
removal.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Any, TypedDict

from homeassistant.core import HomeAssistant, callback
from homeassistant.helpers.storage import Store

from .commands import CommandRegistry
from .const import STORAGE_KEY, STORAGE_VERSION, STORE_DELAY_SAVE_S

_LOGGER = logging.getLogger(__name__)


class StoredData(TypedDict):
    """The Store document."""

    outstanding: dict[str, dict[str, Any]]
    seen_results: list[list[Any]]
    published_sensor_topics: list[str]


def empty_data() -> StoredData:
    """A fresh, empty document."""
    return {"outstanding": {}, "seen_results": [], "published_sensor_topics": []}


def normalise(data: object) -> StoredData:
    """Coerce whatever was on disk into the document shape, dropping unreadable parts."""
    result = empty_data()
    if not isinstance(data, dict):
        return result
    outstanding = data.get("outstanding")
    if isinstance(outstanding, dict):
        result["outstanding"] = {
            str(task_id): dict(entry)
            for task_id, entry in outstanding.items()
            if isinstance(entry, dict)
        }
    seen = data.get("seen_results")
    if isinstance(seen, list):
        result["seen_results"] = [
            [str(item[0]), int(item[1])]
            for item in seen
            if isinstance(item, (list, tuple))
            and len(item) == 2
            and isinstance(item[0], str)
            and isinstance(item[1], int)
            and not isinstance(item[1], bool)
        ]
    topics = data.get("published_sensor_topics")
    if isinstance(topics, list):
        result["published_sensor_topics"] = [t for t in topics if isinstance(t, str)]
    return result


class CommandStore:
    """Persistence for the command registry and the sensor mirror's published topics."""

    def __init__(self, hass: HomeAssistant) -> None:
        self.hass = hass
        self._store: Store[dict[str, Any]] = Store(hass, STORAGE_VERSION, STORAGE_KEY)
        self._registry: CommandRegistry | None = None
        self._save_task: asyncio.Task[None] | None = None
        self._save_again = False
        #: Topics the sensor mirror has published retained payloads on; it edits this list.
        self.published_sensor_topics: list[str] = []
        self.loaded = False

    async def async_load(self) -> StoredData:
        """Load and normalise the document; ``published_sensor_topics`` is kept on the store."""
        raw = await self._store.async_load()
        data = normalise(raw)
        self.published_sensor_topics = list(data["published_sensor_topics"])
        self.loaded = True
        return data

    def bind(self, registry: CommandRegistry) -> None:
        """Take the registry whose ``export()`` fills ``outstanding`` and ``seen_results``."""
        self._registry = registry

    def snapshot(self) -> dict[str, Any]:
        """The document as it would be written now."""
        data: dict[str, Any] = dict(empty_data())
        if self._registry is not None:
            data.update(self._registry.export())
        data["published_sensor_topics"] = list(self.published_sensor_topics)
        return data

    async def async_save(self) -> None:
        """Write the document now (before a publish and on terminal transitions)."""
        await self._store.async_save(self.snapshot())

    @callback
    def async_request_save(self, urgent: bool) -> None:
        """Schedule a write: at once for ``urgent``, otherwise debounced for housekeeping.

        Urgent requests made while a write is in flight are coalesced into one more
        write after it, so the document on disk always ends up at the latest state.
        """
        if not urgent:
            self._store.async_delay_save(self.snapshot, STORE_DELAY_SAVE_S)
            return
        if self._save_task is not None and not self._save_task.done():
            self._save_again = True
            return
        self._save_task = self.hass.async_create_task(
            self._async_save_loop(), f"{STORAGE_KEY} save", eager_start=False
        )

    async def _async_save_loop(self) -> None:
        try:
            while True:
                self._save_again = False
                await self.async_save()
                if not self._save_again:
                    return
        except Exception:  # a failed write must never kill the caller
            _LOGGER.exception("Writing %s failed", STORAGE_KEY)

    async def async_flush(self) -> None:
        """Wait for an in-flight urgent write, then write once more (entry unload)."""
        if self._save_task is not None and not self._save_task.done():
            await self._save_task
        await self.async_save()

    async def async_remove(self) -> None:
        """Delete the document (entry removal)."""
        if self._save_task is not None and not self._save_task.done():
            await self._save_task
        await self._store.async_remove()
        self.published_sensor_topics = []
