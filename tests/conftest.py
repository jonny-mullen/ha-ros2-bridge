# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Shared pytest fixtures for the Home Assistant integration tests.

The pytest-homeassistant-custom-component plugin supplies ``hass``,
``hass_storage``, ``mqtt_mock_entry``, ``freezer``, the registries and the rest
of Home Assistant's own harness. This file adds: custom integrations enabled
for every test, a fixed millisecond clock, a config entry factory with the
integration's options, the golden loaders over ``contract/`` (parametrised by
``pytest_generate_tests`` exactly as the ROS suite does), a registry harness
that records events and persistence requests, the ROS protocol module imported
by path for the contract cross-check, and ``complete_hub_start``, which drives
a started ``BridgeHub`` through readiness, connection and the replay settle
period. The harness's ``mqtt_client_mock`` is overridden here so that every
consumer gets a paho client whose ``disconnect`` closes its socket.

The harness needs Linux: ``homeassistant.runner`` imports ``fcntl`` and
``resource`` at plugin load, so the suite runs in the ``ha-tests`` container or
WSL2 (see CONTRIBUTING.md), not on native Windows.
"""

from __future__ import annotations

import importlib
import sys
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, field
from datetime import timedelta
from types import ModuleType
from typing import Any
from unittest.mock import Mock

import pytest
from freezegun.api import FrozenDateTimeFactory
from homeassistant.core import HomeAssistant
from homeassistant.util import dt as dt_util
from pytest_homeassistant_custom_component.common import (
    MockConfigEntry,
    async_fire_time_changed,
)

from custom_components.ros2_bridge.commands import CommandRegistry, RegistrySettings
from custom_components.ros2_bridge.const import (
    CONF_INSTANCE_ID,
    DEFAULT_OPTIONS,
    DOMAIN,
    READY_TIMEOUT_S,
    REPLAY_SETTLE_S,
)
from custom_components.ros2_bridge.hub import BridgeHub

from .common import (
    INSTANCE_ID,
    KINDS,
    ROS_PACKAGE_DIR,
    VECTOR_SECRET,
    FakeClock,
    load_examples,
    load_vectors,
)


@pytest.fixture(autouse=True)
def auto_enable_custom_integrations(enable_custom_integrations: None) -> None:
    """Every test may load ``custom_components/ros2_bridge``."""


@pytest.fixture(autouse=True)
def auto_mock_hass_config(mock_hass_config: None) -> None:
    """``configuration.yaml`` reads are mocked (the MQTT entry setup reads it)."""


@pytest.fixture
def mqtt_client_mock(mqtt_client_mock: Any) -> Any:
    """The harness's paho client mock, closing its socket on disconnect as real paho does.

    The harness gives ``connect()`` a side effect that opens the socket (which
    starts the MQTT client's misc timer) but leaves ``disconnect()`` bare, so
    the timer, cancelled only from ``on_socket_close``, would linger after the
    MQTT entry is unloaded at teardown and fail ``verify_cleanup``. Overriding
    the fixture here covers every consumer: ``mqtt_mock_entry``, ``mqtt_setup``
    and the hub rigs.
    """

    def _disconnect(*_args: Any, **_kwargs: Any) -> int:
        mqtt_client_mock.on_socket_close(
            mqtt_client_mock, None, Mock(fileno=Mock(return_value=-1))
        )
        return 0

    mqtt_client_mock.disconnect.side_effect = _disconnect
    return mqtt_client_mock


# --------------------------------------------------------------------------- parametrisation


def pytest_generate_tests(metafunc: pytest.Metafunc) -> None:
    """``hmac_vector``, ``golden_example``, ``valid_<kind>_example``, ``invalid_<kind>_example``."""
    for name in metafunc.fixturenames:
        if name == "hmac_vector":
            vectors = load_vectors()
            metafunc.parametrize(name, vectors, ids=[v["name"] for v in vectors])
        elif name == "golden_example":
            examples = [e for kind in KINDS for e in load_examples(kind, valid=True)]
            metafunc.parametrize(name, examples, ids=[f"{e.kind}/{e.name}" for e in examples])
        elif name.startswith("valid_") and name.endswith("_example"):
            kind = name[len("valid_") : -len("_example")]
            if kind in KINDS:
                examples = load_examples(kind, valid=True)
                metafunc.parametrize(name, examples, ids=[e.name for e in examples])
        elif name.startswith("invalid_") and name.endswith("_example"):
            kind = name[len("invalid_") : -len("_example")]
            if kind in KINDS:
                examples = load_examples(kind, valid=False)
                metafunc.parametrize(name, examples, ids=[e.name for e in examples])


# --------------------------------------------------------------------------- plain fixtures


@pytest.fixture
def clock() -> FakeClock:
    """A fixed clock at the golden examples' epoch; tests advance it explicitly."""
    return FakeClock()


@pytest.fixture
def secret() -> str:
    """The placeholder secret the signed golden commands were made with. Never deploy it."""
    return VECTOR_SECRET


@pytest.fixture(scope="session")
def ros_protocol() -> ModuleType:
    """``ros2_ha_bridge.protocol`` imported by path from the ROS 2 package (no rclpy needed)."""
    package_dir = str(ROS_PACKAGE_DIR)
    if package_dir not in sys.path:
        sys.path.insert(0, package_dir)
    return importlib.import_module("ros2_ha_bridge.protocol")


@dataclass
class RegistryHarness:
    """A ``CommandRegistry`` on a fake clock with its events and persistence requests recorded."""

    clock: FakeClock
    registry: CommandRegistry
    events: list[tuple[str, dict[str, Any]]] = field(default_factory=list)
    persists: list[bool] = field(default_factory=list)

    def events_named(self, name: str) -> list[dict[str, Any]]:
        return [data for event, data in self.events if event == name]

    def clear(self) -> None:
        self.events.clear()
        self.persists.clear()


@pytest.fixture
def harness(clock: FakeClock) -> RegistryHarness:
    """A registry with default settings, deadlines enabled, recording events and persists."""
    harness = RegistryHarness(clock=clock, registry=CommandRegistry(clock=clock))
    harness.registry.set_callbacks(
        on_event=lambda name, data: harness.events.append((name, data)),
        on_persist=harness.persists.append,
    )
    harness.registry.update_settings(RegistrySettings())
    harness.registry.resume_deadlines()
    return harness


# --------------------------------------------------------------------------- Home Assistant


@pytest.fixture
def make_config_entry(hass: HomeAssistant) -> Callable[..., MockConfigEntry]:
    """Create the integration's single config entry (added to hass) with options overrides."""

    def _make(
        *, options: dict[str, Any] | None = None, instance_id: str = INSTANCE_ID
    ) -> MockConfigEntry:
        entry = MockConfigEntry(
            domain=DOMAIN,
            title="ROS 2 Bridge",
            unique_id=DOMAIN,
            data={CONF_INSTANCE_ID: instance_id},
            options={**DEFAULT_OPTIONS, **(options or {})},
        )
        entry.add_to_hass(hass)
        return entry

    return _make


@pytest.fixture
async def mqtt_setup(mqtt_mock_entry: Callable[[], Awaitable[Any]]) -> Any:
    """The mocked MQTT integration set up and connected; returns the ``mqtt_mock``."""
    return await mqtt_mock_entry()


@pytest.fixture
def complete_hub_start(
    hass: HomeAssistant, freezer: FrozenDateTimeFactory
) -> Callable[[BridgeHub], Awaitable[None]]:
    """Drive a started hub to ``ready`` and ``mqtt_connected`` and past ``REPLAY_SETTLE_S``.

    The MQTT client performs queued subscriptions from a debounced background
    task, so the helper advances time a second at a time (waiting for background
    tasks) until ``async_on_subscribe_done`` has confirmed the ack and result
    filters, which the mock normally does within two rounds. If it has not after
    ``READY_TIMEOUT_S`` rounds the production bound would have fired, so
    readiness is forced the same way. It then connects if needed and advances
    past the settle period so the first reconciliation has run.
    """

    async def _complete(hub: BridgeHub) -> None:
        for _ in range(READY_TIMEOUT_S):
            if hub.ready:
                break
            freezer.tick(timedelta(seconds=1))
            async_fire_time_changed(hass, dt_util.utcnow())
            # Two passes: the SUBACK callback is queued with call_soon by the mock and the
            # subscription task is not tracked by hass, so one pass may leave it un-resumed.
            await hass.async_block_till_done(wait_background_tasks=True)
            await hass.async_block_till_done(wait_background_tasks=True)
        if not hub.ready:
            hub._set_ready()
        if not hub.mqtt_connected:
            hub._on_connection_status(True)
        freezer.tick(timedelta(seconds=REPLAY_SETTLE_S + 1))
        async_fire_time_changed(hass, dt_util.utcnow())
        await hass.async_block_till_done(wait_background_tasks=True)
        assert hub.reconciled, "the settle timer did not fire"

    return _complete


@pytest.fixture
def advance(
    hass: HomeAssistant, freezer: FrozenDateTimeFactory
) -> Callable[[float], Awaitable[None]]:
    """Advance frozen time by seconds and run every timer that fell due."""

    async def _advance(seconds: float) -> None:
        freezer.tick(timedelta(seconds=seconds))
        async_fire_time_changed(hass, dt_util.utcnow())
        await hass.async_block_till_done(wait_background_tasks=True)

    return _advance
