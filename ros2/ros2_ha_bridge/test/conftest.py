# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Shared fixtures for the ros2_ha_bridge unit tests.

Fixtures: ``clock``, ``scheduler`` and ``broker`` (the fakes), ``secret`` (the
placeholder secret of the signed golden commands), loaders over
``contract/examples`` and ``contract/hmac_vectors.json``, and
``schema_validator`` over the schemas vendored in ``ros2_ha_bridge/schemas``,
and :func:`assert_imports_without`, the import-hygiene check each module's
test file runs in a fresh interpreter.

Parametrisation: a test that takes a fixture named ``hmac_vector``,
``golden_example``, ``valid_<kind>_example`` or ``invalid_<kind>_example`` is
parametrised over the matching contract files by :func:`pytest_generate_tests`,
with the file names as test ids. ``<kind>`` is one of :data:`KINDS`.

The contract folder is found by walking up from this file (or from the
``HA_ROS2_BRIDGE_CONTRACT_DIR`` environment variable); when it is absent, as in
an installed-package run without the repository, those tests collect as
skipped and the schema validator still works from the vendored copy.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from collections.abc import Callable, Iterable, Iterator
from dataclasses import dataclass
from importlib import resources
from pathlib import Path
from typing import Any

import pytest
from jsonschema import Draft202012Validator
from jsonschema.exceptions import best_match
from referencing import Registry
from referencing.jsonschema import DRAFT202012

import ros2_ha_bridge

from .fakes import FakeBroker, FakeClock, FakeScheduler

SCHEMA_ID_BASE = 'https://ha-ros2-bridge.invalid/contract/1/'
KINDS: tuple[str, ...] = (
    'announce',
    'availability',
    'state',
    'heartbeat_robot',
    'heartbeat_ha',
    'command',
    'ack',
    'feedback',
    'result',
    'sensor',
)
SCHEMA_NAMES: tuple[str, ...] = ('common', *KINDS)
#: The placeholder secret of ``contract/hmac_vectors.json`` and the signed golden commands.
VECTOR_SECRET = 'example-shared-secret-change-me'
CONTRACT_DIR_ENV = 'HA_ROS2_BRIDGE_CONTRACT_DIR'


def find_contract_dir() -> Path | None:
    """The repository's ``contract/`` folder, or ``None`` when running without it."""
    override = os.environ.get(CONTRACT_DIR_ENV)
    if override:
        candidate = Path(override)
        return candidate if (candidate / 'hmac_vectors.json').is_file() else None
    for parent in Path(__file__).resolve().parents:
        candidate = parent / 'contract'
        if (candidate / 'hmac_vectors.json').is_file():
            return candidate
    return None


CONTRACT_DIR = find_contract_dir()

#: Makes ``ros2_ha_bridge`` importable in a fresh interpreter: the package's parent, which is
#: the source checkout here and the colcon install's site-packages under ``colcon test``.
PACKAGE_PARENT = Path(ros2_ha_bridge.__file__).resolve().parent.parent


def assert_imports_without(
    module: str, forbidden: Iterable[str], *, expected: Iterable[str] = ()
) -> None:
    """Import ``module`` in a fresh interpreter and check what its import graph pulls in.

    Fails when a top-level package named in ``forbidden`` (``rclpy``, ``paho``)
    is in ``sys.modules`` after the import, or when one named in ``expected``
    is not. A fresh interpreter is used on purpose: the test process itself
    may already hold rclpy (under ``colcon test`` in the Jazzy container,
    where ``pytest.importorskip('rclpy')`` runs at collection) or paho (the
    link tests), so the process's own ``sys.modules`` says nothing about the
    module under test.
    """
    code = '\n'.join(
        (
            'import sys',
            f'import {module}',
            "loaded = {name.split('.')[0] for name in sys.modules}",
            f'forbidden = sorted(loaded & set({sorted(forbidden)!r}))',
            f'missing = sorted(set({sorted(expected)!r}) - loaded)',
            "assert not forbidden, 'imported: ' + ', '.join(forbidden)",
            "assert not missing, 'not imported: ' + ', '.join(missing)",
        )
    )
    env = dict(os.environ)
    env['PYTHONPATH'] = os.pathsep.join(
        path for path in (str(PACKAGE_PARENT), env.get('PYTHONPATH', '')) if path
    )
    completed = subprocess.run(
        [sys.executable, '-c', code],
        check=False,
        env=env,
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert completed.returncode == 0, (
        f'{module} in a fresh interpreter: '
        f'{completed.stderr.strip() or completed.stdout.strip() or completed.returncode}'
    )


@dataclass(frozen=True)
class GoldenExample:
    """One file from ``contract/examples``."""

    kind: str
    name: str
    path: Path
    payload: Any
    #: The file's bytes exactly as stored (pretty-printed JSON), the way a test may feed them.
    raw: bytes
    valid: bool


def load_examples(kind: str, valid: bool) -> list[GoldenExample]:
    """Every example of ``kind`` from the valid or invalid folder, sorted by name."""
    if CONTRACT_DIR is None:
        return []
    folder = CONTRACT_DIR / 'examples' / ('valid' if valid else 'invalid') / kind
    examples: list[GoldenExample] = []
    for path in sorted(folder.glob('*.json')):
        raw = path.read_bytes()
        examples.append(GoldenExample(kind, path.stem, path, json.loads(raw), raw, valid))
    return examples


def load_vectors() -> list[dict[str, Any]]:
    """The HMAC vectors, each with secret, topic, payload, canonical and sig."""
    if CONTRACT_DIR is None:
        return []
    with (CONTRACT_DIR / 'hmac_vectors.json').open(encoding='utf-8') as handle:
        document = json.load(handle)
    vectors: list[dict[str, Any]] = list(document['vectors'])
    return vectors


def pytest_generate_tests(metafunc: pytest.Metafunc) -> None:
    for name in metafunc.fixturenames:
        if name == 'hmac_vector':
            vectors = load_vectors()
            metafunc.parametrize(name, vectors, ids=[v['name'] for v in vectors])
        elif name == 'golden_example':
            examples = [e for kind in KINDS for e in load_examples(kind, valid=True)]
            metafunc.parametrize(name, examples, ids=[f'{e.kind}/{e.name}' for e in examples])
        elif name.startswith('valid_') and name.endswith('_example'):
            kind = name[len('valid_') : -len('_example')]
            if kind in KINDS:
                examples = load_examples(kind, valid=True)
                metafunc.parametrize(name, examples, ids=[e.name for e in examples])
        elif name.startswith('invalid_') and name.endswith('_example'):
            kind = name[len('invalid_') : -len('_example')]
            if kind in KINDS:
                examples = load_examples(kind, valid=False)
                metafunc.parametrize(name, examples, ids=[e.name for e in examples])


# --------------------------------------------------------------------------- fakes


@pytest.fixture
def clock() -> FakeClock:
    """A fake clock starting at the golden examples' epoch (2025-10-13T12:00:00Z)."""
    return FakeClock()


@pytest.fixture
def scheduler(clock: FakeClock) -> FakeScheduler:
    """A fake scheduler fired by ``clock.advance()``."""
    return FakeScheduler(clock)


@pytest.fixture
def broker(clock: FakeClock) -> Iterator[FakeBroker]:
    """An in-memory broker; clients are stopped at teardown, callback errors fail the test."""
    fake = FakeBroker(clock)
    yield fake
    for client in list(fake.clients):
        client.loop_stop()
    errors = fake.callback_errors
    assert not errors, f'callbacks raised inside fake MQTT clients: {errors!r}'


@pytest.fixture
def secret() -> str:
    """The placeholder secret the signed golden commands were made with. Never deploy it."""
    return VECTOR_SECRET


# --------------------------------------------------------------------------- golden loaders


@pytest.fixture(scope='session')
def contract_dir() -> Path:
    if CONTRACT_DIR is None:
        pytest.skip(f'contract folder not found (set {CONTRACT_DIR_ENV} to point at it)')
    return CONTRACT_DIR


@pytest.fixture(scope='session')
def hmac_vectors(contract_dir: Path) -> list[dict[str, Any]]:
    return load_vectors()


@pytest.fixture(scope='session')
def valid_examples(contract_dir: Path) -> Callable[[str], dict[str, GoldenExample]]:
    """``valid_examples(kind)``: file stem to :class:`GoldenExample`, valid folder."""

    def loader(kind: str) -> dict[str, GoldenExample]:
        return {e.name: e for e in load_examples(kind, valid=True)}

    return loader


@pytest.fixture(scope='session')
def invalid_examples(contract_dir: Path) -> Callable[[str], dict[str, GoldenExample]]:
    """``invalid_examples(kind)``: file stem to :class:`GoldenExample`, invalid folder."""

    def loader(kind: str) -> dict[str, GoldenExample]:
        return {e.name: e for e in load_examples(kind, valid=False)}

    return loader


# --------------------------------------------------------------------------- schema validator


class SchemaValidator:
    """Draft 2020-12 validation against the vendored schemas, with the relative ``$ref`` registry.

    The registry code mirrors ``scripts/validate_contract.py`` so that the ROS
    suite needs only the installed package (``colcon test`` runs without the
    repository root on its path).
    """

    def __init__(self) -> None:
        root = resources.files('ros2_ha_bridge').joinpath('schemas')
        self.documents: dict[str, dict[str, Any]] = {}
        registry: Registry[Any] = Registry()
        for name in SCHEMA_NAMES:
            text = root.joinpath(f'{name}.schema.json').read_text(encoding='utf-8')
            document = json.loads(text)
            uri = f'{SCHEMA_ID_BASE}{name}.schema.json'
            assert document.get('$id') == uri, (
                f'{name}.schema.json has $id {document.get("$id")!r}'
            )
            self.documents[name] = document
            registry = registry.with_resource(uri, DRAFT202012.create_resource(document))
        self.registry = registry
        self._validators: dict[str, Draft202012Validator] = {}

    def validator(self, kind: str) -> Draft202012Validator:
        if kind not in KINDS:
            raise ValueError(f'unknown kind {kind!r}; expected one of {KINDS}')
        if kind not in self._validators:
            self._validators[kind] = Draft202012Validator(
                self.documents[kind], registry=self.registry
            )
        return self._validators[kind]

    def errors(self, kind: str, payload: Any) -> list[str]:
        """Human-readable errors; empty means ``payload`` is a valid ``kind``."""
        found = sorted(
            self.validator(kind).iter_errors(payload),
            key=lambda error: [str(part) for part in error.absolute_path],
        )
        lines: list[str] = []
        for error in found:
            path = '/'.join(str(part) for part in error.absolute_path) or '<root>'
            if error.context:
                inner = best_match(error.context)
                inner_path = '/'.join(str(part) for part in inner.absolute_path) or '<root>'
                lines.append(
                    f'{path}: no {error.validator} branch matched; '
                    f'closest: {inner_path}: {inner.message}'
                )
            else:
                lines.append(f'{path}: {error.message}')
        return lines

    def is_valid(self, kind: str, payload: Any) -> bool:
        return not self.errors(kind, payload)

    def assert_valid(self, kind: str, payload: Any) -> None:
        errors = self.errors(kind, payload)
        assert not errors, f'{kind} payload rejected by the schema:\n  ' + '\n  '.join(errors)

    def assert_invalid(self, kind: str, payload: Any) -> None:
        assert self.errors(kind, payload), f'{kind} payload accepted by the schema: {payload!r}'


@pytest.fixture(scope='session')
def schema_validator() -> SchemaValidator:
    """Validate payloads against the vendored contract schemas."""
    return SchemaValidator()
