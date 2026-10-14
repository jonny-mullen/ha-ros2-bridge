# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Validate the ha-ros2-bridge MQTT contract.

Run from the repository root with no arguments::

    python scripts/validate_contract.py

It checks, in order, that every schema under ``contract/schemas`` compiles as
JSON Schema draft 2020-12 with its relative ``$ref`` values resolving, that
every file under ``contract/examples/valid/<kind>/`` validates against the
schema of its kind and fits its size limit, that every file under
``contract/examples/invalid/<kind>/`` is rejected by the schema or by the size
limit, and that every entry in ``contract/hmac_vectors.json`` recomputes to
the stored signature. Any mismatch exits non-zero with a per-file report.

The module also exposes :func:`validate`, :func:`check_size`,
:func:`canonical_json` and :func:`compute_sig` for reuse by the test suites of
both halves.
"""

from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import sys
from collections.abc import Iterable
from pathlib import Path
from typing import Any

# jsonschema ships no inline types; the stubs live in the separate
# types-jsonschema package, which the contract venv does not carry.
from jsonschema import Draft202012Validator  # type: ignore[import-untyped]
from jsonschema.exceptions import (  # type: ignore[import-untyped]
    SchemaError,
    ValidationError,
    best_match,
)
from referencing import Registry
from referencing.jsonschema import DRAFT202012

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONTRACT_DIR = REPO_ROOT / "contract"
SCHEMA_ID_BASE = "https://ha-ros2-bridge.invalid/contract/1/"
DRAFT_URI = "https://json-schema.org/draft/2020-12/schema"

#: Payload kinds, one schema file each. ``common`` holds shared definitions only.
KINDS: tuple[str, ...] = (
    "announce",
    "availability",
    "state",
    "heartbeat_robot",
    "heartbeat_ha",
    "command",
    "ack",
    "feedback",
    "result",
    "sensor",
    "joints",
)
SCHEMA_FILES: tuple[str, ...] = ("common", *KINDS)

MIN_VALID_EXAMPLES = 2
MIN_INVALID_EXAMPLES = 1

#: Byte limits from the contract (16 KiB per message, 64 KiB for announce),
#: applied here to the compact UTF-8 encoding of a payload, the smallest form
#: it can take on the wire. A receiver measures the bytes it was actually
#: given; these constants let the fixtures and both test suites agree.
MAX_PAYLOAD_BYTES = 16 * 1024
MAX_ANNOUNCE_BYTES = 64 * 1024


class ContractError(Exception):
    """Raised when the contract folder itself is malformed."""


def canonical_json(payload: dict[str, Any]) -> bytes:
    """Return the canonical UTF-8 bytes used for signing.

    ``sig`` is removed first so a signed payload canonicalises to the same bytes
    as the unsigned one.
    """
    unsigned = {key: value for key, value in payload.items() if key != "sig"}
    return json.dumps(unsigned, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode(
        "utf-8"
    )


def compute_sig(secret: str, topic: str, payload: dict[str, Any]) -> str:
    """Return ``hex(HMAC-SHA256(secret, topic + "\\n" + canonical))``."""
    message = topic.encode("utf-8") + b"\n" + canonical_json(payload)
    return hmac.new(secret.encode("utf-8"), message, hashlib.sha256).hexdigest()


def size_limit(kind: str) -> int:
    """Return the byte limit for a payload kind."""
    return MAX_ANNOUNCE_BYTES if kind == "announce" else MAX_PAYLOAD_BYTES


def encoded_size(payload: Any) -> int:
    """Return the length in bytes of the compact UTF-8 JSON encoding of ``payload``."""
    return len(json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8"))


def check_size(kind: str, payload: Any) -> str | None:
    """Return an error string when ``payload`` is over the size limit of ``kind``."""
    size = encoded_size(payload)
    limit = size_limit(kind)
    if size > limit:
        return f"<root>: {size} bytes in compact encoding, over the {kind} limit of {limit}"
    return None


def _discriminated(context: list[ValidationError]) -> list[ValidationError]:
    """Drop oneOf branches whose ``type`` discriminator did not match.

    The command schema tells task and cancel branches apart by ``type``. When
    a payload says ``type: task`` the cancel branch fails on that const first,
    and ``best_match`` would otherwise report the cancel branch's missing
    ``target_id`` rather than the real problem in the task branch.
    """
    by_branch: dict[int, list[ValidationError]] = {}
    for inner in context:
        by_branch.setdefault(int(inner.relative_schema_path[0]), []).append(inner)
    matching = [
        errors
        for errors in by_branch.values()
        if not any(
            inner.validator == "const" and list(inner.absolute_path)[-1:] == ["type"]
            for inner in errors
        )
        # A nullable field's "type: null" branch says nothing useful about a
        # non-null value; prefer the branch that describes the value.
        and not all(
            inner.validator == "type" and inner.validator_value == "null" for inner in errors
        )
    ]
    chosen = matching or list(by_branch.values())
    return [inner for errors in chosen for inner in errors]


def _format_error(error: ValidationError) -> str:
    path = "/".join(str(part) for part in error.absolute_path) or "<root>"
    if error.context:
        # oneOf and friends: the outer message repeats the whole instance, so
        # lead with the closest failing branch instead.
        inner = best_match(_discriminated(error.context))
        inner_path = "/".join(str(part) for part in inner.absolute_path) or "<root>"
        return (
            f"{path}: no {error.validator} branch matched; closest: {inner_path}: {inner.message}"
        )
    return f"{path}: {error.message}"


class ContractSchemas:
    """The compiled schema set for one contract folder."""

    def __init__(self, schemas_dir: Path) -> None:
        self.schemas_dir = schemas_dir
        self.documents: dict[str, dict[str, Any]] = {}
        registry: Registry[Any] = Registry()
        for name in SCHEMA_FILES:
            path = schemas_dir / f"{name}.schema.json"
            if not path.is_file():
                raise ContractError(f"missing schema file {path}")
            with path.open(encoding="utf-8") as handle:
                document = json.load(handle)
            expected_id = f"{SCHEMA_ID_BASE}{name}.schema.json"
            if document.get("$id") != expected_id:
                raise ContractError(
                    f"{path.name}: $id is {document.get('$id')!r}, expected {expected_id!r}"
                )
            if document.get("$schema") != DRAFT_URI:
                raise ContractError(
                    f"{path.name}: $schema is {document.get('$schema')!r}, expected {DRAFT_URI!r}"
                )
            self.documents[name] = document
            registry = registry.with_resource(expected_id, DRAFT202012.create_resource(document))
        self.registry = registry
        self._validators: dict[str, Draft202012Validator] = {}

    def check_schema(self, name: str) -> None:
        """Raise ``SchemaError`` when the named schema is not valid 2020-12."""
        Draft202012Validator.check_schema(self.documents[name])

    def check_refs(self, name: str) -> list[str]:
        """Resolve every ``$ref`` in the named schema; return the unresolved ones."""
        base_uri = f"{SCHEMA_ID_BASE}{name}.schema.json"
        resolver = self.registry.resolver(base_uri=base_uri)
        unresolved: list[str] = []

        def walk(node: Any) -> None:
            if isinstance(node, dict):
                ref = node.get("$ref")
                if isinstance(ref, str):
                    try:
                        resolver.lookup(ref)
                    except Exception as error:  # noqa: BLE001 - any resolver failure
                        unresolved.append(f"{ref}: {error}")
                for value in node.values():
                    walk(value)
            elif isinstance(node, list):
                for item in node:
                    walk(item)

        walk(self.documents[name])
        return unresolved

    def validator(self, kind: str) -> Draft202012Validator:
        if kind not in KINDS:
            raise ContractError(f"unknown kind {kind!r}; expected one of {KINDS}")
        if kind not in self._validators:
            self._validators[kind] = Draft202012Validator(
                self.documents[kind], registry=self.registry
            )
        return self._validators[kind]

    def validate(self, kind: str, payload: Any) -> list[str]:
        """Return a list of human-readable errors; empty means valid."""
        errors = sorted(
            self.validator(kind).iter_errors(payload),
            key=lambda error: [str(part) for part in error.absolute_path],
        )
        return [_format_error(error) for error in errors]


_DEFAULT_SCHEMAS: ContractSchemas | None = None


def validate(kind: str, payload: Any, schemas_dir: Path | None = None) -> list[str]:
    """Validate ``payload`` as ``kind`` and return the list of errors.

    ``kind`` is one of :data:`KINDS`. For ``availability`` the payload is the
    plain string ``online`` or ``offline``; for every other kind it is the
    decoded JSON object. An empty list means the payload is valid.
    """
    global _DEFAULT_SCHEMAS
    if schemas_dir is not None:
        return ContractSchemas(schemas_dir).validate(kind, payload)
    if _DEFAULT_SCHEMAS is None:
        _DEFAULT_SCHEMAS = ContractSchemas(DEFAULT_CONTRACT_DIR / "schemas")
    return _DEFAULT_SCHEMAS.validate(kind, payload)


def _load_json(path: Path) -> Any:
    with path.open(encoding="utf-8") as handle:
        return json.load(handle)


def _example_files(folder: Path) -> list[Path]:
    return sorted(path for path in folder.glob("*.json") if path.is_file())


class Report:
    """Collects per-check lines and the overall verdict."""

    def __init__(self, quiet: bool) -> None:
        self.quiet = quiet
        self.failures = 0
        self.lines: list[str] = []

    def ok(self, text: str) -> None:
        if not self.quiet:
            self.lines.append(f"  ok    {text}")

    def fail(self, text: str) -> None:
        self.failures += 1
        self.lines.append(f"  FAIL  {text}")

    def heading(self, text: str) -> None:
        self.lines.append(text)


def check_schemas(schemas: ContractSchemas, report: Report) -> None:
    report.heading(f"Schemas ({len(SCHEMA_FILES)} files)")
    for name in SCHEMA_FILES:
        try:
            schemas.check_schema(name)
        except SchemaError as error:
            report.fail(f"{name}.schema.json: {error.message}")
            continue
        unresolved = schemas.check_refs(name)
        if unresolved:
            report.fail(f"{name}.schema.json: unresolved $ref: {'; '.join(unresolved)}")
            continue
        report.ok(f"{name}.schema.json")


def _all_errors(schemas: ContractSchemas, kind: str, payload: Any) -> list[str]:
    """Schema errors followed by the size check; empty means the payload passes."""
    errors = schemas.validate(kind, payload)
    size_error = check_size(kind, payload)
    if size_error is not None:
        errors.append(size_error)
    return errors


def check_examples(schemas: ContractSchemas, examples_dir: Path, report: Report) -> None:
    valid_root = examples_dir / "valid"
    invalid_root = examples_dir / "invalid"
    for root in (valid_root, invalid_root):
        for folder in sorted(path for path in root.iterdir() if path.is_dir()):
            if folder.name not in KINDS:
                report.fail(f"{folder}: folder name is not a known kind")

    report.heading("Valid examples (must validate)")
    for kind in KINDS:
        files = _example_files(valid_root / kind)
        if len(files) < MIN_VALID_EXAMPLES:
            report.fail(
                f"valid/{kind}: {len(files)} example(s), need at least {MIN_VALID_EXAMPLES}"
            )
        for path in files:
            label = f"valid/{kind}/{path.name}"
            try:
                payload = _load_json(path)
            except json.JSONDecodeError as error:
                report.fail(f"{label}: not JSON ({error})")
                continue
            errors = _all_errors(schemas, kind, payload)
            if errors:
                report.fail(f"{label}: {len(errors)} error(s)")
                for text in errors:
                    report.lines.append(f"          {text}")
            else:
                report.ok(label)

    report.heading("Invalid examples (must be rejected)")
    for kind in KINDS:
        files = _example_files(invalid_root / kind)
        if len(files) < MIN_INVALID_EXAMPLES:
            report.fail(
                f"invalid/{kind}: {len(files)} example(s), need at least {MIN_INVALID_EXAMPLES}"
            )
        for path in files:
            label = f"invalid/{kind}/{path.name}"
            try:
                payload = _load_json(path)
            except json.JSONDecodeError as error:
                report.fail(f"{label}: not JSON ({error})")
                continue
            errors = _all_errors(schemas, kind, payload)
            if errors:
                report.ok(f"{label}  (rejected: {_shorten(errors[0])})")
            else:
                report.fail(f"{label}: accepted but must be rejected")


def _shorten(text: str, limit: int = 110) -> str:
    text = " ".join(text.split())
    return text if len(text) <= limit else text[: limit - 3] + "..."


def check_hmac_vectors(schemas: ContractSchemas, vectors_path: Path, report: Report) -> None:
    report.heading("HMAC vectors (must recompute)")
    if not vectors_path.is_file():
        report.fail(f"{vectors_path.name}: missing")
        return
    document = _load_json(vectors_path)
    vectors: Iterable[dict[str, Any]] = document.get("vectors", [])
    count = 0
    for vector in vectors:
        count += 1
        name = str(vector.get("name", f"vector {count}"))
        missing = [key for key in ("secret", "topic", "payload", "sig") if key not in vector]
        if missing:
            report.fail(f"{name}: missing {', '.join(missing)}")
            continue
        payload = vector["payload"]
        if "sig" in payload:
            report.fail(f"{name}: payload must be stored without sig")
            continue
        canonical = canonical_json(payload)
        stored_canonical = vector.get("canonical")
        if stored_canonical is not None and stored_canonical != canonical.decode("utf-8"):
            report.fail(f"{name}: stored canonical string differs from recomputed")
            continue
        actual = compute_sig(vector["secret"], vector["topic"], payload)
        if actual != vector["sig"]:
            report.fail(f"{name}: expected {vector['sig']} got {actual}")
            continue
        schema_errors = schemas.validate("command", payload)
        if schema_errors:
            report.fail(f"{name}: payload is not a valid command: {schema_errors[0]}")
            continue
        signed = dict(payload, sig=actual)
        if schemas.validate("command", signed):
            report.fail(f"{name}: signed payload is not a valid command")
            continue
        report.ok(f"{name}  sig={actual[:16]}...")
    if count < 4:
        report.fail(f"{vectors_path.name}: {count} vector(s), need at least 4")


def run(contract_dir: Path, quiet: bool) -> int:
    report = Report(quiet)
    try:
        schemas = ContractSchemas(contract_dir / "schemas")
    except (ContractError, json.JSONDecodeError) as error:
        print(f"FAIL  {error}")
        return 1
    check_schemas(schemas, report)
    check_examples(schemas, contract_dir / "examples", report)
    check_hmac_vectors(schemas, contract_dir / "hmac_vectors.json", report)
    for line in report.lines:
        print(line)
    if report.failures:
        print(f"RESULT: FAIL ({report.failures} problem(s))")
        return 1
    print("RESULT: PASS")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "--contract-dir",
        type=Path,
        default=DEFAULT_CONTRACT_DIR,
        help="folder holding schemas/, examples/ and hmac_vectors.json",
    )
    parser.add_argument("--quiet", action="store_true", help="print only failures and the verdict")
    args = parser.parse_args(argv)
    return run(args.contract_dir.resolve(), args.quiet)


if __name__ == "__main__":
    sys.exit(main())
