# Contributing

Thank you for looking at ha-ros2-bridge. This page is short on purpose: the rules below are the ones that cause review churn when missed.

## Before you start

- Read [docs/architecture.md](docs/architecture.md) and [docs/mqtt-contract.md](docs/mqtt-contract.md).
- Check [docs/decisions.md](docs/decisions.md). If your change reverses a recorded decision, say so in the pull request and add a dated entry; do not quietly contradict the log.
- The project is pre-release. Until 0.1.0 is tagged, the delivery phases in `docs/research/2026-10-13-requirements-and-plan.md` set the order of work.

## File headers

Every Python file starts with exactly these two lines:

```text
# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
```

`scripts/check_spdx.py` enforces this in pre-commit and in CI. YAML and JSON files carry no header. ament_copyright is deliberately not used because it cannot read SPDX headers (see the decision log).

## Writing style

- British English in all prose, documentation, docstrings, log messages and user-facing strings: colour, behaviour, licence, authorise, catalogue, normalise.
- Code identifiers may use American spelling where a library requires it (for example `color` in a third-party API). Our own identifiers follow the contract's spelling (`rejected_unauthorised`).
- No em-dashes (U+2014) anywhere, in code comments included. Use a comma, a colon, or a new sentence.
- No emojis anywhere.
- No hard-coded hosts, IP addresses, usernames or secrets. Example files use placeholders such as `broker.example` and environment variables such as `ROS2_HA_BRIDGE_MQTT_PASSWORD`.

## Code quality

- `ruff` and `ruff-format` for style and formatting. `custom_components/` uses the defaults; `ros2/` uses line length 99 and single quotes, matching ament conventions. Configuration lives in `pyproject.toml`.
- `mypy --strict` on both halves.
- The HA half validates with `probatio` (never `import voluptuous`) and uses only public `homeassistant.components.mqtt` helpers.
- The ROS 2 half keeps `protocol.py`, `authoriser.py`, `task_manager.py`, `dedupe_store.py`, `watchdog.py`, `sensor_mirror.py` and `executors/` free of rclpy and paho imports so they test anywhere.
- Both `protocol.py` copies must produce identical canonical JSON and HMAC signatures; `scripts/sync_contract.py --check` and the vendored schemas under `ros2/ros2_ha_bridge/ros2_ha_bridge/schemas/` must stay byte-identical to `contract/schemas/`.
- Tests: the HA suite needs 95 percent coverage overall and 100 percent on `config_flow.py`; every row of `docs/failure-modes.md` names a test id that must exist.

## Running the checks

From the repository root, with the project virtual environment's interpreter (on the development PC that is `.venv`):

```text
python -m ruff check .
python -m ruff format --check .
python -m mypy --strict ros2/ros2_ha_bridge/ros2_ha_bridge   # ROS 2 half, any Python 3.12+
python -m mypy --strict custom_components                     # HA half, needs homeassistant installed (Python 3.14)
python scripts/check_spdx.py
python scripts/check_versions.py
python scripts/validate_contract.py
python scripts/sync_contract.py --check
```

`check_versions.py` lets the code run ahead of `CHANGELOG.md` between releases: when the five version strings agree on a version newer than the last release heading and there is an entry under `## [Unreleased]`, it reports the heading as the last release and passes. `release.yml` runs it with `--expect` and `--released`, which insist on a `## [X.Y.Z]` heading for the tag, so the heading is added when the release is cut, not before.

The rclpy-free ROS 2 core tests and the contract tests run on any Python 3.12 or newer. The HA suite needs Python 3.14 and runs natively on Linux (CI, the `ha-tests` container or WSL2); on a Windows PC run it through `tools/windows_test_shim` (see its README). The full ROS 2 suite runs under `colcon test` in the `ros:jazzy-ros-base` container via `tools/compose.yaml`. `pre-commit run --all-files` runs the fast subset.

## Decision log rule

Every decision is dated and goes in [docs/decisions.md](docs/decisions.md): design choices, defaults, reversals, and questions that are waiting on an answer. A pull request that changes behaviour without a decision log entry is not complete. Entries are appended, never rewritten.

## Pull requests

- One change per pull request, with the tests that prove it.
- Update `CHANGELOG.md` under Unreleased.
- Keep the contract additive. A change to schema 1 that removes a field, changes a type or changes a meaning is a new schema and needs a decision log entry first.
- Commits and pull requests follow the same language rules as the documentation.

## Safety note for contributors with real hardware

Test against the fake executor first. Never leave a retained message on a `cmd` topic. Never label anything in HA or MQTT as an e-stop. See [SECURITY.md](SECURITY.md) for the authorisation model.
