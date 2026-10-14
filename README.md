# ha-ros2-bridge

Home Assistant and ROS 2, side by side, joined by MQTT. Home Assistant owns the sensors, dashboards, automations and voice; ROS 2 owns the robots. This repository ships both halves of the bridge and the written contract between them.

**Status: pre-release, phase 0 (scaffold and contract). Nothing here moves an arm yet.**

## What it is

An HA custom integration plus a ROS 2 package, joined by a written MQTT contract. Not an add-on.

- `ros2/ros2_ha_bridge/` (ament_python, Jazzy, paho-mqtt 2.1.0): one process per robot. Announces the robot and its task catalogue to HA, authorises and runs commands through a `TaskExecutor` plugin, publishes state, availability (Last Will), heartbeat, acks and results.
- `custom_components/ros2_bridge/` (HACS, rides HA's own MQTT connection, no pip requirements): one device per announced robot, services `send_task`, `cancel_task` and `get_robots`, bus events for ack and result, two Assist intents, diagnostics and repairs.
- `contract/`: JSON schemas, golden and must-reject examples and HMAC test vectors that both halves are tested against. Schema 1 only ever gains optional fields.

Why this shape: every living HA-to-ROS 2 precedent is a purpose-built ROS node publishing MQTT discovery; ROS 2 cannot usefully run inside an HA add-on; an add-on cannot create HA entities or services by itself; stock MQTT discovery alone cannot give non-admin users or Assist a typed task action (`mqtt.publish` is admin-only since HA 2026.10); and HA's own MQTT connection means adopters enter broker details once, in HA.

## Priorities

1. **Fast.** Sensor events reach ROS 2 and HA in parallel over the broker, not serially through HA automations. Latency of the HA-to-ROS 2 hop is a first-class requirement with measured targets.
2. **Safe by absence.** HA is advisory; ROS 2 owns safety. Commands are never retained, carry an id, an issue time and a maximum age, and are authorised in the node itself. HA going away never affects the arms by default. Hardware e-stop stays hardwired and outside this project.
3. **Honest about outcomes.** Every command id ends in exactly one named outcome: accepted or rejected with a reason, then succeeded, cancelled or aborted with a structured code, re-adopted across HA restarts, and synthesised only when the robot truly never answers.
4. **Shareable.** Packaged for HACS and for `colcon` from day one, with nothing that assumes one household's entity names, topics or addresses.

## Features (planned for 0.1.0)

- One HA device per robot, discovered from what the robot announces; scales from one arm to a fleet.
- State sensor (`idle`, `busy`, `error`), last-result sensor, problem sensor, task-result event entity, one button per parameterless task plus cancel.
- `send_task` with `wait: ack`, `wait: result` or `wait: none`, typed responses, and bus events any automation can react to (speech, notification, lights).
- Assist intents so "reset the arm" works from a Voice PE, acknowledging at once and reporting the outcome afterwards.
- Request-response with ROS 2 action semantics over MQTT: goal, feedback, result, cancel, with a structured result code (for example `collision`, `no_trajectory`, `timeout`) and a human-readable reason.
- A fast sensor path (ROS 2 subscribes to Zigbee2MQTT directly) and a normalised sensor path (HA republishes any entity in one canonical schema).
- Executor plugin interface with a shipped fake executor for testing without hardware, and a MoveIt 2 executor example for Isaac Sim or real arms.
- Application-layer authorisation in the node (UUID ids, allowlist, freshness window, dedupe, retained-command drop) and an optional shared-secret HMAC.
- Apache-2.0, SPDX headers, CI on both halves, a contract test suite both must pass.

## Repository layout

```text
LICENSE  README.md  CHANGELOG.md  CONTRIBUTING.md  SECURITY.md
hacs.json  pyproject.toml  requirements_test.txt  .pre-commit-config.yaml
.github/workflows/          ha.yml, ros2.yml, contract.yml, release.yml
contract/                   README, schemas/, examples/{valid,invalid}/, hmac_vectors.json
custom_components/ros2_bridge/   the Home Assistant integration (HACS)
ros2/ros2_ha_bridge/        the ROS 2 package (ament_python, Jazzy)
tests/                      HA integration tests
scripts/                    validate_contract.py, sync_contract.py, check_spdx.py, check_versions.py
docs/                       architecture, mqtt-contract, decisions, research/, and the rest as phases land
```

Two folders from the design are not in a clone yet: `brand/` (the HACS icon) arrives with the 0.1.0 release in phase 8, and `tools/` (Docker rig, compose, probes, end-to-end checklist) arrives with phase 2.

## Running the contract validator

From the repository root, with the project virtual environment's interpreter (needs only `jsonschema` 4.18 or newer):

```text
python scripts/validate_contract.py
python scripts/sync_contract.py --check
```

The first checks that every schema compiles as JSON Schema draft 2020-12, that every example under `contract/examples/valid/` validates, that every example under `contract/examples/invalid/` is rejected, and that every HMAC vector recomputes; it exits non-zero with a per-file report on any mismatch. The second confirms the ROS 2 package's vendored copy of the schemas is byte-identical. Both halves' test suites import the same module, so the two implementations must agree on every fixture. `contract/README.md` describes the checks in full, and CI runs them as `contract.yml`.

## Documentation

- [docs/architecture.md](docs/architecture.md): the two halves, the threading model, the data flows, the executor plugin interface.
- [docs/mqtt-contract.md](docs/mqtt-contract.md): every topic and payload with examples, command handling order, reply routing, failure behaviour.
- [docs/decisions.md](docs/decisions.md): the dated decision log, including the interview record and the open questions.
- [docs/research/README.md](docs/research/README.md): the research and design workflow outputs this project was built from.
- [CONTRIBUTING.md](CONTRIBUTING.md) and [SECURITY.md](SECURITY.md).

Install guides, configuration reference, writing an executor, Assist sentences, failure modes, safety and troubleshooting pages arrive with the phases that implement them (see the delivery phases in `docs/research/2026-10-13-requirements-and-plan.md`).

## Requirements (target)

- Home Assistant Core 2026.10 or newer with the MQTT integration configured against an MQTT 5 broker (the Mosquitto add-on qualifies).
- ROS 2 Jazzy on Ubuntu 24.04 (Python 3.12) for the robot side; paho-mqtt 2.1.0 from pip.
- Both machines on NTP within 2 s of each other (commands carry an issue time and a maximum age).

## Licence

Apache-2.0. See [LICENSE](LICENSE). Every source file carries an SPDX header.
