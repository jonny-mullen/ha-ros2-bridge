# ha-ros2-bridge

Home Assistant and ROS 2, side by side, joined by MQTT. Home Assistant owns the sensors, dashboards, automations and voice; ROS 2 owns the robots. This repository ships both halves of the bridge and the written contract between them.

**Status: 0.1.0, released 2026-10-14. The ROS 2 package passes `colcon test` in the Jazzy container, the Home Assistant integration passes its suite and hassfest, and the live end-to-end checklist was run against a Home Assistant 2026.10.1 instance with two fake arms in Docker (`tools/e2e_checklist.md`). Nothing here has moved a real arm; the MoveIt executor is experimental until the Isaac site rehearsal.**

Towards 0.2.0 (unreleased, 2026-10-14): optional joint telemetry. The announce may describe a robot's joints, the node publishes their positions on a non-retained topic at up to 2 Hz, Home Assistant makes one sensor per joint, the fake executor simulates seven joints so a dashboard moves without hardware, and `docs/examples/dashboard.yaml` is a phone-first dashboard for the two fake arms (`docs/dashboard.md`). Not yet run end to end on a live instance.

## What it is

An HA custom integration plus a ROS 2 package, joined by a written MQTT contract. Not an add-on.

- `ros2/ros2_ha_bridge/` (ament_python, Jazzy, paho-mqtt 2.1.0): one process per robot. Announces the robot and its task catalogue to HA, authorises and runs commands through a `TaskExecutor` plugin, publishes state, availability (Last Will), heartbeat, acks and results.
- `custom_components/ros2_bridge/` (HACS, rides HA's own MQTT connection, no pip requirements): one device per announced robot, actions `send_task`, `cancel_task` and `get_robots`, bus events for ack and result, two Assist intents, diagnostics and repairs.
- `contract/`: JSON schemas, golden and must-reject examples and HMAC test vectors that both halves are tested against. Schema 1 only ever gains optional fields.

Why this shape: every living HA-to-ROS 2 precedent is a purpose-built ROS node publishing MQTT discovery; ROS 2 cannot usefully run inside an HA add-on; an add-on cannot create HA entities or services by itself; stock MQTT discovery alone cannot give non-admin users or Assist a typed task action (`mqtt.publish` is admin-only since HA 2026.10); and HA's own MQTT connection means adopters enter broker details once, in HA.

## What works today

- **The ROS 2 node** (`bridge_node`) with the shipped `fake` executor: announce, availability by Last Will, retained state, heartbeat with counters, the contract's command handling in order (retained drop, size, UUID id, persisted dedupe, schema, robot id, optional HMAC, freshness, allowlist and params, state gating, executor), per-task retained acks and results with MQTT 5 expiry, pause and resume, task timeout with cancel grace and forced finish, the three watchdog modes, the sensor mirror onto `~/mqtt_in`, session takeover handling, TLS options, launch files for one robot and a fleet. The rclpy-free core and the contract tests run on native Windows Python; the rclpy and paho layers have their own tests for `colcon test` in the Jazzy container.
- **A MoveIt 2 reference executor** (`ros2_ha_bridge/executors/moveit.py`): drives `/move_action` through an adapter and maps `MoveItErrorCodes` onto the result vocabulary. Unit-tested with a fake adapter only; it has not been run against a real `move_group` and is marked experimental.
- **The Home Assistant integration** (`ros2_bridge`): config flow with MQTT discovery and prefix validation, options flow with reload, a push hub on eight wildcard subscriptions (the eighth, `{prefix}/+/joints`, since 0.2.0), a `CommandRegistry` backed by a Store so every command id ends in exactly one outcome across restarts and reloads, state, last-result and problem entities, a task-result event entity, one button per parameterless task plus cancel, the three actions with `wait: ack`, `result` or `none`, three bus events, two Assist intents, the normalised sensor republish, five repairs and redacted diagnostics. Test suite at 96 percent coverage (run through `tools/windows_test_shim` on this PC; natively on Ubuntu in CI).
- **The contract** (`contract/`) with its validator, the vendored-copy check and the HMAC vectors; both `protocol.py` copies produce identical canonical JSON and signatures.
- **The rig** (`tools/`): a `ros:jazzy-ros-base` image, `compose.yaml` with two fake arms, a test runner, an optional local broker, an MQTT client shell, `clock_check.py` (round trip and clock offset against HA), `latency_probe.py` (command-to-ack latency and heartbeat jitter; argument parser and dry run verified) and the chaos check list.
- **Documentation**: every page listed under "Documentation" below, including the failure-modes table whose rows name real test ids, checked by `scripts/check_failure_modes.py`.

## Not yet verified

- **The MoveIt executor against real MoveIt and Isaac Sim.** Unit-tested with a fake adapter only; the simulator rehearsal at the Isaac site is pending (`docs/simulator_checklist.md`; interview decisions Q12, Q13, Q20).
- **Signed commands end to end on the live broker** (the HMAC second pass of checklist step 6), a topic-prefix change on a live instance, and the node-side `mirror.topics` subscription outside the unit tests.
- **A HACS install from a tagged release and CI on GitHub**, because the public repository does not exist yet (decision log, open question (b)); and the Mosquitto add-on ACL finding reproduced on a live add-on (`docs/security.md`).
- **A real Voice PE.** The voice check used Home Assistant's conversation service as a typed stand-in.
- **Joint telemetry on a live broker and the example dashboard on a live Home Assistant.** Added 2026-10-14 towards 0.2.0; not yet part of the end-to-end checklist.

## Priorities

1. **Fast.** Sensor events reach ROS 2 and HA in parallel over the broker, not serially through HA automations. Latency of the HA-to-ROS 2 hop is a first-class requirement with measured targets.
2. **Safe by absence.** HA is advisory; ROS 2 owns safety. Commands are never retained, carry an id, an issue time and a maximum age, and are authorised in the node itself. HA going away never affects the arms by default. Hardware e-stop stays hardwired and outside this project.
3. **Honest about outcomes.** Every command id ends in exactly one named outcome: accepted or rejected with a reason, then succeeded, cancelled or aborted with a structured code, re-adopted across HA restarts, and synthesised only when the robot truly never answers.
4. **Shareable.** Packaged for HACS and for `colcon` from day one, with nothing that assumes one household's entity names, topics or addresses.

## Features (the 0.1.0 scope, plus the 0.2.0 joint telemetry)

- One HA device per robot, discovered from what the robot announces; scales from one arm to a fleet.
- State sensor (`idle`, `busy`, `error`), last-result sensor, problem sensor, task-result event entity, one button per parameterless task plus cancel.
- `send_task` with `wait: ack`, `wait: result` or `wait: none`, typed responses, and bus events any automation can react to (speech, notification, lights).
- Assist intents so "reset the arm" works from a Voice PE, acknowledging at once and reporting the outcome afterwards.
- Request-response with ROS 2 action semantics over MQTT: goal, feedback, result, cancel, with a structured result code (for example `collision`, `no_trajectory`, `timeout`) and a human-readable reason.
- A fast sensor path (ROS 2 subscribes to Zigbee2MQTT directly) and a normalised sensor path (HA republishes any entity in one canonical schema).
- Executor plugin interface with a shipped fake executor for testing without hardware, and a MoveIt 2 reference executor for Isaac Sim or real arms.
- Optional joint telemetry (towards 0.2.0): an announce may carry a `joints` block (unit, names, limits, rate); the node publishes positions on `{prefix}/{robot_id}/joints` (QoS 0, never retained, 5 s expiry, on change only, at most `telemetry.joints_rate_hz` per second); Home Assistant creates one sensor per joint with the limits as attributes and a documented recorder exclude; executors report positions through `BridgeContext.report_joints`. A phone-first example dashboard with gauges per joint (`docs/dashboard.md`).
- Application-layer authorisation in the node (UUID ids, allowlist, freshness window, dedupe, retained-command drop) and an optional shared-secret HMAC.
- Apache-2.0, SPDX headers, CI on both halves, a contract test suite both must pass.

## Repository layout

```text
LICENSE  README.md  CHANGELOG.md  CONTRIBUTING.md  SECURITY.md
hacs.json  pyproject.toml  requirements_test.txt  .pre-commit-config.yaml  .editorconfig  .dockerignore
.github/workflows/          ha.yml, ros2.yml, contract.yml, release.yml
brand/                      empty until the HACS icon lands with the 0.1.0 release
contract/                   README, schemas/, examples/{valid,invalid}/, hmac_vectors.json
custom_components/ros2_bridge/
                            the Home Assistant integration (HACS): config_flow, hub, commands, store,
                            services, intent, sensors_out, sensor, binary_sensor, button, event,
                            diagnostics, strings.json, translations/en.json, icons.json, quality_scale.yaml
ros2/ros2_ha_bridge/        the ROS 2 package (ament_python, Jazzy):
                              ros2_ha_bridge/   bridge_node, mqtt_link, ros_adapters, parameters, protocol,
                                                authoriser, task_manager, dedupe_store, watchdog, sensor_mirror,
                                                executors/{base,fake,moveit}.py, schemas/ (vendored contract)
                              launch/ config/ test/ package.xml setup.py
tests/                      HA integration tests (pytest-homeassistant-custom-component), snapshots/
scripts/                    validate_contract.py, sync_contract.py, check_spdx.py, check_versions.py,
                            check_failure_modes.py
tools/                      Dockerfile.ros2, compose.yaml, entrypoint.sh, mosquitto/, clock_check.py,
                            latency_probe.py, e2e_checklist.md, windows_test_shim/, README.md
docs/                       the pages listed below, examples/ (automations, sentences, dashboard), research/
```

## Running the checks

From the repository root, with the project virtual environment's interpreter (`.venv` on the development PC; Python 3.12 or newer with `pytest`, `jsonschema`, `paho-mqtt`, `ruff` and `mypy`):

```text
python scripts/validate_contract.py
python scripts/sync_contract.py --check
python scripts/check_versions.py
python scripts/check_spdx.py
python scripts/check_failure_modes.py
python -m pytest ros2/ros2_ha_bridge/test -k "not node"
python -m ruff check . && python -m ruff format --check .
python -m mypy --strict ros2/ros2_ha_bridge/ros2_ha_bridge   # ROS 2 half, any Python 3.12+
python -m mypy --strict custom_components                     # HA half, needs homeassistant installed (Python 3.14)
```

The contract validator checks that every schema compiles as JSON Schema draft 2020-12, that every example under `contract/examples/valid/` validates, that every example under `contract/examples/invalid/` is rejected, and that every HMAC vector recomputes; `sync_contract.py --check` confirms the ROS 2 package's vendored schemas are byte-identical. The rclpy tests skip without ROS 2 and run under `colcon test` in the container (`tools/README.md`). The Home Assistant suite needs Python 3.14 and Linux; `CONTRIBUTING.md` has the exact commands, including the Windows shim.

## Documentation

Start here:

- [docs/architecture.md](docs/architecture.md): the two halves, the threading model, the data flows, the executor plugin interface, measured latency.
- [docs/mqtt-contract.md](docs/mqtt-contract.md): every topic and payload with examples, command handling order, reply routing, failure behaviour, versioning.
- [docs/decisions.md](docs/decisions.md): the dated decision log, including the interview record and the open questions.

Installing and configuring:

- [docs/install-ha.md](docs/install-ha.md): the Home Assistant side: install, add, options, entities, actions, events, repairs, voice, removal.
- [docs/install-ros2.md](docs/install-ros2.md): the ROS 2 side on Ubuntu 24.04 with Jazzy, as a service, and the Docker route.
- [docs/configuration.md](docs/configuration.md): every node parameter with its default.
- [docs/upgrading.md](docs/upgrading.md): upgrading either half, the schema and Store versioning rules, the Home Assistant churn watch-list, the ROS 2 distribution outlook.

Using it:

- [docs/sensors.md](docs/sensors.md): the fast and the normalised sensor paths, how a ROS 2 node reads them, and the joints path back into Home Assistant with its recorder exclude.
- [docs/quick-start.md](docs/quick-start.md): both halves installed in an evening, a first task, how communication works, and testing without the Isaac site.
- [docs/dashboard.md](docs/dashboard.md): the example Lovelace dashboard for the fake arms (`docs/examples/dashboard.yaml`): installing it, the views, where the gauge limits come from, adapting it to other robots.
- [docs/assist.md](docs/assist.md): voice control with Assist, the sentences, the replies and their limits.
- [docs/examples/](docs/examples/): example automations, a sentence-trigger automation, the custom sentences file and the dashboard.
- [docs/troubleshooting.md](docs/troubleshooting.md): symptom to cause, rejection codes, repairs, diagnostics, watching the bus, logs.

Safety, security and failure behaviour:

- [docs/safety.md](docs/safety.md): where safety sits and what the bridge is not.
- [docs/security.md](docs/security.md): the authorisation model in depth; [SECURITY.md](SECURITY.md) is the reporting policy and the short form.
- [docs/failure-modes.md](docs/failure-modes.md): every failure mode with its mechanism, what HA shows and the test that proves it.

Extending and testing:

- [contract/README.md](contract/README.md): the machine-checked contract.
- [tools/README.md](tools/README.md): the Docker rig, the probes and the chaos checks.
- [tools/e2e_checklist.md](tools/e2e_checklist.md): the manual end-to-end procedure on a live Home Assistant with the fake executor.
- [docs/writing-an-executor.md](docs/writing-an-executor.md): the executor plugin interface, with the MoveIt 2 reference executor walked through.
- [docs/simulator_checklist.md](docs/simulator_checklist.md): the Isaac Sim rehearsal checklist for the simulator site.
- [docs/research/README.md](docs/research/README.md): the research and design workflow outputs this project was built from.
- [CONTRIBUTING.md](CONTRIBUTING.md): headers, style, checks, the decision log rule.

## Requirements (target)

- Home Assistant Core 2026.10 or newer with the MQTT integration configured against an MQTT 5 broker (the Mosquitto add-on qualifies).
- ROS 2 Jazzy on Ubuntu 24.04 (Python 3.12) for the robot side; paho-mqtt 2.1.0 from pip.
- Both machines on NTP within 2 s of each other (commands carry an issue time and a maximum age).

## Licence

Apache-2.0. See [LICENSE](LICENSE). Every source file carries an SPDX header.
