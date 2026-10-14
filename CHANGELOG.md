# Changelog

All notable changes to ha-ros2-bridge are recorded here. The format follows Keep a Changelog, and the project will follow Semantic Versioning from the first tag. Dates are ISO 8601. Each release section is also the body of the GitHub release (`release.yml`).

## [Unreleased]

Pre-release work towards 0.1.0, by delivery phase. Nothing in this tree has moved an arm; the fake executor is the only one that has run. The dated subsections below are working days, not releases.

### 2026-10-14

Phase 3 (Home Assistant integration) and the phase 4 to 7 hardening, documentation and simulator material.

#### Added

- Phase 3, `custom_components/ros2_bridge`: config flow (user step, MQTT discovery on `ros2_bridge/+/announce`, confirm step, prefix validation) and `OptionsFlowWithReload` with nine options; the `BridgeHub` push hub on seven wildcard subscriptions with segment filtering, readiness gating, HA heartbeat, housekeeping and reconciliation after the retained replay; `CommandRegistry` and the version 1 Store (`ros2_bridge.commands`) so every command id ends in exactly one outcome across restarts and options reloads; the sensor, binary sensor, button and event platforms with `async_entity_id_changed`; actions `send_task`, `cancel_task` and `get_robots` with typed responses and multi-target validation before the first publish; bus events `ros2_bridge_task_ack`, `ros2_bridge_task_result` and `ros2_bridge_task_late_result`; intents `Ros2BridgeSendTask` and `Ros2BridgeCancelTask` registered once; the normalised sensor republish with retained-topic clearing; five repairs; redacted diagnostics; `strings.json`, `translations/en.json`, `icons.json`, `quality_scale.yaml`.
- Phase 3 tests under `tests/` (96 percent coverage, `config_flow.py` at 100 percent) with snapshots, and `tools/windows_test_shim` so the Linux-only harness runs on the Windows development PC without changing test semantics.
- Phase 3 documentation: `docs/install-ha.md`, `docs/assist.md`, `docs/sensors.md`, `docs/troubleshooting.md`, `docs/examples/` (automations, a sentence-trigger automation, the custom sentences file).
- Phase 4, failure modes: `docs/failure-modes.md`, the failure-modes table where every row names the test id that proves it, with "no test yet" rows allowed but listed, and `scripts/check_failure_modes.py`, which fails when a named test does not exist.
- Phase 4, safety and security pages: `docs/safety.md` (where safety sits, what the bridge is not) and `docs/security.md` (the authorisation model in depth, alongside the `SECURITY.md` reporting policy).
- Phase 5, latency: `tools/latency_probe.py`, a paho-mqtt 2.1.0 command line that measures command-to-ack latency (against an unknown task name by default, so the executor is never consulted, or an announced task with `--task`), the publish-to-PUBACK round trip and heartbeat period jitter, with optional HMAC signing from the environment, a `--dry-run` that validates the arguments and prints the plan, and a JSON summary. Sensor-path latency is measured at the simulator site, not by this probe.
- Phase 7, executors: `ros2_ha_bridge/executors/moveit.py`, a MoveIt 2 reference executor driving `/move_action` through an adapter and mapping `MoveItErrorCodes` onto the result vocabulary, unit-tested with a fake adapter only and marked experimental; `docs/writing-an-executor.md` (the plugin interface, the self-cancelling timer pattern, the result-code mapping); and `docs/simulator_checklist.md`, the simulator checklist for the Isaac site.
- Phase 7, release preparation: `docs/upgrading.md` (upgrading either half, schema and Store versioning, the Home Assistant churn watch-list, the ROS 2 distribution outlook); `tools/e2e_checklist.md`, the manual end-to-end procedure on a live Home Assistant with the fake executor, steps 0 to 13 with a results table.

#### Changed

- `README.md`: status, the real repository layout, what works today and what is not yet verified, links to every documentation page.
- `custom_components/ros2_bridge/quality_scale.yaml`: the `docs-*` rules whose pages now exist are marked done with the page named; `integration-owner` stays todo until the public repository exists.
- `docs/decisions.md`: phase 3 and phase 4 to 7 decisions appended, and open question (h) on releasing 0.1.0 after fake-only verification.
- Repair pass, same day: the MoveIt executor's `stop` task clears the robot error only when move_group confirms the goal ended (with `moveit.hold_on_cancel` true; at once with it false), a goal move_group rejects after a cancel request ends `cancelled` rather than `aborted/unavailable`, and the module and `docs/writing-an-executor.md` carry the experimental marker; `unavailable` joins the recommended result-code vocabulary in `docs/mqtt-contract.md` and the schema description; `docs/configuration.md` gains the `moveit.*` table and `docs/install-ros2.md` the `ROS2_HA_BRIDGE_WITH_MOVEIT` note; `docs/safety.md` qualifies the no-replay promise with the recorded one-second dedupe window; the two Markdown Python fences that failed `ruff format --check` are reformatted.

#### Pending verification

- The container `colcon build` and `colcon test` run and the live Home Assistant end-to-end checklist are blocked on the development PC until WSL is reset; both are recorded as pending, not done.
- The MoveIt reference executor has not been run against a real `move_group` or Isaac Sim; the simulator rehearsal is pending.
- No latency numbers have been measured yet; `docs/architecture.md` still shows the provisional targets.

### 2026-10-13

Phases 0 to 2: scaffold and contract, the rclpy-free ROS 2 node core with the fake executor, and the ROS 2 packaging and Docker rig.

#### Added

- Phase 0, repository scaffold: Apache-2.0 `LICENSE`, SPDX headers on every Python and shell file with `scripts/check_spdx.py` as the single header gate, `.editorconfig`, `.gitignore`, `.pre-commit-config.yaml`, CI workflows for the HA half (`ha.yml`), the ROS 2 half (`ros2.yml`), the contract (`contract.yml`) and releases (`release.yml`), `scripts/check_versions.py`, `README.md`, `CONTRIBUTING.md`, `SECURITY.md`.
- Phase 0, `contract/`: JSON schemas for every payload (schema 1), golden examples and must-reject examples including the malicious-id set, HMAC test vectors, `scripts/validate_contract.py` and `scripts/sync_contract.py`.
- Phase 0 documentation: `docs/decisions.md` (the 20 interview rows, the recorded assumptions, the design panel decisions, the open questions), `docs/mqtt-contract.md`, `docs/architecture.md`, `docs/research/` (the three workflow outputs and the requirements and plan, redacted for publication).
- Phase 1, `ros2/ros2_ha_bridge/ros2_ha_bridge`: the rclpy-free core (`protocol.py`, `authoriser.py` with the contract's steps 0 to 9 and the two-phase ack, `dedupe_store.py` persisted across restarts, `task_manager.py` with the single motion slot, timeout, cancel grace, forced finish, pause and resume, `watchdog.py` with the three modes, `sensor_mirror.py`, `executors/base.py` and `executors/fake.py`), the test doubles (`FakeBroker`, `FakeMqttClient`, `FakeClock`, `FakeScheduler`) and the tests that run on native Windows Python with no ROS installed, including the contract tests against the vendored schemas.
- Phase 2, the rclpy and paho layers: `mqtt_link.py` (MQTT 5, `clean_start`, Last Will, bounded inbound deque, publish guard, session takeover handling with exit after `mqtt.takeover_exit_after`), `ros_adapters.py` (steady clock, self-cancelling one-shot timers, guard-condition main-thread posting), `parameters.py` (the declaration table, environment fallbacks for the two secrets, validation), `bridge_node.py`, `launch/bridge.launch.py` and `launch/fleet.launch.py` with respawn limits, `config/bridge.example.yaml` and `config/fleet.example.yaml`, `package.xml`, `setup.py`, and `docs/install-ros2.md` and `docs/configuration.md`.
- Phase 2, the Docker rig in `tools/`: `Dockerfile.ros2` (`ros:jazzy-ros-base`, paho-mqtt 2.1.0 from pip, `colcon build`), `compose.yaml` (two fake arms, a test runner, a clock-check service, an optional anonymous local broker, an MQTT client shell, one bridge network and no host networking), `entrypoint.sh`, `.env.example`, `clock_check.py`, and the chaos checks in `tools/README.md`.

#### Not yet (as of that day)

- No Home Assistant integration code, no measured latency, no voice sentences, no simulator rehearsal; those followed on 2026-10-14 or are still pending (see above).
