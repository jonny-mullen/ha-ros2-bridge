# Upgrading

How to move either half of ha-ros2-bridge to a newer version, what stays compatible, and the Home Assistant and ROS 2 changes on the horizon that the code already allows for or will have to. Nothing here is release notes; those are in `CHANGELOG.md`.

## The compatibility promise

- **Contract schema 1 is additive.** Every payload carries `"schema": 1`, receivers ignore unknown fields, and schema 1 only ever gains optional fields or new values in an open vocabulary (result `code`, `source.kind`, heartbeat counters). The closed sets (ack codes, result `status`, task `kind`, `state`) do not change inside schema 1. So a newer node works with an older integration and the other way round; the older side merely does not see the newer optional fields. A change that removes a field, changes a type, or changes the meaning of an existing value is schema 2: a new `$id` base under `contract/schemas/`, published alongside schema 1 with a migration note, and a new major version of both halves (`docs/mqtt-contract.md`, Versioning).
- **Both halves carry the same version number per release.** `scripts/check_versions.py` keeps `manifest.json`, `const.py`, `package.xml`, `setup.py`, the ROS package's `__init__.py` and the latest `CHANGELOG.md` heading in agreement; the release workflow refuses a tag that does not match. Upgrade the two halves in either order; they are not required to match at any instant, only to speak schema 1.
- **The Home Assistant Store document is version 1.** The integration keeps its outstanding commands, the ids of results it has already seen and the sensor topics it has published in `.storage/ros2_bridge.commands` (`STORAGE_VERSION = 1`, key `ros2_bridge.commands`). The loader normalises whatever is on disk and drops unreadable parts, so a document written by an older build loads; a future change to the document's shape that cannot be read that way would raise the Store version and ship a migration with it, and would be listed here.
- **The node's dedupe file is a plain JSON cache.** `data_dir/dedupe.json` (default `~/.ros/ros2_ha_bridge/{robot.id}/`) holds recently answered command ids and their acks for `command.dedupe_window_s`. A newer node that cannot read an older file logs one warning and starts memory-only; nothing else depends on it.
- **Options survive.** The integration's config entry (the generated `instance_id`) and its options are not rewritten by an upgrade.

## Upgrading the Home Assistant integration

### Through HACS

1. HACS shows the new version under Updates once the release is tagged (releases are read from the GitHub repository; see open question (b) in `docs/decisions.md` for when that repository exists).
2. Read the release's section in `CHANGELOG.md` first; anything that needs a hand is listed there under "Changed" or "Upgrade notes".
3. Update in HACS and restart Home Assistant when it asks. Outstanding commands at the moment of the restart are reconciled afterwards from the Store and the retained replies: a task still running on the robot is re-adopted and its real result fires once as `ros2_bridge_task_result` (`docs/mqtt-contract.md`, Reply routing and reconciliation).
4. Check: Settings, Devices and services, ROS 2 Bridge loads without a repair issue; the diagnostics download shows `hub.ready`, `hub.mqtt_connected` and `hub.reconciled` true and `entry.version` as expected; press one task button on a fake robot if one is available.

To go back, HACS offers Redownload with a version picker; restart afterwards.

### By copying the folder

Replace `<config>/custom_components/ros2_bridge/` with the new folder in full (delete the old one first, so a file removed upstream does not linger; `translations/` included), then restart Home Assistant. The same checks apply. To go back, copy the previous folder in the same way.

### What an upgrade never requires

Re-adding the integration, re-entering the prefix or the secret, or touching `.storage/`. If a future release ever needs one of these, it is written in `CHANGELOG.md` under that version.

## Upgrading the ROS 2 package

1. Pull the new sources into the workspace (`~/bridge_ws/src/ros2_ha_bridge` in `docs/install-ros2.md`).
2. If `requirements.txt` or `package.xml` changed, re-run step 3 of the install guide (`rosdep install` and the pip install of paho-mqtt). paho-mqtt stays at 2.1.0 until this page says otherwise.
3. Rebuild and restart the node:

   ```text
   cd ~/bridge_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --event-handlers console_direct+
   source install/setup.bash
   ```

   then restart however the node is run: `systemctl restart <unit>` for the service in `docs/install-ros2.md`, Ctrl-C and `ros2 launch` again for an interactive run, or `docker compose build && docker compose up -d arm1 arm2` for the rig in `tools/`. The launch file respawns a node that exits (the launch arguments `respawn`, `respawn_delay` and `respawn_max_retries`: on, 2 s and 3 by default; `docs/configuration.md`, Launch arguments), which covers a crash during the swap but not a deliberate stop; a clean stop publishes `offline` and the restart publishes `online`, the announce and the state again.
4. Check: the node's log says `connected to the MQTT broker as ...`; the retained announce on `{prefix}/{robot_id}/announce` carries the new `bridge_version`; Home Assistant's device shows the new catalogue if it changed (buttons follow a re-announce: a renamed task is a new button and the old one disappears, which breaks dashboards that referenced it; task names are an adopter's public contract).
5. One robot at a time in a fleet, because each node is its own process and the catalogue is per robot.

Parameters files keep working across versions: a parameter that gains a new name is listed here with its old name. A parameter that no longer exists is reported by rclpy at start-up as unused, not as an error.

## Per-release notes

### 0.2.0 (unreleased)

Joint telemetry is on by default for every executor that announces joints: the fake executor always does, and the MoveIt executor does when `moveit.joint_names` or the first pose names the joints. A 0.1.0 site that upgrades the node with no configuration change therefore gains a `joints` block in its announce, a `{prefix}/{robot_id}/joints` topic at `telemetry.joints_rate_hz` (2.0) and, with the MoveIt executor, a `/joint_states` subscription; a Home Assistant that upgrades gains one sensor per announced joint, enabled and recorded by default. To keep 0.1.0 behaviour, set `telemetry.joints_rate_hz: 0.0` on the node (no topic, no announce block, so no sensors); to keep the sensors but not their history, add the recorder exclude from `docs/sensors.md` (`entity_globs: sensor.*_joint_*`). Nothing on the wire that existed in 0.1.0 changed, and no option or parameter was renamed; the options and parameters added on 2026-10-14 all default to the values 0.1.0 had as constants (`CHANGELOG.md`).

## Home Assistant churn watch-list

The integration targets Home Assistant Core 2026.10 or newer (`hacs.json`). These are the Core changes the design identified as touching an MQTT custom integration, with what the code does about each. Re-check this table at every monthly Core release; the pinned test harness in `requirements_test.txt` (`pytest-homeassistant-custom-component`, tracking Core 2026.10.1) is the trip-wire for a change that breaks the suite, and bumping it is the first step of each monthly check. A Dependabot configuration for that bump is on the release (phase 8) list and does not exist yet.

| Core release | Change | Where the integration stands | Action on upgrade |
|---|---|---|---|
| 2026.6 | Home Assistant's MQTT integration requires an MQTT 5 broker; its session is MQTT 5 by default. | The bridge relies on MQTT 5 for message expiry on commands. If the session is not MQTT 5 the integration publishes the command without an expiry, raises the `mqtt_v5_required` repair once, and the robots still refuse stale commands by `max_age_ms`. | None. Keep the MQTT integration's protocol at 5. |
| 2026.9 | Schema validation runs on probatio; voluptuous is no longer installed, only aliased in `sys.modules`, with no stated removal date for the alias; error messages are worded differently. | `config_flow.py`, `services.py` and `intent.py` import probatio directly and type schemas as `VolSchemaType` and `VolDictType`; the test suite asserts on error keys, not on probatio's wording. | None for the alias removal. When probatio itself changes a validator's wording or API with a Core release, the harness bump shows it in `tests/test_config_flow.py` and `tests/test_services.py`. |
| 2026.10 | New entity lifecycle hooks: `async_prepare_to_add_to_hass` and `async_entity_id_changed` (developer blog, 2026-10-12). `add_to_platform_start` must not be overridden. | `entity.py` defines `async_entity_id_changed` on the base entity and calls the parent's hook when the base class has one (`Entity` at 2026.10.1 does not define it yet); nothing overrides `add_to_platform_start`. | None. |
| 2026.11 | `entity_id` changes are applied in place for integrations that implement `async_entity_id_changed`; an integration that overrides `async_added_to_hass` or `async_will_remove_from_hass` must implement it to opt in. | Opted in (above). Nothing of ours caches the entity id: subscriptions key on the robot id and the dispatcher signals. | None; confirm with `tests/test_entities.py`, which checks the hook exists and calls `super()`. |
| 2027.11 | The compatibility path for integrations that did not opt in is removed. | Not affected (opted in since 2026.10). | None. |
| 2026.12 | Options flows that call `async_update_reload_and_abort` on an entry that also has an update listener break (`breaks_in_ha_version` 2026.12.0). | The options flow is an `OptionsFlowWithReload` and the integration registers no update listener; the hub re-reads its options on the reload. | None. Never add `entry.add_update_listener`. |
| 2027.6 | The MQTT publish API stops accepting `None` as a payload (typed payloads). | Every `mqtt.async_publish` call in `hub.py` and `sensors_out.py` passes bytes: encoded JSON, or `b""` for a retained clear. Subscriptions use `encoding=None` and handle bytes. | None; `grep -n async_publish custom_components/ros2_bridge/*.py` at that release to confirm no `None` crept in. |
| 2027.10 | `DeviceEntry.config_entries` is removed for custom integrations (deprecated since the 2026-09-15 developer blog); the `remove_config_entry_id` form of `async_update_device` raises from 2027.8. | The integration never reads `DeviceEntry.config_entries`; a tombstone removes the device with `async_remove_device` by its `(ros2_bridge, robot_id)` identifier (decision log, 2026-10-14). | None. |
| 2026.10 | `mqtt.publish` and `mqtt.dump` are administrator-only. | Not used. Commands leave only through the integration's own actions, buttons and intents, which run with the calling user's permissions. | None; it is the reason the integration has its own actions. |

Two things the table does not cover but every monthly check should: the HACS validation action and hassfest (`.github/workflows/ha.yml`) run against the current Core, so a new hassfest rule or a `quality_scale.yaml` schema change appears there first; and `custom_components/ros2_bridge/manifest.json` must keep every key hassfest requires at that release.

## ROS 2 distribution outlook

| Distribution | Status (as of 2026-10-14) | Position of this project |
|---|---|---|
| Jazzy Jalisco | LTS, Ubuntu 24.04, Python 3.12, supported to May 2029. | The target (interview decision Q6). CI builds and tests in `ros:jazzy-ros-base` (`.github/workflows/ros2.yml`, `tools/Dockerfile.ros2`). Isaac Sim 6.1's ROS 2 bridge supports Humble and Jazzy only, so the simulator site stays on Jazzy. |
| Kilted Kaiju | Non-LTS, Ubuntu 24.04, end of life late 2026 (November or December; sources disagree). | Not a target. Nothing in the package is Jazzy-specific on purpose, so it is expected to build there, but it is not tested and Kilted's end of life makes it a poor place to be. |
| Lyrical Luth | LTS, released 2026-05-22, Ubuntu 26.04, Python 3.14 (inferred from the Ubuntu 26.04 packages; REP 2000 has no Lyrical section yet), supported to May 2031. | The next target when the robot machine moves to Ubuntu 26.04. A "Lyrical CI job" is on the post-release track. Not supported by Isaac Sim 6.1. |
| Humble Hawksbill | LTS, Ubuntu 22.04, Python 3.10, to May 2027. | Not a target and not tested. By inspection the ROS half uses no syntax newer than Python 3.10, but ruff's `py312` target does not guard that and nothing is promised there. |

### What a move to Lyrical needs

Nothing in the design depends on Jazzy specifically; the list is what has to be re-verified, not rewritten.

1. **Python 3.14.** The ROS half is written to the `py312` target in `pyproject.toml` (`ruff` enforces the syntax floor) and the Home Assistant half already runs on 3.14, so the language is not the obstacle. paho-mqtt 2.1.0 on Python 3.14 is what Home Assistant 2026.10 itself runs, which is strong evidence it works there; the node checks the paho version at import and nothing else.
2. **rclpy.** The node uses periodic timers (one-shots are self-cancelling periodic timers because Jazzy has no single-fire timer), a `GuardCondition` triggered from paho's thread, a `SingleThreadedExecutor`, a `STEADY_TIME` clock, declared parameters with descriptors, and a `std_msgs/String` publisher. Each is checked in `ros2/ros2_ha_bridge/test/test_ros_adapters.py` and `test_bridge_node.py`; run them under `colcon test` in a `ros:lyrical-ros-base` container before anything else. If Lyrical's rclpy gains a one-shot timer, `RclpyOneShot` in `ros_adapters.py` is the one place to use it.
3. **The rig and CI.** `tools/Dockerfile.ros2` has the base image on its `FROM` line and `.github/workflows/ros2.yml` names `ros:jazzy-ros-base` and `ROS_DISTRO: jazzy`; a Lyrical job is a second matrix entry, not a rewrite. `rosdep` keys in `package.xml` (`rclpy`, `rcl_interfaces`, `std_msgs`, `action_msgs`, `launch`, `launch_ros`, `ros2launch`, `python3-yaml`) exist on every active distribution.
4. **Linters.** `ament_flake8` and `ament_pep257` ship per distribution; the package's `.flake8` reconciles ruff and ament on Jazzy, and the first Lyrical run shows whether a rule moved.
5. **The MoveIt reference executor.** `moveit_msgs` error code names and values are checked against the Jazzy release of `moveit_msgs`; a distribution move re-checks that mapping (`docs/writing-an-executor.md`).
6. **Isaac Sim.** Not available on Lyrical (6.1 supports Humble and Jazzy). A site that uses Isaac Sim keeps a Jazzy robot machine, and because the bridge never talks to Isaac directly, the two halves of a mixed fleet (Jazzy here, Lyrical there) still meet on the same broker and the same schema 1.

A move to Lyrical is a decision for the decision log, with its date, after the `colcon test` run above has been seen green.

## Pinned versions to watch

| Pin | Where | Why it matters |
|---|---|---|
| paho-mqtt 2.1.0 | `ros2/ros2_ha_bridge/requirements.txt`, Home Assistant's own MQTT integration | Both halves speak MQTT 5 through it; the node refuses an older paho. Ubuntu 24.04's apt package is 1.6.1 and must not be used. |
| Home Assistant 2026.10.0 minimum | `hacs.json` | The first release the integration was written against; probatio, the lifecycle hooks and administrator-only `mqtt.publish` are all in place from here. |
| pytest-homeassistant-custom-component 0.13.372 | `requirements_test.txt` | Tracks Core 2026.10.1 and is the monthly trip-wire described above. |
| Mosquitto add-on | Home Assistant's add-on store | The stock add-on does not enforce topic ACLs (`SECURITY.md`); a future add-on release that does is a reason to add broker ACLs as a second layer, not to remove the node's own checks. |

## After any upgrade

Run `tools/clock_check.py` (or `docker compose run --rm clock-check --require-heartbeat`), press one task button on a fake robot, and look at the diagnostics download once. Steps 1 to 3 of `tools/e2e_checklist.md` are the short version of a post-upgrade check; the whole checklist is for a release.
