# Home Assistant + ROS2 robotic arm bridge: requirements and plan

Status: interviewing (started 2026-10-13). Working folder `<repo root>` is empty (greenfield, not a git repo).

## Context

Jonny wants Home Assistant (HA OS, Mosquitto add-on already installed) to work alongside ROS2 running on a separate machine on the LAN. HA owns sensors and triggers; ROS2 directly controls robotic arms. Everything must run locally. The open design question is whether to build an HA add-on, an HA custom integration, or use MQTT with HA's existing MQTT integration plus a bridge on the ROS2 side.

## Interview record (one question at a time, dated)

| # | Date | Question | Answer | Decision or consequence |
|---|------|----------|--------|-------------------------|
| 1 | 2026-10-13 | First end-to-end scenario: which HA event should make an arm do what? | Not fixed yet. Wants a general framework so ROS2 reacts as fast as possible to any HA trigger. Example: no motion for 5 min, HA turns lights off and tells ROS2 to reset arms to default position. HA and ROS2 side by side, coexisting. Jonny's own idea: HA scripts for everything, MQTT publish as the first step so ROS2 reacts first. | Design must be a generic, low-latency event/command channel, not a bespoke one-scenario link. Latency of the HA-to-ROS2 hop is a first-class requirement. |
| 2 | 2026-10-13 | Does ROS2 need to send state back into HA, or is it one way? | Two way. Example setting: a kitchen with lights, sensors and two robotic arms for assistive care. HA must know what state the robots are in. Critical requirement: ROS2 must react as fast as possible for safety reasons. | Robot state must appear in HA as entities (status, busy, fault, position) for dashboards and automation conditions. Assistive-care context raises the bar: anything safety-relevant must not depend solely on HA or MQTT being up. Heartbeat/watchdog both ways is now a requirement, not a nice-to-have. |
| 3 | 2026-10-13 | Is HA in the safety path (HA signals that must stop or slow the arms), or convenience only? | Not fixed. Whichever route is fastest and safest to get the signal to the robot. Ideally HA and ROS2 both learn of changes as soon as possible. | Design goal: sensor events should reach ROS2 and HA in parallel where possible, rather than serially through HA automations. Mosquitto as a shared bus can do this if the sensors already publish to MQTT (Zigbee2MQTT). HA automations remain the route for derived or timed triggers (for example "no motion for 5 minutes"). Hardware e-stop remains authoritative and outside this design. |
| 4 | 2026-10-13 | How will the kitchen sensors connect to HA (Zigbee2MQTT, ZHA/Z-Wave/Thread, mixed)? | Mainly Zigbee2MQTT, so already on Mosquitto. Other protocols may be used by some sensors and must be catered for. | Two sensor paths: (a) fast path, ROS2 subscribes directly to Zigbee2MQTT topics for raw events; (b) normalised path, HA republishes any non-MQTT sensor to a canonical MQTT topic so ROS2 sees one consistent schema regardless of source protocol. The canonical schema is a deliverable. |
| 5 | 2026-10-13 | Is this for your own setup only, or to be shared or published for others? | 2: to be shared or published (GitHub, HACS or add-on store). | Must be packaged properly from day one: a ROS2 package for the bridge node, clear install docs, versioning, a licence, tests, and an HA-side artefact that installs cleanly (HACS integration or documented MQTT discovery). Configuration must not assume Jonny's entity names or topics. |
| 6 | 2026-10-13 | Which ROS2 distro and OS is (or will be) on the ROS2 machine? | 1: Jazzy on Ubuntu 24.04, already in use. | Target Jazzy (Python 3.12, rclpy, ament_python packaging, LTS to May 2029). Keep the bridge distro-agnostic where cheap so Kilted and later work too. |
| 7 | 2026-10-13 | What commands should HA be able to send: fixed named commands only, or parameterised ones? | 3: both, named commands first, parameters later. | Phase 1 command set is a small enum (reset, pause, resume, stop at minimum) surfaced in HA as buttons or a select. The message schema must carry an optional parameters object from day one so phase 2 (poses, tasks with arguments) is additive, not a breaking change. |
| 8 | 2026-10-13 | What is an acceptable worst-case delay from a sensor event to the arm starting to react? | 4: no figure yet, asked for a recommendation. | Provisional targets to confirm against research: fast path (Zigbee2MQTT topic to ROS2 callback) under 50 ms broker-to-node, under 200 ms end to end; HA-automation path (derived or timed triggers) under 1 s. The sensor radio itself, not the bus, is expected to dominate. Heartbeat interval and timeout to be derived from these numbers. Jonny to confirm or tighten once measured on his LAN. |
| 9 | 2026-10-13 | What drives the arms in ROS2 today (MoveIt 2, vendor driver, custom nodes, nothing yet)? | It depends: MoveIt 2 or any other custom controller. The real need is to translate voice (HA Assist via Voice PE) into a clear set of actions for the arm. Eventually "pick up the mug" resolves to a specific mug with a unique ID and an exact planned trajectory. If MoveIt fails it returns a failure code with distinct meanings (collision, self-collision, trajectory not possible, and so on). | The bridge must be controller-agnostic: HA sends intents or tasks, a ROS2-side task executor maps them to MoveIt or a custom controller. The contract is request-response (ROS2 action semantics: goal, feedback, result, cancel) carried over MQTT, with a structured result code and human-readable reason so HA can speak the outcome back. Voice is a first-class input, so the HA side needs Assist intent handling and a service that can await a result. This pushes the HA side towards a custom integration rather than discovery-only MQTT entities. |
| 10 | 2026-10-13 | When a voice command starts a task, should the spoken reply wait for the outcome, or acknowledge immediately and report later? | 1: acknowledge immediately, report outcome or failure afterwards by speech and/or notification. Jonny stressed it is not just voice: any sensor can be a trigger (the motion example). | Every task is asynchronous regardless of trigger source. HA side needs: a send-task service that returns a task id and a fast accepted/rejected ack; task progress and result surfaced as HA events and entities so any automation (speech, notification, lights) can react to the outcome. No blocking waits inside Assist. |
| 11 | 2026-10-13 | Should HA address each arm separately, the pair as one unit, or both? | 1: each arm separately, each with its own state and commands in HA. Must scale from 1 to 12 or more robots because it is to be shared; current case is 2 arms. | One HA device per robot, discovered dynamically from what ROS2 announces (no hard-coded count). Topic namespace carries a robot id. Group actions ("reset everything") are left to HA automations targeting several devices, not a special unit-level target. |
| 12 | 2026-10-13 | Do you have a simulated or fake-hardware setup so we can test without moving real arms? | Yes, a simulated environment exists; everything can be tested there when needed. Added later the same day: the simulator is NVIDIA Isaac (Isaac Sim); Jonny has no physical access to the arms at present. | End-to-end verification runs against Isaac Sim first, via its ROS2 bridge and whatever drives the simulated arm (MoveIt 2 or custom). The bridge itself never talks to Isaac directly; the executor plugin does, so Isaac changes the test plan, not the architecture. The bridge must also ship its own fake executor so the HA side can be tested with no ROS2 or Isaac at all (adopters and CI). Isaac Sim's ROS2 Jazzy support and where Isaac runs are being checked (see research 2). |
| 13 | 2026-10-13 | How will I reach the ROS2 machine to develop and test (SSH from here, Jonny runs commands, or code only)? | No ROS2 set up here at present; that comes later. Goal is a framework that can be copied to a different site where HAOS runs on bare metal and ROS2 Jazzy runs on Ubuntu. | Deliverable is a portable repository, not a live deployment. Local testing must not depend on the target machines: candidate is ROS2 Jazzy in Docker or WSL2 on this PC plus a throwaway HA container, with the bridge's fake executor standing in for the arms. Jonny's live HA instance is not a test bed. |
| 14 | 2026-10-13 | May I use Docker or WSL2 on this PC to run ROS2 Jazzy and a test HA locally? | Use the live HA instance on the network (HAOS at the live HA host (address withheld), Mosquitto add-on on the same box, HA already uses it). Everything (broker address, port, credentials, topics) must be configurable for different environments. ROS2 side has no home yet here, so ROS2 Jazzy runs in Docker on this PC for testing (Docker Desktop 28.1.1 and WSL2 present). | Test rig: ROS2 Jazzy container on this PC running the bridge with the fake executor, pointed at Mosquitto on the live HA host (address withheld); HA integration installed on the live HA. Safeguards because the HA is live: dedicated MQTT user and ACL scoped to the bridge topic prefix, no retained command messages, fake executor only, nothing touches real arms. No hard-coded IPs anywhere in the code; broker settings come from HA's MQTT integration on the HA side and from ROS2 parameters or env vars on the robot side. |
| 15 | 2026-10-13 | MQTT security: dedicated user with ACL, TLS, or reuse an existing user? | 3: reuse the existing MQTT user for testing. Credentials must be a configuration option; the shareable integration and ROS2 package must let each user enter their own broker details. | Broker host, port, username, password, TLS on/off and topic prefix are all configuration on both sides (ROS2 parameters or env vars; HA side inherits HA's own MQTT connection, so no second credential entry there). Because the stock Mosquitto add-on 7.1.1 does not enforce ACLs (research finding), the ROS2 node carries application-layer authorisation by default: a command allowlist, stale-command rejection, and an optional shared-secret HMAC on commands that adopters can switch on. Broker ACLs are documented as an extra layer for brokers that support them. |
| 16 | 2026-10-13 | If the HA heartbeat stops arriving, what should the default behaviour be? | 3: nothing, HA going away must not affect the arms. | HA is advisory; ROS2 owns all safety behaviour. The HA-to-ROS2 heartbeat is kept but is informational only by default (exposed as an "HA link" state on the ROS2 side, logged, no motion effect). Watchdog mode is a ROS2 parameter (`none` default, `inhibit_new`, `stop_and_hold`) so other adopters can choose. The ROS2-to-HA direction (availability via LWT, state with expiry) is unaffected and remains required. |
| 17 | 2026-10-13 | What robot state should HA receive: discrete states and events, a 1 Hz summary, or live joint data? | 1: discrete states and events only (idle, busy, error, task done, task failed with reason). | State topic is retained, published on change only, with a small enum plus current task id. Results are events with a structured code and reason. No high-rate telemetry, so no recorder exclusions and no JSON message introspection needed. Extra telemetry can be added later as optional, non-retained topics. |
| 18 | 2026-10-13 | Which open-source licence for the published repository? | 1: Apache-2.0. | LICENSE file plus SPDX headers in source files; package.xml and manifest.json carry the licence string. Public GitHub repository assumed because HACS requires one. |
| 19 | 2026-10-13 | What should the project be called? | 1: `ha-ros2-bridge`. | Repository `ha-ros2-bridge` (monorepo), HA integration domain `ros2_bridge` under `custom_components/ros2_bridge/`, ROS2 package `ros2_ha_bridge` under `ros2/ros2_ha_bridge/`. Default MQTT topic prefix `ros2_bridge/`. |

| 20 | 2026-10-13 | Where does Isaac Sim run? | 2: only at the other site, not reachable at present. | Isaac testing is deferred until that site is reachable. Local verification uses the fake executor in the ROS2 Jazzy container plus the live HA. The repository ships an Isaac and MoveIt 2 executor example and a written test procedure so the other site can run it without Jonny or me present. |

Interview closed 2026-10-13 after 20 questions. Remaining choices are recorded as assumptions below rather than asked, because each has a conventional default.

## Assumptions recorded instead of asked (2026-10-13)

- Non-administrator household members and carers will start tasks from dashboards or voice, so command entry goes through the integration's own service and discovery buttons, never through `mqtt.publish` in scripts.
- Network may be Wi-Fi at some sites, so TLS is a configuration option on the ROS2 side (off by default on a wired LAN).
- CPU architecture is not constrained: the ROS2 node is pure Python; the test container uses the multi-arch `ros:jazzy` image.
- This PC (checked 2026-10-13): NVIDIA TITAN X (Pascal), 32 GB RAM, 12 logical processors, Docker Desktop 28.1.1, WSL2 with Ubuntu 22.04 and Ubuntu distros, Python 3.13.7, git 2.51. Isaac Sim's published requirements call for an RTX-class GPU, so Isaac Sim is not expected to run here (to be confirmed by research 2). Local testing therefore uses the fake executor; Isaac testing happens on the machine that hosts Isaac Sim.
- Local repository lives in `<repo root>` and is initialised as a git repository on first implementation step. Creating the public GitHub repository is an outward-facing action and will be confirmed with Jonny at that step, not assumed.
- During development the integration is copied into the live HA's `custom_components/ros2_bridge/`. Routes available per the HA management workspace rules (local, not in this repository): the HA MCP `ha_write_file` tool first, the `Y:\` Samba share as last resort and only with Jonny's confirmation. `.storage/` is never touched. HACS (custom repository) is the published route once the GitHub repository exists.
- Example automations and a sample dashboard for the kitchen scenario ship as documentation, not as changes to Jonny's live HA.
- Hardware e-stop, protective stop and collision handling are the arm controller's responsibility and out of scope for this project.

## Verified facts about the live HA (read over MCP, 2026-10-13)

- HA Core 2026.10.1, HAOS, language en-GB, timezone Europe/London.
- Mosquitto add-on 7.1.1 (core repo), running, ports 1883 (MQTT), 1884 (WebSocket), 8883 and 8884 (TLS variants). `customize.active` is false, so no ACL file is in use yet. `require_certificate` is false. One add-on-defined login exists (name withheld); the add-on also accepts HA user accounts for MQTT auth. Secrets are not recorded here.
- MQTT integration: one config entry (source hassio, title core-mosquitto), discovery enabled, discovery prefix `homeassistant`, birth and will messages on `homeassistant/status` (payloads online/offline, QoS 0, not retained).
- Consequence: the HA birth message on `homeassistant/status` is the standard hook for the ROS2 bridge to re-announce discovery after an HA restart. A dedicated MQTT user for the bridge needs the add-on's customize ACL feature turned on, which is a config change on the live add-on and needs Jonny's approval before implementation.

## Research (background workflow wf_0adfedda-e11, read-only web sweep, completed 2026-10-13)

Eleven agents, 924 tool calls, primary sources only. Full output with every source URL: `docs/research/2026-10-13-landscape-research.json`. Headlines that shape the design:

### Transport verdict
- MQTT via the existing Mosquitto add-on is the right transport. Every living precedent (MowgliNext 2026, OASIS 2026, an April 2026 HA forum patrol robot) chose "purpose-built ROS2 node publishing HA MQTT discovery". No HA add-on, HACS integration or sidecar for ROS2 exists anywhere (GitHub searches 2026-10-13).
- A ROS2 runtime inside an HA add-on is ruled out: needs host networking (against HA add-on security guidance), couples HA to the arm's distro and RMW, no prior art.
- rosbridge (WebSocket JSON) is maintained (jazzy 2.7.1, actions supported) but has had no authentication since 2021; only useful as a throwaway prototype, not the product.
- zenoh, Foxglove bridge and HTTP shims: not recommended for this (CDR payloads, no retained messages, no maintained clients, or simply more work than MQTT for the same node).

### ROS2 side
- No path avoids a ROS2 node of roughly 250 to 400 lines: discovery payloads, birth/online, command validation, ack and de-duplication, heartbeat watchdog and action lifecycle are behaviour, not config. ika-rwth-aachen/mqtt_client 2.5.0 (apt on Jazzy, MIT, maintained) can carry raw topics but emits no discovery, no birth, is MQTT 3.1.1 only, cannot do services or actions ("not planned", issue #72), and its JSON mode is 2.5 months old with no CI tests and silent zero-fill on missing fields. Decision: write one purpose-built rclpy node using paho-mqtt 2.1.0 (CallbackAPIVersion.VERSION2), which is the same library HA itself pins. Reference implementations: OASIS hass_mqtt_bridge_node.py (319 lines, Python) and MowgliNext mqtt_bridge_node.cpp plus its docs/MQTT_CONTROL.md.
- MQTT 3.1.1 and MQTT 5 clients coexist on Mosquitto 2.1.2 (verified), so HA's 2026.6 MQTT 5 requirement does not constrain the ROS side; use MQTT 5 anyway for message expiry on replies.

### HA side
- HA Core 2026.10.1: device-based MQTT discovery (since 2024.11) creates all entities from one payload per robot; platforms available include button, select, number, text, sensor, binary_sensor and event. Subentries made by hand in the UI cannot be exported to discovery later, so discovery from the ROS side is the route.
- HA does not wait for SUBACK (0.5 s subscribe debounce), so a fast non-retained ack can be missed. Replies (ack, result) must be published retained on per-command topics with an MQTT 5 message expiry (60 to 300 s). The MQTT event entity ignores retained replays, so it is the right carrier for results on dashboards.
- mqtt.publish is administrator-only since 2026.10 (PR #182045). A script a non-admin runs from a dashboard fails at the publish step. Automations are unaffected. This is a reason for a custom integration with its own service.
- Stock HA scripts can do request-response over MQTT (verified at tag 2026.10.1: wait_for_trigger on an mqtt trigger with payload_json, timeout, response_variable), so a discovery-only phase is viable before any integration code exists.
- A thin MQTT-based custom integration (rides HA's MQTT connection via the mqtt component helpers, pattern used by tasmota and pglab) is the justified form of integration: typed services with responses, subscribe-before-publish, Assist intent handlers, per-robot devices. Estimated 8 to 12 files, 600 to 1,200 lines with tests. HACS needs a public GitHub repo with releases. It cannot import rclpy (HA core is Alpine/musl Python 3.14), which is fine: the integration speaks MQTT only.

### Safety findings
- The stock Mosquitto add-on 7.1.1 does not enforce an ACL file via the customize folder (home-assistant/addons issue #4571 open since 2026-04-17, PR #4831 unmerged). Any authenticated MQTT client (Zigbee2MQTT, ESPHome, any HA user) can publish arm commands. Options: application-layer authorisation on the ROS side (command allowlist plus HMAC or nonce), pin the add-on to 6.5.2, run a standalone broker, or wait for the fix. This is from an open issue, not tested by me; to be verified on the live add-on before relying on it.
- Physical e-stop stays hardwired; nothing in HA or MQTT is labelled e-stop. MQTT carries at most a non-safety "inhibit" or "stop".
- Fail safe by absence: HA publishes a heartbeat every 1 to 2 s; the ROS2 watchdog treats silence beyond 3 to 5 s as loss of permission, refuses new goals, optionally cancels the active goal, and reports "inhibited". Broker keepalive alone is too slow (Mosquitto drops a client after 1.5 x keepalive, 90 s at defaults; HA minimum 15 s).
- Commands: never retained, QoS 1, with command id (UUID), issued_at and max_age; ROS side rejects stale commands and de-duplicates. clean_session true on the ROS side so a restart never drains a backlog of motion commands.
- Action mapping: cmd/goal, cmd/cancel, goal/<id>/ack, goal/<id>/feedback (QoS 0, throttled), goal/<id>/result with action_msgs/GoalStatus codes, plus a retained goal/current.
- Buttons for named motions rather than a select (selecting an item must not start motion).
- Standards posture: ISO 10218-1/-2:2025 and ISO 13482 are not obligations for a domestic arm but their principles transfer. Cybersecurity counts as safety: authenticated MQTT, TLS if Wi-Fi is shared.

### Versions (verified 2026-10-13)
- ROS2: Jazzy (LTS to May 2029, Ubuntu 24.04, Python 3.12), Kilted (EOL late 2026), Lyrical Luth (released 2026-05-22, Ubuntu 26.04, Python 3.14). Default RMW is Fast DDS on all.
- HA Core 2026.10.1, Python 3.14; MQTT integration pins paho-mqtt 2.1.0; requires an MQTT 5 broker since 2026.6.
- Mosquitto add-on 7.1.1 = Mosquitto 2.1.2 + mosquitto-go-auth 3.0.0 (upstream archived 2025-06-08). Mosquitto 2.1 deprecated acl_file and password_file (removal at 3.0).

### Research 2: NVIDIA Isaac Sim and MoveIt 2 (workflow wf_509a2a66-6c0, completed 2026-10-13)

Full output: `docs/research/2026-10-13-isaac-sim-moveit.json`.

- Pin Isaac Sim 6.1.0 (released September 2026). Its ROS2 bridge officially supports Humble and Jazzy only; Ubuntu 24.04 with Jazzy is NVIDIA's recommended pairing and shares Python 3.12 with Isaac. Kilted is unsupported (NVIDIA staff, 2026-01-16). Topics only publish while the simulation is playing.
- Isaac Sim cannot run on this PC: 6.x dropped Windows 10, needs an RT-core GPU with 16 GB VRAM minimum (RTX 4080 class), and the Pascal TITAN X cannot reach the required 595 driver branch. Containers are Linux-only and unsupported under WSL2. Confirmed: Isaac is deferred to the other site.
- The executor plugin should drive MoveIt 2 through a plain rclpy action client on `/move_action` (moveit_msgs/action/MoveGroup, moveit_msgs 2.6.0 on Jazzy, moveit 2.12.4). Named poses are resolved by the plugin (SRDF group states or a YAML map) into JointConstraints, because MotionPlanRequest has no named-target field. Unknown pose names are rejected before MoveIt is called.
- MoveItErrorCodes has 31 values; success only when `val == 1`. Proposed mapping to bridge result codes: SUCCEEDED, REJECTED, PLANNING_FAILED, EXECUTION_FAILED, TIMED_OUT, CANCELLED, UNAVAILABLE, INTERNAL_ERROR, with the raw MoveIt value and message always carried in the result payload so HA automations can branch on the fine-grained code. The full table is in the research output.
- Three test tiers behind one xacro switch (`ros2_control_hardware_type`): Tier 1 `mock_components/GenericSystem` (runs on this PC, no GPU, covers the whole MQTT to executor to move_group to error-code path including negative tests); Tier 2 Isaac classic bridge with `topic_based_hardware_interfaces` 1.1.0 (`sum_wrapped_joint_states: true`); Tier 3 Isaac Sim 6.1 in-process ros2_control (controllers must be re-spawned after every Stop/Play).
- Local topology on this PC: everything that speaks DDS stays inside one Docker network (or one WSL2 distro); only MQTT over TCP crosses to HA. Set `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. Never attempt DDS from Docker Desktop to anything outside its network.
- Requirements for the Isaac site are listed in the research output (Ubuntu 24.04, RT-core GPU 16 GB+, driver 595 branch, Jazzy system install, moveit 2.12.4, ros2_control 4.49, ros2_controllers 4.43, simulation-interfaces 1.5.0+, Fast DDS UDPv4 profile if containerised). The site's GPU is unknown and must be checked before scheduling Isaac tests.

### Unverified items carried forward
- Exact JSON that mqtt_client emits for JointState (moot if we write our own node).
- Whether a non-admin manually triggering an automation that publishes MQTT works at runtime (source says yes, no test covers it).
- Whether `mosquitto_acl_file.so` is present in the add-on image.
- OASIS reported late and out-of-order HA events over a two-node MQTT path (2025-05-01); likely a double-hop artefact. Our design has one hop and must measure.

## Design panel (workflow wf_ad9a3fc5-008, completed 2026-10-13)

Three independent designs (simplest, safety-first, adopter-first), three judges (one each picked a different winner, margins thin), one synthesis grafting the best of each, two sceptics (11 and 8 objections, both "needs repair"), one repair pass applying every blocker and major objection. Full final design, with every verified HA and ROS API fact and its URL: `docs/research/2026-10-13-design-panel.json`. The plan below is that design, condensed.

## Recommendation

**An HA custom integration plus a ROS2 package, joined by a written MQTT contract. Not an add-on.**

- `ros2/ros2_ha_bridge/` (ament_python, Jazzy, paho-mqtt 2.1.0 via pip): one process per robot. Announces the robot and its task catalogue to HA, authorises and runs commands through a `TaskExecutor` plugin, publishes state, availability (LWT), heartbeat, acks and results.
- `custom_components/ros2_bridge/` (HACS, rides HA's own MQTT connection via the `mqtt` component helpers, no pip requirements): one device per announced robot, services `send_task`, `cancel_task`, `get_robots`, bus events for ack and result, two Assist intents, diagnostics, repairs.
- `contract/`: JSON schemas, golden and must-reject examples and HMAC test vectors that both halves are tested against. Schema 1 only ever gains optional fields.

Why this shape (sources in the research sections): every living HA-to-ROS2 precedent is a purpose-built ROS node publishing MQTT discovery; ROS2 cannot usefully run inside an HA add-on; an add-on cannot create HA entities or services by itself; stock MQTT discovery alone cannot give non-admin users or Assist a typed task action (`mqtt.publish` is admin-only since 2026.10); HA's own MQTT connection means adopters enter broker details once, in HA.

### Repository layout (monorepo `ha-ros2-bridge`, Apache-2.0, local path `<repo root>`)

```
LICENSE  README.md  CHANGELOG.md  hacs.json  pyproject.toml  requirements_test.txt  .pre-commit-config.yaml
.github/workflows/{ha,ros2,contract,release}.yml      brand/icon.png
contract/README.md  contract/schemas/*.schema.json  contract/examples/{valid,invalid}/<kind>/*.json  contract/hmac_vectors.json
custom_components/ros2_bridge/
  __init__.py manifest.json const.py protocol.py models.py hub.py commands.py store.py entity.py
  sensor.py binary_sensor.py event.py button.py services.py services.yaml intent.py sensors_out.py
  config_flow.py diagnostics.py repairs.py strings.json translations/en.json icons.json quality_scale.yaml
ros2/ros2_ha_bridge/
  package.xml setup.py setup.cfg requirements.txt .flake8 resource/ config/*.example.yaml launch/{bridge,fleet}.launch.py
  ros2_ha_bridge/{bridge_node,mqtt_link,protocol,authoriser,task_manager,watchdog,sensor_mirror,dedupe_store}.py
  ros2_ha_bridge/executors/{base,fake}.py   ros2_ha_bridge/schemas/*.json (vendored, tests only)
  test/ (fakes.py with FakeBroker and FakeMqttClient, one test module per source module, test_contract.py)
tests/ (HA: config_flow, init, hub, commands, entities, services, intent, sensors_out, diagnostics, repairs, contract, snapshots/)
tools/ (Dockerfile.ros2, Dockerfile.ha-tests, compose.yaml, mosquitto/, .env.example, e2e_checklist.md, latency_probe.py, clock_check.py)
scripts/ (check_spdx.py, check_versions.py, sync_contract.py)
docs/ (architecture, mqtt-contract, failure-modes, safety, security, install-ha, install-ros2, configuration,
       writing-an-executor, assist, sensors, troubleshooting, upgrading, decisions, examples/, research/)
```

### MQTT contract (essentials)

Conventions: `{prefix}` defaults to `ros2_bridge` (not `homeassistant`, no wildcards or slashes at the ends); robot ids and task names match `^[a-z][a-z0-9_]{0,31}$`, `ha` reserved; command ids are UUID4 and are validated before any topic is built; every payload carries `"schema": 1`; 16 KiB limit (64 KiB for announce); MQTT 5 both sides; ROS connects with `clean_start=True`.

| Topic | Dir | QoS | Retain | Notes |
|---|---|---|---|---|
| `{prefix}/{robot}/announce` | ROS to HA | 1 | yes | identity, `session`, `instance`, task catalogue with `kind` (motion, control, query) and params schema subset; empty payload is a tombstone; republished on connect and on HA birth |
| `{prefix}/{robot}/availability` | ROS to HA | 1 | yes | `online`; `offline` as LWT and on clean shutdown |
| `{prefix}/{robot}/state` | ROS to HA | 1 | yes | `idle`, `busy`, `error` plus `task_id`, `paused`, `inhibited`, `ha_link`, `executor_health`, `error`, `seq` |
| `{prefix}/{robot}/heartbeat` | ROS to HA | 0 | no, 5 s expiry | every 2 s; counters and `ha_clock_offset_ms` |
| `{prefix}/{robot}/cmd` | HA to ROS | 1 | never | `type` task or cancel, `id`, `task`, `params`, `issued_at`, `max_age_ms`, `source`, optional `sig`; node subscribes Retain As Published and drops and clears any retained copy |
| `{prefix}/{robot}/task/{id}/ack` | ROS to HA | 1 | yes, 120 s expiry | closed set of codes (`accepted`, `rejected_busy`, `rejected_stale`, `rejected_unauthorised`, ...); duplicate id republishes the stored ack |
| `{prefix}/{robot}/task/{id}/feedback` | ROS to HA | 0 | no, 10 s expiry | optional, throttled to 2 Hz |
| `{prefix}/{robot}/task/{id}/result` | ROS to HA | 1 | yes, 300 s expiry | exactly one per accepted task; `status` succeeded, cancelled, aborted (GoalStatus 4, 5, 6); `code` from a recommended vocabulary (`collision`, `self_collision`, `no_trajectory`, `timeout`, ...); `data` carries the raw MoveIt code |
| `{prefix}/ha/heartbeat` | HA to ROS | 0 | no, 5 s expiry | every 2 s; informational unless the node's watchdog mode says otherwise |
| `{prefix}/ha/sensors/{domain}/{object_id}` | HA to ROS | 0 | yes | canonical republish of chosen HA entities (the normalised sensor path) |
| `zigbee2mqtt/...` | Z2M | as published | | fast path: the node mirrors chosen topics unchanged onto `~/mqtt_in` |

Command handling on the node, first failure wins: retained drop, oversize drop, UUID id check (no reply possible otherwise), schema, robot id match, HMAC when a secret is set, persisted dedupe (republishes stored ack), freshness window with 2 s skew tolerance, allowlist, params, state gating by kind (motion refused while busy, error or inhibited), then `executor.start`. HA side matches replies by `id` alone, re-adopts running tasks across HA restarts from a Store, synthesises `no_ack`, `no_result`, `node_restarted` only when the robot truly never answers, and fires `ros2_bridge_task_result` exactly once per id.

### ROS2 node (essentials)

- Threading: paho network thread appends to a bounded deque and triggers an rclpy GuardCondition; a SingleThreadedExecutor on the main thread does all state changes and all publishes; publishes are guarded (no exception ends the process). rclpy Jazzy has no one-shot timer, so one-shots are periodic timers that cancel themselves first.
- Executor plugin interface (`executors/base.py`): `TaskSpec`, `TaskRequest`, `TaskResult`, `TaskRejected`, `TaskHandle` (thread-safe `feedback` and `finish`), `BridgeContext` (`post`, `set_error`, `clear_error`, `set_paused`, `ha_link_ok`), abstract `TaskExecutor` with `setup`, `tasks`, `start`, `cancel`, optional `stop_and_hold`, `health`, `shutdown`. Loaded by entry point group `ros2_ha_bridge.executors` or `package.module:Class`.
- Fake executor tasks: `reset`, `move_to {pose}`, `wait {seconds}`, `fail {code}`, `hang`, `raise_error`, `clear_error`, `stop`, `pause`, `resume`. Deterministic with an injected clock.
- Key parameters (all in `config/bridge.example.yaml`, no real host named anywhere): `robot.id` and identity fields; `mqtt.host`, `mqtt.port`, `mqtt.username`, `mqtt.password` (empty reads `ROS2_HA_BRIDGE_MQTT_PASSWORD`), `mqtt.tls.*`, `mqtt.keepalive_s` 15, `mqtt.client_id` derived from robot id; `topic_prefix`; `executor`; `tasks.allow`; `command.secret` (empty reads `ROS2_HA_BRIDGE_COMMAND_SECRET`, empty means HMAC off), `command.max_age_cap_ms` 30000, `command.clock_skew_tolerance_ms` 2000, `command.dedupe_window_s` 300; `reply.ack_expiry_s` 120, `reply.result_expiry_s` 300; `heartbeat.interval_s` 2; `watchdog.mode` none, `watchdog.timeout_s` 5; `mirror.topics`; `data_dir` for the persisted dedupe file.
- Duplicate robot id: same client id means the broker sends DISCONNECT 0x8E (session taken over); the node backs off and exits non-zero after three takeovers in ten minutes; HA raises a repair.
- Packaging: `package.xml` format 3, Apache-2.0, `exec_depend` rclpy std_msgs action_msgs launch launch_ros; `test_depend` ament_flake8, ament_pep257, pytest. ament_copyright is not used (it cannot read SPDX headers); `scripts/check_spdx.py` is the header gate. Launch files set `respawn=True, respawn_max_retries=3` and pass the two secret env vars through.

### HA integration (essentials)

- `manifest.json`: domain `ros2_bridge`, `dependencies: ["mqtt"]`, `mqtt: ["ros2_bridge/+/announce"]` (MQTT discovery of the first robot), `integration_type: hub`, `iot_class: local_push`, `single_config_entry: true`, `requirements: []`. Validation uses `probatio` directly (voluptuous is not installed at 2026.10.1; verified).
- Config flow: aborts if the MQTT integration is absent or not ready; asks only `topic_prefix` and optional `shared_secret`; broker details are never asked (inherited from HA's MQTT). Options (OptionsFlowWithReload): prefix, secret, `heartbeat_interval_s` 2, `ack_timeout_s` 2, `command_max_age_ms` 5000, `result_timeout_margin_s` 10, `max_pending_per_robot` 16, `heartbeat_unavailable_after_s` 30, `mirror_entities`.
- Runtime: push hub, no coordinator; seven wildcard subscriptions at setup; `async_on_subscribe_done` gates readiness; `async_subscribe_connection_status` drives availability and suppresses the heartbeat while disconnected; a `CommandRegistry` backed by `Store` (`ros2_bridge.commands` v1) survives reloads and restarts; reconciliation runs after a 3 s replay-settle period.
- Entities per robot (unique_id `{robot_id}_...`): `state` sensor (enum idle, busy, error, with attributes), `last_result` sensor (motion tasks only), `problem` binary sensor, `task_result` event entity, one button per parameterless task, a `cancel` button. Unavailable when offline, not announced, MQTT disconnected, or heartbeat stale beyond the timer.
- Services: `send_task` (device or robot id, task, params, `wait` ack, result or none, timeout, max age; optional response), `cancel_task`, `get_robots`. Bus events: `ros2_bridge_task_ack`, `ros2_bridge_task_result`, `ros2_bridge_task_late_result`.
- Assist: `Ros2BridgeSendTask` and `Ros2BridgeCancelTask` registered once in `async_setup` and never removed; resolve the robot by id, name, area or the satellite's area; wait for the ack at most 1.5 s; speak "Starting {label} on {robot}" or the refusal reason; never wait for the result (Q10). Example sentences ship in `docs/examples/custom_sentences/en/`.
- Repairs: `clock_skew`, `hmac_rejected`, `invalid_announce`, `mqtt_v5_required`, `duplicate_robot_id`. Diagnostics redacted. `quality_scale.yaml` with every rule present and honest exemptions. Entities implement `async_entity_id_changed` (2026.11 lifecycle change).

### Delivery phases (each with a done criterion)

0. **Scaffold and contract.** `git init`, LICENSE and SPDX headers, `contract/` schemas, examples and HMAC vectors, `docs/decisions.md` seeded with the 20 interview rows and the design decisions, CI on empty suites, research copied into `docs/research/`. Done when `contract.yml` is green and both validators agree on every fixture. Four one-at-a-time questions for Jonny at this phase: (a) `manifest.json` has no licence key, so the licence lives in LICENSE, SPDX headers, `package.xml` and `setup.py`; (b) when to create the public GitHub repository; (c) whether the node should call `stop_and_hold` once at start-up (proposed off); (d) whether a missing ack should be its own event type or `timeout` with `code: no_ack` (proposed the latter).
1. **ROS node core and fake executor** (rclpy-free modules, FakeBroker, FakeClock, all core tests). Done when the core suite passes on native Windows Python with no ROS installed.
2. **ROS packaging and rig** (`bridge_node.py`, `mqtt_link.py`, launch, Dockerfile, compose). Done when `colcon build` and `colcon test` pass in the Jazzy container and two fake arms run against the live Mosquitto with retained announce, state and availability visible, acks and results on a published command, a malicious id dropped without a crash, LWT on kill, and a duplicate-id container exiting after three takeovers.
3. **HA integration core** (manifest, flows, hub, registry, Store, four platforms, services, events, intents, strings, diagnostics, tests). Done when the HA suite passes with coverage at or above 95 percent, hassfest and the HACS action pass, and end-to-end steps 0 to 5 pass on the live HA.
4. **Safety and robustness** (HMAC end to end, reconciliation, pause suspension, late ack, cancel semantics, retained replay rule, staleness, publish-failed tracking, device removal, five repairs, `failure-modes.md` with a test id per row, `safety.md`, `security.md`, chaos checks). Done when end-to-end steps 6 to 9, 12 and 13 pass and `contract.yml` enforces the failure-modes table.
5. **Sensor paths and latency** (`mirror_entities`, `SensorMirror`, `latency_probe.py`, example no-motion automation). Done when step 10 passes, measured numbers are in `docs/architecture.md` with the date, and Jonny has confirmed or tightened the provisional targets (one question), after which heartbeat and watchdog defaults are re-derived.
6. **Voice** (intent resolution by area and satellite, sentences, sentence-trigger alternative, speak-back automation). Done when step 11 passes, including after an options save.
7. **Simulator rehearsal** (`docs/writing-an-executor.md` with the MoveIt 2 executor and the result-code table from research 2, `simulator_checklist.md` for the Isaac site). Done when the Isaac site has run it and recorded findings, or Jonny has recorded a dated decision to release on fake-only verification (one question).
8. **Release 0.1.0** (docs complete, CHANGELOG, Dependabot, tag, HACS custom repository install replacing the copied folder). Done when a clean HACS install plus a clean `colcon build` from the tag reproduce steps 1 to 3 from the documentation alone.

Post-release track: executor template package, bloom release, Lyrical CI job, optional discovery-only mode, queueing or pre-emption, ROS `/diagnostics`, reading HA's configured discovery prefix if a public helper appears.

## Verification

- **Unit, no broker, no HA, no ROS:** `pytest` on the rclpy-free ROS core and on the contract tests runs on this PC's Python. HA tests run in the `ha-tests` container (`python:3.14-slim` with the pinned `pytest-homeassistant-custom-component`) or WSL2, with `mqtt_mock_entry` and `async_fire_mqtt_message`. ROS full suite under `colcon test` in `ros:jazzy-ros-base`.
- **Contract:** `contract.yml` checks schemas compile, every example passes or fails as its folder says in both implementations, canonical JSON and HMAC signatures are identical across the two `protocol.py` copies, vendored schemas are byte-identical, and every `failure-modes.md` row names an existing test.
- **Rig:** `tools/compose.yaml` runs two fake arms in the Jazzy container against Mosquitto on the live HA (address from `.env`, never committed). Chaos checks: `kill` (LWT), `pause` (connected but silent), retained plant on `cmd`, 60 s old command, tampered signature, malicious id, duplicate robot id, clock skew via `clock_check.py`.
- **Live HA end to end (`tools/e2e_checklist.md`, 14 steps, summarised):** clocks within 1 s; install and restart with no errors; discovery card, two devices idle, buttons present; `reset` runs to `succeeded` with events; `move_to` with `wait: result`, missing param raises `invalid_params`, `fail {code: collision}` gives `aborted/collision`; `wait` then `cancel_task` gives `cancelled`; pause longer than the margin produces no spurious timeout; second `reset` while busy gives `rejected_busy`; stale, duplicate, retained, malicious-id and tampered-signature commands behave as the contract says; `hang` forces `aborted/timeout` with `problem` on; node pause and kill drive `problem` then unavailability then recovery; HA restart mid-task re-adopts and fires exactly one result; watchdog modes behave; mirror topics and prefix change clear old topics; latency numbers recorded; voice sentences work before and after an options save; duplicate robot id raises and clears the repair; deleting an offline robot clears its retained topics.
- **Safeguards on the live HA (Q14):** fake executor only, existing MQTT user, no real arms, nothing retained on `cmd` except the deliberate test plant, `ha_get_skill_guide` before any HA write, `ha_write_file` for copying the integration (Samba only as last resort with Jonny's confirmation), `.storage` never touched.
- **Isaac tier (deferred, Q20):** `simulator_checklist.md` plus `docs/writing-an-executor.md` let the other site run the MoveIt 2 executor against Isaac Sim 6.1.0 on Ubuntu 24.04 with Jazzy, starting with `ros2 topic echo /clock` from the bridge host, then the Tier 1 negative tests, then the project scene.

## Top risks (full list of 26 in the design output)

- The stock Mosquitto add-on does not enforce ACLs, so authorisation lives in the node (UUID check, allowlist, freshness, dedupe, retained drop, optional HMAC off by default per Q15). Stated plainly in `security.md`.
- Probatio is young (0.13.0) and arrives with HA monthly releases; Dependabot's harness bump is the trip-wire.
- Freshness needs both machines on NTP within 2 s; diagnosable through `clock_skew_ms`, the repair and `clock_check.py`.
- paho-mqtt 2.1.0 must come from pip on Ubuntu 24.04 (apt ships 1.6.1); whether rosdep's pip installer clears PEP 668 without `PIP_BREAK_SYSTEM_PACKAGES=1` is unverified.
- A plugin that blocks for ever inside `start()` stalls that robot's node only; HA shows `problem` within about 6 s and unavailability after 30 s (recorded decision: no thread-pool guard).
- The live HA is the test bed, so an integration bug can raise repairs or log noise there despite the safeguards.
- Outward-facing actions needing Jonny's explicit go-ahead at the time: creating the public GitHub repository, codeowner handle, tagging releases, HACS listing.
