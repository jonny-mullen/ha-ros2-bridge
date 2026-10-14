# Quick start

Get ha-ros2-bridge running in an evening: the Home Assistant integration on a Home Assistant OS box with the Mosquitto add-on, and the ROS 2 package on an Ubuntu 24.04 machine with ROS 2 Jazzy or in Docker, with the shipped `fake` executor so nothing moves. `docs/install-ha.md` and `docs/install-ros2.md` are the full pages; this is the short path.

## 1. What you get and what you need

**What you get.**

- One Home Assistant device per robot, built from what the robot announces over MQTT: a state sensor, a last-result sensor, a problem sensor, a task-result event entity, one button per parameterless task and a Cancel task button, and, for a robot that announces its joints, one sensor per joint with the live position (the fake arms announce seven).
- Three actions (`ros2_bridge.send_task`, `ros2_bridge.cancel_task`, `ros2_bridge.get_robots`), three bus events and two Assist intents, so an automation or a spoken sentence can start a named task and react to its outcome.
- A ROS 2 node that authorises every command itself and runs it through an executor plugin; the shipped `fake` executor exercises the whole path with no hardware.

**What you need.**

- Home Assistant 2026.10 or newer with the MQTT integration connected to an MQTT 5 broker; the Mosquitto add-on qualifies.
- A machine with ROS 2 Jazzy on Ubuntu 24.04 (Python 3.12), or any PC with Docker and Compose v2.
- A broker login (user name and password) for the robot node: with the Mosquitto add-on, any Home Assistant user (make a dedicated non-administrator one for the robots) or a login under the add-on's Logins option; a standalone broker needs a user created there. The stock add-on enforces no topic ACLs, so that login can publish anywhere on the broker (`docs/security.md`).
- Both machines on one LAN, able to reach the broker, and on NTP within 2 s of each other (commands carry an issue time and a maximum age).

## 2. Install the Home Assistant integration

### Option A: HACS custom repository

1. HACS, three-dot menu, Custom repositories.
2. Repository `https://github.com/jonnytinkererofthings/ha-ros2-bridge`, type Integration, Add.
3. Search HACS for "ROS 2 Bridge", Download, then restart Home Assistant.

The 0.1.0 end-to-end run installed by copying the folder (Option B); the HACS route has not yet been exercised against the published repository.

### Option B: copy the folder

1. Get the files: `git clone` the repository on any machine, or download the source archive of the 0.1.0 release (tag `v0.1.0`) from the GitHub Releases page.
2. Copy its `custom_components/ros2_bridge` folder into the Home Assistant configuration folder, so that `<config>/custom_components/ros2_bridge/manifest.json` exists (Samba share add-on, File editor add-on or `scp`).
3. Restart Home Assistant. The integration has no pip requirements.

### Add the integration

1. Settings, Devices and services, Add integration, "ROS 2 Bridge".
2. The one form asks for two fields:

| Field | Default | Notes |
|---|---|---|
| Topic prefix | `ros2_bridge` | Must equal `topic_prefix` on every robot. No slashes at the ends, no `+` or `#`, not `homeassistant`, not starting with the reserved segment `ha` (so not `ha` or `ha/...`; `hallway_bridge` is fine). |
| Shared secret | empty | Optional. Empty sends unsigned commands (the default). Set it to the same value as `ROS2_HA_BRIDGE_COMMAND_SECRET` on the robots to sign every command with HMAC-SHA256. |

It never asks for the broker: host, port, credentials and TLS come from the MQTT integration. Only one entry is allowed per Home Assistant.

**Discovery.** If a robot is already announcing under the default prefix, a discovered card with the robot's name appears under Settings, Devices and services; press Configure, confirm, and the entry is created. Either way the integration creates one device per announced robot within a few seconds, named from the announce, with entities named device first (for example `sensor.left_arm_state`). Until a robot announces, there are no devices.

## 3. Install the ROS 2 package

### Get the code

Both routes need the repository: clone it (on Ubuntu, `sudo apt install git` first if `git` is missing), or unpack the release archive named in Option B as `~/ha-ros2-bridge`:

```bash
git clone https://github.com/jonnytinkererofthings/ha-ros2-bridge.git ~/ha-ros2-bridge
```

Then pick one route. Both run the `fake` executor by default.

### Native route (Ubuntu 24.04, ROS 2 Jazzy)

Steps 1 to 7 assume one shell; a new terminal needs the two `source` lines of step 7 first.

1. Add the ROS 2 apt source and key as in the official Jazzy guide for Debian packages (`https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html`), then ROS 2 and the build tools:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-base python3-colcon-common-extensions python3-rosdep python3-pip
sudo rosdep init   # once per machine; skip if it says it is already initialised
rosdep update
source /opt/ros/jazzy/setup.bash
```

2. Put the package in a workspace:

```bash
mkdir -p ~/bridge_ws/src
ln -s ~/ha-ros2-bridge/ros2/ros2_ha_bridge ~/bridge_ws/src/ros2_ha_bridge
```

3. Dependencies. ROS packages come from rosdep; paho-mqtt comes from pip, because Ubuntu's `python3-paho-mqtt` is 1.6.1 and the node refuses any paho-mqtt 1.x at import (`requirements.txt` pins `paho-mqtt==2.1.0`). On Ubuntu 24.04 pip refuses to install into the system Python unless told that this is intended (PEP 668), hence the flag:

```bash
cd ~/bridge_ws
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
python3 -m pip install --break-system-packages -r src/ros2_ha_bridge/requirements.txt
python3 -c "import paho.mqtt; print(paho.mqtt.__version__)"   # must print 2.1.0
```

4. Build and source:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/bridge_ws
colcon build --event-handlers console_direct+
source install/setup.bash
```

5. Parameters file. Copy the example and change what differs on your site:

```bash
cp ~/ha-ros2-bridge/ros2/ros2_ha_bridge/config/bridge.example.yaml ~/arm_left.yaml
```

Set at least these keys (nested keys become dotted parameter names: `robot: {id: ...}` is `robot.id`):

```yaml
/**:
  ros__parameters:
    robot:
      id: arm_left              # ^[a-z][a-z0-9_]{0,31}$; "ha" is reserved; unique per robot
      name: Left arm            # the device name shown in Home Assistant
    mqtt:
      host: broker.example.invalid
      port: 1883
      username: changeme        # empty means an anonymous connection
    topic_prefix: ros2_bridge   # must match the Home Assistant integration
    executor: fake              # the shipped fake executor; nothing moves
```

6. The node reads the broker password from the environment, never from the parameters file or a launch argument:

```bash
export ROS2_HA_BRIDGE_MQTT_PASSWORD='changeme'
```

7. Launch:

```bash
source /opt/ros/jazzy/setup.bash
source ~/bridge_ws/install/setup.bash
ros2 launch ros2_ha_bridge bridge.launch.py params_file:=$HOME/arm_left.yaml
```

The good line in the log is `connected to the MQTT broker as 'ros2_ha_bridge_arm_left'`. Within a few seconds the device "Left arm" appears under Settings, Devices and services, ROS 2 Bridge. One launch per robot, each with its own parameters file; `fleet.launch.py` starts several from one list. `docs/install-ros2.md`, step 7, lists the launch arguments (parameters file, node name, namespace, log level and the respawn policy) and the fleet file format.

### Docker route (any PC with Docker)

Run every command below from `tools/`, where `docker compose` finds `compose.yaml` and `.env` without flags.

1. Create `.env` from the example and fill in the broker: `MQTT_HOST`, `MQTT_PORT`, `MQTT_USERNAME`, `ROS2_HA_BRIDGE_MQTT_PASSWORD`; leave `ROS2_HA_BRIDGE_COMMAND_SECRET` empty and `TOPIC_PREFIX` at `ros2_bridge`. Git ignores the file.

```bash
cd ~/ha-ros2-bridge/tools
cp .env.example .env
```

2. Build the image and start two fake arms:

```bash
docker compose build
docker compose up -d arm1 arm2
docker compose logs -f arm1
```

Service `arm1` announces as robot `fake_arm_1` ("Fake arm 1") and `arm2` as `fake_arm_2` ("Fake arm 2"). Within a few seconds both appear as devices under Settings, Devices and services, ROS 2 Bridge. `docker compose down` removes the arms and the network.

Only the broker, the prefix and the two fake ids come from the environment (`.env`, and the per-service variables in `compose.yaml`). Any other parameter in `docs/configuration.md` (TLS, `watchdog.mode`, `mirror.topics`, `telemetry.joints_rate_hz`, `tasks.allow`) is set in one of two ways: an environment variable `ROS2_HA_BRIDGE_PARAM_<NAME>`, where `<NAME>` is the dotted parameter name upper-cased with each dot written as a double underscore (`ROS2_HA_BRIDGE_PARAM_WATCHDOG__MODE=inhibit_new`, `ROS2_HA_BRIDGE_PARAM_MQTT__TLS__ENABLED=true`; pass it with `-e` or add it to `.env` and the service in `compose.yaml`), or a parameters file mounted into the container and named with `ROS2_HA_BRIDGE_PARAMS_FILE`; `tools/README.md`, Run two fake arms, has both commands, and a TLS certificate named either way needs its own mount. The two secrets stay in their own variables and are refused by the generic one. The rig is for testing: it runs the fake executor only, and a container on its network cannot reach a `move_group`, so a robot machine uses the native route.

## 4. First task

Native route: read `arm_left` and `left_arm` wherever `fake_arm_1` appears below.

1. Developer tools, Events, "Listen to events": enter `ros2_bridge_task_result` and press Start listening.
2. Settings, Devices and services, ROS 2 Bridge, then the device: press the **Reset to home** button (`button.fake_arm_1_reset_to_home`). Leave **Hang** alone for now; it exists to exercise the timeout path and leaves the robot in Error after about 15 s until Clear error or Stop is pressed. Or send a parameterised task from Developer tools, Actions (YAML mode, response shown):

```yaml
action: ros2_bridge.send_task
data:
  robot_id: fake_arm_1
  task: move_to
  params:
    pose: home
  wait: result
```

`wait` is `ack` (default), `result` or `none`; `device_id` works in place of `robot_id`.

3. What to watch:

- **State** (`sensor.fake_arm_1_state`): Busy, with `task` and `task_id` attributes and feedback (`stage: executing`, `progress` rising), then Idle after 2 s (`fake.task_duration_s`).
- **Last result** (`sensor.fake_arm_1_last_result`): Succeeded, with `code: ok`, `duration_ms` near 2000 and `synthetic: false`.
- **The events**: one `ros2_bridge_task_ack` with `accepted: true`, then exactly one `ros2_bridge_task_result` with `status: succeeded` and `kind: motion`. The `event.fake_arm_1_task_result` entity triggers at the same moment, so an automation can use either.
- **Problem** (`binary_sensor.fake_arm_1_problem`) stays off.

4. Watch the broker itself from any machine with `mosquitto-clients` installed (`sudo apt install mosquitto-clients`):

```bash
mosquitto_sub -h broker.example.invalid -p 1883 -u changeme -P "$ROS2_HA_BRIDGE_MQTT_PASSWORD" -V 5 -v -t 'ros2_bridge/#'
```

A test convenience: the variable exported in step 6 keeps the password out of your shell history; from `tools/`, `docker compose run --rm mqtt` does the same with the `.env` credentials. You see the retained availability, announce and state at once, then the first `joints` message (0.2.0, the fake arm's home pose), then a heartbeat every 2 s, and on each command the ack, `busy`, feedback, `joints` while the arm moves, the result and `idle`.

### Joints and the dashboard

Each fake arm announces seven joints (`j1` to `j7`, degrees), so each device also gets `sensor.fake_arm_1_joint_1_j1` to `sensor.fake_arm_1_joint_7_j7`, updated at up to 2 Hz while a task moves them and holding between tasks; each carries its limits, the reported velocity (degrees per second for the fake arms, zero at rest) and a `moving` flag as attributes. `docs/examples/dashboard.yaml` is a ready-made dashboard for the two fake arms (an Overview plus one view per arm with the seven joint gauges, the task buttons, a history graph and the result log): Settings, Dashboards, Add dashboard, open its raw configuration editor and paste the file in; `docs/dashboard.md` explains it and how to adapt it to other robots. Joint sensors update often, so `docs/sensors.md` shows the recorder exclude to keep them out of long-term history.

## 5. How communication works

Three terms: retained, the broker keeps the last message on a topic and hands it to new subscribers; QoS 1, delivered at least once; Last Will, a message the broker publishes for a client that dies without disconnecting.

```text
 Home Assistant OS                 Mosquitto add-on (MQTT 5)             ROS 2 machine
 +------------------+   cmd (QoS 1, never retained)   -->   +---------------------------+
 | ros2_bridge      |   ha/heartbeat, ha/sensors/...  -->   | bridge_node, one per      |
 | rides HA's own   |  <--  announce, availability, state   |  robot: authorise, then   |
 | MQTT client      |  <--  heartbeat, task/{id}/ack,       |  run through the executor |
 +------------------+       task/{id}/feedback, .../result  |  plugin (fake, moveit)    |
   devices, actions,                 ^                      +-------------+-------------+
   events, intents    zigbee2mqtt/{name} from Zigbee2MQTT,                | ROS 2 (DDS, local)
                      read by both sides in parallel                      v
                                                             move_group / controllers / Isaac Sim
```

One MQTT broker and nothing else sits between the halves; neither imports the other, and the written contract (`docs/mqtt-contract.md`, schema 1) is what both are tested against. Every bridge topic starts with `{prefix}`, default `ros2_bridge`; the only topics outside it are Home Assistant's birth topic `homeassistant/status`, which makes the node re-announce, and whatever `mirror.topics` names (for example `zigbee2mqtt/...`). Payloads are JSON objects carrying `schema: 1`, with epoch-millisecond timestamps.

**Robot to Home Assistant.**

- `{prefix}/{robot_id}/announce`, retained: identity and task catalogue; Home Assistant builds the device and its buttons from it.
- `{prefix}/{robot_id}/availability`, retained: `online` on connect, `offline` as the Last Will, so a dead node is noticed without polling.
- `{prefix}/{robot_id}/state`, retained, on change: `idle`, `busy` or `error`.
- `{prefix}/{robot_id}/heartbeat` every 2 s with counters; Home Assistant answers with `{prefix}/ha/heartbeat` every 2 s, so each side knows the other is alive.

**Commands.**

- Home Assistant publishes on `{prefix}/{robot_id}/cmd` at QoS 1, never retained, with a UUID4 `id`, `issued_at` and `max_age_ms`.
- The node checks, in order: retained drop, size, UUID id, dedupe, schema, robot id, optional HMAC, freshness, allowlist and params, state gating, then the executor; a replayed id gets its stored ack back and never runs twice.
- One retained ack on `{prefix}/{robot_id}/task/{task_id}/ack` (expiry 120 s), then exactly one retained result on `.../task/{task_id}/result` (expiry 300 s).
- Feedback on `.../task/{task_id}/feedback`, QoS 0, at most 2 Hz, shown only as attributes of the state sensor.
- A cancel travels on the same `cmd` topic as `type: cancel`; its `accepted` ack is terminal (no result of its own), and the cancelled task reports `cancelled` under its own id.
- Home Assistant matches replies by id, fires `ros2_bridge_task_result` exactly once per command, and synthesises `timeout` or `lost` only when the robot truly never answers.

**What ROS 2 sees.** Commands never appear as ROS topics, actions or services. The node calls the executor plugin (a Python class named by the `executor` parameter) in-process on its main thread, and only the executor talks ROS: the MoveIt executor is an action client on `/move_action` to `move_group`. The node's own ROS surfaces are two topics, `~/mqtt_in` (`std_msgs/String` envelopes of mirrored MQTT messages; the `mirror.ros_topic` parameter) and the latched `~/ha_link` (`std_msgs/Bool`; `watchdog.ros_topic`); `~` is the node name (the `node_name` launch argument, `bridge_node` by default), so `/bridge_node/mqtt_in` by default.

**Sensors.** Two paths. Fast: the node subscribes to the Zigbee2MQTT topics in `mirror.topics` and republishes them unchanged on `~/mqtt_in`, in parallel with Home Assistant rather than through it. Normalised: Home Assistant republishes the entities in its "Mirrored entities" option under `{prefix}/ha/sensors/{domain}/{object_id}` in one canonical schema, whatever the sensor's radio.

**Safety.** Home Assistant is advisory; ROS 2 owns safety and authorises every command itself. `watchdog.mode` decides what losing Home Assistant's heartbeat does: `none` (default, informational only), `inhibit_new` (refuse new motion while the link is down) or `stop_and_hold` (also stop the active task once per link loss). Nothing on MQTT is an emergency stop. With the stock Mosquitto add-on every MQTT client that can log in can publish commands; switch the shared secret on both sides before a real arm (`docs/security.md`).

## 6. Can it be tested without a simulator or a real arm?

Yes. The `fake` executor is the default (`executor: fake`), runs natively, in the Docker rig or in CI, and is what the 0.1.0 end-to-end run used. Its catalogue:

| Task | Kind | What it exercises |
|---|---|---|
| `reset` | motion | Succeeds with `ok` after 2 s; the button path. |
| `move_to {pose}` | motion | A required parameter: `send_task` with `params`, and `data.pose` in the result. |
| `wait {seconds}` | motion | A long task with progress feedback; cancel, pause and resume. |
| `fail {code}` | motion | Aborts with a MoveIt failure class (`collision`, `no_trajectory`, `planning_failed` and so on); the failure branches of your automations. |
| `hang` | motion | Never finishes and ignores cancel: the 10 s timeout, the 5 s cancel grace, the forced `aborted/timeout` and the robot in `error`. |
| `raise_error`, `clear_error` | control | The `error` state and the problem sensor. |
| `stop` | control | Ends the running task `cancelled/preempted` and clears any error. |
| `pause`, `resume` | control | Freeze and continue a running task; Home Assistant's result deadline pauses with it. |

Two further steps before a simulator or a real arm:

1. `tools/e2e_checklist.md`, the manual end-to-end procedure (steps 0 to 13) against a live Home Assistant with the two fake arms: from the clock check through the chaos checks, a Home Assistant restart mid-task, the watchdog modes and voice.
2. The MoveIt executor, Tier 1 of `docs/simulator_checklist.md`: on any Jazzy machine, no GPU, it proves the MQTT to executor to `move_group` to error-code path and the negative tests before a simulator or an arm is available.
   - Install: `ros-jazzy-moveit`, a MoveIt configuration for an arm on `mock_components/GenericSystem` (ros2_control's fake hardware; the `moveit_resources` Panda is enough) and a running `move_group`; the bridge ships only the executor. Its one extra dependency, `moveit_msgs`, comes from `ROS2_HA_BRIDGE_WITH_MOVEIT=1 rosdep install --from-paths src --ignore-src -y --rosdistro jazzy` or `sudo apt install ros-jazzy-moveit-msgs`.
   - Configure: start from `config/moveit.example.yaml` (`executor: moveit`, `moveit.group_name`, `moveit.poses`). Section 5 of `docs/simulator_checklist.md` has the exact steps.
   - Caveat: experimental; unit-tested with a fake adapter, never yet run against a real `move_group`.

## 7. Where to go next

- `docs/configuration.md`: every node parameter with its default, the launch arguments, the environment variables and what is fixed on purpose.
- `docs/troubleshooting.md`: symptom to cause, rejection codes, repairs, diagnostics.
- `docs/assist.md`: voice control with Assist, the sentences and the replies.
- `docs/writing-an-executor.md`: the plugin interface for your own controller.
- `docs/security.md`: the authorisation model, HMAC signing, TLS and what to configure per site.
