# tools: the local ROS 2 rig

Everything here runs the ROS 2 half of ha-ros2-bridge in Docker on a development PC, with the fake executor standing in for the arms, against the broker Home Assistant already uses (interview decision Q14) or against a throwaway local broker for CI and offline work. Nothing in this folder touches a real arm.

| File | Purpose |
|---|---|
| `Dockerfile.ros2` | `ros:jazzy-ros-base` plus paho-mqtt 2.1.0 from pip, the ament linters, pytest and a `colcon build` of `ros2/ros2_ha_bridge`. Built from the repository root so `contract/` travels with the image for the contract tests. |
| `compose.yaml` | Services `arm1`, `arm2`, `test`, `clock-check`, `local-broker` (profile) and `mqtt` (profile), one named bridge network. |
| `entrypoint.sh` | Sources ROS 2 and the workspace, renders a parameters file from the environment (through `render_params.py`), then runs `launch`, `run`, `test`, `clock-check` or whatever command it is given. |
| `render_params.py` | The renderer: a mounted base file plus the environment overrides, typed from the node's own parameter table. Importable, so `ros2/ros2_ha_bridge/test/test_render_params.py` drives it without a container. |
| `.env.example` | Placeholders for the variables `compose.yaml` reads. Copy to `.env`, which git ignores. |
| `mosquitto/mosquitto.conf` | Anonymous Mosquitto for the `local-broker` profile. |
| `clock_check.py` | Measures the MQTT round trip and, when the HA integration is running, the clock offset against Home Assistant. |
| `latency_probe.py` | Measures command-to-ack latency (an unknown task name by default, so the executor is never consulted), the publish-to-PUBACK round trip and heartbeat period jitter; `--dry-run` prints the plan. |

The node, the launch files and `config/bridge.example.yaml` live in `ros2/ros2_ha_bridge/`; this folder only packages and drives them.

## Prerequisites

- Docker Engine or Docker Desktop with Compose v2 (`docker compose version`).
- A broker: the Mosquitto that Home Assistant uses (MQTT 5, reachable from this PC), or the `local-broker` profile below.
- `tools/.env`: copy `tools/.env.example` and fill it in. The file is ignored by git; never commit it. `MQTT_USERNAME` empty means anonymous, which is what the local broker expects.

All commands below are run from this folder (`tools/`), so `docker compose` finds `compose.yaml` and `.env` without flags.

## Build

```text
docker compose build
```

That is the same as `docker build -f tools/Dockerfile.ros2 -t ha-ros2-bridge:dev .` from the repository root. The first build pulls the Jazzy base image and takes several minutes; later builds reuse the apt, pip and rosdep layers unless `requirements.txt` or `package.xml` changed.

## Run two fake arms

```text
docker compose up arm1 arm2
```

Each container runs one `bridge_node` process with the fake executor, `arm1` as robot `fake_arm_1` and `arm2` as `fake_arm_2` (the `ROS2_HA_BRIDGE_ROBOT_ID` variable in `compose.yaml`). The node takes the broker host, port and user name from `.env` through the rendered parameters file, and the password from `ROS2_HA_BRIDGE_MQTT_PASSWORD` in its environment. Add `-d` to run in the background, `docker compose logs -f arm1` to follow one node, and `docker compose down` to stop and remove everything (the network included).

### Setting any other parameter

Only six node parameters have a named variable (`robot.id`, `robot.name`, `mqtt.host`, `mqtt.port`, `mqtt.username`, `topic_prefix`). Every other parameter in `docs/configuration.md` is set in one of two ways, which combine.

**A generic variable per parameter.** `ROS2_HA_BRIDGE_PARAM_<NAME>`, where `<NAME>` is the dotted parameter name in upper case with every dot written as a double underscore:

```text
docker compose run --rm -e ROS2_HA_BRIDGE_PARAM_WATCHDOG__MODE=inhibit_new -e ROS2_HA_BRIDGE_PARAM_TELEMETRY__JOINTS_RATE_HZ=5 arm1
```

`ROS2_HA_BRIDGE_PARAM_MQTT__TLS__ENABLED=true` sets `mqtt.tls.enabled`, `ROS2_HA_BRIDGE_PARAM_TASKS__ALLOW=reset,stop` sets `tasks.allow`, `ROS2_HA_BRIDGE_PARAM_DATA_DIR=/data` sets `data_dir`. The value is converted to the parameter's declared type from the node's own table: an integer parameter needs an integer (`30`), a double any number (`5` or `0.5`), a bool `true` or `false`, a string array a comma-separated list (`reset,stop`) or a YAML flow list (`[reset, stop]`), and a string parameter takes the text as it is. An unset or empty variable is no override. A name the node does not declare, a value of the wrong type, and `mqtt.password` or `command.secret` in this form (the node reads those from `ROS2_HA_BRIDGE_MQTT_PASSWORD` and `ROS2_HA_BRIDGE_COMMAND_SECRET`) stop the container before the node starts, with the variable named in the log. For a permanent setting, list the variable under the service's `environment:` in `compose.yaml` (there is a commented example under `arm1`); a line in `.env` alone is not passed into the container, because compose only reads `.env` for `${...}` substitution. The `moveit.*` scalars are typed too (`moveit.poses` as the path of a poses file; inline poses go in the mounted file), but the image is built without `moveit_msgs` (below).

**A mounted parameters file.** Mount it and name it with `ROS2_HA_BRIDGE_PARAMS_FILE` (the path inside the container):

```text
docker compose run --rm -v "$PWD/my_params.yaml:/params.yaml:ro" -e ROS2_HA_BRIDGE_PARAMS_FILE=/params.yaml -e ROS2_HA_BRIDGE_ROBOT_ID= -e ROS2_HA_BRIDGE_ROBOT_NAME= arm1
```

The environment overrides listed in the table below are applied on top of the mounted file. `MQTT_HOST`, `MQTT_PORT`, `MQTT_USERNAME` and `TOPIC_PREFIX` come from `.env`, so leave a variable empty there if the file should win. `ROS2_HA_BRIDGE_ROBOT_ID` and `ROS2_HA_BRIDGE_ROBOT_NAME` are set per service in `compose.yaml` (`fake_arm_1` and `Fake arm 1` for `arm1`), so to take the robot id and name from your file pass them empty on the command line as above (an empty variable means no override); without that, a file naming `robot.id: arm_left` silently runs as `fake_arm_1`. Any earlier spelling of an overridden parameter in the file (nested, dotted or mixed) is removed before the override is written, so an override always wins. `compose.yaml` has a commented `volumes:` block per arm for the file, a TLS certificate and the data directory; with it in place, set `ROS2_HA_BRIDGE_PARAMS_FILE=/params.yaml` in `.env` instead of passing `-e` and `-v` by hand.

### What the entrypoint reads

| Variable | Becomes | Set where |
|---|---|---|
| `ROS2_HA_BRIDGE_PARAMS_FILE` | the base parameters file the overrides are applied to (mount it into the container) | `.env` (passed through by `compose.yaml`) or `-e` on `docker compose run` |
| `ROS2_HA_BRIDGE_ROBOT_ID` | `robot.id` | `compose.yaml`, per service |
| `ROS2_HA_BRIDGE_ROBOT_NAME` | `robot.name` | `compose.yaml`, per service |
| `MQTT_HOST`, `MQTT_PORT`, `MQTT_USERNAME`, `TOPIC_PREFIX` | `mqtt.host`, `mqtt.port`, `mqtt.username`, `topic_prefix` | `.env` |
| `ROS2_HA_BRIDGE_PARAM_<NAME>` | the parameter `<NAME>` names (dots as double underscores), typed from the node's table | `-e` on `docker compose run`, or under a service's `environment:` |
| `ROS2_HA_BRIDGE_MQTT_PASSWORD`, `ROS2_HA_BRIDGE_COMMAND_SECRET` | read by the node itself; never written to the rendered file | `.env` |
| `ROS_DOMAIN_ID` | the DDS domain inside the containers (rig hygiene only) | `.env` |
| `RMW_IMPLEMENTATION`, `ROS_AUTOMATIC_DISCOVERY_RANGE` | the DDS isolation of the rig (`rmw_fastrtps_cpp`, `LOCALHOST`; see Networking) | `.env`, defaults in `compose.yaml` |
| `ROS2_HA_BRIDGE_RENDERED_PARAMS`, `ROS2_HA_BRIDGE_WS`, `ROS_DISTRO` | where the rendered file lands (`/run/ros2_ha_bridge/params.yaml`), the workspace (`/ws`) and the ROS 2 distribution sourced (`jazzy`); internals of the image | `Dockerfile.ros2` |

An empty or unset variable means no override. The rendered file is logged at start (`entrypoint: rendered ... with overrides ...`) with every override and its typed value.

### Three limits of the rig

- A TLS certificate named in a parameters file or a variable must be mounted too: `-v "$PWD/ca.pem:/certs/ca.pem:ro"` with `mqtt.tls.ca_cert: /certs/ca.pem` (or `ROS2_HA_BRIDGE_PARAM_MQTT__TLS__CA_CERT=/certs/ca.pem`); a certificate path that does not exist inside the container is a fatal start-up error.
- `data_dir` is inside the container (`/root/.ros/ros2_ha_bridge`, the node's default), so the dedupe store survives `docker compose restart` and stop/start but not `docker compose down` followed by `up`; the contract's protection against a QoS 1 resend across a node restart therefore does not hold across a recreate. Uncomment the named volume in `compose.yaml` if that matters for a test.
- The image is built without `moveit_msgs` (`Dockerfile.ros2` runs `rosdep` without `ROS2_HA_BRIDGE_WITH_MOVEIT`), so `executor: moveit` runs only on a native install (`docs/install-ros2.md`).

## Run the tests

```text
docker compose run --rm test
```

This runs `colcon test` on the package inside the image (pytest over `ros2/ros2_ha_bridge/test`, including the rclpy tests that skip on a machine without ROS, plus the ament linters once the package ships `test_flake8.py` and `test_pep257.py`) and then `colcon test-result --verbose`, whose exit code is the container's. Extra arguments go to `colcon test`, for example `docker compose run --rm test --pytest-args -k authoriser`.

## Watch the bus

The `mqtt` service is an `eclipse-mosquitto` client shell whose `$MQ` variable already holds the connection flags from `.env` (host, port, MQTT 5, and the credentials when a user name is set), so the password never appears on the host command line. Quote the argument with single quotes so your own shell leaves `$MQ` and `$TOPIC_PREFIX` alone.

```text
docker compose run --rm mqtt
docker compose run --rm mqtt 'mosquitto_sub $MQ -v -t "$TOPIC_PREFIX/fake_arm_1/#"'
```

The first form subscribes to everything under the prefix and prints retained availability, announce (with `instance`) and state at once, then the heartbeat every two seconds. A password containing spaces does not survive the unquoted `$MQ` expansion; use one without for this rig.

## Send a command by hand

A valid command needs a fresh UUID4 id and a current `issued_at`; the shell inside the `mqtt` service can make both:

```text
docker compose run --rm mqtt 'ID=$(cat /proc/sys/kernel/random/uuid); NOW=$(( $(date +%s) * 1000 )); mosquitto_pub $MQ -q 1 -D publish message-expiry-interval 5 -t "$TOPIC_PREFIX/fake_arm_1/cmd" -m "{\"schema\":1,\"id\":\"$ID\",\"type\":\"task\",\"robot_id\":\"fake_arm_1\",\"task\":\"reset\",\"params\":{},\"issued_at\":$NOW,\"max_age_ms\":5000,\"timeout_s\":null,\"source\":{\"origin\":\"tools\",\"kind\":\"manual\"}}"; echo "sent $ID"'
```

Within a second the retained ack appears on `{prefix}/fake_arm_1/task/<id>/ack`, the state goes `busy`, and after the fake task duration (2 s) the retained result and the `idle` state follow. Watch them with the subscription above. With `ROS2_HA_BRIDGE_COMMAND_SECRET` set on the node, an unsigned command like this one is answered `rejected_unauthorised`, which is the expected result of that check.

## Local broker profile (CI and offline)

```text
docker compose --profile local-broker up -d local-broker
```

Then set `MQTT_HOST=local-broker` and `MQTT_USERNAME=` (empty) in `.env`; every other service reaches the broker by that name on the compose network. The listener is anonymous and is not published on the host. `docker compose --profile local-broker down` removes it.

## Clock check

```text
docker compose run --rm clock-check
docker compose run --rm clock-check --threshold-s 0.5 --require-heartbeat
```

The probe publishes a few timestamped messages under `{prefix}/ha/probe/<client>` and reads them back to measure the broker round trip, then waits for the HA integration's heartbeat on `{prefix}/ha/heartbeat` and reports how far this machine's clock is from Home Assistant's. It exits 1 when the offset exceeds the threshold (1 s by default; the node tolerates 2 s of skew), 2 when the broker cannot be reached, 3 when `--require-heartbeat` is given and no heartbeat arrives. Before the HA integration exists (phase 3) only the round trip is reported. The container's clock is the Docker VM's clock; run `python3 tools/clock_check.py` natively on the robot machine for the number that matters there.

## Chaos checks

The design's chaos list, with the exact commands. Keep a `docker compose run --rm mqtt` subscription open in a second terminal while running them.

| Check | Command | What to expect |
|---|---|---|
| Kill (Last Will) | `docker compose kill arm1` | The broker publishes the retained `offline` on `{prefix}/fake_arm_1/availability` within a few seconds (the socket closes with the process; at most 1.5 x the 15 s keepalive). `docker compose up -d arm1` brings it back with a new `session` and republished announce and state. |
| Pause (connected but silent) | `docker compose pause arm1` then `docker compose unpause arm1` | The heartbeat stops at once while the TCP session stays up, so HA shows `problem` within about 6 s and makes the entities unavailable after 30 s; the broker drops the silent client after about 22 s and publishes the will. `unpause` clears it. |
| Broker restart | `docker compose --profile local-broker restart local-broker` | Both nodes log the disconnect, reconnect with paho's backoff, and republish availability, announce and state; nothing is lost for in-flight commands because replies are retained with expiry. (Local profile only; never restart the live Home Assistant broker for a test.) |
| Clock skew on the node (faketime) | `docker compose run --rm -e FAKETIME_DONT_FAKE_MONOTONIC=1 arm1 faketime '+30s' /ws/tools/entrypoint.sh launch` | The node believes it is 30 s ahead; every fresh command is answered `rejected_future` with `clock_skew_ms` around -30000, and `clock_check.py` against the live HA would report the same offset. `FAKETIME_DONT_FAKE_MONOTONIC` keeps DDS and the timers on the real monotonic clock. |
| Stale command (no faketime) | Send the command above with `NOW=$(( ($(date +%s) - 60) * 1000 ))` | `rejected_stale` with the measured skew; the same id sent again republishes that stored ack byte for byte. |
| Retained plant, node connected | `docker compose run --rm mqtt 'mosquitto_pub $MQ -q 1 -r -t "$TOPIC_PREFIX/fake_arm_1/cmd" -m "{\"schema\":1,\"id\":\"planted\"}"'` | Dropped at once (Retain As Published), `dropped_retained` increments in the next heartbeat, and the node clears the topic: `docker compose run --rm mqtt 'mosquitto_sub $MQ -v -t "$TOPIC_PREFIX/fake_arm_1/cmd" -W 3'` prints nothing. No ack, no crash. |
| Retained plant, node stopped | `docker compose stop arm1`, plant as above, `docker compose start arm1` | Dropped at the first connect; the topic is cleared and the counter increments. |
| Malicious id | Send the command above with `ID='x/#'` | Dropped before any topic is built: `dropped_invalid_id` increments, no ack is published and the node keeps heartbeating. |
| Takeover (duplicate robot id) | `docker compose run --rm -e ROS2_HA_BRIDGE_ROBOT_ID=fake_arm_1 arm2` while `arm1` runs | Both nodes connect with client id `ros2_ha_bridge_fake_arm_1`; the broker disconnects one with reason code 142 (`Session taken over`), or Mosquitto with reason code 0 within `mqtt.takeover_suspect_s` (10 s) of its connect, each logs the ERROR naming the other node, backs off, and the one that reaches `mqtt.takeover_exit_after` (3) takeovers within `mqtt.takeover_window_s` (ten minutes) exits with status 4. HA raises `duplicate_robot_id`. |
| Oversize payload | Send 17 KiB on the command topic, for example `-m "$(head -c 17408 /dev/zero \| tr '\0' x)"` | `dropped_oversize` increments; nothing else happens. |

After the checks, `docker compose down` removes the arms, the one-off containers and the network. Retained acks and results expire by themselves (120 s and 300 s); a retained `offline` is replaced the next time the node connects.

## Networking

`compose.yaml` creates the user-defined bridge network `ha-ros2-bridge` and puts every service on it; no service uses host networking. DDS traffic never leaves that network: the containers have no interface on the LAN, and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` keeps each node's discovery inside its own container, so the two fake arms never discover each other and nothing multicasts onto the PC's network. MQTT is the only traffic that leaves, to `MQTT_HOST`. `ROS_DOMAIN_ID` (42 by default) is there for hygiene only. `RMW_IMPLEMENTATION` (`rmw_fastrtps_cpp`) and `ROS_AUTOMATIC_DISCOVERY_RANGE` (`LOCALHOST`) can be changed from `.env` without editing `compose.yaml`, for someone who knows what they are doing; with the defaults, a container on this network can never reach a `move_group` or a controller outside it, which is the reason the rig runs the fake executor only. The image runs the node as root inside the container, which is fine for a disposable rig and not a pattern for a deployment; `docs/install-ros2.md` describes a native install. A clean stop (`docker compose stop`) gives the node `mqtt.offline_wait_s` (2 s) to get the retained `offline` acknowledged plus one second for the ROS executor; the services' `stop_grace_period` (15 s) covers both with room to spare.
