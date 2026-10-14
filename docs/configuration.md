# Configuration reference (ROS 2 node)

Every setting of `bridge_node` is a ROS 2 parameter, declared with a descriptor and a default, so `ros2 param describe` works on a running node and a parameters file (`docs/install-ros2.md`, step 5) overrides only what differs on your site. `config/bridge.example.yaml` in the package lists them all with these defaults. No default names a real host, user or secret; the two secrets come from environment variables.

Parameter names use dots. In a parameters file they are written as nested maps under `ros__parameters` (`mqtt: {host: ...}` gives `mqtt.host`), which is how rcl flattens them. Two rules of rcl's parameter files to know. `executor` is itself a string parameter, so its three siblings cannot be nested under it: write `executor.call_warn_s`, `executor.cancel_grace_s` and `executor.health_poll_s` as dotted keys at the same level, as `config/bridge.example.yaml` does (dotted and nested spellings name the same parameter, so `tasks.allow:` and `mirror: {topics: [...]}` are both fine). Types are strict at declaration: an integer parameter (`mqtt.port`, `reply.ack_expiry_s`, `command.dedupe_size`) must be written without a decimal point, while a double accepts either `2` or `2.0`.

The launch files add a few arguments of their own (Launch arguments, below), and the Docker rig in `tools/` can set any parameter from the environment (Environment variables, below).

## Robot identity

| Parameter | Default | Meaning |
|---|---|---|
| `robot.id` | none; required (the example file uses `fake_arm_1`) | The robot's id on the broker and in Home Assistant. Must match `^[a-z][a-z0-9_]{0,31}$`; `ha` is reserved and refused at start-up, and so is an empty id. Names every topic (`{prefix}/{robot.id}/...`), the MQTT client id and the data directory. One node process per id. |
| `robot.name` | `""` | The device name shown in Home Assistant; empty uses `robot.id`. |
| `robot.manufacturer` | `""` | Announce identity; empty is sent as `null`. |
| `robot.model` | `""` | Announce identity. |
| `robot.sw_version` | `""` | Announce identity (the driver or executor version, for instance). |
| `robot.hw_version` | `""` | Announce identity. |
| `robot.serial_number` | `""` | Announce identity. |
| `robot.suggested_area` | `""` | The Home Assistant area the device is suggested for. |

## MQTT connection

| Parameter | Default | Meaning |
|---|---|---|
| `mqtt.host` | `localhost` | Broker host name or address. |
| `mqtt.port` | `1883` | Broker port (8883 is the usual TLS port). |
| `mqtt.username` | `""` | Broker user name; empty connects anonymously. |
| `mqtt.password` | `""` | Broker password. Empty reads `ROS2_HA_BRIDGE_MQTT_PASSWORD` from the environment, which is the intended way. A value given here is read once, logged as a warning that names the variable, and undeclared at once, so it never stays readable through `ros2 param get`; it still sits in the params file. |
| `mqtt.client_id` | `ros2_ha_bridge_{robot.id}` | The MQTT client id. Keep it a function of `robot.id`: two nodes claiming one robot then collide visibly on the broker (session takeover, below) instead of silently sharing a robot. Empty (the declared default) derives it; a set value is accepted with a warning in the start-up log. |
| `mqtt.keepalive_s` | `15` | MQTT keepalive. The broker publishes the Last Will (`availability offline`) within about 1.5 times this after a node dies without disconnecting. |
| `mqtt.reconnect_min_s` | `1` | Shortest reconnect delay (paho's `reconnect_delay_set`). |
| `mqtt.reconnect_max_s` | `30` | Longest reconnect delay; the delay doubles per attempt up to this. |
| `mqtt.connect_timeout_s` | `0` | With a positive value the node exits non-zero if no CONNACK (accepted or refused) has arrived within this many seconds of start-up, so a supervisor can restart it; `0` keeps retrying for ever. |
| `mqtt.takeover_exit_after` | `3` | After this many session takeovers within `mqtt.takeover_window_s` the node exits with status 4, which lets the launch file's `respawn_max_retries` stop two nodes with the same `robot.id` from flapping for ever. A takeover is a broker-sent DISCONNECT with reason code 142 (`Session taken over`), or any broker-sent DISCONNECT within `mqtt.takeover_suspect_s` of a successful connect (next row). Each one doubles the reconnect delay and logs an ERROR of the form `MQTT session for client id 'ros2_ha_bridge_arm_left' was taken over by another connection (...): most likely a second bridge node is running with the same robot.id. Takeover 1 of 3 within 10 minutes; reconnecting after 2 s`. `0` disables the exit. |
| `mqtt.takeover_suspect_s` | `10.0` | The heuristic behind the previous row: Mosquitto 2.1 evicts the older holder of a duplicated client id with a plain broker-sent DISCONNECT (reason code 0), not 142, so a broker-initiated disconnect this soon after connecting counts as a takeover whatever its reason code. A broker that closes a freshly connected client for another reason (an administrator kick, EMQX refusing a subscribe with `deny_action = disconnect`) is counted the same way, so on such a broker widen it or set `0`, which leaves reason code 142 as the only signal. |
| `mqtt.takeover_window_s` | `600.0` | The sliding window, in seconds, the takeover count is measured over; the doubled reconnect delay is restored at the first connect or ordinary disconnect after a whole window without a takeover. Greater than 0. |
| `mqtt.max_payload_bytes` | `16384` | Inbound size limit; larger messages are dropped unparsed and counted as `dropped_oversize`. It bounds every inbound message of the node: commands, Home Assistant heartbeats and mirrored sensor messages alike. Home Assistant's own limit stays at the contract's 16 KiB (Fixed by the contract, below), so raising it buys nothing on the HA side; it exists for sites whose mirrored sensor payloads are larger than 16 KiB. |
| `mqtt.inbound_queue_max` | `256` | Bound of the queue between paho's network thread and the node's main thread; a full queue drops the newest message and counts `dropped_queue_full`. |
| `mqtt.offline_wait_s` | `2.0` | How long a clean stop waits for the retained `availability offline` to be acknowledged before relying on the Last Will; `0` does not wait. After the node's own shutdown the process gives the rclpy executor a further, fixed second to wind down, so any supervisor's stop grace (systemd `TimeoutStopSec`, compose `stop_grace_period`) must exceed `mqtt.offline_wait_s` plus one second. |

The connection is plain TCP, or TCP with TLS (below). MQTT over WebSockets (a broker behind a reverse proxy) is not supported; there is no transport parameter.

### TLS

TLS is off by default, which is fine on a wired LAN. Switch it on wherever the network is shared Wi-Fi or leaves the building.

| Parameter | Default | Meaning |
|---|---|---|
| `mqtt.tls.enabled` | `false` | Use TLS for the broker connection. Set `mqtt.port` to the broker's TLS port as well. |
| `mqtt.tls.ca_cert` | `""` | Path to the CA certificate (PEM) that signed the broker's certificate. Empty uses the system's trust store. |
| `mqtt.tls.certfile` | `""` | Path to a client certificate, for brokers that authenticate clients by certificate. |
| `mqtt.tls.keyfile` | `""` | Path to the client certificate's private key. |
| `mqtt.tls.insecure` | `false` | Skip hostname verification of the broker certificate. For testing against a self-signed certificate only; it does not disable encryption. |

A certificate file that does not exist is a fatal start-up error (the node refuses to start rather than fall back to plain TCP).

## Topics

| Parameter | Default | Meaning |
|---|---|---|
| `topic_prefix` | `ros2_bridge` | The prefix of every topic. Validated like the Home Assistant side: no leading or trailing slash, no `+` or `#`, not `homeassistant` and not starting with the reserved segment `ha`. Must match the integration's prefix. |
| `ha_status_topic` | `homeassistant/status` | Home Assistant's birth and will topic. The node re-announces when `online` appears there. |
| `announce.jitter_s` | `2.0` | The re-announce after Home Assistant's birth message is delayed by a random 0 to this many seconds, so a fleet does not announce in one burst. |

## Logging

| Parameter | Default | Meaning |
|---|---|---|
| `logging.warn_interval_s` | `10.0` | Least spacing of the rate-limited WARN lines: an oversize message, a full inbound queue, a retained command, a command with an unusable id, an unauthorised command. The next line of the same kind says how many were suppressed in between. `0` warns on every event. The ROS log level itself is the `log_level` launch argument. |

## Executor

| Parameter | Default | Meaning |
|---|---|---|
| `executor` | `fake` | Which `TaskExecutor` to load: an entry-point name in the `ros2_ha_bridge.executors` group, or `package.module:Class`. Unknown or of the wrong type is a fatal start-up error. `fake` moves nothing. |
| `executor.call_warn_s` | `0.1` | A plugin call (`start`, `cancel`, `stop_and_hold`) longer than this logs a warning and marks `executor_health` degraded. |
| `executor.cancel_grace_s` | `5.0` | How long a cancel may take to produce a result before the node forces `aborted` (`timeout` or `executor_unresponsive`) and puts the robot into `error`. |
| `executor.health_poll_s` | `1.0` | How often `health()` is polled; a change is published in the state. |
| `tasks.allow` | `[]` | Allowlist of task names. Empty means every task the executor offers; otherwise only the listed names are announced to Home Assistant and accepted in commands (the announced catalogue is the intersection). |

## Commands

| Parameter | Default | Meaning |
|---|---|---|
| `command.secret` | `""` | The shared HMAC secret. Empty reads `ROS2_HA_BRIDGE_COMMAND_SECRET`; empty after both means signing is off and unsigned commands are accepted (Q15, off by default). With a secret, a missing or wrong `sig` is answered `rejected_unauthorised`, and the announce says `hmac_required: true`. As with `mqtt.password`, a value given here is warned about and undeclared once read. |
| `command.max_age_cap_ms` | `30000` | Upper bound on a command's own `max_age_ms`; the acceptance window is `min(max_age_ms, cap)`. |
| `command.clock_skew_tolerance_ms` | `2000` | How far the sender's clock may be ahead of or behind the node's before a command is `rejected_future` or `rejected_stale`. Needs NTP on both machines; `tools/clock_check.py` measures the real offset. |
| `command.dedupe_window_s` | `300` | How long a command id is remembered so that a redelivery republishes the stored ack instead of running the task again. Keep it at least `reply.ack_expiry_s`. |
| `command.dedupe_size` | `1024` | How many ids the dedupe store keeps at most. |
| `command.dedupe_write_interval_s` | `1.0` | Least spacing between two rewrites of `dedupe.json` (accepted ids are coalesced into one write at the end of the interval; a clean stop flushes what is pending). It is also the width of the window in which a crash can lose a newly accepted id. `0` writes at every accepted ack; flash storage may want longer. |
| `command.task_timeout_cap_s` | `600` | Upper bound on any task timeout, whether announced by the executor or requested by a command; the catalogue is announced with the capped values. |
| `data_dir` | `~/.ros/ros2_ha_bridge` | The directory under which `{robot.id}/dedupe.json` holds the accepted command ids persisted across a node restart (`~` is expanded). Empty keeps the dedupe store in memory only. A directory that cannot be created or written logs one warning and the node runs memory-only. In the Docker rig this directory is inside the container unless a volume is mounted (`tools/README.md`, Three limits of the rig). |

## Replies

| Parameter | Default | Meaning |
|---|---|---|
| `reply.ack_expiry_s` | `120` | Message Expiry Interval of the retained ack. |
| `reply.result_expiry_s` | `300` | Message Expiry Interval of the retained result. |
| `reply.feedback_max_rate_hz` | `2` | Per-task throttle on feedback messages; `0` disables throttling. |
| `reply.retained_rejections_per_minute` | `30` | Rejected acks beyond this rate are published without the retain flag, so a misfiring automation cannot fill the broker's retained store. |

## Heartbeat and watchdog

| Parameter | Default | Meaning |
|---|---|---|
| `heartbeat.interval_s` | `2` | Period of the node's heartbeat (`{prefix}/{robot.id}/heartbeat`), which carries the counters and `ha_clock_offset_ms`. `0` disables it, and Home Assistant then does no staleness tracking for this robot. |
| `watchdog.mode` | `none` | What losing Home Assistant's heartbeat does; see below. |
| `watchdog.timeout_s` | `5.0` | Seconds without a Home Assistant heartbeat before the link counts as down. Clamped to at least 2.5 times the interval Home Assistant announces, so a slow heartbeat never trips the watchdog by itself. The watchdog evaluates on a fixed 0.5 s tick, so detection latency is the timeout plus up to 0.5 s. |
| `watchdog.ros_topic` | `~/ha_link` | The latched `std_msgs/Bool` ROS topic that mirrors the HA link (`true` while up); `~` is the node's own namespace, so the default resolves to `/bridge_node/ha_link` under the default node name. Non-empty; a name rclpy refuses is a start-up error. Its QoS (keep last 1, reliable, transient local, so a late subscriber gets the current value at once) is the latched contract of the topic and is not a parameter. |

## Joint telemetry (0.2.0)

Read only when the executor announces joints (`TaskExecutor.joints()` returns a `JointInfo`; the fake executor does, with seven joints in degrees, and the MoveIt executor does for the group's joints in radians). The positions travel on `{prefix}/{robot.id}/joints` at QoS 0, never retained, with a 5 s Message Expiry Interval, and Home Assistant shows them as one sensor per joint (`docs/sensors.md` has the recorder exclude for installations that do not want them in history). Velocities travel in the same message when the executor reports them, in the announced unit per second: the fake arm reports the derivative of its interpolation (constant per joint while a motion runs, zero once it holds, is paused or has arrived) and the MoveIt executor forwards the `velocity` field of `joint_states` when the message carries one.

| Parameter | Default | Meaning |
|---|---|---|
| `telemetry.joints_rate_hz` | `2.0` | The most `joints` messages per second. `0` switches joint telemetry off: nothing is published, the announce omits its `joints` block and Home Assistant creates no joint sensors (and removes any it had). |
| `telemetry.joints_min_delta` | `0.0` | The least movement of any one joint, in the announced unit, that publishes a new message. `0` (the default) means the unit's own default: `0.5` for `deg`, `0.01` for `rad`; it cannot mean "every change", so set a tiny positive value such as `0.001` for that (the start-up log names the effective delta). A (re)connect and every announce publish the latest positions once whatever the movement, and a report that arrives inside the rate interval is held and published when the interval has passed, so the final pose of a motion always goes out. |

The executor reports as often as its driver does; the node drops what the two rules above do not need, so a 500 Hz `joint_states` stream costs two messages a second on the broker. `seq` increments per publication within a session and Home Assistant drops a regressing copy.

### Watchdog modes

Home Assistant publishes `{prefix}/ha/heartbeat` every two seconds while its MQTT client is connected. The node tracks it, exposes the result as `ha_link` (`up` or `down`) in its state and on the latched ROS topic `watchdog.ros_topic` (`~/ha_link`), and applies `watchdog.mode`:

| Mode | Behaviour |
|---|---|
| `none` (default) | Informational only. The link is recorded and published, nothing else changes. This is the interview decision (Q16): Home Assistant going away must not affect the arms. |
| `inhibit_new` | While the link is down, `inhibited` is set and new motion tasks are refused with `rejected_inhibited`; running tasks continue, and control tasks (stop, pause, resume) are still accepted. The link counts as down from start-up until the first heartbeat. |
| `stop_and_hold` | As `inhibit_new`, and additionally, once per link loss, the executor's `stop_and_hold` is called and the active motion task ends with `cancelled/inhibited`. Fires only on a transition from up to down, never at start-up. The robot does not enter `error`; `inhibited` is what keeps new motion out until the heartbeat returns. |

Nothing on MQTT is an emergency stop. The modes exist for adopters whose safety case wants Home Assistant in the loop; the physical e-stop and the controller's protective functions remain the robot's own (`SECURITY.md`).

## Sensor mirror (fast path)

| Parameter | Default | Meaning |
|---|---|---|
| `mirror.topics` | `[]` | MQTT topics or filters (`zigbee2mqtt/kitchen_motion`, `zigbee2mqtt/+`, `ros2_bridge/ha/sensors/#`) whose messages are republished unchanged, as JSON envelopes, on the ROS topic below. A filter that MQTT would refuse is a fatal start-up error. |
| `mirror.availability_suffix` | `/availability` | Each entry's availability sibling is subscribed too and feeds `source_available` in the envelope. Empty disables availability tracking. |
| `mirror.ros_topic` | `~/mqtt_in` | The `std_msgs/String` topic the envelopes go out on (`~` is the node's namespace). |
| `mirror.ros_queue_depth` | `10` | Keep-last history depth of that publisher, the only buffer between a chatty `#` filter and a slow ROS subscriber; DDS drops beyond it with nothing logged or counted on the node. At least 1. |

## Fake executor

| Parameter | Default | Meaning |
|---|---|---|
| `fake.task_duration_s` | `2.0` | How long `reset`, `move_to` and `fail` take, which is also how long the seven simulated joints take to reach the target pose. |
| `fake.feedback_hz` | `2` | Feedback rate of a running fake motion task; `0` disables feedback. The joints are reported on the same ticks (and at the end of every motion), so `0` also stops the joints moving on the dashboard until the task ends. |

The fake's joints are `j1` to `j7` in degrees, limited to [-170, 170] except `j4` and `j6` at [-120, 120]; the named poses `move_to` accepts are `home`, `pick`, `place` and `wave`, and the arm starts at `home`. There is nothing to configure about them.

## MoveIt executor (experimental)

Read by `executor: moveit` (`ros2_ha_bridge/executors/moveit.py`), which has been unit-tested with a fake adapter only and not yet run against a real `move_group`. `config/moveit.example.yaml` is a complete parameters file. The executor needs `moveit_msgs`, which `package.xml` declares only under the condition `ROS2_HA_BRIDGE_WITH_MOVEIT == 1` (`docs/install-ros2.md`, step 3).

| Parameter | Default | Meaning |
|---|---|---|
| `moveit.group_name` | required | The planning group (`ur_manipulator`, `panda_arm`). |
| `moveit.poses` | required | Pose name to `{joint_name: position}` (inline, or flattened by rcl to `moveit.poses.<pose>.<joint>`), or the path of a YAML file holding that mapping, optionally under a `poses` key. At least one pose. |
| `moveit.planning_time_s` | `5.0` | `allowed_planning_time`. |
| `moveit.num_attempts` | `1` | `num_planning_attempts`. |
| `moveit.velocity_scale`, `moveit.acceleration_scale` | `0.1`, `0.1` | The two scaling factors, in (0, 1]. |
| `moveit.action_name` | `/move_action` | The MoveGroup action. |
| `moveit.joint_tolerance_rad` | `0.01` | `tolerance_above` and `tolerance_below` of every `JointConstraint` sent to move_group, in radians; greater than 0. Loosen it for a planner that cannot settle inside a hundredth of a radian, tighten it for one that can. |
| `moveit.motion_timeout_s` | `120.0` | The timeout announced for `move_to` and `home`. Home Assistant's generated buttons send no timeout of their own, so this is the value a button's result deadline is planned on; at a low `moveit.velocity_scale` a long move can need more. Greater than 0; the node clamps it to `command.task_timeout_cap_s`. |
| `moveit.control_timeout_s` | `5.0` | The timeout announced for `stop` and `refresh_poses`. Greater than 0; clamped the same way. |
| `moveit.hold_on_cancel` | `true` | Whether a cancelled task finishes, and the `stop` task clears a robot error, only once move_group confirms the goal ended. |
| `moveit.joint_states_topic` | `/joint_states` | The `sensor_msgs/JointState` topic the joint telemetry (0.2.0) is read from, with the sensor-data QoS (best effort). |
| `moveit.joint_names` | `[]` | The joints reported to Home Assistant, in order, in radians and without limits. Empty means the joints of the first configured pose in their written order. Names are announced lower-cased with other characters replaced by `_`; if two names collapse into one, or a name has nothing left, the telemetry is switched off with a warning. |

These are read from `ctx.parameters` when present and otherwise declared by the executor on the node; see `docs/writing-an-executor.md`, Parameters.

## Launch arguments

Not node parameters: these belong to the two launch files (`docs/install-ros2.md`, step 7, shows them in use and describes the fleet file).

`bridge.launch.py`:

| Argument | Default | Meaning |
|---|---|---|
| `params_file` | the package's `config/bridge.example.yaml` (robot `fake_arm_1`, broker `localhost`) | The parameters file for the node. |
| `node_name` | `bridge_node` | The ROS node name. It is the `~` of `mirror.ros_topic` (`~/mqtt_in`) and `watchdog.ros_topic` (`~/ha_link`), so give each node that shares a DDS domain its own name. |
| `namespace` | empty | The ROS namespace the node runs in, under which `~` resolves too; empty means none. |
| `log_level` | `info` | The ROS log level (`debug` for more). |
| `respawn` | `true` | Whether an exited node is started again. |
| `respawn_delay` | `2.0` | Seconds to wait before a respawn; at least 0. |
| `respawn_max_retries` | `3` | How many respawns before the launch gives up; `-1` means without limit, as launch_ros defines it. The limit exists so that a node that exits on purpose (session taken over by a duplicate robot id, no CONNACK within `mqtt.connect_timeout_s`) does not flap for ever; a supervisor that counts restarts itself (systemd `Restart=`) runs with `respawn:=false`. |

`fleet.launch.py` takes `fleet_file` (default: the package's `config/fleet.example.yaml`, two fake arms), `log_level` and the same three respawn arguments as the policy of every node; each entry of the fleet file may set `namespace` and override any of `respawn`, `respawn_delay` and `respawn_max_retries` for its own node, and its key is the node name.

## Environment variables

| Variable | Read when | Meaning |
|---|---|---|
| `ROS2_HA_BRIDGE_MQTT_PASSWORD` | `mqtt.password` is empty | The broker password. |
| `ROS2_HA_BRIDGE_COMMAND_SECRET` | `command.secret` is empty | The shared HMAC secret; empty keeps signing off. |

These are the only two the node reads. The launch files pass them through from the environment they were started in and never put them on a command line. Prefer them to the matching parameters: a secret in a params file is readable by whoever can read the file, and a declared parameter is readable by any participant on the DDS domain, which is why the node undeclares the two secret parameters the moment it has read them and logs a warning when it had to.

The Docker rig in `tools/` additionally reads `MQTT_HOST`, `MQTT_PORT`, `MQTT_USERNAME`, `TOPIC_PREFIX`, `ROS2_HA_BRIDGE_ROBOT_ID` and `ROS2_HA_BRIDGE_ROBOT_NAME` to render a parameters file for the container, `ROS2_HA_BRIDGE_PARAMS_FILE` to name a mounted file to start from, and `ROS2_HA_BRIDGE_PARAM_<NAME>` for any other parameter on this page (`moveit.poses` as the path of a poses file; inline poses go in the mounted file), where `<NAME>` is the dotted name in upper case with dots as double underscores (`ROS2_HA_BRIDGE_PARAM_WATCHDOG__MODE=inhibit_new`, `ROS2_HA_BRIDGE_PARAM_MQTT__TLS__ENABLED=true`), typed from the node's own table; the two secrets are refused in that form. `ROS2_HA_BRIDGE_RENDERED_PARAMS` (default `/run/ros2_ha_bridge/params.yaml`), `ROS2_HA_BRIDGE_WS` (`/ws`) and `ROS_DISTRO` (`jazzy`) are internals of the image. `tools/README.md`, What the entrypoint reads, has the table and the rules; those are a convenience of the rig's entrypoint, not of the node.

## Fixed by the contract

These are not parameters. They are either contract-level values both halves assume, bounds that protect the broker and Home Assistant, or the shipped fake's test fixture; raising an issue is the route to make one tunable. The node-side values are listed here; the Home Assistant integration's own fixed values are documented with the integration.

| Value | Where | Why fixed |
|---|---|---|
| Payload limits: 16 KiB per message, 64 KiB per announce | `protocol.MAX_PAYLOAD_BYTES`, `MAX_ANNOUNCE_BYTES`; the announce size check at start-up | The contract's size limits (`docs/mqtt-contract.md`). `mqtt.max_payload_bytes` is the node's inbound check; Home Assistant drops anything over 16 KiB regardless, and an announce over 64 KiB is a start-up error (shorten `tasks.allow`). |
| Result `data` cap 8 KiB (8192 bytes) and 512 characters for an unserialisable value's `repr` | `task_manager` | Half the payload limit, leaving room for the other result fields. |
| Message Expiry Intervals: heartbeat 5 s, joints 5 s, feedback 10 s | `protocol.HEARTBEAT_EXPIRY_S`, `JOINTS_EXPIRY_S`, `FEEDBACK_EXPIRY_S` | The contract's topic table; both halves decide staleness from them. Ack and result expiry are parameters (`reply.*`) because the table names them so. |
| Per-topic QoS and retain: `cmd` subscribed at QoS 1 with Retain As Published; HA heartbeat, HA status and mirror subscriptions at QoS 0; announce, availability and state published QoS 1 retained; heartbeat, joints and feedback QoS 0 non-retained; acks and results QoS 1 retained | `bridge_node`, `sensor_mirror.MIRROR_QOS` | The contract's topic table. |
| MQTT 5 with `clean_start`, the Last Will on the availability topic, reason code 142 as the takeover signal, paho 2.x | `mqtt_link` | Contract and dependency requirements (`docs/mqtt-contract.md`, `docs/install-ros2.md` step 3). |
| The wire vocabulary: schema 1, the id and name patterns, the reserved robot id `ha`, the discovery prefix `homeassistant`, 32 joints at most, the goal status mapping, `capabilities.cancel` and `capabilities.feedback` true | `protocol` | Contract definitions; changing any is schema 2. |
| Watchdog tick 0.5 s and the floor of 2.5 heartbeat intervals on `watchdog.timeout_s` | `watchdog.TICK_S`, `MIN_TIMEOUT_FACTOR` | Design (`docs/failure-modes.md`, `docs/architecture.md`); the tick bounds detection latency, the floor keeps a slow HA heartbeat from tripping the watchdog by itself. |
| The 60 s window behind `reply.retained_rejections_per_minute` | `authoriser` | The definition of "per minute". |
| `dedupe.json`, file version 1, and the atomic temp-file-plus-replace rewrite | `dedupe_store` | The persistence format (`docs/failure-modes.md`); the write pacing is `command.dedupe_write_interval_s`. |
| The warn-once memory of the sensor mirror: 1000 topics at most under a `#` filter, and the envelope keys | `sensor_mirror` | A bound on memory; the envelope is the ROS-side contract of the fast path (`docs/architecture.md`). |
| Node name `bridge_node` under `ros2 run`, and the latched QoS of `watchdog.ros_topic` | `bridge_node` | The name is the `node_name` launch argument (or `--ros-args -r __node:=`); the QoS is what makes the topic latched. |
| One second for the rclpy executor to wind down after the node's own shutdown | `bridge_node.main` | Fixed; documented with `mqtt.offline_wait_s` so a supervisor's stop grace covers both. |
| The fake executor's joints `j1` to `j7` in degrees with their limits, the poses `home`, `pick`, `place` and `wave`, `fail` covering half the way to `pick`, `hang` at 10 s, the catalogue timeouts (60, 120, 600, 10 and 5 s) and `wait` capped at 600 s | `executors/fake.py` | A deterministic test fixture that must match the contract's golden announce and the shipped dashboard. |
| MoveIt executor: the pose name `home` that earns its own task, `JointConstraint` weight 1.0, `plan_only` false, the best-effort QoS on `joint_states`, the goal status values 4, 5 and 6 and the `MoveItErrorCodes` mapping | `executors/moveit.py` | Design (`docs/writing-an-executor.md`); the tolerances and the announced timeouts are parameters (`moveit.*`). |
| The two secrets travel only as environment variables, never as launch arguments, rig overrides or parameters in a file | the launch files, `tools/entrypoint.sh` | A security decision (`docs/security.md`); the matching parameters still exist for a params file. |
| Every bridge timer on one steady clock, one-shots over periodic timers, the `[name] message` log format | `ros_adapters` | Design decisions recorded in the module and `docs/architecture.md`. |
| Process exit statuses 0, 2, 3 and 4 | `bridge_node` | What a supervisor acts on (`docs/install-ros2.md`, Exit statuses). |

## Fixed values (Home Assistant side)

The integration's counterpart of the table above: not options, because they are either contract-level values both halves assume or bounds that protect the event loop, the Store and the broker. Raising an issue is the route to make one tunable. Everything else on the Home Assistant side is an option (`docs/install-ha.md`, section 3).

| Value | Where | Why fixed |
|---|---|---|
| The same payload limits: 16 KiB per message, 64 KiB per announce | `const.MAX_PAYLOAD_BYTES`, `MAX_ANNOUNCE_BYTES` | The contract's limits, applied to the bytes received. The node's `mqtt.max_payload_bytes` is tunable for the mirrored sensor payloads it accepts; the Home Assistant half stays a hard bound because the integration never needs larger messages and `tasks.allow` keeps an announce small. |
| HA heartbeat Message Expiry Interval 5 s | `HA_HEARTBEAT_EXPIRY_S` | The contract's topic table, like the node's three above. It is shorter than the 10 s maximum of the heartbeat interval option, which is harmless: the message is QoS 0 and non-retained and the node's watchdog clamps on `interval_ms`. |
| Per-topic QoS, retain and expiry: announce, availability, state, heartbeat, ack and result subscribed at QoS 1, feedback and joints at QoS 0; `cmd` published QoS 1, never retained, expiry `ceil(max_age_ms / 1000)`; sensor republish QoS 0 retained without expiry; empty retained QoS 1 tombstones | `hub`, `sensors_out` | The contract's topic table. |
| Ready timeout 10 s | `READY_TIMEOUT_S` | How long the hub waits for its acknowledgement and result subscriptions to be confirmed before it logs a warning and continues; commands raise `not_ready` until then. A bound on a start-up condition, never reached on a working broker. |
| Housekeeping tick 1 s, with the 1 s button slack and the 2 s action slack | `HOUSEKEEPING_INTERVAL_S`, `button.ACK_WAIT_SLACK_S`, `services.WAIT_SLACK_S` | The tick is the resolution at which every timeout is decided; the two slacks exist only so the registry's own timer fires before a waiting button press or action gives up, so they must stay at least one tick. |
| Diagnostics show the 50 most recent commands; up to 500 terminal commands stay in memory for late acknowledgements | `DIAGNOSTICS_RECENT_COMMANDS`, `MAX_TERMINAL_COMMANDS` | Memory bounds. A terminal command is also dropped once the result expiry plus the margin has passed, so 500 is reached only by a fleet busier than any the integration is sized for. |
| Store write delay 5 s, and the Store's key and version | `STORE_DELAY_SAVE_S`, `STORAGE_KEY`, `STORAGE_VERSION` | Debounce of housekeeping-only Store writes (transitions are written at once); the persistence identity is versioned and a change needs a migration (`docs/upgrading.md`). |
| 32 joints at most | `MAX_JOINTS` | The contract schema's maximum (`common.schema.json`), not an integration choice. |
| The `moving` rule of the joint sensors: strict `>` against the threshold, and reported velocities decide alone | `hub._joint_moving` | The strict comparison is the node's own `telemetry.joints_min_delta` rule, so the two defaults agree; a payload that carries velocities is the robot's own statement of rest or motion, and the position delta (a distance per publication, not a speed) decides only for a robot that sends none. The threshold and the hold themselves are options (`docs/install-ha.md`, section 3). |
| The `moving` rule of the joint sensors: strict `>` against the threshold, and reported velocities decide alone | `hub._joint_moving` | The strict comparison is the node's own `telemetry.joints_min_delta` rule, so the two defaults agree; a payload that carries velocities is the robot's own statement of rest or motion, and the position delta (a distance per publication, not a speed) decides only for a robot that sends none. The threshold and the hold themselves are options (`docs/install-ha.md`, section 3). |
| Instance-window floor 2 s in the duplicate robot id heuristic | `hub._note_identity` | Two different `instance` values within one announced heartbeat interval open the repair; the interval is floored at 2 s so a robot that announces no heartbeat (`heartbeat.interval_s` 0) is still checked. |
| Repair severities and the diagnostics redaction set | `hub._open_issue`, `diagnostics.TO_REDACT` | `hmac_rejected` and `duplicate_robot_id` are errors, the rest warnings; the shared secret and user ids are always redacted. Design and privacy choices, not site settings. |
| The HA-synthesised codes `no_ack`, `no_result`, `node_restarted` and `ha_restarted`, and the outcome vocabulary | `const` | Fixed by the contract's reply routing and by automations written against the bus events. |
| MQTT discovery only for the default prefix; one config entry per Home Assistant | `manifest.json` | A manifest is static, so discovery watches `ros2_bridge/+/announce` only; the hub, the Store and the three actions are singletons by design (`docs/install-ha.md`, section 2). |
| Entity naming from the announce, with renames in the registry | `entity.py`, `strings.json` | Home Assistant fixes entity ids at creation; renaming devices and entities in the registry is the supported path. A relabelled task keeps its button name until the entry is reloaded. |

## Not parameters

- `hold_on_startup` (calling the executor's `stop_and_hold` once at start-up) is an open question in `docs/decisions.md` and does not exist yet; the proposed default is off.
- Every setting of the Home Assistant side belongs to the integration's config and options flow and is documented with it (`docs/install-ha.md`, section 3): `topic_prefix`, `shared_secret`, `heartbeat_interval_s`, `ack_timeout_s`, `command_max_age_ms`, `result_timeout_margin_s`, `max_pending_per_robot`, `heartbeat_unavailable_after_s`, `intent_ack_wait_s`, `joint_moving_threshold`, `joint_moving_hold_s`, `mirror_entities` and `mirror_extra_attributes`, and under Advanced `ack_expiry_s`, `result_expiry_s`, `replay_settle_s`, `session_change_grace_s`, `feedback_throttle_ms`, `heartbeat_stale_intervals`, `clock_skew_warn_ms`, `duplicate_session_changes`, `duplicate_session_window_s` and `duplicate_quiet_s`.

## The Home Assistant side

The integration inherits the broker host, port, credentials and TLS settings from Home Assistant's own MQTT integration, so broker details are entered once, in Home Assistant, and the integration itself asks only for the topic prefix and the optional shared secret; the install steps and every option are in `docs/install-ha.md`. Four of its options must agree with node parameters on every robot: `ack_expiry_s` with `reply.ack_expiry_s`, `result_expiry_s` with `reply.result_expiry_s`, `command_max_age_ms` at or below `command.max_age_cap_ms`, and `clock_skew_warn_ms` below `command.clock_skew_tolerance_ms`.
