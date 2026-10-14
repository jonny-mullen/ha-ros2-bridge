# Configuration reference (ROS 2 node)

Every setting of `bridge_node` is a ROS 2 parameter, declared with a descriptor and a default, so `ros2 param describe` works on a running node and a parameters file (`docs/install-ros2.md`, step 5) overrides only what differs on your site. `config/bridge.example.yaml` in the package lists them all with these defaults. No default names a real host, user or secret; the two secrets come from environment variables.

Parameter names use dots. In a parameters file they are written as nested maps under `ros__parameters` (`mqtt: {host: ...}` gives `mqtt.host`), which is how rcl flattens them.

## Robot identity

| Parameter | Default | Meaning |
|---|---|---|
| `robot.id` | `fake_arm` (in the example file) | The robot's id on the broker and in Home Assistant. Must match `^[a-z][a-z0-9_]{0,31}$`; `ha` is reserved and refused at start-up. Names every topic (`{prefix}/{robot.id}/...`), the MQTT client id and the data directory. One node process per id. |
| `robot.name` | `""` | The device name shown in Home Assistant. |
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
| `mqtt.client_id` | `ros2_ha_bridge_{robot.id}` | The MQTT client id. Keep it a function of `robot.id`: two nodes claiming one robot then collide visibly on the broker (session takeover, below) instead of silently sharing a robot. |
| `mqtt.keepalive_s` | `15` | MQTT keepalive. The broker publishes the Last Will (`availability offline`) within about 1.5 times this after a node dies without disconnecting. |
| `mqtt.reconnect_min_s` | `1` | Shortest reconnect delay (paho's `reconnect_delay_set`). |
| `mqtt.reconnect_max_s` | `30` | Longest reconnect delay; the delay doubles per attempt up to this. |
| `mqtt.connect_timeout_s` | `0` | With a positive value the node exits non-zero if no CONNACK (accepted or refused) has arrived within this many seconds of start-up, so a supervisor can restart it; `0` keeps retrying for ever. |
| `mqtt.takeover_exit_after` | `3` | After this many `Session taken over` disconnects (reason code 142) within ten minutes the node exits non-zero, which lets the launch file's `respawn_max_retries` stop two nodes with the same `robot.id` from flapping for ever. `0` disables the exit. |
| `mqtt.max_payload_bytes` | `16384` | Inbound size limit; larger messages are dropped unparsed and counted as `dropped_oversize`. |
| `mqtt.inbound_queue_max` | `256` | Bound of the queue between paho's network thread and the node's main thread; a full queue drops the newest message and counts `dropped_queue_full`. |

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
| `command.task_timeout_cap_s` | `600` | Upper bound on any task timeout, whether announced by the executor or requested by a command; the catalogue is announced with the capped values. |
| `data_dir` | `~/.ros/ros2_ha_bridge/{robot.id}` | Holds `dedupe.json`, the accepted command ids persisted across a node restart. A directory that cannot be created or written logs one warning and the node runs memory-only. |

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
| `watchdog.timeout_s` | `5.0` | Seconds without a Home Assistant heartbeat before the link counts as down. Clamped to at least 2.5 times the interval Home Assistant announces, so a slow heartbeat never trips the watchdog by itself. |

### Watchdog modes

Home Assistant publishes `{prefix}/ha/heartbeat` every two seconds while its MQTT client is connected. The node tracks it, exposes the result as `ha_link` (`up` or `down`) in its state and on the latched ROS topic `~/ha_link`, and applies `watchdog.mode`:

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

## Fake executor

| Parameter | Default | Meaning |
|---|---|---|
| `fake.task_duration_s` | `2.0` | How long `reset`, `move_to` and `fail` take. |
| `fake.feedback_hz` | `2` | Feedback rate of a running fake motion task; `0` disables feedback. |

## Environment variables

| Variable | Read when | Meaning |
|---|---|---|
| `ROS2_HA_BRIDGE_MQTT_PASSWORD` | `mqtt.password` is empty | The broker password. |
| `ROS2_HA_BRIDGE_COMMAND_SECRET` | `command.secret` is empty | The shared HMAC secret; empty keeps signing off. |

These are the only two the node reads. The launch files pass them through from the environment they were started in and never put them on a command line. Prefer them to the matching parameters: a secret in a params file is readable by whoever can read the file, and a declared parameter is readable by any participant on the DDS domain, which is why the node undeclares the two secret parameters the moment it has read them and logs a warning when it had to. The Docker rig in `tools/` additionally reads `MQTT_HOST`, `MQTT_PORT`, `MQTT_USERNAME`, `TOPIC_PREFIX` and `ROS2_HA_BRIDGE_ROBOT_ID` to render a parameters file for the container; those are a convenience of the rig's entrypoint, not of the node.

## Not parameters

- `hold_on_startup` (calling the executor's `stop_and_hold` once at start-up) is an open question in `docs/decisions.md` and does not exist yet; the proposed default is off.
- The HA-side settings (`ack_timeout_s`, `result_timeout_margin_s`, `heartbeat_interval_s`, `mirror_entities`, the prefix and the secret) belong to the integration's config and options flow and are documented with it.

## The Home Assistant side

The integration inherits the broker host, port, credentials and TLS settings from Home Assistant's own MQTT integration, so broker details are entered once, in Home Assistant, and the integration itself asks only for the topic prefix and the optional shared secret; the install steps are in `docs/install-ha.md` (phase 3).
