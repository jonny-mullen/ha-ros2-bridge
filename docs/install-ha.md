# Installing the Home Assistant side

This page installs `ros2_bridge`, the Home Assistant half of ha-ros2-bridge, as a custom integration. It rides Home Assistant's own MQTT connection, so there is no second set of broker credentials to enter and nothing to `pip install`. The ROS 2 half is installed separately on the robot machine (`docs/install-ros2.md`); the two only ever meet on the broker.

Any robot that runs `ros2_ha_bridge` works, including one running the shipped `fake` executor, which moves nothing and is the right thing to install against first.

## Requirements

- Home Assistant Core 2026.10 or newer (`hacs.json` sets the minimum).
- The MQTT integration configured and connected to an MQTT 5 broker. The Mosquitto add-on qualifies. The bridge inherits the broker host, port, user name, password and TLS settings from that integration and never asks for them.
- At least one robot node publishing under the same topic prefix (default `ros2_bridge`). Until a robot announces, the integration creates no devices.
- Both machines on NTP within 2 s of each other. Commands carry an issue time and a maximum age, and a robot refuses commands whose clock does not agree with its own.

## 1. Install the integration

### Through HACS (custom repository)

1. In HACS, open the three-dot menu and choose Custom repositories.
2. Enter `https://github.com/jonnytinkererofthings/ha-ros2-bridge`, choose the type Integration and add it.
3. Search HACS for "ROS 2 Bridge", download it, and restart Home Assistant when HACS asks.

### By copying the folder

Copy the `custom_components/ros2_bridge` folder from the repository into the Home Assistant configuration folder, so that `<config>/custom_components/ros2_bridge/manifest.json` exists, then restart Home Assistant. Any route that can write to the configuration folder works (the Samba share add-on, the File editor add-on, or `scp` to a Core install). Nothing else needs installing: the integration lists no pip requirements.

## 2. Add the integration

### When a robot is already announcing (discovery)

The integration asks Home Assistant's MQTT integration to watch `ros2_bridge/+/announce`. When a robot's retained announce is seen there, a discovered card appears under Settings, Devices and services, titled with the robot's announced name. Pressing Configure shows the step "Robot found":

> Found robot {robot_name} announcing under the topic prefix {topic_prefix}. Create the ROS 2 Bridge entry?

with one optional field, the shared secret. Submit to create the entry. Discovery only fires for the default prefix, because the watched filter is fixed in the manifest; a site that uses another prefix adds the integration by hand.

### Adding it by hand

Settings, Devices and services, Add integration, "ROS 2 Bridge". The one form asks for:

| Field | Default | Meaning |
|---|---|---|
| Topic prefix | `ros2_bridge` | The first part of every bridge topic (prefix, then the robot id, then the message kind). No leading or trailing slash, no `+` or `#`, not the MQTT discovery prefix `homeassistant`, and not starting with the reserved segment `ha`. Must match the `topic_prefix` parameter on the robots. |
| Shared secret | empty | Optional. When set, every command is signed with HMAC-SHA256 and the robots refuse unsigned commands; it must equal `ROS2_HA_BRIDGE_COMMAND_SECRET` on the robots. Leave empty to send unsigned commands (the default, interview decision Q15). |

It does not ask for the broker: host, port, credentials and TLS come from the MQTT integration. The flow aborts with a plain message if the MQTT integration is not configured, if its client does not become available, or if the bridge is already configured (only one entry is allowed per Home Assistant).

Once the entry exists the integration subscribes to the eight bridge filters, receives the retained announce, availability and state of every robot in the replay burst, and creates one device per robot within a few seconds. If no device appears, see `docs/troubleshooting.md`. Commands fail with `not_ready` until the acknowledgement and result subscriptions are confirmed, at most 10 s after start (a fixed bound; `docs/configuration.md`, Fixed values).

## 3. Options

Settings, Devices and services, ROS 2 Bridge, Configure. Saving reloads the entry; commands that are outstanding at that moment survive the reload and are reconciled afterwards.

Every setting of the Home Assistant side is on this one form. The values 0.1.0 carried as constants became options on 2026-10-14 (towards 0.2.0), each defaulting to exactly the value it had, so an upgraded entry behaves as before until something is changed. The timings shared with the nodes and the heuristics behind two repairs sit in a collapsed "Advanced" section at the bottom of the form. Timeouts are evaluated once a second (the hub's housekeeping tick), so a value below 1 s, or the fraction of a second in any value, takes effect at the next tick, up to 1 s late. What is deliberately not an option is listed under Fixed values in `docs/configuration.md`.

| Option | Range | Default | What it does |
|---|---|---|---|
| Topic prefix | text, same rules as above | `ros2_bridge` | Changing it moves every subscription and publication; the robots must be configured with the same prefix. Sensor topics published under the old prefix are cleared. |
| Shared secret | text (hidden) | empty | Leave empty to send unsigned commands. When set, it must match the secret configured on the robots. |
| Heartbeat interval | 0 to 10 s, step 0.5 | 2 | How often Home Assistant publishes its heartbeat to the robots. 0 disables it; robots then report the link as down. |
| Acknowledgement timeout | 0.5 to 10 s, step 0.1 | 2.0 | How long to wait for a robot to acknowledge a command before it is reported as not acknowledged (outcome unknown). Decided at the next whole-second tick. |
| Command maximum age | 1000 to 600000 ms, step 100 | 5000 | How long a command stays valid after it is issued. Robots refuse older commands as stale, and each node clamps the value to its own `command.max_age_cap_ms` (default 30000), so a value above the node's cap buys nothing until the cap is raised too. |
| Result timeout margin | 0 to 300 s, step 1 | 10 | Extra seconds beyond a task's announced timeout before Home Assistant reports a missing result. |
| Maximum pending commands per robot | 1 to 100 | 16 | How many commands may be outstanding per robot at once; further commands are refused. |
| Unavailable after missing heartbeats for | 0 to 600 s, step 1 | 30 | Seconds without a robot heartbeat before its entities become unavailable. 0 disables this. Keep it above "Heartbeat stale intervals" times the robot's `heartbeat.interval_s`, or the entities go unavailable before the problem sensor can warn (a node announcing a 60 s interval needs more than 180 s here, or 0). |
| Voice acknowledgement wait | 0.5 to 5 s, step 0.1 | 1.5 | How long a voice command waits for the robot's acknowledgement before replying "I will report back" (section 8). Never longer than the acknowledgement timeout. |
| Joint moving threshold | 0 to 180, step 0.001, in the robot's announced unit | 0, meaning the unit's default (0.5 for degrees, 0.01 for radians) | The threshold behind a joint sensor's `moving` attribute (section 4). When the robot reports velocities they decide alone: `moving` is on while the reported velocity exceeds this, in the announced unit per second, so a reported 0 is at rest even if the position still changed. A robot that reports no velocities is judged by its positions: on when one changed by more than this since the previous report. The defaults equal the node's `telemetry.joints_min_delta` defaults, so every movement the node thought worth publishing counts. |
| Joint moving hold | 0 to 30 s, step 0.5 | 0, meaning two publication intervals of the robot's announced rate, never less than 1 s | How long a report's `moving` verdict stands when no newer report arrives, after which the joint counts as still (decided at the next whole-second tick). The node publishes positions only when they change, so a lone joint moving slower than `telemetry.joints_min_delta` times half the announced rate per second reports less often than two intervals; raise this if `moving` flickers during a slow sweep. |
| Mirrored entities | entity picker, several | none | Entities republished to the robots under `{prefix}/ha/sensors/...` in the canonical sensor schema, one retained message per entity, on every state or attribute change (`docs/sensors.md`). |
| Extra mirrored attributes | text, one name per entry | none | Attribute names added to the built-in allowlist of attributes a mirrored entity may carry to the robots (`docs/sensors.md` lists the built-in names), for example `pm2_5`, `moisture`, `distance` or a template sensor's own attribute. Everything published here is readable by every client of the broker: never add locations, tokens, identifiers or anything else private. |

### Advanced

Collapsed by default. The first two must equal the matching node parameters on every robot; the rest shape how the integration judges what it sees. Their defaults are the nodes' defaults and the values 0.1.0 shipped as constants.

| Option | Range | Default | What it does |
|---|---|---|---|
| Acknowledgement expiry | 10 to 3600 s | 120 | Must equal the nodes' `reply.ack_expiry_s`: how long a retained acknowledgement lives on the broker. It is how long a cancel command waits for its acknowledgement. |
| Result expiry | 10 to 86400 s | 300 | Must equal the nodes' `reply.result_expiry_s`: how long a retained result lives on the broker. Home Assistant remembers seen result ids for this plus the result timeout margin (310 s by default), which is what makes `ros2_bridge_task_result` fire exactly once per task across restarts; a node whose result expiry is longer than this value replays a retained result after a later restart as a second event. |
| Replay settle period | 1 to 30 s | 3 | Quiet period after the subscriptions are confirmed and MQTT is connected before outstanding commands are reconciled against the retained replay burst. A large fleet, a TLS broker over Wi-Fi or a big retained replay may want longer; too short re-synthesises outcomes a retained result would have answered. |
| Session change grace | 0 to 10 s, step 0.5 | 1 | After a robot's session changes (its node restarted), how long its retained results are waited for before its outstanding commands are marked `lost` (`node_restarted`). A slow broker that delivers a restarted node's retained result shortly after its announce wants 2 to 3 s. |
| Feedback throttle | 0 to 5000 ms, step 50 | 500 | The least time between two feedback writes (progress, stage, message) into a robot's state sensor; 0 writes every feedback. The default mirrors the node's `reply.feedback_max_rate_hz` of 2; set it to 1000 divided by that rate to show every message. A throttled write waits for the next housekeeping tick, so the final feedback of a task can show up to the throttle plus 1 s late. |
| Heartbeat stale intervals | 2 to 20 | 3 | How many of the robot's announced heartbeat intervals may pass without a heartbeat before it counts as stale (section 4). A jittery Wi-Fi site may want 5 to stop the problem sensor flapping. |
| Clock skew warning | 100 to 10000 ms, step 100 | 1000 | An acknowledgement reporting a clock difference of at least this opens the clock skew repair (section 7). Keep it below the nodes' `command.clock_skew_tolerance_ms` (default 2000); a site that raised the node's tolerance raises this with it. A `rejected_stale` or `rejected_future` opens the repair whatever this value. |
| Duplicate robot id: session changes | 1 to 20 | 3 | More than this many session changes of one robot within the window open the `duplicate_robot_id` repair (section 7). |
| Duplicate robot id: window | 60 to 3600 s, step 30 | 300 | The sliding window the session changes are counted over; it also bounds how long a second instance's identity is remembered. |
| Duplicate robot id: quiet period | 60 to 3600 s, step 30 | 600 | Seconds without new evidence after which the repair closes by itself. A site that restarts its nodes often on purpose (nightly) raises the count or the window instead of living with the repair. |

## 4. What appears for each robot

One device per announced robot, named from the announce (`robot`, `manufacturer`, `model`, `sw_version`, `hw_version`, `serial_number` and the suggested area, where the robot gave them). The entities below are named with the device name first, so a robot announced as "Left arm" gets `sensor.left_arm_state`, `button.left_arm_cancel_task` and so on.

| Entity | States | What it means |
|---|---|---|
| State (sensor, enum) | Idle, Busy, Error | The robot's discrete state from its retained `state` message (Q17). Attributes: `task_id`, `task`, `since`, `paused`, `inhibited`, `ha_link`, `watchdog_mode`, `executor_health`, `error_code`, `error_reason`, `session`, `instance`, and the latest feedback as `progress`, `stage`, `message`. The feedback attributes, `since` and `session` are not recorded. Feedback is written at most once per "Feedback throttle" (default 500 ms) per robot. |
| Last result (sensor, enum) | Succeeded, Cancelled, Aborted, Rejected, Timed out, Lost | The outcome of the most recent motion task. Control and query tasks (stop, pause, resume) never change it. Attributes: `task_id`, `task`, `code`, `reason`, `data`, `finished_at`, `duration_ms`, `synthetic` (true when Home Assistant decided the outcome because the robot never answered). |
| Problem (binary sensor) | on, off | On while the robot reports `error`, its executor health is `error`, its heartbeat is stale (no heartbeat for "Heartbeat stale intervals" of the robot's announced interval, by default 3, about 6 s at the node's default 2 s) while the retained availability still says `online`, or the `duplicate_robot_id` repair is open. The `reasons` attribute lists which (`error`, `executor_error`, `heartbeat_stale`, `duplicate_robot_id`). |
| Task result (event) | Succeeded, Cancelled, Aborted, Rejected, Timed out, Lost | Triggers exactly when the `ros2_bridge_task_result` bus event fires for this robot, with the same data as attributes (`task_id`, `task`, `kind`, `code`, `reason`, `data`, `goal_status`, `started_at`, `finished_at`, `duration_ms`, `synthetic`, `replayed`). A late result does not re-trigger it. |
| Joint 1 {name}, Joint 2 {name}, ... (sensor, one per announced joint) | a number in ° or rad | Added 2026-10-14 towards 0.2.0. Created only for a robot whose announce carries a `joints` block: one sensor per joint in the announced order, named "Joint {n} {joint name}" (for the fake executor `sensor.fake_arm_1_joint_1_j1` to `sensor.fake_arm_1_joint_7_j7`), unit ° or rad as announced, a suggested display precision of one decimal for degrees and three for radians (changeable per entity in its settings dialog, as for any sensor), no state class. Attributes: `index` (from 0), `joint_name`, `limit_min`, `limit_max` (null when no limits are announced), `velocity` (the node's reported velocity for the joint, in the unit per second; null when the node sends none) and `moving` (true while the latest report shows the joint moving: the reported velocity exceeds the "Joint moving threshold" option, or, for a robot that sends no velocities, the position changed by more than it since the previous report; reported velocities decide alone, so a report with velocity 0 is at rest whatever the position did; the verdict stands for the "Joint moving hold" option after the report, by default two publication intervals of the announced rate and at least 1 s, after which the joint counts as still until the next report; null before the first report). `velocity` and `moving` are not recorded. Updated from the robot's `joints` topic at up to its announced `rate_hz`; keeps the last value when the stream stalls and is unavailable with the robot. Removed when a later announce drops the joint or the key. Enabled by default; see the recorder note below. |
| One button per task (button) | press | One button for every announced task that needs no parameters, named with the task's announced label (for the fake executor: Reset to home, Stop, Pause, Resume, and so on). A task with required parameters gets no button and is reachable through the `send_task` action. Pressing publishes the command and waits for the acknowledgement; a refusal or a missing acknowledgement is shown as an error on the button. The buttons follow the catalogue when a robot re-announces. |
| Cancel task (button) | press | Cancels the robot's active motion task. |

Every entity of a robot is unavailable while Home Assistant's MQTT client is disconnected, while the robot's availability topic says `offline` (its Last Will), until the robot has announced since Home Assistant started, and after no heartbeat has arrived for "Unavailable after missing heartbeats for" seconds.

**Recorder note for the joint sensors.** They are ordinary sensors, so the recorder keeps every change (up to two per joint per second while an arm moves, by the node's default rate) for its purge window. An installation that does not want them in the database excludes them in `configuration.yaml` and restarts:

```yaml
recorder:
  exclude:
    entity_globs:
      - sensor.*_joint_*
```

The gauges of the example dashboard (`docs/dashboard.md`) still move with the exclude in place, because they show the live state; its history graph of the joints then stays empty. `docs/sensors.md` has the details of the joints path.

## 5. Actions (services)

The three actions are registered when Home Assistant starts and exist whether or not the entry is loaded; a call while it is not loaded fails with "The ROS 2 Bridge is not loaded". Non-administrator users can call them (and press the buttons, and use voice); the integration never exposes a raw MQTT publish.

### `ros2_bridge.send_task`

Sends a task command to one or more robots and optionally waits.

| Field | Required | Meaning |
|---|---|---|
| `device_id` | one of the two | One or more robot devices (the device picker is filtered to this integration). |
| `robot_id` | one of the two | The robot's id as announced on the broker, as an alternative to a device. Both may be given. |
| `task` | yes | The announced task name, for example `reset` or `move_to`. |
| `params` | no | Parameters for the task, matching the schema the robot announced. |
| `wait` | no, default `ack` | `ack` returns after the acknowledgement, `result` after the task's result, `none` at once. |
| `timeout_s` | no | Overrides the task's announced timeout; the robot may clamp it. |
| `max_age_ms` | no, at least 1000 | Overrides the "Command maximum age" option for this command. The form offers up to 600000 ms; whatever the value, the node clamps it to its `command.max_age_cap_ms`. |

Every target is validated before the first command is published: an unknown robot, an unavailable robot, an unknown task, bad parameters or a robot at its pending cap fails the whole call with nothing sent. Without a response variable, a refusal raises "Robot {robot_id} refused task {task}: {reason}", a missing acknowledgement raises "Robot {robot_id} did not acknowledge the command in time; the outcome is unknown", and with `wait: result` a missing result raises "Robot {robot_id} did not report a result for task {task} before the deadline", so a script stops at the step that did not happen. With a response variable nothing short of a validation or publish problem raises; the outcome is in the response.

A button-like reset by device:

```yaml
action: ros2_bridge.send_task
target: {}
data:
  device_id: 3f9a1c2e4b5d6f708192a3b4c5d6e7f8
  task: reset
```

A parameterised task by robot id, waiting for the result and reading it:

```yaml
- action: ros2_bridge.send_task
  data:
    robot_id: arm_left
    task: move_to
    params:
      pose: home
    wait: result
  response_variable: outcome
- condition: template
  value_template: "{{ outcome.tasks[0].result.status == 'succeeded' }}"
```

The response is `{"tasks": [...]}` with one entry per robot: `robot_id`, `task_id`, `task`, `kind`, `state`, `accepted`, `code`, `reason`, `clock_skew_ms`, and with `wait: result` a `result` object (`task_id`, `task`, `status`, `goal_status`, `code`, `reason`, `data`, `started_at`, `finished_at`, `duration_ms`, `synthetic`).

### `ros2_bridge.cancel_task`

Cancels the running task of one or more robots, or one task by its id.

| Field | Required | Meaning |
|---|---|---|
| `device_id`, `robot_id` | one of the two | As for `send_task`. |
| `task_id` | no | The id of the task to cancel. Empty cancels the robot's active motion task. |
| `wait` | no, default `ack` | `ack` returns when the cancel is acknowledged, `result` when the cancelled task's own result arrives, `none` at once. |
| `max_age_ms` | no | As for `send_task`. |

```yaml
action: ros2_bridge.cancel_task
data:
  robot_id: arm_left
  wait: ack
```

A cancel's `accepted` acknowledgement is its terminal outcome; the cancelled task reports `cancelled` under its own id. The response entry carries `target_id` in addition to the fields above.

### `ros2_bridge.get_robots`

Returns every announced robot. It takes no fields and must be called with a response variable:

```yaml
- action: ros2_bridge.get_robots
  response_variable: robots
- repeat:
    for_each: "{{ robots.robots | selectattr('available') | map(attribute='robot_id') | list }}"
    sequence:
      - action: ros2_bridge.send_task
        data:
          robot_id: "{{ repeat.item }}"
          task: reset
          wait: none
        continue_on_error: true
```

Each entry has `robot_id`, `device_id`, `robot`, `available`, `state`, `task`, `task_id`, `paused`, `ha_link`, `problem`, `instance`, `session` and `tasks` (the catalogue: `robot`, `label`, `kind`, `timeout_s`, `params`, `description`, `needs_params`).

## 6. Bus events

Automations react to outcomes through three events on the Home Assistant bus. Every event carries `robot_id` and `device_id`, so a trigger can be filtered to one robot.

| Event | When | Fields |
|---|---|---|
| `ros2_bridge_task_ack` | Every live acknowledgement, accepted or rejected, and every late acceptance | `robot_id`, `device_id`, `task_id`, `type` (`task` or `cancel`), `task`, `kind`, `accepted`, `code`, `reason`, `clock_skew_ms`, `late` (true for an acceptance that arrived after the acknowledgement timeout), `synthetic`, `source` |
| `ros2_bridge_task_result` | Exactly once per task command, when its outcome is known: a result from the robot, a rejection, or a timeout or loss decided by Home Assistant | `robot_id`, `device_id`, `task_id`, `task`, `kind` (`motion`, `control` or `query`), `status` (`succeeded`, `cancelled`, `aborted`, `rejected`, `timeout`, `lost`), `goal_status`, `code`, `reason`, `data`, `started_at`, `finished_at`, `duration_ms`, `synthetic` (true when Home Assistant decided the outcome), `replayed` (true for a retained result delivered after a restart), `source` |
| `ros2_bridge_task_late_result` | A real result that arrived after Home Assistant had already synthesised an outcome for the same id | The same fields as `ros2_bridge_task_result` |

Codes in `ros2_bridge_task_result`: for `succeeded`, `cancelled` and `aborted` the robot's own code (for example `ok`, `collision`, `no_trajectory`, `timeout`); for `rejected` the acknowledgement code (for example `rejected_busy`); for `timeout` one of `no_ack` (outcome unknown) or `no_result`; for `lost` one of `node_restarted` or `ha_restarted`. `docs/examples/automations.yaml` has a speak-back automation and a notification automation built on these events.

```yaml
triggers:
  - trigger: event
    event_type: ros2_bridge_task_result
    event_data:
      robot_id: arm_left
```

## 7. Repairs

The integration raises repair issues under Settings, System, Repairs. None needs a button press; each closes by itself when the condition clears.

| Repair | Raised when | Meaning |
|---|---|---|
| Clock skew with robot {robot_id} | An acknowledgement reports a clock difference of at least the "Clock skew warning" option (default 1000 ms), or a command is rejected as `rejected_stale` or `rejected_future` | Commands carry an issue time and a maximum age, so a difference above the robot's tolerance makes it refuse them. Check that both machines use NTP and agree to within two seconds. Keep the option below the node's `command.clock_skew_tolerance_ms`, or a site that tolerates more skew on purpose sees the repair as a warning only. Closes once an acknowledgement arrives with an acceptable difference. |
| Robot {robot_id} refused the command signature | A command is rejected as `rejected_unauthorised`, or the robot announces `hmac_required` while no shared secret is configured here | The shared secret in the options must match the secret on the robot, or be cleared on both sides to send unsigned commands. Closes once the robot accepts a command (or announces without requiring a signature). |
| Robot {robot_id} sent an invalid announce | The retained message on the robot's announce topic does not decode as a schema 1 announce, or names another robot | The robot is not shown until it announces correctly. Check the robot's bridge version and task catalogue and restart its node. Closes once a valid announce arrives. |
| The MQTT connection does not support message expiry | Publishing a command with a message expiry interval was refused because Home Assistant's MQTT session is not MQTT 5 | Commands were sent without an expiry, so a stale command can sit in the broker until a robot reads it; the robots still refuse commands older than their maximum age. Set the MQTT integration's protocol to 5 and make sure the broker supports it. Raised once for the connection, not per robot; closes once a publish with an expiry succeeds. |
| Two robot nodes may be using the id {robot_id} | Two different `instance` values (host and pid) are seen for one robot within one heartbeat interval (the robot's announced interval, floored at 2 s so a robot announcing no heartbeat is still checked), or more than "Duplicate robot id: session changes" (3) session changes are seen within "Duplicate robot id: window" (300 s) | Two nodes with one id take each other's broker session over in turn and their state, acknowledgements and results interleave. A node that crashes and restarts repeatedly trips this check too. Give each robot its own `robot.id` and restart the nodes. Closes after "Duplicate robot id: quiet period" (600 s) without new evidence. |

## 8. Voice

The integration registers two Assist intents, `Ros2BridgeSendTask` and `Ros2BridgeCancelTask`. `docs/assist.md` explains how to install the sentences and what the replies are. A spoken command waits for the robot's acknowledgement for at most the "Voice acknowledgement wait" option (default 1.5 s, never longer than the acknowledgement timeout) before it replies "I will report back"; a site with a slow link, or a satellite whose pipeline tolerates longer waits, raises it so the spoken reply names the real outcome. The replies themselves are one table (`SPEECH` at the top of `intent.py`), worded for any robot (never "arm"), so an adopter or a translator changes them in one place.

## 9. Uninstalling

Removal is in two layers, because the retained messages on the broker belong to two different owners.

1. **Robots' retained topics belong to the node.** The retained announce, state and availability of each robot stay on the broker after the integration is gone, and would rediscover the robot if it were ever reinstalled. To clear them from Home Assistant, stop the robot's node (so its availability turns `offline`) and delete the robot's device from its device page. Device deletion is only offered for an offline or unknown robot; it publishes an empty retained message to the robot's announce, state and availability topics, and works whether or not the entry is loaded. Alternatively, clear them with `mosquitto_pub -r -n` on the robot machine.
2. **Delete the entry.** Settings, Devices and services, ROS 2 Bridge, Delete. The integration clears every `{prefix}/ha/sensors/...` topic it published (an empty retained publish for each topic it recorded in its store), deletes its store document (`.storage/ros2_bridge.commands`) and closes its repair issues. The Home Assistant heartbeat is not retained, so there is nothing to clear for it.
3. **Remove the files** through HACS (Remove), or by deleting `<config>/custom_components/ros2_bridge/`, then restart Home Assistant. The three actions and the two intents stay registered until that restart.
