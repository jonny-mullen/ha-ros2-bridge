# MQTT contract

This page is the readable form of the contract between the ROS 2 node (`ros2/ros2_ha_bridge`) and the Home Assistant integration (`custom_components/ros2_bridge`). The machine-checked form is described in [`contract/README.md`](../contract/README.md): JSON schemas under `contract/schemas/`, golden and must-reject examples under `contract/examples/`, and HMAC test vectors in `contract/hmac_vectors.json`. Both halves are tested against those files. If this page and the schemas ever disagree, the schemas are what the tests enforce; please raise an issue so this page is corrected.

Contract version: schema 1. Schema 1 only ever gains optional fields.

## Conventions

- `{prefix}` defaults to `ros2_bridge`. It must not start or end with `/`, must not contain `+` or `#`, must not be `homeassistant` (HA's default discovery prefix), and must not start with the segment `ha`. Both halves validate it before use.
- `{robot_id}` and task names match `^[a-z][a-z0-9_]{0,31}$`. The segment `ha` is reserved for HA's own topics and is refused as a robot id.
- Command ids (`{task_id}`) are UUID4 strings matching `^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$`. Both halves validate the topic segment and the payload field against these patterns before any registry, topic or publish use. Anything else is dropped and counted, never answered, because there is no safe topic to answer on.
- Every JSON payload is a UTF-8 object carrying `"schema": 1`. Receivers ignore unknown fields. Timestamps are integer epoch milliseconds UTC.
- Size limits: 16 KiB per message, 64 KiB for `announce`. Larger payloads are dropped unparsed and counted.
- MQTT 5 on both sides. The node connects with `clean_start=True`, so broker-queued copies of commands are never drained after a restart; publisher-side QoS 1 retransmissions are caught by the persisted dedupe store instead. Commands are never retained. Replies are retained with a Message Expiry Interval (ack 120 s, result 300 s; both are node parameters).
- `instance` is `"<hostname>:<pid>"` of the publishing node process. It travels in `announce`, `state` and `heartbeat` so that two nodes claiming one robot id are visible.
- `session` is a fresh UUID4 per node process. HA uses it only to detect a node restart; it never filters replies.

## Topics

| Topic | Direction | QoS | Retain | Expiry | Notes |
|---|---|---|---|---|---|
| `{prefix}/{robot_id}/announce` | ROS to HA | 1 | yes | none | Identity plus task catalogue. An empty retained payload is a tombstone. Republished on connect, on `homeassistant/status` `online` after 0 to `announce.jitter_s` seconds, and on catalogue change. |
| `{prefix}/{robot_id}/availability` | ROS to HA | 1 | yes | none | `online` on connect; `offline` as the Last Will and on clean shutdown. |
| `{prefix}/{robot_id}/state` | ROS to HA | 1 | yes | none | On change and on (re)connect. |
| `{prefix}/{robot_id}/heartbeat` | ROS to HA | 0 | no | 5 s | Every `heartbeat.interval_s` (default 2; 0 disables, and HA then does no staleness tracking). |
| `{prefix}/{robot_id}/cmd` | HA to ROS | 1 | never | `ceil(max_age_ms / 1000)` | The node subscribes with `SubscribeOptions(qos=1, retainAsPublished=True)`, so a planted retained copy is seen with the RETAIN flag both live and at reconnect, dropped, counted and cleared with an empty retained publish. The node's own clearing echo (empty payload) is ignored silently. |
| `{prefix}/{robot_id}/task/{task_id}/ack` | ROS to HA | 1 | yes | `reply.ack_expiry_s` (120) | One distinct ack per id; a duplicate command republishes the stored ack bytes. |
| `{prefix}/{robot_id}/task/{task_id}/feedback` | ROS to HA | 0 | no | 10 s | Optional, throttled to `reply.feedback_max_rate_hz` (2). |
| `{prefix}/{robot_id}/task/{task_id}/result` | ROS to HA | 1 | yes | `reply.result_expiry_s` (300) | Exactly one per accepted task command. Cancel commands have no result (ack-terminal). |
| `{prefix}/ha/heartbeat` | HA to ROS | 0 | no | 5 s | Every `heartbeat_interval_s` (default 2; 0 disables). Not published while HA's MQTT client is disconnected. HA's own `+/heartbeat` handler discards the `ha` segment. |
| `{prefix}/ha/sensors/{domain}/{object_id}` | HA to ROS | 0 | yes | none | Canonical republish of the `mirror_entities` option. Cleared when an entity leaves the option, when the prefix changes, and on entry removal. |
| `homeassistant/status` | HA birth and will | 0 | no | none | The node re-announces on `online`. |
| `zigbee2mqtt/{name}` and `zigbee2mqtt/{name}/availability` | Zigbee2MQTT | as published | as configured | none | Fast path: mirrored unchanged onto the node's `~/mqtt_in` ROS topic together with the source availability. |

### Subscriptions

- HA subscribes once at setup to `{prefix}/+/announce`, `{prefix}/+/availability`, `{prefix}/+/state`, `{prefix}/+/heartbeat`, `{prefix}/+/task/+/ack` and `{prefix}/+/task/+/result` at QoS 1, and to `{prefix}/+/task/+/feedback` at QoS 0. Every wildcard handler first checks the robot segment against the pattern and discards `ha`. Subscription completion on the ack and result filters gates the hub's readiness (bounded at 10 s).
- The node subscribes to its own `cmd` (QoS 1, Retain As Published), to `{prefix}/ha/heartbeat`, to `homeassistant/status`, and to each `mirror.topics` entry plus its availability sibling (QoS 0).

## Payloads

All examples use the prefix `ros2_bridge`, the robot `arm_left`, and timestamps around 2026-10-13T12:00:00Z (epoch 1791892800000 ms). Example ids are complete and valid so they can be pasted into a test.

### announce

Topic `ros2_bridge/arm_left/announce`, QoS 1, retained.

```json
{
  "schema": 1,
  "robot_id": "arm_left",
  "name": "Left arm",
  "manufacturer": "Example Robotics",
  "model": "UR3e",
  "sw_version": "driver 2.4.1",
  "hw_version": null,
  "serial_number": null,
  "suggested_area": "Kitchen",
  "bridge_version": "0.1.0",
  "executor": {"name": "fake", "version": "0.1.0"},
  "session": "9f1c7a0e-4a1b-4b2c-8d1e-0f1a2b3c4d5e",
  "instance": "robotpc:4123",
  "capabilities": {"cancel": true, "feedback": true, "hmac_required": false},
  "watchdog_mode": "none",
  "heartbeat_interval_s": 2,
  "tasks": [
    {"name": "reset", "label": "Reset to home", "kind": "motion", "timeout_s": 60, "params": null},
    {"name": "stop", "label": "Stop", "kind": "control", "timeout_s": 5, "params": null},
    {"name": "pause", "label": "Pause", "kind": "control", "timeout_s": 5, "params": null},
    {"name": "resume", "label": "Resume", "kind": "control", "timeout_s": 5, "params": null},
    {
      "name": "move_to",
      "label": "Move to named pose",
      "kind": "motion",
      "timeout_s": 120,
      "params": {
        "type": "object",
        "properties": {"pose": {"type": "string"}},
        "required": ["pose"]
      }
    }
  ],
  "ts": 1791892800000
}
```

- `tasks` is the allowlist after the operator's `tasks.allow` intersection. A task with no required params gets a button in HA; every task is callable through `send_task`.
- `kind` is `motion` (rejected while busy, in error or inhibited; owns the single active slot), `control` (accepted in any state: stop, pause, resume) or `query` (accepted in any state, no motion).
- `params` is a JSON Schema subset: `type: object`, `properties` whose `type` is one of string, number, integer or boolean, optional `enum`, `minimum` and `maximum`, plus `required`. It is validated by hand-written code on both sides, not by a general JSON Schema library.
- `capabilities.hmac_required` tells HA whether the node will refuse unsigned commands.
- An empty retained payload on this topic is a tombstone: HA removes the device. A second tombstone for an unknown robot is a no-op.

### availability

Topic `ros2_bridge/arm_left/availability`, QoS 1, retained. The payload is the plain string `online` or `offline`, not JSON. `offline` is registered as the node's Last Will and is also published on clean shutdown.

### state

Topic `ros2_bridge/arm_left/state`, QoS 1, retained, published on change and on (re)connect.

```json
{
  "schema": 1,
  "robot_id": "arm_left",
  "state": "busy",
  "task_id": "6a0c2b58-1d7e-4a1f-9b2a-3c4d5e6f7a8b",
  "task": "move_to",
  "since": 1791892801000,
  "paused": false,
  "inhibited": false,
  "ha_link": "up",
  "watchdog_mode": "none",
  "executor_health": {"level": "ok", "message": ""},
  "error": null,
  "session": "9f1c7a0e-4a1b-4b2c-8d1e-0f1a2b3c4d5e",
  "instance": "robotpc:4123",
  "seq": 42,
  "ts": 1791892801003
}
```

- `state` is exactly `idle`, `busy` or `error` (Q17). `paused`, `inhibited` and `ha_link` are separate fields, not states.
- `error` is `null` or `{"code": "...", "reason": "..."}`. A failed task alone never sets `error`; only an executor fault or a forced finish does.
- `seq` increments per publication within a session. HA drops a state whose `seq` regresses within the same session.
- HA uses `task_id` together with `paused` to suspend and resume its own result deadline for that command.

### heartbeat (ROS to HA)

Topic `ros2_bridge/arm_left/heartbeat`, QoS 0, not retained, Message Expiry Interval 5 s, every 2 s by default.

```json
{
  "schema": 1,
  "robot_id": "arm_left",
  "session": "9f1c7a0e-4a1b-4b2c-8d1e-0f1a2b3c4d5e",
  "instance": "robotpc:4123",
  "seq": 1203,
  "ts": 1791892804000,
  "uptime_s": 2406,
  "ha_clock_offset_ms": 12,
  "counters": {
    "accepted": 12,
    "rejected": 1,
    "duplicates": 0,
    "dropped_retained": 0,
    "dropped_oversize": 0,
    "dropped_invalid_id": 0,
    "dropped_queue_full": 0,
    "publish_errors": 0,
    "takeovers": 0
  }
}
```

### heartbeat (HA to ROS)

Topic `ros2_bridge/ha/heartbeat`, QoS 0, not retained, Message Expiry Interval 5 s, every 2 s by default. Informational unless the node's watchdog mode says otherwise (Q16).

```json
{
  "schema": 1,
  "ts": 1791892800000,
  "seq": 1234,
  "interval_ms": 2000,
  "instance_id": "c2a4f6e8-1b3d-4f5a-9c7e-2d4f6a8b0c1e"
}
```

### cmd, type task

Topic `ros2_bridge/arm_left/cmd`, QoS 1, never retained, Message Expiry Interval `ceil(max_age_ms / 1000)`.

```json
{
  "schema": 1,
  "id": "6a0c2b58-1d7e-4a1f-9b2a-3c4d5e6f7a8b",
  "type": "task",
  "robot_id": "arm_left",
  "task": "move_to",
  "params": {"pose": "home"},
  "issued_at": 1791892800950,
  "max_age_ms": 5000,
  "timeout_s": null,
  "source": {
    "origin": "ha",
    "instance_id": "c2a4f6e8-1b3d-4f5a-9c7e-2d4f6a8b0c1e",
    "kind": "intent",
    "user_id": "a1b2c3d4e5f60718293a4b5c6d7e8f90",
    "context_id": "01J9ZX4K7M2N8P3Q5R6S7T8V9W",
    "device_id": "3f9a1c2e4b5d6f708192a3b4c5d6e7f8"
  },
  "sig": "84c9b26aa7afe51eb312ae6ed7d254f76876e49376a406f783169d14709184ab"
}
```

- `id` is a UUID4 generated by HA and names the reply topics. `robot_id` must equal the topic segment.
- `timeout_s` null means the announced value for that task; a value is clamped to the node's `command.task_timeout_cap_s`.
- `source.kind` names how the command was raised (for example `intent` or `button`); the other `source` fields carry HA's own identifiers for diagnostics. The node records `source` in acks and logs but never interprets it.
- `sig` is present only when a shared secret is configured. The value above was computed with the placeholder secret `example-shared-secret` over this exact topic and payload; that secret is for reading this page, never for a deployment.

### cmd, type cancel

Same topic and publish settings as a task command.

```json
{
  "schema": 1,
  "id": "0c1d9e7a-5b3f-4c2d-8e1f-6a7b8c9d0e1f",
  "type": "cancel",
  "robot_id": "arm_left",
  "target_id": null,
  "issued_at": 1791892802000,
  "max_age_ms": 5000,
  "source": {
    "origin": "ha",
    "instance_id": "c2a4f6e8-1b3d-4f5a-9c7e-2d4f6a8b0c1e",
    "kind": "button",
    "user_id": null,
    "context_id": "01J9ZY5A2B3C4D5E6F7G8H9JKM",
    "device_id": null
  }
}
```

- `target_id` null means the active motion task. A specific task id cancels that task only.
- A cancel command's `accepted` ack is its terminal outcome. The cancelled task's own result (`status: cancelled`) follows under the cancelled task's id.

#### Signing (HMAC)

When a secret is configured on both sides:

```text
canonical = json.dumps(payload_without_sig, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
sig       = hex(HMAC-SHA256(secret, topic + "\n" + canonical))
```

The topic is bound into the signature so a signed command cannot be replayed at another robot. The node compares in constant time. Test vectors live in `contract/hmac_vectors.json`; the canonical form of the task example above is:

```text
{"id":"6a0c2b58-1d7e-4a1f-9b2a-3c4d5e6f7a8b","issued_at":1791892800950,"max_age_ms":5000,"params":{"pose":"home"},"robot_id":"arm_left","schema":1,"source":{"context_id":"01J9ZX4K7M2N8P3Q5R6S7T8V9W","device_id":"3f9a1c2e4b5d6f708192a3b4c5d6e7f8","instance_id":"c2a4f6e8-1b3d-4f5a-9c7e-2d4f6a8b0c1e","kind":"intent","origin":"ha","user_id":"a1b2c3d4e5f60718293a4b5c6d7e8f90"},"task":"move_to","timeout_s":null,"type":"task"}
```

#### Freshness

A command is accepted when

```text
-skew <= now - issued_at <= min(max_age_ms, command.max_age_cap_ms) + skew
```

with `skew = command.clock_skew_tolerance_ms` (default 2000). Otherwise the ack is `rejected_stale` (too old) or `rejected_future` (issued ahead of the node's clock), and the ack carries the measured `clock_skew_ms` so HA can raise its `clock_skew` repair.

### ack

Topic `ros2_bridge/arm_left/task/6a0c2b58-1d7e-4a1f-9b2a-3c4d5e6f7a8b/ack`, QoS 1, retained, Message Expiry Interval 120 s.

An accepted task:

```json
{
  "schema": 1,
  "id": "6a0c2b58-1d7e-4a1f-9b2a-3c4d5e6f7a8b",
  "robot_id": "arm_left",
  "type": "task",
  "task": "move_to",
  "accepted": true,
  "code": "accepted",
  "reason": "",
  "received_at": 1791892800962,
  "clock_skew_ms": 12,
  "session": "9f1c7a0e-4a1b-4b2c-8d1e-0f1a2b3c4d5e",
  "ts": 1791892800963
}
```

A rejected task (the same command arriving while `reset` is still running):

```json
{
  "schema": 1,
  "id": "6a0c2b58-1d7e-4a1f-9b2a-3c4d5e6f7a8b",
  "robot_id": "arm_left",
  "type": "task",
  "task": "move_to",
  "accepted": false,
  "code": "rejected_busy",
  "reason": "Task 'reset' (id 51f0a3b7-2c4d-4e6f-8a9b-0c1d2e3f4a5b) is running",
  "received_at": 1791892800962,
  "clock_skew_ms": 12,
  "session": "9f1c7a0e-4a1b-4b2c-8d1e-0f1a2b3c4d5e",
  "ts": 1791892800963
}
```

Codes form a closed set: `accepted`, `rejected_malformed`, `rejected_unknown_task`, `rejected_invalid_params`, `rejected_stale`, `rejected_future`, `rejected_unauthorised`, `rejected_busy`, `rejected_error`, `rejected_inhibited`, `rejected_executor`, `rejected_no_such_task`.

- A repeated id republishes the stored ack bytes unchanged.
- A command whose `id` is not a UUID gets no ack (there is no safe topic to publish on) and is counted as `dropped_invalid_id`.
- For `type: cancel`, `accepted` is the terminal outcome of that command id. `rejected_no_such_task` means the target was not running.

### feedback

Topic `ros2_bridge/arm_left/task/6a0c2b58-1d7e-4a1f-9b2a-3c4d5e6f7a8b/feedback`, QoS 0, not retained, Message Expiry Interval 10 s, at most 2 Hz.

```json
{
  "schema": 1,
  "id": "6a0c2b58-1d7e-4a1f-9b2a-3c4d5e6f7a8b",
  "robot_id": "arm_left",
  "progress": 0.4,
  "stage": "executing",
  "message": "waypoint 2 of 5",
  "session": "9f1c7a0e-4a1b-4b2c-8d1e-0f1a2b3c4d5e",
  "ts": 1791892802100
}
```

`progress` is 0.0 to 1.0 or null. Feedback is optional; HA shows it as attributes of the state sensor and never records it.

### result

Topic `ros2_bridge/arm_left/task/6a0c2b58-1d7e-4a1f-9b2a-3c4d5e6f7a8b/result`, QoS 1, retained, Message Expiry Interval 300 s. Exactly one per accepted task command.

```json
{
  "schema": 1,
  "id": "6a0c2b58-1d7e-4a1f-9b2a-3c4d5e6f7a8b",
  "robot_id": "arm_left",
  "task": "move_to",
  "kind": "motion",
  "status": "aborted",
  "goal_status": 6,
  "code": "collision",
  "reason": "Planned path collides with object 'mug_42'",
  "data": {"moveit_error_code": -12},
  "started_at": 1791892800970,
  "finished_at": 1791892803410,
  "session": "9f1c7a0e-4a1b-4b2c-8d1e-0f1a2b3c4d5e",
  "ts": 1791892803412
}
```

- `status` maps onto `action_msgs/GoalStatus`: `succeeded` is 4, `cancelled` is 5, `aborted` is 6. `goal_status` repeats the number.
- `kind` repeats the task kind so HA can route control and query results away from the `last_result` sensor.
- `code` matches `^[a-z][a-z0-9_]{0,63}$`. Recommended vocabulary: `ok`, `cancelled`, `preempted`, `collision`, `self_collision`, `no_trajectory`, `planning_failed`, `control_failed`, `timeout`, `invalid_goal`, `controller_error`, `inhibited`, `executor_error`, `executor_unresponsive`, `nothing_to_pause`, `unknown`.
- `data` is free-form and carries the executor's raw detail (for a MoveIt 2 executor, the raw `MoveItErrorCodes` value) so automations can branch on the fine-grained cause.
- HA-synthesised codes never travel on the wire: `no_ack`, `no_result`, `node_restarted`, `ha_restarted`.

### sensors (canonical path, HA to ROS)

Topic `ros2_bridge/ha/sensors/binary_sensor/kitchen_motion`, QoS 0, retained, on every state change and once at setup.

```json
{
  "schema": 1,
  "entity_id": "binary_sensor.kitchen_motion",
  "domain": "binary_sensor",
  "object_id": "kitchen_motion",
  "state": "on",
  "available": true,
  "device_class": "motion",
  "unit": null,
  "attributes": {"friendly_name": "Kitchen motion"},
  "last_changed": "2026-10-13T12:00:00.123456+00:00",
  "ts": 1791892800123
}
```

- HA states `unavailable` and `unknown` become `"state": null, "available": false`.
- `attributes` is filtered to a documented allowlist so that large or private attributes never leave HA.
- The topic is cleared with an empty retained publish when the entity leaves the `mirror_entities` option, when the prefix changes, or when the integration is removed.

## Command handling on the node

All of this runs on the node's main thread, in this order, and the first failure wins.

0. Delivery with the RETAIN flag set on `cmd` (possible live because of Retain As Published): an empty payload from the node's own clear is ignored; anything else is dropped with a WARN, counted as `dropped_retained`, and cleared with an empty retained publish. No ack.
1. Oversize (above `mqtt.max_payload_bytes`, 16 KiB): dropped and counted as `dropped_oversize`. No ack.
2. JSON parse; `id` must be a string matching the UUID pattern, else dropped, counted as `dropped_invalid_id`, rate-limited WARN. No ack. Everything after this point has a safe reply topic.
3. `schema == 1` and required fields correctly typed, else `rejected_malformed`.
4. Payload `robot_id` equals this node's id (the topic segment is already ours by subscription), else `rejected_malformed`.
5. HMAC when a secret is configured, else `rejected_unauthorised` (rate-limited WARN).
6. Dedupe (persisted store): a known id republishes the stored ack bytes, retained, with the same expiry, and stops. This runs before freshness so that a redelivery can never downgrade an `accepted` ack to `rejected_stale`.
7. Freshness: `rejected_stale` or `rejected_future`, with `clock_skew_ms` in the ack.
8. For `type: task`: allowlist (`rejected_unknown_task`), params subset (`rejected_invalid_params`), `timeout_s` clamp.
9. State gating by kind: `motion` is refused while busy (`rejected_busy`), in error (`rejected_error`) or inhibited (`rejected_inhibited`); `control` and `query` pass.
10. `executor.start`: `TaskRejected` becomes `rejected_executor` with the plugin's reason; any other exception becomes `rejected_executor` with one traceback logged. On success: ack `accepted`, dedupe store write (memory and file), `busy` state published for motion, task timeout timer armed.
11. For `type: cancel`: steps 0 to 7, then resolve `target_id` (null means the active motion task); unknown gives `rejected_no_such_task`; otherwise ack `accepted` (terminal for the cancel id) and `executor.cancel(target)`.

After acceptance: `handle.finish()` publishes the retained result with `kind` and returns the robot to `idle` for motion tasks. A task timeout calls `cancel()`, then after `executor.cancel_grace_s` (5) forces `aborted/timeout`, or `aborted/executor_unresponsive` if the cancel produced nothing, and the robot goes to `error` until the executor clears it through `ctx.clear_error()` (for the fake executor, the `clear_error` task; the end-to-end checklist also expects the fake executor's `stop` to clear it). A paused task stays `busy` with `paused: true` and a frozen timeout clock. Rejected acks beyond `reply.retained_rejections_per_minute` (30) are published non-retained so a misfiring automation cannot fill the broker's retained store.

## Reply routing and reconciliation on the HA side

- Replies (`ack`, `feedback`, `result`) are matched by `id` alone. The `session` field is informational.
- `ack`, live or retained: resolves the pending command if one is outstanding. `accepted` moves `sent` to `running` and fires `ros2_bridge_task_ack`; `accepted: false` is terminal `rejected`. A late `accepted` ack for a command already in `timeout_no_ack` re-opens it as `running` and fires `ros2_bridge_task_ack` with `late: true`; a late rejected ack leaves the command terminal. Retained acks never fire events except in that re-open case. For `type: cancel`, an `accepted` ack is terminal `done` and no result is expected.
- `result`: if `id` is in `seen_results`, ignore. Otherwise fire `ros2_bridge_task_result` exactly once (with `replayed: true` for a retained delivery), update entities, and record `(id, finished_at)`. A result may terminate a command in `sent` (no ack seen yet) as well as in `running`. A result that arrives after HA has already synthesised a terminal outcome fires `ros2_bridge_task_late_result` instead, so the "exactly once" rule for `task_result` holds.
- `kind: motion` results update the `last_result` sensor; control and query results update only the event entity and the bus event.
- Command state machine: `created -> sent -> (running | rejected | timeout_no_ack) -> (succeeded | cancelled | aborted | timeout_no_result | lost)`, plus `timeout_no_ack -> running` on a late accepted ack, `sent -> terminal` on a result, and for cancels `sent -> done | rejected | timeout_no_ack`. Terminal exactly once.
- The ack timer starts when HA's publish returns (PUBACK received, or HA's own 10 s ACK wait logged). The result deadline is `sent_at + effective timeout_s + result_timeout_margin_s`, suspended while the retained state shows the task paused.
- Reconciliation runs once per hub start, after the hub is ready and MQTT is connected plus a 3 s quiet period (`replay_settle_s`) for the retained replay burst, and again whenever a robot's `session` changes (after a 1 s grace). For each outstanding command: result already seen, nothing to do; robot's current session equals the stored session and the deadline has not passed, re-adopt as `running` and re-arm the remaining deadline (suspended if the retained state shows it paused); session differs, `lost` (`node_restarted`); deadline passed, `timeout` (`no_result`); robot not yet announced, wait for its announce or the deadline, whichever comes first.
- Synthesised outcomes (`no_ack`, `no_result`, `node_restarted`, `ha_restarted`) are produced only when the robot truly never answers. `no_ack` means "outcome unknown", not "failed" (see open question (d) in `docs/decisions.md`).

## Failure behaviour

The full table, with a test id per row, will live in `docs/failure-modes.md` (phase 4) and is enforced by `contract.yml`.

| Failure | Mechanism | What HA shows |
|---|---|---|
| Broker restart | Retained announce, state and availability republished on reconnect; QoS 1 replies queued by paho | Robots unavailable while HA's MQTT is down, then recover; in-flight commands resolve from retained replies or re-adoption |
| HA restart or options reload mid-task | Store carries session and deadline; reconciliation after the replay burst | Running task re-adopted; its real result fires as `task_result`, never `late_result` |
| ROS node restart | New `session`; stored session per command | Non-terminal commands for that robot become `lost` after a 1 s grace unless a result arrived |
| Node connected but hung | Heartbeat stops while the MQTT keepalive continues | `problem` on (`heartbeat_stale`); entities unavailable after `heartbeat_unavailable_after_s` (30) |
| Dead node, broker down at the moment of death (no LWT) | Same staleness rule | Unavailable after the same timer despite retained `online` |
| QoS 1 redelivery, same process | Dedupe store republishes the stored ack | Nothing new |
| Publisher-side QoS 1 resend across a node restart | Dedupe store persisted to disk, reloaded for ids younger than `max_age_cap_ms + skew` | Nothing new; one `executor.start` |
| Planted retained command | Retain As Published subscription: drop, count, clear | `dropped_retained` in diagnostics |
| Command with a non-UUID or wildcard id | Dropped before any topic is built | `dropped_invalid_id` counter |
| Stale or skewed command | Freshness window | `rejected_stale` or `rejected_future`; `clock_skew` repair |
| Lost ack | Ack retained 120 s; a late accepted ack re-opens the command | `timeout` (`no_ack`, documented as outcome unknown) until a late ack arrives |
| Result before ack (MQTT orders per topic only) | `sent` to terminal transition | Result applied normally |
| Pause longer than the margin | HA suspends the result deadline while the state shows the task paused | No spurious timeout |
| Publish fails (HA's client disconnected) | paho keeps the QoS 1 message and resends on reconnect; the command stays `sent` | `publish_failed` raised to the caller; outcome still tracked |
| Two nodes, one robot id, same client id | Broker DISCONNECT 0x8E (session taken over); node logs ERROR, backs off, exits after `mqtt.takeover_exit_after` takeovers; HA counts session flaps | `duplicate_robot_id` repair |
| Two nodes, one robot id, distinct client ids | Differing `instance` within one heartbeat interval | `duplicate_robot_id` repair |
| Executor never finishes | Task timeout, cancel, grace, forced `aborted/timeout` or `executor_unresponsive` | `last_result` aborted, robot `error`, `problem` on |
| Misfiring automation | Rejected acks beyond `reply.retained_rejections_per_minute` published non-retained; `max_pending_per_robot` on the HA side | `too_many_pending` error to the caller |

## Versioning

- `schema` stays at 1 for as long as every change is additive: new optional fields, or new values in an open vocabulary (result `code`, `source.kind`, heartbeat counters). The ack code set, result `status`, task `kind` and `state` are closed; adding a value to any of them is schema 2 on a new `$id` base, published alongside schema 1 with a migration note.
- A change that removes a field, changes a type, or changes the meaning of an existing value is a new schema number and a new major version of both halves.
- `scripts/check_versions.py` keeps the bridge version in `manifest.json`, `package.xml`, `setup.py` and the announce payload in agreement.
