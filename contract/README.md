# ha-ros2-bridge MQTT contract, version 1

This folder is the single written agreement between the two halves of
ha-ros2-bridge: the ROS 2 package `ros2_ha_bridge` and the Home Assistant
custom integration `ros2_bridge`. Both halves are tested against the files
here and nothing else. If a payload is not described in this folder it is
not part of the contract.

Contents:

| Path | What it holds |
|---|---|
| `schemas/common.schema.json` | Shared definitions (`robot_id`, `task_id`, `params_subset`, `ack_code`, `joint_unit`, `joint_name`, `joint_limits` and so on) |
| `schemas/<kind>.schema.json` | One JSON Schema (draft 2020-12) per payload kind |
| `examples/valid/<kind>/*.json` | Golden examples. The only hand-written fixtures either test suite may use |
| `examples/invalid/<kind>/*.json` | Must-reject examples. Each file name says what is wrong with it |
| `hmac_vectors.json` | Signature test vectors both halves must reproduce exactly |
| `../scripts/validate_contract.py` | Checks all of the above and exposes `validate(kind, payload)` |
| `../scripts/sync_contract.py` | Keeps the ROS 2 package's vendored copy of `schemas/` byte-identical |

## Conventions

- `{prefix}` is the configured topic prefix, default `ros2_bridge`. It has no
  leading or trailing slash, contains no `+` or `#`, and is never
  `homeassistant` (the HA discovery prefix) nor anything starting with the
  segment `ha`.
- `{robot_id}` and task names match `^[a-z][a-z0-9_]{0,31}$`. The id `ha` is
  reserved for Home Assistant's own topics and is refused as a robot id.
- `{task_id}` (the command id) is a lower-case UUID4 string matching
  `^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$`.
  Both halves validate the topic segment and the payload field against these
  patterns before any registry lookup, topic build or publish. Anything else is
  dropped and counted, never answered, because a bad id has no safe reply
  topic.
- Every JSON payload is a UTF-8 encoded object carrying `"schema": 1`. The one
  exception is `availability`, whose payload is the plain word `online` or
  `offline` with no JSON wrapper.
- Timestamps (`ts`, `issued_at`, `received_at`, `since`, `started_at`,
  `finished_at`) are integer epoch milliseconds, UTC.
- `instance` is `"<hostname>:<pid>"` of the publishing node process, carried in
  `announce`, `state` and `heartbeat` so that two nodes claiming one robot id
  are visible on the HA side.
- `session` is a new UUID4 per node process. HA uses it only to detect a node
  restart; replies are routed by `id` alone and `session` never filters them.
- MQTT 5 on both sides. The node connects with `clean_start=True`, so broker
  queued commands are never drained after a node restart.

## Size limits

| Payload | Limit |
|---|---|
| `announce` | 64 KiB |
| Every other message | 16 KiB |

Larger payloads are dropped unparsed and counted (`dropped_oversize`). A JSON
Schema cannot express a byte limit, so `validate_contract.py` measures each
example's compact UTF-8 encoding (`separators=(",", ":")`,
`ensure_ascii=False`, the smallest form it could take on the wire) against the
limit of its kind. `examples/invalid/command/oversize_over_16kib.json` and
`examples/invalid/announce/oversize_over_64kib.json` are otherwise valid
payloads that are rejected on size alone. A receiver measures the bytes it was
actually given.

## Topic table

| Topic | Direction | QoS | Retain | Expiry | Notes |
|---|---|---|---|---|---|
| `{prefix}/{robot_id}/announce` | ROS to HA | 1 | yes | none | Identity plus task catalogue. An empty retained payload is a tombstone. Republished on connect, on `homeassistant/status` `online` after a 0 to `announce.jitter_s` delay, and on catalogue change. |
| `{prefix}/{robot_id}/availability` | ROS to HA | 1 | yes | none | `online` on connect; `offline` as the Last Will and on clean shutdown. Plain words, not JSON. |
| `{prefix}/{robot_id}/state` | ROS to HA | 1 | yes | none | Published on change and on (re)connect. |
| `{prefix}/{robot_id}/heartbeat` | ROS to HA | 0 | no | 5 s | Every `heartbeat.interval_s` (default 2; 0 disables, and HA then does no staleness tracking). |
| `{prefix}/{robot_id}/joints` | ROS to HA | 0 | no | 5 s | Optional (0.2.0). Joint positions in the unit and order of the announce `joints` block. Published only by a node whose announce carries that block, at most `telemetry.joints_rate_hz` per second (default 2), only when a position moved by more than `telemetry.joints_min_delta`, plus once on every (re)connect and announce. |
| `{prefix}/{robot_id}/cmd` | HA to ROS | 1 | never | `ceil(max_age_ms / 1000)` s | The node subscribes with Retain As Published, so a planted retained command is seen with its retain flag, dropped, counted and cleared with an empty retained publish. The node's own clearing echo is ignored silently. |
| `{prefix}/{robot_id}/task/{task_id}/ack` | ROS to HA | 1 | yes | `reply.ack_expiry_s` (120 s) | One distinct ack per command id. A repeated id republishes the stored ack bytes. |
| `{prefix}/{robot_id}/task/{task_id}/feedback` | ROS to HA | 0 | no | 10 s | Optional. Throttled on the node to `reply.feedback_max_rate_hz` (2). |
| `{prefix}/{robot_id}/task/{task_id}/result` | ROS to HA | 1 | yes | `reply.result_expiry_s` (300 s) | Exactly one per accepted task command. Cancel commands have no result. |
| `{prefix}/ha/heartbeat` | HA to ROS | 0 | no | 5 s | Every `heartbeat_interval_s` (default 2; 0 disables). Not published while HA's MQTT client is disconnected. |
| `{prefix}/ha/sensors/{domain}/{object_id}` | HA to ROS | 0 | yes | none | Canonical republish of the entities chosen in `mirror_entities`. Cleared with an empty retained publish when an entity leaves the option, when the prefix changes, and on entry removal. |
| `homeassistant/status` | HA birth and will | 0 | no | none | Not part of this contract; the node re-announces on `online`. |
| `zigbee2mqtt/{name}` and `zigbee2mqtt/{name}/availability` | Zigbee2MQTT | as published | as configured | none | Fast path. The node mirrors chosen topics unchanged onto its `~/mqtt_in` ROS topic; nothing here is reformatted. |

HA subscribes once at setup to `{prefix}/+/announce`, `+/availability`,
`+/state`, `+/heartbeat`, `+/task/+/ack` and `+/task/+/result` (QoS 1) and
`+/task/+/feedback` and `+/joints` (QoS 0). Every wildcard handler first
checks the robot segment against the pattern and discards the segment `ha`. The node subscribes
to its own `cmd` (QoS 1, Retain As Published), `{prefix}/ha/heartbeat`,
`homeassistant/status` and each configured mirror topic with its availability
sibling (QoS 0).

## Retain and expiry rules

- Commands are never retained. A retained message on `cmd` is an attack or a
  mistake; the node drops it, counts `dropped_retained` and clears it.
- Replies (`ack`, `result`) are retained so that HA, which does not wait for
  the broker's SUBACK, can still find a reply that arrived before its
  subscription settled. They carry an MQTT 5 Message Expiry Interval so the
  broker forgets them (ack 120 s, result 300 s, both node parameters).
- `announce`, `availability` and `state` are retained without expiry so HA sees
  the robot immediately after its own restart. An empty retained payload on any
  of them clears it; HA treats an empty `announce` as a tombstone that removes
  the device.
- `heartbeat` in both directions, `feedback` and `joints` are not retained and
  expire in seconds, so a late subscriber never sees stale liveness data or a
  stale pose. A joint sensor in HA keeps its last value until the robot itself
  goes unavailable.
- `ha/sensors/...` is retained without expiry so a node that starts later sees
  the last known state; `unavailable` and `unknown` become `state: null` with
  `available: false` rather than an absent message.

## Payload reference

Each kind has a schema in `schemas/` and golden examples in `examples/valid/`.
The fields below are the required ones; every schema also sets
`"additionalProperties": true` because receivers ignore unknown fields.

### announce

Required: `schema`, `robot_id`, `name`, `bridge_version`, `session`,
`instance`, `capabilities` (`cancel`, `feedback`, `hmac_required`),
`watchdog_mode` (`none`, `inhibit_new`, `stop_and_hold`), `heartbeat_interval_s`,
`tasks`, `ts`. Optional: `manufacturer`, `model`, `sw_version`, `hw_version`,
`serial_number`, `suggested_area` (all nullable strings), `executor`
(`name`, `version`), `joints` (0.2.0, see below).

Each entry of `tasks` requires `name`, `label`, `kind` (`motion`, `control`,
`query`), `timeout_s` (greater than 0) and `params` (null or a params subset
schema, see below); `description` is optional. `tasks` is the allowlist after
the operator's `tasks.allow` intersection. A task with no required params gets a
button in HA; every task is callable through the `send_task` service.

Task kinds: `motion` is rejected while the robot is busy, in error or
inhibited and owns the single active slot; `control` is accepted in any state
(stop, pause, resume); `query` is accepted in any state and moves nothing.

#### announce `joints` block (optional, 0.2.0)

When present, `joints` is an object requiring `unit` (`deg` or `rad`),
`names` (1 to 32 joint names, each matching `^[a-z][a-z0-9_]{0,31}$`, in
publication order), `limits` (null, or one `[minimum, maximum]` pair of numbers
per name in the announced unit) and `rate_hz` (a number greater than 0, the
node's maximum publish rate on the `joints` topic). A robot without joints, or
a node with `telemetry.joints_rate_hz` set to 0, omits the key. The schema
cannot compare the length of `limits` with the length of `names`; both halves
check it in code, and HA ignores a block whose lengths differ.

HA creates one sensor per name (unique id `{robot_id}_joint_{index}`,
index from 0), removes the sensors a later announce no longer lists, and
removes them all when a later announce drops the key.

### params subset

A task's `params` is a JSON Schema subset validated by hand-written code on
both sides, not by a general validator:

```json
{"type": "object",
 "properties": {"pose": {"type": "string", "enum": ["home", "rest"]},
                "speed": {"type": "number", "minimum": 0, "maximum": 1}},
 "required": ["pose"]}
```

`type` must be `object`; each property has `type` in `string`, `number`,
`integer`, `boolean` and may carry `enum`, `minimum`, `maximum` and
`description`; `required` lists property names. Nothing else is interpreted.

### availability

The plain bytes `online` or `offline`. Not JSON. The example files hold the
quoted strings `"online"` and `"offline"` because the validator reads JSON;
an implementation compares the raw bytes.

### state

Required: `schema`, `robot_id`, `state` (`idle`, `busy`, `error`), `task_id`
(UUID4 or null), `task` (task name or null), `since`, `paused`, `inhibited`,
`ha_link` (`up`, `down`), `watchdog_mode`, `executor_health` (`level` in `ok`,
`degraded`, `error` plus `message`), `error` (null or `code` plus `reason`),
`session`, `instance`, `seq`, `ts`.

`seq` increments per publication within a session; HA drops a state whose
`seq` regresses within the same session. A failed task alone never sets
`error`. HA uses `task_id` plus `paused` to suspend and resume its own result
deadline for that command.

### heartbeat (ROS to HA)

Required: `schema`, `robot_id`, `session`, `instance`, `seq`, `ts`, `uptime_s`,
`counters`. Optional: `ha_clock_offset_ms` (integer or null before any HA
heartbeat has been seen). `counters` requires `accepted`, `rejected`,
`duplicates`, `dropped_retained`, `dropped_oversize`, `dropped_invalid_id`,
`dropped_queue_full`, `publish_errors`, `takeovers`; new counters may be added
as optional fields.

### heartbeat (HA to ROS)

Required: `schema`, `ts`, `seq`, `interval_ms` (at least 1), `instance_id`.
Informational on the node unless `watchdog.mode` says otherwise; `ts` feeds
`ha_clock_offset_ms` in the robot heartbeat.

### joints (ROS to HA, optional, 0.2.0)

Required: `schema`, `robot_id`, `positions` (1 to 32 numbers, in the unit and
order of the announce `joints.names`), `velocities` (null, or one number per
position in the announced unit per second), `session`, `instance`, `seq`,
`ts`.

Published only by a node whose announce carries a `joints` block, at most
`telemetry.joints_rate_hz` times per second (default 2), and only when some
position moved by more than `telemetry.joints_min_delta` (default 0.5 for
`deg`, 0.01 for `rad`) since the last publish, plus once on every (re)connect
and announce. `seq` increments per publication within a session; HA drops a
`joints` payload whose `seq` regresses within the same session, one whose
`positions` length differs from the announced `names`, and one whose
`velocities` are not one per position (a mismatch is not a schema failure, so
it is not in the must-reject set). Never retained, 5 s
expiry, 16 KiB limit.

### cmd

A task command or a cancel command, told apart by `type`.

Common required fields: `schema`, `id` (UUID4), `type` (`task`, `cancel`),
`robot_id` (must equal the topic segment), `issued_at`, `max_age_ms` (at
least 1), `source` (`origin` and `kind` required; `instance_id` optional, a non-empty
string when present; `user_id`, `context_id`, `device_id` optional and
nullable). Optional: `sig` (64 hex
characters, present only when a shared secret is configured).

`type: task` adds `task` (required), `params` (required object, empty when
the task takes none) and `timeout_s` (optional; null or absent means the
announced value; a value is clamped to `command.task_timeout_cap_s`).

`type: cancel` adds `target_id` (required; a UUID4 or null for the active
motion task). `accepted` is the terminal outcome of the cancel's own id; the
cancelled task's result follows under its own id.

### ack

Required: `schema`, `id`, `robot_id`, `type`, `task` (null for a cancel or an
unreadable command), `accepted`, `code`, `reason`, `received_at`,
`clock_skew_ms` (integer or null), `session`, `ts`.

`accepted` is true exactly when `code` is `accepted`; the schema enforces this.
Codes (closed set): `accepted`, `rejected_malformed`, `rejected_unknown_task`,
`rejected_invalid_params`, `rejected_stale`, `rejected_future`,
`rejected_unauthorised`, `rejected_busy`, `rejected_error`,
`rejected_inhibited`, `rejected_executor`, `rejected_no_such_task`.

A repeated id republishes the stored ack. A command whose `id` is not a UUID4
gets no ack because there is no safe topic to publish to; it is counted as
`dropped_invalid_id`.

### feedback

Required: `schema`, `id`, `robot_id`, `progress` (0 to 1, or null), `stage`,
`message`, `session`, `ts`.

### result

Required: `schema`, `id`, `robot_id`, `task`, `kind`, `status`, `goal_status`,
`code`, `reason`, `data` (object), `started_at`, `finished_at`, `session`,
`ts`.

`status` maps onto `action_msgs/GoalStatus` and the schema enforces the pair:
`succeeded` 4, `cancelled` 5, `aborted` 6. `kind` repeats the task kind so HA
routes control and query results away from the `last_result` sensor. `code`
matches `^[a-z][a-z0-9_]{0,63}$`; the recommended vocabulary is `ok`,
`cancelled`, `preempted`, `collision`, `self_collision`, `no_trajectory`,
`planning_failed`, `control_failed`, `timeout`, `invalid_goal`,
`controller_error`, `inhibited`, `executor_error`, `executor_unresponsive`,
`nothing_to_pause`, `unknown`. `data` carries executor detail such as the raw
MoveIt error code. The HA-synthesised codes `no_ack`, `no_result`,
`node_restarted` and `ha_restarted` never travel on the wire.

### sensor (canonical path)

Required: `schema`, `entity_id`, `domain`, `object_id`, `state` (string or
null), `available`, `device_class` (nullable), `unit` (nullable),
`attributes` (object, filtered to the documented allowlist), `last_changed`
(ISO 8601 string or null), `ts`.

## Command signature (HMAC)

When a shared secret is configured on both sides, every command carries `sig`
and the node rejects any command whose signature is missing or wrong with
`rejected_unauthorised`.

```
canonical = json.dumps(payload_without_sig, sort_keys=True,
                       separators=(",", ":"), ensure_ascii=False).encode("utf-8")
message   = topic.encode("utf-8") + b"\n" + canonical
sig       = hex(HMAC-SHA256(secret.encode("utf-8"), message))
```

- `topic` is the full publish topic including the prefix, so a signature made
  for one robot or one prefix cannot be replayed on another.
- `sort_keys` applies at every nesting level; list order is preserved;
  non-ASCII characters are signed as their UTF-8 bytes, not as `\u` escapes.
- The comparison is constant-time on the lower-case hex strings.
- `hmac_vectors.json` holds seven vectors (a task command, a cancel command, a
  payload with non-ASCII text, a payload with nested params, the same payload
  on another topic, the same payload with another secret, and the signed
  example printed in `docs/mqtt-contract.md`) with the secret, topic, payload,
  canonical string and expected signature. Secrets in that file
  are placeholders for tests only.

## Freshness rule

The node accepts a command only when

```
-skew <= now - issued_at <= min(max_age_ms, command.max_age_cap_ms) + skew
```

with `skew = command.clock_skew_tolerance_ms` (default 2000 ms) and
`command.max_age_cap_ms` default 30000 ms. Outside the window the ack is
`rejected_stale` (too old) or `rejected_future` (issued ahead of the node's
clock) and `clock_skew_ms` carries the measured difference so HA can raise its
`clock_skew` repair. The persisted dedupe store is consulted before freshness,
so a redelivered command republishes its stored ack rather than being
downgraded to `rejected_stale`. Freshness depends on both machines keeping
time within the tolerance; a future `issued_at` is a runtime rejection, not a
schema failure, so it is not in the must-reject example set.

## Versioning rule

`schema` is `1` and stays `1`. Within schema 1 a change may only add optional
fields, add values to an open vocabulary (`code`, `source.kind`, counters) or
loosen a constraint that was tighter than intended. It may never remove or
rename a field, make an optional field required, add a value to a closed set
(`ack_code`, `result_status`, `task_kind`, `state`) that an older receiver
would have to understand, or change a topic. Receivers ignore unknown fields
so an older half keeps working against a newer one. Anything that breaks that
promise is schema 2 on a new `$id` base, published alongside schema 1 with a
migration note.

Additive changes so far within schema 1:

- 2026-10-14 (0.2.0): joint telemetry. The optional `joints` block in
  `announce`, the new `joints` payload on `{prefix}/{robot_id}/joints`, and the
  shared definitions `joint_unit`, `joint_name` and `joint_limits`. A 0.1.0
  Home Assistant ignores the block and never subscribes to the topic; a 0.1.0
  node never publishes either, and a 0.2.0 Home Assistant then makes no joint
  sensors. Nothing that existed in 0.1.0 changed.

The schema `$id` values live under `https://ha-ros2-bridge.invalid/contract/1/`.
The `.invalid` top-level domain is reserved and never resolves; the ids are
identifiers only and nothing is fetched over the network.

## Running the validator

From the repository root, with the project virtual environment active or by
naming its interpreter:

```
python scripts/validate_contract.py
python scripts/validate_contract.py --quiet
python scripts/sync_contract.py --check
```

`validate_contract.py` checks that every schema compiles as draft 2020-12 with
its `$ref` values resolving, that every file under `examples/valid/<kind>/`
validates and is within its size limit, that every file under
`examples/invalid/<kind>/` is rejected (by its schema or by the size limit),
and that every HMAC vector recomputes. It exits non-zero on any mismatch with a
per-file report. It needs only `jsonschema` (4.18 or newer, for the
`referencing` registry).

`sync_contract.py --check` confirms that the ROS 2 package's vendored copy of
the schemas is byte-identical to `schemas/`; run it without `--check` to
refresh the copy after editing a schema. CI runs both.

From tests, import the module and call `validate(kind, payload)`, which returns
a list of error strings (empty means valid), or `compute_sig(secret, topic,
payload)` for the signature.
