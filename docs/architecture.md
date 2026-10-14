# Architecture

ha-ros2-bridge is one Apache-2.0 monorepo with a written MQTT contract consumed by two halves. Neither half imports the other; the contract (`contract/`, readable form in [mqtt-contract.md](mqtt-contract.md)) is the only thing they share, and both are tested against the same fixtures.

```text
 Home Assistant OS                          Mosquitto                    ROS 2 machine (Ubuntu 24.04, Jazzy)
 +-----------------------------+          +-----------+          +-----------------------------------+
 | custom_components/          |  MQTT 5  |           |  MQTT 5  | ros2/ros2_ha_bridge               |
 |   ros2_bridge (HACS)        |<-------->|  broker   |<-------->|   bridge_node, one process per    |
 |   rides HA's own MQTT       |          |  (add-on) |          |   robot, paho-mqtt 2.1.0 + rclpy  |
 |   connection; no pip deps   |          |           |          |   TaskExecutor plugin (fake,      |
 +-----------------------------+          +-----+-----+          |   MoveIt 2, custom)               |
   entities, services, events,                  ^                +-----------------+-----------------+
   Assist intents, repairs                      |                                  | ROS 2 (DDS, local only)
                                   Zigbee2MQTT, ESPHome,                          v
                                   other MQTT publishers           move_group / controllers / Isaac Sim
```

## The two halves and the contract

**ROS 2 side** (`ros2/ros2_ha_bridge/`, ament_python, Jazzy). One `bridge_node` process per robot. `BridgeNode(rclpy.node.Node)` is wiring only. The modules that do the work (`protocol.py`, `authoriser.py`, `task_manager.py`, `dedupe_store.py`, `watchdog.py`, `sensor_mirror.py`, `executors/base.py`, `executors/fake.py`) import neither rclpy nor paho, so they are unit-tested on any machine, including native Windows Python with no ROS installed. The node announces the robot and its task catalogue, authorises and runs commands through a `TaskExecutor` plugin, and publishes state, availability (Last Will), a heartbeat, and per-task retained acks and results with MQTT 5 message expiry. One process per robot means a stalled executor stalls only its own robot.

**Home Assistant side** (`custom_components/ros2_bridge/`, HACS). A thin custom integration that rides HA's own MQTT connection through the `mqtt` component helpers (the pattern tasmota uses), with no pip requirements and no rclpy. One device per announced robot; `state`, `last_result` and `problem` entities; a `task_result` event entity; one button per parameterless task plus `cancel`; services `send_task`, `cancel_task` and `get_robots`; three bus events; two Assist intents; diagnostics and five repair issues. It is a push hub with no coordinator: seven wildcard subscriptions at setup, a `CommandRegistry` backed by a `Store` so every command id ends in exactly one named outcome across HA restarts and options reloads, and reconciliation after the retained replay burst.

**Contract** (`contract/`). JSON schemas for every payload, golden and must-reject examples, HMAC test vectors. Schema 1 only ever gains optional fields. Both halves carry a `protocol.py` that must produce identical canonical JSON and signatures; `contract.yml` checks it.

Why this shape and not an add-on: every living HA-to-ROS2 precedent is a purpose-built ROS node publishing MQTT discovery; ROS 2 cannot usefully run inside an HA add-on (host networking, distro coupling); an add-on cannot create HA entities or services by itself; stock MQTT discovery alone cannot give non-admin users or Assist a typed task action (`mqtt.publish` is admin-only since HA 2026.10); and HA's own MQTT connection means adopters enter broker details once, in HA. The reasons are recorded with dates in [decisions.md](decisions.md).

## Threading model (ROS 2 node)

```text
paho network thread (loop_start)                   rclpy SingleThreadedExecutor (main thread)
 on_message: size check, append to deque(maxlen)   GuardCondition callback drains the deque:
   (topic, payload, retain, rx_ts), guard.trigger()  cmd -> Authoriser -> TaskManager -> executor.start/cancel
   full deque: drop, count dropped_queue_full        heartbeat -> Watchdog; ha status -> re-announce after jitter
 on_connect / on_disconnect: append event, trigger   mirror -> SensorMirror -> ~/mqtt_in
 MqttLink.publish() called only from the main        timers: watchdog tick 0.5 s, health poll 1 s, heartbeat 2 s,
   thread; every call wrapped in try/except          per-task timeout and cancel grace (self-cancelling timers)
   (ValueError, OSError): ERROR log, publish_errors  TaskHandle.finish/feedback from a plugin thread -> ctx.post -> deque
```

- The paho thread never touches robot state. It does a size check, appends `(topic, payload, retain, rx_ts)` to a bounded deque (`mqtt.inbound_queue_max`, 256) and triggers an rclpy `GuardCondition`. Triggering a guard condition from another thread is documented as safe by rcl. A full deque drops the message and counts `dropped_queue_full`.
- All robot state lives on the main thread, and plugin methods are called there too. Plugin calls are timed: longer than `executor.call_warn_s` (0.1 s) logs a warning and marks `executor_health` degraded; a call that blocks for ever stalls this robot only, its heartbeat stops and HA shows `problem` within about 6 s. There is deliberately no thread-pool guard around plugin calls (recorded decision).
- A plugin that does its work on its own thread (an action client callback, for example) reports back through `TaskHandle.feedback` and `TaskHandle.finish`, which post a closure onto the same deque via `BridgeContext.post`, so the result is applied on the main thread like everything else.
- rclpy Jazzy has no one-shot timer (verified), so every one-shot (task timeout, cancel grace, fake task completion, announce jitter) is a periodic timer whose callback calls `timer.cancel()` as its first line. Pause cancels the timer and stores the remaining period; resume creates a fresh timer for the remainder.
- No publish may end the process. `MqttLink.publish` validates the topic against the contract patterns first (a wildcard or over-long topic is impossible by construction), then catches `ValueError` and `OSError` from paho, logs at ERROR with the topic, increments `publish_errors` and returns False.

## Data flows

### Fast sensor path (Zigbee2MQTT to ROS 2, no HA in the loop)

```text
Zigbee device -> Zigbee2MQTT -> zigbee2mqtt/{name} on Mosquitto
                                   |-> HA's MQTT integration (entities, automations)      in parallel
                                   |-> bridge node (mirror.topics) -> deque -> SensorMirror -> ~/mqtt_in
```

The node subscribes to each configured `mirror.topics` entry and its availability sibling at QoS 0 and republishes the payload unchanged as a `std_msgs/String` envelope on `~/mqtt_in` with the source topic and `source_available`. Any ROS 2 node, the executor included, can subscribe. HA receives the same message from the broker at the same time, so neither side waits for the other. This is the path for raw events where every millisecond matters (Q3).

#### Sensor mirror envelope

`SensorMirror` (`sensor_mirror.py`) turns every message on a mirrored topic, or on its availability sibling, into one JSON object with these keys, always present and always in this order:

```json
{"topic": "zigbee2mqtt/kitchen_motion",
 "payload": "{\"occupancy\":true}",
 "encoding": "utf-8",
 "retain": false,
 "source_available": true,
 "rx_ts_ms": 1760356800123}
```

- `payload` is the message decoded as UTF-8 and `encoding` is `utf-8`; when the bytes are not UTF-8, `payload` is their base64 form and `encoding` is `base64`. Nothing is reformatted in between.
- `source_available` is the last availability seen on the topic's sibling (Zigbee2MQTT's `{"state":"online"}` and the plain word `online` are both understood, as are both `offline` forms), or `null` when no sibling message has been seen. An empty availability payload (a retained clear) resets it to `null`.
- `rx_ts_ms` is the epoch millisecond the node received the message; `retain` is the delivery's RETAIN flag.
- An availability message is mirrored as an envelope too, so a consumer learns that a sensor went offline without waiting for a data message that will not come.
- Nothing raises on an inbound message: a bad topic, a bad payload type, an oversize message, an unparseable availability word or a failing output callable is logged and counted. Only the configuration (the topic list and the suffix) is validated eagerly, with `TypeError` for a wrong type and `ValueError` for a bad value, because a bad filter is an operator error that should stop the node at start-up rather than silence a sensor path.

### Normalised sensor path (any HA entity to ROS 2)

```text
Any HA entity (ZHA, Z-Wave, Thread, template, ...) -> HA state machine
   -> ros2_bridge sensors_out (mirror_entities option) -> {prefix}/ha/sensors/{domain}/{object_id}, retained QoS 0
   -> bridge node (if listed in mirror.topics) -> ~/mqtt_in
```

HA republishes the chosen entities in one canonical JSON shape (see the `sensors` payload in the contract) so the ROS 2 side sees one schema regardless of the sensor's radio. Retained, so a newly started node sees the current value at once. Topics are cleared when an entity leaves the option, when the prefix changes, or when the integration is removed. Derived and timed triggers ("no motion for 5 minutes") stay in HA automations, which then use the command path.

### Command path (HA to robot)

```text
Button press | automation calling send_task | Assist intent
   -> integration: resolve robot, validate task and params against the announced catalogue,
      build command (UUID4 id, issued_at, max_age_ms, source, optional sig)
   -> Store.async_save (command outstanding before anything leaves HA)
   -> mqtt.async_publish {prefix}/{robot}/cmd, QoS 1, not retained, expiry ceil(max_age_ms/1000)
   -> broker -> node paho thread: size check, deque, guard trigger
   -> node main thread: Authoriser steps 0 to 9 in execution order (retained drop, oversize, UUID id,
      dedupe, schema, robot id, HMAC, freshness, allowlist and params, state gating)
   -> TaskManager -> executor.start(req, handle)
   -> ack published retained (120 s expiry) -> state busy published retained
   <- HA: ack matched by id, sent -> running, ros2_bridge_task_ack fired, wait: ack callers released
```

The first failing step wins and the ack says which. Dedupe runs the moment the id is known, before the schema, signature and freshness checks, so a replayed id always gets the stored ack bytes back and a later malformed or unsigned replay can never overwrite a stored `accepted` ack. A command whose id is not a UUID gets no ack at all because there is no safe topic to answer on; it is counted instead. The full order is in [mqtt-contract.md](mqtt-contract.md#command-handling-on-the-node).

### Result path (robot to HA)

```text
Executor finishes (any thread) -> TaskHandle.finish(TaskResult) -> ctx.post -> deque -> main thread
   -> result published retained (300 s expiry) with status, goal_status, code, reason, data, kind
   -> state idle (or error on a forced finish) published retained
   <- HA hub: result matched by id, seen_results checked, ros2_bridge_task_result fired exactly once,
      last_result updated (motion only), task_result event entity triggered, wait: result callers released
```

Feedback travels the same way at QoS 0, throttled to 2 Hz, and appears as attributes on the state sensor. If the robot never answers, HA synthesises `timeout` (`no_ack` or `no_result`) or `lost` (`node_restarted`) from its own deadlines and the robot's `session`, and only then; a real result arriving after a synthesised outcome fires `ros2_bridge_task_late_result` so `task_result` still fires exactly once per id.

## Executor plugin interface (summary)

The executor is the only part of the system that knows how an arm moves. It lives in `ros2_ha_bridge/executors/base.py` and is loaded by name from the entry-point group `ros2_ha_bridge.executors` or as `package.module:Class`. The full interface with its docstrings is in that file and in `docs/writing-an-executor.md` (phase 7); the essentials:

| Type | Purpose |
|---|---|
| `TaskSpec` | Frozen dataclass, one catalogue entry: `name`, `label`, `kind: TaskKind = 'motion'` (`motion`, `control`, `query`), `timeout_s: float = 60.0`, `params: Mapping | None = None` (a params-subset JSON Schema, `None` for a task without parameters), `description: str = ''`. Checked with `protocol.validate_task_spec` at start-up. |
| `TaskRequest` | Frozen dataclass handed to `start`: `id`, `task`, `params`, `issued_at`, `timeout_s` (already resolved: the command's value or the announced default, clamped to the cap), `source` (HA's record of who raised the command; never interpreted). |
| `TaskResult` | Frozen dataclass passed to `finish`: `status: ResultStatus` (`succeeded`, `cancelled`, `aborted`), `code: str`, `reason: str = ''`, `data: Mapping = {}`; the `goal_status` property derives 4, 5 or 6. `code` is normalised before it reaches the wire. |
| `TaskRejected(code='rejected_executor', reason='')` | Raise from `start` to refuse a task; the ack is `rejected_executor` with the given reason (`code` is logged, not sent) and the robot stays idle. |
| `TaskHandle` | Protocol, one per task, every method thread-safe: `task_id`, `feedback(progress=None, stage='', message='')` (throttled by the bridge) and `finish(result)` (exactly once). Calls after the terminal finish, a second `finish` included, are logged and ignored. |
| `BridgeContext` | Given to `setup`: attributes `robot_id`, `logger`, `node` (the rclpy node in the real bridge, `None` in unit tests), `clock`, `scheduler`, `parameters` (read-only mapping keyed by dotted names, for example `fake.task_duration_s`); thread-safe methods `post(fn)`, `set_error(code, reason)`, `clear_error()`, `set_paused(paused)`, `ha_link_ok()`. |
| `TaskExecutor` | Abstract base with class attributes `name` and `version` (announced to HA). Abstract: `setup(ctx)` (blocking waits allowed here only), `tasks() -> Sequence[TaskSpec]`, `start(req, handle)` (must return promptly; may call `handle.finish` before returning), `cancel(task_id)` (must lead to `finish(cancelled)` within `executor.cancel_grace_s`, else the bridge forces `aborted/executor_unresponsive`). Defaults: `stop_and_hold(reason)` does nothing, `health() -> (level, message)` reports `('ok', '')`, `shutdown()` does nothing. |
| `load_executor(spec)` | Resolves `package.module:Attribute` or an entry-point name in `ros2_ha_bridge.executors` to an instance (a concrete subclass is instantiated with no arguments); every failure raises `ExecutorLoadError`, which the node treats as fatal. |

The shipped fake executor implements `reset`, `move_to {pose}`, `wait {seconds}`, `fail {code}`, `hang`, `raise_error`, `clear_error`, `stop`, `pause` and `resume`, deterministically with an injected clock, so the whole HA side can be exercised with no ROS 2 or simulator present. A MoveIt 2 executor drives `/move_action` through a plain rclpy action client: `send_goal_async` inside `start`, feedback callback to `handle.feedback`, `get_result_async` mapping GoalStatus 4, 5, 6, `cancel_goal_async` from `cancel`, and `MoveItErrorCodes` mapped to the result vocabulary with the raw value in `data`.

## Where safety sits

HA is advisory; ROS 2 owns safety. The HA-to-robot heartbeat is informational by default (`watchdog.mode: none`, Q16): HA going away must not affect the arms. Adopters who want the opposite set `inhibit_new` or `stop_and_hold`. The robot-to-HA direction (availability via Last Will, retained state, heartbeat staleness) is always on, so HA can tell a dead node from a quiet one. Hardware e-stop, protective stop and collision handling belong to the arm controller and are outside this project; nothing on MQTT is labelled e-stop. The authorisation model is in [SECURITY.md](../SECURITY.md).

## Measured latency

First measurement, 2026-10-14, with `tools/latency_probe.py` on the development PC: broker is the Mosquitto add-on on the Home Assistant OS box, the bridge node runs in Docker Desktop on a Windows PC on the same wired LAN, so the node's half of the path crosses the Docker VM's network stack.

| Measure (20 commands, 14 heartbeat intervals) | min | median | p95 | max |
|---|---|---|---|---|
| Command publish to retained ack seen (round trip through the broker and the node) | 41 ms | 49 ms | 59 ms | 61 ms |
| Publish to PUBACK (broker hop, QoS 1) | 0.8 ms | 1.5 ms | 2.4 ms | 41 ms |
| Heartbeat period as received (nominal 2000 ms) | 1971 ms | 2010 ms | 2073 ms | 2073 ms |
| Heartbeat jitter (absolute) | 2.5 ms | 23 ms | 73 ms | 73 ms |

Reading: one way from a publish on the broker to the node acting on it is about half the round trip, roughly 25 ms, well inside the provisional 200 ms fast-path target. The broker itself adds about a millisecond. Clock skew reported by the node ranged from -59 ms to +14 ms over the run; the Docker VM's clock drifts against the HA host by a few hundred milliseconds between corrections, which is a test-rig artefact (a real site runs NTP on the ROS 2 host).


Provisional targets from the plan (Q8), to be confirmed or tightened by Jonny once `tools/latency_probe.py` has run on a real LAN:

| Path | Target |
|---|---|
| Fast sensor path, broker to node callback | under 50 ms |
| Fast sensor path, end to end (radio event to executor callback) | under 200 ms |
| HA-automation path (derived or timed triggers) to executor callback | under 1 s |

The sensor radio, not the bus, is expected to dominate. Measured numbers, the date, the machines and the method will be recorded here at phase 5, after which the heartbeat and watchdog defaults are re-derived and the change logged in [decisions.md](decisions.md).
