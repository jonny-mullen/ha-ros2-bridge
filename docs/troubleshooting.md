# Troubleshooting

Symptom first, then the likely causes in the order worth checking, then how to confirm. The two tools that settle most questions are the diagnostics download (what Home Assistant believes) and `mosquitto_sub` on the broker (what is actually there); both are at the end of this page, as are the log locations.

## Symptom to cause

| Symptom | Likely cause | How to confirm and fix |
|---|---|---|
| No robot appears after adding the integration | The node has not announced: it is not running, or cannot reach the broker | `mosquitto_sub` on `{prefix}/+/announce` shows nothing retained. Check the node's log for `connected to the MQTT broker as ...`; a refused connection is logged once in plain words. |
| | Prefix mismatch between the node's `topic_prefix` and the integration's Topic prefix option | The announce is retained under a prefix the integration is not subscribed to. Subscribe to `#` briefly to see where it went, then align the two prefixes. |
| | The MQTT integration is not connected | The entry raises "not ready" on setup and retries; every bridge entity is unavailable. Fix the MQTT integration first (Settings, Devices and services, MQTT). |
| | The announce is invalid | The repair "Robot {robot_id} sent an invalid announce" is open with the reason. Update or restart the node. |
| | The robot id is `ha`, or does not match `^[a-z][a-z0-9_]{0,31}$` | The node refuses `ha` at start-up; the integration discards other bad segments and counts them under `discarded_segments` in diagnostics. Rename the robot. |
| A robot's entities are unavailable | The node is offline: its Last Will set the availability topic to `offline` | `mosquitto_sub` on `{prefix}/{robot_id}/availability` shows `offline`. Start the node. |
| | The heartbeat is stale: the node is connected but hung, or the broker went down at the moment the node died (no Last Will) | Diagnostics show `heartbeat_unavailable: true` and a large `heartbeat_age_s`; the log says "Robot ... is unavailable: no heartbeat for N s". Restart the node. The threshold is the option "Unavailable after missing heartbeats for" (default 30 s). |
| | Home Assistant's MQTT client is disconnected | Every robot's entities go unavailable together; the log says "MQTT disconnected; robots are unavailable until it returns". Check the broker and the MQTT integration. |
| | The robot has not announced since Home Assistant started | Availability needs an announce in this run. A node that only publishes state but whose retained announce was cleared needs a restart so it announces again. |
| The problem sensor is on | The robot reports `error` (a forced finish or an executor fault) | The State sensor shows Error with `error_code` and `error_reason`. For the fake executor the `clear_error` task clears it; a real executor clears it when its `clear_error` hook is called. |
| | The executor health is `error` | `executor_health` on the State sensor. The executor plugin's own log says why. |
| | The heartbeat is stale while the availability still says `online` | `reasons` lists `heartbeat_stale`. Three missed heartbeat intervals trip it, long before the unavailability threshold. |
| | The `duplicate_robot_id` repair is open | See the repair below. |
| A button press shows an error | The robot refused the command | The error text carries the acknowledgement code and reason (table below). |
| | No acknowledgement within the timeout | "did not acknowledge the command in time; the outcome is unknown". See `no_ack` below. |
| | The entry was reloaded while the press was pending | "The ROS 2 Bridge is not loaded". Press again. |
| `send_task` fails before anything is sent | Unknown robot, unavailable robot, unknown task, invalid parameters, or the per-robot pending cap | These are validation errors and nothing was published, to any of the targets. The message names the robot, the task and the problem. |
| Commands are published but never answered | The node ignores them: the `cmd` topic is right but the payload's `robot_id` must equal the topic's robot segment | Watch `{prefix}/{robot_id}/cmd` and the node's heartbeat counters: `dropped_invalid_id`, `dropped_oversize`, `dropped_retained`, `dropped_queue_full` say which guard dropped it. |
| Every command is rejected as stale or from the future | Clock skew between the two machines | The `clock_skew` repair opens with the measured skew. NTP on both; `tools/clock_check.py` on the robot machine measures the real offset. |
| Every command is rejected as unauthorised | Shared secret mismatch, or the robot requires a signature and the option is empty | The `hmac_rejected` repair opens with which of the two it is. |
| A task "succeeded" but the arm did not move | The `fake` executor is still selected on the node | It is the shipped default and moves nothing. A real executor is a deliberate change to the node's `executor` parameter. |
| The Last result sensor did not change after a stop or pause | By design | Control and query results update the Task result event and the bus event, never the Last result sensor, which shows motion tasks only. |
| A result event fired twice | It did not: the second is `ros2_bridge_task_late_result` | A real result that arrived after Home Assistant had already synthesised an outcome fires the late event, so `ros2_bridge_task_result` still fires exactly once per id. |

## Rejection codes

A command's acknowledgement carries one of a closed set of codes (`docs/mqtt-contract.md`). Home Assistant surfaces it as the `code` of a `rejected` outcome, in the button's error, in the action's response and in `ros2_bridge_task_ack`.

| Code | The node is saying | What to do |
|---|---|---|
| `accepted` | The command was taken on | Nothing; the result follows. |
| `rejected_malformed` | The payload failed schema 1, or its `robot_id` did not equal the topic's | Mismatched bridge versions, or a hand-built command. Check both halves are on the same contract schema. |
| `rejected_unknown_task` | The task is not in the announced catalogue (after the operator's `tasks.allow` intersection) | Use a task from the robot's Get robots catalogue, or widen `tasks.allow` on the node. |
| `rejected_invalid_params` | The parameters failed the task's announced schema on the node | Home Assistant validates them first, so this normally means the catalogue changed between the announce and the command. |
| `rejected_stale` | The command was older than `min(max_age_ms, command.max_age_cap_ms)` plus the skew tolerance when the node processed it | Clock skew, a slow broker, or a node whose queue is backed up. The `clock_skew` repair opens; check NTP. |
| `rejected_future` | The command's `issued_at` is ahead of the node's clock by more than the tolerance | Clock skew the other way. Same fix. |
| `rejected_unauthorised` | A shared secret is configured on the node and the signature was missing or wrong | The `hmac_rejected` repair opens. Make the secrets equal, or clear both. |
| `rejected_busy` | A motion task was sent while another motion task is running | One motion at a time, no queue. Wait for the result, or cancel the running task first. |
| `rejected_error` | A motion task was sent while the robot is in `error` | Clear the error first (the executor's `clear_error`; for the fake executor the `clear_error` task). |
| `rejected_inhibited` | The node's watchdog is in `inhibit_new` or `stop_and_hold` mode and Home Assistant's heartbeat is not arriving | Restore the heartbeat (the Heartbeat interval option must not be 0, and MQTT must be connected) or change `watchdog.mode` on the node. |
| `rejected_executor` | The executor plugin refused to start the task, or raised | The plugin's reason is in the acknowledgement; the traceback is in the node's log. |
| `rejected_no_such_task` | A cancel named a task that is not running | The task had already finished, or the id was wrong. |

## `timeout` with code `no_ack`: outcome unknown

If no acknowledgement arrives within the Acknowledgement timeout option (default 2 s), Home Assistant reports the command as `timeout` with code `no_ack`. This means "outcome unknown", not "failed": the command may have been lost on the way (nothing happened), or it may have been accepted and the acknowledgement lost on the way back (the task is running). The design keeps the two apart on purpose (open question (d) in `docs/decisions.md`).

What happens next: the node retains every acknowledgement for 120 s. If an `accepted` acknowledgement turns up late, the command is re-opened as `running`, `ros2_bridge_task_ack` fires with `late: true`, and the real result fires `ros2_bridge_task_late_result` when it arrives, so an automation that cares can still learn the truth. If nothing turns up, the command stays `timeout`.

Causes worth checking: Home Assistant's MQTT client disconnected at the moment of publish (the error says so, and paho resends the QoS 1 message on reconnect); the node hung (the problem sensor shows `heartbeat_stale`); a prefix mismatch, which shows as a robot that also never updates its state.

## Repairs

Settings, System, Repairs. None has a fix button; each closes by itself.

### Clock skew with robot {robot_id}

Opened when an acknowledgement reports `clock_skew_ms` of 1000 or more in either direction, or when a command is rejected as `rejected_stale` or `rejected_future`. The node's tolerance is `command.clock_skew_tolerance_ms` (default 2000); beyond it commands are refused. Put both machines on NTP and check with `tools/clock_check.py` on the robot machine. The issue closes once an acknowledgement arrives with an acceptable difference.

### Robot {robot_id} refused the command signature

Opened when a command is rejected as `rejected_unauthorised`, or when a robot announces `hmac_required: true` while the Shared secret option is empty. The description says which: "the robot refused the signature (secret mismatch)" or "the robot requires signed commands". The secret must be identical on both sides (`ROS2_HA_BRIDGE_COMMAND_SECRET` on the node, the Shared secret option here) or empty on both. It closes once the robot accepts a command, or announces without requiring a signature.

### Robot {robot_id} sent an invalid announce

The retained message on the robot's announce topic does not decode as a schema 1 announce, or its `robot_id` differs from the topic. The robot is not shown until it announces correctly. Check the bridge versions on both sides and the executor's task catalogue (task names must match `^[a-z][a-z0-9_]{0,31}$`, parameter schemas the documented subset), then restart the node. Closes once a valid announce arrives.

### The MQTT connection does not support message expiry

Publishing a command with a message expiry interval was refused because Home Assistant's MQTT session is not MQTT 5. The command was republished without the expiry, so a stale command can sit in the broker until a robot reads it; the robots still refuse commands older than their maximum age, so nothing unsafe follows, but the defence in depth is thinner. Set the MQTT integration's protocol to 5 (Settings, Devices and services, MQTT, Configure, advanced options) and make sure the broker supports it (the Mosquitto add-on does). Raised once for the connection, not per robot. Closes once a publish with an expiry succeeds.

### Two robot nodes may be using the id {robot_id}

Opened on either of two pieces of evidence: two different `instance` values (`hostname:pid`) seen for one robot within a heartbeat interval, or more than three `session` changes within five minutes. Two nodes with one `robot.id` share one MQTT client id, so they take each other's broker session over in turn, and their state, acknowledgements and results interleave; the node logs `Session taken over` and exits after `mqtt.takeover_exit_after` takeovers so the launch file's respawn limit can stop the flapping.

A node that crashes and restarts repeatedly trips the same check, because every restart is a new session, so this repair can also mean one node in a crash loop: look at the node's log before looking for a second machine. Give each robot its own `robot.id`, or fix the crash, then restart. The problem sensor shows `duplicate_robot_id` while the repair is open. Closes after ten quiet minutes.

## Reading the diagnostics

Settings, Devices and services, ROS 2 Bridge, the three-dot menu, Download diagnostics gives the whole picture; the same menu on a robot's device page gives that robot alone. The shared secret never appears (only `secret_configured: true/false`) and user ids are redacted.

| Section | What to look at |
|---|---|
| `entry.options` | The options as saved; check the prefix first. |
| `hub.mqtt_connected`, `hub.ready`, `hub.reconciled` | All three should be true a few seconds after start. `ready` false means the ack and result subscriptions were never confirmed; `reconciled` false means the 3 s settle period after readiness and connection has not passed. |
| `hub.subscriptions` | The seven filters with their QoS; they must carry the prefix the robots use. |
| `hub.heartbeat` | `seq` rising and `last_sent_at` recent means Home Assistant is publishing its heartbeat. |
| `hub.robots.{id}.availability`, `.available`, `.heartbeat_age_s`, `.heartbeat_stale`, `.heartbeat_unavailable` | Why the entities are or are not available. |
| `hub.robots.{id}.session`, `.instance`, `.session_change_count` | A rising change count is a restarting node or a duplicate id. |
| `hub.robots.{id}.counters` | The node's own counters from its heartbeat: `accepted`, `rejected`, `duplicates`, `dropped_retained`, `dropped_oversize`, `dropped_invalid_id`, `dropped_queue_full`, `publish_errors`, `takeovers`. |
| `hub.robots.{id}.last_ack`, `.last_result`, `.pending`, `.problem_reasons` | The most recent reply, the last motion outcome, outstanding command ids, and why the problem sensor is on. |
| `hub.commands` | The last 50 commands with their state, codes and deadlines. |
| `hub.counters` | The integration's own: `discarded_segments`, `invalid_payloads`, `mismatched_payloads`, `seq_regressions`, `handler_errors`, `publish_errors`, `heartbeats_sent`, plus the registry's `unmatched_replies`, `replayed_results_ignored`, `late_results`, `duplicate_acks`, `synthesised`. |
| `hub.issues` | The open repair ids. |
| `store` | The persisted document: outstanding commands, seen result ids and the sensor topics published. |

## Watching the topics with mosquitto_sub

From any machine that can reach the broker, with `mosquitto-clients` installed. Replace the host, user and password with your broker's; use MQTT 5 (`-V 5`) so expiry and retain-as-published behave as the bridge expects.

Everything under the prefix, with topic names:

```text
mosquitto_sub -h broker.example.invalid -p 1883 -u changeme -P 'the-broker-password' -V 5 -v -t 'ros2_bridge/#'
```

On subscribing you should see the retained set at once: each robot's `availability` (`online` or `offline`), `announce` (the catalogue, a `session` and an `instance`) and `state`; then a heartbeat per robot every two seconds, and Home Assistant's own heartbeat on `ros2_bridge/ha/heartbeat`. Useful variations:

| Purpose | Command |
|---|---|
| Only the retained set, then exit | add `--retained-only` |
| Only live traffic, no retained replay | add `-R` |
| One robot | `-t 'ros2_bridge/arm_left/#'` |
| Commands Home Assistant sends | `-t 'ros2_bridge/+/cmd'` (never retained; if `--retained-only` shows one here, something planted it and the node will drop and clear it) |
| Replies for one task id | `-t 'ros2_bridge/arm_left/task/<task_id>/#'` |
| The normalised sensors | `-t 'ros2_bridge/ha/sensors/#'` |
| Where a mis-prefixed robot went | `-t '#' --retained-only` and look for `/announce` |

To clear a retained message by hand (for example a robot's announce after the robot is gone for good), publish an empty retained payload:

```text
mosquitto_pub -h broker.example.invalid -p 1883 -u changeme -P 'the-broker-password' -V 5 -r -n -t 'ros2_bridge/arm_left/announce'
```

Never publish anything retained to a `cmd` topic.

## Where the logs are

**Home Assistant.** Settings, System, Logs, and filter on `ros2_bridge`. The integration logs robot arrivals, departures and offline transitions at info, stale heartbeats and failed publishes at warning, and duplicate-id evidence at error. For the per-message detail (discarded topics, dropped sequence numbers, invalid payloads) turn on debug logging from the integration's page (the three-dot menu, Enable debug logging; disabling it again downloads the log), or in `configuration.yaml`:

```yaml
logger:
  default: warning
  logs:
    custom_components.ros2_bridge: debug
    homeassistant.components.mqtt: info
```

**The robot node.** Whatever started it: the terminal for `ros2 launch`, `journalctl -u <unit>` for a systemd service, `docker compose logs` for the Docker rig in `tools/`. The launch file takes a `log_level` argument for more detail. The node's heartbeat counters, visible in the diagnostics and on the heartbeat topic, say which guard dropped a command without needing the log at all.

**The broker.** The Mosquitto add-on's log (Settings, Add-ons, Mosquitto broker, Log) shows connections, disconnections and, for a duplicate robot id, the repeated session takeovers of one client id.
