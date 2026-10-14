# End-to-end checklist: the Docker rig against a live Home Assistant

This is the manual end-to-end procedure from the design's testing strategy (steps 0 to 13), written against the names the code uses. It runs the two fake arms of `tools/compose.yaml` against the broker a live Home Assistant 2026.10.1 already uses, with the `ros2_bridge` integration installed on that Home Assistant. Nothing in it moves an arm. Fill in the results table at the end with the date and the outcome of each step; a step that is skipped is recorded as skipped, with the reason.

Status: first run on 2026-10-14 against the live Home Assistant 2026.10.1 and the Mosquitto add-on, with the two fake arms in Docker on the development PC; results and notes are in the table at the end.

## Safeguards (read before step 0)

1. **Fake executor only.** Both containers run `executor: fake`, the default. Do not change it; the `executor` parameter is never touched during this procedure.
2. **Existing MQTT user.** The rig uses the broker user Home Assistant already has (interview decision Q15). `tools/.env` holds the broker host, port, user name and password; it is ignored by git and is never pasted into chat, a commit or this file.
3. **No real arms.** Robot ids are `fake_arm_1` and `fake_arm_2`, device names "Fake arm 1" and "Fake arm 2". No other robot id is used.
4. **Nothing retained on `cmd`, except the deliberate plant in step 6.** Commands are published without the retain flag everywhere else. After step 6 the node has cleared the plant; confirm it with the retained-only subscription given there before moving on.
5. **Home Assistant writes go through the documented route.** Files under the Home Assistant configuration folder (the integration itself, the sentences file) are written with the HA MCP tools: `ha_get_skill_guide` first, then `ha_write_file`. The Samba share is the last resort and only with Jonny's confirmation. Options are changed through the integration's own options page (or the equivalent MCP call), never by editing files.
6. **`.storage` is never touched.** The integration's own store (`.storage/ros2_bridge.commands`) is read through the diagnostics download; nothing under `.storage` is opened, edited or deleted by hand.
7. **Scratch topics stay under the reserved `ha` segment.** Where a step needs a topic of its own (step 10), it uses `ros2_bridge/ha/e2e/...`, which neither the node nor the integration subscribes to.
8. **Clean up at the end.** `docker compose down` removes the arms, one-off containers and the network; retained acks and results expire by themselves (120 s and 300 s); the scratch topics of step 10 are cleared with an empty retained publish; options changed during the run are set back.

## Names used below

| Thing | Name |
|---|---|
| Robots | `fake_arm_1` (service `arm1`), `fake_arm_2` (service `arm2`) |
| Topic prefix | `ros2_bridge` (the default on both sides) |
| Entities of fake_arm_1 (expected ids, generated from the device name; confirm on the device page) | `sensor.fake_arm_1_state`, `sensor.fake_arm_1_last_result`, `binary_sensor.fake_arm_1_problem`, `event.fake_arm_1_task_result`, `button.fake_arm_1_reset_to_home`, `button.fake_arm_1_hang`, `button.fake_arm_1_raise_error`, `button.fake_arm_1_clear_error`, `button.fake_arm_1_stop`, `button.fake_arm_1_pause`, `button.fake_arm_1_resume`, `button.fake_arm_1_cancel_task` |
| Tasks without a button (required parameters) | `move_to {pose}`, `wait {seconds}`, `fail {code}`; reachable through `ros2_bridge.send_task` |
| Actions | `ros2_bridge.send_task`, `ros2_bridge.cancel_task`, `ros2_bridge.get_robots` |
| Bus events | `ros2_bridge_task_ack`, `ros2_bridge_task_result`, `ros2_bridge_task_late_result` |
| Options (label, key, default) | Topic prefix `topic_prefix` `ros2_bridge`; Shared secret `shared_secret` empty; Heartbeat interval `heartbeat_interval_s` 2; Acknowledgement timeout `ack_timeout_s` 2.0; Command maximum age `command_max_age_ms` 5000; Result timeout margin `result_timeout_margin_s` 10; Maximum pending commands per robot `max_pending_per_robot` 16; Unavailable after missing heartbeats for `heartbeat_unavailable_after_s` 30; Mirrored entities `mirror_entities` none |
| Repair ids | `clock_skew_fake_arm_1`, `hmac_rejected_fake_arm_1`, `invalid_announce_fake_arm_1`, `duplicate_robot_id_fake_arm_2`, `mqtt_v5_required` (titles in `strings.json` under `issues`) |
| Node timings (fake executor) | task duration 2 s (`fake.task_duration_s`); `hang` timeout 10 s plus cancel grace 5 s; heartbeat every 2 s; keepalive 15 s; ack retained 120 s; result retained 300 s |
| HA timings | problem on after 3 missed heartbeats (6 s); entities unavailable after 30 s without a heartbeat; replay settle 3 s; session change grace 1 s; clock skew repair at 1000 ms; duplicate id repair closes after 600 quiet seconds |

## Where to look

| Place | How |
|---|---|
| Entity states | Settings, Devices and services, ROS 2 Bridge, the robot's device page; or Developer tools, States; or the MCP `ha_get_state` for one entity. The State sensor's attributes carry `task_id`, `task`, `paused`, `inhibited`, `ha_link`, `error_code`, `error_reason`, `session`, `instance`, and the latest feedback (`progress`, `stage`, `message`). |
| Bus events | Developer tools, Events, "Listen to events": enter `ros2_bridge_task_result` (and in a second tab `ros2_bridge_task_ack`), press Start listening before the action. |
| Actions | Developer tools, Actions, YAML mode, with "Response" visible for `wait: result` and `get_robots`. |
| Diagnostics | Settings, Devices and services, ROS 2 Bridge, three-dot menu, Download diagnostics. Keys named below are paths in that file, for example `hub.robots.fake_arm_1.counters.dropped_retained`. |
| Repairs | Settings, System, Repairs. |
| HA log | Settings, System, Logs, filter `ros2_bridge`; or the MCP `ha_get_logs`. |
| The bus itself | From `tools/`: `docker compose run --rm mqtt` subscribes to everything under the prefix with topic names; `docker compose run --rm mqtt 'mosquitto_sub $MQ -v -t "$TOPIC_PREFIX/fake_arm_1/#"'` for one robot; add `--retained-only` to see only the retained set, `-R` for live traffic only, `-W 3` to exit after three seconds. The quoting keeps `$MQ` for the container's shell. |
| Node logs | `docker compose logs -f arm1` (and `arm2`). |
| Node counters | The heartbeat on `ros2_bridge/fake_arm_1/heartbeat` every 2 s carries `counters` (`accepted`, `rejected`, `duplicates`, `dropped_retained`, `dropped_oversize`, `dropped_invalid_id`, `dropped_queue_full`, `publish_errors`, `takeovers`); the same numbers appear in the diagnostics under `hub.robots.fake_arm_1.counters`. |

Every `docker compose` command below is run from `tools/` on the development PC with `tools/.env` filled in (`tools/README.md`). "Send a command by hand" means the `mosquitto_pub` line in `tools/README.md` under that heading, which generates a fresh UUID4 `ID` and a current `NOW`.

## Steps

### Step 0: clocks and the broker

Action: `docker compose build` once, then `docker compose run --rm clock-check`. Before the integration is installed only the round trip can be measured; repeat after step 2 as `docker compose run --rm clock-check --require-heartbeat --threshold-s 1.0`.

Expect: the probe connects with the `.env` credentials, reports a round trip of a few milliseconds, and (after step 2) "this machine's clock is N ms ahead of/behind Home Assistant's" with N under 1000 and exit status 0. A failure here (exit 1) means NTP needs attention before anything else; the node refuses commands beyond 2000 ms of skew.

Where: the command's own output. `docker compose run --rm mqtt --retained-only -W 3`-style subscriptions in later steps use the same credentials, so a refused connection shows here first.

### Step 1: install the integration

Action: put `custom_components/ros2_bridge/` into the Home Assistant configuration folder so that `<config>/custom_components/ros2_bridge/manifest.json` exists. Route: HACS custom repository once the public repository exists (open question (b)); otherwise write each file of the folder with `ha_write_file` after `ha_get_skill_guide` (the `translations/en.json` file included). Restart Home Assistant (Settings, System, Restart, or the MCP `ha_restart`).

Expect: after the restart the log filtered on `ros2_bridge` shows no error and no traceback. Settings, Devices and services, Add integration lists "ROS 2 Bridge". Nothing else changes yet: no device, no entity, because no robot has announced.

Where: HA log; the Add integration dialog.

### Step 2: discovery and two idle robots

Action: `docker compose up -d arm1 arm2`, then `docker compose logs -f arm1` until it says `connected to the MQTT broker as 'ros2_ha_bridge_fake_arm_1'`. In Home Assistant, Settings, Devices and services: a Discovered card "Fake arm 1" (or "Fake arm 2") appears within a few seconds. Press Configure; the step "Robot found" reads "Found robot Fake arm 1 announcing under the topic prefix ros2_bridge. Create the ROS 2 Bridge entry?"; leave the shared secret empty and submit.

Expect: one entry "ROS 2 Bridge" with two devices, "Fake arm 1" and "Fake arm 2". Each device has the State sensor at Idle, Last result unknown, Problem off, the Task result event entity, seven task buttons (Reset to home, Hang, Raise error, Clear error, Stop, Pause, Resume) and Cancel task. `mosquitto_sub --retained-only` on `ros2_bridge/#` shows, per robot, `availability online`, the announce (ten tasks, a `session`, an `instance` of the form `host:pid`) and the `idle` state; then a heartbeat per robot every 2 s and Home Assistant's own `ros2_bridge/ha/heartbeat` every 2 s. Diagnostics: `hub.ready`, `hub.mqtt_connected` and `hub.reconciled` all true; `hub.subscriptions` lists the seven filters.

Where: Devices and services; the device pages; `docker compose run --rm mqtt`; diagnostics.

### Step 3: one button press, end to end

Action: start the event listener on `ros2_bridge_task_result`, then press `button.fake_arm_1_reset_to_home`.

Expect: the State sensor goes Busy with `task: reset` and a `task_id`, feedback attributes update (`stage: executing`, `progress` rising) and after 2 s it returns to Idle. Last result becomes Succeeded with `task: reset`, `code: ok`, a `duration_ms` near 2000, `synthetic: false`. Events: one `ros2_bridge_task_ack` with `accepted: true`, `code: accepted`, `source.kind: button`, then exactly one `ros2_bridge_task_result` with `status: succeeded`, `kind: motion`. On the bus: a retained ack on `ros2_bridge/fake_arm_1/task/<task_id>/ack`, a `busy` state, feedback at 2 Hz, a retained result on `.../result`, an `idle` state. Problem stays off.

Where: the device page; the events tab; `mosquitto_sub -v -t "ros2_bridge/fake_arm_1/#"`.

### Step 4: parameterised tasks and cancel through the actions

Action, in Developer tools, Actions (YAML mode, response shown):

1. `ros2_bridge.send_task` with `robot_id: fake_arm_1`, `task: move_to`, `params: {pose: home}`, `wait: result`.
2. The same call with `params: {}`.
3. `ros2_bridge.send_task` with `task: fail`, `params: {code: collision}`, `wait: result`.
4. `ros2_bridge.send_task` with `task: wait`, `params: {seconds: 20}`, `wait: none`; then within those 20 s `ros2_bridge.cancel_task` with `robot_id: fake_arm_1`, `wait: result`.

Expect:

1. The response arrives after about 2 s: `tasks[0].accepted: true`, `tasks[0].result.status: succeeded`, `result.code: ok`, `result.data.pose: home`. Last result Succeeded, `task: move_to`.
2. The call fails before anything is published with "The parameters for task move_to on robot fake_arm_1 are invalid: ..." naming the missing `pose`. No ack, no event, nothing on `cmd`.
3. `result.status: aborted`, `result.code: collision`, `result.reason` "Planned path collides with an object", `result.data.failure_class: collision`. Last result Aborted. Problem stays off: a failed task alone never puts the robot into error.
4. The State sensor shows Busy with `task: wait` and rising `progress`. The cancel's response carries `target_id` equal to the wait's `task_id` and `result.status: cancelled`, `result.code: cancelled`, reason "Cancelled by request". Events: the wait's id fires `ros2_bridge_task_result` with `status: cancelled`; the cancel's own id fires only `ros2_bridge_task_ack` (`type: cancel`, `accepted: true`) and never a `task_result`. The robot returns to Idle.

Where: the Actions response panel; the events tab; the State and Last result sensors.

### Step 5: pause longer than the margin, stop, and a busy rejection

Action:

1. `ros2_bridge.send_task` with `task: wait`, `params: {seconds: 30}`, `wait: none`.
2. Press `button.fake_arm_1_pause`. Wait 15 s (the Result timeout margin is 10 s, so this is margin plus 5 s).
3. Press `button.fake_arm_1_resume`, then `button.fake_arm_1_stop`.
4. Press `button.fake_arm_1_reset_to_home` twice within a second.

Expect:

1. Busy, `task: wait`.
2. The State sensor stays Busy with the attribute `paused: true`; feedback stops; during the 15 s no event of any kind fires for the wait (its result deadline is suspended while the state shows it paused). The pause's own result (`kind: control`, `status: succeeded`) fires `ros2_bridge_task_result` and triggers the Task result event entity but does not change Last result.
3. After Resume, `paused: false` and feedback continues. After Stop, the wait ends with `status: cancelled`, `code: preempted`, reason "Stopped by task 'stop'"; Last result becomes Cancelled (the wait is a motion task), the stop's own result (`control`, `succeeded`, `data.stopped` listing the wait's id) does not touch Last result. The robot is Idle.
4. The first press is accepted; the second shows an error on the button: "Robot fake_arm_1 refused task reset: Task 'reset' (id ...) is running". Events for the second: `ros2_bridge_task_ack` with `accepted: false`, `code: rejected_busy`, and one `ros2_bridge_task_result` with `status: rejected`, `code: rejected_busy`. The first press completes normally 2 s later.

Where: the State sensor's attributes (`paused`); the events tab; the button's error toast.

### Step 6: the authorisation path, by hand on the broker

Action, each with the "Send a command by hand" line from `tools/README.md` through `docker compose run --rm mqtt '...'`, watching `mosquitto_sub -v -t "ros2_bridge/fake_arm_1/#"` in a second terminal:

1. Stale: set `NOW=$(( ($(date +%s) - 60) * 1000 ))` in the command line, send.
2. Duplicate: send the same line again with the same `ID` (fresh `NOW` or not, it does not matter).
3. Retained plant, node connected: `mosquitto_pub $MQ -q 1 -r -t "$TOPIC_PREFIX/fake_arm_1/cmd" -m '{"schema":1,"id":"planted"}'`.
4. Retained plant, node stopped: `docker compose stop arm1`, publish the same retained message, `docker compose start arm1`.
5. Malicious id: send the normal command line with `ID='x/#'`.
6. Signature: set `ROS2_HA_BRIDGE_COMMAND_SECRET` to a test value in `tools/.env`, `docker compose up -d arm1` again (the restart picks it up), and set the same value in the Shared secret option. Press Reset to home: accepted. Then send a hand-built command carrying `"sig":"0000000000000000000000000000000000000000000000000000000000000000"` (64 hex zeros, which passes the schema and fails the check). Then clear the secret on both sides again (empty the option, empty the variable, `docker compose up -d arm1`).

Expect:

1. A retained ack with `code: rejected_stale`, `clock_skew_ms` near 60000 and a reason naming the window. Side effect in Home Assistant: the hub applies repairs to every ack it sees, so the repair "Clock skew with robot fake_arm_1" opens; it closes at the next ack with an acceptable skew (press any button).
2. The second delivery on the same ack topic is byte for byte the first one (same `ts`, same `received_at`); the node's `duplicates` counter goes up by one and `rejected` does not.
3. No ack. The node's log warns "dropped a retained message of N bytes on ... and cleared it"; `dropped_retained` goes up by one in the next heartbeat; `mosquitto_sub $MQ -v -t "$TOPIC_PREFIX/fake_arm_1/cmd" --retained-only -W 3` prints nothing, which is the proof that the topic was cleared. The node keeps heartbeating.
4. The same, at the node's first connect after `start`.
5. No ack, no topic built; `dropped_invalid_id` goes up by one; heartbeats continue (the node is alive). Nothing appears on any `task/x/#` topic because none exists.
6. The announce now says `capabilities.hmac_required: true`. The button press is accepted (Home Assistant signs). The hand-built command is answered `rejected_unauthorised` with the reason "sig does not match this topic and payload"; the repair "Robot fake_arm_1 refused the command signature" opens with "the robot refused the signature (secret mismatch)" and closes after the next accepted command (press a button). After clearing both secrets the announce says `hmac_required: false` and unsigned hand-built commands are accepted again.

Where: `mosquitto_sub` for the acks; the heartbeat counters or the diagnostics `hub.robots.fake_arm_1.counters`; Settings, System, Repairs; `docker compose logs arm1`.

### Step 7: the executor that never finishes

Action: press `button.fake_arm_1_hang`. Wait 15 s. Then press `button.fake_arm_1_stop`.

Expect: Busy with `task: hang` for 10 s; the node then asks the executor to cancel, which the fake ignores, and after the 5 s grace it forces the result: `ros2_bridge_task_result` with `status: aborted`, `code: timeout`; Last result Aborted; the State sensor shows Error with `error_code: timeout` and `error_reason`; Problem turns on with `reasons: [error]`. A Reset to home press now is refused with `rejected_error`. Stop clears it: the fake executor's `stop` calls `clear_error`, the State sensor returns to Idle, Problem turns off.

Where: the device page; the events tab; the State sensor's `error_code` attribute.

### Step 8: a silent node, a dead node, and Home Assistant restarting mid-task

Action and expectations, in order:

1. `docker compose pause arm1`. Within about 6 s (three missed heartbeats) Problem turns on with `reasons: [heartbeat_stale]` while the retained availability still says `online`. The broker drops the silent client after about 1.5 times the 15 s keepalive and publishes the Last Will `offline`, which makes every entity of fake_arm_1 unavailable; if the broker were slower the 30 s "Unavailable after missing heartbeats for" timer would do the same. Diagnostics: `hub.robots.fake_arm_1.heartbeat_stale: true` and, after 30 s, `heartbeat_unavailable: true`. `docker compose unpause arm1`: the node reconnects by itself, republishes `online`, announce and state with the same `session`, entities return, Problem turns off.
2. `docker compose kill arm1`. The Last Will arrives within a few seconds; entities unavailable within about 25 s at the latest. `docker compose up -d arm1`: the robot returns with a new `session` and `instance`; no second device and no duplicate entities appear (same unique ids); the log notes the session change and nothing is marked lost, because no command was outstanding.
3. `ros2_bridge.send_task` with `task: wait`, `params: {seconds: 60}`, `wait: none`; then restart Home Assistant (Settings, System, Restart) within the first 20 s. After Home Assistant returns: the State sensor shows Busy with the same `task_id` (re-adopted from the Store after the 3 s replay settle); the real result fires exactly one `ros2_bridge_task_result` with `status: succeeded` and never a `ros2_bridge_task_late_result`; Last result Succeeded. Diagnostics after the restart: `hub.reconciled: true` and the command listed in `hub.commands` with its state.
4. Restart Home Assistant again within five minutes of that result (the retained result is still on the broker). Nothing re-fires: the id is in `seen_results` and the diagnostics counter `hub.counters.replayed_results_ignored` goes up by one.
5. Send `wait {seconds: 60}` again and, while it runs, save the options page without changing anything (or change the Acknowledgement timeout by 0.1 s). The entry reloads; the command survives the reload and its result fires once as `ros2_bridge_task_result`.

Where: the device page; diagnostics; the events tab; `docker compose logs arm1`.

### Step 9: the heartbeat the other way, and the watchdog modes

The node's `watchdog.mode` is a parameter, not an environment variable, so this step mounts a parameters file as `tools/README.md` shows. Create `tools/e2e_inhibit.yaml`:

```yaml
/**:
  ros__parameters:
    watchdog:
      mode: inhibit_new
```

and `tools/e2e_hold.yaml` with `mode: stop_and_hold`. Each run below replaces the `arm1` service container, so stop it first; two nodes must never claim `fake_arm_1` at once.

Action and expectations:

1. Set the option Heartbeat interval to 0 and save. The node's state (attribute `ha_link`) shows `down` after the watchdog timeout (5 s floor, at least 2.5 announced intervals); the node logs the link change; nothing else happens: buttons still work, tasks still run (`watchdog.mode` is `none`). Set the option back to 2; `ha_link` returns to `up`.
2. `docker compose stop arm1`, then `docker compose run --rm -v "$PWD/e2e_inhibit.yaml:/params.yaml:ro" -e ROS2_HA_BRIDGE_PARAMS_FILE=/params.yaml arm1`. The announce says `watchdog_mode: inhibit_new`. With the Heartbeat interval at 0 again: the State sensor shows `inhibited: true`; Reset to home is refused with `rejected_inhibited` ("New motion is inhibited while the HA link is down"); Stop is accepted (control tasks pass in any state). Heartbeat interval back to 2: `inhibited: false`, Reset to home accepted.
3. Stop that container (Ctrl-C) and run it again with `e2e_hold.yaml`. Send `wait {seconds: 60}`; while it runs set the Heartbeat interval to 0. When the link drops, the wait ends with `status: cancelled`, `code: inhibited` (the executor's `stop_and_hold` was called once), the robot is Idle, not Error, and `inhibited: true` keeps new motion out until the heartbeat is back. Restore the option, stop the container, `docker compose up -d arm1` to return to the default mode.

Where: the State sensor's `ha_link` and `inhibited` attributes; the button error; the events tab; `docker compose logs`.

### Step 10: the two sensor paths and the latency probe

Mirror topics are a node parameter too. Create `tools/e2e_mirror.yaml`:

```yaml
/**:
  ros__parameters:
    mirror:
      topics:
        - zigbee2mqtt/<a real sensor friendly name, read only>
        - ros2_bridge/ha/e2e/motion
        - ros2_bridge/ha/sensors/#
```

Run `arm1` with it as in step 9 (stop the service container first). Watch the ROS topic inside that container: `docker exec -it <container name from docker ps> /ws/tools/entrypoint.sh ros2 topic echo /bridge_node/mqtt_in`.

Action and expectations:

1. Trigger the real Zigbee2MQTT sensor (walk past it). One envelope arrives on `/bridge_node/mqtt_in` with `topic: zigbee2mqtt/<name>`, the device's own payload unchanged as the `payload` string, `encoding: utf-8`, `retain: false`, `source_available: true` (Zigbee2MQTT's retained availability was seen at subscribe) and an `rx_ts_ms`. Home Assistant's own entity for that sensor changes at the same moment; neither waited for the other.
2. Scratch topic: `mosquitto_pub $MQ -t "$TOPIC_PREFIX/ha/e2e/motion/availability" -m online -r`, then `mosquitto_pub $MQ -t "$TOPIC_PREFIX/ha/e2e/motion" -m '{"occupancy":true}'`: an envelope with `source_available: true`. Then `mosquitto_pub $MQ -t "$TOPIC_PREFIX/ha/e2e/motion/availability" -m offline -r`: an envelope for the availability message itself with `source_available: false`, and the next data message carries `false` too. Clear both at the end: `mosquitto_pub $MQ -r -n -t "$TOPIC_PREFIX/ha/e2e/motion/availability"` (and the data topic if it was published retained).
3. Normalised path: add one entity (for example the same motion sensor's `binary_sensor`) to the option Mirrored entities and save. `mosquitto_sub --retained-only -t "ros2_bridge/ha/sensors/#" -W 3` shows one retained canonical message (`schema: 1`, `entity_id`, `state`, `available`, `device_class`, the allowlisted `attributes`, `last_changed`, `ts`); the same arrives as an envelope on `/bridge_node/mqtt_in` with `source_available: null`. Every state change republishes it. Diagnostics: `store.published_sensor_topics` lists the topic.
4. Remove the entity from the option and save: the retained topic is cleared (the retained-only subscription prints nothing) and `store.published_sensor_topics` is empty.
5. Add the entity back, then change the option Topic prefix to `ros2_bridge_e2e` and save: the old-prefix topic `ros2_bridge/ha/sensors/...` is cleared and the new one `ros2_bridge_e2e/ha/sensors/...` is published; the robots show unavailable while the prefix differs from the nodes'. Set the prefix back to `ros2_bridge`: the robots return within seconds, the `ros2_bridge_e2e/...` topic is cleared.
6. Latency probe, from the development PC with the broker password in the environment (`ROS2_HA_BRIDGE_MQTT_PASSWORD`): `.venv\Scripts\python.exe tools\latency_probe.py --host <broker> --username <user> --robot fake_arm_1` (host and user as in `tools/.env`, never committed). The default task name is unknown to the node, so the 20 commands are answered `rejected_unknown_task` without touching the executor. Record the JSON summary's `command_to_ack_ms`, `publish_to_puback_ms`, `period_ms`, `jitter_abs_ms` and `transport_jitter_abs_ms` with the date in the results table and in `docs/architecture.md` (Measured latency). Side effect: the integration counts the 20 acks as `unmatched_replies`. The probe is not in the Docker image; to run it from a container, mount it: `docker compose run --rm -v "$PWD/latency_probe.py:/ws/tools/latency_probe.py:ro" --entrypoint /ws/tools/entrypoint.sh arm1 python3 /ws/tools/latency_probe.py --robot fake_arm_1`.

Afterwards stop the one-off container and `docker compose up -d arm1`.

Where: `ros2 topic echo` inside the container; `mosquitto_sub`; the options page; diagnostics `store.published_sensor_topics`; the probe's output.

### Step 11: voice

Action: write `docs/examples/custom_sentences/en/ros2_bridge.yaml` to `<config>/custom_sentences/en/ros2_bridge.yaml` with `ha_write_file` (after `ha_get_skill_guide`), reload Conversation (Developer tools, YAML, Conversation). Put the "Fake arm 1" device in the area of the Voice PE (its device page, area) and leave "Fake arm 2" without an area. Then:

1. Developer tools, Assist: type "tell fake arm 1 to reset".
2. From the Voice PE in that area: say "reset the arm".
3. Say "reset the arm" again while the first reset is still running.
4. Save any option (reload the entry) and repeat 1.

Expect:

1. The reply "Starting Reset to home on Fake arm 1"; the robot goes Busy; the command's `ros2_bridge_task_ack` carries `source.kind: intent`. The spoken name is matched against the announced name "Fake arm 1" and the id spoken as words; "fake arm one" would not match, which is a known limit of the matching, not a failure.
2. The same reply from the satellite; the area of the device that heard the command picked fake_arm_1 because it is the only robot in that area. With two robots and no area, the reply would be "Which arm do you mean?".
3. "Fake arm 1 refused: Task 'reset' (id ...) is running".
4. Still handled, with the same reply; the intents are registered once and survive an options reload.

If the speak-back automation from `docs/examples/automations.yaml` (automation 2) is installed, the outcome is announced on the satellite about 2 s after each accepted reset.

Where: the Assist dialog; the satellite; the events tab (`source.kind`).

### Step 12: two nodes, one robot id

Action: with `arm2` running, start a second node claiming the same id: `docker compose run --rm -e ROS2_HA_BRIDGE_ROBOT_ID=fake_arm_2 arm1`. Watch both logs.

Expect: both connect with client id `ros2_ha_bridge_fake_arm_2`; the broker disconnects one with reason code 142 (`Session taken over`); each node logs an ERROR naming the other `instance`, backs off and reconnects, and the one that reaches three takeovers within ten minutes exits non-zero (no restart policy, so it stays down). In Home Assistant the repair "Two robot nodes may be using the id fake_arm_2" opens, from two different `instance` values within one heartbeat interval or from more than three session changes in five minutes; Problem on "Fake arm 2" turns on with `reasons: [duplicate_robot_id]`. After the survivor is alone for ten quiet minutes the repair closes by itself and Problem turns off. If the `arm2` service container was the one that exited, `docker compose up -d arm2` after stopping the extra container.

Where: `docker compose logs arm2` and the one-off container's output; Repairs; the Problem sensor's `reasons`; diagnostics `hub.robots.fake_arm_2.session_change_count` and `counters.takeovers`.

### Step 13: deleting an offline robot

Action: `docker compose stop arm2` (a clean shutdown publishes `offline`). On the device page of "Fake arm 2", three-dot menu, Delete. Then reload the entry (Settings, Devices and services, ROS 2 Bridge, three-dot menu, Reload).

Expect: deletion is offered because the robot is offline; the device and its entities disappear; `mosquitto_sub --retained-only -t "ros2_bridge/fake_arm_2/#" -W 3` prints nothing (announce, state and availability cleared with empty retained publishes); after the reload the device does not come back. Starting `arm2` again later re-announces and recreates the device, which is the expected behaviour, not a failure.

Where: the device page; `mosquitto_sub`; Devices and services after the reload.

## Clean-up

`docker compose down` from `tools/`; `mosquitto_pub $MQ -r -n` on the two scratch topics of step 10 if they were left; options back to their defaults (Heartbeat interval 2, Mirrored entities empty, Topic prefix `ros2_bridge`, Shared secret empty); `ROS2_HA_BRIDGE_COMMAND_SECRET` emptied in `tools/.env`; the sentences file may stay. Retained acks and results expire by themselves.

## Results

Outcome is one of pass, fail, skipped. A fail or a skip gets a note and, where it is a defect, an issue reference.

| Step | What | Date | Outcome | Notes |
|---|---|---|---|---|
| 0 | Clock check, round trip and offset | | | |
| 1 | Install and restart, no errors | 2026-10-14 | pass | Copied over the Samba share, HA restarted. First run logged two findings (intent platform hook name, import inside the event loop), both fixed, redeployed and HA restarted again: no ros2_bridge log entries afterwards. |
| 2 | Discovery, two idle robots, buttons present | 2026-10-14 | pass | Two fake arms from tools/compose.yaml against the live Mosquitto: 24 entities created (state idle, problem off, 8 task buttons each, cancel, last_result, task_result event). HA heartbeat seen by the nodes within 2 s (ha_link up). |
| 3 | Button press: busy, succeeded, events, idle | 2026-10-14 | pass | button.fake_arm_1_reset_to_home: retained ack (accepted, clock_skew_ms -217), retained result succeeded/ok after 2002 ms, event entity fired once, last_result succeeded, state back to idle. |
| 4 | move_to, invalid params, fail collision, wait then cancel | 2026-10-14 | pass | send_task move_to without pose: ServiceValidationError "'pose' is required". fail {code: collision} with wait result: aborted/collision with data {simulated, failure_class}. wait {seconds: 60} then cancel_task with wait result: the cancel command itself ended done (ack-terminal) and the target's result was cancelled (goal_status 5, reason "Cancelled by request"). |
| 5 | Pause beyond the margin, resume, stop, rejected_busy | 2026-10-14 | pass | reset while wait was running: rejected_busy naming the running task, surfaced in the service response and in last_result. Pause button: state busy with paused true; Resume button cleared it; the task was then cancelled (step 4). The pause-beyond-the-margin timing case was not held long enough to exercise the deadline suspension and is covered by tests/test_commands.py. |
| 6 | Stale, duplicate, retained plants, malicious id, signature | | | |
| 7 | hang: forced aborted/timeout, error, stop clears | | | |
| 8 | pause, kill, HA restart mid-task, second restart, options save mid-task | | | |
| 9 | Heartbeat 0, inhibit_new, stop_and_hold | | | |
| 10 | Mirror topics, mirror_entities, prefix change, latency probe | | | |
| 11 | Sentences, voice replies, after an options reload | | | |
| 12 | Duplicate robot id, repair opens and closes | | | |
| 13 | Delete an offline robot, topics cleared, does not return | | | |

Latency probe numbers (step 10, item 6), copied from the JSON summary:

| Date | Broker and network | command_to_ack_ms (min, median, p95, max) | publish_to_puback_ms (median) | heartbeat period_ms (median) | jitter_abs_ms (p95) | transport_jitter_abs_ms (p95) |
|---|---|---|---|---|---|---|
| | | | | | | |
