# Safety

This page says, in plain words, what ha-ros2-bridge does and does not do for the safety of a robotic arm that Home Assistant can start moving. It is written for the person who decides whether to install it at a site. The security side (who may send commands) is in [security.md](security.md); the failure-by-failure detail is in [failure-modes.md](failure-modes.md).

## The one-sentence version

Home Assistant is advisory. ROS 2 owns safety. The bridge carries requests and reports outcomes; it is not a safety function, and losing Home Assistant does nothing to the arms unless a site deliberately asks for that.

## Where safety sits

The interview fixed this on 2026-10-13 (`docs/decisions.md`, rows 3 and 16):

- **Q3** asked whether Home Assistant is in the safety path. The answer was "whichever route is fastest and safest", and the design's consequence is that raw sensor events reach ROS 2 and Home Assistant in parallel over the broker (the fast sensor path) rather than serially through Home Assistant automations, while the hardware e-stop remains authoritative and outside this design.
- **Q16** asked what should happen when Home Assistant's heartbeat stops. The answer was "nothing, HA going away must not affect the arms". So the heartbeat from Home Assistant to the robot is informational by default, and the robot never depends on Home Assistant being alive.

Everything that protects a person near the arm lives in the robot: its safety-rated controller, its protective stop, its collision handling, its speed and force limits, and the hardware emergency stop. The bridge neither replaces nor extends any of them. Nothing on MQTT and nothing in Home Assistant is labelled, or may be labelled, an emergency stop (`CONTRIBUTING.md` makes this a review rule). The `stop` task that the shipped fake executor offers, and any `stop` a real executor offers, is a software request that travels over a broker and a Wi-Fi or Ethernet network with no guaranteed delivery time; it is a convenience, not a safety stop.

## Out of scope

Recorded as an assumption on 2026-10-13: hardware e-stop, protective stop and collision handling are the arm controller's responsibility and out of scope for this project. If any of those misbehave, the report goes to the arm vendor, not to this repository.

MQTT is not a real-time channel either. The design's provisional latency targets (under 50 ms broker to node on the fast sensor path, under 1 s for a Home Assistant automation) are targets to be measured in phase 5, not guarantees (`docs/architecture.md`, "Measured latency"). All motion control loops stay inside ROS 2; Home Assistant only issues goals.

## What the bridge promises

These are properties of the code as it stands, each with its tests listed in [failure-modes.md](failure-modes.md).

- **One named outcome per command id.** A command is acknowledged `accepted` or rejected with a reason, and an accepted task ends in exactly one result: `succeeded`, `cancelled` or `aborted` with a structured code. If the robot never answers, Home Assistant synthesises `timeout` or `lost` from its own deadlines and only then. `timeout` with code `no_ack` means the outcome is unknown, not that the task failed.
- **No replay.** Every command carries a UUID id. A redelivered or replayed id gets the stored acknowledgement back, byte for byte, from a store that survives a node restart for as long as the command could still be fresh; the task is not started twice, with one recorded exception: a node that crashes within a second of accepting a command may lose that id before the dedupe file is written, and a resend inside the freshness window (`command.max_age_cap_ms` plus skew) would then run it again; see [failure-modes.md](failure-modes.md) and [security.md](security.md). The lookup runs before the signature and freshness checks, so a deliberately broken replay cannot change a stored `accepted` into a rejection.
- **Stale rejection.** A command carries the time it was issued and a maximum age (Home Assistant's default `command_max_age_ms` is 5000). The node accepts it only inside that window, capped at `command.max_age_cap_ms` (30000), plus `command.clock_skew_tolerance_ms` (2000) either side. A command that sat in a broker queue, or in the node's own inbound queue while the node was stalled, is refused with `rejected_stale`. This needs both machines on NTP; the ack carries the measured skew and Home Assistant raises the `clock_skew` repair when it drifts.
- **No retained commands.** Commands are published without the retain flag and with an MQTT 5 message expiry. The node subscribes with Retain As Published, so a retained command, whether planted while the node is connected or left on the broker before it connects, is seen with its flag, dropped unparsed, counted and cleared. A node that restarts connects with `clean_start`, so a broker-queued backlog of commands is never drained either.
- **One motion at a time, and only announced tasks.** A motion task is refused while the robot is busy, in error or inhibited. Only tasks the robot announced, narrowed by the operator's `tasks.allow`, can run, with parameters validated against the announced schema on both sides.
- **Buttons, not selects.** Named motions appear in Home Assistant as buttons, because a button press is an explicit act; selecting an item in a select entity must never start an arm moving (design decision, 2026-10-13). Parameterised tasks go through the `send_task` action.
- **Commands enter only through the integration.** The `send_task` and `cancel_task` actions, the task and cancel buttons and the two Assist intents are the only ways to raise a command, each publishing with the calling user's id and context in `source`; the integration exposes no raw publish, and `mqtt.publish` is administrator-only in Home Assistant since 2026.10.
- **A hung executor cannot hang the fleet.** One process per robot. A task that runs past its timeout is asked to cancel and, after `executor.cancel_grace_s` (5 s), forced to `aborted`, and the robot stays in `error` until the executor clears it. A node whose executor blocks for ever stops heartbeating; Home Assistant shows `problem` within about 6 s and marks the robot unavailable after `heartbeat_unavailable_after_s` (30 s). Other robots are untouched.

## What the bridge does not promise

- It does not stop an arm. It can ask, through `stop` or a cancel, and in `stop_and_hold` mode it can ask the executor to hold; whether and how fast the arm stops is the executor's and the controller's business.
- It does not guarantee delivery or latency. It guarantees that every command ends in a named outcome, including "unknown".
- It does not know who is standing near the arm. An automation that fires `reset` when the kitchen has been empty for five minutes is only as safe as the sensors and the rule behind it. See the checklist below.
- It does not authenticate replies. Home Assistant trusts acknowledgements and results by command id; a hostile broker could lie about an outcome, which can mislead an automation but cannot move the arm (see [security.md](security.md)).
- It does not prevent two Home Assistant instances from commanding the same robot.

## The watchdog, and what losing Home Assistant does

Home Assistant publishes `{prefix}/ha/heartbeat` every `heartbeat_interval_s` seconds (integration option, default 2; 0 disables it) while its MQTT client is connected. The node tracks it, exposes the result as `ha_link` (`up` or `down`) in its retained state and on the latched ROS topic `~/ha_link`, and applies `watchdog.mode`. The node evaluates the link every 0.5 s and counts it down after `watchdog.timeout_s` (default 5.0) without a heartbeat, where the timeout is clamped to at least 2.5 times the interval Home Assistant announces, so a slow heartbeat never trips the watchdog on its own.

| `watchdog.mode` | What a lost heartbeat does | Who should choose it |
|---|---|---|
| `none` (default) | Nothing but a record: `ha_link` goes `down`, the change is logged and published, no task is touched and no command is refused. This is Q16. | Every site where the robot's own safety functions are trusted and Home Assistant is a convenience. |
| `inhibit_new` | As `none`, and `inhibited` is set while the link is down, so new motion tasks are refused with `rejected_inhibited`; running tasks continue and control tasks (stop, pause, resume) are still accepted. The link counts as down from start-up until the first heartbeat, so a node that boots while Home Assistant is away refuses motion until it appears. | A site that wants Home Assistant's presence as a precondition for starting motion, for example because the occupancy sensors that gate motion live in Home Assistant. |
| `stop_and_hold` | As `inhibit_new`, and additionally, once per link loss, the executor's `stop_and_hold` is called and the active motion task ends with `cancelled/inhibited`. It fires only on a transition from up to down, never at start-up, and the robot does not enter `error`; `inhibited` is what keeps new motion out until the heartbeat returns. | A site whose own safety case puts Home Assistant in the loop and has an executor whose `stop_and_hold` does something meaningful. The shipped fake executor's version ends its running task; a real executor must implement its own hold. |

Three cautions:

- The active modes do not make the system safer by themselves. They make the arm stop or refuse work when a broker, a Wi-Fi link or a Home Assistant update hiccups, which is a different trade-off (availability against the risk of motion without supervision). Choose them for a reason you can write down.
- Whether the node should call the executor's `stop_and_hold` once at start-up (`hold_on_startup`) is open question (c) in `docs/decisions.md`; it does not exist yet and the proposed default is off.
- The robot-to-Home Assistant direction (availability through the Last Will, retained state, heartbeat staleness) is always on, whatever the mode, so Home Assistant can always tell a dead node from a quiet one.

## What happens on each failure

The short version; every row of [failure-modes.md](failure-modes.md) names the mechanism and the tests.

| If this happens | The arm | Home Assistant |
|---|---|---|
| Home Assistant restarts, or its MQTT client drops | Nothing (mode `none`); a running task carries on and finishes | Re-adopts the running task after the retained replay settles and reports its real result once |
| The broker restarts | Nothing; the node reconnects, republishes its retained state and sends queued replies | Robots unavailable while its own client is down, then back |
| The node dies | Depends on the executor and controller, not on the bridge; the Last Will marks the robot `offline` | Entities unavailable at once; outstanding commands `lost` or `timeout` |
| The node hangs with the connection up | A running task is stuck in the executor; no new command is processed | `problem` within about 6 s, unavailable after 30 s |
| The executor never finishes a task | Asked to cancel at the timeout, forced to `aborted` after the grace, robot in `error` until cleared | Last result `aborted`, `problem` on, motion refused with `rejected_error` |
| Someone replays or plants a command | Nothing new: the stored ack is republished, or the retained message is dropped and cleared | Counters in the diagnostics |
| The clocks drift apart | Commands refused as stale or future | `clock_skew` repair |
| Two nodes claim one robot id | They take each other's broker session over in turn; after three takeovers in ten minutes one exits | `duplicate_robot_id` repair, `problem` on |

## The assistive-care context and the standards

The first scenario is a kitchen with lights, sensors and two arms used for assistive care (interview Q2). That raises the bar in two ways: the person the arms serve may not be able to move out of the way or reach an e-stop quickly, and other people (carers, family) will start tasks from dashboards and by voice without knowing how the system works.

The landscape research (`docs/research/2026-10-13-landscape-research.json`, the "safety and reliability patterns" angle) summarised the standards posture for a home like this: ISO 10218-1 and ISO 10218-2:2025 (which absorbed ISO/TS 15066, the collaborative-robot technical specification) and ISO 13482 (personal care robots) do not apply as obligations to a domestic arm, but their principles transfer. The research named three, and this project adopts them as its posture, not as a claim of compliance:

1. **Safety functions belong to the robot's safety-rated hardware.** Not to a broker, not to Home Assistant, not to this bridge. This is why Q16's default is `none` and why nothing here is called an e-stop.
2. **Collaboration is an application property.** Whether an arm may move depends on who can be within its reach at that moment, and an automation that starts motion is part of the application. So assess who can be in reach when Home Assistant fires a trigger, and design the trigger (and the task the executor runs) for that, not for the empty-room case alone.
3. **Cybersecurity now counts as safety.** A command path that anyone on the network can inject into is a safety problem. Use authenticated MQTT, use TLS where the network is shared Wi-Fi, and switch the shared-secret signing on where the broker is shared (see [security.md](security.md)).

The same research notes which parts it could not verify: the exact wording of the ISO 10218:2025 collaborative classes and performance-level requirements, and the revision status of ISO 13482, because the publishers' pages were not readable at the time. Nothing in this project depends on that wording. The research also lists the stop hooks that exist in a typical ROS 2 stack (MoveIt's trajectory execution stop event, MoveIt Servo's command timeout, the joint trajectory controller's command timeout and decelerate-on-cancel, and an action server that sends a hold-position trajectory on cancel); those are an executor author's concern; they are listed in `docs/writing-an-executor.md` ("Cancel, stop and hold") and remain unverified by this project, whose MoveIt executor uses only `cancel_goal` on the MoveGroup action.

## Before letting Home Assistant start motion: a site checklist

Work through this with the fake executor first (`tools/compose.yaml` runs two fake arms against your broker), then again with the real executor in simulation, then on the arm.

1. **The arm's own safety is complete without the bridge.** E-stop, protective stop, speed and force limits and collision handling are configured and tested on the controller, and they work with the bridge node stopped.
2. **The executor behaves under the bridge's rules.** `start` returns promptly; `cancel` leads to `finish(cancelled)` within `executor.cancel_grace_s`; `stop_and_hold` does something you have watched; a forced `aborted` and the `error` state that follows are understood by whoever will clear them. Run the fake executor's `hang` task once (it never finishes, so the node forces `aborted/timeout` and the robot goes to `error` until `clear_error`) so you have seen a forced finish before a real executor produces one.
3. **Both machines keep time.** NTP on the Home Assistant host and the ROS 2 host, offsets under a second (`tools/clock_check.py`), and the `clock_skew` repair stays closed through a day of use.
4. **Every robot has its own `robot.id`**, the derived MQTT client id is left in place, and the `duplicate_robot_id` repair has never opened.
5. **The catalogue is narrowed.** `tasks.allow` on the node lists only the tasks this site wants Home Assistant to be able to start, and every parameterised task has a schema that refuses out-of-range values on the node (`minimum`, `maximum`, `enum`).
6. **Triggers are designed for people in reach.** For each automation that starts motion, write down who could be near the arm when it fires, what sensor evidence the rule relies on, and what happens if that sensor is offline (the canonical sensor payload carries `available: false`; the fast-path envelope carries `source_available`). Prefer a trigger that requires positive evidence of an empty workspace over one that infers it from silence.
7. **The watchdog mode is a decision, not a default you did not read.** `none` unless the site's safety case says otherwise; if `inhibit_new` or `stop_and_hold`, the `heartbeat_interval_s` option is not 0 and the behaviour on a Home Assistant restart has been watched.
8. **Only the integration's entry points can command the arm.** No `mqtt.publish` scripts, no dashboard button that bypasses `send_task`; non-administrator users act through buttons, actions and voice, which carry their user id in every command.
9. **The security posture matches the broker.** Read [security.md](security.md): dedicated MQTT user where the broker enforces ACLs, TLS on Wi-Fi, the shared secret on a shared broker, and nothing retained on a `cmd` topic, ever.
10. **The outcome path is wired.** Something (a notification, speech, a light) reacts to `ros2_bridge_task_result` with `status` other than `succeeded`, and to the `problem` sensor, so a forced `aborted` or a stale heartbeat is noticed by a person. A rule that branches on `code` should read the robot state too: `timeout` with the robot `idle` is the executor's own timeout (MoveIt's `TIMED_OUT`, for example), while `timeout` with the robot `error` is the bridge's forced finish.
11. **The people who will use it know the limits.** Carers and household members know that the buttons and voice are requests, that "the arm did not answer" is a real outcome, and where the physical e-stop is.
