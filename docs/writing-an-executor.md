# Writing an executor

An executor is the one piece of robot-specific code in ha-ros2-bridge. The bridge node (`ros2/ros2_ha_bridge`) owns MQTT, authorisation, the state machine, timeouts and the wire format; the executor owns how a task becomes motion. This page is the contract between the two, read from the code: `ros2_ha_bridge/executors/base.py` (the interface), `task_manager.py` (what the bridge does around every call), `bridge_node.py` and `ros_adapters.py` (threads and timers), `executors/fake.py` (the shipped template) and `executors/moveit.py` (the reference MoveIt 2 executor). Where this page and the code differ, the code and its tests win; please raise an issue.

Nothing on this page is a safety mechanism. Hardware e-stop, protective stop and collision handling belong to the arm controller (`docs/safety.md`, "Where safety sits", and `SECURITY.md`, "Scope"); the bridge's `stop`, cancel and `stop_and_hold` are convenience and supervision.

## The interface in one table

Everything an executor touches lives in `ros2_ha_bridge/executors/base.py`.

| Type | What it is |
|---|---|
| `TaskSpec(name, label, kind='motion', timeout_s=60.0, params=None, description='')` | One catalogue entry. `name` matches `^[a-z][a-z0-9_]{0,31}$`; `kind` is `motion`, `control` or `query`; `params` is `None` or a params-subset JSON Schema (`type: object`, properties typed string, number, integer or boolean with optional `enum`, `minimum`, `maximum`, `description`, plus `required`). Checked with `protocol.validate_task_spec` at start-up. |
| `TaskRequest(id, task, params, issued_at, timeout_s, source)` | What `start` receives. `timeout_s` is final (the command's value or the announced default, clamped to `command.task_timeout_cap_s`); `source` is Home Assistant's record of who raised the command and is for logging only. |
| `TaskResult(status, code, reason='', data={})` | What `finish` takes. `status` is `succeeded`, `cancelled` or `aborted`; `goal_status` derives 4, 5 or 6. |
| `TaskRejected(code='rejected_executor', reason='')` | Raise from `start` to refuse a task. The ack is `rejected_executor` with your reason; `code` is logged, not sent. |
| `TaskHandle` | One per task: `task_id`, `feedback(progress=None, stage='', message='')`, `finish(result)`. Thread-safe; calls after the terminal `finish` are logged and ignored. |
| `BridgeContext` | Given to `setup`: `robot_id`, `logger`, `node` (the rclpy node, `None` in unit tests), `clock`, `scheduler`, `parameters`; thread-safe methods `post(fn)`, `set_error(code, reason)`, `clear_error()`, `set_paused(paused)`, `ha_link_ok()` and, since 0.2.0, `report_joints(positions, velocities=None)`. |
| `JointInfo(names, unit='deg', limits=None)` | What an executor announces about its joints (0.2.0, optional): 1 to 32 names matching `^[a-z][a-z0-9_]{0,31}$` in report order, `unit` `deg` or `rad`, `limits` `None` or one `(minimum, maximum)` pair per name. Checked with `protocol.validate_joint_info` at start-up. |
| `TaskExecutor` | The abstract base: class attributes `name` and `version` (announced to HA); abstract `setup`, `tasks`, `start`, `cancel`; defaults for `joints` (`None`), `stop_and_hold` (nothing), `health` (`('ok', '')`) and `shutdown` (nothing). |
| `load_executor(spec)` | Resolves the node's `executor` parameter: `package.module:Attribute`, or an entry-point name in the group `ros2_ha_bridge.executors`. A class is instantiated with no arguments. Any failure is `ExecutorLoadError`, fatal for the node. |

## Every method: when it is called and on which thread

"Main thread" below is the thread that constructs the node and then spins its `SingleThreadedExecutor` (`bridge_node.main`). Every executor method is called there. The only other threads in the process are paho's network thread, which never calls into the executor, and whatever threads the executor starts itself.

| Method | Called | Thread | What the bridge does around it |
|---|---|---|---|
| `__init__()` | Once, by `load_executor`, during node construction. | Main | Must take no arguments. Do nothing that can fail here; wait for `setup`. |
| `setup(ctx)` | Once, during node construction (`BridgeNode._build`), before the MQTT connection exists. | Main | Blocking waits (a driver, a server) are allowed here and nowhere else. Any exception becomes a `ConfigError`: the node logs it and exits with status 2 before announcing anything. |
| `tasks()` | Once, right after `setup`. | Main | Every spec is checked with `validate_task_spec`; duplicate names are an error; the operator's `tasks.allow` is intersected; the result is announced on every connect and HA birth. It is not read again (`docs/decisions.md`, open question (g)), so a catalogue that changes at run time is not re-announced. |
| `joints()` | Once, right after `tasks`. | Main | Optional (0.2.0). Return a `JointInfo` to have the node announce the joints and publish what `ctx.report_joints` delivers, or leave the default `None`. The object is checked with `validate_joint_info` (a bad one is a `ConfigError`), and with `telemetry.joints_rate_hz` set to 0 it is accepted but neither announced nor published. Like `tasks`, it is not read again. |
| `start(req, handle)` | For every task command that passed contract steps 0 to 9 (`TaskManager.start`). | Main | Timed: longer than `executor.call_warn_s` (0.1 s) logs a warning and marks the executor `degraded`. Must return promptly. `TaskRejected` gives the ack `rejected_executor` with your reason; any other exception gives `rejected_executor` with `executor.start raised X: message` and one traceback in the log. Either way the robot stays idle. You may call `handle.finish` before returning; then no deadline is armed. Otherwise the deadline timer is armed for `req.timeout_s` as `start` returns, and for a `motion` task the retained `busy` state is published. |
| `cancel(task_id)` | From a cancel command (`TaskManager.cancel`), from the task deadline (`_on_deadline`), and from `stop_and_hold` when your own `stop_and_hold` did not finish the task. | Main | Timed. Must lead to `handle.finish(...)` within `executor.cancel_grace_s` (5 s). Otherwise the bridge forces the result: `aborted/timeout` when the cancel came from the deadline and `cancel` returned normally, `aborted/executor_unresponsive` when `cancel` raised or when a cancel not caused by the deadline produced nothing. Every forced finish puts the robot into `error` until `ctx.clear_error()`. A cancel for an id you do not know should log and return; the bridge handles the rest. |
| `stop_and_hold(reason)` | From the watchdog tick, once per loss of the HA heartbeat, only in `watchdog.mode: stop_and_hold`. | Main | Timed. Halt motion and hold position. The active motion task ends `cancelled/inhibited` whatever you report: if you finished it through its handle, your result is overridden; if not, the bridge calls `cancel` and completes the task itself. The robot does not enter `error`; `inhibited` keeps new motion out until the heartbeat returns. |
| `health()` | Every `executor.health_poll_s` (1 s) from a node timer (`TaskManager.poll_health`). | Main | Return `('ok' or 'degraded' or 'error', message)`. A raise or an unknown level is recorded as `error` with the exception text. A change is published in the retained state as `executor_health`, and an `error` level turns Home Assistant's problem sensor on (`docs/troubleshooting.md`). Keep it cheap and non-blocking. |
| `shutdown()` | Once, from `BridgeNode.shutdown`, after the timers, the watchdog and the MQTT link (availability `offline`, disconnect) have been stopped; also while abandoning a node whose construction failed after the executor was loaded. | Main | Release drivers and threads. Nothing you finish here is published any more. Exceptions are logged and ignored. |

The bridge never calls two executor methods at once: everything is serialised on the main thread. That is also why a method that blocks stalls this robot only, and why there is deliberately no thread pool around plugin calls (`docs/decisions.md`, "No thread-pool guard"): the heartbeat stops, HA shows `problem` within about 6 s, and other robots are other processes.

## The lifecycle, in order

1. **Construction and `setup`.** The node declares its parameters, loads the executor, builds the task manager and gives you a `BridgeContext` wired to it. Read your parameters, open drivers, wait for servers if you must, raise for a misconfiguration. Nothing has been announced yet, so a raise here costs nothing.
2. **`tasks`.** Return a tuple of `TaskSpec`. A task with no required parameters gets a button in Home Assistant; every task is callable through the `send_task` service. Use `motion` for anything that moves (one at a time, refused while busy, in error or inhibited), `control` for stop, pause, resume and their like (accepted in any state, never touching the motion slot), `query` for reads (accepted in any state).
3. **`joints`, if you have them.** Return a `JointInfo` and then call `ctx.report_joints(positions)` whenever the driver gives you a new reading (see "Joint telemetry" below). Nothing else changes: the node throttles, announces and publishes.
4. **`start` returns promptly.** Kick the work off (an action goal, a thread, a timer through `ctx.scheduler`) and return. The ack `accepted` is published after `start` returns, and the deadline runs from there.
5. **Feedback.** Call `handle.feedback(progress, stage, message)` as often as you like from any thread; the bridge throttles to `reply.feedback_max_rate_hz` (2 Hz) and publishes it at QoS 0 with a 10 s expiry. `progress` is `0.0` to `1.0` or `None`.
6. **`finish` exactly once.** `handle.finish(TaskResult(...))` from any thread. The result is published retained (300 s expiry) and a motion task releases the slot (`idle` is published). A second `finish`, or feedback after it, is logged and dropped, so your cancel path may race your completion path without guards.
7. **Cancel within `cancel_grace_s`.** When `cancel(task_id)` arrives, stop the work and `finish(cancelled)` within 5 s. If the motion had already completed, finishing `succeeded` is right: the bridge reports whatever you report, as long as it arrives in time.
8. **`stop_and_hold`.** Only with the watchdog in `stop_and_hold` mode. Halt and hold; the bridge takes care of the result.
9. **`health`.** Report `error` when you cannot do your job (a server gone), `degraded` when you can but badly, `ok` otherwise.
10. **`shutdown`.** Stop what is running and release everything.

## Threading rules and `BridgeContext.post`

- Executor methods run on the main thread. Robot state (`busy`, `error`, the active task) lives there and nowhere else.
- `TaskHandle.feedback`, `TaskHandle.finish`, `ctx.set_error`, `ctx.clear_error`, `ctx.set_paused`, `ctx.report_joints` and `ctx.ha_link_ok` may be called from any thread: each hands a closure to `ctx.post`, which the node implements as "append to a deque and trigger an rclpy `GuardCondition`" (`ros_adapters.MainThreadPost`). The closure runs on the main thread in posting order, and a closure that raises is logged, never propagated. In unit tests `post` is usually `lambda fn: fn()`, which applies the call at once.
- Use `ctx.post(fn)` yourself for anything of your own that must touch main-thread state from a worker thread or a callback. rclpy callbacks (futures, subscriptions, action feedback) already run on the executor thread; posting from them costs one extra hop and keeps your code correct whichever thread an adapter calls from, which is what the MoveIt executor does.
- Timers: `ctx.scheduler.call_later(delay_s, fn)` returns a handle with `cancel()` and `remaining_s()`; `fn` runs once on the main thread. In the node this is `RclpyScheduler`, a periodic rclpy timer that cancels itself on its first line (rclpy Jazzy has no one-shot timer). `ctx.clock.monotonic()` and every timer run on one `STEADY_TIME` clock, so an NTP step cannot stretch a deadline, and `use_sim_time` does not slow the bridge's clocks either: a paused simulator does not pause a task's timeout.
- `ctx.node` is the rclpy `Node` in the real bridge and `None` in unit tests. Create your publishers, clients and action clients from it in `setup`, and keep every rclpy import behind a function or a constructor so your module imports (and your tests run) on a machine without ROS.
- `ctx.parameters` is a read-only mapping of the node's parameters by dotted name, minus the two secrets. The node's parameter table (`parameters.py`) is closed: it declares `fake.*` for the shipped fake executor and nothing for anyone else, so an executor that needs its own parameters must declare them on `ctx.node` itself (the MoveIt executor shows how) or read a file of its own.

## Pause

Pause is a convention between the executor and the bridge, not something the bridge can do to an arm:

- `ctx.set_paused(True)` with a motion task running keeps the robot `busy`, publishes `paused: true`, cancels the task's deadline timer and remembers the seconds that were left. Home Assistant suspends its own result deadline while the retained state shows the task paused. With no motion task running the call is ignored with a warning.
- `ctx.set_paused(False)` re-arms exactly the remaining seconds.
- `paused` clears by itself when the motion task finishes.
- Freezing your own timers and the physical motion is your job. The fake executor implements `pause` and `resume` as control tasks that stop and restart its completion and feedback timers around `ctx.set_paused`; a real executor would hold the controller. If your arm cannot pause, do not offer the tasks.

## Joint telemetry (0.2.0, optional)

An executor that knows where its joints are can let Home Assistant show them: one sensor per joint, a gauge per joint on a dashboard, a history graph of a motion. Nothing here is a safety signal; a position on the `joints` topic is a display value with a 5 s expiry (`docs/mqtt-contract.md`, "joints").

- **Announce.** Override `joints()` to return `JointInfo(names, unit, limits)`: the joint names in the order you will report them (1 to 32, each matching `^[a-z][a-z0-9_]{0,31}$`), the unit of every position (`deg` or `rad`) and either `None` or one `(minimum, maximum)` pair per name in that unit (Home Assistant uses the pairs for gauge ranges). The node validates it at start-up (`protocol.validate_joint_info`) and refuses to start on a bad one, then announces it as the `joints` block together with `telemetry.joints_rate_hz`. With that parameter at 0 the block is omitted and nothing is published, whatever you report.
- **Report.** Call `ctx.report_joints(positions, velocities=None)` from any thread, as often as the driver gives you a reading: one number per announced name, in the announced order and unit, and optionally one velocity per position in the unit per second. The values are copied and the call is posted to the main thread, exactly like `handle.feedback`. A report of the wrong length, or with a value that is not a finite number, is dropped with one warning; a report before any sink exists (a unit test without `report_joints`, or an executor that announced no joints) is dropped silently.
- **The node does the rest** (`ros2_ha_bridge/joints_publisher.py`): at most `telemetry.joints_rate_hz` publications per second (default 2), only when some position moved by more than `telemetry.joints_min_delta` since the last publication (default 0.5 for `deg`, 0.01 for `rad`), plus once on every (re)connect and announce. A report that lands inside the rate interval is held and published by a trailing one-shot once the interval has passed, so the final resting pose of a motion always reaches the wire. `seq` increments per publication within the session; a publish that the link refuses (disconnected) consumes no `seq` and the sample goes out at the reconnect. Report your starting pose once in `setup`, as the fake does, so the sensors have a value before the first task.
- **The fake executor** simulates seven joints, `j1` to `j7` in degrees with limits of [-170, 170] except `j4` and `j6` at [-120, 120], starting at `home` (all zero). `reset` and `move_to {home, pick, place, wave}` interpolate linearly from the current positions to the target over `fake.task_duration_s`, reporting on every feedback tick and at the end; `wait` holds; `fail` moves halfway towards `pick` and aborts; `pause` freezes and `resume` continues; a cancel, `stop` or `stop_and_hold` freezes the joints where they are. Positions persist across tasks, so a dashboard sees a continuous arm.
- **The MoveIt executor** forwards `sensor_msgs/JointState` (below).

## Results, codes and how they reach Home Assistant

`TaskResult.status` is one of `succeeded`, `cancelled`, `aborted` and maps to `action_msgs/GoalStatus` 4, 5, 6. `code` should match `^[a-z][a-z0-9_]{0,63}$` and come from the contract's recommended vocabulary where one fits: `ok`, `cancelled`, `preempted`, `collision`, `self_collision`, `no_trajectory`, `planning_failed`, `control_failed`, `timeout`, `invalid_goal`, `controller_error`, `inhibited`, `executor_error`, `executor_unresponsive`, `nothing_to_pause`, `unknown`. The vocabulary is open (`docs/mqtt-contract.md`, Versioning): the MoveIt executor adds `unavailable` for "MoveIt or its server is not in a state to try, retry later". `reason` is one human sentence; `data` is free-form JSON for automations to branch on.

What happens to a result after `finish`:

1. The handle marks itself finished at once and posts the result to the main thread.
2. `TaskManager._complete` sanitises it: an unknown `status` becomes `aborted/executor_error`; a non-string `code` becomes `unknown`; `data` must be a JSON-encodable mapping under 8 KiB, else it is replaced by `{"unserialisable": "..."}` or `{"truncated": true, "bytes": n}`; a lone surrogate in a string becomes U+FFFD. Each repair is logged once.
3. The `code` is normalised onto the wire pattern when the payload is built (`protocol.normalise_result_code`: lower-cased, other characters to `_`, 64 characters at most, `unknown` if nothing remains).
4. The result is published once, retained, QoS 1, Message Expiry Interval `reply.result_expiry_s` (300 s), on `{prefix}/{robot_id}/task/{task_id}/result` with `kind` repeated from the spec; then the state goes `idle` (motion) or stays as it was (control, query). A failed task alone never sets the robot's `error`; only `ctx.set_error` and a forced finish do.
5. Home Assistant matches the result by id, fires `ros2_bridge_task_result` exactly once (with `replayed: true` for a retained delivery after a restart), updates the robot's `last_result` sensor for motion tasks only, triggers the task-result event entity, and releases a `send_task` caller waiting with `wait: result`. A result that arrives after HA has already synthesised an outcome fires `ros2_bridge_task_late_result` instead.

Acks are not yours to write: `accepted` is published when `start` returns, and the rejections you can cause are `rejected_executor` (from `TaskRejected` or an exception) and nothing else.

## The fake executor as a template

`ros2_ha_bridge/executors/fake.py` is the shape to copy. Its parts, top to bottom:

- Module constants: the entry-point name, parameter defaults, the announced timeout of the pathological task, the failure classes it can imitate.
- The catalogue as a module-level tuple of `TaskSpec`, with the params schemas written out, and a name-to-spec mapping for `start`.
- A `_Run` dataclass per task in flight: request, handle, timers, what it will finish with.
- The seven joints as module constants (`JOINT_NAMES`, `JOINT_LIMITS`, `JOINT_INFO`, the `POSES` table) and `interpolate`, the one pure function behind every motion.
- `setup`: keep the context, take the child logger, read `fake.task_duration_s` and `fake.feedback_hz` from `ctx.parameters` with tolerant coercion (a bad value falls back to the default), require `ctx.scheduler`, report the starting pose once.
- `joints`: return `JOINT_INFO`.
- `start`: look the spec up, run `protocol.validate_params(spec.params, req.params)` and raise `TaskRejected` with the problems, then dispatch by task name. Motion tasks arm a completion timer and a feedback ticker through `ctx.scheduler`, and the ticker both reports feedback and moves the joints a step along the way; control tasks finish synchronously inside `start`.
- `cancel`: pop the run, freeze the joints where the motion was, disarm its timers, `finish(cancelled/cancelled)`. An id it does not know is logged and ignored, which is also how `hang` exercises the bridge's forced finish.
- `stop_and_hold`: drain every run and finish each `cancelled/inhibited`.
- `health`: return a field a test can set; `shutdown`: drain the runs.
- Private helpers that raise `RuntimeError` when used before `setup`.

Its test, `test/test_fake_executor.py`, is the template for testing (below).

## The MoveIt 2 executor, walked through

Experimental: this executor has been exercised only against a fake `MoveGroupAdapter` (`test/test_moveit_executor.py`) and has not yet run against a real `move_group`; the first run happens at the simulator site through `docs/simulator_checklist.md`.

`ros2_ha_bridge/executors/moveit.py` is the reference for a real arm: it drives `move_group` through a plain rclpy `ActionClient` on `/move_action` (`moveit_msgs/action/MoveGroup`), the binding the Isaac Sim research recommends because planning and execution end in one `MoveItErrorCodes` value and the dependency set stays at rclpy plus moveit_msgs. It is selected with `executor: moveit` (the entry point in `setup.py`) or `ros2_ha_bridge.executors.moveit:MoveItExecutor`; `config/moveit.example.yaml` is a complete parameters file.

### Two layers

- **Pure core**, no rclpy or moveit_msgs anywhere: `MoveItSettings` (validated parameters), `parse_poses` and `load_poses_file`, `build_goal`, `map_error_code`, `build_catalogue`, `announce_joint_names` and `MoveItExecutor`. It talks to MoveIt through the `MoveGroupAdapter` protocol: `server_available()`, `send_goal(goal_dict, feedback) -> GoalHandle` (with `add_done(callback)` and `cancel()`), `subscribe_joint_states(topic, callback)` (0.2.0) and `close()`.
- **`RclpyMoveGroupAdapter`**, which imports `rclpy.action.ActionClient` and the `moveit_msgs` types inside its constructor, builds the `MoveGroup.Goal` message from the goal dict, and wraps the goal-response and result futures in a `_RclpyGoalHandle` that fires `add_done` exactly once with a `GoalOutcome` (accepted or not, terminal `GoalStatus`, `error_code`, `message`, `planning_time`). A cancel before acceptance is sent as soon as the goal is accepted. Its `subscribe_joint_states` creates a `sensor_msgs/JointState` subscription with the sensor-data QoS (best effort, which matches a reliable `joint_state_broadcaster` as well as a best-effort one) and hands each message on as plain name, position and velocity lists.

`MoveItExecutor()` takes an optional `adapter` (what the tests inject) or `adapter_factory`; the node passes neither and gets the rclpy adapter built on `ctx.node`.

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `moveit.group_name` | required | The planning group (`ur_manipulator`, `panda_arm`). |
| `moveit.poses` | required | Pose name to `{joint_name: position}` (inline, or flattened by rcl to `moveit.poses.<pose>.<joint>`), or the path of a YAML file holding that mapping, optionally under a `poses` key. At least one pose. |
| `moveit.planning_time_s` | `5.0` | `allowed_planning_time`. |
| `moveit.num_attempts` | `1` | `num_planning_attempts`. |
| `moveit.velocity_scale`, `moveit.acceleration_scale` | `0.1`, `0.1` | The two scaling factors, in (0, 1]. |
| `moveit.action_name` | `/move_action` | The MoveGroup action. |
| `moveit.hold_on_cancel` | `true` | Whether a cancelled task finishes only once move_group confirms the goal ended (below). |
| `moveit.joint_states_topic` | `/joint_states` | The `sensor_msgs/JointState` topic the joint telemetry (0.2.0) is read from. |
| `moveit.joint_names` | `[]` | The joints to report, in order; empty means the joints of the first configured pose, in their written order. |

Joint telemetry: `joints()` returns the chosen joints in radians with no limits (MoveIt does not publish them on `JointState`), under names made to fit the contract's pattern by `announce_joint_names` (lower-cased, other characters to `_`, 32 characters at most; `Gripper.Finger` becomes `gripper_finger`). When two names collapse into one or a name has nothing left, the executor logs a warning and announces no joints rather than a misleading set. Every `JointState` message that carries all the chosen joints is forwarded to `ctx.report_joints` in the announced order, with velocities when the message has one per name; a message that lacks one of them is skipped (one warning). The node throttles the stream, so the driver's 50 Hz or 500 Hz costs nothing on the broker.

`MotionPlanRequest` has no named-target field, so the pose is resolved here. In `setup` the executor merges `ctx.parameters` over what it reads from the node: the bridge's parameter table does not declare `moveit.*`, so the executor declares its scalars on `ctx.node` itself (read-only, dynamically typed) and rclpy applies the params-file overrides at declaration. Inline poses flatten to names nobody can declare in advance, so the executor first reads the node's parameter overrides (through `node.handle.get_parameters`) to discover them; if that ever fails it logs a warning, and a poses file (`moveit.poses` as a path) is the route that still works. A wrong value, an unreadable file or an empty pose list raises in `setup`, which stops the node before it announces. Unknown `moveit.*` names are logged as probable typos.

### Catalogue

- `move_to {pose}` (motion, `moveit.motion_timeout_s`, 120 s by default): `pose` is an enum of the configured names, so Home Assistant validates it before anything is published and the node validates it again at step 8.
- `home` (motion, `moveit.motion_timeout_s`): only when a pose named `home` exists, so there is a button for it.
- `stop` (control, `moveit.control_timeout_s`, 5 s by default): cancels the running move (it ends `cancelled/preempted`, "Stopped by task 'stop'") and clears the robot's error, so an operator can recover from a forced finish without another task. With `moveit.hold_on_cancel` true and a goal in flight, `ctx.clear_error()` is called only when move_group's terminal result arrives, so an `idle` robot always means the arm is holding; the `stop` result says so in its reason and carries `awaiting_confirmation: true`. With `hold_on_cancel` false, or nothing in flight, the error is cleared at once. A pending clear is dropped with a warning when `stop_and_hold`, `shutdown` or a newer task abandons the goal; send `stop` again then.
- `refresh_poses` (query, `moveit.control_timeout_s`): reloads the poses and reports `added`, `removed` and `changed`. Changed joint positions for existing names and removed poses take effect at once. A new pose name is refused by the node (step 8 validates `params` against the catalogue announced at start-up) and by Home Assistant (against the same enum) until the node restarts; `test_poses_from_a_yaml_file_and_refresh` shows both. A file that fails to parse leaves the current poses in place and returns `aborted/executor_error` with the reason.

### `start`, step by step

1. Unknown task name or parameters that fail the spec: `TaskRejected`. An unknown pose (possible after a refresh removed one) is `TaskRejected` with the known names in the reason. No MoveIt call has happened.
2. If a goal from an earlier task is still pending (the bridge forced that task, MoveIt never answered), its goal is cancelled first so two goals never overlap.
3. If `server_available()` is false, the task finishes at once with `aborted/unavailable` and the reason names the action; nothing is sent.
4. Otherwise `build_goal` produces the dict and the adapter sends it. The dict is `{'pose', 'request': {group_name, goal_constraints: [{name, joint_constraints: [{joint_name, position, tolerance_above, tolerance_below, weight}]}], num_planning_attempts, allowed_planning_time, max_velocity_scaling_factor, max_acceleration_scaling_factor}, 'planning_options': {'plan_only': False}}` with `moveit.joint_tolerance_rad` (0.01 rad by default) as `tolerance_above` and `tolerance_below` and weight 1.0 per joint. An exception from the adapter finishes the task `aborted/executor_error`.
5. Feedback: the action's `state` string (`PLANNING`, `MONITOR`, `IDLE`) is forwarded as `handle.feedback(None, state.lower(), 'move_group state: STATE')`.
6. The outcome: a rejected goal is `aborted/unavailable` (or `cancelled` with `hold_confirmed: true` when this executor had already asked for its cancel: nothing was executing); a result without a `MoveItErrorCodes` value is `aborted/executor_error`; otherwise the table below applies, and `data` carries `pose`, `group_name`, `action_name`, `moveit_error_code`, `moveit_error_name`, `message`, `goal_status`, `planning_time_s` and `error_class`.

### Cancel, stop and hold

Every cancel path sends `cancel_goal` to move_group, which stops trajectory execution and leaves the controllers holding position. `moveit.hold_on_cancel` decides when the bridge task is reported finished:

- `true` (default): the task stays `busy` until move_group's terminal result arrives, normally `PREEMPTED (-7)`, and then finishes `cancelled` with `hold_confirmed: true`. If the motion completed before the cancel took effect, the result is the real one (`succeeded/ok`). If move_group never answers, the bridge forces the task after `executor.cancel_grace_s`, so give that parameter the time your controller needs to stop.
- `false`: the task finishes `cancelled` with `hold_confirmed: false` as soon as the cancel request has been sent; the confirmation that follows is dropped.

The code says why: `cancelled/cancelled` ("Cancelled by request") for a cancel command or the task deadline, `cancelled/preempted` for the `stop` task, `cancelled/inhibited` for the watchdog's `stop_and_hold` (the bridge's override). A `PREEMPTED` that this executor did not ask for is another move_group client taking over and is reported `aborted/preempted`. The `stop` task's error clear follows the same flag: with `hold_on_cancel` true it happens when the confirmation arrives, not when `stop` returns (see the catalogue above).

Two codes to read together with the robot state. `aborted/timeout` with the robot `idle` is MoveIt's own `TIMED_OUT (-6)`: the planner or controller overran its allowance, an ordinary finish with `data.moveit_error_code` -6, and a retry is reasonable. `aborted/timeout` with the robot `error` is the bridge's forced finish after the task deadline and the cancel grace, with no MoveIt code in `data`. An automation that branches on `code` alone cannot tell them apart.

Stop hooks a real executor can use, as listed by the landscape research (`docs/safety.md`) and not verified by this project: MoveIt's trajectory execution stop (what `cancel_goal` on the MoveGroup action triggers), MoveIt Servo's command timeout, the `joint_trajectory_controller`'s command timeout and its decelerate-on-cancel behaviour, and an action server that sends a hold-position trajectory on cancel. This executor uses only the first; the others are an executor author's concern and belong in the simulator findings once one has been watched.

`health()` is `('ok', '')` while the action server is ready and `('error', 'move_group action server /move_action is not available')` otherwise, so Home Assistant's problem sensor shows a stopped move_group or a paused simulator. `shutdown()` cancels a goal still in flight (the bridge cannot report its outcome any more) and destroys the client.

### `MoveItErrorCodes` mapping

Every value of `moveit_msgs/msg/MoveItErrorCodes` at moveit_msgs 2.6.0 (the Jazzy release; -8, -9 and -20 are unused there). Success only when `val == 1`. The class column follows the research's proposal, except that `PREEMPTED (-7)` has its own class `preempted` here (the research filed a pre-empt by another client under EXECUTION_FAILED) so that `data.error_class` tells a foreign pre-empt from a controller failure; the status and code columns are what the bridge publishes; the MoveIt name and value always appear in the reason and in `data`.

| `val` | MoveIt name | Class | Bridge status / code |
|---|---|---|---|
| 1 | `SUCCESS` | succeeded | `succeeded` / `ok` |
| -1 | `PLANNING_FAILED` | planning_failed | `aborted` / `planning_failed` |
| -2 | `INVALID_MOTION_PLAN` | planning_failed | `aborted` / `planning_failed` |
| -3 | `MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE` | planning_failed | `aborted` / `planning_failed` |
| -10 | `START_STATE_IN_COLLISION` | planning_failed | `aborted` / `collision` |
| -11 | `START_STATE_VIOLATES_PATH_CONSTRAINTS` | planning_failed | `aborted` / `planning_failed` |
| -12 | `GOAL_IN_COLLISION` | planning_failed | `aborted` / `collision` |
| -13 | `GOAL_VIOLATES_PATH_CONSTRAINTS` | planning_failed | `aborted` / `planning_failed` |
| -14 | `GOAL_CONSTRAINTS_VIOLATED` | planning_failed | `aborted` / `planning_failed` |
| -31 | `NO_IK_SOLUTION` | planning_failed | `aborted` / `planning_failed` |
| -4 | `CONTROL_FAILED` | execution_failed | `aborted` / `control_failed` |
| -6 | `TIMED_OUT` | timed_out | `aborted` / `timeout` |
| -7 | `PREEMPTED` | preempted | `cancelled` / `cancelled` when this executor cancelled, else `aborted` / `preempted` |
| -15 | `INVALID_GROUP_NAME` | rejected | `aborted` / `invalid_goal` |
| -16 | `INVALID_GOAL_CONSTRAINTS` | rejected | `aborted` / `invalid_goal` |
| -17 | `INVALID_ROBOT_STATE` | rejected | `aborted` / `invalid_goal` |
| -18 | `INVALID_LINK_NAME` | rejected | `aborted` / `invalid_goal` |
| -19 | `INVALID_OBJECT_NAME` | rejected | `aborted` / `invalid_goal` |
| -26 | `START_STATE_INVALID` | rejected | `aborted` / `invalid_goal` |
| -27 | `GOAL_STATE_INVALID` | rejected | `aborted` / `invalid_goal` |
| -28 | `UNRECOGNIZED_GOAL_TYPE` | rejected | `aborted` / `invalid_goal` |
| -5 | `UNABLE_TO_AQUIRE_SENSOR_DATA` (MoveIt's spelling) <!-- codespell:ignore aquire --> | unavailable | `aborted` / `unavailable` |
| -22 | `COLLISION_CHECKING_UNAVAILABLE` | unavailable | `aborted` / `unavailable` |
| -23 | `ROBOT_STATE_STALE` | unavailable | `aborted` / `unavailable` |
| -24 | `SENSOR_INFO_STALE` | unavailable | `aborted` / `unavailable` |
| -25 | `COMMUNICATION_FAILURE` | unavailable | `aborted` / `unavailable` |
| -21 | `FRAME_TRANSFORM_FAILURE` | internal_error | `aborted` / `executor_error` |
| -29 | `CRASH` | internal_error | `aborted` / `executor_error` |
| -30 | `ABORT` | internal_error | `aborted` / `executor_error` |
| 0 | `UNDEFINED` | internal_error | `aborted` / `executor_error` |
| 99999 | `FAILURE` | internal_error | `aborted` / `executor_error` |
| any other | `UNKNOWN` | internal_error | `aborted` / `executor_error` |

Outcomes that never yield a code: goal rejected by move_group and action server absent: `aborted` / `unavailable` (a goal rejected after this executor asked for its cancel is `cancelled`). Controllers missing after an Isaac Stop/Play are not detected here: move_group then reports -4 `CONTROL_FAILED`, or nothing until the bridge's own timeout; `docs/simulator_checklist.md`, section 7, has the Stop/Play outcomes. A result whose `GoalStatus` is `CANCELED` after this executor asked for the cancel is reported `cancelled` even if the code is not `PREEMPTED`. MoveIt does not tell self-collision from environment collision by code, nor "no trajectory" from "planning failed", so this executor never produces `self_collision` or `no_trajectory`; `data.message` carries MoveIt's text for anyone who needs the difference.

`test/test_moveit_executor.py` runs every row of the table through a real `TaskManager` and validates each published result against the contract schema.

## Packaging an executor in another package

An executor does not have to live in this repository. In your own ament_python package:

1. Depend on the bridge: `<exec_depend>ros2_ha_bridge</exec_depend>` in `package.xml`, and import `ros2_ha_bridge.executors.base` (and `ros2_ha_bridge.protocol.validate_params` if you want the same parameter checking).
2. Register an entry point in `setup.py`:

   ```python
   setup(
       name="my_arm_bridge",
       entry_points={
           "ros2_ha_bridge.executors": [
               "my_arm = my_arm_bridge.executor:MyArmExecutor",
           ],
       },
   )
   ```

   After `colcon build` and sourcing the workspace, `executor: my_arm` loads it; until then `executor: my_arm_bridge.executor:MyArmExecutor` loads it by path. `load_executor` lists the installed names in its error when a name is unknown.
3. Keep rclpy and driver imports out of module scope (a function or constructor import, or `importlib.import_module` as `moveit.py` does), so the module imports and tests on any machine and `mypy --strict` passes without the driver's stubs.
4. Parameters: declare your own on `ctx.node` in `setup` (rclpy applies the params file at declaration), or read a file whose path is one parameter. `ctx.parameters` will carry only the bridge's own table unless the bridge is changed.
5. Start every file with the two SPDX lines if you contribute it back, and follow `CONTRIBUTING.md` (British English, no em-dashes, no hosts or secrets in examples).
6. Give the node a parameters file like `config/moveit.example.yaml`: `executor` naming your entry point, your own `<name>.*` block, the broker placeholders unchanged.

## Testing with the bridge's fakes

The rclpy-free core and its fakes let an executor be tested on any Python 3.12 or newer, with no ROS and no broker. The pattern used by `test/test_fake_executor.py` and `test/test_moveit_executor.py`:

- `FakeClock` and `FakeScheduler` from `test/fakes.py`: time moves only through `clock.advance(seconds)`, which fires due timers in order, so deadlines, the cancel grace and feedback throttling are deterministic. The `clock` and `scheduler` fixtures in `test/conftest.py` build them.
- A real `TaskManager(robot_id=..., executor=yours, publish=capture, post=lambda fn: fn(), clock=clock, scheduler=scheduler, session=..., instance=...)`, with `publish` appending `(topic, payload, qos, retain, expiry_s)` to a list after encoding the payload with `protocol.encode_payload` (so anything the encoder rejects fails in the test, not on the robot).
- `executor.setup(manager.bridge_context(ha_link_ok=lambda: True, parameters={...}, report_joints=capture))` with your parameters as a plain dict and, if you report joints, a `report_joints(positions, velocities)` callable that records what arrived; `ctx.node` stays `None`, so your rclpy layer must be replaceable, which is what the MoveIt executor's `adapter` argument is for. The throttle itself is tested on its own in `test/test_joints_publisher.py`; an executor test only needs to see the right numbers arrive in the right order.
- Commands built as JSON and decoded with `protocol.decode_command`, resolved with `authoriser.resolve_request(command, spec)`, then `manager.start(request, spec)` returns the ack and `manager.cancel(command)` the cancel ack. The rig classes in the two test files wrap this; copy one.
- `schema_validator` (fixture) validates every published payload against the vendored contract schemas: `schema_validator.assert_valid('result', payload)`.
- `assert_imports_without('your.module', {'rclpy', 'paho'})` (from `test/conftest.py`) imports the module in a fresh interpreter and fails if a forbidden package was pulled in.
- Cover at least: the catalogue validates (`protocol.validate_task_spec`) and announces (`Announce(...).to_payload()` against the `announce` schema, with `joints=AnnounceJoints.from_joint_info(executor.joints(), 2.0)` if you have joints, which `protocol.validate_joint_info` must accept); a bad parameter is refused before any driver call; a success publishes `succeeded` with the data you promise; each failure class maps to the code you document; cancel ends the task `cancelled` within the grace; the forced path (`clock.advance(timeout)` then `clock.advance(5)`) gives `aborted/timeout` and the robot in `error`; `stop_and_hold` ends the task `cancelled/inhibited`; `health` reports what you claim; joint reports carry one value per announced name in the announced order.

For the full node against a fake broker, `test/test_bridge_node.py` needs rclpy and runs under `colcon test` in the Jazzy container (`tools/README.md`); an executor's own node-level test can follow it.
