# Simulator checklist: the Isaac Sim site

This is the procedure for the site that has NVIDIA Isaac Sim (interview decisions Q12, Q13 and Q20 in `docs/decisions.md`): what to install, in which order to try things, which failures to provoke on purpose, and what to write down so that the findings can be acted on without Jonny or the developer present. It was written on 2026-10-14 from the research record `docs/research/2026-10-13-isaac-sim-moveit.json`, before anyone had run it; the "Unverified" section at the end lists what that research could not confirm. Everything here is simulation. Nothing in this project is an emergency stop, and no step here is a procedure for a physical arm.

The bridge never talks to Isaac Sim. The chain under test is:

```text
Home Assistant -> MQTT -> bridge_node (executor: moveit) -> /move_action -> move_group -> ros2_control -> Isaac Sim
```

Only the last two hops change between the three tiers below; the bridge, the MoveIt executor and the Home Assistant integration are the same throughout.

## 0. Before you start

- Read `docs/install-ros2.md`, `docs/install-ha.md`, `docs/configuration.md` and the MoveIt section of `docs/writing-an-executor.md`.
- Run the fake executor end to end first (`tools/README.md`): a problem that also shows with `executor: fake` is a bridge or Home Assistant problem, not a MoveIt or Isaac one.
- Decide where findings go: one dated file, `docs/research/YYYY-MM-DD-isaac-sim-findings.md`, plus a dated entry in `docs/decisions.md` for every choice the run forces (a changed default, a tier abandoned). The "What to record" section says what each step must leave behind.

## 1. Prerequisites

Versions pinned by the research on 2026-10-13 (the Jazzy rosdistro and NVIDIA's 6.1 documentation); check for newer ones but record what you actually used.

| Item | Pinned | Notes |
|---|---|---|
| Isaac Sim | 6.1.0 | GitHub tag v6.1.0 (10 Sep 2026, marked latest). 6.2.0 exists only as an NGC container tag with no docs; do not plan on it. Python 3.12. |
| OS | Ubuntu 24.04 | NVIDIA's recommended pairing with Jazzy for 6.1. Windows 10 is not supported by any 6.x; WSL2 is deprecated and cannot run the container. |
| ROS 2 | Jazzy (system install, Python 3.12) | `ros-jazzy-desktop` or `ros-base`; source it before launching Isaac Sim when the project workspace or custom interfaces are needed, or rely on Isaac Sim's bundled Jazzy when only common interfaces are used. |
| MoveIt | moveit 2.12.4 (`ros-jazzy-moveit`) | Includes `moveit_ros_move_group`, `moveit_simple_controller_manager` and `moveit_py` (not used by the bridge). |
| moveit_msgs | 2.6.0 | The executor's only MoveIt dependency. `export ROS2_HA_BRIDGE_WITH_MOVEIT=1` before `rosdep install` for the bridge workspace (the dependency is conditional in `package.xml`), or `sudo apt install ros-jazzy-moveit-msgs` by hand. |
| ros2_control | 4.49.0 | |
| ros2_controllers | 4.43.0 | `joint_trajectory_controller`, `joint_state_broadcaster`. |
| topic_based_hardware_interfaces | 1.1.0 | Tier 2 only (`joint_state_topic_hardware_interface/JointStateTopicSystem`, with an alias for the old plugin name since 1.1.0). PickNik's `topic_based_ros2_control` has no Jazzy release; do not use it. |
| simulation-interfaces | 1.5.0 or newer | For the Simulation Control services (`/set_simulation_state`, `/reset_simulation`, `/step_simulation`, `/load_world`), enabled with `--/isaac/startup/ros_sim_control_extension=True`. |
| GPU | RT cores, 16 GB VRAM or more | Minimum tier GeForce RTX 4080 16 GB; GPUs without RT cores (A100, H100) and every Pascal card are unsupported. Record the card and VRAM of the Isaac machine before scheduling anything. |
| NVIDIA driver | 595 series (6.1.0 was tested on Linux 595.58.03) | Or the current Production Branch for newer cards. |
| CPU, RAM, disk | 4 cores and 32 GB RAM minimum; 8 cores and 64 GB recommended; 50 to 500 GB SSD | From the 6.1 requirements table. |
| Container route (optional) | `nvcr.io/nvidia/isaac-sim:6.1.0` | Headless, Linux only, about 9.9 GB; `--gpus all -e ACCEPT_EULA=Y --network=host -u 1234:1234` plus the documented cache volumes. Whether an anonymous pull works is unverified; have an NGC key ready. |
| DDS | Fast DDS (`rmw_fastrtps_cpp`) on every participant, one `ROS_DOMAIN_ID` | Containers or two machines additionally need the UDPv4-only `~/.ros/fastdds.xml` exported as `FASTRTPS_DEFAULT_PROFILES_FILE` in every terminal and container before Isaac Sim starts. |
| Broker | The site's MQTT 5 broker, reachable from the bridge machine | Credentials in the environment, never in files that are committed (`docs/install-ros2.md`, step 6). |
| NVIDIA workspaces | `IsaacSim-ros_workspaces` `jazzy_ws` built with colcon | `isaacsim_bringup` (`run_isaacsim.launch.xml`), `isaac_moveit` (Franka, Tier 2 reference), `isaac_ros2_control_demo` (UR10, Tier 3 reference), `isaac_tutorials`. |
| ha-ros2-bridge | This repository, `ros2/ros2_ha_bridge` built in a colcon workspace | `docs/install-ros2.md`; paho-mqtt 2.1.0 from pip. The Home Assistant integration installed on the site's HA (`docs/install-ha.md`). |

Isaac Sim is free for internal research and development; the container needs `ACCEPT_EULA=Y`. Remote viewing (WebRTC client on TCP 49100 and UDP 47998, or the Compose web viewer on TCP 8210) has no authentication: private network only, one client per instance.

## 2. The three tiers behind one xacro switch

NVIDIA's `isaac_moveit` package and `moveit_resources`' `panda.ros2_control.xacro` select the hardware plugin with one xacro argument, `ros2_control_hardware_type`. Keep that pattern for the site's own robot description so the same MoveIt configuration serves all three tiers and only the launch argument changes.

| Tier | Where | Hardware plugin | What it proves |
|---|---|---|---|
| 1 | Any Ubuntu 24.04 with Jazzy, no GPU (a WSL2 distro or a container is enough) | `mock_components/GenericSystem` | The whole MQTT to executor to `move_group` to error-code path, and every negative test below. Run this tier first, and again after every change to the bridge. |
| 2 | The Isaac machine, Isaac Sim's classic bridge | `joint_state_topic_hardware_interface/JointStateTopicSystem` from `topic_based_hardware_interfaces` 1.1.0 | Real simulated dynamics with an external `ros2_control_node`. OmniGraph `ROS2 Publish Joint State` and `ROS2 Subscribe Joint State` on `/isaac_joint_states` and `/isaac_joint_commands` (renamed so they do not collide with `joint_state_broadcaster`'s `/joint_states`); plugin parameters `joint_commands_topic`, `joint_states_topic`, `trigger_joint_command_threshold`, and `sum_wrapped_joint_states: true` because Isaac reports positions wrapping between 2 pi and minus 2 pi; spawners for `joint_state_broadcaster` and the arm's `joint_trajectory_controller`; `use_sim_time: true`. NVIDIA's reference is the Franka `isaac_moveit.launch.xml`. |
| 3 | The Isaac machine, Isaac Sim 6.1 in-process ros2_control (`isaacsim.ros2.control` 0.1.6, Humble and Jazzy only) | None in the URDF: the `controller_manager` runs inside Isaac Sim, configured by a `ROS2ControlManager` OmniGraph node (`targetPrim`, `controllerConfig`, optional `urdfPath`, `namespace`, `publishRobotDescription`, `useSimTime`) or `Ros2ControlManager.setup(...)` | The URDF is synthesised from USD on each Play and the command interface per joint is inferred from the drive (stiffness above 0 gives position, damping only gives velocity, no gains gives effort). Exposes `/joint_states`, `/robot_description` (transient local), `/clock`, `<ns>/controller_manager` and `/<controller>/follow_joint_trajectory`, which MoveIt's `moveit_simple_controller_manager` drives. NVIDIA's verified reference is the UR10: `./python.sh standalone_examples/api/isaacsim.ros2.control/ur10_ros2_control_demo.py` then `ros2 launch isaac_ros2_control_demo ur10_in_process.launch.py`; `ros2_control_smoke_test.py --headless` prints `DEMO SMOKE TEST: PASS`. |

For every Isaac tier: `use_sim_time: true` everywhere with `/clock` from `ROS2 Publish Clock`; topics exist only while the simulation is playing; the executor must tolerate controllers being absent for a while and a stalled `/clock` while the simulation is paused. Two consequences for the bridge, both by design:

- The bridge's own timers (task deadlines, the cancel grace, heartbeats, the watchdog) run on a steady clock, never on sim time (`docs/architecture.md`, threading model). Pausing the simulation with a move in flight does not pause the task's timeout: the bridge asks for a cancel at the deadline and forces `aborted/timeout` after the grace, and the robot shows `error` until `stop` has been sent and move_group has confirmed the goal ended (with `moveit.hold_on_cancel` true; with it false the clear happens as soon as the cancel request is sent).
- While `move_group` or its controllers are absent, the MoveIt executor's health is `error` (Home Assistant's problem sensor comes on) and a `move_to` ends at once with `aborted/unavailable`; nothing is queued.

## 3. First smoke test: the clock

Before any arm test, on the Isaac machine (and from inside the bridge container if the bridge is containerised):

1. Start Isaac Sim headless (`./runheadless.sh` in the container, or `ros2 launch isaacsim_bringup run_isaacsim.launch.xml ros_distro:=jazzy headless:=webrtc standalone:=<script>`), load the scene, press Play (or let the standalone script play).
2. `ros2 topic echo /clock`. Messages must arrive and advance. Nothing arrives while the simulation is stopped or paused.
3. `ros2 topic list` shows `/joint_states` (Tier 3) or `/isaac_joint_states` (Tier 2); `ros2 control list_controllers` shows the broadcaster and the trajectory controller `active` (Tier 3 after the spawners, Tier 2 after `ros2_control_node` and its spawners).
4. Tier 3 only: run NVIDIA's `ros2_control_smoke_test.py --headless` and record `DEMO SMOKE TEST: PASS`.

If step 2 fails, fix DDS first (same RMW, same domain id, the UDPv4 profile in every shell and container, Isaac Sim started after the environment was set); nothing further is meaningful until `/clock` is seen from where the bridge will run. Record the exact environment that made it work.

## 4. Tier 1 negative tests, with the codes they should provoke

Run these with `mock_components/GenericSystem` first (no GPU needed), then repeat the ones that still apply in Tiers 2 and 3. Each row names the provocation, the `MoveItErrorCodes` value the research expects, and what the bridge publishes for it (`docs/writing-an-executor.md`, mapping table). Watch `{prefix}/{robot_id}/task/+/result` on the broker and the `last_result` sensor in Home Assistant. Record the value you actually saw in `data.moveit_error_code` and `data.moveit_error_name`; the mapping is the thing under test, and a surprise is a finding, not a failure of the procedure.

| Provocation | Expected MoveIt code | Expected bridge result |
|---|---|---|
| A pose with a joint well outside its limits or far outside the workspace (add it to `moveit.poses`, `refresh_poses`, restart the node so HA offers it) | -1 `PLANNING_FAILED` or -31 `NO_IK_SOLUTION` | `aborted` / `planning_failed`, MoveIt name in the reason |
| A collision object placed on the goal pose (publish it through the planning scene) | -12 `GOAL_IN_COLLISION` | `aborted` / `collision` |
| The arm started inside an obstacle (collision object around the current state) | -10 `START_STATE_IN_COLLISION` | `aborted` / `collision` |
| `moveit.planning_time_s: 0.001` (restart the node) | -6 `TIMED_OUT` | `aborted` / `timeout` |
| `moveit.group_name: no_such_group` (restart the node) | -15 `INVALID_GROUP_NAME` | `aborted` / `invalid_goal` |
| `send_task` with a pose name that is not configured | none; Home Assistant refuses it against the announced enum (`invalid_params`); forced past HA, the node answers `rejected_executor` before any MoveIt call | no result (ack-level rejection) |
| Cancel mid-motion (`cancel_task`, or the Cancel button, while a long move runs) | -7 `PREEMPTED` | `cancelled` / `cancelled`, `data.hold_confirmed: true` with the default `moveit.hold_on_cancel` |
| The `stop` task mid-motion | -7 `PREEMPTED` | the move ends `cancelled` / `preempted` ("Stopped by task 'stop'"); the stop task itself `succeeded` |
| A second move while one runs | none | ack `rejected_busy` |
| `move_group` not running | none | `aborted` / `unavailable`; executor health `error`; HA problem sensor on |
| Pause or stop Isaac Sim mid-trajectory (Tiers 2 and 3 only) | -4 `CONTROL_FAILED`, or no result until the bridge's own timeout | `aborted` / `control_failed`, or `aborted` / `timeout` forced by the bridge with the robot in `error` until `stop` has been sent and move_group confirms the goal ended |

Known flaky case to keep out of the bridge's account: the Franka hand close goal of NVIDIA's 6.1 MoveIt tutorial can fail or abort on some machines, and NVIDIA's 6.0.1 notes reported intermittent timeouts on Humble (Jazzy recommended). Use the arm group, not the hand, for these tests.

## 5. Running the bridge node with the MoveIt executor

On the machine that will host the bridge (the Isaac machine itself in the simplest topology; both native so Fast DDS can use shared memory):

1. Install the bridge as in `docs/install-ros2.md`, with the MoveIt dependency included:

   ```text
   cd ~/bridge_ws
   export ROS2_HA_BRIDGE_WITH_MOVEIT=1
   rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
   python3 -m pip install -r src/ros2_ha_bridge/requirements.txt   # or the venv route
   colcon build --event-handlers console_direct+
   source install/setup.bash
   python3 -c "import moveit_msgs.action, paho.mqtt; print(paho.mqtt.__version__)"
   ```

   The last line must print `2.1.0` and raise nothing.
2. Copy `ros2/ros2_ha_bridge/config/moveit.example.yaml` to a site file (for example `/etc/ros2_ha_bridge/sim_arm.yaml`) and set: `robot.id` (one per simulated arm; the example says `arm_left`), `robot.name`, `mqtt.host`, `mqtt.port`, `mqtt.username`, `topic_prefix` (must equal the integration's), `moveit.group_name` (the SRDF group: `ur_manipulator` for the UR10 reference, `panda_arm` for the Franka), and `moveit.poses` with joint names from that group and positions checked in RViz or the simulator. Leave the password out of the file.
3. Export the secrets and launch, with the simulation playing and `move_group` up:

   ```text
   export ROS2_HA_BRIDGE_MQTT_PASSWORD='...'
   source /opt/ros/jazzy/setup.bash && source ~/bridge_ws/install/setup.bash
   ros2 launch ros2_ha_bridge bridge.launch.py params_file:=/etc/ros2_ha_bridge/sim_arm.yaml
   ```

   The start-up log must show `executor 'moveit' loaded`, a `moveit executor ready` line naming the group, the action and the poses with `server_available=True`, and `connected to the MQTT broker as 'ros2_ha_bridge_<robot.id>'`. A configuration error (an unknown pose file, a scale outside (0, 1], an empty group) stops the node with status 2 and a plain message; fix the file and relaunch.
4. On the broker, `mosquitto_sub -V 5 -v -t '<prefix>/#'` shows the retained availability `online`, the announce with the tasks `move_to`, `home` (if a `home` pose exists), `stop` and `refresh_poses`, the `idle` state, and a heartbeat every two seconds.
5. In Home Assistant the robot appears (Discovered, confirm) with buttons for `home`, `stop` and `refresh_poses`, and `move_to` through Developer tools, Actions, `ros2_bridge.send_task` with `params: {pose: ...}` and `wait: result`.
6. Positive tests before the negative ones: `home` moves the arm and ends `succeeded/ok`; `move_to` each configured pose; `send_task` with `wait: result` returns the result inline; `refresh_poses` after editing the poses file reports `added`, `removed`, `changed`.
7. Isaac tiers: press Stop in Isaac Sim with the node running, then Play. See section 7 for what to expect and what to re-spawn.

`tools/README.md` has the `mosquitto_pub` form of a hand-built command for a broker without the integration; `docs/troubleshooting.md` maps every ack code to a cause.

## 6. What to record

One dated findings file, `docs/research/YYYY-MM-DD-isaac-sim-findings.md`, with these sections, each dated and naming the machine:

- **Environment.** Isaac Sim version and route (container or workstation), Ubuntu, kernel, GPU and VRAM, driver, `ros2 pkg xml moveit | grep version`, `moveit_msgs`, `ros2_control`, `ros2_controllers`, `topic_based_hardware_interfaces` (Tier 2), `simulation_interfaces`, the RMW and domain id, whether a `fastdds.xml` was needed, paho-mqtt version, the bridge commit.
- **Smoke test.** The `/clock` result from the bridge's host, and for Tier 3 the `DEMO SMOKE TEST: PASS` line.
- **Tier reached and the xacro switch value used.**
- **Codes seen.** For every row of section 4: the provocation as actually done, `data.moveit_error_code`, `data.moveit_error_name`, `data.message`, and the bridge's `status` and `code`. A code outside the table of `docs/writing-an-executor.md`, or a row that provoked a different code than expected, is a finding for the mapping and belongs in `docs/decisions.md` as a dated entry.
- **Latency numbers.** For a handful of commands from Home Assistant (a button press, a `send_task` from an automation, an Assist intent if voice is set up): the ack's `clock_skew_ms` (`received_at - issued_at`, which with both clocks on NTP is the HA to node command latency), `started_at - issued_at` from the result, and the result's `finished_at - started_at` (motion time). Run `tools/clock_check.py` on the bridge machine first and record its offset, so the latency figures can be corrected for it. These numbers feed phase 5's targets in `docs/architecture.md` (under 1 s for the automation path is the provisional figure).
- **Cancel timing.** How long move_group takes to confirm a cancel (`PREEMPTED`) in each tier, which decides whether `executor.cancel_grace_s` (5 s) is enough with `moveit.hold_on_cancel: true`.
- **Stop/Play behaviour.** What happened to the node, the executor health, a move in flight and the controllers on each Stop and Play (section 7).
- **Anything unverified that this run settled** (section 8), so the research record can be updated.
- **Open items** the run could not do, and why.

Then one dated entry per decision in `docs/decisions.md`: for example a changed `moveit.*` default, a tier declared not worth pursuing, a mapping row corrected, or the phase 7 decision itself ("Isaac verified on this date" or "0.1.0 released on fake-only verification").

## 7. The Stop/Play caveat

In Tier 3 (in-process ros2_control), pressing Stop in Isaac Sim tears the `controller_manager` down and Play recreates it, so the controllers must be re-spawned after every Stop/Play (NVIDIA's documented limitation; setup errors also latch until the next Stop/Play, there is one `ROS2ControlManager` per articulation, and the update rate is clamped to the physics rate). In Tier 2 the external `ros2_control_node` survives, but the joint-state topics fall silent while stopped. In both tiers `move_group` loses its controllers and the bridge sees it like this:

- A move in flight when Stop is pressed ends with -4 `CONTROL_FAILED` (`aborted/control_failed`) if move_group notices, or runs into the bridge's task timeout: cancel requested, no confirmation, `aborted/timeout` forced after `executor.cancel_grace_s`, robot in `error` until the `stop` task has been sent and move_group confirms the goal ended (with `moveit.hold_on_cancel` true the clear waits for that confirmation, so an `idle` robot means the arm is holding; with it false the clear is immediate).
- While stopped, the MoveGroup action server usually stays up, so a new `move_to` is accepted and then fails at MoveIt rather than at the bridge; if move_group itself is restarted the executor reports `aborted/unavailable` and health `error` until the server is back.
- After Play, re-spawn `joint_state_broadcaster` and the trajectory controller (Tier 3), confirm `ros2 control list_controllers` shows them `active`, confirm `/clock` advances, and send `home` again before trusting any further result.

The bridge node itself does not need a restart across a Stop/Play; if a run shows that it does, that is a finding.

## 8. Unverified at the time of writing

From the research record, the items this checklist depends on but that nobody has confirmed yet; each one is a line in the findings file once settled:

- Enabling the Jazzy bridge inside the `nvcr.io/nvidia/isaac-sim:6.1.0` headless container is expected to work by composition of documented pieces; no NVIDIA page walks through it, which is why the `/clock` smoke test comes first.
- Whether an anonymous `docker pull` of the 6.1.0 container works without an NGC key.
- Whether a GPU below the published minimum runs a simple arm scene.
- The exact apt names `ros-jazzy-joint-state-topic-hardware-interface` and `ros-jazzy-moveit-py` follow the convention but were not read from a package index.
- Fetching the SRDF's named group states from move_group's `robot_description_semantic` parameter instead of a poses file was not researched; the executor reads its own poses on purpose.
- The MoveIt code each negative test provokes is the research's expectation, not an observation; MoveIt's answer for a joint goal outside its limits in particular may be a request-error code (-16 or -27, `invalid_goal`) rather than -1.
- The executor's reading of inline `moveit.poses.*` entries from the node's parameter overrides uses rclpy's node handle (`handle.get_parameters`) and has not been run on a Jazzy node here; if the start-up log warns that the overrides could not be read, set `moveit.poses` to a file path, which is the route the unit tests cover, and record it.
- The GPU and VRAM of the site's Ubuntu machine against the RT-core and 16 GB requirement.
