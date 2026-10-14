# The example dashboard

`docs/examples/dashboard.yaml` is a complete Lovelace dashboard for the two fake arms of `tools/compose.yaml` (`fake_arm_1` and `fake_arm_2`), built for a phone first and from native cards only. It is the dashboard the joint telemetry of 0.2.0 was added for (`docs/decisions.md`, 2026-10-14): each arm's seven joints as gauges that move while a task runs, next to the arm's state, its task buttons and its results. This page explains how to install it, what each view shows, where the gauge limits come from and how to adapt it to other robots.

## Installing it

1. Settings, Dashboards, Add dashboard, "New dashboard from scratch". Give it the title "Robot arms" and create it.
2. Open the new dashboard and press the pencil at the top right (Edit dashboard).
3. Open the three-dot menu at the top right and choose "Raw configuration editor".
4. Delete what the editor shows, paste the whole file, save, close the editor and press Done.

The file is a dashboard's complete configuration (a top-level `views:` list), so it replaces whatever a dashboard already holds; copy an existing dashboard's raw configuration out first. The YAML uses anchors (`&band_170`) and aliases (`*band_170`) for the repeated gauge bands; Home Assistant expands them when it saves, so the stored configuration contains plain values and the raw editor shows them expanded from then on.

The cards show "Entity not found" until the arms have announced with their `joints` block: start the two fake arms (`tools/README.md`) and the integration creates the joint sensors within a few seconds. The entity ids the file expects are listed in its leading comment; confirm them on each device page (Settings, Devices and services, ROS 2 Bridge, then the device) the first time.

## The views

The dashboard has three views, shown as tabs: Overview, Fake arm 1 and Fake arm 2. All three use the sections layout, which stacks everything in one column on a phone and spreads the sections over two or three columns on a tablet or a desktop.

### Overview

One block per arm, the arm's name as its heading. A red Problem badge appears next to the name only while the arm's problem sensor is on (robot in `error`, executor unhealthy, stale heartbeat or a duplicate id; `docs/install-ha.md`). Below the heading:

- the State tile (Idle, Busy or Error, and how long ago it changed);
- a row of five buttons, Reset, Stop, Pause, Resume and Cancel, pressing the button entities the integration creates (`button.fake_arm_1_reset_to_home`, `_stop`, `_pause`, `_resume`, `_cancel_task`);
- the Last result tile (Succeeded, Cancelled, Aborted, Rejected, Timed out or Lost, and when).

A refusal shows as an error on the button, as it does on the device page: pressing Reset while the arm is busy reports `rejected_busy`.

### Fake arm 1 and Fake arm 2

The same five sections for each arm, in this order on a phone:

1. **Status**: the Problem badge on the heading, the State tile across the full width, then Last result and Problem side by side.
2. **Joints**: seven gauge cards, two per row on a phone, one per joint in the announced order (J1 to J7). Each needle shows the joint's current position in degrees against its announced limits; the outer bands are amber so that a joint near its limit is obvious at a glance. Tapping a gauge opens the sensor's more-info dialog with its history and attributes.
3. **Tasks**: the same five buttons as the Overview, then a row of named poses, Pick, Place and Wave. The pose buttons call `ros2_bridge.send_task` with `task: move_to` and `params: {pose: ...}`, which is how the gauges are made to move from the dashboard; Reset returns to the home pose. The fake executor interpolates between poses over `fake.task_duration_s` (2 s by default), so each press is a visible sweep. The buttons sit directly under the gauges so that a press and its effect are on one screen.
4. **Last 30 minutes**: a history graph of the seven joint sensors with a fixed axis of -170 to 170 degrees, so a sweep looks the same whichever joints move. It needs the recorder to record the joint sensors (see below).
5. **Task results**: a logbook card on the arm's task-result event entity, listing each outcome of the last 24 hours as it fires (the event type is the outcome: succeeded, cancelled, aborted, rejected, timed out or lost). Tapping an entry opens the entity, whose attributes hold the task, code, reason and duration of the latest result.

The fake executor's test tasks (Hang, Raise error, Clear error) have buttons on the device page but not on the dashboard; copy a button block and change the entity id to add one.

## Where the gauge limits come from

The gauge card cannot read its range from an entity, so `min` and `max` are static values in the YAML, copied from the limits the robot announces (`docs/mqtt-contract.md`, the `joints` block). The fake executor announces seven joints in degrees, `j1`, `j2`, `j3`, `j5` and `j7` at -170 to 170 and `j4` and `j6` at -120 to 120; the file carries exactly those numbers, with amber bands over the outer 20 degrees (15 for the two narrower joints) and the theme's info colour for the working range. The same limits are on each joint sensor as the attributes `limit_min` and `limit_max`, and in the integration's diagnostics download, so a mismatch can be checked without reading the broker.

If the robot's limits change (a different executor, a different arm) the gauges do not follow by themselves: edit `min`, `max` and the `from` values of the bands, and `min_y_axis` and `max_y_axis` on the history graph. A robot that announces `limits: null` needs a range of your choosing. A MoveIt executor reports radians, so the unit changes as well: the sensors carry it, but the gauges and the graph axis need matching numbers, for instance -3.2 to 3.2.

## Adapting it to other robots

- **Entity ids follow the device name, not the robot id.** Home Assistant builds every entity id from the device name in the announce and the entity's name, lower case with underscores: a robot announced as "Left arm" has `sensor.left_arm_state`, `button.left_arm_reset_to_home` and `sensor.left_arm_joint_1_shoulder_pan` (if its first joint is named `shoulder_pan`), whatever its robot id. The pose buttons are the exception: `ros2_bridge.send_task` takes the robot id (`robot_id: arm_left`).
- **Joints.** One gauge and one history-graph entry per announced joint; the sensors are numbered from 1 in the announced order and carry the joint name after the number. Add or delete gauges to match, and set each gauge's `min`, `max` and bands from the announced limits.
- **Tasks and poses.** The five task buttons exist for any executor that announces `reset`, `stop`, `pause` and `resume` without parameters (the MoveIt reference executor does); the Cancel button exists for every robot. The pose names are the fake executor's; use the names your executor accepts for `move_to`, or remove the row.
- **More arms.** Copy one arm's section in the Overview and one whole arm view, then replace the ids and names.
- **Headings and labels** are plain text; change them freely.

## The recorder and the 30-minute graph

The joint sensors are recorded like any other sensor. An arm in motion produces up to two state changes per joint per second (the node's `telemetry.joints_rate_hz`, and only when a joint moved by more than `telemetry.joints_min_delta`), which is what the history graph draws. `docs/sensors.md` shows the recorder exclude (`entity_globs: sensor.*_joint_*`) for installations that do not want them in the database; with the exclude in place the gauges still move, because they show the live state, but the history graph stays empty. The graph's `hours_to_show: 0.5` is set in YAML on purpose: the card's visual editor only offers whole hours, so change it in the raw configuration editor, or set it to `1`.

## After pasting

- Open each view on the phone once; a card that names a missing entity shows "Entity not found" with the id, which is the id to fix.
- Press Pick on one arm and watch its gauges sweep; press Reset and watch them return. Pause freezes the needles; Resume continues.
- Settings, System, Logs shows a frontend warning if a card option is not understood by the running Home Assistant version; the file was written against Home Assistant 2026.10 and has not yet been opened on a live instance.
