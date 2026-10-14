# Installing the ROS 2 side

This page installs `ros2_ha_bridge`, the ROS 2 half of ha-ros2-bridge, on the machine that drives the robot: Ubuntu 24.04 with ROS 2 Jazzy (interview decision Q6). One `bridge_node` process runs per robot. The Home Assistant half is installed separately (`docs/install-ha.md`, phase 3), and the two only ever meet on the MQTT broker.

Nothing here moves a real arm: the package ships with the `fake` executor selected, and a real executor is a deliberate change to the `executor` parameter (`docs/writing-an-executor.md`, phase 7).

## Requirements

- Ubuntu 24.04 (Noble) with ROS 2 Jazzy installed from the official apt repository, `ros-jazzy-ros-base` or larger. Python 3.12 comes with it.
- An MQTT 5 broker that Home Assistant uses too (the Mosquitto add-on qualifies), reachable from this machine, with a user the node may use. The stock Mosquitto add-on does not enforce topic ACLs (see `SECURITY.md`), which is why the node authorises every command itself.
- paho-mqtt 2.1.0 from pip. Ubuntu's `python3-paho-mqtt` is 1.6.1 and lacks the MQTT 5 subscribe options and the VERSION2 callback API the node uses; the node refuses to import an older paho.
- Both machines on NTP within 2 s of each other. Commands carry an issue time and a maximum age, and `tools/clock_check.py` measures the offset.

## 1. Install ROS 2 Jazzy

Follow the official Jazzy installation guide for Debian packages (sources, keys, then `sudo apt install ros-jazzy-ros-base`), and the build tools:

```text
sudo apt update
sudo apt install ros-jazzy-ros-base python3-colcon-common-extensions python3-rosdep python3-pip
sudo rosdep init   # once per machine; skip if it says it is already initialised
rosdep update
```

Source ROS 2 in every shell that builds or runs the node (or add the line to `~/.bashrc`):

```text
source /opt/ros/jazzy/setup.bash
```

## 2. Get the package into a workspace

The package lives at `ros2/ros2_ha_bridge` in the repository. Clone the repository and put the package in a colcon workspace, either by copying it or, when the clone stays where it is, by symlinking it:

```text
mkdir -p ~/bridge_ws/src
git clone https://github.com/<owner>/ha-ros2-bridge.git ~/ha-ros2-bridge
ln -s ~/ha-ros2-bridge/ros2/ros2_ha_bridge ~/bridge_ws/src/ros2_ha_bridge
```

Replace `<owner>` with the repository's owner once it is published; until then, copy the folder from wherever you have it.

## 3. Install the dependencies

The ROS dependencies come from rosdep. paho-mqtt comes from pip, and on Ubuntu 24.04 pip refuses to install into the system Python unless told that this is intended (PEP 668, "externally managed environment"). Two ways round it; pick one.

To use the MoveIt 2 executor (`executor: moveit`), run the `rosdep install` line of either option as `ROS2_HA_BRIDGE_WITH_MOVEIT=1 rosdep install --from-paths src --ignore-src -y --rosdistro jazzy` instead, or `sudo apt install ros-jazzy-moveit-msgs` by hand; `package.xml` declares `moveit_msgs` under that condition only, so the plain command below does not install it.

Option A, install into the system Python (simplest; the ROS 2 Python modules are there already):

```text
cd ~/bridge_ws
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
python3 -m pip install --break-system-packages -r src/ros2_ha_bridge/requirements.txt
```

Option B, a virtual environment that can still see the ROS 2 modules (`--system-site-packages` is what makes `import rclpy` work inside it):

```text
cd ~/bridge_ws
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
python3 -m venv --system-site-packages ~/bridge_venv
~/bridge_venv/bin/python -m pip install -r src/ros2_ha_bridge/requirements.txt
```

With option B, activate the environment (`source ~/bridge_venv/bin/activate`) before `colcon build` and before running the node, so that the entry point script resolves paho from the environment. Whichever option you chose, check the version:

```text
python3 -c "import paho.mqtt; print(paho.mqtt.__version__)"
```

It must print `2.1.0`.

## 4. Build

```text
cd ~/bridge_ws
source /opt/ros/jazzy/setup.bash
colcon build --event-handlers console_direct+
source install/setup.bash
```

To run the package's own tests here as well (pytest, the contract tests against the vendored schemas, the rclpy adapter tests, and the ament linters), install `python3-pytest`, `ros-jazzy-ament-flake8`, `ros-jazzy-ament-pep257` and `jsonschema` (pip, the same way as paho), then `colcon test --event-handlers console_direct+` followed by `colcon test-result --verbose`.

## 5. Write the parameters file

Every node parameter has a default, listed in `docs/configuration.md`, and `config/bridge.example.yaml` in the package shows them all. Copy that file and change what differs on your site. The minimum for one robot is its id and the broker:

```yaml
/**:
  ros__parameters:
    robot:
      id: arm_left              # ^[a-z][a-z0-9_]{0,31}$; "ha" is reserved
      name: Left arm            # the device name shown in Home Assistant
    mqtt:
      host: broker.example.invalid
      port: 1883
      username: changeme        # empty means an anonymous connection
    topic_prefix: ros2_bridge   # must match the Home Assistant integration
    executor: fake              # the shipped fake executor; nothing moves
```

The MQTT password and the optional command secret are never put in this file or on a command line (see the next step). `robot.id` must be unique per robot on the broker: the MQTT client id is derived from it on purpose, so two nodes claiming the same id collide visibly instead of silently sharing a robot.

## 6. Set the two environment variables

The node reads two secrets from the environment when the matching parameter is empty:

| Variable | Parameter it fills | Meaning |
|---|---|---|
| `ROS2_HA_BRIDGE_MQTT_PASSWORD` | `mqtt.password` | The broker password for `mqtt.username`. Leave unset for an anonymous broker. |
| `ROS2_HA_BRIDGE_COMMAND_SECRET` | `command.secret` | The shared HMAC secret. Unset or empty means commands are not signed and the node accepts unsigned ones (the default, Q15). Set the same value in the Home Assistant integration to switch signing on. |

For an interactive run:

```text
export ROS2_HA_BRIDGE_MQTT_PASSWORD='the-broker-password'
```

For a service, put them in a root-only file such as `/etc/ros2_ha_bridge/arm_left.env` (mode 0600) and point the unit's `EnvironmentFile=` at it; never write them into the parameters file or a launch argument, which would land in process listings and logs.

## 7. Run

```text
source /opt/ros/jazzy/setup.bash
source ~/bridge_ws/install/setup.bash
ros2 launch ros2_ha_bridge bridge.launch.py params_file:=/path/to/arm_left.yaml
```

The launch file starts one `bridge_node` with the parameters file, passes the two environment variables through, and respawns the node (two seconds delay, at most three retries) if it exits, which is also how a node that keeps losing its MQTT session to a duplicate robot id is stopped from flapping for ever. For a second robot, run a second launch with its own parameters file; `fleet.launch.py` starts several from one list.

Without launch, the same node runs as:

```text
ros2 run ros2_ha_bridge bridge_node --ros-args --params-file /path/to/arm_left.yaml
```

The log tells you what happened next. `connected to the MQTT broker as 'ros2_ha_bridge_arm_left'` is the good case. A refused connection is logged once in plain words (bad user name or password, not authorised, the broker does not support MQTT 5) and retried with backoff; a broker that cannot be reached at all is logged once and retried for ever unless `mqtt.connect_timeout_s` is set, in which case the node exits non-zero after that many seconds without an answer.

## 8. Verify on the broker

With `mosquitto-clients` installed (`sudo apt install mosquitto-clients`) on any machine that can reach the broker:

```text
mosquitto_sub -h broker.example.invalid -p 1883 -u changeme -P 'the-broker-password' -V 5 -v -t 'ros2_bridge/#'
```

You should see at once, retained: `ros2_bridge/arm_left/availability online`, the announce (with the task catalogue, a `session` and an `instance` of the form `host:pid`) and the `idle` state; then a heartbeat every two seconds with the node's counters. Stop the node cleanly (Ctrl-C) and the availability turns `offline`; kill it and the broker publishes the same `offline` as the Last Will. `docs/mqtt-contract.md` describes every payload, and `tools/README.md` shows how to send a command by hand and the chaos checks.

## Running as a service

A minimal systemd unit for one robot, as a sketch (adjust paths and the user):

```text
[Unit]
Description=ha-ros2-bridge node for arm_left
After=network-online.target
Wants=network-online.target

[Service]
User=ros
EnvironmentFile=/etc/ros2_ha_bridge/arm_left.env
ExecStart=/bin/bash -lc 'source /opt/ros/jazzy/setup.bash && source /home/ros/bridge_ws/install/setup.bash && exec ros2 launch ros2_ha_bridge bridge.launch.py params_file:=/etc/ros2_ha_bridge/arm_left.yaml'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

`Restart=on-failure` sits above the launch file's own respawn: the launch file retries a crashing node three times, and if the whole launch gives up, systemd restarts it after five seconds.

## The Docker route

For development, CI, or a PC without ROS 2, `tools/` packages the same node into a `ros:jazzy-ros-base` image with two fake arms, a test runner, a clock probe and an optional local broker; `tools/README.md` has the commands. It is the route the project's own end-to-end checks use. It is not a deployment route for a robot machine: the container has no access to the arm's drivers, and ROS 2 discovery is deliberately kept inside the compose network.

## Upgrading

Pull the new sources, re-run step 3 if `requirements.txt` or `package.xml` changed, then `colcon build` and restart the node. The contract (schema 1) only ever gains optional fields, so a newer node keeps working with an older Home Assistant integration and the other way round; `docs/upgrading.md` (phase 8) will list anything that needs attention per release.
