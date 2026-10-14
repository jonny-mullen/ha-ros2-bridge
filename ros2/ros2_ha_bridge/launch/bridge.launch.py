# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Launch one bridge node from a parameter file.

    ros2 launch ros2_ha_bridge bridge.launch.py params_file:=/path/to/my_robot.yaml

``params_file`` defaults to the package's ``config/bridge.example.yaml`` (the
fake executor, robot ``fake_arm_1``, broker ``localhost``). The node is
respawned after an exit, two seconds later, at most three times, so that a
node that exits on purpose (session taken over by a duplicate robot id, no
CONNACK within ``mqtt.connect_timeout_s``) does not flap for ever.

Secrets never travel as launch arguments or parameters here: the two
environment variables ``ROS2_HA_BRIDGE_MQTT_PASSWORD`` and
``ROS2_HA_BRIDGE_COMMAND_SECRET`` are passed through from the launching
shell, and the node reads them when the matching parameter is empty.
"""

# ruff reads the package's own launch/ directory as the first-party module "launch",
# which is why the import order below is exempted from I001.
from launch import LaunchDescription  # noqa: I001
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE = 'ros2_ha_bridge'
SECRET_ENVIRONMENT = ('ROS2_HA_BRIDGE_MQTT_PASSWORD', 'ROS2_HA_BRIDGE_COMMAND_SECRET')
RESPAWN = {'respawn': True, 'respawn_delay': 2.0, 'respawn_max_retries': 3}


def secret_environment() -> dict:
    """The two secret variables, passed through unchanged (empty when unset)."""
    return {name: EnvironmentVariable(name, default_value='') for name in SECRET_ENVIRONMENT}


def generate_launch_description() -> LaunchDescription:
    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare(PACKAGE), 'config', 'bridge.example.yaml']
        ),
        description='Parameter file for the node (see config/bridge.example.yaml).',
    )
    node_name = DeclareLaunchArgument(
        'node_name', default_value='bridge_node', description='Name of the node.'
    )
    log_level = DeclareLaunchArgument(
        'log_level', default_value='info', description='ROS log level for the node.'
    )
    node = Node(
        package=PACKAGE,
        executable='bridge_node',
        name=LaunchConfiguration('node_name'),
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file')],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        additional_env=secret_environment(),
        **RESPAWN,
    )
    return LaunchDescription([params_file, node_name, log_level, node])
