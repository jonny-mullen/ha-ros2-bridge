# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Launch one bridge node per robot from a fleet file.

    ros2 launch ros2_ha_bridge fleet.launch.py fleet_file:=/path/to/fleet.yaml

The fleet file (default: the package's ``config/fleet.example.yaml``, two fake
arms) names the robots under a ``robots`` key, either as a mapping of node
name to settings or as a list::

    robots:
      arm_left:                               # the node name
        params_file: bridge.example.yaml      # relative to the fleet file, or absolute
        parameters:                           # optional overrides applied after the file
          robot.id: arm_left
      arm_right:
        params_file: arm_right.yaml

    robots:                                   # the list form
      - arm_left.yaml                         # a bare path: node name from the file stem
      - name: arm_right
        params_file: arm_right.yaml

A robot's ``params_file`` defaults to ``bridge.example.yaml`` next to the fleet
file when it is omitted, so a fleet of fake arms needs only ``parameters``
overrides. Every node is respawned after an exit, two seconds later, at most
three times (see ``bridge.launch.py``), and the two secret environment
variables are passed through from the launching shell.
"""

# ruff reads the package's own launch/ directory as the first-party module "launch",
# which is why the import order below is exempted from I001.
from pathlib import Path  # noqa: I001
from typing import Any

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE = 'ros2_ha_bridge'
DEFAULT_PARAMS_FILE = 'bridge.example.yaml'
SECRET_ENVIRONMENT = ('ROS2_HA_BRIDGE_MQTT_PASSWORD', 'ROS2_HA_BRIDGE_COMMAND_SECRET')
RESPAWN = {'respawn': True, 'respawn_delay': 2.0, 'respawn_max_retries': 3}


def secret_environment() -> dict:
    """The two secret variables, passed through unchanged (empty when unset)."""
    return {name: EnvironmentVariable(name, default_value='') for name in SECRET_ENVIRONMENT}


def read_fleet(fleet_path: Path) -> list[dict[str, Any]]:
    """Normalise the fleet file into ``[{name, params_file, parameters}, ...]``."""
    with fleet_path.open(encoding='utf-8') as handle:
        document = yaml.safe_load(handle)
    if not isinstance(document, dict) or 'robots' not in document:
        raise ValueError(f'{fleet_path}: expected a mapping with a robots key')
    robots = document['robots']
    entries: list[tuple[str | None, Any]] = []
    if isinstance(robots, dict):
        entries = [(str(name), item) for name, item in robots.items()]
    elif isinstance(robots, list):
        entries = [(None, item) for item in robots]
    else:
        raise TypeError(f'{fleet_path}: robots must be a mapping or a list')
    if not entries:
        raise ValueError(f'{fleet_path}: robots is empty')

    base = fleet_path.parent
    normalised: list[dict[str, Any]] = []
    for key, item in entries:
        if isinstance(item, str):
            item = {'params_file': item}
        if item is None:
            item = {}
        if not isinstance(item, dict):
            raise TypeError(f'{fleet_path}: each robot must be a mapping or a path')
        params_file = Path(str(item.get('params_file', DEFAULT_PARAMS_FILE)))
        if not params_file.is_absolute():
            params_file = base / params_file
        if not params_file.is_file():
            raise ValueError(f'{fleet_path}: params file {params_file} does not exist')
        name = item.get('name', key) or params_file.stem
        parameters = item.get('parameters', {}) or {}
        if not isinstance(parameters, dict):
            raise TypeError(f'{fleet_path}: parameters of {name} must be a mapping')
        normalised.append(
            {'name': str(name), 'params_file': str(params_file), 'parameters': dict(parameters)}
        )
    names = [entry['name'] for entry in normalised]
    if len(set(names)) != len(names):
        raise ValueError(f'{fleet_path}: node names must be unique, got {names}')
    return normalised


def launch_fleet(context: LaunchContext) -> list[Node]:
    fleet_path = Path(LaunchConfiguration('fleet_file').perform(context))
    log_level = LaunchConfiguration('log_level').perform(context)
    nodes: list[Node] = []
    for robot in read_fleet(fleet_path):
        parameters: list[Any] = [robot['params_file']]
        if robot['parameters']:
            parameters.append(robot['parameters'])
        nodes.append(
            Node(
                package=PACKAGE,
                executable='bridge_node',
                name=robot['name'],
                output='screen',
                emulate_tty=True,
                parameters=parameters,
                arguments=['--ros-args', '--log-level', log_level],
                additional_env=secret_environment(),
                **RESPAWN,
            )
        )
    return nodes


def generate_launch_description() -> LaunchDescription:
    fleet_file = DeclareLaunchArgument(
        'fleet_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare(PACKAGE), 'config', 'fleet.example.yaml']
        ),
        description='Fleet file naming one node per robot (see config/fleet.example.yaml).',
    )
    log_level = DeclareLaunchArgument(
        'log_level', default_value='info', description='ROS log level for every node.'
    )
    return LaunchDescription([fleet_file, log_level, OpaqueFunction(function=launch_fleet)])
