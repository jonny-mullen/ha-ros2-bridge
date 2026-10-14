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
        namespace: left                       # optional ROS namespace (default: none)
        respawn_max_retries: -1               # optional: respawn, respawn_delay,
      arm_right:                              #   respawn_max_retries per robot
        params_file: arm_right.yaml

    robots:                                   # the list form
      - arm_left.yaml                         # a bare path: node name from the file stem
      - name: arm_right
        params_file: arm_right.yaml

A robot's ``params_file`` defaults to ``bridge.example.yaml`` next to the fleet
file when it is omitted, so a fleet of fake arms needs only ``parameters``
overrides. Node names must be unique: the name is the ``~`` of
``mirror.ros_topic`` and ``watchdog.ros_topic``.

Launch arguments: ``fleet_file``, ``log_level`` (``info``, applied to every
node) and the respawn policy of every node, ``respawn`` (``true``),
``respawn_delay`` (``2.0``) and ``respawn_max_retries`` (``3``; ``-1`` means
without limit), which a robot entry may override key by key (see
``bridge.launch.py`` for what the policy is for). The two secret environment
variables are passed through from the launching shell to every node.
"""

# ruff reads the package's own launch/ directory as the first-party module "launch",
# which is why the import order below is exempted from I001.
import math  # noqa: I001
from pathlib import Path
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
#: Defaults of the three respawn launch arguments (as the strings launch hands over).
DEFAULT_RESPAWN = 'true'
DEFAULT_RESPAWN_DELAY = '2.0'
DEFAULT_RESPAWN_MAX_RETRIES = '3'
#: The per-robot keys that override the launch arguments of the same name.
RESPAWN_KEYS = ('respawn', 'respawn_delay', 'respawn_max_retries')
#: The spellings launch itself accepts for a boolean argument.
TRUE_WORDS = frozenset({'true', '1', 'yes', 'on'})
FALSE_WORDS = frozenset({'false', '0', 'no', 'off'})


def secret_environment() -> dict[str, Any]:
    """The two secret variables, passed through unchanged (empty when unset)."""
    return {name: EnvironmentVariable(name, default_value='') for name in SECRET_ENVIRONMENT}


def respawn_options(respawn: str, delay: str, max_retries: str) -> dict[str, Any]:
    """The ``Node`` keyword arguments for the three respawn values, validated.

    The same rules as ``bridge.launch.py``: a boolean word, a number of
    seconds of at least 0, an integer of at least -1 (unlimited).
    """
    word = str(respawn).strip().lower()
    if word in TRUE_WORDS:
        enabled = True
    elif word in FALSE_WORDS:
        enabled = False
    else:
        raise ValueError(f'respawn must be true or false, not {respawn!r}')
    try:
        delay_s = float(str(delay).strip())
    except ValueError:
        raise ValueError(f'respawn_delay must be a number of seconds, not {delay!r}') from None
    if not math.isfinite(delay_s) or delay_s < 0:
        raise ValueError(f'respawn_delay must be at least 0, not {delay!r}')
    try:
        retries = int(str(max_retries).strip())
    except ValueError:
        raise ValueError(
            f'respawn_max_retries must be an integer (-1 for unlimited), not {max_retries!r}'
        ) from None
    if retries < -1:
        raise ValueError(f'respawn_max_retries must be -1 or at least 0, not {max_retries!r}')
    return {'respawn': enabled, 'respawn_delay': delay_s, 'respawn_max_retries': retries}


def read_fleet(fleet_path: Path) -> list[dict[str, Any]]:
    """Normalise the fleet file into ``[{name, params_file, parameters, namespace, respawn}, ...]``.

    ``namespace`` is ``None`` or a non-empty string; ``respawn`` holds only the
    respawn keys the entry gave (as strings), to be merged over the launch
    arguments by :func:`launch_fleet`.
    """
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
        namespace = item.get('namespace')
        if namespace is not None and not isinstance(namespace, str):
            raise TypeError(f'{fleet_path}: namespace of {name} must be a string')
        namespace = (namespace or '').strip() or None
        respawn = {key: str(item[key]) for key in RESPAWN_KEYS if item.get(key) is not None}
        normalised.append(
            {
                'name': str(name),
                'params_file': str(params_file),
                'parameters': dict(parameters),
                'namespace': namespace,
                'respawn': respawn,
            }
        )
    names = [entry['name'] for entry in normalised]
    if len(set(names)) != len(names):
        raise ValueError(f'{fleet_path}: node names must be unique, got {names}')
    return normalised


def fleet_respawn_options(robot: dict[str, Any], defaults: dict[str, str]) -> dict[str, Any]:
    """The respawn ``Node`` arguments for one robot: its own keys over the launch arguments."""
    merged = {**defaults, **robot['respawn']}
    try:
        return respawn_options(
            merged['respawn'], merged['respawn_delay'], merged['respawn_max_retries']
        )
    except ValueError as error:
        raise ValueError(f'robot {robot["name"]}: {error}') from None


def launch_fleet(context: LaunchContext) -> list[Node]:
    fleet_path = Path(LaunchConfiguration('fleet_file').perform(context))
    log_level = LaunchConfiguration('log_level').perform(context)
    defaults = {key: LaunchConfiguration(key).perform(context) for key in RESPAWN_KEYS}
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
                namespace=robot['namespace'],
                output='screen',
                emulate_tty=True,
                parameters=parameters,
                arguments=['--ros-args', '--log-level', log_level],
                additional_env=secret_environment(),
                **fleet_respawn_options(robot, defaults),
            )
        )
    return nodes


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument(
            'fleet_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare(PACKAGE), 'config', 'fleet.example.yaml']
            ),
            description='Fleet file naming one node per robot (see config/fleet.example.yaml).',
        ),
        DeclareLaunchArgument(
            'log_level', default_value='info', description='ROS log level for every node.'
        ),
        DeclareLaunchArgument(
            'respawn',
            default_value=DEFAULT_RESPAWN,
            description='Start a node again after it exits (true or false); per-robot override.',
        ),
        DeclareLaunchArgument(
            'respawn_delay',
            default_value=DEFAULT_RESPAWN_DELAY,
            description='Seconds to wait before a respawn (at least 0); per-robot override.',
        ),
        DeclareLaunchArgument(
            'respawn_max_retries',
            default_value=DEFAULT_RESPAWN_MAX_RETRIES,
            description='Respawns before giving up; -1 means without limit; per-robot override.',
        ),
    ]
    return LaunchDescription([*arguments, OpaqueFunction(function=launch_fleet)])
