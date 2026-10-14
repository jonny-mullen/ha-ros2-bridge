# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Launch one bridge node from a parameter file.

    ros2 launch ros2_ha_bridge bridge.launch.py params_file:=/path/to/my_robot.yaml

Launch arguments (``docs/configuration.md``, Launch arguments):

``params_file``
    The parameters file; defaults to the package's ``config/bridge.example.yaml``
    (the fake executor, robot ``fake_arm_1``, broker ``localhost``).
``node_name`` (``bridge_node``)
    The node name. It is the ``~`` of ``mirror.ros_topic`` (``~/mqtt_in``) and
    ``watchdog.ros_topic`` (``~/ha_link``), so two nodes on one DDS domain need
    different names.
``namespace`` (empty)
    The ROS namespace the node runs in, under which ``~`` resolves too.
``log_level`` (``info``)
    The ROS log level of the node.
``respawn`` (``true``), ``respawn_delay`` (``2.0``), ``respawn_max_retries`` (``3``)
    Whether an exited node is started again, after how many seconds, and how
    many times (``-1`` means without limit, as launch_ros defines it). The
    limit exists so that a node that exits on purpose (session taken over by
    a duplicate robot id, no CONNACK within ``mqtt.connect_timeout_s``) does
    not flap for ever; a supervisor that counts restarts itself (systemd
    ``Restart=``) runs with ``respawn:=false``.

Secrets never travel as launch arguments or parameters here: the two
environment variables ``ROS2_HA_BRIDGE_MQTT_PASSWORD`` and
``ROS2_HA_BRIDGE_COMMAND_SECRET`` are passed through from the launching
shell, and the node reads them when the matching parameter is empty.
"""

# ruff reads the package's own launch/ directory as the first-party module "launch",
# which is why the import order below is exempted from I001.
import math  # noqa: I001
from typing import Any

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE = 'ros2_ha_bridge'
SECRET_ENVIRONMENT = ('ROS2_HA_BRIDGE_MQTT_PASSWORD', 'ROS2_HA_BRIDGE_COMMAND_SECRET')
#: Defaults of the three respawn launch arguments (as the strings launch hands over).
DEFAULT_RESPAWN = 'true'
DEFAULT_RESPAWN_DELAY = '2.0'
DEFAULT_RESPAWN_MAX_RETRIES = '3'
#: The spellings launch itself accepts for a boolean argument.
TRUE_WORDS = frozenset({'true', '1', 'yes', 'on'})
FALSE_WORDS = frozenset({'false', '0', 'no', 'off'})


def secret_environment() -> dict[str, Any]:
    """The two secret variables, passed through unchanged (empty when unset)."""
    return {name: EnvironmentVariable(name, default_value='') for name in SECRET_ENVIRONMENT}


def respawn_options(respawn: str, delay: str, max_retries: str) -> dict[str, Any]:
    """The ``Node`` keyword arguments for the three respawn arguments, validated.

    ``respawn`` is a boolean word, ``delay`` a number of seconds of at least 0
    and ``max_retries`` an integer of at least -1 (unlimited). A bad value
    raises ``ValueError`` naming the argument, which launch prints and stops on.
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


def launch_node(context: LaunchContext) -> list[Node]:
    """Build the node once the arguments are known (``respawn_delay`` must be a number)."""
    namespace = LaunchConfiguration('namespace').perform(context).strip()
    options = respawn_options(
        LaunchConfiguration('respawn').perform(context),
        LaunchConfiguration('respawn_delay').perform(context),
        LaunchConfiguration('respawn_max_retries').perform(context),
    )
    return [
        Node(
            package=PACKAGE,
            executable='bridge_node',
            name=LaunchConfiguration('node_name').perform(context),
            namespace=namespace or None,
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file').perform(context)],
            arguments=[
                '--ros-args',
                '--log-level',
                LaunchConfiguration('log_level').perform(context),
            ],
            additional_env=secret_environment(),
            **options,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare(PACKAGE), 'config', 'bridge.example.yaml']
            ),
            description='Parameter file for the node (see config/bridge.example.yaml).',
        ),
        DeclareLaunchArgument(
            'node_name',
            default_value='bridge_node',
            description='Name of the node; the ~ of mirror.ros_topic and watchdog.ros_topic.',
        ),
        DeclareLaunchArgument(
            'namespace', default_value='', description='ROS namespace of the node (empty: none).'
        ),
        DeclareLaunchArgument(
            'log_level', default_value='info', description='ROS log level for the node.'
        ),
        DeclareLaunchArgument(
            'respawn',
            default_value=DEFAULT_RESPAWN,
            description='Start the node again after it exits (true or false).',
        ),
        DeclareLaunchArgument(
            'respawn_delay',
            default_value=DEFAULT_RESPAWN_DELAY,
            description='Seconds to wait before a respawn (at least 0).',
        ),
        DeclareLaunchArgument(
            'respawn_max_retries',
            default_value=DEFAULT_RESPAWN_MAX_RETRIES,
            description='How many respawns before giving up; -1 means without limit.',
        ),
    ]
    return LaunchDescription([*arguments, OpaqueFunction(function=launch_node)])
