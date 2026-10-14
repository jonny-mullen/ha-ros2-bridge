# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Tests for the two launch files' own logic: arguments, respawn policy, fleet file, namespace.

``launch`` and ``launch_ros`` are not installed on a machine without ROS 2,
so when they cannot be imported the few names the launch files use are
stubbed (a stub records what it was constructed with), which is enough to
check ``respawn_options``, ``read_fleet`` and the keyword arguments the
``Node`` actions receive. With the real packages (``colcon test`` in the
Jazzy container) the same tests run against them, and the launch description
is additionally generated for real.
"""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path
from types import ModuleType
from typing import Any

import pytest
import yaml

import ros2_ha_bridge

LAUNCH_DIR = Path(ros2_ha_bridge.__file__).resolve().parent.parent / 'launch'
CONFIG_DIR = Path(ros2_ha_bridge.__file__).resolve().parent.parent / 'config'


def _share_dir(sub: str) -> Path | None:
    """``launch/`` or ``config/`` of the source checkout, else of the installed package."""
    source = Path(ros2_ha_bridge.__file__).resolve().parent.parent / sub
    if source.is_dir():
        return source
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return None
    installed = Path(get_package_share_directory('ros2_ha_bridge')) / sub
    return installed if installed.is_dir() else None


class Recorded:
    """A stand-in for a launch class: remembers its arguments."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        self.args = args
        self.kwargs = kwargs


class StubLaunchConfiguration(Recorded):
    def perform(self, context: Any) -> str:
        value: str = context.launch_configurations[self.args[0]]
        return value


class StubContext:
    def __init__(self, **values: str) -> None:
        self.launch_configurations = dict(values)


def _stub_module(name: str, **attrs: Any) -> ModuleType:
    module = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(module, key, value)
    return module


@pytest.fixture(scope='module')
def real_launch() -> bool:
    try:
        import launch_ros  # noqa: F401

        import launch  # noqa: F401
    except ImportError:
        return False
    return True


@pytest.fixture
def launch_modules(real_launch: bool, monkeypatch: pytest.MonkeyPatch) -> dict[str, Any]:
    """The names the launch files import, real or stubbed; returns the ``Node`` class."""
    if real_launch:
        from launch.launch_context import LaunchContext
        from launch_ros.actions import Node

        return {'Node': Node, 'context': lambda **values: _real_context(LaunchContext, values)}

    class Node(Recorded):
        pass

    class DeclareLaunchArgument(Recorded):
        pass

    class OpaqueFunction(Recorded):
        pass

    class LaunchDescription(Recorded):
        pass

    stubs = {
        'launch': _stub_module('launch', LaunchDescription=LaunchDescription),
        'launch.actions': _stub_module(
            'launch.actions',
            DeclareLaunchArgument=DeclareLaunchArgument,
            OpaqueFunction=OpaqueFunction,
        ),
        'launch.launch_context': _stub_module('launch.launch_context', LaunchContext=StubContext),
        'launch.substitutions': _stub_module(
            'launch.substitutions',
            EnvironmentVariable=Recorded,
            LaunchConfiguration=StubLaunchConfiguration,
            PathJoinSubstitution=Recorded,
        ),
        'launch_ros': _stub_module('launch_ros'),
        'launch_ros.actions': _stub_module('launch_ros.actions', Node=Node),
        'launch_ros.substitutions': _stub_module(
            'launch_ros.substitutions', FindPackageShare=Recorded
        ),
    }
    for name, module in stubs.items():
        monkeypatch.setitem(sys.modules, name, module)
    return {
        'Node': Node,
        'DeclareLaunchArgument': DeclareLaunchArgument,
        'context': lambda **values: StubContext(**values),
    }


def _real_context(context_cls: Any, values: dict[str, str]) -> Any:
    context = context_cls()
    context.launch_configurations.update(values)
    return context


def load_launch_file(name: str) -> ModuleType:
    launch_dir = _share_dir('launch')
    if launch_dir is None:
        pytest.skip('launch files not found')
    path = launch_dir / name
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def bridge_launch(launch_modules: dict[str, Any]) -> ModuleType:
    return load_launch_file('bridge.launch.py')


@pytest.fixture
def fleet_launch(launch_modules: dict[str, Any]) -> ModuleType:
    return load_launch_file('fleet.launch.py')


def bridge_defaults(**overrides: str) -> dict[str, str]:
    values = {
        'params_file': '/etc/ros2_ha_bridge/arm_left.yaml',
        'node_name': 'bridge_node',
        'namespace': '',
        'log_level': 'info',
        'respawn': 'true',
        'respawn_delay': '2.0',
        'respawn_max_retries': '3',
    }
    values.update(overrides)
    return values


# --------------------------------------------------------------------------- respawn options


@pytest.mark.parametrize('name', ['bridge.launch.py', 'fleet.launch.py'])
def test_respawn_options_parse_and_validate(launch_modules: dict[str, Any], name: str) -> None:
    module = load_launch_file(name)
    options = module.respawn_options
    assert options('true', '2.0', '3') == {
        'respawn': True,
        'respawn_delay': 2.0,
        'respawn_max_retries': 3,
    }
    assert options(
        module.DEFAULT_RESPAWN, module.DEFAULT_RESPAWN_DELAY, module.DEFAULT_RESPAWN_MAX_RETRIES
    ) == {
        'respawn': True,
        'respawn_delay': 2.0,
        'respawn_max_retries': 3,
    }, 'the launch argument defaults are the policy the docs state'
    assert options('False', '0', '-1') == {
        'respawn': False,
        'respawn_delay': 0.0,
        'respawn_max_retries': -1,
    }
    assert options(' yes ', ' 0.5 ', ' 0 ')['respawn'] is True
    assert options('off', '10', '7') == {
        'respawn': False,
        'respawn_delay': 10.0,
        'respawn_max_retries': 7,
    }
    for bad, fragment in (
        (('maybe', '2.0', '3'), 'respawn must be true or false'),
        (('true', 'soon', '3'), 'respawn_delay'),
        (('true', '-1', '3'), 'respawn_delay must be at least 0'),
        (('true', 'inf', '3'), 'respawn_delay'),
        (('true', '2.0', '2.5'), 'respawn_max_retries'),
        (('true', '2.0', '-2'), 'respawn_max_retries must be -1'),
    ):
        with pytest.raises(ValueError, match=fragment):
            options(*bad)


# --------------------------------------------------------------------------- bridge.launch.py


def test_bridge_launch_declares_the_documented_arguments(
    bridge_launch: ModuleType, launch_modules: dict[str, Any], real_launch: bool
) -> None:
    description = bridge_launch.generate_launch_description()
    if real_launch:
        from launch.actions import DeclareLaunchArgument

        entities = description.entities
        names = [e.name for e in entities if isinstance(e, DeclareLaunchArgument)]
    else:
        entities = description.args[0]
        names = [
            e.args[0] for e in entities if isinstance(e, launch_modules['DeclareLaunchArgument'])
        ]
    assert names == [
        'params_file',
        'node_name',
        'namespace',
        'log_level',
        'respawn',
        'respawn_delay',
        'respawn_max_retries',
    ]


def test_bridge_launch_node_follows_the_arguments(
    bridge_launch: ModuleType, launch_modules: dict[str, Any], real_launch: bool
) -> None:
    make_context = launch_modules['context']
    nodes = bridge_launch.launch_node(make_context(**bridge_defaults()))
    assert len(nodes) == 1 and isinstance(nodes[0], launch_modules['Node'])
    custom = bridge_launch.launch_node(
        make_context(
            **bridge_defaults(
                node_name='left_bridge',
                namespace=' /robots/left ',
                log_level='debug',
                respawn='false',
                respawn_delay='0.5',
                respawn_max_retries='-1',
            )
        )
    )
    assert len(custom) == 1
    if real_launch:
        return
    kwargs = nodes[0].kwargs
    assert kwargs['name'] == 'bridge_node' and kwargs['namespace'] is None
    assert kwargs['parameters'] == ['/etc/ros2_ha_bridge/arm_left.yaml']
    assert kwargs['arguments'] == ['--ros-args', '--log-level', 'info']
    assert (kwargs['respawn'], kwargs['respawn_delay'], kwargs['respawn_max_retries']) == (
        True,
        2.0,
        3,
    )
    assert set(kwargs['additional_env']) == set(bridge_launch.SECRET_ENVIRONMENT)
    kwargs = custom[0].kwargs
    assert kwargs['name'] == 'left_bridge' and kwargs['namespace'] == '/robots/left'
    assert kwargs['arguments'][-1] == 'debug'
    assert (kwargs['respawn'], kwargs['respawn_delay'], kwargs['respawn_max_retries']) == (
        False,
        0.5,
        -1,
    )
    with pytest.raises(ValueError, match='respawn_max_retries'):
        bridge_launch.launch_node(make_context(**bridge_defaults(respawn_max_retries='-5')))


# --------------------------------------------------------------------------- fleet.launch.py


def test_fleet_example_file_reads_as_two_fake_arms(fleet_launch: ModuleType) -> None:
    config_dir = _share_dir('config')
    assert config_dir is not None
    robots = fleet_launch.read_fleet(config_dir / 'fleet.example.yaml')
    assert [r['name'] for r in robots] == ['fake_arm_1', 'fake_arm_2']
    assert all(Path(r['params_file']).name == 'bridge.example.yaml' for r in robots)
    assert robots[0]['parameters'] == {'robot.id': 'fake_arm_1', 'robot.name': 'Fake arm 1'}
    assert all(r['namespace'] is None and r['respawn'] == {} for r in robots)


def test_fleet_file_accepts_namespace_and_respawn_keys(
    fleet_launch: ModuleType, tmp_path: Path
) -> None:
    (tmp_path / 'bridge.example.yaml').write_text(
        '/**:\n  ros__parameters: {}\n', encoding='utf-8'
    )
    (tmp_path / 'arm_right.yaml').write_text('/**:\n  ros__parameters: {}\n', encoding='utf-8')
    fleet = tmp_path / 'fleet.yaml'
    fleet.write_text(
        yaml.safe_dump(
            {
                'robots': {
                    'arm_left': {
                        'parameters': {'robot.id': 'arm_left'},
                        'namespace': ' left ',
                        'respawn_max_retries': -1,
                    },
                    'arm_right': {
                        'params_file': 'arm_right.yaml',
                        'namespace': '',
                        'respawn': False,
                        'respawn_delay': 0.5,
                    },
                }
            }
        ),
        encoding='utf-8',
    )
    left, right = fleet_launch.read_fleet(fleet)
    assert left['namespace'] == 'left' and left['respawn'] == {'respawn_max_retries': '-1'}
    assert left['params_file'] == str(tmp_path / 'bridge.example.yaml'), 'the default beside it'
    assert right['namespace'] is None, 'empty means none'
    assert right['respawn'] == {'respawn': 'False', 'respawn_delay': '0.5'}
    defaults = {'respawn': 'true', 'respawn_delay': '2.0', 'respawn_max_retries': '3'}
    assert fleet_launch.fleet_respawn_options(left, defaults) == {
        'respawn': True,
        'respawn_delay': 2.0,
        'respawn_max_retries': -1,
    }
    assert fleet_launch.fleet_respawn_options(right, defaults) == {
        'respawn': False,
        'respawn_delay': 0.5,
        'respawn_max_retries': 3,
    }
    bad = dict(left, respawn={'respawn_delay': 'soon'})
    with pytest.raises(ValueError, match='robot arm_left: respawn_delay'):
        fleet_launch.fleet_respawn_options(bad, defaults)

    # The list form, with a bare path (node name from the stem) and a mapping item.
    fleet.write_text(
        yaml.safe_dump({'robots': ['arm_right.yaml', {'name': 'spare', 'namespace': 'b'}]}),
        encoding='utf-8',
    )
    stem, spare = fleet_launch.read_fleet(fleet)
    assert stem['name'] == 'arm_right' and spare['name'] == 'spare' and spare['namespace'] == 'b'

    for document, fragment in (
        ({'robots': {'a': {'namespace': 7}}}, 'namespace'),
        ({'robots': {'a': {}, 'b': {'name': 'a'}}}, 'unique'),
        ({'robots': {'a': {'params_file': 'missing.yaml'}}}, 'does not exist'),
        ({'robots': {'a': {'parameters': [1]}}}, 'parameters'),
        ({'robots': []}, 'empty'),
        ({'robots': 'arm'}, 'mapping or a list'),
        ({'fleet': {}}, 'robots key'),
    ):
        fleet.write_text(yaml.safe_dump(document), encoding='utf-8')
        with pytest.raises((ValueError, TypeError), match=fragment):
            fleet_launch.read_fleet(fleet)


def test_fleet_launch_builds_one_node_per_robot(
    fleet_launch: ModuleType, launch_modules: dict[str, Any], real_launch: bool, tmp_path: Path
) -> None:
    (tmp_path / 'bridge.example.yaml').write_text(
        '/**:\n  ros__parameters: {}\n', encoding='utf-8'
    )
    fleet = tmp_path / 'fleet.yaml'
    fleet.write_text(
        yaml.safe_dump(
            {
                'robots': {
                    'arm_left': {'parameters': {'robot.id': 'arm_left'}, 'namespace': 'left'},
                    'arm_right': {'respawn': False},
                }
            }
        ),
        encoding='utf-8',
    )
    context = launch_modules['context'](
        fleet_file=str(fleet),
        log_level='warn',
        respawn='true',
        respawn_delay='1.5',
        respawn_max_retries='4',
    )
    nodes = fleet_launch.launch_fleet(context)
    assert len(nodes) == 2 and all(isinstance(n, launch_modules['Node']) for n in nodes)
    description = fleet_launch.generate_launch_description()
    assert description is not None
    if real_launch:
        return
    left, right = (node.kwargs for node in nodes)
    assert left['name'] == 'arm_left' and left['namespace'] == 'left'
    assert left['parameters'] == [str(tmp_path / 'bridge.example.yaml'), {'robot.id': 'arm_left'}]
    assert left['arguments'] == ['--ros-args', '--log-level', 'warn']
    assert (left['respawn'], left['respawn_delay'], left['respawn_max_retries']) == (True, 1.5, 4)
    assert right['namespace'] is None and right['parameters'] == [
        str(tmp_path / 'bridge.example.yaml')
    ]
    assert (right['respawn'], right['respawn_delay'], right['respawn_max_retries']) == (
        False,
        1.5,
        4,
    ), 'a per-robot key overrides only itself'
    names = [
        e.args[0]
        for e in description.args[0]
        if isinstance(e, launch_modules['DeclareLaunchArgument'])
    ]
    assert names == ['fleet_file', 'log_level', 'respawn', 'respawn_delay', 'respawn_max_retries']
