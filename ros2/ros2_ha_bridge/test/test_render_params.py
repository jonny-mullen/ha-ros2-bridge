# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Tests for ``tools/render_params.py``, the Docker rig's parameters renderer.

The renderer is driven as a module (``render(environ)``), never through a
container: the six named variables, the generic ``ROS2_HA_BRIDGE_PARAM_*``
form typed from the node's own parameter table, the refusals (an unknown
name, a wrong type, the two secrets), how overrides land in a base file, and
that what it renders is a configuration ``settings_from_mapping`` accepts.
The file is found by walking up from this test (the repository's ``tools/``
folder, or ``/ws/tools`` in the image); without it these tests skip.
"""

from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path
from types import ModuleType
from typing import Any

import pytest
import yaml

from ros2_ha_bridge.executors.moveit import SCALAR_PARAMETERS
from ros2_ha_bridge.parameters import PARAMETERS, settings_from_mapping

TOOLS_ENV = 'HA_ROS2_BRIDGE_TOOLS_DIR'


def find_renderer() -> Path | None:
    override = os.environ.get(TOOLS_ENV)
    if override:
        candidate = Path(override) / 'render_params.py'
        return candidate if candidate.is_file() else None
    for parent in Path(__file__).resolve().parents:
        candidate = parent / 'tools' / 'render_params.py'
        if candidate.is_file():
            return candidate
    return None


RENDERER = find_renderer()


@pytest.fixture(scope='module')
def rp() -> ModuleType:
    if RENDERER is None:
        pytest.skip(f'tools/render_params.py not found (set {TOOLS_ENV} to point at tools/)')
    spec = importlib.util.spec_from_file_location('render_params_under_test', RENDERER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    # Registered before execution: dataclasses resolve postponed annotations through
    # sys.modules[cls.__module__].
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def flatten(params: dict[str, Any], prefix: str = '') -> dict[str, Any]:
    """The dotted view rcl builds from a ``ros__parameters`` mapping."""
    flat: dict[str, Any] = {}
    for key, value in params.items():
        name = f'{prefix}{key}'
        if isinstance(value, dict):
            flat.update(flatten(value, name + '.'))
        else:
            flat[name] = value
    return flat


def params(result: Any, entry: str = '/**') -> dict[str, Any]:
    section: dict[str, Any] = result.document[entry]['ros__parameters']
    return section


# --------------------------------------------------------------------------- names


def test_variable_names_round_trip_for_every_declared_parameter(rp: ModuleType) -> None:
    names = [spec.name for spec in PARAMETERS] + [name for name, _, _ in SCALAR_PARAMETERS]
    for name in names:
        variable = rp.variable_name(name)
        assert variable.startswith('ROS2_HA_BRIDGE_PARAM_') and variable == variable.upper()
        assert '.' not in variable and rp.parameter_name(variable) == name
    assert rp.variable_name('mqtt.tls.enabled') == 'ROS2_HA_BRIDGE_PARAM_MQTT__TLS__ENABLED'
    assert rp.parameter_name('ROS2_HA_BRIDGE_PARAM_WATCHDOG__MODE') == 'watchdog.mode'
    with pytest.raises(ValueError):
        rp.parameter_name('MQTT_HOST')


def test_kinds_cover_the_node_table_and_the_moveit_scalars(rp: ModuleType) -> None:
    kinds = rp.parameter_kinds()
    for spec in PARAMETERS:
        assert kinds[spec.name] == spec.kind, spec.name
    assert kinds['moveit.group_name'] == 'string'
    assert kinds['moveit.planning_time_s'] == 'double'
    assert kinds['moveit.num_attempts'] == 'integer'
    assert kinds['moveit.hold_on_cancel'] == 'bool'
    assert kinds['moveit.joint_names'] == 'string_array'
    assert kinds['moveit.joint_tolerance_rad'] == 'double'


# --------------------------------------------------------------------------- rendering


def test_no_variables_render_an_empty_document(rp: ModuleType) -> None:
    result = rp.render({})
    assert result.document == {'/**': {'ros__parameters': {}}} and result.overrides == ()


def test_the_named_variables_still_work_and_empty_means_no_override(rp: ModuleType) -> None:
    result = rp.render(
        {
            'ROS2_HA_BRIDGE_ROBOT_ID': 'fake_arm_1',
            'ROS2_HA_BRIDGE_ROBOT_NAME': 'Fake arm 1',
            'MQTT_HOST': 'local-broker',
            'MQTT_PORT': '1884',
            'MQTT_USERNAME': '',
            'TOPIC_PREFIX': 'lab',
        }
    )
    assert params(result) == {
        'robot': {'id': 'fake_arm_1', 'name': 'Fake arm 1'},
        'mqtt': {'host': 'local-broker', 'port': 1884},
        'topic_prefix': 'lab',
    }
    assert [(o.variable, o.name) for o in result.overrides] == [
        ('ROS2_HA_BRIDGE_ROBOT_ID', 'robot.id'),
        ('ROS2_HA_BRIDGE_ROBOT_NAME', 'robot.name'),
        ('MQTT_HOST', 'mqtt.host'),
        ('MQTT_PORT', 'mqtt.port'),
        ('TOPIC_PREFIX', 'topic_prefix'),
    ]
    assert isinstance(params(result)['mqtt']['port'], int)
    with pytest.raises(rp.RenderError, match='MQTT_PORT'):
        rp.render({'MQTT_PORT': 'eighteen-eighty-three'})
    with pytest.raises(rp.RenderError, match='MQTT_PORT'):
        rp.render({'MQTT_PORT': '1883.0'})


def test_generic_variables_are_typed_from_the_parameter_table(rp: ModuleType) -> None:
    result = rp.render(
        {
            'ROS2_HA_BRIDGE_PARAM_WATCHDOG__MODE': 'inhibit_new',
            'ROS2_HA_BRIDGE_PARAM_WATCHDOG__TIMEOUT_S': '8',
            'ROS2_HA_BRIDGE_PARAM_MQTT__TLS__ENABLED': 'true',
            'ROS2_HA_BRIDGE_PARAM_MQTT__TLS__CA_CERT': '/certs/ca.pem',
            'ROS2_HA_BRIDGE_PARAM_MQTT__KEEPALIVE_S': '30',
            'ROS2_HA_BRIDGE_PARAM_TELEMETRY__JOINTS_RATE_HZ': '5',
            'ROS2_HA_BRIDGE_PARAM_TASKS__ALLOW': 'reset, stop,,wait',
            'ROS2_HA_BRIDGE_PARAM_MIRROR__TOPICS': '[zigbee2mqtt/+, ros2_bridge/ha/sensors/#]',
            'ROS2_HA_BRIDGE_PARAM_ROBOT__NAME': '1.5',
            'ROS2_HA_BRIDGE_PARAM_DATA_DIR': '',
            'ROS2_HA_BRIDGE_PARAM_MOVEIT__PLANNING_TIME_S': '3',
            'ROS2_HA_BRIDGE_ROBOT_ID': 'arm_left',
        }
    )
    flat = flatten(params(result))
    assert flat == {
        'robot.id': 'arm_left',
        'mirror.topics': ['zigbee2mqtt/+', 'ros2_bridge/ha/sensors/#'],
        'moveit.planning_time_s': 3.0,
        'mqtt.keepalive_s': 30,
        'mqtt.tls.ca_cert': '/certs/ca.pem',
        'mqtt.tls.enabled': True,
        'robot.name': '1.5',
        'tasks.allow': ['reset', 'stop', 'wait'],
        'telemetry.joints_rate_hz': 5.0,
        'watchdog.mode': 'inhibit_new',
        'watchdog.timeout_s': 8.0,
    }
    assert isinstance(flat['mqtt.keepalive_s'], int) and isinstance(
        flat['watchdog.timeout_s'], float
    )
    assert isinstance(flat['robot.name'], str), 'a string parameter takes the text as it is'
    assert 'data_dir' not in flat, 'an empty variable is no override'
    # The named variable comes first; generic ones follow in variable order.
    assert next(o.variable for o in result.overrides) == 'ROS2_HA_BRIDGE_ROBOT_ID'
    assert [o.variable for o in result.overrides][1:] == sorted(
        o.variable for o in result.overrides[1:]
    )
    # What was rendered is a configuration the node accepts, with the values typed right.
    settings = settings_from_mapping(flat, environ={})
    assert settings.watchdog.mode == 'inhibit_new' and settings.watchdog.timeout_s == 8.0
    assert settings.mqtt.tls.enabled and settings.mqtt.keepalive_s == 30
    assert settings.executor.tasks_allow == ('reset', 'stop', 'wait')
    assert settings.mirror.topics == ('zigbee2mqtt/+', 'ros2_bridge/ha/sensors/#')
    assert settings.robot.name == '1.5'


@pytest.mark.parametrize(
    ('variable', 'value', 'fragment'),
    [
        (
            'ROS2_HA_BRIDGE_PARAM_MQTT__PASSWORD',
            'placeholder-password-value',
            'ROS2_HA_BRIDGE_MQTT_PASSWORD',
        ),
        (
            'ROS2_HA_BRIDGE_PARAM_COMMAND__SECRET',
            'placeholder-secret-value',
            'ROS2_HA_BRIDGE_COMMAND_SECRET',
        ),
        ('ROS2_HA_BRIDGE_PARAM_WATCHDOG__MOOD', 'calm', 'names no parameter'),
        ('ROS2_HA_BRIDGE_PARAM_MQTT_HOST', 'x', 'names no parameter'),
        ('ROS2_HA_BRIDGE_PARAM_MQTT__PORT', 'abc', 'needs an integer'),
        ('ROS2_HA_BRIDGE_PARAM_MQTT__PORT', '1.5', 'needs an integer'),
        ('ROS2_HA_BRIDGE_PARAM_MQTT__PORT', 'true', 'needs an integer'),
        ('ROS2_HA_BRIDGE_PARAM_MQTT__TLS__ENABLED', 'maybe', 'needs true or false'),
        ('ROS2_HA_BRIDGE_PARAM_MQTT__TLS__ENABLED', '1', 'needs true or false'),
        ('ROS2_HA_BRIDGE_PARAM_HEARTBEAT__INTERVAL_S', 'fast', 'needs a number'),
        ('ROS2_HA_BRIDGE_PARAM_HEARTBEAT__INTERVAL_S', '.inf', 'finite'),
        ('ROS2_HA_BRIDGE_PARAM_TASKS__ALLOW', '[reset, 3]', 'list of strings'),
        ('ROS2_HA_BRIDGE_PARAM_TASKS__ALLOW', '[reset', 'not valid YAML'),
    ],
)
def test_refusals_name_the_variable(
    rp: ModuleType, variable: str, value: str, fragment: str
) -> None:
    with pytest.raises(rp.RenderError) as excinfo:
        rp.render({variable: value})
    message = str(excinfo.value)
    assert variable in message and fragment in message, message
    if 'PASSWORD' in variable or 'SECRET' in variable:
        assert value not in message, 'a refused secret is never echoed'


def test_overrides_land_in_the_base_file_and_replace_every_spelling(
    rp: ModuleType, tmp_path: Path
) -> None:
    base = tmp_path / 'base.yaml'
    base.write_text(
        yaml.safe_dump(
            {
                '/arm_left': {
                    'ros__parameters': {
                        'robot': {'id': 'arm_left', 'name': 'Left arm'},
                        'mqtt': {'host': 'old-broker', 'tls.enabled': False},
                        'mqtt.port': 1883,
                        'executor': 'fake',
                        'executor.call_warn_s': 0.1,
                        'watchdog': {'mode': 'none', 'timeout_s': 5.0},
                    }
                }
            }
        ),
        encoding='utf-8',
    )
    result = rp.render(
        {
            'ROS2_HA_BRIDGE_PARAMS_FILE': str(base),
            'MQTT_HOST': 'new-broker',
            'MQTT_PORT': '8883',
            'ROS2_HA_BRIDGE_PARAM_MQTT__TLS__ENABLED': 'true',
            'ROS2_HA_BRIDGE_PARAM_EXECUTOR__CALL_WARN_S': '0.5',
            'ROS2_HA_BRIDGE_PARAM_WATCHDOG__MODE': 'stop_and_hold',
        }
    )
    assert list(result.document) == ['/arm_left'], 'the single node entry, not /**'
    section = params(result, '/arm_left')
    assert section['robot'] == {'id': 'arm_left', 'name': 'Left arm'}, 'untouched'
    assert section['mqtt'] == {'host': 'new-broker', 'port': 8883, 'tls': {'enabled': True}}
    assert 'mqtt.port' not in section and 'tls.enabled' not in section['mqtt']
    assert section['executor'] == 'fake', 'a value on the path is kept'
    assert section['executor.call_warn_s'] == 0.5, 'written as a dotted key beside it'
    assert section['watchdog'] == {'mode': 'stop_and_hold', 'timeout_s': 5.0}
    flat = flatten(section)
    assert (
        len(flat) == len(set(flat)) and settings_from_mapping(flat, environ={}).mqtt.port == 8883
    )


def test_set_parameter_handles_the_awkward_shapes(rp: ModuleType) -> None:
    declared = rp.declared_names()
    assert {'executor', 'executor.call_warn_s', 'moveit.poses'} <= declared
    section: dict[str, Any] = {
        'executor': {'call_warn_s': 0.1},
        'moveit': {'poses': {'home': {'j': 1}}},
    }
    rp.set_parameter(section, 'executor', 'moveit', declared)
    assert section['executor'] == 'moveit', 'a scalar replaces the nested map that sat there'
    assert section['executor.call_warn_s'] == 0.1, 'a declared parameter beneath it survives'
    rp.set_parameter(section, 'moveit.poses', '/etc/poses.yaml', declared)
    assert section['moveit'] == {'poses': '/etc/poses.yaml'}, (
        'a path replaces inline poses: nothing beneath moveit.poses is a declared parameter'
    )
    rp.set_parameter(section, 'a.b.c', 1, declared)
    rp.set_parameter(section, 'a.b', 2, declared)
    assert section['a'] == {'b': 2}, 'an undeclared subtree goes with the name'
    # Without a table nothing beneath the name is kept (the named variables alone).
    bare: dict[str, Any] = {'executor': {'call_warn_s': 0.1}}
    rp.set_parameter(bare, 'executor', 'moveit')
    assert bare == {'executor': 'moveit'}


@pytest.mark.parametrize(
    'spelling',
    [
        pytest.param(
            {'executor': 'fake', 'executor.call_warn_s': 0.2, 'executor.cancel_grace_s': 7.0},
            id='dotted',
        ),
        pytest.param({'executor': {'call_warn_s': 0.2, 'cancel_grace_s': 7.0}}, id='nested'),
    ],
)
def test_executor_override_keeps_the_declared_parameters_beneath_it(
    rp: ModuleType, tmp_path: Path, spelling: dict[str, Any]
) -> None:
    """``ROS2_HA_BRIDGE_PARAM_EXECUTOR`` replaces ``executor`` and nothing else.

    rcl spells ``executor.call_warn_s`` and ``executor.cancel_grace_s`` as children of
    the string parameter ``executor``, in either the dotted or the nested form; an
    adopter's tuned cancel grace must not revert to the default because the executor
    was switched from the environment.
    """
    base = tmp_path / 'base.yaml'
    base.write_text(
        yaml.safe_dump({'/**': {'ros__parameters': {'robot': {'id': 'arm'}, **spelling}}}),
        encoding='utf-8',
    )
    result = rp.render(
        {'ROS2_HA_BRIDGE_PARAMS_FILE': str(base), 'ROS2_HA_BRIDGE_PARAM_EXECUTOR': 'moveit'}
    )
    flat = flatten(params(result))
    assert flat == {
        'robot.id': 'arm',
        'executor': 'moveit',
        'executor.call_warn_s': 0.2,
        'executor.cancel_grace_s': 7.0,
    }
    section = params(result)
    assert section['executor'] == 'moveit' and 'executor.call_warn_s' in section, (
        'the survivors are written as dotted keys beside the scalar'
    )
    settings = settings_from_mapping(flat, environ={})
    assert settings.executor.cancel_grace_s == 7.0 and settings.executor.call_warn_s == 0.2


def test_base_file_problems_are_reported(rp: ModuleType, tmp_path: Path) -> None:
    with pytest.raises(rp.RenderError, match='cannot read'):
        rp.render({'ROS2_HA_BRIDGE_PARAMS_FILE': str(tmp_path / 'absent.yaml')})
    not_a_mapping = tmp_path / 'list.yaml'
    not_a_mapping.write_text('- a\n- b\n', encoding='utf-8')
    with pytest.raises(rp.RenderError, match='must be a mapping'):
        rp.render({'ROS2_HA_BRIDGE_PARAMS_FILE': str(not_a_mapping)})
    broken = tmp_path / 'broken.yaml'
    broken.write_text('a: {b\n', encoding='utf-8')
    with pytest.raises(rp.RenderError, match='not valid YAML'):
        rp.render({'ROS2_HA_BRIDGE_PARAMS_FILE': str(broken)})
    empty = tmp_path / 'empty.yaml'
    empty.write_text('', encoding='utf-8')
    result = rp.render({'ROS2_HA_BRIDGE_PARAMS_FILE': str(empty)})
    assert result.document == {'/**': {'ros__parameters': {}}}
    two_nodes = tmp_path / 'two.yaml'
    two_nodes.write_text(
        '/a:\n  ros__parameters: {}\n/b:\n  ros__parameters: {}\n', encoding='utf-8'
    )
    result = rp.render({'ROS2_HA_BRIDGE_PARAMS_FILE': str(two_nodes), 'TOPIC_PREFIX': 'lab'})
    assert result.document['/**'] == {'ros__parameters': {'topic_prefix': 'lab'}}


def test_main_writes_the_file_and_reports(
    rp: ModuleType,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    out = tmp_path / 'run' / 'params.yaml'
    for variable in list(os.environ):
        if variable.startswith('ROS2_HA_BRIDGE_PARAM_') or variable in rp.NAMED_VARIABLES:
            monkeypatch.delenv(variable)
    monkeypatch.delenv('ROS2_HA_BRIDGE_PARAMS_FILE', raising=False)
    monkeypatch.setenv('ROS2_HA_BRIDGE_ROBOT_ID', 'fake_arm_2')
    monkeypatch.setenv('ROS2_HA_BRIDGE_PARAM_FAKE__TASK_DURATION_S', '0.5')
    assert rp.main([str(out)]) == 0
    loaded = yaml.safe_load(out.read_text(encoding='utf-8'))
    assert loaded == {
        '/**': {
            'ros__parameters': {'robot': {'id': 'fake_arm_2'}, 'fake': {'task_duration_s': 0.5}}
        }
    }
    err = capsys.readouterr().err
    assert (
        'rendered' in err and "robot.id='fake_arm_2'" in err and 'fake.task_duration_s=0.5' in err
    )

    monkeypatch.setenv('ROS2_HA_BRIDGE_PARAM_MQTT__PASSWORD', 'placeholder-secret')
    assert rp.main([str(out)]) == 1
    err = capsys.readouterr().err
    assert 'ROS2_HA_BRIDGE_PARAM_MQTT__PASSWORD' in err and 'placeholder-secret' not in err
    assert rp.main([]) == 2
