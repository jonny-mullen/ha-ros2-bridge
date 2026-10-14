# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Tests for :mod:`ros2_ha_bridge.parameters`, the rclpy-free half of the node's configuration.

Settings are built from a flat mapping exactly as the node hands rclpy's
values over: the declaration table and the design's defaults, the two
environment fallbacks, the derived client id, every validation rule with a
plain message naming the parameter, the dedupe window rule, the start-up log
masking and the parameter view the executor plugin receives.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

import pytest

from ros2_ha_bridge import parameters
from ros2_ha_bridge.parameters import (
    ENV_COMMAND_SECRET,
    ENV_MQTT_PASSWORD,
    MASK,
    PARAMETER_NAMES,
    PARAMETERS,
    SECRET_PARAMETERS,
    BridgeSettings,
    ConfigError,
    default_values,
    derived_client_id,
    describe_settings,
    executor_parameters,
    format_settings,
    settings_from_mapping,
)

from .conftest import assert_imports_without

ROBOT = 'arm_left'


def build(environ: dict[str, str] | None = None, **values: Any) -> BridgeSettings:
    """Settings for ``arm_left`` with ``values`` keyed by parameter name (``robot_id`` shorthand)."""
    mapping: dict[str, Any] = {'robot.id': values.pop('robot_id', ROBOT)}
    mapping.update(values)
    return settings_from_mapping(mapping, environ={} if environ is None else environ)


def refused(**values: Any) -> str:
    """The ConfigError message for ``values``."""
    with pytest.raises(ConfigError) as excinfo:
        build(**values)
    return str(excinfo.value)


# --------------------------------------------------------------------------- hygiene


def test_module_imports_neither_rclpy_nor_paho() -> None:
    assert_imports_without('ros2_ha_bridge.parameters', {'rclpy', 'paho'})


def test_table_names_are_unique_and_match_the_settings_mapping() -> None:
    assert len(set(PARAMETER_NAMES)) == len(PARAMETER_NAMES)
    assert set(PARAMETER_NAMES) == set(default_values())
    settings = build()
    assert list(settings.as_mapping()) == list(PARAMETER_NAMES), 'same names, same order'
    assert SECRET_PARAMETERS <= set(PARAMETER_NAMES)
    for spec in PARAMETERS:
        assert spec.description and not spec.description.endswith(' ')
        assert spec.kind in ('string', 'integer', 'double', 'bool', 'string_array')
        if spec.kind == 'string_array':
            assert spec.default == () and spec.default_value == []
        elif spec.kind == 'double':
            assert isinstance(spec.default, float) and not isinstance(spec.default, bool)
        elif spec.kind == 'integer':
            assert isinstance(spec.default, int) and not isinstance(spec.default, bool)
        elif spec.kind == 'bool':
            assert isinstance(spec.default, bool)
        else:
            assert isinstance(spec.default, str)


# --------------------------------------------------------------------------- defaults


def test_defaults_match_the_design_table() -> None:
    settings = build()
    assert settings.robot.id == ROBOT and settings.robot.name == ROBOT
    assert settings.robot.manufacturer is None and settings.robot.suggested_area is None
    mqtt = settings.mqtt
    assert (mqtt.host, mqtt.port, mqtt.username, mqtt.password) == ('localhost', 1883, '', '')
    assert mqtt.keepalive_s == 15
    assert (mqtt.reconnect_min_s, mqtt.reconnect_max_s) == (1, 30)
    assert mqtt.connect_timeout_s == 0.0
    assert mqtt.takeover_exit_after == 3
    assert mqtt.max_payload_bytes == 16384
    assert mqtt.inbound_queue_max == 256
    assert not mqtt.tls.enabled and not mqtt.tls.insecure
    assert settings.topic_prefix == 'ros2_bridge'
    assert settings.ha_status_topic == 'homeassistant/status'
    assert settings.announce_jitter_s == 2.0
    executor = settings.executor
    assert executor.spec == 'fake'
    assert (executor.call_warn_s, executor.cancel_grace_s, executor.health_poll_s) == (
        0.1,
        5.0,
        1.0,
    )
    assert executor.tasks_allow == ()
    command = settings.command
    assert command.secret == '' and not command.hmac_required
    assert (command.max_age_cap_ms, command.clock_skew_tolerance_ms) == (30000, 2000)
    assert (command.dedupe_window_s, command.dedupe_size) == (300.0, 1024)
    assert command.task_timeout_cap_s == 600.0
    assert command.keep_on_load_ms == 32000
    assert settings.data_dir == Path('~/.ros/ros2_ha_bridge').expanduser()
    assert settings.dedupe_dir == Path('~/.ros/ros2_ha_bridge').expanduser() / ROBOT
    reply = settings.reply
    assert (reply.ack_expiry_s, reply.result_expiry_s) == (120, 300)
    assert reply.feedback_max_rate_hz == 2.0
    assert reply.retained_rejections_per_minute == 30
    assert settings.heartbeat_interval_s == 2.0
    telemetry = settings.telemetry
    assert (telemetry.joints_rate_hz, telemetry.joints_min_delta) == (2.0, 0.0)
    assert telemetry.joints_enabled
    assert telemetry.min_delta_for('deg') == 0.5 and telemetry.min_delta_for('rad') == 0.01
    assert (settings.watchdog.mode, settings.watchdog.timeout_s) == ('none', 5.0)
    mirror = settings.mirror
    assert (mirror.topics, mirror.availability_suffix, mirror.ros_topic) == (
        (),
        '/availability',
        '~/mqtt_in',
    )
    assert (settings.fake.task_duration_s, settings.fake.feedback_hz) == (2.0, 2.0)


def test_missing_names_take_the_default_and_unknown_names_are_ignored() -> None:
    settings = settings_from_mapping({'robot.id': ROBOT, 'not.a.parameter': 1}, environ={})
    assert settings.mqtt.port == 1883


def test_identity_strings_become_none_when_empty_and_name_falls_back_to_the_id() -> None:
    full = build(
        **{
            'robot.name': 'Left arm',
            'robot.manufacturer': 'Example Robotics',
            'robot.model': 'UR3e',
            'robot.sw_version': 'driver 2.4.1',
            'robot.hw_version': 'rev C',
            'robot.serial_number': 'SN-0001',
            'robot.suggested_area': 'Kitchen',
        }
    )
    assert full.robot.name == 'Left arm'
    assert full.robot.manufacturer == 'Example Robotics' and full.robot.hw_version == 'rev C'
    assert full.robot.serial_number == 'SN-0001' and full.robot.suggested_area == 'Kitchen'
    blank = build(**{'robot.name': '   '})
    assert blank.robot.name == ROBOT


# --------------------------------------------------------------------------- environment fallbacks


def test_empty_password_and_secret_read_the_environment() -> None:
    environ = {ENV_MQTT_PASSWORD: 'env-password', ENV_COMMAND_SECRET: 'env-secret'}
    settings = build(environ)
    assert settings.mqtt.password == 'env-password' and settings.mqtt.password_from_env
    assert settings.command.secret == 'env-secret' and settings.command.secret_from_env
    assert settings.command.hmac_required

    explicit = build(environ, **{'mqtt.password': 'param-password', 'command.secret': 'param-s'})
    assert explicit.mqtt.password == 'param-password' and not explicit.mqtt.password_from_env
    assert explicit.command.secret == 'param-s' and not explicit.command.secret_from_env

    nothing = build({ENV_MQTT_PASSWORD: '', ENV_COMMAND_SECRET: ''})
    assert nothing.mqtt.password == '' and not nothing.mqtt.password_from_env
    assert nothing.command.secret == '' and not nothing.command.hmac_required


def test_os_environ_is_the_default_source(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(ENV_MQTT_PASSWORD, 'from-os-environ')
    monkeypatch.setenv(ENV_COMMAND_SECRET, 'secret-from-os-environ')
    settings = settings_from_mapping({'robot.id': ROBOT})
    assert settings.mqtt.password == 'from-os-environ'
    assert settings.command.secret == 'secret-from-os-environ'


# --------------------------------------------------------------------------- derived values


def test_client_id_is_derived_from_the_robot_id_unless_overridden(
    caplog: pytest.LogCaptureFixture,
) -> None:
    assert derived_client_id('arm_left') == 'ros2_ha_bridge_arm_left'
    settings = build()
    assert settings.mqtt.client_id == 'ros2_ha_bridge_arm_left' and settings.mqtt.client_id_derived
    caplog.set_level(logging.WARNING, logger='ros2_ha_bridge.parameters')
    custom = build(**{'mqtt.client_id': ' custom-id '})
    assert custom.mqtt.client_id == 'custom-id' and not custom.mqtt.client_id_derived
    assert any('custom-id' in r.getMessage() for r in caplog.records)


def test_data_dir_is_expanded_and_empty_means_memory_only() -> None:
    settings = build(data_dir='/var/lib/example')
    assert settings.data_dir == Path('/var/lib/example')
    assert settings.dedupe_dir == Path('/var/lib/example') / ROBOT
    memory = build(data_dir='')
    assert memory.data_dir is None and memory.dedupe_dir is None
    assert memory.as_mapping()['data_dir'] == ''


def test_dedupe_window_is_raised_to_the_ack_expiry(caplog: pytest.LogCaptureFixture) -> None:
    caplog.set_level(logging.WARNING, logger='ros2_ha_bridge.parameters')
    settings = build(**{'command.dedupe_window_s': 10, 'reply.ack_expiry_s': 120})
    assert settings.command.dedupe_window_s == 120.0
    assert any('dedupe_window_s' in r.getMessage() for r in caplog.records)
    wide = build(**{'command.dedupe_window_s': 600, 'reply.ack_expiry_s': 120})
    assert wide.command.dedupe_window_s == 600.0


def test_lists_are_deduplicated_and_tasks_allow_is_checked() -> None:
    settings = build(
        **{'tasks.allow': ['reset', 'stop', 'reset'], 'mirror.topics': ['a/b', 'a/b']}
    )
    assert settings.executor.tasks_allow == ('reset', 'stop')
    assert settings.mirror.topics == ('a/b',)
    assert 'tasks.allow' in refused(**{'tasks.allow': ['Reset']})
    assert 'tasks.allow' in refused(**{'tasks.allow': 'reset'})
    assert 'tasks.allow' in refused(**{'tasks.allow': ['reset', 3]})


def test_integers_are_accepted_for_doubles_but_not_the_other_way_round() -> None:
    settings = build(**{'heartbeat.interval_s': 2, 'watchdog.timeout_s': 7})
    assert settings.heartbeat_interval_s == 2.0 and settings.watchdog.timeout_s == 7.0
    assert isinstance(settings.heartbeat_interval_s, float)
    assert 'mqtt.port must be an integer' in refused(**{'mqtt.port': 1883.0})
    assert 'mqtt.port must be an integer' in refused(**{'mqtt.port': True})
    assert 'mqtt.tls.enabled must be true or false' in refused(**{'mqtt.tls.enabled': 1})
    assert 'heartbeat.interval_s must be a number' in refused(**{'heartbeat.interval_s': '2'})
    assert 'heartbeat.interval_s must be a number' in refused(**{'heartbeat.interval_s': True})
    assert 'finite' in refused(**{'heartbeat.interval_s': float('nan')})
    assert 'mqtt.host must be a string' in refused(**{'mqtt.host': 1})


# --------------------------------------------------------------------------- validation


@pytest.mark.parametrize(
    ('values', 'fragment'),
    [
        ({'robot_id': ''}, 'robot.id is required'),
        ({'robot_id': '  '}, 'robot.id is required'),
        ({'robot_id': 'ha'}, 'reserved'),
        ({'robot_id': 'Arm'}, 'robot.id'),
        ({'robot_id': 'arm left'}, 'robot.id'),
        ({'robot_id': 'a' * 33}, 'robot.id'),
        ({'topic_prefix': 'homeassistant'}, 'topic_prefix'),
        ({'topic_prefix': 'a/+'}, 'topic_prefix'),
        ({'topic_prefix': '/x'}, 'topic_prefix'),
        ({'topic_prefix': 'ha/x'}, 'topic_prefix'),
        ({'ha_status_topic': ''}, 'ha_status_topic'),
        ({'ha_status_topic': 'a/#/b'}, 'ha_status_topic'),
        ({'mqtt.host': ''}, 'mqtt.host'),
        ({'mqtt.host': '   '}, 'mqtt.host'),
        ({'mqtt.port': 0}, 'mqtt.port'),
        ({'mqtt.port': 65536}, 'mqtt.port'),
        ({'mqtt.keepalive_s': -1}, 'mqtt.keepalive_s'),
        ({'mqtt.reconnect_min_s': 0}, 'mqtt.reconnect_min_s'),
        ({'mqtt.reconnect_min_s': 5, 'mqtt.reconnect_max_s': 4}, 'mqtt.reconnect_max_s'),
        ({'mqtt.connect_timeout_s': -0.5}, 'mqtt.connect_timeout_s'),
        ({'mqtt.takeover_exit_after': -1}, 'mqtt.takeover_exit_after'),
        ({'mqtt.max_payload_bytes': 0}, 'mqtt.max_payload_bytes'),
        ({'mqtt.inbound_queue_max': 0}, 'mqtt.inbound_queue_max'),
        ({'announce.jitter_s': -1}, 'announce.jitter_s'),
        ({'executor': ''}, 'executor'),
        ({'executor': '   '}, 'executor'),
        ({'executor.call_warn_s': -0.1}, 'executor.call_warn_s'),
        ({'executor.cancel_grace_s': 0}, 'executor.cancel_grace_s'),
        ({'executor.health_poll_s': 0}, 'executor.health_poll_s'),
        ({'command.max_age_cap_ms': 0}, 'command.max_age_cap_ms'),
        ({'command.clock_skew_tolerance_ms': -1}, 'command.clock_skew_tolerance_ms'),
        ({'command.dedupe_window_s': 0}, 'command.dedupe_window_s'),
        ({'command.dedupe_size': 0}, 'command.dedupe_size'),
        ({'command.task_timeout_cap_s': 0}, 'command.task_timeout_cap_s'),
        ({'reply.ack_expiry_s': -1}, 'reply.ack_expiry_s'),
        ({'reply.result_expiry_s': -1}, 'reply.result_expiry_s'),
        ({'reply.feedback_max_rate_hz': -1}, 'reply.feedback_max_rate_hz'),
        ({'reply.retained_rejections_per_minute': -1}, 'reply.retained_rejections_per_minute'),
        ({'heartbeat.interval_s': -1}, 'heartbeat.interval_s'),
        ({'telemetry.joints_rate_hz': -1}, 'telemetry.joints_rate_hz'),
        ({'telemetry.joints_rate_hz': 'fast'}, 'telemetry.joints_rate_hz'),
        ({'telemetry.joints_min_delta': -0.1}, 'telemetry.joints_min_delta'),
        ({'watchdog.mode': 'panic'}, 'watchdog.mode'),
        ({'watchdog.timeout_s': 0}, 'watchdog.timeout_s'),
        ({'mirror.topics': ['a/#/b']}, 'mirror'),
        ({'mirror.topics': ['']}, 'mirror'),
        ({'mirror.availability_suffix': 'availability'}, 'mirror'),
        ({'mirror.availability_suffix': '/+'}, 'mirror'),
        ({'mirror.ros_topic': ''}, 'mirror.ros_topic'),
        ({'fake.task_duration_s': 0}, 'fake.task_duration_s'),
        ({'fake.feedback_hz': -1}, 'fake.feedback_hz'),
    ],
)
def test_misconfiguration_is_refused_with_the_parameter_named(
    values: dict[str, Any], fragment: str
) -> None:
    message = refused(**values)
    assert fragment in message, message


def test_valid_edge_values_are_accepted() -> None:
    settings = build(
        **{
            'mqtt.port': 65535,
            'mqtt.keepalive_s': 0,
            'mqtt.reconnect_min_s': 30,
            'mqtt.reconnect_max_s': 30,
            'mqtt.takeover_exit_after': 0,
            'heartbeat.interval_s': 0,
            'reply.feedback_max_rate_hz': 0,
            'reply.retained_rejections_per_minute': 0,
            'watchdog.mode': 'stop_and_hold',
            'mirror.availability_suffix': '',
            'mirror.topics': ['zigbee2mqtt/+', 'ros2_bridge/ha/sensors/#'],
            'executor.call_warn_s': 0,
            'robot_id': 'a' * 32,
        }
    )
    assert settings.mqtt.takeover_exit_after == 0 and settings.heartbeat_interval_s == 0.0
    assert settings.watchdog.mode == 'stop_and_hold'
    assert settings.mirror.availability_suffix == ''
    assert settings.mirror.topics == ('zigbee2mqtt/+', 'ros2_bridge/ha/sensors/#')
    assert settings.robot.id == 'a' * 32


def test_telemetry_rate_zero_disables_and_a_delta_applies_to_both_units() -> None:
    off = build(**{'telemetry.joints_rate_hz': 0})
    assert off.telemetry.joints_rate_hz == 0.0 and not off.telemetry.joints_enabled
    custom = build(**{'telemetry.joints_rate_hz': 10, 'telemetry.joints_min_delta': 0.25})
    assert custom.telemetry.joints_rate_hz == 10.0 and custom.telemetry.joints_enabled
    assert custom.telemetry.min_delta_for('deg') == 0.25
    assert custom.telemetry.min_delta_for('rad') == 0.25
    assert custom.as_mapping()['telemetry.joints_min_delta'] == 0.25
    view = executor_parameters(custom)
    assert view['telemetry.joints_rate_hz'] == 10.0


def test_tls_settings_are_carried_through() -> None:
    settings = build(
        **{
            'mqtt.tls.enabled': True,
            'mqtt.tls.ca_cert': '/etc/example/ca.pem',
            'mqtt.tls.certfile': '/etc/example/client.pem',
            'mqtt.tls.keyfile': '/etc/example/client.key',
            'mqtt.tls.insecure': True,
        }
    )
    tls = settings.mqtt.tls
    assert tls.enabled and tls.insecure
    assert (tls.ca_cert, tls.certfile, tls.keyfile) == (
        '/etc/example/ca.pem',
        '/etc/example/client.pem',
        '/etc/example/client.key',
    )


# --------------------------------------------------------------------------- presentation


def test_describe_masks_the_secrets_and_names_their_source() -> None:
    environ = {ENV_MQTT_PASSWORD: 'env-password', ENV_COMMAND_SECRET: 'env-secret'}
    shown = describe_settings(build(environ))
    assert shown['mqtt.password'] == f'{MASK} (from {ENV_MQTT_PASSWORD})'
    assert shown['command.secret'] == f'{MASK} (from {ENV_COMMAND_SECRET})'
    shown = describe_settings(build(**{'mqtt.password': 'p', 'command.secret': 's'}))
    assert shown['mqtt.password'] == MASK and shown['command.secret'] == MASK
    shown = describe_settings(build())
    assert shown['mqtt.password'] == '' and shown['command.secret'] == ''
    assert shown['mqtt.client_id'] == 'ros2_ha_bridge_arm_left', 'effective, derived values'
    assert list(shown) == list(PARAMETER_NAMES)


def test_format_settings_never_contains_a_secret() -> None:
    settings = build(**{'mqtt.password': 'hunter-two', 'command.secret': 'top-secret-value'})
    line = format_settings(settings)
    assert 'hunter-two' not in line and 'top-secret-value' not in line
    assert f"mqtt.password='{MASK}'" in line and f"command.secret='{MASK}'" in line
    assert "robot.id='arm_left'" in line and 'mqtt.port=1883' in line
    assert '\n' not in line


def test_repr_of_settings_never_contains_a_secret() -> None:
    settings = build(**{'mqtt.password': 'hunter-two', 'command.secret': 'top-secret-value'})
    for text in (repr(settings), repr(settings.mqtt), repr(settings.command), str(settings)):
        assert 'hunter-two' not in text and 'top-secret-value' not in text
    assert settings.mqtt.password == 'hunter-two'
    assert settings.command.secret == 'top-secret-value'


def test_executor_parameter_view_excludes_the_secrets() -> None:
    settings = build(
        **{
            'mqtt.password': 'p',
            'command.secret': 's',
            'fake.task_duration_s': 0.5,
        }
    )
    view = executor_parameters(settings)
    assert not SECRET_PARAMETERS & set(view)
    assert view['fake.task_duration_s'] == 0.5 and view['fake.feedback_hz'] == 2.0
    assert view['robot.id'] == ROBOT
    assert set(view) == set(PARAMETER_NAMES) - SECRET_PARAMETERS


def test_settings_are_frozen() -> None:
    settings = build()
    with pytest.raises(AttributeError):
        settings.topic_prefix = 'other'  # type: ignore[misc]
    with pytest.raises(AttributeError):
        settings.mqtt.port = 1  # type: ignore[misc]


def test_module_exports_are_consistent() -> None:
    for name in parameters.__all__:
        assert hasattr(parameters, name), name
