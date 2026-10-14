# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Contract conformance of everything the node puts on the wire.

Every payload the foundation encoders can build (``Announce``,
``StatePayload``, ``HeartbeatPayload``, ``AckPayload``, ``FeedbackPayload``,
``ResultPayload``) is checked, over a realistic spread of values, against the
vendored schema of its kind through the conftest validator; every golden
command decodes through ``protocol.decode_command`` and the ack the node would
answer with validates too; sensor-mirror envelopes are valid JSON wrapping the
golden sensor payloads unchanged; and the vendored schemas are byte-identical
to ``contract/schemas``. ``test_protocol.py`` checks the encoders against the
golden examples field by field; this module checks the wider value space.
"""

from __future__ import annotations

import json
import os
from importlib import resources
from pathlib import Path
from typing import Any

import pytest

from ros2_ha_bridge import protocol
from ros2_ha_bridge.executors.base import TaskSpec
from ros2_ha_bridge.protocol import (
    ACK_CODES,
    DEFAULT_PREFIX,
    MAX_ANNOUNCE_BYTES,
    MAX_PAYLOAD_BYTES,
    RESULT_STATUSES,
    STATES,
    TASK_KINDS,
    WATCHDOG_MODES,
    AckPayload,
    Announce,
    Capabilities,
    Counters,
    DecodedCommand,
    ExecutorHealth,
    ExecutorInfo,
    FeedbackPayload,
    HeartbeatPayload,
    ResultPayload,
    RobotError,
    StatePayload,
)
from ros2_ha_bridge.sensor_mirror import (
    ENCODING_BASE64,
    ENCODING_UTF8,
    ENVELOPE_KEYS,
    SensorMirror,
)

from .conftest import CONTRACT_DIR, CONTRACT_DIR_ENV, KINDS, SCHEMA_NAMES
from .fakes import DEFAULT_EPOCH_MS, FakeClock

ROBOT = 'arm_left'
SESSION = '9f1c7a0e-4a1b-4b2c-8d1e-0f1a2b3c4d5e'
INSTANCE = 'robotpc:4123'
TASK_ID = '6a0c2b58-1d7e-4a1f-9b2a-3c4d5e6f7a8b'
CANCEL_ID = '0c1d4e6f-2a3b-4c5d-8e9f-0a1b2c3d4e5f'
T0 = DEFAULT_EPOCH_MS

MOVE_TO_PARAMS: dict[str, Any] = {
    'type': 'object',
    'properties': {
        'pose': {'type': 'string', 'enum': ['home', 'rest', 'Küche']},
        'speed': {'type': 'number', 'minimum': 0, 'maximum': 1, 'description': 'Fraction'},
        'retries': {'type': 'integer', 'minimum': 0, 'maximum': 3},
        'gentle': {'type': 'boolean'},
    },
    'required': ['pose'],
}


def _tasks() -> list[TaskSpec]:
    return [
        TaskSpec('reset', 'Reset to home', 'motion', 60),
        TaskSpec('stop', 'Stop', 'control', 5, None, 'Cancel the active task'),
        TaskSpec('move_to', 'Move to named pose', 'motion', 120.5, MOVE_TO_PARAMS),
        TaskSpec('status', 'Report status', 'query', 2),
    ]


# --------------------------------------------------------------------------- encoder cases


def _announce_cases() -> dict[str, Announce]:
    return {
        'full_identity_and_executor': Announce(
            robot_id=ROBOT,
            name='Left arm',
            manufacturer='Example Robotics',
            model='UR3e',
            sw_version='driver 2.4.1',
            hw_version='rev C',
            serial_number='SN-0001',
            suggested_area='Küche',
            executor=ExecutorInfo('fake', '0.1.0'),
            session=SESSION,
            instance=INSTANCE,
            tasks=_tasks(),
            ts=T0,
        ),
        'minimal': Announce(
            robot_id='arm_right',
            name='Right arm',
            session=SESSION,
            instance='bench-pc.local:988',
            tasks=[TaskSpec('reset', 'Reset to home')],
            ts=T0,
        ),
        'hmac_required_no_cancel_no_feedback': Announce(
            robot_id='gripper_1',
            name='Bench gripper',
            capabilities=Capabilities(cancel=False, feedback=False, hmac_required=True),
            watchdog_mode='stop_and_hold',
            heartbeat_interval_s=0.5,
            executor=ExecutorInfo('custom', '1.2.0'),
            session=SESSION,
            instance=INSTANCE,
            tasks=_tasks(),
            ts=T0,
        ),
        'heartbeat_disabled_empty_catalogue': Announce(
            robot_id='a' * 32,
            name='Thirty-two character robot id',
            watchdog_mode='inhibit_new',
            heartbeat_interval_s=0,
            session=SESSION,
            instance=INSTANCE,
            tasks=[],
            ts=0,
        ),
    }


def _state_cases() -> dict[str, StatePayload]:
    return {
        'idle_ha_link_down': StatePayload(
            robot_id=ROBOT,
            state='idle',
            since=T0,
            session=SESSION,
            instance=INSTANCE,
            seq=0,
            ts=T0,
        ),
        'busy_move_to_ha_link_up': StatePayload(
            robot_id=ROBOT,
            state='busy',
            task_id=TASK_ID,
            task='move_to',
            since=T0 + 1000,
            ha_link='up',
            session=SESSION,
            instance=INSTANCE,
            seq=42,
            ts=T0 + 1003,
        ),
        'busy_paused_inhibited': StatePayload(
            robot_id=ROBOT,
            state='busy',
            task_id=TASK_ID,
            task='wait',
            since=T0,
            paused=True,
            inhibited=True,
            watchdog_mode='inhibit_new',
            session=SESSION,
            instance=INSTANCE,
            seq=43,
            ts=T0 + 5000,
        ),
        'error_with_degraded_executor': StatePayload(
            robot_id=ROBOT,
            state='error',
            since=T0,
            watchdog_mode='stop_and_hold',
            executor_health=ExecutorHealth('degraded', 'start() took 0.35 s'),
            error=RobotError('timeout', "Task 'hang' did not finish within 10 s"),
            session=SESSION,
            instance=INSTANCE,
            seq=44,
            ts=T0 + 70_000,
        ),
        'error_code_normalised': StatePayload(
            robot_id=ROBOT,
            state='error',
            since=T0,
            executor_health=ExecutorHealth('error', 'Driver lost: Küche controller'),
            error=RobotError('Controller Error!', ''),
            session=SESSION,
            instance=INSTANCE,
            seq=45,
            ts=T0,
        ),
    }


def _heartbeat_cases() -> dict[str, HeartbeatPayload]:
    busy = Counters(
        accepted=12,
        rejected=3,
        duplicates=2,
        dropped_retained=1,
        dropped_oversize=1,
        dropped_invalid_id=4,
        dropped_queue_full=1,
        publish_errors=2,
        takeovers=1,
    )
    return {
        'before_first_ha_heartbeat': HeartbeatPayload(
            robot_id=ROBOT,
            session=SESSION,
            instance=INSTANCE,
            seq=0,
            ts=T0,
            uptime_s=0.5,
            counters=Counters(),
        ),
        'steady_positive_offset': HeartbeatPayload(
            robot_id=ROBOT,
            session=SESSION,
            instance=INSTANCE,
            seq=1203,
            ts=T0 + 4000,
            uptime_s=2406,
            counters=busy,
            ha_clock_offset_ms=12,
        ),
        'negative_offset_fractional_uptime': HeartbeatPayload(
            robot_id=ROBOT,
            session=SESSION,
            instance=INSTANCE,
            seq=7,
            ts=T0,
            uptime_s=14.25,
            counters=busy,
            ha_clock_offset_ms=-1800,
        ),
    }


def _ack_cases() -> dict[str, AckPayload]:
    cases: dict[str, AckPayload] = {}
    for code in ACK_CODES:
        cases[f'task_{code}'] = AckPayload(
            id=TASK_ID,
            robot_id=ROBOT,
            type='task',
            task='move_to',
            code=code,
            reason='' if code == 'accepted' else f'because {code}',
            received_at=T0 + 962,
            clock_skew_ms=12,
            session=SESSION,
            ts=T0 + 963,
        )
    cases['cancel_accepted'] = AckPayload(
        id=CANCEL_ID,
        robot_id=ROBOT,
        type='cancel',
        code='accepted',
        received_at=T0 + 2011,
        clock_skew_ms=0,
        session=SESSION,
        ts=T0 + 2012,
    )
    cases['cancel_no_such_task'] = AckPayload(
        id=CANCEL_ID,
        robot_id=ROBOT,
        type='cancel',
        code='rejected_no_such_task',
        reason='No task is running',
        received_at=T0,
        clock_skew_ms=-2500,
        session=SESSION,
        ts=T0,
    )
    cases['malformed_without_task_or_skew'] = AckPayload(
        id=TASK_ID,
        robot_id=ROBOT,
        type='task',
        code='rejected_malformed',
        reason='issued_at is missing',
        received_at=T0,
        session=SESSION,
        ts=T0 + 1,
    )
    cases['future_negative_skew'] = AckPayload(
        id=TASK_ID,
        robot_id=ROBOT,
        type='task',
        task='reset',
        code='rejected_future',
        reason='issued 3 s ahead of this clock',
        received_at=T0,
        clock_skew_ms=-3000,
        session=SESSION,
        ts=T0,
    )
    return cases


def _feedback_cases() -> dict[str, FeedbackPayload]:
    return {
        'progress_unknown': FeedbackPayload(id=TASK_ID, robot_id=ROBOT, session=SESSION, ts=T0),
        'progress_zero': FeedbackPayload(
            id=TASK_ID, robot_id=ROBOT, progress=0.0, stage='planning', session=SESSION, ts=T0
        ),
        'progress_half_non_ascii': FeedbackPayload(
            id=TASK_ID,
            robot_id=ROBOT,
            progress=0.5,
            stage='executing',
            message='Wegpunkt 2 von 5 (Küche)',
            session=SESSION,
            ts=T0 + 2100,
        ),
        'progress_complete': FeedbackPayload(
            id=TASK_ID, robot_id=ROBOT, progress=1, stage='done', session=SESSION, ts=T0 + 3000
        ),
    }


def _result_cases() -> dict[str, ResultPayload]:
    cases: dict[str, ResultPayload] = {}
    codes = {'succeeded': 'ok', 'cancelled': 'preempted', 'aborted': 'collision'}
    for status in RESULT_STATUSES:
        for kind in TASK_KINDS:
            cases[f'{kind}_{status}'] = ResultPayload(
                id=TASK_ID,
                robot_id=ROBOT,
                task='move_to',
                kind=kind,
                status=status,
                code=codes[status],
                reason='' if status == 'succeeded' else f'{status} by the executor',
                started_at=T0 + 970,
                finished_at=T0 + 3410,
                session=SESSION,
                ts=T0 + 3412,
            )
    cases['nested_data_and_normalised_code'] = ResultPayload(
        id=TASK_ID,
        robot_id=ROBOT,
        task='move_to',
        kind='motion',
        status='aborted',
        code='MoveIt: PLANNING_FAILED',
        reason="Planned path collides with object 'mug_42' (Küche)",
        data={
            'moveit_error_code': -12,
            'contacts': [{'link': 'wrist_3', 'object': 'mug_42'}],
            'nested': {'depth': {'ok': False, 'ratio': 0.25, 'note': None}},
        },
        started_at=T0,
        finished_at=T0,
        session=SESSION,
        ts=T0,
    )
    return cases


ENCODER_CASES: list[tuple[str, str, Any, int]] = [
    *(('announce', n, p, MAX_ANNOUNCE_BYTES) for n, p in _announce_cases().items()),
    *(('state', n, p, MAX_PAYLOAD_BYTES) for n, p in _state_cases().items()),
    *(('heartbeat_robot', n, p, MAX_PAYLOAD_BYTES) for n, p in _heartbeat_cases().items()),
    *(('ack', n, p, MAX_PAYLOAD_BYTES) for n, p in _ack_cases().items()),
    *(('feedback', n, p, MAX_PAYLOAD_BYTES) for n, p in _feedback_cases().items()),
    *(('result', n, p, MAX_PAYLOAD_BYTES) for n, p in _result_cases().items()),
]


@pytest.mark.parametrize(
    ('kind', 'encoder', 'limit'),
    [(kind, encoder, limit) for kind, _, encoder, limit in ENCODER_CASES],
    ids=[f'{kind}/{name}' for kind, name, _, _ in ENCODER_CASES],
)
def test_encoder_payload_validates_against_its_schema(
    kind: str, encoder: Any, limit: int, schema_validator: Any
) -> None:
    payload = encoder.to_payload()
    schema_validator.assert_valid(kind, payload)
    assert payload['schema'] == protocol.SCHEMA_VERSION
    wire = protocol.encode_payload(payload)
    assert len(wire) <= limit
    decoded = json.loads(wire)
    assert decoded == payload, 'the wire form round-trips to the same JSON document'
    schema_validator.assert_valid(kind, decoded)
    # Robot identity on every per-robot payload.
    assert protocol.is_valid_robot_id(payload['robot_id'])
    assert protocol.is_valid_session(payload['session'])


def test_encoder_cases_cover_the_closed_sets() -> None:
    acks = {p.code for k, _, p, _ in ENCODER_CASES if k == 'ack'}
    assert acks == set(ACK_CODES)
    results = {(p.kind, p.status) for k, _, p, _ in ENCODER_CASES if k == 'result'}
    assert results == {(kind, status) for kind in TASK_KINDS for status in RESULT_STATUSES}
    states = {p.state for k, _, p, _ in ENCODER_CASES if k == 'state'}
    assert states == set(STATES)
    modes = {p.watchdog_mode for k, _, p, _ in ENCODER_CASES if k in ('announce', 'state')}
    assert modes == set(WATCHDOG_MODES)


def test_announce_task_specs_pass_the_params_subset_check() -> None:
    for announce in _announce_cases().values():
        for spec in announce.tasks:
            assert protocol.validate_task_spec(spec) == []


def test_result_and_error_codes_are_normalised_onto_the_wire(schema_validator: Any) -> None:
    result = _result_cases()['nested_data_and_normalised_code'].to_payload()
    assert result['code'] == 'moveit_planning_failed'
    assert result['goal_status'] == 6
    state = _state_cases()['error_code_normalised'].to_payload()
    assert state['error'] == {'code': 'controller_error', 'reason': ''}
    for kind, payload in (('result', result), ('state', state)):
        schema_validator.assert_valid(kind, payload)


# --------------------------------------------------------------------------- golden commands


def test_golden_command_decodes_and_its_ack_validates(
    valid_command_example: Any, schema_validator: Any, secret: str
) -> None:
    payload = valid_command_example.payload
    schema_validator.assert_valid('command', payload)
    for raw in (valid_command_example.raw, protocol.encode_payload(payload)):
        decoded = protocol.decode_command(raw, expected_robot_id=payload['robot_id'])
        assert isinstance(decoded, DecodedCommand), decoded
        assert decoded.id == payload['id'] and decoded.robot_id == payload['robot_id']
        assert decoded.type == payload['type']
        assert decoded.payload == payload
        schema_validator.assert_valid('command', dict(decoded.payload))
        topic = protocol.cmd_topic(DEFAULT_PREFIX, decoded.robot_id)
        if decoded.sig is not None:
            assert protocol.verify_sig(secret, topic, decoded.payload)
        # The ack the node would answer with names the same id and type and validates.
        ack = AckPayload(
            id=decoded.id,
            robot_id=decoded.robot_id,
            type=decoded.type,
            task=decoded.task,
            code='accepted',
            received_at=decoded.issued_at + 12,
            clock_skew_ms=12,
            session=SESSION,
            ts=decoded.issued_at + 13,
        )
        ack_payload = ack.to_payload()
        schema_validator.assert_valid('ack', ack_payload)
        assert protocol.ack_topic(DEFAULT_PREFIX, decoded.robot_id, decoded.id).endswith(
            f'/task/{ack_payload["id"]}/ack'
        )
        if decoded.is_task:
            assert decoded.task is not None and protocol.is_valid_task_name(decoded.task)
            assert protocol.validate_params(None, decoded.params) == []
        else:
            assert decoded.task is None


def test_golden_example_round_trips_through_the_wire_encoding(golden_example: Any) -> None:
    wire = protocol.encode_payload(golden_example.payload)
    assert json.loads(wire) == golden_example.payload
    assert json.loads(golden_example.raw) == golden_example.payload


# --------------------------------------------------------------------------- sensor envelopes


def test_sensor_envelope_wraps_the_golden_payload_unchanged(
    valid_sensor_example: Any, schema_validator: Any, clock: FakeClock
) -> None:
    payload = valid_sensor_example.payload
    topic = protocol.ha_sensor_topic(DEFAULT_PREFIX, payload['domain'], payload['object_id'])
    texts: list[str] = []
    mirror = SensorMirror([f'{DEFAULT_PREFIX}/ha/sensors/#'], texts.append, clock=clock)
    assert mirror.subscriptions() == [(f'{DEFAULT_PREFIX}/ha/sensors/#', 0)]
    assert mirror.handle(topic, valid_sensor_example.raw, retain=True)
    assert len(texts) == 1
    texts[0].encode('utf-8')
    envelope = json.loads(texts[0])
    assert list(envelope) == list(ENVELOPE_KEYS)
    assert envelope['topic'] == topic
    assert envelope['encoding'] == ENCODING_UTF8
    assert envelope['retain'] is True
    assert envelope['source_available'] is None, 'the canonical path carries its own available'
    assert envelope['rx_ts_ms'] == DEFAULT_EPOCH_MS
    assert envelope['payload'] == valid_sensor_example.raw.decode('utf-8')
    inner = json.loads(envelope['payload'])
    assert inner == payload
    schema_validator.assert_valid('sensor', inner)


def test_sensor_envelope_is_valid_json_for_zigbee2mqtt_and_binary_payloads(
    clock: FakeClock,
) -> None:
    texts: list[str] = []
    mirror = SensorMirror(['zigbee2mqtt/kitchen_motion'], texts.append, clock=clock)
    assert mirror.handle('zigbee2mqtt/kitchen_motion/availability', b'{"state":"online"}')
    assert mirror.handle(
        'zigbee2mqtt/kitchen_motion',
        '{"occupancy":true,"battery":97,"voltage":2975,"room":"Küche"}'.encode(),
    )
    assert mirror.handle('zigbee2mqtt/kitchen_motion', b'\xff\x00binary')
    envelopes = [json.loads(text) for text in texts]
    assert [e['source_available'] for e in envelopes] == [True, True, True]
    assert envelopes[1]['encoding'] == ENCODING_UTF8
    assert json.loads(envelopes[1]['payload'])['room'] == 'Küche'
    assert envelopes[2]['encoding'] == ENCODING_BASE64
    for text in texts:
        assert text == json.dumps(json.loads(text), separators=(',', ':'), ensure_ascii=False)


# --------------------------------------------------------------------------- contract folder


def test_contract_folder_is_found_inside_the_repository() -> None:
    """The golden fixtures must never be skipped silently when the repository is present.

    ``conftest.contract_dir`` skips when the folder is absent so that an
    installed-package run (colcon without the source tree) still passes. Inside
    the repository, or with ``HA_ROS2_BRIDGE_CONTRACT_DIR`` set, that skip would
    hide every golden-example and HMAC-vector test behind a green run, so this
    test fails loudly instead.
    """
    repository_contract = Path(__file__).resolve().parents[3] / 'contract'
    override = os.environ.get(CONTRACT_DIR_ENV)
    if override:
        assert CONTRACT_DIR is not None, (
            f'{CONTRACT_DIR_ENV}={override!r} names no contract folder'
        )
        assert CONTRACT_DIR == Path(override)
    elif repository_contract.is_dir():
        assert CONTRACT_DIR == repository_contract, (
            f'{repository_contract} exists but conftest did not find it'
        )
    else:
        pytest.skip('not running inside the repository; the installed-package run has no fixtures')
    assert (CONTRACT_DIR / 'hmac_vectors.json').is_file()
    assert any((CONTRACT_DIR / 'examples' / 'valid').iterdir())


# --------------------------------------------------------------------------- vendored schemas


def _vendored_schemas() -> dict[str, bytes]:
    root = resources.files('ros2_ha_bridge').joinpath('schemas')
    return {
        entry.name: entry.read_bytes()
        for entry in root.iterdir()
        if entry.is_file() and entry.name.endswith('.schema.json')
    }


def test_vendored_schemas_are_byte_identical_to_the_contract(contract_dir: Path) -> None:
    source_dir = contract_dir / 'schemas'
    source = {path.name: path.read_bytes() for path in sorted(source_dir.glob('*.schema.json'))}
    vendored = _vendored_schemas()
    assert source, f'no schemas found under {source_dir}'
    assert sorted(source) == sorted(f'{name}.schema.json' for name in SCHEMA_NAMES)
    assert sorted(vendored) == sorted(source), 'vendored copy has missing or stale schemas'
    for name, data in source.items():
        assert vendored[name] == data, f'{name} differs; run scripts/sync_contract.py'
        document = json.loads(data)
        assert document['$id'].endswith('/' + name)
        assert document['$schema'] == 'https://json-schema.org/draft/2020-12/schema'


def test_every_kind_has_a_schema_and_a_validator(schema_validator: Any) -> None:
    for kind in KINDS:
        assert schema_validator.validator(kind) is schema_validator.validator(kind)
    with pytest.raises(ValueError):
        schema_validator.validator('common')
