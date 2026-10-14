# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Contract tests for :mod:`ros2_ha_bridge.protocol`.

Every HMAC vector recomputes, every golden command decodes, every must-reject
command is refused with the class its file name implies, every outbound
payload the module can build validates against the vendored schemas (and
matches the golden example it mirrors), topic builders refuse bad prefixes and
ids, and the params-subset validator covers each rule both ways.

How the must-reject command files map onto :func:`protocol.decode_command`:
``oversize_*`` gives ``oversize``, ``id_*`` gives ``invalid_id`` and everything
else gives ``malformed``. One nuance is documented in
:func:`test_invalid_command_example_is_refused`: ``id_70kb`` is caught by the
size gate first (the contract runs the size check before parsing), and only
with the size limit disabled does its id get as far as the UUID check. No
must-reject command fixture is deferred to a later stage: freshness and
signature verification have no schema fixtures, by design.
"""

from __future__ import annotations

import json
import re
from typing import Any

import pytest

from ros2_ha_bridge import protocol
from ros2_ha_bridge.executors.base import JointInfo, TaskSpec
from ros2_ha_bridge.protocol import (
    ACK_CODES,
    COMMAND_TYPES,
    DEFAULT_PREFIX,
    JOINT_UNITS,
    MAX_ANNOUNCE_BYTES,
    MAX_PAYLOAD_BYTES,
    RESULT_STATUSES,
    STATES,
    TASK_KINDS,
    AckPayload,
    Announce,
    AnnounceJoints,
    Capabilities,
    Counters,
    DecodedCommand,
    DecodeError,
    ExecutorHealth,
    ExecutorInfo,
    FeedbackPayload,
    HaHeartbeat,
    HeartbeatPayload,
    JointsPayload,
    ResultPayload,
    RobotError,
    RobotTopics,
    StatePayload,
)

from .conftest import assert_imports_without

ROBOT = 'arm_left'
SESSION = '9f1c7a0e-4a1b-4b2c-8d1e-0f1a2b3c4d5e'
INSTANCE = 'robotpc:4123'
TASK_ID = '6a0c2b58-1d7e-4a1f-9b2a-3c4d5e6f7a8b'
OTHER_ID = '7b3e9d21-5f6a-4c8b-9e0d-1f2a3b4c5d6e'
IDENTITY_KEYS = (
    'manufacturer',
    'model',
    'sw_version',
    'hw_version',
    'serial_number',
    'suggested_area',
)


def _without_null_identity(payload: dict[str, Any]) -> dict[str, Any]:
    """Drop the nullable identity keys when null, to compare with a minimal golden announce."""
    return {k: v for k, v in payload.items() if not (k in IDENTITY_KEYS and v is None)}


def _command_bytes(**overrides: Any) -> bytes:
    payload: dict[str, Any] = {
        'schema': 1,
        'id': TASK_ID,
        'type': 'task',
        'robot_id': ROBOT,
        'task': 'move_to',
        'params': {'pose': 'home'},
        'issued_at': 1760356800950,
        'max_age_ms': 5000,
        'timeout_s': None,
        'source': {'origin': 'ha', 'kind': 'intent'},
    }
    for key, value in overrides.items():
        if value is ...:
            payload.pop(key, None)
        else:
            payload[key] = value
    try:
        return protocol.encode_payload(payload)
    except UnicodeEncodeError:
        # A lone surrogate has no UTF-8 form; send it escaped, as an attacker would.
        return json.dumps(payload).encode('ascii')


# --------------------------------------------------------------------------- hygiene


def test_module_imports_neither_rclpy_nor_paho() -> None:
    assert_imports_without('ros2_ha_bridge.protocol', {'rclpy', 'paho'})


def test_closed_sets_match_the_contract() -> None:
    assert ACK_CODES == (
        'accepted',
        'rejected_malformed',
        'rejected_unknown_task',
        'rejected_invalid_params',
        'rejected_stale',
        'rejected_future',
        'rejected_unauthorised',
        'rejected_busy',
        'rejected_error',
        'rejected_inhibited',
        'rejected_executor',
        'rejected_no_such_task',
    )
    assert RESULT_STATUSES == ('succeeded', 'cancelled', 'aborted')
    assert TASK_KINDS == ('motion', 'control', 'query')
    assert STATES == ('idle', 'busy', 'error')
    assert COMMAND_TYPES == ('task', 'cancel')
    assert JOINT_UNITS == ('deg', 'rad')
    assert protocol.MAX_JOINTS == 32 and protocol.JOINTS_EXPIRY_S == 5
    assert protocol.COUNTER_NAMES == (
        'accepted',
        'rejected',
        'duplicates',
        'dropped_retained',
        'dropped_oversize',
        'dropped_invalid_id',
        'dropped_queue_full',
        'publish_errors',
        'takeovers',
    )


# --------------------------------------------------------------------------- HMAC vectors


def test_hmac_vector_recomputes(hmac_vector: dict[str, Any]) -> None:
    payload = hmac_vector['payload']
    secret, topic = hmac_vector['secret'], hmac_vector['topic']
    assert protocol.canonical_json(payload).decode('utf-8') == hmac_vector['canonical']
    sig = protocol.compute_sig(secret, topic, payload)
    assert sig == hmac_vector['sig']
    signed = dict(payload, sig=sig)
    assert protocol.canonical_json(signed) == protocol.canonical_json(payload)
    assert protocol.compute_sig(secret, topic, signed) == sig
    assert protocol.verify_sig(secret, topic, signed)
    assert protocol.verify_sig(secret, topic, payload, sig)


def test_hmac_vector_rejects_tampering(hmac_vector: dict[str, Any]) -> None:
    payload = hmac_vector['payload']
    secret, topic, sig = hmac_vector['secret'], hmac_vector['topic'], hmac_vector['sig']
    signed = dict(payload, sig=sig)
    assert not protocol.verify_sig(secret + 'x', topic, signed)
    assert not protocol.verify_sig(secret, topic + '/x', signed)
    assert not protocol.verify_sig(secret, 'lab/' + topic, signed)
    tampered = dict(signed, issued_at=payload['issued_at'] + 1)
    assert not protocol.verify_sig(secret, topic, tampered)
    assert not protocol.verify_sig(secret, topic, payload, sig.upper())
    assert not protocol.verify_sig(secret, topic, payload, sig[:-1])
    assert not protocol.verify_sig(secret, topic, payload, sig + '0')
    assert not protocol.verify_sig(secret, topic, payload, None)
    assert not protocol.verify_sig(secret, topic, payload, 42)
    assert not protocol.verify_sig(secret, topic, payload, b'')


def test_canonical_json_sorts_every_level_keeps_list_order_and_utf8() -> None:
    payload = {'b': {'z': [2, 1], 'a': 'Küche'}, 'a': True, 'sig': 'ignored', 'n': None}
    assert (
        protocol.canonical_json(payload)
        == '{"a":true,"b":{"a":"Küche","z":[2,1]},"n":null}'.encode()
    )
    assert protocol.encode_payload({'x': 'é', 'y': 1}) == '{"x":"é","y":1}'.encode()


# --------------------------------------------------------------------------- golden commands


def test_valid_command_example_decodes(
    valid_command_example: Any, hmac_vectors: list[dict[str, Any]], secret: str
) -> None:
    payload = valid_command_example.payload
    decoded: DecodedCommand | DecodeError = DecodeError('not_json', 'unset')
    for raw in (valid_command_example.raw, protocol.encode_payload(payload)):
        decoded = protocol.decode_command(raw)
        assert isinstance(decoded, DecodedCommand), decoded
        assert decoded.id == payload['id']
        assert decoded.type == payload['type']
        assert decoded.robot_id == payload['robot_id']
        assert decoded.issued_at == payload['issued_at']
        assert decoded.max_age_ms == payload['max_age_ms']
        assert decoded.source == payload['source']
        assert decoded.payload == payload
        assert decoded.size == len(raw)
        if payload['type'] == 'task':
            assert decoded.is_task and not decoded.is_cancel
            assert decoded.task == payload['task']
            assert decoded.params == payload['params']
            timeout = payload.get('timeout_s')
            assert decoded.timeout_s == (None if timeout is None else float(timeout))
            assert decoded.target_id is None
        else:
            assert decoded.is_cancel and not decoded.is_task
            assert decoded.target_id == payload['target_id']
            assert decoded.task is None and decoded.params is None
        assert isinstance(
            protocol.decode_command(raw, expected_robot_id=payload['robot_id']), DecodedCommand
        )
        mismatch = protocol.decode_command(raw, expected_robot_id='other_arm')
        assert isinstance(mismatch, DecodeError) and mismatch.kind == 'malformed'
        assert mismatch.command_id == payload['id']

    assert isinstance(decoded, DecodedCommand)
    topic = protocol.cmd_topic(DEFAULT_PREFIX, payload['robot_id'])
    if 'sig' in payload:
        vectors = [v for v in hmac_vectors if v['sig'] == payload['sig']]
        assert vectors, f'{valid_command_example.name} carries a sig that is in no HMAC vector'
        vector = vectors[0]
        assert vector['topic'] == topic
        assert decoded.sig == payload['sig']
        assert protocol.verify_sig(vector['secret'], topic, decoded.payload)
        assert protocol.verify_sig(secret, topic, decoded.payload)
        assert not protocol.verify_sig(secret, topic + '/x', decoded.payload)
    else:
        assert decoded.sig is None
        assert not protocol.verify_sig(secret, topic, decoded.payload)


def _expected_decode_kind(name: str) -> str:
    if name.startswith('oversize'):
        return 'oversize'
    if name.startswith('id_'):
        return 'invalid_id'
    return 'malformed'


def test_invalid_command_example_is_refused(invalid_command_example: Any) -> None:
    """Every must-reject command fails decode_command with the class its name implies.

    ``id_70kb`` is over 16 KiB, so with the default limit the size gate refuses
    it before the id is ever looked at (``oversize``); with the limit disabled
    the id itself is refused (``invalid_id``). Both are asserted. The
    ``oversize`` fixture is also fed in its compact wire form, which is still
    over the limit, as ``scripts/validate_contract.py`` guarantees.
    """
    name, payload = invalid_command_example.name, invalid_command_example.payload
    expected = _expected_decode_kind(name)
    inputs = [invalid_command_example.raw, protocol.encode_payload(payload)]
    for raw in inputs:
        if name == 'id_70kb':
            gated = protocol.decode_command(raw)
            assert isinstance(gated, DecodeError) and gated.kind == 'oversize'
            result = protocol.decode_command(raw, max_bytes=None)
        else:
            result = protocol.decode_command(raw)
        assert isinstance(result, DecodeError), f'{name} decoded as a command'
        assert result.kind == expected, f'{name}: {result.kind} ({result.reason})'
        assert result.reason
        assert result.size == len(raw)
        if expected == 'malformed':
            assert result.replyable
            assert result.command_id == payload['id']
            assert result.ack_type in COMMAND_TYPES
            assert protocol.ack_topic(DEFAULT_PREFIX, ROBOT, result.command_id)
        else:
            assert not result.replyable
            assert result.command_id is None


def test_malformed_hints_feed_the_rejected_malformed_ack() -> None:
    result = protocol.decode_command(_command_bytes(schema=...))
    assert isinstance(result, DecodeError)
    assert result.kind == 'malformed'
    assert result.command_type == 'task' and result.ack_type == 'task'
    assert result.task == 'move_to'
    assert result.issued_at == 1760356800950

    result = protocol.decode_command(_command_bytes(type='goal', task='move to', issued_at=1.5))
    assert isinstance(result, DecodeError)
    assert result.kind == 'malformed'
    assert result.command_type is None and result.ack_type == 'task'
    assert result.task is None and result.issued_at is None

    result = protocol.decode_command(_command_bytes(type='cancel', target_id='x'))
    assert isinstance(result, DecodeError)
    assert result.kind == 'malformed' and result.command_type == 'cancel'


@pytest.mark.parametrize(
    ('raw', 'kind', 'fragment'),
    [
        (b'', 'not_json', 'not JSON'),
        (b'\xff\xfe', 'not_json', 'not UTF-8'),
        (b'{"id": "' + TASK_ID.encode() + b'"', 'not_json', 'not JSON'),
        (b'NaN', 'not_json', 'NaN'),
        (b'{"id":"' + TASK_ID.encode() + b'","schema":1,"x":Infinity}', 'not_json', 'Infinity'),
        (b'[]', 'invalid_id', 'not a JSON object'),
        (b'"text"', 'invalid_id', 'not a JSON object'),
        (b'{}', 'invalid_id', 'id is missing'),
        (b'{"id": 7}', 'invalid_id', 'id is missing'),
        (b'{"id": null}', 'invalid_id', 'id is missing'),
        (b'{"id": "' + TASK_ID.encode() + b'\\n"}', 'invalid_id', 'id is missing'),
    ],
)
def test_decode_command_without_a_safe_reply_topic(raw: bytes, kind: str, fragment: str) -> None:
    result = protocol.decode_command(raw)
    assert isinstance(result, DecodeError)
    assert result.kind == kind
    assert fragment in result.reason
    assert not result.replyable


@pytest.mark.parametrize(
    ('overrides', 'fragment'),
    [
        ({'schema': True}, 'schema must be 1'),
        ({'schema': 1.0}, 'schema must be 1'),
        ({'schema': '1'}, 'schema must be 1'),
        ({'type': None}, "type must be 'task' or 'cancel'"),
        ({'robot_id': 'ha'}, 'robot_id'),
        ({'robot_id': 'Arm'}, 'robot_id'),
        ({'issued_at': -1}, 'issued_at'),
        ({'issued_at': True}, 'issued_at'),
        ({'max_age_ms': 0}, 'max_age_ms'),
        ({'max_age_ms': 5000.0}, 'max_age_ms'),
        ({'source': []}, 'source must be an object'),
        ({'source': {'origin': '', 'kind': 'intent'}}, 'source.origin'),
        (
            {'source': {'origin': 'ha', 'kind': 'intent', 'instance_id': None}},
            'source.instance_id',
        ),
        ({'source': {'origin': 'ha', 'kind': 'intent', 'user_id': 5}}, 'source.user_id'),
        ({'sig': 'A' * 64}, 'sig'),
        ({'sig': 5}, 'sig'),
        ({'params': []}, 'params must be an object'),
        ({'timeout_s': True}, 'timeout_s'),
        ({'timeout_s': -2}, 'timeout_s'),
        ({'timeout_s': 'soon'}, 'timeout_s'),
        ({'type': 'cancel', 'target_id': ...}, 'target_id is required'),
        ({'type': 'cancel', 'target_id': TASK_ID.upper()}, 'target_id must be null'),
        ({'params': {'note': '\ud800'}}, 'surrogate'),
    ],
)
def test_decode_command_malformed_reasons(overrides: dict[str, Any], fragment: str) -> None:
    result = protocol.decode_command(_command_bytes(**overrides))
    assert isinstance(result, DecodeError), result
    assert result.kind == 'malformed'
    assert fragment in result.reason
    assert result.command_id == TASK_ID


def test_decode_command_accepts_optional_fields_and_unknown_keys() -> None:
    raw = _command_bytes(
        timeout_s=..., extra='ignored', source={'origin': 'ha', 'kind': 'x', 'later': 1}
    )
    decoded = protocol.decode_command(raw)
    assert isinstance(decoded, DecodedCommand)
    assert decoded.timeout_s is None
    assert decoded.payload['extra'] == 'ignored'
    decoded = protocol.decode_command(_command_bytes(timeout_s=30))
    assert isinstance(decoded, DecodedCommand) and decoded.timeout_s == 30.0
    decoded = protocol.decode_command(
        _command_bytes(type='cancel', target_id=None, task=..., params=...)
    )
    assert isinstance(decoded, DecodedCommand) and decoded.is_cancel and decoded.target_id is None


def test_decode_command_size_gate_measures_the_bytes_given() -> None:
    raw = _command_bytes()
    assert isinstance(protocol.decode_command(raw, max_bytes=len(raw)), DecodedCommand)
    result = protocol.decode_command(raw, max_bytes=len(raw) - 1)
    assert (
        isinstance(result, DecodeError) and result.kind == 'oversize' and result.size == len(raw)
    )
    assert isinstance(protocol.decode_command(raw, max_bytes=None), DecodedCommand)


# --------------------------------------------------------------------------- HA heartbeat


def test_valid_ha_heartbeat_decodes(valid_heartbeat_ha_example: Any) -> None:
    payload = valid_heartbeat_ha_example.payload
    for raw in (valid_heartbeat_ha_example.raw, protocol.encode_payload(payload)):
        beat = protocol.decode_ha_heartbeat(raw)
        assert isinstance(beat, HaHeartbeat), beat
        assert (beat.ts, beat.seq, beat.interval_ms, beat.instance_id) == (
            payload['ts'],
            payload['seq'],
            payload['interval_ms'],
            payload['instance_id'],
        )


def test_invalid_ha_heartbeat_is_refused(invalid_heartbeat_ha_example: Any) -> None:
    result = protocol.decode_ha_heartbeat(invalid_heartbeat_ha_example.raw)
    assert isinstance(result, DecodeError), result
    assert result.kind == 'malformed' and result.reason


def test_ha_heartbeat_size_and_json_gates() -> None:
    oversize = protocol.decode_ha_heartbeat(b'{' + b' ' * MAX_PAYLOAD_BYTES + b'}')
    assert isinstance(oversize, DecodeError) and oversize.kind == 'oversize'
    not_json = protocol.decode_ha_heartbeat(b'{')
    assert isinstance(not_json, DecodeError) and not_json.kind == 'not_json'
    not_object = protocol.decode_ha_heartbeat(b'[1]')
    assert isinstance(not_object, DecodeError) and not_object.kind == 'malformed'


# --------------------------------------------------------------------------- outbound payloads


def test_golden_example_validates_against_vendored_schema(
    golden_example: Any, schema_validator: Any
) -> None:
    schema_validator.assert_valid(golden_example.kind, golden_example.payload)
    if golden_example.kind == 'command':
        schema_validator.assert_invalid(
            golden_example.kind, dict(golden_example.payload, schema=2)
        )


def test_invalid_announce_example_is_refused(
    invalid_announce_example: Any, schema_validator: Any
) -> None:
    """Every must-reject announce fails the schema, except the oversize one, which fails on size."""
    payload = invalid_announce_example.payload
    if invalid_announce_example.name.startswith('oversize'):
        schema_validator.assert_valid('announce', payload)
        assert len(protocol.encode_payload(payload)) > MAX_ANNOUNCE_BYTES
    else:
        schema_validator.assert_invalid('announce', payload)


def test_invalid_joints_example_is_refused(
    invalid_joints_example: Any, schema_validator: Any
) -> None:
    schema_validator.assert_invalid('joints', invalid_joints_example.payload)


def test_ack_payloads_match_golden_examples(valid_examples: Any, schema_validator: Any) -> None:
    golden = valid_examples('ack')
    cases = {
        'accepted_task': AckPayload(
            id=TASK_ID,
            robot_id=ROBOT,
            type='task',
            task='move_to',
            code='accepted',
            received_at=1760356800962,
            clock_skew_ms=12,
            session=SESSION,
            ts=1760356800963,
        ),
        'rejected_busy': AckPayload(
            id=TASK_ID,
            robot_id=ROBOT,
            type='task',
            task='move_to',
            code='rejected_busy',
            reason="Task 'reset' (id 51f0a2b3-c4d5-4e6f-a7b8-c9d0e1f2a3b4) is running",
            received_at=1760356800962,
            clock_skew_ms=12,
            session=SESSION,
            ts=1760356800963,
        ),
        'accepted_cancel': AckPayload(
            id='0c1d4e6f-2a3b-4c5d-8e9f-0a1b2c3d4e5f',
            robot_id=ROBOT,
            type='cancel',
            code='accepted',
            received_at=1760356802011,
            clock_skew_ms=11,
            session=SESSION,
            ts=1760356802012,
        ),
        'rejected_malformed_no_skew': AckPayload(
            id='e5f6a7b8-c9d0-4e1f-8a2b-3c4d5e6f7a8b',
            robot_id=ROBOT,
            type='task',
            code='rejected_malformed',
            reason='issued_at is missing',
            received_at=1760356840000,
            session=SESSION,
            ts=1760356840001,
        ),
    }
    assert set(cases) == set(golden), 'a golden ack example has no mirror in this test'
    for name, ack in cases.items():
        payload = ack.to_payload()
        assert payload == golden[name].payload, name
        assert payload['accepted'] is (ack.code == 'accepted')
        schema_validator.assert_valid('ack', payload)
        assert len(protocol.encode_payload(payload)) <= MAX_PAYLOAD_BYTES


@pytest.mark.parametrize('code', ACK_CODES)
def test_every_ack_code_validates(code: Any, schema_validator: Any) -> None:
    ack = AckPayload(
        id=TASK_ID,
        robot_id=ROBOT,
        type='task',
        task='move_to',
        code=code,
        reason='' if code == 'accepted' else 'why',
        received_at=1760356800962,
        clock_skew_ms=None,
        session=SESSION,
        ts=1760356800963,
    )
    assert ack.accepted is (code == 'accepted')
    schema_validator.assert_valid('ack', ack.to_payload())


def test_result_payloads_match_golden_examples(valid_examples: Any, schema_validator: Any) -> None:
    golden = valid_examples('result')
    cases = {
        'aborted_collision': ResultPayload(
            id=TASK_ID,
            robot_id=ROBOT,
            task='move_to',
            kind='motion',
            status='aborted',
            code='collision',
            reason="Planned path collides with object 'mug_42'",
            data={'moveit_error_code': -12},
            started_at=1760356800970,
            finished_at=1760356803410,
            session=SESSION,
            ts=1760356803412,
        ),
        'cancelled_preempted': ResultPayload(
            id=OTHER_ID,
            robot_id=ROBOT,
            task='wait',
            kind='motion',
            status='cancelled',
            code='preempted',
            reason='Stopped by command 2d4f6a8c-0e1b-4a3c-8d5e-6f7a8b9c0d1e',
            started_at=1760356805000,
            finished_at=1760356809000,
            session=SESSION,
            ts=1760356809001,
        ),
        'succeeded_reset': ResultPayload(
            id='51f0a2b3-c4d5-4e6f-a7b8-c9d0e1f2a3b4',
            robot_id=ROBOT,
            task='reset',
            kind='motion',
            status='succeeded',
            code='ok',
            started_at=1760356800120,
            finished_at=1760356802120,
            session=SESSION,
            ts=1760356802121,
        ),
        'control_nothing_to_pause': ResultPayload(
            id='4e6a8c0f-1b3d-4d5f-9a7c-8e0f2a4b6c8d',
            robot_id=ROBOT,
            task='pause',
            kind='control',
            status='aborted',
            code='nothing_to_pause',
            reason='No task is running',
            started_at=1760356850000,
            finished_at=1760356850001,
            session=SESSION,
            ts=1760356850002,
        ),
    }
    assert set(cases) == set(golden), 'a golden result example has no mirror in this test'
    for name, result in cases.items():
        payload = result.to_payload()
        assert payload == golden[name].payload, name
        schema_validator.assert_valid('result', payload)


@pytest.mark.parametrize(
    ('status', 'goal_status'), [('succeeded', 4), ('cancelled', 5), ('aborted', 6)]
)
def test_result_goal_status_follows_status(
    status: Any, goal_status: int, schema_validator: Any
) -> None:
    result = ResultPayload(
        id=TASK_ID,
        robot_id=ROBOT,
        task='move_to',
        kind='motion',
        status=status,
        code='Collision Detected!',
        started_at=1,
        finished_at=2,
        session=SESSION,
        ts=3,
    )
    assert result.goal_status == goal_status
    payload = result.to_payload()
    assert payload['goal_status'] == goal_status
    assert payload['code'] == 'collision_detected'
    schema_validator.assert_valid('result', payload)
    schema_validator.assert_invalid('result', dict(payload, goal_status=7))


@pytest.mark.parametrize(
    ('given', 'expected'),
    [
        ('ok', 'ok'),
        ('Collision', 'collision'),
        ('no-trajectory found', 'no_trajectory_found'),
        ('123abc', 'abc'),
        ('', 'unknown'),
        ('___', 'unknown'),
        (None, 'unknown'),
        ('a' * 70, 'a' * 64),
    ],
)
def test_normalise_result_code(given: Any, expected: str) -> None:
    assert protocol.normalise_result_code(given) == expected
    assert protocol.is_valid_result_code(protocol.normalise_result_code(given))


def test_state_payloads_match_golden_examples(valid_examples: Any, schema_validator: Any) -> None:
    golden = valid_examples('state')
    cases = {
        'busy_move_to': StatePayload(
            robot_id=ROBOT,
            state='busy',
            task_id=TASK_ID,
            task='move_to',
            since=1760356801000,
            ha_link='up',
            session=SESSION,
            instance=INSTANCE,
            seq=42,
            ts=1760356801003,
        ),
        'idle': StatePayload(
            robot_id=ROBOT,
            state='idle',
            since=1760356803410,
            ha_link='up',
            session=SESSION,
            instance=INSTANCE,
            seq=43,
            ts=1760356803412,
        ),
        'busy_paused': StatePayload(
            robot_id=ROBOT,
            state='busy',
            task_id=OTHER_ID,
            task='wait',
            since=1760356801000,
            paused=True,
            ha_link='up',
            session=SESSION,
            instance=INSTANCE,
            seq=45,
            ts=1760356805000,
        ),
        'error_after_timeout': StatePayload(
            robot_id=ROBOT,
            state='error',
            since=1760356870000,
            inhibited=True,
            ha_link='down',
            watchdog_mode='inhibit_new',
            executor_health=ExecutorHealth('degraded', 'start() took 0.35 s'),
            error=RobotError(
                'timeout', "Task 'hang' did not finish within 10 s and ignored cancel"
            ),
            session=SESSION,
            instance=INSTANCE,
            seq=44,
            ts=1760356870002,
        ),
    }
    assert set(cases) == set(golden), 'a golden state example has no mirror in this test'
    for name, state in cases.items():
        payload = state.to_payload()
        assert payload == golden[name].payload, name
        schema_validator.assert_valid('state', payload)


def test_heartbeat_payloads_match_golden_examples(
    valid_examples: Any, schema_validator: Any
) -> None:
    golden = valid_examples('heartbeat_robot')
    steady_counters = Counters(accepted=12, rejected=1)
    cases = {
        'steady': HeartbeatPayload(
            robot_id=ROBOT,
            session=SESSION,
            instance=INSTANCE,
            seq=1203,
            ts=1760356804000,
            uptime_s=2406,
            ha_clock_offset_ms=12,
            counters=steady_counters,
        ),
        'before_first_ha_heartbeat': HeartbeatPayload(
            robot_id=ROBOT,
            session=SESSION,
            instance=INSTANCE,
            seq=0,
            ts=1760356800000,
            uptime_s=0.5,
            counters=Counters(),
        ),
    }
    assert set(cases) == set(golden), 'a golden heartbeat example has no mirror in this test'
    for name, beat in cases.items():
        payload = beat.to_payload()
        assert payload == golden[name].payload, name
        schema_validator.assert_valid('heartbeat_robot', payload)
    steady_counters.takeovers += 1
    assert cases['steady'].to_payload()['counters']['takeovers'] == 1


def test_feedback_payloads_match_golden_examples(
    valid_examples: Any, schema_validator: Any
) -> None:
    golden = valid_examples('feedback')
    cases = {
        'executing_waypoint': FeedbackPayload(
            id=TASK_ID,
            robot_id=ROBOT,
            progress=0.4,
            stage='executing',
            message='waypoint 2 of 5',
            session=SESSION,
            ts=1760356802100,
        ),
        'progress_unknown': FeedbackPayload(
            id=OTHER_ID, robot_id=ROBOT, session=SESSION, ts=1760356802600
        ),
    }
    assert set(cases) == set(golden), 'a golden feedback example has no mirror in this test'
    for name, feedback in cases.items():
        payload = feedback.to_payload()
        assert payload == golden[name].payload, name
        schema_validator.assert_valid('feedback', payload)


def _full_catalogue_tasks(pose_enum: list[str] | None = None) -> list[TaskSpec]:
    pose: dict[str, Any] = {'type': 'string'}
    if pose_enum is not None:
        pose['enum'] = pose_enum
    return [
        TaskSpec('reset', 'Reset to home', 'motion', 60),
        TaskSpec('stop', 'Stop', 'control', 5),
        TaskSpec('pause', 'Pause', 'control', 5),
        TaskSpec('resume', 'Resume', 'control', 5),
        TaskSpec(
            'move_to',
            'Move to named pose',
            'motion',
            120,
            {'type': 'object', 'properties': {'pose': pose}, 'required': ['pose']},
        ),
    ]


SEVEN_JOINTS = ('j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'j7')
SEVEN_LIMITS = (
    (-170.0, 170.0),
    (-170.0, 170.0),
    (-170.0, 170.0),
    (-120.0, 120.0),
    (-170.0, 170.0),
    (-120.0, 120.0),
    (-170.0, 170.0),
)


def test_announce_matches_golden_examples(valid_examples: Any, schema_validator: Any) -> None:
    golden = valid_examples('announce')
    full = Announce(
        robot_id=ROBOT,
        name='Left arm',
        manufacturer='Example Robotics',
        model='UR3e',
        sw_version='driver 2.4.1',
        suggested_area='Kitchen',
        bridge_version='0.1.0',
        executor=ExecutorInfo('fake', '0.1.0'),
        session=SESSION,
        instance=INSTANCE,
        tasks=_full_catalogue_tasks(),
        ts=1760356800000,
    )
    assert full.to_payload() == golden['full_catalogue'].payload
    with_joints = Announce(
        robot_id='fake_arm_1',
        name='Fake arm 1',
        model='fake seven-joint arm',
        suggested_area='Workshop',
        bridge_version='0.2.0',
        executor=ExecutorInfo('fake', '0.2.0'),
        session=SESSION,
        instance=INSTANCE,
        tasks=_full_catalogue_tasks(['home', 'pick', 'place', 'wave']),
        joints=AnnounceJoints.from_joint_info(
            JointInfo(names=SEVEN_JOINTS, unit='deg', limits=SEVEN_LIMITS), rate_hz=2.0
        ),
        ts=1760356800000,
    )
    assert with_joints.to_payload() == golden['with_joints'].payload
    assert list(with_joints.to_payload())[-2:] == ['joints', 'ts'], 'joints sits before ts'
    assert 'joints' not in full.to_payload(), 'a robot without joints omits the key'
    minimal = Announce(
        robot_id='arm_right',
        name='Right arm',
        bridge_version='0.1.0',
        session='3e7a1c9b-5d2f-4b8e-a1c3-6f0d2e4a8b7c',
        instance='robotpc:4124',
        capabilities=Capabilities(cancel=False, feedback=False, hmac_required=True),
        tasks=[TaskSpec('reset', 'Reset to home', 'motion', 60)],
        ts=1760356800000,
    )
    assert (
        _without_null_identity(minimal.to_payload()) == golden['minimal_required_fields'].payload
    )
    grip_params = {
        'type': 'object',
        'properties': {
            'object_class': {'type': 'string', 'enum': ['mug', 'plate', 'bottle']},
            'force_n': {
                'type': 'number',
                'minimum': 0,
                'maximum': 40,
                'description': 'Grip force in newtons',
            },
            'retries': {'type': 'integer', 'minimum': 0, 'maximum': 3},
            'gentle': {'type': 'boolean'},
        },
        'required': ['object_class'],
    }
    gripper = Announce(
        robot_id='gripper_1',
        name='Bench gripper',
        bridge_version='0.1.0',
        session='b1c2d3e4-f5a6-4b7c-8d9e-0f1a2b3c4d5e',
        instance='bench-pc.local:988',
        capabilities=Capabilities(cancel=False, feedback=False, hmac_required=True),
        watchdog_mode='inhibit_new',
        heartbeat_interval_s=0,
        executor=ExecutorInfo('custom', '1.2.0'),
        tasks=[
            TaskSpec(
                'grip',
                'Grip',
                'motion',
                15.5,
                grip_params,
                'Close the gripper on the named object class',
            ),
            TaskSpec('status', 'Report status', 'query', 2),
        ],
        ts=1760356800000,
    )
    assert (
        _without_null_identity(gripper.to_payload())
        == golden['params_with_enum_and_range'].payload
    )
    assert set(golden) == {
        'full_catalogue',
        'minimal_required_fields',
        'params_with_enum_and_range',
        'with_joints',
    }
    for announce in (full, minimal, gripper, with_joints):
        payload = announce.to_payload()
        schema_validator.assert_valid('announce', payload)
        assert len(protocol.encode_payload(payload)) <= protocol.MAX_ANNOUNCE_BYTES
        for spec in announce.tasks:
            assert protocol.validate_task_spec(spec) == []
    assert Announce.__dataclass_fields__['bridge_version'].default == protocol.__version__


def test_joints_payloads_match_golden_examples(valid_examples: Any, schema_validator: Any) -> None:
    golden = valid_examples('joints')
    cases = {
        'seven_joints_deg': JointsPayload(
            robot_id='fake_arm_1',
            positions=[12.5, -34.0, 90.25, -45.5, 0.0, 60.75, -10.0],
            velocities=None,
            session=SESSION,
            instance=INSTANCE,
            seq=7,
            ts=1760356803500,
        ),
        'with_velocities_rad': JointsPayload(
            robot_id=ROBOT,
            positions=(0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0),
            velocities=(0.0, 0.0, 0.1234, 0.0, -0.05, 0.0),
            session='3e7a1c9b-5d2f-4b8e-a1c3-6f0d2e4a8b7c',
            instance='robotpc:4124',
            seq=0,
            ts=1760356801000,
        ),
    }
    assert set(cases) == set(golden), 'a golden joints example has no mirror in this test'
    for name, joints in cases.items():
        payload = joints.to_payload()
        assert payload == golden[name].payload, name
        schema_validator.assert_valid('joints', payload)
        assert len(protocol.encode_payload(payload)) <= MAX_PAYLOAD_BYTES
        assert all(isinstance(value, float) for value in payload['positions'])
    schema_validator.assert_invalid('joints', dict(cases['seven_joints_deg'].to_payload(), seq=-1))
    schema_validator.assert_invalid(
        'joints', dict(cases['seven_joints_deg'].to_payload(), positions=[])
    )
    schema_validator.assert_invalid(
        'joints', dict(cases['seven_joints_deg'].to_payload(), positions=[0.0] * 33)
    )


def test_announce_joints_block_follows_the_schema(schema_validator: Any) -> None:
    info = JointInfo(names=SEVEN_JOINTS, unit='deg', limits=SEVEN_LIMITS)
    block = AnnounceJoints.from_joint_info(info, 2)
    assert block.to_payload() == {
        'unit': 'deg',
        'names': list(SEVEN_JOINTS),
        'limits': [list(pair) for pair in SEVEN_LIMITS],
        'rate_hz': 2.0,
    }
    no_limits = AnnounceJoints.from_joint_info(JointInfo(('a', 'b'), 'rad'), 0.5)
    assert no_limits.to_payload() == {
        'unit': 'rad',
        'names': ['a', 'b'],
        'limits': None,
        'rate_hz': 0.5,
    }

    def announce(block: AnnounceJoints) -> dict[str, Any]:
        return Announce(
            robot_id=ROBOT,
            name='x',
            session=SESSION,
            instance=INSTANCE,
            tasks=[],
            joints=block,
            ts=0,
        ).to_payload()

    schema_validator.assert_valid('announce', announce(block))
    schema_validator.assert_valid('announce', announce(no_limits))
    schema_validator.assert_invalid(
        'announce', announce(AnnounceJoints('deg', SEVEN_JOINTS, None, 0))
    )
    schema_validator.assert_invalid('announce', announce(AnnounceJoints('deg', (), None, 2.0)))
    schema_validator.assert_invalid(
        'announce',
        announce(AnnounceJoints('turns', SEVEN_JOINTS, None, 2.0)),  # type: ignore[arg-type]
    )
    schema_validator.assert_invalid(
        'announce', announce(AnnounceJoints('deg', ('J1',), None, 2.0))
    )


@pytest.mark.parametrize(
    ('info', 'fragment'),
    [
        (JointInfo(SEVEN_JOINTS, 'deg', SEVEN_LIMITS), None),
        (JointInfo(('j1',), 'rad', None), None),
        (JointInfo(['shoulder_pan_joint', 'elbow_joint'], 'rad', [(-3.14, 3.14), (0, 2)]), None),
        (JointInfo((), 'deg', None), 'between 1 and 32'),
        (JointInfo(tuple(f'j{i}' for i in range(33)), 'deg', None), 'between 1 and 32'),
        (JointInfo(('J1',), 'deg', None), 'must match'),
        (JointInfo(('1j',), 'deg', None), 'must match'),
        (JointInfo(('j1', 'j1'), 'deg', None), 'distinct'),
        (JointInfo(('j1',), 'turns', None), 'unit'),  # type: ignore[arg-type]
        (JointInfo(('j1', 'j2'), 'deg', ((-1, 1),)), 'one pair per name'),
        (JointInfo(('j1',), 'deg', ((1, -1),)), 'above maximum'),
        (JointInfo(('j1',), 'deg', ((-1, 'x'),)), 'pair of finite numbers'),  # type: ignore[list-item]
        (JointInfo(('j1',), 'deg', ((-1,),)), 'pair of finite numbers'),  # type: ignore[list-item]
        (JointInfo(('j1',), 'deg', ((-1, float('nan')),)), 'pair of finite numbers'),
        (JointInfo('j1', 'deg', None), 'list of strings'),
        (JointInfo(('j1',), 'deg', 'wide'), 'null or a list'),  # type: ignore[arg-type]
    ],
)
def test_validate_joint_info(info: JointInfo, fragment: str | None) -> None:
    errors = protocol.validate_joint_info(info)
    if fragment is None:
        assert errors == []
        assert info.count == len(info.names)
    else:
        assert errors and fragment in errors[0], errors


# --------------------------------------------------------------------------- topics


def test_topic_builders_follow_the_topic_table() -> None:
    assert protocol.announce_topic(DEFAULT_PREFIX, ROBOT) == 'ros2_bridge/arm_left/announce'
    assert (
        protocol.availability_topic(DEFAULT_PREFIX, ROBOT) == 'ros2_bridge/arm_left/availability'
    )
    assert protocol.state_topic(DEFAULT_PREFIX, ROBOT) == 'ros2_bridge/arm_left/state'
    assert protocol.heartbeat_topic(DEFAULT_PREFIX, ROBOT) == 'ros2_bridge/arm_left/heartbeat'
    assert protocol.joints_topic(DEFAULT_PREFIX, ROBOT) == 'ros2_bridge/arm_left/joints'
    assert protocol.cmd_topic(DEFAULT_PREFIX, ROBOT) == 'ros2_bridge/arm_left/cmd'
    assert protocol.ack_topic('lab', ROBOT, TASK_ID) == f'lab/arm_left/task/{TASK_ID}/ack'
    assert (
        protocol.feedback_topic('lab', ROBOT, TASK_ID) == f'lab/arm_left/task/{TASK_ID}/feedback'
    )
    assert protocol.result_topic('lab', ROBOT, TASK_ID) == f'lab/arm_left/task/{TASK_ID}/result'
    assert protocol.ha_heartbeat_topic('site/a') == 'site/a/ha/heartbeat'
    assert (
        protocol.ha_sensor_topic(DEFAULT_PREFIX, 'binary_sensor', 'kitchen_motion')
        == 'ros2_bridge/ha/sensors/binary_sensor/kitchen_motion'
    )
    topics = RobotTopics(ROBOT)
    assert topics.prefix == DEFAULT_PREFIX
    assert topics.announce == 'ros2_bridge/arm_left/announce'
    assert topics.availability == 'ros2_bridge/arm_left/availability'
    assert topics.state == 'ros2_bridge/arm_left/state'
    assert topics.heartbeat == 'ros2_bridge/arm_left/heartbeat'
    assert topics.joints == 'ros2_bridge/arm_left/joints'
    assert topics.cmd == 'ros2_bridge/arm_left/cmd'
    assert topics.ha_heartbeat == 'ros2_bridge/ha/heartbeat'
    assert topics.ack(TASK_ID) == f'ros2_bridge/arm_left/task/{TASK_ID}/ack'
    assert topics.feedback(TASK_ID) == f'ros2_bridge/arm_left/task/{TASK_ID}/feedback'
    assert topics.result(TASK_ID) == f'ros2_bridge/arm_left/task/{TASK_ID}/result'
    for topic in (topics.announce, topics.cmd, topics.ack(TASK_ID), topics.ha_heartbeat):
        assert protocol.validate_publish_topic(topic) == topic
    assert protocol.DEFAULT_HA_STATUS_TOPIC == 'homeassistant/status'


@pytest.mark.parametrize(
    'prefix',
    ['ros2_bridge', 'lab', 'site/a/robots', 'home_assistant', 'haha', 'robots/ha', 'x' * 100],
)
def test_good_prefixes_are_accepted(prefix: str) -> None:
    assert protocol.is_valid_prefix(prefix)
    assert protocol.prefix_error(prefix) is None
    assert protocol.validate_prefix(prefix) == prefix
    assert protocol.cmd_topic(prefix, ROBOT) == f'{prefix}/arm_left/cmd'
    assert RobotTopics(ROBOT, prefix).prefix == prefix


@pytest.mark.parametrize(
    'prefix',
    [
        '',
        '/x',
        'x/',
        '/',
        '+',
        'a/+',
        '#',
        'a/#',
        'a+b',
        'a//b',
        'homeassistant',
        'homeassistant/bridge',
        'ha',
        'ha/x',
        'a\x00b',
        None,
        7,
    ],
)
def test_bad_prefixes_are_refused(prefix: Any) -> None:
    assert not protocol.is_valid_prefix(prefix)
    assert protocol.prefix_error(prefix)
    with pytest.raises(ValueError):
        protocol.validate_prefix(prefix)
    with pytest.raises(ValueError):
        protocol.announce_topic(prefix, ROBOT)
    with pytest.raises(ValueError):
        protocol.ha_heartbeat_topic(prefix)
    with pytest.raises(ValueError):
        RobotTopics(ROBOT, prefix)


@pytest.mark.parametrize(
    'robot_id',
    ['ha', 'Arm', 'arm left', '1arm', '', 'a' * 33, 'arm-left', 'arm/left', 'arm+', 'arm#'],
)
def test_bad_robot_ids_are_refused(robot_id: str) -> None:
    assert not protocol.is_valid_robot_id(robot_id)
    with pytest.raises(ValueError):
        protocol.validate_robot_id(robot_id)
    with pytest.raises(ValueError):
        protocol.state_topic(DEFAULT_PREFIX, robot_id)
    with pytest.raises(ValueError):
        protocol.joints_topic(DEFAULT_PREFIX, robot_id)
    with pytest.raises(ValueError):
        RobotTopics(robot_id)
    assert protocol.is_valid_robot_id('a' * 32)
    assert protocol.is_valid_robot_id('arm_left')
    assert protocol.is_valid_task_name('ha'), (
        'ha is reserved as a robot id only, not as a task name'
    )


@pytest.mark.parametrize(
    'task_id',
    [
        TASK_ID.upper(),
        '6a0c2b58-1d7e-1a1f-9b2a-3c4d5e6f7a8b',
        '6a0c2b58-1d7e-4a1f-7b2a-3c4d5e6f7a8b',
        '6a0c2b58/1d7e-4a1f-9b2a-3c4d5e6f7a8b',
        '6a0c2b58-1d7e-4a1f-9b2a-3c4d5e6f7a8+',
        'x/#',
        'cmd-123',
        '',
        TASK_ID + '\n',
        TASK_ID[:-1],
    ],
)
def test_bad_task_ids_are_refused(task_id: str) -> None:
    assert not protocol.is_valid_task_id(task_id)
    with pytest.raises(ValueError):
        protocol.validate_task_id(task_id)
    for builder in (protocol.ack_topic, protocol.feedback_topic, protocol.result_topic):
        with pytest.raises(ValueError):
            builder(DEFAULT_PREFIX, ROBOT, task_id)
    with pytest.raises(ValueError):
        RobotTopics(ROBOT).ack(task_id)


@pytest.mark.parametrize(
    ('domain', 'object_id'),
    [('Binary', 'x'), ('sensor', 'a b'), ('', 'x'), ('sensor', 'a/b'), ('sensor', '+')],
)
def test_bad_sensor_segments_are_refused(domain: str, object_id: str) -> None:
    with pytest.raises(ValueError):
        protocol.ha_sensor_topic(DEFAULT_PREFIX, domain, object_id)


@pytest.mark.parametrize(
    'topic', ['', 'a/+/b', 'a/#', 'a\x00b', pytest.param('x' * 65536, id='over-65535-bytes')]
)
def test_validate_publish_topic_refuses_unsafe_topics(topic: str) -> None:
    with pytest.raises(ValueError):
        protocol.validate_publish_topic(topic)


# --------------------------------------------------------------------------- params subset

GRIP_SCHEMA: dict[str, Any] = {
    'type': 'object',
    'properties': {
        'object_class': {'type': 'string', 'enum': ['mug', 'plate', 'bottle']},
        'force_n': {'type': 'number', 'minimum': 0, 'maximum': 40},
        'retries': {'type': 'integer', 'minimum': 0, 'maximum': 3},
        'gentle': {'type': 'boolean'},
        'flag': {'type': 'integer', 'enum': [0, 1]},
    },
    'required': ['object_class'],
}


@pytest.mark.parametrize(
    ('params', 'fragments'),
    [
        # type: object
        ({'object_class': 'mug'}, []),
        ('mug', ['params must be an object']),
        ([], ['params must be an object']),
        # string
        ({'object_class': 'plate'}, []),
        ({'object_class': 7}, ["'object_class' must be a string"]),
        # number
        ({'object_class': 'mug', 'force_n': 12.5}, []),
        ({'object_class': 'mug', 'force_n': 12}, []),
        ({'object_class': 'mug', 'force_n': '12'}, ["'force_n' must be a number"]),
        ({'object_class': 'mug', 'force_n': True}, ["'force_n' must be a number"]),
        # integer
        ({'object_class': 'mug', 'retries': 2}, []),
        ({'object_class': 'mug', 'retries': 2.0}, []),
        ({'object_class': 'mug', 'retries': 2.5}, ["'retries' must be a integer"]),
        ({'object_class': 'mug', 'retries': True}, ["'retries' must be a integer"]),
        # boolean
        ({'object_class': 'mug', 'gentle': False}, []),
        ({'object_class': 'mug', 'gentle': 1}, ["'gentle' must be a boolean"]),
        ({'object_class': 'mug', 'gentle': 'yes'}, ["'gentle' must be a boolean"]),
        # enum
        ({'object_class': 'bottle'}, []),
        ({'object_class': 'spoon'}, ["'object_class' must be one of"]),
        ({'object_class': 'mug', 'flag': 1}, []),
        ({'object_class': 'mug', 'flag': True}, ["'flag' must be a integer"]),
        ({'object_class': 'mug', 'flag': 2}, ["'flag' must be one of"]),
        # minimum
        ({'object_class': 'mug', 'force_n': 0}, []),
        ({'object_class': 'mug', 'force_n': -0.1}, ["'force_n' must be at least 0"]),
        ({'object_class': 'mug', 'retries': -1}, ["'retries' must be at least 0"]),
        # maximum
        ({'object_class': 'mug', 'force_n': 40}, []),
        ({'object_class': 'mug', 'force_n': 40.5}, ["'force_n' must be at most 40"]),
        ({'object_class': 'mug', 'retries': 4}, ["'retries' must be at most 3"]),
        # required
        ({}, ["'object_class' is required"]),
        ({'force_n': 1}, ["'object_class' is required"]),
        # undeclared parameters are not interpreted (JSON Schema semantics)
        ({'object_class': 'mug', 'colour': 'red'}, []),
        # several problems are all reported
        (
            {'force_n': 99, 'retries': 'two'},
            [
                "'object_class' is required",
                "'force_n' must be at most 40",
                "'retries' must be a integer",
            ],
        ),
    ],
)
def test_validate_params_rules(params: Any, fragments: list[str]) -> None:
    errors = protocol.validate_params(GRIP_SCHEMA, params)
    if not fragments:
        assert errors == []
    else:
        assert len(errors) == len(fragments), errors
        for fragment, error in zip(fragments, errors, strict=True):
            assert fragment in error


def test_validate_params_with_no_schema_accepts_any_object() -> None:
    assert protocol.validate_params(None, {}) == []
    assert protocol.validate_params(None, {'anything': 1}) == []
    assert protocol.validate_params(None, 'x') == ['params must be an object']
    assert protocol.validate_params({'type': 'object', 'properties': {}}, {}) == []


@pytest.mark.parametrize(
    ('schema', 'fragments'),
    [
        (GRIP_SCHEMA, []),
        ({'type': 'object', 'properties': {}}, []),
        ({'type': 'object', 'properties': {'x': {'type': 'string', 'description': 'd'}}}, []),
        ('object', ['params schema must be an object']),
        ({'type': 'array', 'properties': {}}, ["'type' must be 'object'"]),
        ({'type': 'object'}, ["'properties' must be an object"]),
        ({'type': 'object', 'properties': []}, ["'properties' must be an object"]),
        ({'type': 'object', 'properties': {'x': 'string'}}, ["property 'x' must be an object"]),
        (
            {'type': 'object', 'properties': {'x': {'type': 'array'}}},
            ["property 'x' type must be one of"],
        ),
        ({'type': 'object', 'properties': {'x': {}}}, ["property 'x' type must be one of"]),
        (
            {'type': 'object', 'properties': {'x': {'type': 'string', 'enum': []}}},
            ["property 'x' enum"],
        ),
        (
            {'type': 'object', 'properties': {'x': {'type': 'string', 'enum': [None]}}},
            ["property 'x' enum"],
        ),
        (
            {'type': 'object', 'properties': {'x': {'type': 'number', 'minimum': '0'}}},
            ["property 'x' minimum"],
        ),
        (
            {'type': 'object', 'properties': {'x': {'type': 'number', 'maximum': True}}},
            ["property 'x' maximum"],
        ),
        (
            {'type': 'object', 'properties': {'x': {'type': 'number', 'description': 1}}},
            ["property 'x' description"],
        ),
        (
            {'type': 'object', 'properties': {}, 'required': 'x'},
            ["'required' must be a list of strings"],
        ),
        (
            {'type': 'object', 'properties': {}, 'required': [1]},
            ["'required' must be a list of strings"],
        ),
        (
            {'type': 'object', 'properties': {}, 'required': ['x', 'x']},
            ["'required' must not repeat"],
        ),
    ],
)
def test_validate_params_schema_rules(schema: Any, fragments: list[str]) -> None:
    errors = protocol.validate_params_schema(schema)
    if not fragments:
        assert errors == []
    else:
        assert len(errors) == len(fragments), errors
        for fragment, error in zip(fragments, errors, strict=True):
            assert fragment in error


def test_validate_params_schema_agrees_with_the_announce_schema(schema_validator: Any) -> None:
    announce = Announce(
        robot_id=ROBOT,
        name='x',
        session=SESSION,
        instance=INSTANCE,
        tasks=[TaskSpec('grip', 'Grip', 'motion', 10, GRIP_SCHEMA)],
        ts=0,
    )
    schema_validator.assert_valid('announce', announce.to_payload())
    bad = Announce(
        robot_id=ROBOT,
        name='x',
        session=SESSION,
        instance=INSTANCE,
        tasks=[
            TaskSpec(
                'grip',
                'Grip',
                'motion',
                10,
                {'type': 'object', 'properties': {'x': {'type': 'array'}}},
            )
        ],
        ts=0,
    )
    schema_validator.assert_invalid('announce', bad.to_payload())
    assert protocol.validate_task_spec(bad.tasks[0])


@pytest.mark.parametrize(
    ('spec', 'fragment'),
    [
        (TaskSpec('reset', 'Reset'), None),
        (TaskSpec('status', 'Status', 'query', 2, None, 'Reports status'), None),
        (TaskSpec('Reset', 'Reset'), 'task name'),
        (TaskSpec('move to', 'Move'), 'task name'),
        (TaskSpec('reset', ''), 'label'),
        (TaskSpec('reset', 'Reset', 'dance'), 'kind'),  # type: ignore[arg-type]
        (TaskSpec('reset', 'Reset', 'motion', 0), 'timeout_s'),
        (TaskSpec('reset', 'Reset', 'motion', -5), 'timeout_s'),
        (TaskSpec('reset', 'Reset', 'motion', 5, {'type': 'object'}), "'properties'"),
    ],
)
def test_validate_task_spec(spec: TaskSpec, fragment: str | None) -> None:
    errors = protocol.validate_task_spec(spec)
    if fragment is None:
        assert errors == []
    else:
        assert errors and fragment in errors[0], errors


# --------------------------------------------------------------------------- freshness


@pytest.mark.parametrize(
    ('now', 'issued_at', 'max_age_ms', 'verdict', 'skew'),
    [
        (10_000, 10_000, 5000, 'fresh', 0),
        (15_000, 10_000, 5000, 'fresh', 5000),
        (17_000, 10_000, 5000, 'fresh', 7000),
        (17_001, 10_000, 5000, 'stale', 7001),
        (8_000, 10_000, 5000, 'fresh', -2000),
        (7_999, 10_000, 5000, 'future', -2001),
        (42_000, 10_000, 60_000, 'fresh', 32_000),
        (42_001, 10_000, 60_000, 'stale', 32_001),
    ],
)
def test_check_freshness_window(
    now: int, issued_at: int, max_age_ms: int, verdict: str, skew: int
) -> None:
    result = protocol.check_freshness(now, issued_at, max_age_ms)
    assert result.verdict == verdict
    assert result.clock_skew_ms == skew
    assert result.window_ms == min(max_age_ms, protocol.DEFAULT_MAX_AGE_CAP_MS)
    expected_code = {'fresh': None, 'stale': 'rejected_stale', 'future': 'rejected_future'}[
        verdict
    ]
    assert result.ack_code == expected_code


def test_check_freshness_honours_custom_cap_and_skew() -> None:
    assert (
        protocol.check_freshness(1600, 1000, 5000, max_age_cap_ms=500, skew_ms=100).verdict
        == 'fresh'
    )
    assert (
        protocol.check_freshness(1601, 1000, 5000, max_age_cap_ms=500, skew_ms=100).verdict
        == 'stale'
    )
    assert (
        protocol.check_freshness(900, 1000, 5000, max_age_cap_ms=500, skew_ms=100).verdict
        == 'fresh'
    )
    assert (
        protocol.check_freshness(899, 1000, 5000, max_age_cap_ms=500, skew_ms=100).verdict
        == 'future'
    )


# --------------------------------------------------------------------------- identities


def test_local_instance_and_new_session_match_their_patterns() -> None:
    instance = protocol.local_instance()
    assert protocol.is_valid_instance(instance)
    assert re.fullmatch(r'[^:]+:[0-9]+', instance)
    session = protocol.new_session()
    assert protocol.is_valid_session(session)
    assert protocol.is_valid_task_id(session)
    assert protocol.new_session() != session
    assert not protocol.is_valid_instance('robotpc')
    assert not protocol.is_valid_instance('robotpc:')
    assert not protocol.is_valid_instance(':4123')


def test_decoded_json_round_trip_is_compact() -> None:
    payload = AckPayload(
        id=TASK_ID,
        robot_id=ROBOT,
        type='task',
        task='move_to',
        code='accepted',
        received_at=1,
        clock_skew_ms=0,
        session=SESSION,
        ts=2,
    ).to_payload()
    raw = protocol.encode_payload(payload)
    assert b' ' not in raw and b'\n' not in raw
    assert json.loads(raw) == payload
