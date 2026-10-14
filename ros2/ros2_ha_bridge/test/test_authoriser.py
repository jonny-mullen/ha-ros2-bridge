# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Tests for :mod:`ros2_ha_bridge.authoriser`.

One test per ack code; ids that are not a UUID, carry a wildcard or are
oversize are dropped with the right counter and no ack; a retained delivery is
dropped and flagged for clearing while the clearing echo is ignored; the
signature is bound to the topic; the freshness window is checked at both
edges; the dedupe store answers a redelivery with the stored bytes even after
the command went stale, and across a simulated restart; the allowlist, the
params subset, the timeout clamp and state gating by kind; and the cancel
pass-through that the task manager finishes. The golden commands from
``contract/examples`` are driven through the whole pipeline.
"""

from __future__ import annotations

import logging
from collections.abc import Iterable
from pathlib import Path
from typing import Any

import pytest

from ros2_ha_bridge import protocol
from ros2_ha_bridge.authoriser import (
    Ack,
    Authoriser,
    AuthoriserConfig,
    Drop,
    GatingState,
    Ignore,
)
from ros2_ha_bridge.dedupe_store import DedupeStore
from ros2_ha_bridge.executors.base import TaskRequest, TaskSpec
from ros2_ha_bridge.protocol import DEFAULT_PREFIX, Counters, RobotError, RobotTopics

from .conftest import VECTOR_SECRET, assert_imports_without
from .fakes import FakeClock, FakeScheduler

ROBOT = 'arm_left'
SESSION = '9f1c7a0e-4a1b-4b2c-8d1e-0f1a2b3c4d5e'
OTHER_SESSION = '3e7a1c9b-5d2f-4b8e-a1c3-6f0d2e4a8b7c'
TASK_ID = '6a0c2b58-1d7e-4a1f-9b2a-3c4d5e6f7a8b'
RESET_ID = '51f0a2b3-c4d5-4e6f-a7b8-c9d0e1f2a3b4'
CANCEL_ID = '0c1d4e6f-2a3b-4c5d-8e9f-0a1b2c3d4e5f'
OTHER_ID = '7b3e9d21-5f6a-4c8b-9e0d-1f2a3b4c5d6e'
SPARE_IDS = (
    'd4e5f6a7-b8c9-4d0e-9f1a-2b3c4d5e6f7a',
    '8c1f2e3d-4a5b-4c6d-8e7f-9a0b1c2d3e4f',
    'e5f6a7b8-c9d0-4e1f-8a2b-3c4d5e6f7a8b',
    '4e6a8c0f-1b3d-4d5f-9a7c-8e0f2a4b6c8d',
    'b1f3c5d7-9e2a-4f6b-8c0d-1e2f3a4b5c6d',
)
#: The golden task command's issued_at and the golden accepted ack's received_at (skew 12 ms).
ISSUED_AT = 1760356800950
RECEIVED_AT = ISSUED_AT + 12
CMD_TOPIC = f'{DEFAULT_PREFIX}/{ROBOT}/cmd'
LOGGER_NAME = 'ros2_ha_bridge.authoriser'
MOVE_TO_PARAMS: dict[str, Any] = {
    'type': 'object',
    'properties': {'pose': {'type': 'string'}},
    'required': ['pose'],
}


def _catalogue() -> list[TaskSpec]:
    """The golden full catalogue plus a query task and a task announced above the timeout cap."""
    return [
        TaskSpec('reset', 'Reset to home', 'motion', 60),
        TaskSpec('stop', 'Stop', 'control', 5),
        TaskSpec('pause', 'Pause', 'control', 5),
        TaskSpec('resume', 'Resume', 'control', 5),
        TaskSpec('move_to', 'Move to named pose', 'motion', 120, MOVE_TO_PARAMS),
        TaskSpec('status', 'Report status', 'query', 2),
        TaskSpec('slow', 'Slow motion', 'motion', 900),
    ]


def _command(**overrides: Any) -> dict[str, Any]:
    """The golden ``task_move_to_unsigned`` command with overrides (``...`` removes a key)."""
    payload: dict[str, Any] = {
        'schema': 1,
        'id': TASK_ID,
        'type': 'task',
        'robot_id': ROBOT,
        'task': 'move_to',
        'params': {'pose': 'home'},
        'issued_at': ISSUED_AT,
        'max_age_ms': 5000,
        'timeout_s': None,
        'source': {'origin': 'ha', 'kind': 'intent'},
    }
    for key, value in overrides.items():
        if value is ...:
            payload.pop(key, None)
        else:
            payload[key] = value
    return payload


def _cancel(**overrides: Any) -> dict[str, Any]:
    payload = _command(
        id=CANCEL_ID, type='cancel', target_id=None, task=..., params=..., timeout_s=...
    )
    payload.update(overrides)
    return payload


def _raw(payload: dict[str, Any]) -> bytes:
    return protocol.encode_payload(payload)


def _signed(payload: dict[str, Any], secret: str = VECTOR_SECRET, topic: str = CMD_TOPIC) -> bytes:
    return _raw(dict(payload, sig=protocol.compute_sig(secret, topic, payload)))


def _warnings(caplog: pytest.LogCaptureFixture) -> list[str]:
    return [r.getMessage() for r in caplog.records if r.levelno >= logging.WARNING]


class Rig:
    """An authoriser with its store, counters and a mutable gating state, for one test."""

    def __init__(
        self,
        clock: FakeClock,
        *,
        robot_id: str = ROBOT,
        prefix: str = DEFAULT_PREFIX,
        session: str = SESSION,
        secret: str = '',
        tasks: Iterable[TaskSpec] | None = None,
        data_dir: Path | None = None,
        scheduler: FakeScheduler | None = None,
        **config: Any,
    ) -> None:
        self.clock = clock
        self.state = GatingState()
        self.state_calls = 0
        self.counters = Counters()
        self.topics = RobotTopics(robot_id, prefix)
        self.store = DedupeStore(clock=clock, data_dir=data_dir, scheduler=scheduler)
        self.config = AuthoriserConfig(secret=secret, **config)
        self.authoriser = Authoriser(
            topics=self.topics,
            session=session,
            clock=clock,
            dedupe=self.store,
            tasks=_catalogue() if tasks is None else tasks,
            config=self.config,
            state_view=self._state_view,
            counters=self.counters,
        )

    def _state_view(self) -> GatingState:
        self.state_calls += 1
        return self.state

    def handle(self, raw: bytes, *, topic: str | None = None, retain: bool = False) -> Any:
        return self.authoriser.handle(
            raw, topic=self.topics.cmd if topic is None else topic, retain=retain
        )

    def ack(self, raw: bytes, **kwargs: Any) -> Ack:
        decision = self.handle(raw, **kwargs)
        assert isinstance(decision, Ack), decision
        return decision

    def drop(self, raw: bytes, **kwargs: Any) -> Drop:
        decision = self.handle(raw, **kwargs)
        assert isinstance(decision, Drop), decision
        return decision

    def counts(self) -> dict[str, int]:
        return {name: value for name, value in self.counters.to_payload().items() if value}


@pytest.fixture
def rig(clock: FakeClock) -> Rig:
    """An idle ``arm_left`` with the full catalogue, HMAC off, clock at the golden received_at."""
    clock.set_now_ms(RECEIVED_AT)
    return Rig(clock)


# --------------------------------------------------------------------------- hygiene


def test_module_imports_neither_rclpy_nor_paho() -> None:
    assert_imports_without('ros2_ha_bridge.authoriser', {'rclpy', 'paho'})


def test_config_defaults_match_the_design_and_are_validated() -> None:
    config = AuthoriserConfig()
    assert config.max_payload_bytes == protocol.MAX_PAYLOAD_BYTES == 16384
    assert config.secret == '' and not config.hmac_required
    assert config.max_age_cap_ms == 30000
    assert config.clock_skew_tolerance_ms == 2000
    assert config.task_timeout_cap_s == 600
    assert config.retained_rejections_per_minute == 30
    assert AuthoriserConfig(secret='s').hmac_required
    bad_cases: list[dict[str, Any]] = [
        {'max_payload_bytes': 0},
        {'max_age_cap_ms': 0},
        {'clock_skew_tolerance_ms': -1},
        {'task_timeout_cap_s': 0},
        {'retained_rejections_per_minute': -1},
        {'warn_interval_s': -1},
    ]
    for bad in bad_cases:
        with pytest.raises(ValueError):
            AuthoriserConfig(**bad)


def test_authoriser_refuses_a_bad_session(clock: FakeClock) -> None:
    with pytest.raises(ValueError):
        Authoriser(
            topics=RobotTopics(ROBOT),
            session='not-a-uuid',
            clock=clock,
            dedupe=DedupeStore(clock=clock),
        )


# --------------------------------------------------------------------------- one test per ack code


def test_accepted_task_matches_the_golden_ack(
    rig: Rig, valid_examples: Any, schema_validator: Any
) -> None:
    golden_cmd = valid_examples('command')['task_move_to_unsigned']
    golden_ack = valid_examples('ack')['accepted_task'].payload
    ack = rig.ack(golden_cmd.raw)
    assert ack.code == 'accepted' and ack.accepted and ack.authorised
    assert not ack.is_duplicate and ack.stored_ack_bytes is None and ack.retain
    assert ack.topic == f'ros2_bridge/arm_left/task/{TASK_ID}/ack'
    assert ack.id == TASK_ID
    assert {k: v for k, v in ack.payload.items() if k != 'ts'} == {
        k: v for k, v in golden_ack.items() if k != 'ts'
    }
    assert ack.payload['ts'] == RECEIVED_AT
    schema_validator.assert_valid('ack', ack.payload)
    assert ack.wire == protocol.encode_payload(ack.payload)
    assert ack.command is not None and ack.command.is_task
    assert ack.spec is not None and ack.spec.name == 'move_to' and ack.spec.kind == 'motion'
    assert ack.request == TaskRequest(
        id=TASK_ID,
        task='move_to',
        params={'pose': 'home'},
        issued_at=ISSUED_AT,
        timeout_s=120.0,
        source=golden_cmd.payload['source'],
    )
    # Nothing is final until the task manager commits: no counter, no dedupe entry.
    assert rig.counts() == {} and TASK_ID not in rig.store
    assert rig.authoriser.commit(ack) is ack
    assert rig.counts() == {'accepted': 1}
    entry = rig.store.lookup(TASK_ID)
    assert entry is not None and entry.accepted and entry.ack_bytes == ack.wire
    assert entry.received_at_ms == RECEIVED_AT


def test_rejected_malformed(rig: Rig, schema_validator: Any) -> None:
    ack = rig.ack(_raw(_command(schema=...)))
    assert ack.code == 'rejected_malformed' and not ack.accepted and not ack.authorised
    assert ack.command is None and ack.request is None
    payload = ack.payload
    assert payload['type'] == 'task' and payload['task'] == 'move_to'
    assert payload['received_at'] == RECEIVED_AT and payload['clock_skew_ms'] == 12
    assert 'schema' in payload['reason']
    schema_validator.assert_valid('ack', payload)
    assert rig.counts() == {'rejected': 1}
    entry = rig.store.lookup(TASK_ID)
    assert entry is not None and not entry.accepted and entry.ack_bytes == ack.wire


def test_rejected_malformed_without_readable_hints(
    rig: Rig, invalid_examples: Any, schema_validator: Any
) -> None:
    """``type`` unreadable gives ``task``; ``issued_at`` unreadable gives a null skew."""
    ack = rig.ack(_raw(_command(type='goal', task='move to', issued_at='soon')))
    assert ack.code == 'rejected_malformed'
    assert ack.payload['type'] == 'task' and ack.payload['task'] is None
    assert ack.payload['clock_skew_ms'] is None
    schema_validator.assert_valid('ack', ack.payload)
    example = invalid_examples('command')['missing_issued_at']
    other = rig.ack(example.raw)
    assert other.code == 'rejected_malformed' and other.payload['clock_skew_ms'] is None
    assert other.id == example.payload['id']


def test_rejected_malformed_when_the_payload_names_another_robot(rig: Rig) -> None:
    ack = rig.ack(_raw(_command(robot_id='arm_right')))
    assert ack.code == 'rejected_malformed'
    assert 'arm_right' in ack.payload['reason'] and ROBOT in ack.payload['reason']


def test_rejected_unknown_task(rig: Rig, schema_validator: Any) -> None:
    ack = rig.ack(_raw(_command(task='dance', params={})))
    assert ack.code == 'rejected_unknown_task'
    assert "'dance'" in ack.payload['reason']
    assert ack.payload['task'] == 'dance' and ack.payload['clock_skew_ms'] == 12
    assert ack.command is not None and ack.spec is None and ack.request is None
    schema_validator.assert_valid('ack', ack.payload)
    assert rig.counts() == {'rejected': 1}
    assert rig.state_calls == 0, 'an unknown task never reaches state gating'


def test_rejected_invalid_params(rig: Rig, schema_validator: Any) -> None:
    missing = rig.ack(_raw(_command(params={})))
    assert missing.code == 'rejected_invalid_params'
    assert "'pose' is required" in missing.payload['reason']
    assert missing.spec is not None and missing.spec.name == 'move_to'
    schema_validator.assert_valid('ack', missing.payload)
    wrong_type = rig.ack(_raw(_command(id=OTHER_ID, params={'pose': 7})))
    assert wrong_type.code == 'rejected_invalid_params'
    assert "'pose' must be a string" in wrong_type.payload['reason']
    assert rig.counts() == {'rejected': 2}
    assert rig.state_calls == 0


def test_rejected_stale(rig: Rig, schema_validator: Any) -> None:
    rig.clock.set_now_ms(ISSUED_AT + 7001)
    ack = rig.ack(_raw(_command()))
    assert ack.code == 'rejected_stale'
    assert ack.payload['clock_skew_ms'] == 7001
    assert ack.payload['received_at'] == ISSUED_AT + 7001
    assert '7001 ms old' in ack.payload['reason'] and '5000 ms' in ack.payload['reason']
    schema_validator.assert_valid('ack', ack.payload)
    assert rig.counts() == {'rejected': 1}


def test_rejected_future(rig: Rig, schema_validator: Any) -> None:
    rig.clock.set_now_ms(ISSUED_AT - 2001)
    ack = rig.ack(_raw(_command()))
    assert ack.code == 'rejected_future'
    assert ack.payload['clock_skew_ms'] == -2001
    assert '2001 ms ahead' in ack.payload['reason']
    schema_validator.assert_valid('ack', ack.payload)
    assert rig.counts() == {'rejected': 1}


def test_rejected_unauthorised(clock: FakeClock, schema_validator: Any) -> None:
    clock.set_now_ms(RECEIVED_AT)
    rig = Rig(clock, secret=VECTOR_SECRET)
    ack = rig.ack(_raw(_command()))
    assert ack.code == 'rejected_unauthorised'
    assert 'sig is missing' in ack.payload['reason']
    assert ack.payload['clock_skew_ms'] == 12
    schema_validator.assert_valid('ack', ack.payload)
    assert rig.counts() == {'rejected': 1}
    assert rig.state_calls == 0


def test_rejected_busy_matches_the_golden_ack(
    rig: Rig, valid_examples: Any, schema_validator: Any
) -> None:
    golden = valid_examples('ack')['rejected_busy'].payload
    rig.state = GatingState(state='busy', task_id=RESET_ID, task='reset')
    ack = rig.ack(valid_examples('command')['task_move_to_unsigned'].raw)
    assert ack.code == 'rejected_busy'
    assert {k: v for k, v in ack.payload.items() if k != 'ts'} == {
        k: v for k, v in golden.items() if k != 'ts'
    }
    schema_validator.assert_valid('ack', ack.payload)
    assert rig.counts() == {'rejected': 1} and rig.state_calls == 1
    rig.state = GatingState(state='busy')
    anonymous = rig.ack(_raw(_command(id=OTHER_ID)))
    assert anonymous.code == 'rejected_busy' and anonymous.payload['reason'] == 'A task is running'


def test_rejected_error(rig: Rig, schema_validator: Any) -> None:
    rig.state = GatingState(state='error', error=RobotError('timeout', "Task 'hang' hung"))
    ack = rig.ack(_raw(_command()))
    assert ack.code == 'rejected_error'
    assert 'timeout' in ack.payload['reason'] and "Task 'hang' hung" in ack.payload['reason']
    schema_validator.assert_valid('ack', ack.payload)
    rig.state = GatingState(state='error')
    bare = rig.ack(_raw(_command(id=OTHER_ID)))
    assert bare.code == 'rejected_error' and bare.payload['reason'] == 'Robot is in error'
    assert rig.counts() == {'rejected': 2}


def test_rejected_inhibited(rig: Rig, schema_validator: Any) -> None:
    rig.state = GatingState(state='idle', inhibited=True)
    ack = rig.ack(_raw(_command()))
    assert ack.code == 'rejected_inhibited'
    assert 'inhibited' in ack.payload['reason']
    schema_validator.assert_valid('ack', ack.payload)
    assert rig.counts() == {'rejected': 1}


def test_rejected_executor_through_reject(rig: Rig, schema_validator: Any) -> None:
    """Step 10 lives in the task manager: it turns an authorised ack into ``rejected_executor``."""
    authorised = rig.ack(_raw(_command()))
    assert authorised.authorised
    rejected = rig.authoriser.reject(authorised, 'rejected_executor', 'driver not ready')
    assert rejected.code == 'rejected_executor' and not rejected.authorised
    assert rejected.payload['reason'] == 'driver not ready'
    for key in ('id', 'robot_id', 'type', 'task', 'received_at', 'clock_skew_ms', 'session'):
        assert rejected.payload[key] == authorised.payload[key], key
    assert rejected.command is authorised.command and rejected.spec is authorised.spec
    assert rejected.request is None
    schema_validator.assert_valid('ack', rejected.payload)
    assert rig.counts() == {'rejected': 1}
    entry = rig.store.lookup(TASK_ID)
    assert entry is not None and not entry.accepted and entry.ack_bytes == rejected.wire
    redelivered = rig.ack(_raw(_command()))
    assert redelivered.is_duplicate and redelivered.stored_ack_bytes == rejected.wire


def test_rejected_no_such_task_through_reject_on_a_cancel(
    rig: Rig, valid_examples: Any, schema_validator: Any
) -> None:
    """Step 11 lives in the task manager: an unresolved cancel target becomes this code."""
    golden = valid_examples('command')['cancel_active_task']
    rig.clock.set_now_ms(golden.payload['issued_at'] + 11)
    authorised = rig.ack(golden.raw)
    assert authorised.authorised and authorised.command is not None
    assert authorised.command.is_cancel and authorised.command.target_id is None
    assert authorised.request is None and authorised.spec is None
    assert authorised.payload['type'] == 'cancel' and authorised.payload['task'] is None
    rejected = rig.authoriser.reject(authorised, 'rejected_no_such_task', 'No task is running')
    assert rejected.code == 'rejected_no_such_task'
    assert rejected.payload['type'] == 'cancel' and rejected.payload['task'] is None
    assert rejected.payload['clock_skew_ms'] == 11
    schema_validator.assert_valid('ack', rejected.payload)
    assert rig.counts() == {'rejected': 1}


# --------------------------------------------------------------------------- cancel pass-through


def test_cancel_passes_through_and_commit_matches_the_golden_ack(
    rig: Rig, valid_examples: Any, schema_validator: Any
) -> None:
    golden_cmd = valid_examples('command')['cancel_active_task']
    golden_ack = valid_examples('ack')['accepted_cancel'].payload
    rig.clock.set_now_ms(golden_cmd.payload['issued_at'] + 11)
    ack = rig.ack(golden_cmd.raw)
    assert ack.authorised and ack.command is not None and ack.command.is_cancel
    assert {k: v for k, v in ack.payload.items() if k != 'ts'} == {
        k: v for k, v in golden_ack.items() if k != 'ts'
    }
    schema_validator.assert_valid('ack', ack.payload)
    assert rig.state_calls == 0, 'a cancel is never gated by robot state'
    rig.authoriser.commit(ack)
    assert rig.counts() == {'accepted': 1}
    entry = rig.store.lookup(CANCEL_ID)
    assert entry is not None and entry.accepted


def test_cancel_keeps_its_target_and_needs_no_catalogue(clock: FakeClock) -> None:
    clock.set_now_ms(RECEIVED_AT)
    rig = Rig(clock, tasks=[])
    ack = rig.ack(_raw(_cancel(target_id=OTHER_ID)))
    assert ack.authorised and ack.command is not None
    assert ack.command.is_cancel and ack.command.target_id == OTHER_ID


# --------------------------------------------------------------------------- drops without an ack


def test_retained_delivery_is_dropped_and_flagged_for_clearing(
    rig: Rig, caplog: pytest.LogCaptureFixture
) -> None:
    raw = _raw(_command())
    with caplog.at_level(logging.WARNING, logger=LOGGER_NAME):
        drop = rig.drop(raw, retain=True)
    assert drop.counter == 'dropped_retained' and drop.clear_retained
    assert drop.size == len(raw)
    assert rig.counts() == {'dropped_retained': 1}
    assert TASK_ID not in rig.store
    assert any('retained' in line for line in _warnings(caplog))
    # The same bytes without the flag are a normal command.
    assert rig.ack(raw).authorised


def test_clearing_echo_is_ignored_silently(rig: Rig, caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.DEBUG, logger=LOGGER_NAME):
        decision = rig.handle(b'', retain=True)
    assert isinstance(decision, Ignore)
    assert rig.counts() == {}
    assert caplog.records == []


def test_empty_payload_without_the_retain_flag_is_dropped(rig: Rig) -> None:
    drop = rig.drop(b'')
    assert drop.counter == 'dropped_invalid_id'
    assert rig.counts() == {'dropped_invalid_id': 1}


@pytest.mark.parametrize(
    'command_id',
    [
        'cmd-123',
        'x/#',
        '+',
        'a/+/b',
        TASK_ID.upper(),
        TASK_ID[:-1],
        '6a0c2b58-1d7e-1a1f-9b2a-3c4d5e6f7a8b',
        '',
        7,
        None,
    ],
)
def test_bad_ids_are_dropped_without_an_ack(rig: Rig, command_id: Any) -> None:
    drop = rig.drop(_raw(_command(id=command_id)))
    assert drop.counter == 'dropped_invalid_id' and not drop.clear_retained
    assert rig.counts() == {'dropped_invalid_id': 1}
    assert len(rig.store) == 0


def test_not_json_is_dropped_as_invalid_id(rig: Rig) -> None:
    for raw in (b'{', b'[]', b'"text"', b'\xff\xfe', b'NaN'):
        drop = rig.drop(raw)
        assert drop.counter == 'dropped_invalid_id'
    assert rig.counts() == {'dropped_invalid_id': 5}


def test_oversize_is_dropped_unparsed(rig: Rig, invalid_examples: Any) -> None:
    example = invalid_examples('command')['oversize_over_16kib']
    drop = rig.drop(example.raw)
    assert drop.counter == 'dropped_oversize' and drop.size == len(example.raw)
    assert rig.counts() == {'dropped_oversize': 1}
    assert example.payload['id'] not in rig.store
    compact = _raw(example.payload)
    assert len(compact) > protocol.MAX_PAYLOAD_BYTES
    assert rig.drop(compact).counter == 'dropped_oversize'


def test_huge_id_is_dropped_by_size_first_then_by_the_id_check(
    clock: FakeClock, invalid_examples: Any
) -> None:
    example = invalid_examples('command')['id_70kb']
    clock.set_now_ms(RECEIVED_AT)
    default = Rig(clock)
    assert default.drop(example.raw).counter == 'dropped_oversize'
    generous = Rig(clock, max_payload_bytes=100_000)
    assert generous.drop(example.raw).counter == 'dropped_invalid_id'
    assert generous.counts() == {'dropped_invalid_id': 1}


def test_drop_warnings_are_rate_limited(rig: Rig, caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.WARNING, logger=LOGGER_NAME):
        for _ in range(5):
            rig.drop(b'[]')
        assert len(_warnings(caplog)) == 1
        assert 'usable id' in _warnings(caplog)[0]
        rig.clock.advance(rig.config.warn_interval_s)
        rig.drop(b'[]')
    warnings = _warnings(caplog)
    assert len(warnings) == 2
    assert '4 similar warnings suppressed' in warnings[1]
    assert rig.counts() == {'dropped_invalid_id': 6}


# --------------------------------------------------------------------------- HMAC


def test_signature_is_bound_to_the_topic(clock: FakeClock, valid_examples: Any) -> None:
    golden = valid_examples('command')['task_move_to_signed']
    clock.set_now_ms(RECEIVED_AT)
    wrong_topic = Rig(clock, secret=VECTOR_SECRET)
    ack = wrong_topic.ack(golden.raw, topic='lab/arm_left/cmd')
    assert ack.code == 'rejected_unauthorised'
    assert 'does not match' in ack.payload['reason']
    right_topic = Rig(clock, secret=VECTOR_SECRET)
    accepted = right_topic.ack(golden.raw)
    assert accepted.authorised
    right_topic.authoriser.commit(accepted)
    assert right_topic.ack(golden.raw, topic=CMD_TOPIC).is_duplicate


def test_signature_made_for_another_topic_is_refused(clock: FakeClock, hmac_vectors: Any) -> None:
    vector = next(v for v in hmac_vectors if v['name'] == 'same_payload_other_topic')
    assert vector['topic'] == 'lab/arm_left/cmd'
    clock.set_now_ms(vector['payload']['issued_at'] + 12)
    rig = Rig(clock, secret=vector['secret'])
    signed = _raw(dict(vector['payload'], sig=vector['sig']))
    assert rig.ack(signed).code == 'rejected_unauthorised'
    lab = Rig(clock, prefix='lab', secret=vector['secret'])
    assert lab.ack(signed).authorised


def test_tampered_payload_and_wrong_secret_are_refused(clock: FakeClock) -> None:
    clock.set_now_ms(RECEIVED_AT)
    rig = Rig(clock, secret=VECTOR_SECRET)
    payload = _command()
    tampered = dict(payload, params={'pose': 'bin'})
    tampered['sig'] = protocol.compute_sig(VECTOR_SECRET, CMD_TOPIC, payload)
    assert rig.ack(_raw(tampered)).code == 'rejected_unauthorised'
    other_secret = Rig(clock, secret='another-example-secret')
    assert other_secret.ack(_signed(_command())).code == 'rejected_unauthorised'
    assert Rig(clock, secret=VECTOR_SECRET).ack(_signed(_command())).authorised


def test_without_a_secret_the_signature_is_ignored(rig: Rig) -> None:
    assert not rig.config.hmac_required
    assert rig.ack(_signed(_command(), secret='wrong-secret')).authorised
    assert rig.ack(_raw(_command(id=OTHER_ID))).authorised
    assert rig.ack(_raw(_command(id=RESET_ID, sig='0' * 64))).authorised


def test_unauthorised_warnings_are_rate_limited(
    clock: FakeClock, caplog: pytest.LogCaptureFixture
) -> None:
    clock.set_now_ms(RECEIVED_AT)
    rig = Rig(clock, secret=VECTOR_SECRET)
    with caplog.at_level(logging.WARNING, logger=LOGGER_NAME):
        for command_id in (TASK_ID, OTHER_ID, RESET_ID):
            rig.ack(_raw(_command(id=command_id)))
    assert len(_warnings(caplog)) == 1
    assert rig.counts() == {'rejected': 3}


def test_unsigned_replay_of_an_accepted_id_republishes_the_stored_ack(clock: FakeClock) -> None:
    """One distinct ack per id: a replay without a signature gets the stored bytes, nothing new.

    Before the dedupe step moved ahead of the signature check, this replay
    produced a second, distinct ``rejected_unauthorised`` ack on the same
    retained topic and so overwrote the ``accepted`` ack on the broker.
    """
    clock.set_now_ms(RECEIVED_AT)
    rig = Rig(clock, secret=VECTOR_SECRET)
    accepted = rig.authoriser.commit(rig.ack(_signed(_command())))
    replay = rig.ack(_raw(_command()))
    assert replay.is_duplicate and replay.code == 'accepted' and replay.retain
    assert replay.stored_ack_bytes == accepted.wire and replay.wire == accepted.wire
    assert replay.topic == accepted.topic
    entry = rig.store.lookup(TASK_ID)
    assert entry is not None and entry.accepted and entry.ack_bytes == accepted.wire
    tampered = rig.ack(_raw(dict(_command(), sig='0' * 64)))
    assert tampered.is_duplicate and tampered.stored_ack_bytes == accepted.wire
    resend = rig.ack(_signed(_command()))
    assert resend.is_duplicate and resend.stored_ack_bytes == accepted.wire
    assert rig.counts() == {'accepted': 1, 'duplicates': 3}


def test_malformed_payload_with_an_accepted_id_republishes_the_stored_ack(
    rig: Rig, caplog: pytest.LogCaptureFixture
) -> None:
    """A broken payload reusing a known id (no secret needed) cannot overwrite the accepted ack."""
    accepted = rig.authoriser.commit(rig.ack(_raw(_command())))
    with caplog.at_level(logging.INFO, logger=LOGGER_NAME):
        for broken in (
            _raw({'id': TASK_ID, 'schema': 2}),
            _raw(_command(robot_id='arm_right')),
            _raw(_command(type='goal', task='move to', issued_at='soon')),
        ):
            replay = rig.ack(broken)
            assert replay.is_duplicate and replay.code == 'accepted'
            assert replay.stored_ack_bytes == accepted.wire and replay.wire == accepted.wire
            assert replay.command is None, 'nothing decodable to hand on'
            assert replay.topic == accepted.topic
    entry = rig.store.lookup(TASK_ID)
    assert entry is not None and entry.accepted and entry.ack_bytes == accepted.wire
    assert rig.counts() == {'accepted': 1, 'duplicates': 3}
    assert not any('rejected command' in r.getMessage() for r in caplog.records)
    # The same broken payloads under a fresh id are rejected_malformed as before.
    fresh = rig.ack(_raw({'id': OTHER_ID, 'schema': 2}))
    assert fresh.code == 'rejected_malformed' and not fresh.is_duplicate


# --------------------------------------------------------------------------- freshness


@pytest.mark.parametrize(
    ('offset_ms', 'code', 'max_age_ms'),
    [
        (7000, 'accepted', 5000),
        (7001, 'rejected_stale', 5000),
        (-2000, 'accepted', 5000),
        (-2001, 'rejected_future', 5000),
        (0, 'accepted', 5000),
        (32000, 'accepted', 60000),
        (32001, 'rejected_stale', 60000),
    ],
)
def test_freshness_window_edges(rig: Rig, offset_ms: int, code: str, max_age_ms: int) -> None:
    rig.clock.set_now_ms(ISSUED_AT + offset_ms)
    ack = rig.ack(_raw(_command(max_age_ms=max_age_ms)))
    assert ack.code == code
    assert ack.payload['clock_skew_ms'] == offset_ms
    assert ack.payload['received_at'] == ISSUED_AT + offset_ms


def test_freshness_honours_the_configured_cap_and_skew(clock: FakeClock) -> None:
    clock.set_now_ms(ISSUED_AT + 600)
    rig = Rig(clock, max_age_cap_ms=500, clock_skew_tolerance_ms=100)
    assert rig.ack(_raw(_command())).authorised
    clock.set_now_ms(ISSUED_AT + 601)
    assert rig.ack(_raw(_command(id=OTHER_ID))).code == 'rejected_stale'
    clock.set_now_ms(ISSUED_AT - 101)
    assert rig.ack(_raw(_command(id=RESET_ID))).code == 'rejected_future'


# --------------------------------------------------------------------------- dedupe


def test_duplicate_republishes_the_stored_ack_even_after_the_command_went_stale(rig: Rig) -> None:
    raw = _raw(_command())
    original = rig.authoriser.commit(rig.ack(raw))
    rig.clock.advance(60)
    again = rig.ack(raw)
    assert again.is_duplicate and not again.authorised
    assert again.code == 'accepted' and again.accepted
    assert again.stored_ack_bytes == original.wire and again.wire == original.wire
    assert again.payload == original.payload
    assert again.topic == original.topic and again.retain
    assert again.command is not None and again.command.id == TASK_ID
    assert rig.counts() == {'accepted': 1, 'duplicates': 1}
    # Proof that dedupe runs before freshness: a different id issued at the same time is stale.
    fresh_id = rig.ack(_raw(_command(id=OTHER_ID)))
    assert fresh_id.code == 'rejected_stale'
    with pytest.raises(ValueError):
        rig.authoriser.commit(again)


def test_rejected_acks_are_deduplicated_too(rig: Rig) -> None:
    raw = _raw(_command(task='dance', params={}))
    first = rig.ack(raw)
    assert first.code == 'rejected_unknown_task'
    rig.clock.advance(1)
    second = rig.ack(raw)
    assert second.is_duplicate and second.code == 'rejected_unknown_task'
    assert second.stored_ack_bytes == first.wire
    assert rig.counts() == {'rejected': 1, 'duplicates': 1}


def test_dedupe_entry_is_forgotten_after_the_window(rig: Rig) -> None:
    raw = _raw(_command())
    rig.authoriser.commit(rig.ack(raw))
    rig.clock.advance(rig.store.window_s + 1)
    assert rig.ack(raw).code == 'rejected_stale'
    assert rig.counts() == {'accepted': 1, 'rejected': 1}


def test_corrupt_dedupe_entry_is_treated_as_unknown(
    rig: Rig, caplog: pytest.LogCaptureFixture
) -> None:
    rig.store.remember(TASK_ID, RECEIVED_AT, b'garbage', accepted=True)
    with caplog.at_level(logging.ERROR, logger=LOGGER_NAME):
        ack = rig.ack(_raw(_command()))
    assert ack.authorised and not ack.is_duplicate
    assert any('no readable ack' in r.getMessage() for r in caplog.records)
    rig.authoriser.commit(ack)
    entry = rig.store.lookup(TASK_ID)
    assert entry is not None and entry.ack_bytes == ack.wire


def test_qos1_resend_across_a_restart_republishes_the_stored_ack(
    clock: FakeClock, tmp_path: Path
) -> None:
    clock.set_now_ms(RECEIVED_AT)
    old_node = Rig(clock, data_dir=tmp_path)
    raw = _raw(_command())
    accepted = old_node.authoriser.commit(old_node.ack(raw))
    assert old_node.store.writes == 1
    del old_node

    clock.advance(3)
    new_node = Rig(clock, data_dir=tmp_path, session=OTHER_SESSION)
    assert new_node.store.loaded == 1
    resend = new_node.ack(raw)
    assert resend.is_duplicate
    assert resend.stored_ack_bytes == accepted.wire
    assert resend.payload['session'] == SESSION, 'the stored ack is republished unchanged'
    assert new_node.counts() == {'duplicates': 1}
    assert new_node.state_calls == 0


# --------------------------------------------------------------------------- allowlist, params, timeout


def test_allowlist_is_the_catalogue_given(clock: FakeClock) -> None:
    clock.set_now_ms(RECEIVED_AT)
    rig = Rig(clock, tasks=[TaskSpec('reset', 'Reset to home', 'motion', 60)])
    assert rig.ack(_raw(_command())).code == 'rejected_unknown_task'
    assert rig.ack(_raw(_command(id=RESET_ID, task='reset', params={}))).authorised
    rig.authoriser.set_tasks(_catalogue())
    assert rig.ack(_raw(_command(id=OTHER_ID))).authorised


@pytest.mark.parametrize(
    ('task', 'timeout_s', 'cap', 'expected'),
    [
        ('move_to', None, 600.0, 120.0),
        ('move_to', 30, 600.0, 30.0),
        ('move_to', 1000, 600.0, 600.0),
        ('slow', None, 600.0, 600.0),
        ('reset', None, 50.0, 50.0),
        ('reset', 10, 50.0, 10.0),
    ],
)
def test_timeout_is_the_announced_value_clamped_to_the_cap(
    clock: FakeClock, task: str, timeout_s: float | None, cap: float, expected: float
) -> None:
    clock.set_now_ms(RECEIVED_AT)
    rig = Rig(clock, task_timeout_cap_s=cap)
    params = {'pose': 'home'} if task == 'move_to' else {}
    ack = rig.ack(_raw(_command(task=task, params=params, timeout_s=timeout_s)))
    assert ack.authorised and ack.request is not None
    assert ack.request.timeout_s == expected
    assert isinstance(ack.request.timeout_s, float)


def test_params_are_validated_against_the_announced_schema(clock: FakeClock) -> None:
    clock.set_now_ms(RECEIVED_AT)
    grip = {
        'type': 'object',
        'properties': {
            'object_class': {'type': 'string', 'enum': ['mug', 'plate']},
            'force_n': {'type': 'number', 'minimum': 0, 'maximum': 40},
        },
        'required': ['object_class'],
    }
    rig = Rig(clock, tasks=[TaskSpec('grip', 'Grip', 'motion', 15, grip)])
    ok = rig.ack(_raw(_command(task='grip', params={'object_class': 'mug', 'force_n': 12})))
    assert ok.authorised and ok.request is not None
    assert ok.request.params == {'object_class': 'mug', 'force_n': 12}
    bad = rig.ack(_raw(_command(id=OTHER_ID, task='grip', params={'object_class': 'x'})))
    assert bad.code == 'rejected_invalid_params' and 'object_class' in bad.payload['reason']
    worse = rig.ack(_raw(_command(id=RESET_ID, task='grip', params={'force_n': 99})))
    assert worse.code == 'rejected_invalid_params'
    assert "'object_class' is required" in worse.payload['reason']
    assert "'force_n' must be at most 40" in worse.payload['reason']


# --------------------------------------------------------------------------- state gating by kind


@pytest.mark.parametrize(
    ('state', 'motion_code'),
    [
        (GatingState(), 'accepted'),
        (GatingState(state='busy', task_id=RESET_ID, task='reset'), 'rejected_busy'),
        (GatingState(state='error', error=RobotError('timeout', 'hung')), 'rejected_error'),
        (GatingState(state='idle', inhibited=True), 'rejected_inhibited'),
        (
            GatingState(state='busy', inhibited=True, task_id=RESET_ID, task='wait'),
            'rejected_busy',
        ),
        (GatingState(state='error', inhibited=True), 'rejected_error'),
        # The task manager reports 'error' while a motion task still runs with an error set;
        # the slot is taken, so this is busy first, as the manager itself answers.
        (
            GatingState(
                state='error', task_id=RESET_ID, task='wait', error=RobotError('timeout', 'hung')
            ),
            'rejected_busy',
        ),
    ],
)
def test_motion_is_gated_while_control_and_query_pass(
    rig: Rig, state: GatingState, motion_code: str
) -> None:
    rig.state = state
    motion = rig.ack(_raw(_command(id=TASK_ID, task='reset', params={})))
    assert motion.code == motion_code
    control = rig.ack(_raw(_command(id=OTHER_ID, task='stop', params={})))
    assert control.authorised and control.spec is not None and control.spec.kind == 'control'
    query = rig.ack(_raw(_command(id=RESET_ID, task='status', params={})))
    assert query.authorised and query.spec is not None and query.spec.kind == 'query'
    assert rig.state_calls == 1, 'only the motion task consulted the state view'


# --------------------------------------------------------------------------- commit and reject


def test_commit_and_reject_guard_against_misuse(rig: Rig) -> None:
    rejected = rig.ack(_raw(_command(task='dance', params={})))
    with pytest.raises(ValueError):
        rig.authoriser.commit(rejected)
    with pytest.raises(ValueError):
        rig.authoriser.reject(rejected, 'rejected_executor', 'x')
    authorised = rig.ack(_raw(_command(id=OTHER_ID)))
    with pytest.raises(ValueError):
        rig.authoriser.reject(authorised, 'accepted', 'x')
    rig.authoriser.commit(authorised)
    with pytest.raises(ValueError):
        rig.authoriser.commit(authorised)
    with pytest.raises(ValueError):
        rig.authoriser.reject(authorised, 'rejected_executor', 'x')
    duplicate = rig.ack(_raw(_command(id=OTHER_ID)))
    assert duplicate.is_duplicate
    with pytest.raises(ValueError):
        rig.authoriser.commit(duplicate)
    assert rig.counts() == {'accepted': 1, 'rejected': 1, 'duplicates': 1}


# --------------------------------------------------------------------------- retained rejections


def test_rejections_beyond_the_per_minute_budget_are_not_retained(clock: FakeClock) -> None:
    clock.set_now_ms(RECEIVED_AT)
    rig = Rig(clock, retained_rejections_per_minute=2)
    flags = [
        rig.ack(_raw(_command(id=command_id, task='dance', params={}))).retain
        for command_id in SPARE_IDS[:3]
    ]
    assert flags == [True, True, False]
    assert rig.counts() == {'rejected': 3}
    assert rig.ack(_raw(_command(id=SPARE_IDS[3]))).retain, 'accepted acks are always retained'
    clock.advance(60)
    late = rig.ack(_raw(_command(id=SPARE_IDS[4], task='dance', params={})))
    assert late.code == 'rejected_stale' and late.retain, 'the budget refills after a minute'
    none_retained = Rig(clock, retained_rejections_per_minute=0)
    assert not none_retained.ack(_raw(_command(id=SPARE_IDS[0], task='dance'))).retain


def test_reject_from_the_task_manager_counts_against_the_budget(clock: FakeClock) -> None:
    clock.set_now_ms(RECEIVED_AT)
    rig = Rig(clock, retained_rejections_per_minute=1)
    first = rig.authoriser.reject(rig.ack(_raw(_command())), 'rejected_executor', 'no')
    second = rig.authoriser.reject(rig.ack(_raw(_command(id=OTHER_ID))), 'rejected_executor', 'no')
    assert first.retain and not second.retain


def test_a_duplicate_is_republished_the_way_the_original_was_retained(clock: FakeClock) -> None:
    """A rejection published non-retained (budget exhausted) stays non-retained on redelivery.

    Otherwise a misfiring automation resending the same id, or a replay of
    rejected ids, would put into the broker's retained store exactly what the
    budget exists to keep out. Accepted acks are always retained.
    """
    clock.set_now_ms(RECEIVED_AT)
    rig = Rig(clock, retained_rejections_per_minute=1)
    retained = rig.ack(_raw(_command(id=SPARE_IDS[0], task='dance', params={})))
    exhausted = rig.ack(_raw(_command(id=SPARE_IDS[1], task='dance', params={})))
    assert retained.retain and not exhausted.retain
    for _ in range(3):
        again = rig.ack(_raw(_command(id=SPARE_IDS[1], task='dance', params={})))
        assert again.is_duplicate and not again.retain
        assert again.stored_ack_bytes == exhausted.wire
    entry = rig.store.lookup(SPARE_IDS[1])
    assert entry is not None and not entry.retain
    clock.advance(60)
    still_not = rig.ack(_raw(_command(id=SPARE_IDS[1], task='dance', params={})))
    assert still_not.is_duplicate and not still_not.retain, 'the budget refilling changes nothing'
    again_retained = rig.ack(_raw(_command(id=SPARE_IDS[0], task='dance', params={})))
    assert again_retained.is_duplicate and again_retained.retain
    fresh = _command(id=SPARE_IDS[2], issued_at=clock.now_ms() - 12)
    accepted = rig.authoriser.commit(rig.ack(_raw(fresh)))
    assert accepted.retain and rig.ack(_raw(fresh)).retain

    none_retained = Rig(clock, retained_rejections_per_minute=0)
    first = none_retained.ack(_raw(_command(id=SPARE_IDS[3], task='dance', params={})))
    duplicate = none_retained.ack(_raw(_command(id=SPARE_IDS[3], task='dance', params={})))
    assert not first.retain and duplicate.is_duplicate and not duplicate.retain


# --------------------------------------------------------------------------- catalogue clamp


def test_announced_timeouts_above_the_cap_are_clamped_in_the_catalogue(clock: FakeClock) -> None:
    """What the node announces is what its timer enforces: HA's deadline never outlives it."""
    clock.set_now_ms(RECEIVED_AT)
    rig = Rig(clock, task_timeout_cap_s=600.0)
    slow = rig.authoriser.tasks['slow']
    assert slow.timeout_s == 600.0 and slow.name == 'slow' and slow.kind == 'motion'
    assert rig.authoriser.tasks['move_to'].timeout_s == 120, 'values within the cap are untouched'
    assert [spec.name for spec in rig.authoriser.catalogue] == [s.name for s in _catalogue()]
    assert max(spec.timeout_s for spec in rig.authoriser.catalogue) == 600.0
    ack = rig.ack(_raw(_command(task='slow', params={})))
    assert ack.authorised and ack.request is not None and ack.spec is not None
    assert ack.request.timeout_s == 600.0 == ack.spec.timeout_s
    tight = Rig(clock, task_timeout_cap_s=50.0)
    assert all(spec.timeout_s <= 50.0 for spec in tight.authoriser.catalogue)
    tight.authoriser.set_tasks([TaskSpec('quick', 'Quick', 'query', 2)])
    assert tight.authoriser.catalogue == (TaskSpec('quick', 'Quick', 'query', 2),)


# --------------------------------------------------------------------------- golden commands


def test_valid_command_example_is_authorised(
    valid_command_example: Any, clock: FakeClock, schema_validator: Any
) -> None:
    payload = valid_command_example.payload
    clock.set_now_ms(payload['issued_at'] + 12)
    rig = Rig(
        clock, robot_id=payload['robot_id'], secret=VECTOR_SECRET if 'sig' in payload else ''
    )
    ack = rig.ack(valid_command_example.raw)
    assert ack.authorised, ack.payload
    assert ack.id == payload['id'] and ack.payload['type'] == payload['type']
    assert ack.payload['clock_skew_ms'] == 12
    assert ack.topic == rig.topics.ack(payload['id'])
    schema_validator.assert_valid('ack', ack.payload)
    assert ack.command is not None and ack.command.payload == payload
    if payload['type'] == 'task':
        assert ack.request is not None and ack.spec is not None
        assert ack.request.task == payload['task'] and ack.request.params == payload['params']
        expected = payload.get('timeout_s')
        assert ack.request.timeout_s == (ack.spec.timeout_s if expected is None else expected)
    else:
        assert ack.request is None and ack.command.target_id == payload['target_id']
    rig.authoriser.commit(ack)
    again = rig.ack(valid_command_example.raw)
    assert again.is_duplicate and again.stored_ack_bytes == ack.wire
    assert rig.counts() == {'accepted': 1, 'duplicates': 1}


def test_invalid_command_example_is_dropped_or_rejected_malformed(
    invalid_command_example: Any, rig: Rig, schema_validator: Any
) -> None:
    name = invalid_command_example.name
    decision = rig.handle(invalid_command_example.raw)
    if name.startswith('oversize') or name == 'id_70kb':
        assert isinstance(decision, Drop) and decision.counter == 'dropped_oversize'
        assert rig.counts() == {'dropped_oversize': 1}
    elif name.startswith('id_'):
        assert isinstance(decision, Drop) and decision.counter == 'dropped_invalid_id'
        assert rig.counts() == {'dropped_invalid_id': 1}
    else:
        assert isinstance(decision, Ack), decision
        assert decision.code == 'rejected_malformed'
        assert decision.id == invalid_command_example.payload['id']
        schema_validator.assert_valid('ack', decision.payload)
        assert rig.counts() == {'rejected': 1}
    assert rig.state_calls == 0
