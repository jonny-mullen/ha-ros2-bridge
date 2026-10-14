# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Command authorisation: steps 0 to 9 of the contract's command handling, as a pure class.

Every message delivered on ``{prefix}/{robot_id}/cmd`` goes through
:meth:`Authoriser.handle`, which applies the contract's checks in order and
stops at the first failure:

0. a retained delivery (seen live thanks to Retain As Published, or replayed
   at reconnect) is dropped and flagged for clearing; the node's own clearing
   echo, an empty retained payload, is ignored silently;
1. an oversize payload is dropped unparsed;
2. a payload whose ``id`` is not a lower-case UUID4 (or is not JSON at all) is
   dropped, because it has no safe reply topic;
6. a known id republishes the stored ack bytes, unchanged, from the
   :class:`~ros2_ha_bridge.dedupe_store.DedupeStore`. This runs the moment the
   id is known, before the schema, signature and freshness checks, because the
   contract promises one distinct ack per id: if any later check could still
   produce a different ack for a known id, an unsigned or deliberately broken
   replay of an accepted command would overwrite the retained ``accepted`` ack
   on the broker, and a redelivery could downgrade it to ``rejected_stale``.
   Republishing the stored bytes gives a replayer nothing it could not already
   read from the retained topic;
3. and 4. a payload that fails the schema, or names another robot, is answered
   ``rejected_malformed``;
5. with a shared secret configured, a missing or wrong signature over the
   delivery topic and the canonical payload is ``rejected_unauthorised``;
7. the freshness window gives ``rejected_stale`` or ``rejected_future`` with
   the measured ``clock_skew_ms``;
8. a task command must name an announced task (``rejected_unknown_task``)
   with valid parameters (``rejected_invalid_params``), and its ``timeout_s`` is
   resolved by :func:`resolve_request` (the announced value when absent, then
   clamped to ``command.task_timeout_cap_s``); the announced catalogue itself
   is clamped to the cap by :meth:`Authoriser.set_tasks`, so the value Home
   Assistant plans its deadline on is the value the node's timer uses;
9. a ``motion`` task is refused while the robot is busy, in error or inhibited
   (:func:`gate_motion`, the one implementation of this rule that the task
   manager's ``motion_rejection`` uses as well); ``control`` and ``query`` tasks
   pass in any state.

The result is a :data:`Decision`: :class:`Ignore`, :class:`Drop` (a counter to
increment and whether to clear the retained message; no ack is possible) or
:class:`Ack` (what to publish). Steps 10 and 11 belong to the task manager: an
:class:`Ack` that is :attr:`Ack.authorised` has passed every check here but is
not yet final. The task manager calls ``executor.start`` with
:attr:`Ack.request` (or resolves a cancel's target) and then exactly one of
:meth:`Authoriser.commit`, which records the accepted ack in the dedupe store
(memory and file), or :meth:`Authoriser.reject`, which turns it into
``rejected_executor`` or ``rejected_no_such_task``. Rejections decided here are
recorded at once.

Counters are the shared :class:`~ros2_ha_bridge.protocol.Counters` the node
reports in its heartbeat. Rejected acks beyond
``reply.retained_rejections_per_minute`` are flagged ``retain=False`` so a
misfiring automation cannot fill the broker's retained store; the publisher
honours the flag and the dedupe store remembers it, so a redelivery of such a
rejection is republished non-retained too. ``now`` for the freshness rule and
``received_at`` are taken when the command is processed on the main thread,
not when the network thread received it, so a stalled node never acts on an
intent that aged in its own queue.

This module imports neither rclpy nor paho; the clock is injected.
"""

from __future__ import annotations

import json
import logging
from collections import deque
from collections.abc import Callable, Iterable
from dataclasses import dataclass, replace
from typing import Any, Literal

from ros2_ha_bridge.clock import Clock
from ros2_ha_bridge.dedupe_store import DedupeEntry, DedupeStore
from ros2_ha_bridge.executors.base import TaskRequest, TaskSpec
from ros2_ha_bridge.protocol import (
    ACK_CODES,
    DEFAULT_CLOCK_SKEW_TOLERANCE_MS,
    DEFAULT_MAX_AGE_CAP_MS,
    MAX_PAYLOAD_BYTES,
    AckCode,
    AckPayload,
    CommandType,
    Counters,
    DecodedCommand,
    DecodeError,
    RobotError,
    RobotState,
    RobotTopics,
    check_freshness,
    decode_command,
    encode_payload,
    is_valid_session,
    validate_params,
    verify_sig,
)

__all__ = [
    'DEFAULT_RETAINED_REJECTIONS_PER_MINUTE',
    'DEFAULT_TASK_TIMEOUT_CAP_S',
    'DEFAULT_WARN_INTERVAL_S',
    'Ack',
    'Authoriser',
    'AuthoriserConfig',
    'Decision',
    'Drop',
    'DropCounter',
    'GatingState',
    'Ignore',
    'StateView',
    'gate_motion',
    'resolve_request',
]

LOGGER = logging.getLogger(__name__)

#: ``command.task_timeout_cap_s`` default.
DEFAULT_TASK_TIMEOUT_CAP_S = 600.0
#: ``reply.retained_rejections_per_minute`` default.
DEFAULT_RETAINED_REJECTIONS_PER_MINUTE = 30
#: Minimum spacing of the rate-limited WARN lines (drops and unauthorised commands).
DEFAULT_WARN_INTERVAL_S = 10.0
#: Length of the window behind ``retained_rejections_per_minute``.
_RATE_WINDOW_S = 60.0

#: The heartbeat counters a :class:`Drop` names.
DropCounter = Literal['dropped_retained', 'dropped_oversize', 'dropped_invalid_id']


@dataclass(frozen=True)
class AuthoriserConfig:
    """The node parameters the authoriser reads (see ``config/bridge.example.yaml``)."""

    #: ``mqtt.max_payload_bytes``: inbound size limit, measured on the bytes received.
    max_payload_bytes: int = MAX_PAYLOAD_BYTES
    #: ``command.secret`` after the environment fallback; empty means HMAC is off.
    secret: str = ''
    #: ``command.max_age_cap_ms``.
    max_age_cap_ms: int = DEFAULT_MAX_AGE_CAP_MS
    #: ``command.clock_skew_tolerance_ms``.
    clock_skew_tolerance_ms: int = DEFAULT_CLOCK_SKEW_TOLERANCE_MS
    #: ``command.task_timeout_cap_s``: the effective task timeout never exceeds this.
    task_timeout_cap_s: float = DEFAULT_TASK_TIMEOUT_CAP_S
    #: ``reply.retained_rejections_per_minute``; 0 publishes every rejection non-retained.
    retained_rejections_per_minute: int = DEFAULT_RETAINED_REJECTIONS_PER_MINUTE
    #: Spacing of the rate-limited WARN lines.
    warn_interval_s: float = DEFAULT_WARN_INTERVAL_S

    def __post_init__(self) -> None:
        if self.max_payload_bytes < 1:
            raise ValueError('max_payload_bytes must be at least 1')
        if self.max_age_cap_ms < 1:
            raise ValueError('max_age_cap_ms must be at least 1')
        if self.clock_skew_tolerance_ms < 0:
            raise ValueError('clock_skew_tolerance_ms must not be negative')
        if self.task_timeout_cap_s <= 0:
            raise ValueError('task_timeout_cap_s must be greater than 0')
        if self.retained_rejections_per_minute < 0:
            raise ValueError('retained_rejections_per_minute must not be negative')
        if self.warn_interval_s < 0:
            raise ValueError('warn_interval_s must not be negative')

    @property
    def hmac_required(self) -> bool:
        """True when a secret is configured; announced as ``capabilities.hmac_required``."""
        return bool(self.secret)


@dataclass(frozen=True)
class GatingState:
    """The slice of robot state that step 9 needs, supplied by a :data:`StateView`.

    ``task`` and ``task_id`` name the running task (for the ``rejected_busy``
    reason) and ``error`` the current fault (for ``rejected_error``); both may
    be ``None`` when unknown. The robot counts as busy when ``state`` is
    ``busy`` or a ``task_id`` is given: the task manager reports ``error`` as
    the state while a motion task is still running with an error set, and
    :func:`gate_motion` must answer ``rejected_busy`` for that robot, as the
    manager itself does. ``TaskManager.gating_state`` builds this view.
    """

    state: RobotState = 'idle'
    inhibited: bool = False
    task_id: str | None = None
    task: str | None = None
    error: RobotError | None = None

    @property
    def busy(self) -> bool:
        """True while a motion task occupies the single active slot."""
        return self.state == 'busy' or self.task_id is not None


#: Returns the current :class:`GatingState`; called once per task command at step 9.
StateView = Callable[[], GatingState]


def gate_motion(state: GatingState) -> tuple[AckCode, str] | None:
    """Contract step 9 for a ``motion`` task: why it is refused now, or ``None`` when it may start.

    Busy, error and inhibited are checked in that order. This is the only
    implementation of the rule: the authoriser applies it to its state view and
    ``TaskManager.motion_rejection`` applies it to the manager's own state, so
    the ack code and reason never depend on which module answered.
    """
    if state.busy:
        if state.task is not None and state.task_id is not None:
            return 'rejected_busy', f"Task '{state.task}' (id {state.task_id}) is running"
        return 'rejected_busy', 'A task is running'
    if state.state == 'error':
        if state.error is None:
            return 'rejected_error', 'Robot is in error'
        detail = f': {state.error.reason}' if state.error.reason else ''
        return 'rejected_error', f'Robot is in error ({state.error.code}){detail}'
    if state.inhibited:
        return 'rejected_inhibited', 'New motion is inhibited while the HA link is down'
    return None


def resolve_request(
    command: DecodedCommand,
    spec: TaskSpec,
    *,
    task_timeout_cap_s: float = DEFAULT_TASK_TIMEOUT_CAP_S,
) -> TaskRequest:
    """Contract step 8's result: the :class:`TaskRequest` ``executor.start`` receives.

    ``timeout_s`` is the command's own value when given, else the announced
    ``spec.timeout_s``, and either is clamped to ``task_timeout_cap_s``. This is
    the one place the effective timeout is decided; the task manager arms its
    deadline from the request it is handed and never recomputes it.
    """
    if not command.is_task or command.task != spec.name:
        raise ValueError(
            f'resolve_request() needs a task command for {spec.name!r}, got '
            f'{command.type!r} for {command.task!r}'
        )
    requested = spec.timeout_s if command.timeout_s is None else command.timeout_s
    return TaskRequest(
        id=command.id,
        task=spec.name,
        params=dict(command.params or {}),
        issued_at=command.issued_at,
        timeout_s=float(min(requested, task_timeout_cap_s)),
        source=dict(command.source),
    )


@dataclass(frozen=True)
class Ignore:
    """Nothing to do: the node's own clearing echo (empty retained payload)."""

    reason: str


@dataclass(frozen=True)
class Drop:
    """No ack is possible; the named counter has already been incremented.

    ``clear_retained`` asks the publisher to clear the command topic with an
    empty retained publish (a planted retained command).
    """

    counter: DropCounter
    reason: str
    size: int = 0
    clear_retained: bool = False


@dataclass(frozen=True)
class Ack:
    """An ack to publish on ``topic``, retained unless ``retain`` is False.

    ``payload`` is the ack object and :attr:`wire` its bytes: for a duplicate
    (``is_duplicate``) they are the stored ack decoded and ``stored_ack_bytes``
    unchanged, as the contract requires, and ``retain`` is how the stored ack
    was published. ``command`` is ``None`` for ``rejected_malformed`` and for a
    duplicate answered to a payload that could not be decoded. For an
    authorised task command ``request`` is the resolved
    :class:`~ros2_ha_bridge.executors.base.TaskRequest` (timeout clamped) and
    ``spec`` the announced task; the task manager hands ``request`` to
    ``executor.start``.
    """

    topic: str
    payload: dict[str, Any]
    code: AckCode
    command: DecodedCommand | None = None
    is_duplicate: bool = False
    stored_ack_bytes: bytes | None = None
    retain: bool = True
    request: TaskRequest | None = None
    spec: TaskSpec | None = None

    @property
    def id(self) -> str:
        """The command id this ack answers."""
        return str(self.payload['id'])

    @property
    def accepted(self) -> bool:
        return self.code == 'accepted'

    @property
    def authorised(self) -> bool:
        """True for a command that passed every check and awaits ``commit()`` or ``reject()``.

        This is a property of how the ack was produced, so it stays True on the
        same object after the task manager has committed it.
        """
        return self.accepted and not self.is_duplicate

    @property
    def wire(self) -> bytes:
        """The bytes to publish: the stored bytes for a duplicate, else the compact payload."""
        if self.stored_ack_bytes is not None:
            return self.stored_ack_bytes
        return encode_payload(self.payload)


#: What :meth:`Authoriser.handle` returns.
Decision = Ignore | Drop | Ack


def _as_ack_code(value: object) -> AckCode | None:
    for code in ACK_CODES:
        if value == code:
            return code
    return None


class _WarningLimiter:
    """At most one WARN line per key per interval; the next line says how many were suppressed."""

    def __init__(self, clock: Clock, interval_s: float, logger: logging.Logger) -> None:
        self._clock = clock
        self._interval_s = interval_s
        self._logger = logger
        self._last: dict[str, float] = {}
        self._suppressed: dict[str, int] = {}

    def warn(self, key: str, message: str, *args: object) -> bool:
        now = self._clock.monotonic()
        last = self._last.get(key)
        if last is not None and now - last < self._interval_s:
            self._suppressed[key] = self._suppressed.get(key, 0) + 1
            return False
        suppressed = self._suppressed.pop(key, 0)
        if suppressed:
            message += ' (%d similar warnings suppressed)'
            args = (*args, suppressed)
        self._logger.warning(message, *args)
        self._last[key] = now
        return True


class _RetainBudget:
    """Sliding one-minute window over retained rejections (``retained_rejections_per_minute``)."""

    def __init__(self, clock: Clock, per_minute: int) -> None:
        self._clock = clock
        self._per_minute = per_minute
        self._times: deque[float] = deque()

    def allow(self) -> bool:
        if self._per_minute <= 0:
            return False
        now = self._clock.monotonic()
        while self._times and now - self._times[0] >= _RATE_WINDOW_S:
            self._times.popleft()
        if len(self._times) >= self._per_minute:
            return False
        self._times.append(now)
        return True


class Authoriser:
    """Applies steps 0 to 9 to every inbound command and builds the ack to publish.

    ``topics`` names this robot (and the prefix the ack topics are built
    under); ``session`` is the node's session id carried in every ack;
    ``tasks`` is the catalogue after the operator's ``tasks.allow``
    intersection, keyed by name; ``state_view`` supplies the robot state for
    step 9 and defaults to an idle, uninhibited robot. ``counters`` may be the
    node's shared instance so the heartbeat reports what happened here.
    """

    def __init__(
        self,
        *,
        topics: RobotTopics,
        session: str,
        clock: Clock,
        dedupe: DedupeStore,
        tasks: Iterable[TaskSpec] = (),
        config: AuthoriserConfig | None = None,
        state_view: StateView | None = None,
        counters: Counters | None = None,
        logger: logging.Logger | None = None,
    ) -> None:
        if not is_valid_session(session):
            raise ValueError(f'session {session!r} is not a lower-case UUID4 string')
        self.topics = topics
        self.robot_id = topics.robot_id
        self.session = session
        self.clock = clock
        self.dedupe = dedupe
        self.config = config if config is not None else AuthoriserConfig()
        self.state_view: StateView = state_view if state_view is not None else GatingState
        self.counters = counters if counters is not None else Counters()
        self.logger = logger if logger is not None else LOGGER
        self.tasks: dict[str, TaskSpec] = {}
        self.set_tasks(tasks)
        self._pending: set[str] = set()
        self._warnings = _WarningLimiter(clock, self.config.warn_interval_s, self.logger)
        self._retain_budget = _RetainBudget(clock, self.config.retained_rejections_per_minute)

    def set_tasks(self, tasks: Iterable[TaskSpec]) -> None:
        """Replace the allowlist (the node calls this when the catalogue changes).

        A task announced with a ``timeout_s`` above ``command.task_timeout_cap_s``
        is stored with the cap instead, so the catalogue the node announces
        (:attr:`catalogue`) names the timeout the node will enforce and Home
        Assistant's result deadline (announced value plus margin) cannot
        outlive the node's timer.
        """
        cap = self.config.task_timeout_cap_s
        catalogue: dict[str, TaskSpec] = {}
        for spec in tasks:
            if spec.timeout_s > cap:
                self.logger.info(
                    'task %r announces timeout_s %g; clamped to command.task_timeout_cap_s %g',
                    spec.name,
                    spec.timeout_s,
                    cap,
                )
                spec = replace(spec, timeout_s=cap)
            catalogue[spec.name] = spec
        self.tasks = catalogue

    @property
    def catalogue(self) -> tuple[TaskSpec, ...]:
        """The allowlist as the node should announce it (timeouts clamped), in the order given."""
        return tuple(self.tasks.values())

    # ----- steps 0 to 9 ----------------------------------------------------------------

    def handle(self, raw: bytes, *, topic: str, retain: bool = False) -> Decision:
        """Decide what to do with one delivery on the command topic.

        ``topic`` is the topic the message arrived on, which the signature
        covers; ``retain`` is the delivery's RETAIN flag.
        """
        now = self.clock.now_ms()
        if retain:
            if not raw:
                return Ignore('empty retained payload: the clearing echo of this node')
            self.counters.dropped_retained += 1
            self._warnings.warn(
                'retained',
                'dropped a retained message of %d bytes on %s and cleared it: '
                'commands are never retained',
                len(raw),
                topic,
            )
            return Drop(
                'dropped_retained',
                'retained delivery on the command topic',
                size=len(raw),
                clear_retained=True,
            )

        decoded = decode_command(
            raw, self.config.max_payload_bytes, expected_robot_id=self.robot_id
        )
        # Step 6 runs as soon as a safe reply topic exists, before any check that could
        # build a second, different ack for the same id (see the module docstring).
        command_id = decoded.command_id if isinstance(decoded, DecodeError) else decoded.id
        if command_id is not None:
            entry = self.dedupe.lookup(command_id)
            if entry is not None:
                duplicate = self._duplicate(
                    command_id, entry, None if isinstance(decoded, DecodeError) else decoded
                )
                if duplicate is not None:
                    return duplicate
        if isinstance(decoded, DecodeError):
            return self._drop_or_malformed(decoded, now)
        command = decoded

        if self.config.secret and not verify_sig(self.config.secret, topic, command.payload):
            reason = (
                'sig is missing but this robot requires signed commands'
                if command.sig is None
                else 'sig does not match this topic and payload'
            )
            self._warnings.warn(
                'unauthorised', 'rejected command %s as unauthorised: %s', command.id, reason
            )
            return self._reject_command(command, 'rejected_unauthorised', reason, now)

        freshness = check_freshness(
            now,
            command.issued_at,
            command.max_age_ms,
            max_age_cap_ms=self.config.max_age_cap_ms,
            skew_ms=self.config.clock_skew_tolerance_ms,
        )
        skew = freshness.clock_skew_ms
        if freshness.verdict == 'stale':
            reason = (
                f'issued_at is {skew} ms old; the window is {freshness.window_ms} ms '
                f'plus {self.config.clock_skew_tolerance_ms} ms tolerance'
            )
            return self._reject_command(command, 'rejected_stale', reason, now, skew)
        if freshness.verdict == 'future':
            reason = (
                f'issued_at is {-skew} ms ahead of this clock; '
                f'the tolerance is {self.config.clock_skew_tolerance_ms} ms'
            )
            return self._reject_command(command, 'rejected_future', reason, now, skew)

        if command.is_cancel:
            return self._authorise(command, now, skew)

        assert command.task is not None and command.params is not None
        spec = self.tasks.get(command.task)
        if spec is None:
            reason = f'task {command.task!r} is not in the catalogue of robot {self.robot_id!r}'
            return self._reject_command(command, 'rejected_unknown_task', reason, now, skew)
        problems = validate_params(spec.params, command.params)
        if problems:
            reason = '; '.join(problems)
            return self._reject_command(
                command, 'rejected_invalid_params', reason, now, skew, spec=spec
            )
        request = resolve_request(command, spec, task_timeout_cap_s=self.config.task_timeout_cap_s)

        if spec.kind == 'motion':
            refusal = gate_motion(self.state_view())
            if refusal is not None:
                code, reason = refusal
                return self._reject_command(command, code, reason, now, skew, spec=spec)

        return self._authorise(command, now, skew, request=request, spec=spec)

    # ----- steps 10 and 11, driven by the task manager -------------------------------------

    def commit(self, ack: Ack) -> Ack:
        """Record an authorised ack as final: ``executor.start`` succeeded or the cancel resolved.

        Writes the accepted ack to the dedupe store (memory and file) and counts
        it. Raises ``ValueError`` for an ack that is not awaiting a decision.
        """
        self._take_pending(ack)
        self.dedupe.remember(ack.id, int(ack.payload['received_at']), ack.wire, accepted=True)
        self.counters.accepted += 1
        self.logger.debug('accepted command %s (%s)', ack.id, ack.payload.get('task'))
        return ack

    def reject(self, ack: Ack, code: AckCode, reason: str) -> Ack:
        """Replace an authorised ack with a rejection decided by the task manager.

        ``code`` is ``rejected_executor`` (the plugin refused or raised) or
        ``rejected_no_such_task`` (a cancel named no running task); any
        rejection code is accepted, ``accepted`` is not. The new ack keeps the
        id, type, task, ``received_at`` and ``clock_skew_ms`` of the original.
        """
        if code == 'accepted':
            raise ValueError("reject() needs a rejection code, not 'accepted'")
        self._take_pending(ack)
        return self._rejection(
            command_id=ack.id,
            command_type=ack.payload['type'],
            task=ack.payload.get('task'),
            code=code,
            reason=reason,
            received_at=int(ack.payload['received_at']),
            clock_skew_ms=ack.payload.get('clock_skew_ms'),
            command=ack.command,
            spec=ack.spec,
        )

    # ----- internals -----------------------------------------------------------------

    def _take_pending(self, ack: Ack) -> None:
        if not ack.authorised or ack.id not in self._pending:
            raise ValueError(f'ack for {ack.id} is not awaiting commit() or reject()')
        self._pending.discard(ack.id)

    def _drop_or_malformed(self, error: DecodeError, now: int) -> Decision:
        if error.kind == 'oversize':
            self.counters.dropped_oversize += 1
            self._warnings.warn('oversize', 'dropped an oversize command: %s', error.reason)
            return Drop('dropped_oversize', error.reason, size=error.size)
        if not error.replyable:
            self.counters.dropped_invalid_id += 1
            self._warnings.warn(
                'invalid_id', 'dropped a command without a usable id: %s', error.reason
            )
            return Drop('dropped_invalid_id', error.reason, size=error.size)
        assert error.command_id is not None
        skew = None if error.issued_at is None else now - error.issued_at
        return self._rejection(
            command_id=error.command_id,
            command_type=error.ack_type,
            task=error.task,
            code='rejected_malformed',
            reason=error.reason,
            received_at=now,
            clock_skew_ms=skew,
            command=None,
        )

    def _duplicate(
        self, command_id: str, entry: DedupeEntry, command: DecodedCommand | None
    ) -> Ack | None:
        """The stored ack for a known id, republished as it was; ``None`` when unreadable."""
        try:
            payload = json.loads(entry.ack_bytes.decode('utf-8'))
        except ValueError:
            payload = None
        code = _as_ack_code(payload.get('code')) if isinstance(payload, dict) else None
        if not isinstance(payload, dict) or code is None:
            self.logger.error(
                'dedupe entry for %s holds no readable ack; treating it as unknown', command_id
            )
            return None
        self.counters.duplicates += 1
        self.logger.info('command %s seen before; republishing its %s ack', command_id, code)
        return Ack(
            topic=self.topics.ack(command_id),
            payload=payload,
            code=code,
            command=command,
            is_duplicate=True,
            stored_ack_bytes=entry.ack_bytes,
            retain=entry.retain,
        )

    def _reject_command(
        self,
        command: DecodedCommand,
        code: AckCode,
        reason: str,
        received_at: int,
        clock_skew_ms: int | None = None,
        *,
        spec: TaskSpec | None = None,
    ) -> Ack:
        if clock_skew_ms is None:
            clock_skew_ms = received_at - command.issued_at
        return self._rejection(
            command_id=command.id,
            command_type=command.type,
            task=command.task,
            code=code,
            reason=reason,
            received_at=received_at,
            clock_skew_ms=clock_skew_ms,
            command=command,
            spec=spec,
        )

    def _rejection(
        self,
        *,
        command_id: str,
        command_type: CommandType,
        task: str | None,
        code: AckCode,
        reason: str,
        received_at: int,
        clock_skew_ms: int | None,
        command: DecodedCommand | None,
        spec: TaskSpec | None = None,
    ) -> Ack:
        payload = AckPayload(
            id=command_id,
            robot_id=self.robot_id,
            type=command_type,
            task=task,
            code=code,
            reason=reason,
            received_at=received_at,
            clock_skew_ms=clock_skew_ms,
            session=self.session,
            ts=self.clock.now_ms(),
        ).to_payload()
        ack = Ack(
            topic=self.topics.ack(command_id),
            payload=payload,
            code=code,
            command=command,
            retain=self._retain_budget.allow(),
            spec=spec,
        )
        # The dedupe lookup ran before anything that could reach here, so this is the first
        # ack for the id (or it replaces an entry whose bytes were unreadable). The retain
        # decision travels with it so a redelivery is republished the same way.
        self.dedupe.remember(command_id, received_at, ack.wire, accepted=False, retain=ack.retain)
        self.counters.rejected += 1
        self.logger.info('rejected command %s: %s: %s', command_id, code, reason)
        return ack

    def _authorise(
        self,
        command: DecodedCommand,
        received_at: int,
        clock_skew_ms: int,
        *,
        request: TaskRequest | None = None,
        spec: TaskSpec | None = None,
    ) -> Ack:
        payload = AckPayload(
            id=command.id,
            robot_id=self.robot_id,
            type=command.type,
            task=command.task,
            code='accepted',
            received_at=received_at,
            clock_skew_ms=clock_skew_ms,
            session=self.session,
            ts=self.clock.now_ms(),
        ).to_payload()
        self._pending.add(command.id)
        return Ack(
            topic=self.topics.ack(command.id),
            payload=payload,
            code='accepted',
            command=command,
            request=request,
            spec=spec,
        )
