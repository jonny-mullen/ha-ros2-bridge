# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Reference MoveIt 2 executor: named joint poses through move_group's ``/move_action``.

``MoveItExecutor`` drives the ``move_group`` node of MoveIt 2 (ROS 2 Jazzy,
moveit 2.12.4, moveit_msgs 2.6.0) through a plain rclpy action client on the
``MoveGroup`` action, the binding the Isaac Sim research recommends
(``docs/research/2026-10-13-isaac-sim-moveit.json``). Planning and execution
happen in one goal (``plan_only`` false), so every failure arrives as one
``MoveItErrorCodes`` value, and the executor's dependency set stays at rclpy
plus moveit_msgs.

Experimental: this executor has been exercised only against a fake
``MoveGroupAdapter`` (``test/test_moveit_executor.py``) and has not yet run
against a real ``move_group``; the first run happens at the simulator site
through ``docs/simulator_checklist.md``.

Two layers, so the logic is unit-testable without ROS:

- A pure core (:class:`MoveItSettings`, :func:`parse_poses`, :func:`build_goal`,
  :func:`map_error_code`, :class:`MoveItExecutor`) that never imports rclpy or
  moveit_msgs. It talks to MoveIt through the :class:`MoveGroupAdapter`
  protocol: ``send_goal(goal_dict, feedback) -> GoalHandle`` with
  ``add_done`` and ``cancel``, plus ``server_available`` and ``close``.
- :class:`RclpyMoveGroupAdapter`, the real adapter, which imports rclpy's
  ``ActionClient`` and the moveit_msgs types lazily inside its constructor and
  builds the ``MoveGroup.Goal`` message from the goal dict.

``MotionPlanRequest`` has no named-target field, so a named pose is resolved
here, from the executor's own configuration, to joint positions sent as
``JointConstraint`` entries in ``goal_constraints``. An unknown pose is refused
with :class:`~ros2_ha_bridge.executors.base.TaskRejected` before any MoveIt
call and never produces a MoveIt code.

Parameters (dotted names, read from ``BridgeContext.parameters`` and, inside
the real node, from the node's own params file; see :meth:`MoveItExecutor.setup`):

``moveit.group_name`` (required)
    The planning group, for example ``ur_manipulator`` or ``panda_arm``.
``moveit.poses``
    Either a mapping of pose name to ``{joint_name: position}`` (inline, or
    flattened by rcl into ``moveit.poses.<pose>.<joint>`` parameters), or the
    path of a YAML file holding that mapping (optionally under a ``poses``
    key). At least one pose is required.
``moveit.planning_time_s`` (5.0), ``moveit.num_attempts`` (1),
``moveit.velocity_scale`` (0.1), ``moveit.acceleration_scale`` (0.1)
    ``allowed_planning_time``, ``num_planning_attempts`` and the two scaling
    factors of the ``MotionPlanRequest``. Scaling factors must lie in (0, 1].
``moveit.action_name`` (``/move_action``)
    The MoveGroup action name.
``moveit.hold_on_cancel`` (true)
    When a task is cancelled (cancel command, task timeout or the ``stop``
    task) the goal is always cancelled at move_group, which stops trajectory
    execution and leaves the controllers holding position. With ``true`` the
    bridge task finishes ``cancelled`` only once move_group confirms the goal
    ended (``PREEMPTED``), so the result means "the arm is holding"; with
    ``false`` it finishes ``cancelled`` as soon as the cancel request has been
    sent, and the confirmation that follows is dropped. The ``stop`` task's
    error clearing follows the same rule (below).
``moveit.joint_states_topic`` (``/joint_states``), ``moveit.joint_names`` (empty)
    Joint telemetry (0.2.0). The adapter subscribes to ``sensor_msgs/JointState``
    on the topic and the executor forwards the positions (and velocities,
    when the message carries them) of the group's joints to
    ``ctx.report_joints``, in radians, with no limits; the node throttles
    them. The joints are ``moveit.joint_names`` when given, otherwise the
    joints of the first configured pose, in that order. Their announced names
    are the MoveIt names made to fit the contract's pattern (lower-case,
    other characters to ``_``, 32 characters); when two names collapse into
    one or a name has nothing left, no joints are announced and a warning
    says why. A ``JointState`` message that lacks one of the joints is
    skipped.

Task catalogue (the ``kind`` decides how the bridge gates it):

``move_to {pose}`` (motion, 120 s)
    ``pose`` is an enum of the configured pose names. Plans and executes.
``home`` (motion, 120 s)
    Present only when a pose named ``home`` exists; the same as ``move_to``
    with that pose, so Home Assistant gets a button for it.
``stop`` (control, 5 s)
    Cancels the running move (the task ends ``cancelled/preempted``) and
    clears any robot error, so an operator can recover from a forced finish.
    With ``hold_on_cancel`` true and a goal in flight the error is cleared
    only when move_group confirms the goal ended, so an ``idle`` robot always
    means the arm is holding; the ``stop`` result says so in its reason and
    carries ``awaiting_confirmation: true``. With ``hold_on_cancel`` false, or
    nothing in flight, the error is cleared at once. A pending clear is
    dropped, with a warning in the log, when the run is abandoned by
    ``stop_and_hold``, ``shutdown`` or a newer task; send ``stop`` again then.
``refresh_poses`` (query, 5 s)
    Re-reads the poses (the YAML file, or the parameters). Changed joint
    positions for existing names and removed poses take effect at once. A new
    pose name is refused until the node restarts: the node validates
    ``params`` against the catalogue it announced at start-up (contract step
    8), and Home Assistant refuses it against the same enum before publishing
    (``docs/decisions.md``, open question (g)).

Result codes (``docs/writing-an-executor.md`` has the full table): success
only when ``val == 1`` (``succeeded/ok``); ``collision`` for
``GOAL_IN_COLLISION`` and ``START_STATE_IN_COLLISION``; ``planning_failed``
for the rest of the planning class; ``control_failed`` for
``CONTROL_FAILED``; ``timeout`` for ``TIMED_OUT``; ``cancelled`` for
``PREEMPTED`` when this executor cancelled, else ``preempted``;
``invalid_goal`` for the request-error class; ``unavailable`` for the
sensor, scene and communication class, for a goal move_group rejected and for
an absent action server; ``executor_error`` for everything else. A goal that
move_group rejects after this executor had already asked for its cancel ends
``cancelled`` (nothing was executing), not ``unavailable``. The MoveIt name
travels in the reason and ``data`` carries ``moveit_error_code``,
``moveit_error_name``, ``message``, ``goal_status``, ``planning_time_s`` and
``pose``.

Threading: the bridge calls ``setup``, ``tasks``, ``start``, ``cancel``,
``stop_and_hold``, ``health`` and ``shutdown`` on the node's main thread. The
adapter's feedback and done callbacks are posted through ``ctx.post`` before
they touch the executor's state, so the core is correct whichever thread an
adapter calls back from (rclpy's callbacks already run on the executor
thread; the post is one extra hop there).
"""

from __future__ import annotations

import importlib
import logging
import math
import os
import re
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal, Protocol, TypeGuard

from ros2_ha_bridge import __version__
from ros2_ha_bridge.executors.base import (
    MAX_JOINTS,
    BridgeContext,
    HealthLevel,
    JointInfo,
    ResultStatus,
    TaskExecutor,
    TaskHandle,
    TaskRejected,
    TaskRequest,
    TaskResult,
    TaskSpec,
)
from ros2_ha_bridge.protocol import is_valid_joint_name, validate_params

__all__ = [
    'CONTROL_TIMEOUT_S',
    'DEFAULT_ACCELERATION_SCALE',
    'DEFAULT_ACTION_NAME',
    'DEFAULT_HOLD_ON_CANCEL',
    'DEFAULT_JOINT_STATES_TOPIC',
    'DEFAULT_NUM_ATTEMPTS',
    'DEFAULT_PLANNING_TIME_S',
    'DEFAULT_VELOCITY_SCALE',
    'ENTRY_POINT_NAME',
    'HOME_POSE',
    'JOINT_TOLERANCE_RAD',
    'MOTION_TIMEOUT_S',
    'MOVEIT_ERROR_CLASSES',
    'MOVEIT_ERROR_NAMES',
    'PARAMETER_PREFIX',
    'POSES_PARAMETER',
    'SCALAR_PARAMETERS',
    'STATUS_ABORTED',
    'STATUS_CANCELED',
    'STATUS_SUCCEEDED',
    'UNAVAILABLE_CODE',
    'ErrorClass',
    'GoalHandle',
    'GoalOutcome',
    'JointStateCallback',
    'MappedCode',
    'MoveGroupAdapter',
    'MoveItExecutor',
    'MoveItSettings',
    'RclpyMoveGroupAdapter',
    'announce_joint_names',
    'build_catalogue',
    'build_goal',
    'error_class',
    'flattened_poses',
    'load_poses_file',
    'map_error_code',
    'parse_poses',
    'unknown_parameter_names',
]

#: The entry-point name this executor is registered under (``executor: moveit``).
ENTRY_POINT_NAME = 'moveit'
#: Every parameter of this executor starts with this prefix.
PARAMETER_PREFIX = 'moveit.'
#: The poses parameter: a mapping, a YAML path, or flattened ``moveit.poses.<pose>.<joint>``.
POSES_PARAMETER = 'moveit.poses'
DEFAULT_ACTION_NAME = '/move_action'
DEFAULT_PLANNING_TIME_S = 5.0
DEFAULT_NUM_ATTEMPTS = 1
DEFAULT_VELOCITY_SCALE = 0.1
DEFAULT_ACCELERATION_SCALE = 0.1
DEFAULT_HOLD_ON_CANCEL = True
#: ``sensor_msgs/JointState`` topic the joint telemetry is read from.
DEFAULT_JOINT_STATES_TOPIC = '/joint_states'
#: ``tolerance_above`` and ``tolerance_below`` of every joint constraint, in radians.
JOINT_TOLERANCE_RAD = 0.01
#: The pose name that also gets its own ``home`` task.
HOME_POSE = 'home'
#: Announced timeouts (the node clamps them to ``command.task_timeout_cap_s``).
MOTION_TIMEOUT_S = 120.0
CONTROL_TIMEOUT_S = 5.0
#: Result code for "MoveIt or its server is not in a state to try": retryable.
UNAVAILABLE_CODE = 'unavailable'

#: ``action_msgs/GoalStatus`` terminal values.
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6

#: The scalar parameters the real adapter declares on the node: name, default, description.
SCALAR_PARAMETERS: tuple[tuple[str, str | int | float | bool | list[str], str], ...] = (
    ('moveit.group_name', '', 'MoveIt executor: the planning group (required).'),
    (POSES_PARAMETER, '', 'MoveIt executor: path of a poses YAML file, or inline poses.'),
    ('moveit.action_name', DEFAULT_ACTION_NAME, 'MoveIt executor: the MoveGroup action name.'),
    (
        'moveit.planning_time_s',
        DEFAULT_PLANNING_TIME_S,
        'MoveIt executor: allowed_planning_time in seconds.',
    ),
    ('moveit.num_attempts', DEFAULT_NUM_ATTEMPTS, 'MoveIt executor: num_planning_attempts.'),
    (
        'moveit.velocity_scale',
        DEFAULT_VELOCITY_SCALE,
        'MoveIt executor: max_velocity_scaling_factor, in (0, 1].',
    ),
    (
        'moveit.acceleration_scale',
        DEFAULT_ACCELERATION_SCALE,
        'MoveIt executor: max_acceleration_scaling_factor, in (0, 1].',
    ),
    (
        'moveit.hold_on_cancel',
        DEFAULT_HOLD_ON_CANCEL,
        'MoveIt executor: finish a cancelled task only once move_group confirms the hold.',
    ),
    (
        'moveit.joint_states_topic',
        DEFAULT_JOINT_STATES_TOPIC,
        'MoveIt executor: the sensor_msgs/JointState topic the joint telemetry is read from.',
    ),
    (
        'moveit.joint_names',
        [],
        "MoveIt executor: the joints to report, in order; empty means the first pose's joints.",
    ),
)
_KNOWN_PARAMETERS = frozenset(name for name, _, _ in SCALAR_PARAMETERS)

#: Every value of ``moveit_msgs/msg/MoveItErrorCodes`` at moveit_msgs 2.6.0 (Jazzy); the values
#: -8, -9 and -20 are unused there. The spelling of -5 is MoveIt's own.
MOVEIT_ERROR_NAMES: Mapping[int, str] = {
    1: 'SUCCESS',
    0: 'UNDEFINED',
    99999: 'FAILURE',
    -1: 'PLANNING_FAILED',
    -2: 'INVALID_MOTION_PLAN',
    -3: 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
    -4: 'CONTROL_FAILED',
    -5: 'UNABLE_TO_AQUIRE_SENSOR_DATA',  # codespell:ignore aquire
    -6: 'TIMED_OUT',
    -7: 'PREEMPTED',
    -10: 'START_STATE_IN_COLLISION',
    -11: 'START_STATE_VIOLATES_PATH_CONSTRAINTS',
    -12: 'GOAL_IN_COLLISION',
    -13: 'GOAL_VIOLATES_PATH_CONSTRAINTS',
    -14: 'GOAL_CONSTRAINTS_VIOLATED',
    -15: 'INVALID_GROUP_NAME',
    -16: 'INVALID_GOAL_CONSTRAINTS',
    -17: 'INVALID_ROBOT_STATE',
    -18: 'INVALID_LINK_NAME',
    -19: 'INVALID_OBJECT_NAME',
    -21: 'FRAME_TRANSFORM_FAILURE',
    -22: 'COLLISION_CHECKING_UNAVAILABLE',
    -23: 'ROBOT_STATE_STALE',
    -24: 'SENSOR_INFO_STALE',
    -25: 'COMMUNICATION_FAILURE',
    -26: 'START_STATE_INVALID',
    -27: 'GOAL_STATE_INVALID',
    -28: 'UNRECOGNIZED_GOAL_TYPE',
    -29: 'CRASH',
    -30: 'ABORT',
    -31: 'NO_IK_SOLUTION',
}

ErrorClass = Literal[
    'succeeded',
    'planning_failed',
    'execution_failed',
    'timed_out',
    'preempted',
    'rejected',
    'unavailable',
    'internal_error',
]

_PLANNING_CODES = frozenset({-1, -2, -3, -31, -10, -11, -12, -13, -14})
_COLLISION_CODES = frozenset({-10, -12})
_EXECUTION_CODES = frozenset({-4})
_TIMED_OUT_CODES = frozenset({-6})
_PREEMPTED_CODES = frozenset({-7})
_REJECTED_CODES = frozenset({-15, -16, -17, -18, -19, -26, -27, -28})
_UNAVAILABLE_CODES = frozenset({-5, -22, -23, -24, -25})


def error_class(val: int) -> ErrorClass:
    """The research's class of a ``MoveItErrorCodes`` value (unknown values are internal errors)."""
    if val == 1:
        return 'succeeded'
    if val in _PLANNING_CODES:
        return 'planning_failed'
    if val in _EXECUTION_CODES:
        return 'execution_failed'
    if val in _TIMED_OUT_CODES:
        return 'timed_out'
    if val in _PREEMPTED_CODES:
        return 'preempted'
    if val in _REJECTED_CODES:
        return 'rejected'
    if val in _UNAVAILABLE_CODES:
        return 'unavailable'
    return 'internal_error'


#: Every known value with its class, for tests and documentation.
MOVEIT_ERROR_CLASSES: Mapping[int, ErrorClass] = {
    val: error_class(val) for val in MOVEIT_ERROR_NAMES
}

_LOGGER_NAME = 'ros2_ha_bridge.executors.moveit'


@dataclass(frozen=True)
class MappedCode:
    """What a ``MoveItErrorCodes`` value becomes on the bridge."""

    status: ResultStatus
    code: str
    error_class: ErrorClass
    moveit_name: str
    reason: str


def map_error_code(
    val: int, *, message: str = '', cancelled_by_bridge: bool = False
) -> MappedCode:
    """Map a ``MoveItErrorCodes.val`` to the bridge's status, code and reason.

    ``cancelled_by_bridge`` says whether this executor asked move_group to
    cancel the goal: ``PREEMPTED`` is then the expected confirmation and maps
    to ``cancelled/cancelled``; otherwise another client pre-empted the goal
    and the task is ``aborted/preempted``.
    """
    name = MOVEIT_ERROR_NAMES.get(val, 'UNKNOWN')
    klass = error_class(val)
    detail = f'MoveIt {name} ({val})'
    if message:
        detail = f'{detail}: {message}'
    if klass == 'succeeded':
        return MappedCode('succeeded', 'ok', klass, name, detail)
    if klass == 'planning_failed':
        code = 'collision' if val in _COLLISION_CODES else 'planning_failed'
        return MappedCode('aborted', code, klass, name, detail)
    if klass == 'execution_failed':
        return MappedCode('aborted', 'control_failed', klass, name, detail)
    if klass == 'timed_out':
        return MappedCode('aborted', 'timeout', klass, name, detail)
    if klass == 'preempted':
        if cancelled_by_bridge:
            return MappedCode(
                'cancelled', 'cancelled', klass, name, f'Cancelled by request ({detail})'
            )
        return MappedCode(
            'aborted',
            'preempted',
            klass,
            name,
            f'{detail}: pre-empted by another move_group client',
        )
    if klass == 'rejected':
        return MappedCode('aborted', 'invalid_goal', klass, name, detail)
    if klass == 'unavailable':
        return MappedCode('aborted', UNAVAILABLE_CODE, klass, name, detail)
    return MappedCode('aborted', 'executor_error', klass, name, detail)


# --------------------------------------------------------------------------- adapter protocol


@dataclass(frozen=True)
class GoalOutcome:
    """How one MoveGroup goal ended, as the adapter reports it.

    ``accepted`` false means move_group rejected the goal (or the request
    itself failed) and no result exists. Otherwise ``status`` is the terminal
    ``action_msgs/GoalStatus``, ``error_code`` the ``MoveItErrorCodes.val`` (or
    ``None`` when the result could not be read), ``message`` the
    ``MoveItErrorCodes.message`` string and ``planning_time_s`` the result's
    ``planning_time``. ``detail`` carries the adapter's own words for a goal
    that never produced a code.
    """

    accepted: bool = True
    status: int = 0
    error_code: int | None = None
    message: str = ''
    planning_time_s: float | None = None
    detail: str = ''


class GoalHandle(Protocol):
    """One sent goal. ``add_done`` fires exactly once, with the outcome; ``cancel`` asks for a cancel."""

    def add_done(self, callback: Callable[[GoalOutcome], None]) -> None:
        """Register ``callback`` for the terminal outcome (called at once if already known)."""

    def cancel(self) -> None:
        """Ask the server to cancel this goal. The outcome still arrives through ``add_done``."""


#: ``callback(names, positions, velocities)`` for one ``sensor_msgs/JointState`` message;
#: ``velocities`` is ``None`` when the message carries none.
JointStateCallback = Callable[[Sequence[str], Sequence[float], Sequence[float] | None], None]


class MoveGroupAdapter(Protocol):
    """The slice of an action client the core needs; :class:`RclpyMoveGroupAdapter` is the real one."""

    def server_available(self) -> bool:
        """True when the MoveGroup action server is ready (non-blocking)."""

    def send_goal(self, goal: Mapping[str, Any], feedback: Callable[[str], None]) -> GoalHandle:
        """Send the goal built by :func:`build_goal`; ``feedback`` receives the action's state string."""

    def subscribe_joint_states(self, topic: str, callback: JointStateCallback) -> None:
        """Deliver every ``sensor_msgs/JointState`` on ``topic`` to ``callback`` (0.2.0)."""

    def close(self) -> None:
        """Release the client (and the joint-state subscription) at shutdown."""


# --------------------------------------------------------------------------- settings and poses

Poses = dict[str, dict[str, float]]


def _is_number(value: object) -> TypeGuard[int | float]:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def parse_poses(document: object, origin: str) -> Poses:
    """Validate a poses document: ``{pose: {joint: position}}``, optionally under a ``poses`` key.

    Raises ``TypeError`` when the document is not a mapping, and ``ValueError``
    naming ``origin`` (the file or the parameter) for a pose that is not a
    non-empty mapping of non-empty joint names to finite numbers, or for an
    empty document.
    """
    if not isinstance(document, Mapping):
        raise TypeError(f'{origin}: expected a mapping of pose name to joint positions')
    if 'poses' in document and isinstance(document['poses'], Mapping):
        document = document['poses']
    poses: Poses = {}
    for raw_name, raw_joints in document.items():
        name = str(raw_name).strip()
        if not name:
            raise ValueError(f'{origin}: a pose name is empty')
        if not isinstance(raw_joints, Mapping) or not raw_joints:
            raise ValueError(
                f'{origin}: pose {name!r} must be a non-empty mapping of joint name to position'
            )
        joints: dict[str, float] = {}
        for raw_joint, raw_position in raw_joints.items():
            joint = str(raw_joint).strip()
            if not joint:
                raise ValueError(f'{origin}: pose {name!r} has an empty joint name')
            if not _is_number(raw_position) or not math.isfinite(float(raw_position)):
                raise ValueError(
                    f'{origin}: pose {name!r} joint {joint!r} must be a finite number, '
                    f'not {raw_position!r}'
                )
            joints[joint] = float(raw_position)
        poses[name] = joints
    if not poses:
        raise ValueError(f'{origin}: no poses defined')
    return poses


def load_poses_file(path: Path) -> Poses:
    """Read and validate a poses YAML file (``OSError`` for the file, ``ValueError`` for its content)."""
    yaml: Any = importlib.import_module('yaml')
    with path.open(encoding='utf-8') as handle:
        try:
            document = yaml.safe_load(handle)
        except yaml.YAMLError as error:
            raise ValueError(f'{path}: not valid YAML: {error}') from error
    return parse_poses(document, str(path))


def flattened_poses(parameters: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    """Collect ``moveit.poses.<pose>.<joint>`` entries into ``{pose: {joint: value}}`` (unvalidated).

    This is the shape rcl gives inline poses from a params file; a joint name
    may itself contain dots, so only the first segment after the prefix is the
    pose name.
    """
    prefix = POSES_PARAMETER + '.'
    nested: dict[str, dict[str, Any]] = {}
    for name, value in parameters.items():
        if not isinstance(name, str) or not name.startswith(prefix):
            continue
        pose, _, joint = name[len(prefix) :].partition('.')
        nested.setdefault(pose, {})[joint] = value
    return nested


def unknown_parameter_names(parameters: Mapping[str, Any]) -> list[str]:
    """``moveit.*`` names this executor does not read, so a typo is logged rather than silent."""
    unknown: list[str] = []
    for name in parameters:
        if not isinstance(name, str) or not name.startswith(PARAMETER_PREFIX):
            continue
        if name in _KNOWN_PARAMETERS or name.startswith(POSES_PARAMETER + '.'):
            continue
        unknown.append(name)
    return sorted(unknown)


def announce_joint_names(raw_names: Sequence[str]) -> tuple[str, ...] | None:
    """The announce form of MoveIt joint names, or ``None`` when they cannot be told apart.

    Each name is lower-cased, runs of characters outside ``[a-z0-9_]`` become
    ``_``, leading characters that are not letters are dropped and the rest
    is cut to 32 characters, so ``shoulder_pan_joint`` stays itself and
    ``Gripper.Finger`` becomes ``gripper_finger``. ``None`` means a name had
    nothing left, two names collapsed into one, or there are none or more
    than 32; the executor then announces no joints.
    """
    if not 1 <= len(raw_names) <= MAX_JOINTS:
        return None
    announced: list[str] = []
    for raw in raw_names:
        if is_valid_joint_name(raw):
            announced.append(raw)
            continue
        cleaned = re.sub(r'[^a-z0-9_]+', '_', str(raw).lower()).strip('_')
        cleaned = re.sub(r'^[^a-z]+', '', cleaned)[:32]
        if not is_valid_joint_name(cleaned):
            return None
        announced.append(cleaned)
    if len(set(announced)) != len(announced):
        return None
    return tuple(announced)


@dataclass(frozen=True)
class MoveItSettings:
    """The executor's parameters, validated. Build with :meth:`from_parameters`."""

    group_name: str
    action_name: str = DEFAULT_ACTION_NAME
    planning_time_s: float = DEFAULT_PLANNING_TIME_S
    num_attempts: int = DEFAULT_NUM_ATTEMPTS
    velocity_scale: float = DEFAULT_VELOCITY_SCALE
    acceleration_scale: float = DEFAULT_ACCELERATION_SCALE
    hold_on_cancel: bool = DEFAULT_HOLD_ON_CANCEL
    #: The poses YAML file, or ``None`` when the poses were given inline.
    poses_file: Path | None = None
    #: The inline poses (empty when a file is used).
    inline_poses: Mapping[str, Mapping[str, float]] = field(default_factory=dict)
    #: ``sensor_msgs/JointState`` topic for the joint telemetry.
    joint_states_topic: str = DEFAULT_JOINT_STATES_TOPIC
    #: The joints to report, in order; empty means the first pose's joints.
    joint_names: tuple[str, ...] = ()

    @property
    def poses_source(self) -> str:
        """Where the poses come from, for logs and results."""
        return str(self.poses_file) if self.poses_file is not None else POSES_PARAMETER

    def load_poses(self) -> Poses:
        """The current poses: re-read from the file, or a copy of the inline mapping."""
        if self.poses_file is not None:
            return load_poses_file(self.poses_file)
        return {name: dict(joints) for name, joints in self.inline_poses.items()}

    @classmethod
    def from_parameters(cls, parameters: Mapping[str, Any]) -> MoveItSettings:
        """Validate the ``moveit.*`` entries of ``parameters``; ``ValueError`` names the offender."""
        group_name = _string(parameters, 'moveit.group_name', '').strip()
        if not group_name:
            raise ValueError('moveit.group_name is required (the MoveIt planning group)')
        action_name = _string(parameters, 'moveit.action_name', DEFAULT_ACTION_NAME).strip()
        if not action_name:
            raise ValueError('moveit.action_name must name the MoveGroup action')
        planning_time_s = _number(parameters, 'moveit.planning_time_s', DEFAULT_PLANNING_TIME_S)
        if planning_time_s <= 0:
            raise ValueError(
                f'moveit.planning_time_s must be greater than 0, not {planning_time_s:g}'
            )
        attempts = _number(parameters, 'moveit.num_attempts', DEFAULT_NUM_ATTEMPTS)
        if attempts < 1 or not float(attempts).is_integer():
            raise ValueError(
                f'moveit.num_attempts must be a whole number of at least 1, not {attempts!r}'
            )
        velocity = _number(parameters, 'moveit.velocity_scale', DEFAULT_VELOCITY_SCALE)
        acceleration = _number(parameters, 'moveit.acceleration_scale', DEFAULT_ACCELERATION_SCALE)
        for name, value in (
            ('moveit.velocity_scale', velocity),
            ('moveit.acceleration_scale', acceleration),
        ):
            if not 0 < value <= 1:
                raise ValueError(f'{name} must be in (0, 1], not {value:g}')
        hold = parameters.get('moveit.hold_on_cancel', DEFAULT_HOLD_ON_CANCEL)
        if not isinstance(hold, bool):
            raise TypeError(f'moveit.hold_on_cancel must be true or false, not {hold!r}')
        joint_states_topic = _string(
            parameters, 'moveit.joint_states_topic', DEFAULT_JOINT_STATES_TOPIC
        ).strip()
        if not joint_states_topic:
            raise ValueError('moveit.joint_states_topic must name a sensor_msgs/JointState topic')
        joint_names = _strings(parameters, 'moveit.joint_names')
        poses_file, inline = _poses_from(parameters)
        return cls(
            group_name=group_name,
            action_name=action_name,
            planning_time_s=planning_time_s,
            num_attempts=int(attempts),
            velocity_scale=velocity,
            acceleration_scale=acceleration,
            hold_on_cancel=hold,
            poses_file=poses_file,
            inline_poses=inline,
            joint_states_topic=joint_states_topic,
            joint_names=joint_names,
        )


def _string(parameters: Mapping[str, Any], name: str, default: str) -> str:
    value = parameters.get(name, default)
    if not isinstance(value, str):
        raise TypeError(f'{name} must be a string, not {type(value).__name__}')
    return value


def _number(parameters: Mapping[str, Any], name: str, default: float) -> float:
    value = parameters.get(name, default)
    if not _is_number(value) or not math.isfinite(float(value)):
        raise ValueError(f'{name} must be a finite number, not {value!r}')
    return float(value)


def _strings(parameters: Mapping[str, Any], name: str) -> tuple[str, ...]:
    """A list-of-strings parameter; absent, ``None`` or empty gives ``()``."""
    value = parameters.get(name)
    if value is None or value == '' or (isinstance(value, (list, tuple)) and not value):
        return ()
    if isinstance(value, (str, bytes)) or not isinstance(value, (list, tuple)):
        raise TypeError(f'{name} must be a list of strings, not {type(value).__name__}')
    names: list[str] = []
    for item in value:
        if not isinstance(item, str) or not item.strip():
            raise ValueError(f'{name} must contain only non-empty strings, found {item!r}')
        names.append(item.strip())
    if len(set(names)) != len(names):
        raise ValueError(f'{name} must not repeat a joint name')
    return tuple(names)


def _poses_from(parameters: Mapping[str, Any]) -> tuple[Path | None, Poses]:
    raw = parameters.get(POSES_PARAMETER)
    flattened = flattened_poses(parameters)
    if isinstance(raw, str) and raw.strip():
        if flattened:
            raise ValueError(
                f'{POSES_PARAMETER} names a file and {POSES_PARAMETER}.* entries exist; give one'
            )
        return Path(os.path.expanduser(raw.strip())), {}
    if isinstance(raw, Mapping):
        if flattened:
            raise ValueError(
                f'{POSES_PARAMETER} is given both as a mapping and flattened; give one'
            )
        return None, parse_poses(raw, POSES_PARAMETER)
    if raw is None or raw == '':
        if flattened:
            return None, parse_poses(flattened, POSES_PARAMETER + '.*')
        raise ValueError(
            f'{POSES_PARAMETER}: no poses configured; give a mapping of pose name to joint '
            'positions, or the path of a YAML file holding one'
        )
    raise TypeError(
        f'{POSES_PARAMETER} must be a mapping or a file path, not {type(raw).__name__}'
    )


# --------------------------------------------------------------------------- goal and catalogue


def build_goal(settings: MoveItSettings, pose: str, joints: Mapping[str, float]) -> dict[str, Any]:
    """The ``MoveGroup.Goal`` as a plain dict: ``request`` (MotionPlanRequest) and ``planning_options``."""
    return {
        'pose': pose,
        'request': {
            'group_name': settings.group_name,
            'goal_constraints': [
                {
                    'name': pose,
                    'joint_constraints': [
                        {
                            'joint_name': joint,
                            'position': position,
                            'tolerance_above': JOINT_TOLERANCE_RAD,
                            'tolerance_below': JOINT_TOLERANCE_RAD,
                            'weight': 1.0,
                        }
                        for joint, position in joints.items()
                    ],
                }
            ],
            'num_planning_attempts': settings.num_attempts,
            'allowed_planning_time': settings.planning_time_s,
            'max_velocity_scaling_factor': settings.velocity_scale,
            'max_acceleration_scaling_factor': settings.acceleration_scale,
        },
        'planning_options': {'plan_only': False},
    }


def build_catalogue(poses: Mapping[str, Any]) -> tuple[TaskSpec, ...]:
    """The task catalogue for the given pose names (``home`` only when such a pose exists)."""
    names = sorted(poses)
    specs: list[TaskSpec] = [
        TaskSpec(
            'move_to',
            'Move to named pose',
            'motion',
            MOTION_TIMEOUT_S,
            {
                'type': 'object',
                'properties': {
                    'pose': {
                        'type': 'string',
                        'enum': names,
                        'description': 'Named pose from moveit.poses',
                    }
                },
                'required': ['pose'],
            },
            'Plan and execute a move to a named joint pose through move_group',
        )
    ]
    if HOME_POSE in poses:
        specs.append(
            TaskSpec(
                'home',
                'Move to home',
                'motion',
                MOTION_TIMEOUT_S,
                None,
                f"Plan and execute a move to the pose named '{HOME_POSE}'",
            )
        )
    specs.append(
        TaskSpec(
            'stop',
            'Stop',
            'control',
            CONTROL_TIMEOUT_S,
            None,
            'Cancel the running move and clear any robot error',
        )
    )
    specs.append(
        TaskSpec(
            'refresh_poses',
            'Refresh poses',
            'query',
            CONTROL_TIMEOUT_S,
            None,
            'Re-read the poses; the announced list updates at the next node start',
        )
    )
    return tuple(specs)


# --------------------------------------------------------------------------- the executor


@dataclass
class _Run:
    """The one motion goal in flight."""

    request: TaskRequest
    handle: TaskHandle
    pose: str
    goal: Mapping[str, Any]
    goal_handle: GoalHandle | None = None
    cancel_requested: bool = False
    #: The result to report when move_group confirms the cancel (code and reason by cause).
    on_cancelled: TaskResult | None = None
    #: Set by ``stop`` with ``hold_on_cancel``: clear the robot error once move_group confirms.
    clear_error_on_outcome: bool = False


AdapterFactory = Callable[[Any, str, logging.Logger], MoveGroupAdapter]


class MoveItExecutor(TaskExecutor):
    """Named joint poses through move_group. See the module docstring."""

    name = ENTRY_POINT_NAME
    version = __version__

    def __init__(
        self,
        adapter: MoveGroupAdapter | None = None,
        *,
        adapter_factory: AdapterFactory | None = None,
    ) -> None:
        """``adapter`` (tests) or ``adapter_factory`` replace the rclpy adapter; the node passes neither."""
        self._adapter: MoveGroupAdapter | None = adapter
        self._adapter_factory: AdapterFactory = (
            adapter_factory if adapter_factory is not None else RclpyMoveGroupAdapter
        )
        self._ctx: BridgeContext | None = None
        self._logger = logging.getLogger(_LOGGER_NAME)
        self._settings: MoveItSettings | None = None
        self._poses: Poses = {}
        self._catalogue: tuple[TaskSpec, ...] = ()
        self._active: _Run | None = None
        #: The MoveIt joint names reported, in announce order, and the announced JointInfo.
        self._joint_names: tuple[str, ...] = ()
        self._joint_info: JointInfo | None = None
        self._joint_states_seen = 0
        self._joint_states_skipped = 0
        self._joint_state_warned = False

    # ------------------------------------------------------------------ views

    @property
    def settings(self) -> MoveItSettings:
        if self._settings is None:
            raise RuntimeError('MoveItExecutor.setup() has not been called')
        return self._settings

    @property
    def poses(self) -> Mapping[str, Mapping[str, float]]:
        """The poses currently accepted by ``move_to`` (after any ``refresh_poses``)."""
        return self._poses

    @property
    def active_task_id(self) -> str | None:
        """Id of the motion task whose goal is in flight, or ``None``."""
        return None if self._active is None else self._active.request.id

    @property
    def joint_names(self) -> tuple[str, ...]:
        """The MoveIt joint names forwarded from ``JointState``, in the announced order."""
        return self._joint_names

    @property
    def joint_states_seen(self) -> int:
        """``JointState`` messages forwarded so far (skipped ones not counted)."""
        return self._joint_states_seen

    @property
    def joint_states_skipped(self) -> int:
        """``JointState`` messages skipped because a configured joint was missing."""
        return self._joint_states_skipped

    # ------------------------------------------------------------------ TaskExecutor

    def setup(self, ctx: BridgeContext) -> None:
        self._ctx = ctx
        self._logger = ctx.logger.getChild('moveit') if ctx.logger is not None else self._logger
        values = self._collect_parameters(ctx)
        for name in unknown_parameter_names(values):
            self._logger.warning('parameter %s is not read by the moveit executor; a typo?', name)
        settings = MoveItSettings.from_parameters(values)
        self._poses = settings.load_poses()
        self._settings = settings
        self._catalogue = build_catalogue(self._poses)
        if self._adapter is None:
            if ctx.node is None:
                raise RuntimeError(
                    'MoveItExecutor needs ctx.node (the rclpy node) or an injected adapter'
                )
            self._adapter = self._adapter_factory(ctx.node, settings.action_name, self._logger)
        self._setup_joints(settings)
        self._logger.info(
            'moveit executor ready for %s: group=%s action=%s poses=%s (from %s) '
            'planning_time_s=%g attempts=%d velocity_scale=%g acceleration_scale=%g '
            'hold_on_cancel=%s server_available=%s joints=%s',
            ctx.robot_id,
            settings.group_name,
            settings.action_name,
            ', '.join(sorted(self._poses)),
            settings.poses_source,
            settings.planning_time_s,
            settings.num_attempts,
            settings.velocity_scale,
            settings.acceleration_scale,
            settings.hold_on_cancel,
            self._server_available(),
            ', '.join(self._joint_names) or 'none',
        )

    def tasks(self) -> Sequence[TaskSpec]:
        if self._settings is None:
            raise RuntimeError('MoveItExecutor.setup() has not been called')
        return self._catalogue

    def joints(self) -> JointInfo | None:
        return self._joint_info

    def start(self, req: TaskRequest, handle: TaskHandle) -> None:
        spec = next((s for s in self._catalogue if s.name == req.task), None)
        if spec is None:
            raise TaskRejected(reason=f'unknown task {req.task!r}')
        problems = validate_params(spec.params, req.params)
        if problems:
            raise TaskRejected(reason='; '.join(problems))
        if req.task in ('move_to', 'home'):
            pose = HOME_POSE if req.task == 'home' else str(req.params['pose'])
            joints = self._poses.get(pose)
            if joints is None:
                known = ', '.join(sorted(self._poses)) or 'none'
                raise TaskRejected(reason=f'unknown pose {pose!r}; known poses: {known}')
            self._begin_motion(req, handle, pose, joints)
        elif req.task == 'stop':
            handle.finish(self._stop())
        else:  # refresh_poses
            handle.finish(self._refresh())

    def cancel(self, task_id: str) -> None:
        run = self._active
        if run is None or run.request.id != task_id:
            self._logger.info('cancel for %s: no goal in flight under that id', task_id)
            return
        self._request_cancel(run, TaskResult('cancelled', 'cancelled', 'Cancelled by request'))

    def stop_and_hold(self, reason: str) -> None:
        self._logger.warning('stop and hold: %s', reason)
        run = self._active
        if run is None:
            return
        self._active = None
        self._drop_pending_clear(run, 'stop_and_hold')
        self._cancel_goal(run)
        run.handle.finish(TaskResult('cancelled', 'inhibited', reason, self._base_data(run.pose)))

    def health(self) -> tuple[HealthLevel, str]:
        if self._adapter is None or self._settings is None:
            return ('error', 'moveit executor is not set up')
        try:
            available = self._adapter.server_available()
        except Exception as error:
            self._logger.debug('adapter.server_available raised', exc_info=True)
            return ('error', f'availability check raised {type(error).__name__}: {error}')
        if available:
            return ('ok', '')
        return ('error', f'move_group action server {self._settings.action_name} is not available')

    def shutdown(self) -> None:
        run = self._active
        self._active = None
        if run is not None:
            self._logger.warning(
                'shutdown with task %s in flight: cancelling its goal at move_group',
                run.request.id,
            )
            self._drop_pending_clear(run, 'shutdown')
            self._cancel_goal(run)
        if self._adapter is not None:
            try:
                self._adapter.close()
            except Exception:
                self._logger.exception('adapter.close raised; ignored')

    # ------------------------------------------------------------------ motion

    def _begin_motion(
        self, req: TaskRequest, handle: TaskHandle, pose: str, joints: Mapping[str, float]
    ) -> None:
        stale = self._active
        if stale is not None:
            # The bridge finished that task without us (a forced timeout, for example); the goal
            # may still be running at move_group, and two goals must not overlap.
            self._logger.warning(
                'task %s starts while the goal of task %s is still pending; cancelling the old goal',
                req.id,
                stale.request.id,
            )
            self._active = None
            self._drop_pending_clear(stale, 'a newer task')
            self._cancel_goal(stale)
        adapter = self._require_adapter()
        if not self._server_available():
            handle.finish(
                TaskResult(
                    'aborted',
                    UNAVAILABLE_CODE,
                    f'move_group action server {self.settings.action_name} is not available',
                    {**self._base_data(pose), 'moveit_error_code': None, 'message': ''},
                )
            )
            return
        goal = build_goal(self.settings, pose, joints)
        run = _Run(req, handle, pose, goal)
        self._active = run
        task_id = req.id
        try:
            run.goal_handle = adapter.send_goal(
                goal, lambda state: self._post(lambda: self._on_feedback(task_id, state))
            )
            run.goal_handle.add_done(
                lambda outcome: self._post(lambda: self._on_outcome(task_id, outcome))
            )
        except Exception as error:
            self._logger.exception('task %s: sending the goal raised', task_id)
            if self._active is run:
                self._active = None
            handle.finish(
                TaskResult(
                    'aborted',
                    'executor_error',
                    f'sending the goal raised {type(error).__name__}: {error}',
                    {**self._base_data(pose), 'moveit_error_code': None, 'message': ''},
                )
            )
            return
        self._logger.info('task %s: goal sent for pose %r', task_id, pose)

    def _request_cancel(self, run: _Run, on_cancelled: TaskResult) -> None:
        run.cancel_requested = True
        run.on_cancelled = on_cancelled
        if run.goal_handle is not None:
            # An exception here propagates to the bridge, which arms the cancel grace and
            # forces aborted/executor_unresponsive: honest, because the arm's state is unknown.
            run.goal_handle.cancel()
        if not self.settings.hold_on_cancel:
            self._active = None
            data = {**self._base_data(run.pose), **on_cancelled.data, 'hold_confirmed': False}
            run.handle.finish(
                TaskResult(on_cancelled.status, on_cancelled.code, on_cancelled.reason, data)
            )

    def _cancel_goal(self, run: _Run) -> None:
        """Best-effort cancel of a goal whose task the bridge no longer tracks."""
        if run.goal_handle is None:
            return
        run.cancel_requested = True
        try:
            run.goal_handle.cancel()
        except Exception:
            self._logger.exception('task %s: cancelling the goal raised', run.request.id)

    def _drop_pending_clear(self, run: _Run, why: str) -> None:
        """Log that the error clear a ``stop`` deferred will not happen, because ``why`` abandoned the run."""
        if not run.clear_error_on_outcome:
            return
        run.clear_error_on_outcome = False
        self._logger.warning(
            "task %s: the error clear pending from 'stop' is dropped because %s abandoned its "
            "goal; if the robot is still in error, send 'stop' again",
            run.request.id,
            why,
        )

    def _on_feedback(self, task_id: str, state: str) -> None:
        run = self._active
        if run is None or run.request.id != task_id:
            return
        run.handle.feedback(None, state.strip().lower(), f'move_group state: {state}')

    def _on_outcome(self, task_id: str, outcome: GoalOutcome) -> None:
        run = self._active
        if run is None or run.request.id != task_id:
            self._logger.debug(
                'task %s: outcome for a task that already finished; dropped', task_id
            )
            return
        self._active = None
        run.handle.finish(self._result_for(run, outcome))
        if run.clear_error_on_outcome:
            # The bridge may already have forced this task (the finish above is then dropped);
            # either way move_group has now confirmed the goal ended, which is what 'stop'
            # promised before clearing the error.
            self._logger.info(
                'task %s: move_group confirmed the goal ended; clearing the robot error as '
                "'stop' promised",
                task_id,
            )
            self._context().clear_error()

    def _result_for(self, run: _Run, outcome: GoalOutcome) -> TaskResult:
        data: dict[str, Any] = {
            **self._base_data(run.pose),
            'moveit_error_code': None,
            'message': '',
        }
        if not outcome.accepted:
            if run.cancel_requested and run.on_cancelled is not None:
                # The cancel arrived before move_group answered and the goal was then rejected:
                # nothing was ever executing, so the task ends as the cancel asked.
                pending = run.on_cancelled
                return TaskResult(
                    'cancelled',
                    pending.code,
                    pending.reason,
                    {
                        **data,
                        **pending.data,
                        'message': outcome.detail or 'move_group rejected the goal',
                        'hold_confirmed': True,
                    },
                )
            return TaskResult(
                'aborted',
                UNAVAILABLE_CODE,
                outcome.detail or 'move_group rejected the goal',
                data,
            )
        if outcome.error_code is None:
            return TaskResult(
                'aborted',
                'executor_error',
                outcome.detail or 'move_group returned no error code',
                {**data, 'goal_status': outcome.status},
            )
        mapped = map_error_code(
            outcome.error_code,
            message=outcome.message,
            cancelled_by_bridge=run.cancel_requested,
        )
        if (
            run.cancel_requested
            and outcome.status == STATUS_CANCELED
            and mapped.status == 'aborted'
        ):
            # The goal was cancelled as asked but move_group reported another code with it.
            mapped = MappedCode(
                'cancelled', 'cancelled', mapped.error_class, mapped.moveit_name, mapped.reason
            )
        data.update(
            {
                'moveit_error_code': outcome.error_code,
                'moveit_error_name': mapped.moveit_name,
                'message': outcome.message,
                'goal_status': outcome.status,
                'planning_time_s': outcome.planning_time_s,
                'error_class': mapped.error_class,
            }
        )
        if mapped.status == 'cancelled' and run.on_cancelled is not None:
            pending = run.on_cancelled
            return TaskResult(
                'cancelled',
                pending.code,
                pending.reason,
                {**data, **pending.data, 'hold_confirmed': True},
            )
        if mapped.status == 'succeeded':
            return TaskResult('succeeded', 'ok', f"Moved to pose '{run.pose}'", data)
        return TaskResult(mapped.status, mapped.code, mapped.reason, data)

    # ------------------------------------------------------------------ control and query tasks

    def _stop(self) -> TaskResult:
        run = self._active
        stopped: list[str] = []
        deferred = False
        if run is not None:
            stopped.append(run.request.id)
            self._request_cancel(
                run,
                TaskResult('cancelled', 'preempted', "Stopped by task 'stop'", {'by': 'stop'}),
            )
            # With hold_on_cancel the run is still in flight here; the error then clears in
            # _on_outcome, once move_group confirms, so an idle robot always means the arm is
            # holding. Without it the run has already finished and the clear happens now.
            deferred = self.settings.hold_on_cancel
            if deferred:
                run.clear_error_on_outcome = True
        if not deferred:
            self._context().clear_error()
        if not stopped:
            reason = 'Nothing was running'
        elif deferred:
            reason = (
                f'Stopped {len(stopped)} task(s); the robot error, if any, clears when '
                'move_group confirms the goal ended'
            )
        else:
            reason = f'Stopped {len(stopped)} task(s)'
        return TaskResult(
            'succeeded',
            'ok',
            reason,
            {'stopped': stopped, 'awaiting_confirmation': deferred},
        )

    def _refresh(self) -> TaskResult:
        source = self.settings.poses_source
        try:
            poses = self.settings.load_poses()
        except (OSError, ValueError, TypeError) as error:
            return TaskResult(
                'aborted',
                'executor_error',
                f'poses not reloaded, keeping the current ones: {error}',
                {'source': source, 'poses': sorted(self._poses)},
            )
        added = sorted(set(poses) - set(self._poses))
        removed = sorted(set(self._poses) - set(poses))
        changed = sorted(n for n in poses if n in self._poses and poses[n] != self._poses[n])
        self._poses = poses
        self._catalogue = build_catalogue(poses)
        self._logger.info(
            'poses refreshed from %s: %d pose(s), added %s, removed %s, changed %s',
            source,
            len(poses),
            added or 'none',
            removed or 'none',
            changed or 'none',
        )
        return TaskResult(
            'succeeded',
            'ok',
            f'{len(poses)} pose(s) loaded from {source}; the announced list updates at the '
            'next node start',
            {
                'source': source,
                'poses': sorted(poses),
                'added': added,
                'removed': removed,
                'changed': changed,
            },
        )

    # ------------------------------------------------------------------ joint telemetry

    def _setup_joints(self, settings: MoveItSettings) -> None:
        """Decide the reported joints, announce them and subscribe to the joint states."""
        raw_names = settings.joint_names
        if not raw_names:
            first = next(iter(self._poses.values()), None)
            raw_names = () if first is None else tuple(first)
        announced = announce_joint_names(raw_names)
        if announced is None:
            self._logger.warning(
                'joint telemetry off: the joint names %r cannot all be announced (each must '
                'reduce to a distinct ^[a-z][a-z0-9_]{0,31}$ name, 1 to 32 of them); set '
                'moveit.joint_names to a list that can',
                list(raw_names),
            )
            self._joint_names = ()
            self._joint_info = None
            return
        self._joint_names = tuple(raw_names)
        self._joint_info = JointInfo(names=announced, unit='rad', limits=None)
        subscribe = getattr(self._adapter, 'subscribe_joint_states', None)
        if not callable(subscribe):
            self._logger.warning(
                'the adapter offers no subscribe_joint_states; joints are announced but '
                'never reported'
            )
            return
        subscribe(settings.joint_states_topic, self._on_joint_state)

    def _on_joint_state(
        self, names: Sequence[str], positions: Sequence[float], velocities: Sequence[float] | None
    ) -> None:
        """One ``JointState``: pick the configured joints in announce order and report them.

        Called on whatever thread the adapter uses (rclpy's executor thread
        for the real one); ``ctx.report_joints`` posts to the bridge's main
        thread, and nothing here touches the executor's task state.
        """
        if not self._joint_names:
            return
        index_by_name = {str(name): index for index, name in enumerate(names)}
        try:
            indexes = [index_by_name[joint] for joint in self._joint_names]
            ordered = [float(positions[index]) for index in indexes]
        except (KeyError, IndexError, TypeError, ValueError):
            self._joint_states_skipped += 1
            if not self._joint_state_warned:
                self._joint_state_warned = True
                missing = [joint for joint in self._joint_names if joint not in index_by_name]
                self._logger.warning(
                    'JointState on %s lacks %s; such messages are skipped (logged once)',
                    self.settings.joint_states_topic,
                    ', '.join(missing) or 'a position for a configured joint',
                )
            return
        speeds: list[float] | None = None
        if velocities is not None and len(velocities) >= len(positions):
            try:
                speeds = [float(velocities[index]) for index in indexes]
            except (IndexError, TypeError, ValueError):
                speeds = None
        self._joint_states_seen += 1
        self._context().report_joints(ordered, speeds)

    # ------------------------------------------------------------------ parameters

    def _collect_parameters(self, ctx: BridgeContext) -> dict[str, Any]:
        """``moveit.*`` from the node's params file (real node) overlaid by ``ctx.parameters``."""
        values: dict[str, Any] = {}
        if ctx.node is not None:
            values.update(_node_parameters(ctx.node, self._logger))
        values.update(
            {
                name: value
                for name, value in ctx.parameters.items()
                if isinstance(name, str) and name.startswith(PARAMETER_PREFIX)
            }
        )
        return values

    # ------------------------------------------------------------------ helpers

    def _base_data(self, pose: str) -> dict[str, Any]:
        settings = self._settings
        return {
            'pose': pose,
            'group_name': None if settings is None else settings.group_name,
            'action_name': None if settings is None else settings.action_name,
        }

    def _server_available(self) -> bool:
        adapter = self._adapter
        if adapter is None:
            return False
        try:
            return bool(adapter.server_available())
        except Exception:
            self._logger.exception('adapter.server_available raised; treated as unavailable')
            return False

    def _require_adapter(self) -> MoveGroupAdapter:
        if self._adapter is None:
            raise RuntimeError('MoveItExecutor.setup() has not been called')
        return self._adapter

    def _context(self) -> BridgeContext:
        if self._ctx is None:
            raise RuntimeError('MoveItExecutor.setup() has not been called')
        return self._ctx

    def _post(self, fn: Callable[[], None]) -> None:
        self._context().post(fn)


# --------------------------------------------------------------------------- node parameters


def _node_parameters(node: Any, logger: logging.Logger) -> dict[str, Any]:
    """Declare this executor's parameters on the node and return their effective values.

    The bridge's own parameter table knows nothing about ``moveit.*``, so the
    executor declares them itself (read-only, dynamically typed so a YAML ``5``
    serves a double) and rclpy applies the params-file overrides at
    declaration. Inline poses flatten to ``moveit.poses.<pose>.<joint>``
    names that cannot be declared without knowing them, so they are found in
    the node's parameter overrides first; when those cannot be read, a poses
    file (``moveit.poses`` as a path) is the route that still works.
    """
    overrides = _parameter_overrides(node, logger)
    values: dict[str, Any] = {}
    for name, default, description in SCALAR_PARAMETERS:
        values[name] = _declare(node, name, default, description, logger)
    prefix = POSES_PARAMETER + '.'
    for name, value in overrides.items():
        if name.startswith(prefix):
            values[name] = _declare(
                node, name, value, 'MoveIt executor: a named pose joint.', logger
            )
    return values


def _parameter_overrides(node: Any, logger: logging.Logger) -> dict[str, Any]:
    """``moveit.*`` entries of the node's params files, declared or not (empty when unreadable)."""
    try:
        parameter_cls: Any = importlib.import_module('rclpy.parameter').Parameter
        handle = node.handle
        with handle:
            found = handle.get_parameters(parameter_cls)
    except Exception:
        logger.warning(
            'could not read the node parameter overrides, so inline moveit.poses.* entries in '
            'the params file are not visible; set moveit.poses to a YAML file path instead',
            exc_info=True,
        )
        return {}
    return {
        str(name): parameter.value
        for name, parameter in dict(found).items()
        if str(name).startswith(PARAMETER_PREFIX)
    }


def _declare(node: Any, name: str, default: Any, description: str, logger: logging.Logger) -> Any:
    descriptor_cls: Any = importlib.import_module('rcl_interfaces.msg').ParameterDescriptor
    descriptor = descriptor_cls(
        name=name, description=description, read_only=True, dynamic_typing=True
    )
    try:
        parameter = node.declare_parameter(name, default, descriptor)
    except Exception as error:
        if type(error).__name__ != 'ParameterAlreadyDeclaredException':
            raise
        logger.debug('parameter %s was already declared; reading it', name)
        parameter = node.get_parameter(name)
    return parameter.value


# --------------------------------------------------------------------------- the rclpy adapter


class RclpyMoveGroupAdapter:
    """:class:`MoveGroupAdapter` over ``rclpy.action.ActionClient`` for ``moveit_msgs/action/MoveGroup``.

    rclpy and moveit_msgs are imported here, in the constructor, and nowhere
    else in this module, so the core imports and tests on a machine without
    ROS. The callbacks run on the node's executor thread (rclpy's rule);
    :class:`MoveItExecutor` posts them through the bridge context anyway.
    """

    def __init__(self, node: Any, action_name: str, logger: logging.Logger) -> None:
        action_client_cls: Any = importlib.import_module('rclpy.action').ActionClient
        action_module: Any = importlib.import_module('moveit_msgs.action')
        msg_module: Any = importlib.import_module('moveit_msgs.msg')
        self._move_group: Any = action_module.MoveGroup
        self._constraints_cls: Any = msg_module.Constraints
        self._joint_constraint_cls: Any = msg_module.JointConstraint
        self._logger = logger
        self._node = node
        self.action_name = action_name
        self._client: Any = action_client_cls(node, self._move_group, action_name)
        self._joint_states: Any = None

    def server_available(self) -> bool:
        return bool(self._client.server_is_ready())

    def send_goal(self, goal: Mapping[str, Any], feedback: Callable[[str], None]) -> GoalHandle:
        message = self.build_message(goal)
        handle = _RclpyGoalHandle(self._logger)

        def on_feedback(feedback_message: Any) -> None:
            feedback(str(getattr(feedback_message.feedback, 'state', '')))

        future = self._client.send_goal_async(message, feedback_callback=on_feedback)
        handle.attach(future)
        return handle

    def subscribe_joint_states(self, topic: str, callback: JointStateCallback) -> None:
        """Subscribe to ``sensor_msgs/JointState`` with the sensor-data QoS (best effort).

        A best-effort subscription matches a reliable publisher
        (``joint_state_broadcaster``) as well as a best-effort one, so the
        telemetry works whichever the robot's driver chose. Velocities are
        passed on only when the message carries one per name.
        """
        joint_state_cls: Any = importlib.import_module('sensor_msgs.msg').JointState
        qos: Any = importlib.import_module('rclpy.qos').qos_profile_sensor_data

        def on_message(message: Any) -> None:
            names = [str(name) for name in message.name]
            positions = [float(value) for value in message.position]
            velocities: list[float] | None = None
            if len(message.velocity) == len(names):
                velocities = [float(value) for value in message.velocity]
            callback(names, positions, velocities)

        if self._joint_states is not None:
            self._node.destroy_subscription(self._joint_states)
        self._joint_states = self._node.create_subscription(
            joint_state_cls, topic, on_message, qos
        )

    def close(self) -> None:
        if self._joint_states is not None:
            try:
                self._node.destroy_subscription(self._joint_states)
            except Exception:
                self._logger.exception('could not destroy the joint_states subscription')
            self._joint_states = None
        self._client.destroy()

    def build_message(self, goal: Mapping[str, Any]) -> Any:
        """A ``MoveGroup.Goal`` from the dict :func:`build_goal` produced."""
        message = self._move_group.Goal()
        request = goal['request']
        plan = message.request
        plan.group_name = str(request['group_name'])
        plan.num_planning_attempts = int(request['num_planning_attempts'])
        plan.allowed_planning_time = float(request['allowed_planning_time'])
        plan.max_velocity_scaling_factor = float(request['max_velocity_scaling_factor'])
        plan.max_acceleration_scaling_factor = float(request['max_acceleration_scaling_factor'])
        constraints_list = []
        for constraint_set in request['goal_constraints']:
            constraints = self._constraints_cls()
            constraints.name = str(constraint_set.get('name', ''))
            for entry in constraint_set['joint_constraints']:
                joint = self._joint_constraint_cls()
                joint.joint_name = str(entry['joint_name'])
                joint.position = float(entry['position'])
                joint.tolerance_above = float(entry['tolerance_above'])
                joint.tolerance_below = float(entry['tolerance_below'])
                joint.weight = float(entry['weight'])
                constraints.joint_constraints.append(joint)
            constraints_list.append(constraints)
        plan.goal_constraints = constraints_list
        message.planning_options.plan_only = bool(goal['planning_options']['plan_only'])
        return message


class _RclpyGoalHandle:
    """:class:`GoalHandle` over the two futures of one rclpy action goal."""

    def __init__(self, logger: logging.Logger) -> None:
        self._logger = logger
        self._callbacks: list[Callable[[GoalOutcome], None]] = []
        self._outcome: GoalOutcome | None = None
        self._goal_handle: Any = None
        self._cancel_pending = False

    def attach(self, goal_future: Any) -> None:
        goal_future.add_done_callback(self._on_goal_response)

    def add_done(self, callback: Callable[[GoalOutcome], None]) -> None:
        if self._outcome is not None:
            callback(self._outcome)
            return
        self._callbacks.append(callback)

    def cancel(self) -> None:
        if self._outcome is not None:
            return
        if self._goal_handle is None:
            # Not accepted yet: cancel as soon as the goal response arrives.
            self._cancel_pending = True
            return
        self._send_cancel()

    def _send_cancel(self) -> None:
        self._cancel_pending = False
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self._on_cancel_response)

    def _on_cancel_response(self, future: Any) -> None:
        try:
            response = future.result()
        except Exception:
            self._logger.exception('cancel_goal response raised; the result decides')
            return
        self._logger.debug(
            'cancel_goal answered with return_code %s', getattr(response, 'return_code', '?')
        )

    def _on_goal_response(self, future: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            self._logger.exception('send_goal raised')
            self._finish(
                GoalOutcome(
                    accepted=False, detail=f'send_goal raised {type(error).__name__}: {error}'
                )
            )
            return
        if goal_handle is None or not getattr(goal_handle, 'accepted', False):
            self._finish(GoalOutcome(accepted=False, detail='move_group rejected the goal'))
            return
        self._goal_handle = goal_handle
        try:
            result_future = goal_handle.get_result_async()
        except Exception as error:
            self._logger.exception('get_result_async raised')
            self._finish(
                GoalOutcome(detail=f'get_result_async raised {type(error).__name__}: {error}')
            )
            return
        result_future.add_done_callback(self._on_result)
        if self._cancel_pending:
            self._send_cancel()

    def _on_result(self, future: Any) -> None:
        try:
            wrapped = future.result()
        except Exception as error:
            self._logger.exception('the result future raised')
            self._finish(
                GoalOutcome(detail=f'the result future raised {type(error).__name__}: {error}')
            )
            return
        result = getattr(wrapped, 'result', None)
        error_code = getattr(result, 'error_code', None)
        val = getattr(error_code, 'val', None)
        planning_time = getattr(result, 'planning_time', None)
        self._finish(
            GoalOutcome(
                accepted=True,
                status=int(getattr(wrapped, 'status', 0)),
                error_code=int(val) if isinstance(val, int) else None,
                message=str(getattr(error_code, 'message', '') or ''),
                planning_time_s=float(planning_time) if _is_number(planning_time) else None,
                detail=''
                if isinstance(val, int)
                else 'the result carries no MoveItErrorCodes value',
            )
        )

    def _finish(self, outcome: GoalOutcome) -> None:
        if self._outcome is not None:
            return
        self._outcome = outcome
        callbacks, self._callbacks = self._callbacks, []
        for callback in callbacks:
            try:
                callback(outcome)
            except Exception:
                self._logger.exception('goal done callback raised; ignored')
