# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The bridge node: one rclpy process per robot, wiring the core modules to MQTT and ROS.

:class:`BridgeNode` contains no robot logic of its own. It declares the
parameters (:mod:`ros2_ha_bridge.parameters`), loads the executor plugin, and
wires the rclpy-free core (:class:`~ros2_ha_bridge.authoriser.Authoriser`,
:class:`~ros2_ha_bridge.task_manager.TaskManager`,
:class:`~ros2_ha_bridge.dedupe_store.DedupeStore`,
:class:`~ros2_ha_bridge.watchdog.Watchdog`,
:class:`~ros2_ha_bridge.sensor_mirror.SensorMirror`) to the
:class:`~ros2_ha_bridge.mqtt_link.MqttLink` on one side and to rclpy timers,
publishers and guard conditions on the other, through the adapters in
:mod:`ros2_ha_bridge.ros_adapters`.

Threading model (docs/architecture.md): paho's network thread only appends to
the link's bounded deque and triggers a guard condition; the guard's callback
runs on the ``SingleThreadedExecutor`` thread, drains the deque and dispatches
each item. Commands go through the authoriser and the task manager, Home
Assistant heartbeats feed the watchdog, HA's birth message schedules a
re-announce after a random jitter, mirrored topics become ``std_msgs/String``
envelopes on ``~/mqtt_in``, and a successful connect publishes the retained
availability, announce and state. Every publish goes through the link's
guarded ``publish``, so nothing on the wire can end the process; every timer
and guard callback is wrapped so an exception is logged, never propagated out
of the executor. Every periodic timer and every one-shot runs on one
``STEADY_TIME`` clock shared with the bridge clock
(:mod:`ros2_ha_bridge.ros_adapters`), never on the node's ``ROS_TIME`` clock,
so a wall-clock step cannot stretch or cut a deadline, a heartbeat or the
watchdog.

Lifecycle: construction does everything that can fail (parameters, the
executor's ``setup``, the announce size check, the TLS configuration) and
raises :class:`~ros2_ha_bridge.parameters.ConfigError` for a fatal
misconfiguration; :meth:`BridgeNode.start` opens the MQTT connection and arms
the periodic timers; :meth:`BridgeNode.shutdown` runs the design's clean
shutdown order (stop inbound, ``availability offline``, ``disconnect()``,
``loop_stop()``, ``executor.shutdown()``, dedupe flush) and
``destroy_node()`` finishes it. :func:`main` spins the node until
:attr:`BridgeNode.exit_future` completes: a session takeover past
``mqtt.takeover_exit_after`` or a missing CONNACK within
``mqtt.connect_timeout_s`` ends the process with a non-zero status so the
launch file's ``respawn_max_retries`` can stop a flapping node.

The paho client class is injectable (``client_factory``) so the in-process
tests run the whole node against ``test/fakes.py``'s broker.
"""

from __future__ import annotations

import importlib
import logging
import os
import random
from collections.abc import Callable, Sequence
from dataclasses import replace
from types import MappingProxyType
from typing import Any

import rclpy
from rclpy.exceptions import ParameterException
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from std_msgs.msg import Bool, String

from ros2_ha_bridge import __version__
from ros2_ha_bridge.authoriser import Ack, Authoriser, AuthoriserConfig, Drop, Ignore
from ros2_ha_bridge.clock import TimerHandle
from ros2_ha_bridge.dedupe_store import DedupeStore
from ros2_ha_bridge.executors.base import (
    BridgeContext,
    ExecutorLoadError,
    TaskExecutor,
    TaskSpec,
    load_executor,
)
from ros2_ha_bridge.mqtt_link import (
    ConnectEvent,
    DisconnectEvent,
    InboundItem,
    InboundMessage,
    MqttLink,
    MqttLinkConfig,
    SubscriptionSpec,
)
from ros2_ha_bridge.parameters import (
    ENV_COMMAND_SECRET,
    ENV_MQTT_PASSWORD,
    PARAMETERS,
    SECRET_PARAMETERS,
    BridgeSettings,
    ConfigError,
    ParameterSpec,
    executor_parameters,
    format_settings,
    settings_from_mapping,
)
from ros2_ha_bridge.protocol import (
    AVAILABILITY_ONLINE,
    HEARTBEAT_EXPIRY_S,
    MAX_ANNOUNCE_BYTES,
    Announce,
    Capabilities,
    Counters,
    DecodeError,
    ExecutorInfo,
    HeartbeatPayload,
    RobotTopics,
    decode_ha_heartbeat,
    encode_payload,
    local_instance,
    new_session,
    validate_task_spec,
)
from ros2_ha_bridge.ros_adapters import (
    MainThreadPost,
    RclpyClock,
    RclpyLogHandler,
    RclpyScheduler,
)
from ros2_ha_bridge.sensor_mirror import SensorMirror
from ros2_ha_bridge.task_manager import TaskManager, TaskManagerSettings
from ros2_ha_bridge.watchdog import TICK_S, Watchdog

__all__ = [
    'DEFAULT_NODE_NAME',
    'EXIT_CONFIG',
    'EXIT_CONNECT_TIMEOUT',
    'EXIT_OK',
    'EXIT_TAKEOVER',
    'HA_LINK_TOPIC',
    'BridgeNode',
    'ConfigError',
    'main',
]

#: Node name under ``ros2 run``; launch files set their own.
DEFAULT_NODE_NAME = 'bridge_node'
#: The latched ``std_msgs/Bool`` that mirrors the HA link.
HA_LINK_TOPIC = '~/ha_link'
#: Process exit statuses. Anything non-zero makes the launch file's respawn count a retry.
EXIT_OK = 0
EXIT_CONFIG = 2
EXIT_CONNECT_TIMEOUT = 3
EXIT_TAKEOVER = 4

_LOGGER_NAME = 'ros2_ha_bridge.bridge_node'
_MQTT_IN_QUEUE_DEPTH = 10

# rcl_interfaces ships no type information and is not in the mypy override list the
# repository configures for rclpy, so the two message classes are fetched by name.
_rcl_interfaces_msg = importlib.import_module('rcl_interfaces.msg')
_ParameterDescriptor: Any = _rcl_interfaces_msg.ParameterDescriptor
_ParameterType: Any = _rcl_interfaces_msg.ParameterType

_PARAMETER_TYPES: dict[str, int] = {
    'string': _ParameterType.PARAMETER_STRING,
    'integer': _ParameterType.PARAMETER_INTEGER,
    'double': _ParameterType.PARAMETER_DOUBLE,
    'bool': _ParameterType.PARAMETER_BOOL,
    'string_array': _ParameterType.PARAMETER_STRING_ARRAY,
}
#: Kinds declared with ``dynamic_typing`` so a YAML ``2`` is accepted for a double and an
#: empty list (which rclpy types as a byte array) for a string array; the values are
#: checked by ``parameters.settings_from_mapping`` instead.
_DYNAMIC_KINDS = frozenset({'double', 'string_array'})


class BridgeNode(Node):  # type: ignore[misc]
    """One robot's bridge. See the module docstring for what it wires and in which order.

    ``client_factory`` replaces ``paho.mqtt.client.Client`` (the tests pass
    ``FakeBroker.client_factory``). Construction raises
    :class:`~ros2_ha_bridge.parameters.ConfigError` for a fatal
    misconfiguration after destroying the half-built node.
    """

    def __init__(
        self,
        node_name: str = DEFAULT_NODE_NAME,
        *,
        context: Any = None,
        namespace: str | None = None,
        parameter_overrides: Sequence[Any] | None = None,
        client_factory: Callable[..., Any] | None = None,
    ) -> None:
        super().__init__(
            node_name,
            context=context,
            namespace=namespace,
            parameter_overrides=list(parameter_overrides) if parameter_overrides else None,
        )
        # Bridge-owned attributes never reuse a name rclpy's Node keeps for itself (_logger,
        # _clock, _timers, ...), or the node's own clock, logger or timer registry would be
        # replaced; test_bridge_node.py checks this against a plain node.
        self._log_handler = RclpyLogHandler.attach(self)
        self._bridge_logger = logging.getLogger(_LOGGER_NAME)
        self._exit_future: Any = Future()
        self._exit_code = EXIT_OK
        self._exit_reason: str | None = None
        self._started = False
        self._closed = False
        self._bridge_timers: list[Any] = []
        self._connect_timeout: TimerHandle | None = None
        self._announce_timer: TimerHandle | None = None
        self._heartbeat_seq = 0
        self._heartbeat_warned: set[str] = set()
        try:
            self._build(client_factory)
        except BaseException:
            self._abandon()
            raise

    # ------------------------------------------------------------------ construction

    def _build(self, client_factory: Callable[..., Any] | None) -> None:
        values = self._declare_parameters()
        self.settings: BridgeSettings = settings_from_mapping(
            values, environ=os.environ, logger=self._bridge_logger
        )
        settings = self.settings
        self._bridge_logger.info(
            'ros2_ha_bridge %s starting for robot %r: %s',
            __version__,
            settings.robot.id,
            format_settings(settings),
        )
        if not settings.command.hmac_required:
            self._bridge_logger.info(
                'command.secret is empty: commands are not signed (set %s or command.secret '
                'to require signed commands)',
                ENV_COMMAND_SECRET,
            )
        self._warn_about_secret_parameters(settings)

        self.session = new_session()
        self.instance = local_instance()
        self._topics = RobotTopics(settings.robot.id, settings.topic_prefix)
        self._counters = Counters()
        self._bridge_clock = RclpyClock()
        self._scheduler = RclpyScheduler(
            self, clock=self._bridge_clock.steady, logger=self._bridge_logger
        )
        self._post = MainThreadPost(self, logger=self._bridge_logger)
        self._started_monotonic = self._bridge_clock.monotonic()

        self._plugin = self._load_executor(settings.executor.spec)
        self._dedupe = DedupeStore(
            clock=self._bridge_clock,
            data_dir=settings.dedupe_dir,
            max_size=settings.command.dedupe_size,
            window_s=settings.command.dedupe_window_s,
            keep_on_load_ms=settings.command.keep_on_load_ms,
            scheduler=self._scheduler,
        )
        if self._dedupe.loaded:
            self._bridge_logger.info(
                'dedupe store: %d recent command id(s) loaded', self._dedupe.loaded
            )
        self._manager = TaskManager(
            robot_id=settings.robot.id,
            executor=self._plugin,
            publish=self._publish_json,
            post=self._post.post,
            clock=self._bridge_clock,
            scheduler=self._scheduler,
            session=self.session,
            instance=self.instance,
            prefix=settings.topic_prefix,
            watchdog_mode=settings.watchdog.mode,
            settings=TaskManagerSettings(
                cancel_grace_s=settings.executor.cancel_grace_s,
                call_warn_s=settings.executor.call_warn_s,
                health_poll_s=settings.executor.health_poll_s,
                feedback_max_rate_hz=settings.reply.feedback_max_rate_hz,
                result_expiry_s=settings.reply.result_expiry_s,
            ),
        )
        self._watchdog = Watchdog(
            clock=self._bridge_clock,
            mode=settings.watchdog.mode,
            timeout_s=settings.watchdog.timeout_s,
            target=self._manager,
            on_link_change=self._on_ha_link_change,
        )
        context = self._manager.bridge_context(
            ha_link_ok=self._watchdog.ha_link_ok,
            parameters=MappingProxyType(executor_parameters(settings)),
            node=self,
        )
        self._setup_executor(context)
        catalogue = self._catalogue(settings.executor.tasks_allow)
        self._authoriser = Authoriser(
            topics=self._topics,
            session=self.session,
            clock=self._bridge_clock,
            dedupe=self._dedupe,
            tasks=catalogue,
            config=AuthoriserConfig(
                max_payload_bytes=settings.mqtt.max_payload_bytes,
                secret=settings.command.secret,
                max_age_cap_ms=settings.command.max_age_cap_ms,
                clock_skew_tolerance_ms=settings.command.clock_skew_tolerance_ms,
                task_timeout_cap_s=settings.command.task_timeout_cap_s,
                retained_rejections_per_minute=settings.reply.retained_rejections_per_minute,
            ),
            state_view=self._manager.gating_state,
            counters=self._counters,
        )
        self._announce = self._build_announce()

        self._mirror = SensorMirror(
            settings.mirror.topics,
            self._publish_mqtt_in,
            clock=self._bridge_clock,
            availability_suffix=settings.mirror.availability_suffix,
            max_payload_bytes=settings.mqtt.max_payload_bytes,
        )
        self._mqtt_in = self.create_publisher(
            String, settings.mirror.ros_topic, _MQTT_IN_QUEUE_DEPTH
        )
        self._ha_link_publisher = self.create_publisher(
            Bool,
            HA_LINK_TOPIC,
            QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._link_guard: Any = self.create_guard_condition(self._drain_link)
        self._link = self._build_link(client_factory)

    def _declare_parameters(self) -> dict[str, Any]:
        values: dict[str, Any] = {}
        for spec in PARAMETERS:
            try:
                parameter = self.declare_parameter(
                    spec.name, spec.default_value, _descriptor(spec)
                )
            except ParameterException as error:
                raise ConfigError(f'parameter {spec.name}: {error}') from error
            values[spec.name] = parameter.value
        # The two secrets are read once and undeclared at once, so a value given through the
        # params file does not stay in the parameter server, where any participant on the DDS
        # domain could read it with ros2 param get. _warn_about_secret_parameters says why the
        # environment variables are the better route.
        for name in sorted(SECRET_PARAMETERS):
            self.undeclare_parameter(name)
        return values

    def _warn_about_secret_parameters(self, settings: BridgeSettings) -> None:
        """One WARN per secret that arrived as a parameter instead of from the environment."""
        given = (
            (
                'mqtt.password',
                bool(settings.mqtt.password) and not settings.mqtt.password_from_env,
                ENV_MQTT_PASSWORD,
            ),
            (
                'command.secret',
                bool(settings.command.secret) and not settings.command.secret_from_env,
                ENV_COMMAND_SECRET,
            ),
        )
        for name, from_parameter, variable in given:
            if from_parameter:
                self._bridge_logger.warning(
                    '%s was given as a ROS parameter: the value sits in the params file and was '
                    'readable through ros2 param until this node undeclared it; set the %s '
                    'environment variable instead and leave the parameter empty',
                    name,
                    variable,
                )

    def _load_executor(self, spec: str) -> TaskExecutor:
        try:
            executor = load_executor(spec)
        except ExecutorLoadError as error:
            raise ConfigError(str(error)) from error
        self._bridge_logger.info(
            'executor %r loaded: %s %s', spec, executor.name, executor.version
        )
        return executor

    def _setup_executor(self, context: BridgeContext) -> None:
        try:
            self._plugin.setup(context)
        except Exception as error:
            self._bridge_logger.exception('executor.setup raised')
            raise ConfigError(
                f'executor {self.settings.executor.spec!r} failed to set up: '
                f'{type(error).__name__}: {error}'
            ) from error

    def _catalogue(self, allow: tuple[str, ...]) -> list[TaskSpec]:
        """The executor's tasks, validated, after the ``tasks.allow`` intersection."""
        try:
            offered = list(self._plugin.tasks())
        except Exception as error:
            raise ConfigError(
                f'executor.tasks() raised {type(error).__name__}: {error}'
            ) from error
        problems = [problem for spec in offered for problem in validate_task_spec(spec)]
        names = [spec.name for spec in offered]
        if len(set(names)) != len(names):
            problems.append('two tasks share a name')
        if problems:
            raise ConfigError(
                'the executor announces an invalid task catalogue: ' + '; '.join(problems)
            )
        if not allow:
            return offered
        catalogue = [spec for spec in offered if spec.name in allow]
        unknown = [name for name in allow if name not in names]
        if unknown:
            self._bridge_logger.warning(
                'tasks.allow names tasks the executor does not offer: %s', ', '.join(unknown)
            )
        return catalogue

    def _build_announce(self) -> Announce:
        robot = self.settings.robot
        announce = Announce(
            robot_id=robot.id,
            name=robot.name,
            manufacturer=robot.manufacturer,
            model=robot.model,
            sw_version=robot.sw_version,
            hw_version=robot.hw_version,
            serial_number=robot.serial_number,
            suggested_area=robot.suggested_area,
            bridge_version=__version__,
            executor=ExecutorInfo(str(self._plugin.name), str(self._plugin.version)),
            session=self.session,
            instance=self.instance,
            capabilities=Capabilities(hmac_required=self.settings.command.hmac_required),
            watchdog_mode=self.settings.watchdog.mode,
            heartbeat_interval_s=self.settings.heartbeat_interval_s,
            tasks=self._authoriser.catalogue,
            ts=self._bridge_clock.now_ms(),
        )
        size = len(encode_payload(announce.to_payload()))
        if size > MAX_ANNOUNCE_BYTES:
            raise ConfigError(
                f'the announce payload is {size} bytes, over the contract limit of '
                f'{MAX_ANNOUNCE_BYTES}; shorten the task catalogue (tasks.allow) or its '
                'descriptions'
            )
        self._bridge_logger.info(
            'announcing %d task(s): %s',
            len(announce.tasks),
            ', '.join(spec.name for spec in announce.tasks) or 'none',
        )
        return announce

    def _build_link(self, client_factory: Callable[..., Any] | None) -> MqttLink:
        mqtt = self.settings.mqtt
        config = MqttLinkConfig(
            host=mqtt.host,
            client_id=mqtt.client_id,
            port=mqtt.port,
            username=mqtt.username,
            password=mqtt.password,
            tls_enabled=mqtt.tls.enabled,
            tls_ca_cert=mqtt.tls.ca_cert,
            tls_certfile=mqtt.tls.certfile,
            tls_keyfile=mqtt.tls.keyfile,
            tls_insecure=mqtt.tls.insecure,
            keepalive_s=mqtt.keepalive_s,
            reconnect_min_s=mqtt.reconnect_min_s,
            reconnect_max_s=mqtt.reconnect_max_s,
            takeover_exit_after=mqtt.takeover_exit_after,
            max_payload_bytes=mqtt.max_payload_bytes,
            inbound_queue_max=mqtt.inbound_queue_max,
        )
        subscriptions = [
            SubscriptionSpec(self._topics.cmd, 1, retain_as_published=True),
            SubscriptionSpec(self._topics.ha_heartbeat, 0),
            SubscriptionSpec(self.settings.ha_status_topic, 0),
            *(SubscriptionSpec(topic, qos) for topic, qos in self._mirror.subscriptions()),
        ]
        try:
            return MqttLink(
                config,
                availability_topic=self._topics.availability,
                wake=self._link_guard.trigger,
                counters=self._counters,
                clock=self._bridge_clock,
                subscriptions=subscriptions,
                client_factory=client_factory,
            )
        except (OSError, ValueError) as error:
            raise ConfigError(f'mqtt: cannot configure the connection: {error}') from error

    def _abandon(self) -> None:
        """Best-effort cleanup of a node whose construction failed."""
        plugin = getattr(self, '_plugin', None)
        if plugin is not None:
            try:
                plugin.shutdown()
            except Exception:
                self._bridge_logger.exception('executor.shutdown raised while abandoning the node')
        post = getattr(self, '_post', None)
        if post is not None:
            post.close()
        self._log_handler.detach()
        try:
            self.destroy_node()
        except Exception:
            self._bridge_logger.exception('destroy_node raised while abandoning the node')

    # ------------------------------------------------------------------ views

    @property
    def topics(self) -> RobotTopics:
        return self._topics

    @property
    def counters(self) -> Counters:
        """The heartbeat counters (shared with the link and the authoriser)."""
        return self._counters

    @property
    def bridge_clock(self) -> RclpyClock:
        """The bridge clock; its ``steady`` rclpy clock drives every timer of this node."""
        return self._bridge_clock

    @property
    def link(self) -> MqttLink:
        return self._link

    @property
    def manager(self) -> TaskManager:
        return self._manager

    @property
    def watchdog(self) -> Watchdog:
        return self._watchdog

    @property
    def executor_plugin(self) -> TaskExecutor:
        return self._plugin

    @property
    def exit_future(self) -> Any:
        """Completes with the exit status when the node asks to end the process."""
        return self._exit_future

    @property
    def exit_code(self) -> int:
        return self._exit_code

    @property
    def exit_reason(self) -> str | None:
        return self._exit_reason

    @property
    def started(self) -> bool:
        return self._started

    @property
    def closed(self) -> bool:
        return self._closed

    # ------------------------------------------------------------------ lifecycle

    def start(self) -> None:
        """Arm the timers and open the MQTT connection; call once, after adding to an executor."""
        if self._started or self._closed:
            return
        self._started = True
        settings = self.settings
        # The first evaluation pushes the starting link state (and, in the active modes, the
        # inhibit) to the manager and the latched ROS topic before anything else happens.
        self._watchdog.tick()
        # The periodic timers run on the bridge's steady clock, like the scheduler's one-shots,
        # so they share the time base the core's timeouts and rate limits are measured on.
        steady = self._bridge_clock.steady
        self._bridge_timers.append(
            self.create_timer(
                TICK_S, self._guarded('watchdog tick', self._watchdog.tick), clock=steady
            )
        )
        self._bridge_timers.append(
            self.create_timer(
                settings.executor.health_poll_s,
                self._guarded('health poll', self._manager.poll_health),
                clock=steady,
            )
        )
        if settings.heartbeat_interval_s > 0:
            self._bridge_timers.append(
                self.create_timer(
                    settings.heartbeat_interval_s,
                    self._guarded('heartbeat', self._publish_heartbeat),
                    clock=steady,
                )
            )
        else:
            self._bridge_logger.info('heartbeat.interval_s is 0: no robot heartbeat is published')
        if settings.mqtt.connect_timeout_s > 0:
            self._connect_timeout = self._scheduler.call_later(
                settings.mqtt.connect_timeout_s, self._on_connect_timeout
            )
        self._link.start()

    def shutdown(self) -> None:
        """The clean shutdown order; ``destroy_node()`` completes it. Idempotent, never raises."""
        if self._closed:
            return
        self._closed = True
        self._bridge_logger.info('shutting down (%s)', self._exit_reason or 'requested')
        for timer in self._bridge_timers:
            try:
                timer.cancel()
                self.destroy_timer(timer)
            except Exception:
                self._bridge_logger.exception('could not destroy a timer at shutdown')
        self._bridge_timers.clear()
        for handle in (self._connect_timeout, self._announce_timer):
            if handle is not None:
                handle.cancel()
        self._connect_timeout = self._announce_timer = None
        self._watchdog.stop()
        self._manager.stop_health_polling()
        self._link.shutdown()
        try:
            self._plugin.shutdown()
        except Exception:
            self._bridge_logger.exception('executor.shutdown raised; ignored')
        try:
            if self._dedupe.flush():
                self._bridge_logger.info('dedupe store flushed')
        except Exception:
            self._bridge_logger.exception('dedupe flush raised; ignored')
        self._post.close()
        try:
            self.destroy_guard_condition(self._link_guard)
        except Exception:
            self._bridge_logger.exception('could not destroy the link guard condition')
        self._bridge_logger.info('bridge node for robot %r stopped', self.settings.robot.id)
        self._log_handler.detach()

    def destroy_node(self) -> None:
        """Run :meth:`shutdown` if it has not run yet, then destroy the rclpy node."""
        if getattr(self, '_link', None) is not None and not self._closed:
            self.shutdown()
        super().destroy_node()

    def _request_exit(self, code: int, reason: str) -> None:
        if self._exit_future.done():
            return
        self._exit_code = code
        self._exit_reason = reason
        self._bridge_logger.error('exiting with status %d: %s', code, reason)
        self._exit_future.set_result(code)

    def _on_connect_timeout(self) -> None:
        self._connect_timeout = None
        if self._closed or self._link.connack_seen:
            return
        mqtt = self.settings.mqtt
        self._request_exit(
            EXIT_CONNECT_TIMEOUT,
            f'no CONNACK from the MQTT broker at {mqtt.host}:{mqtt.port} within '
            f'mqtt.connect_timeout_s ({mqtt.connect_timeout_s:g} s)',
        )

    # ------------------------------------------------------------------ inbound (main thread)

    def _drain_link(self) -> None:
        for item in self._link.drain():
            if self._closed:
                return
            try:
                self._dispatch(item)
            except Exception:
                self._bridge_logger.exception(
                    'handling an inbound %s raised; ignored', type(item).__name__
                )

    def _dispatch(self, item: InboundItem) -> None:
        if isinstance(item, InboundMessage):
            self._on_message(item)
        elif isinstance(item, ConnectEvent):
            self._on_connect_event(item)
        else:
            self._on_disconnect_event(item)

    def _on_message(self, item: InboundMessage) -> None:
        topic = item.topic
        if topic == self._topics.cmd:
            self._on_command(item)
            return
        handled = False
        if topic == self._topics.ha_heartbeat:
            self._on_ha_heartbeat(item)
            handled = True
        elif topic == self.settings.ha_status_topic:
            self._on_ha_status(item)
            handled = True
        if self._mirror.matches(topic):
            self._mirror.handle(topic, item.payload, retain=item.retain, rx_ts_ms=item.rx_ts_ms)
        elif not handled:
            self._bridge_logger.debug(
                'ignoring a message on %r: no handler subscribes to it', topic
            )

    def _on_command(self, item: InboundMessage) -> None:
        decision = self._authoriser.handle(item.payload, topic=item.topic, retain=item.retain)
        if isinstance(decision, Ignore):
            self._bridge_logger.debug('command topic: %s', decision.reason)
            return
        if isinstance(decision, Drop):
            if decision.clear_retained:
                self._link.publish(self._topics.cmd, b'', qos=1, retain=True)
            return
        ack = decision
        if ack.authorised:
            ack = self._complete_authorised(ack)
        self._publish_ack(ack)

    def _complete_authorised(self, ack: Ack) -> Ack:
        """Steps 10 and 11: the task manager decides, then the authoriser commits or rejects."""
        command = ack.command
        assert command is not None
        received_at = int(ack.payload['received_at'])
        if command.is_cancel:
            reply = self._manager.cancel(command, received_at=received_at)
        else:
            assert ack.request is not None and ack.spec is not None
            reply = self._manager.start(ack.request, ack.spec, received_at=received_at)
        if reply.accepted:
            return self._authoriser.commit(ack)
        return self._authoriser.reject(ack, reply.code, reply.reason)

    def _publish_ack(self, ack: Ack) -> None:
        self._link.publish(
            ack.topic,
            ack.wire,
            qos=1,
            retain=ack.retain,
            expiry_s=self.settings.reply.ack_expiry_s,
        )

    def _on_ha_heartbeat(self, item: InboundMessage) -> None:
        beat = decode_ha_heartbeat(item.payload, self.settings.mqtt.max_payload_bytes)
        if isinstance(beat, DecodeError):
            already = beat.kind in self._heartbeat_warned
            log = self._bridge_logger.debug if already else self._bridge_logger.warning
            self._heartbeat_warned.add(beat.kind)
            log('ignoring an HA heartbeat (%s): %s', beat.kind, beat.reason)
            return
        self._watchdog.feed(beat)

    def _on_ha_status(self, item: InboundMessage) -> None:
        word = item.payload.strip().lower()
        if word == AVAILABILITY_ONLINE:
            self._schedule_reannounce()
        else:
            self._bridge_logger.info(
                'Home Assistant reports %r on %s', word.decode('utf-8', 'replace'), item.topic
            )

    def _schedule_reannounce(self) -> None:
        if self._announce_timer is not None:
            self._bridge_logger.debug('HA is online; a re-announce is already pending')
            return
        delay = random.uniform(0.0, self.settings.announce_jitter_s)
        self._bridge_logger.info('Home Assistant is online; re-announcing in %.2f s', delay)
        self._announce_timer = self._scheduler.call_later(delay, self._reannounce)

    def _reannounce(self) -> None:
        self._announce_timer = None
        if self._closed:
            return
        self._publish_announce()

    def _on_connect_event(self, event: ConnectEvent) -> None:
        if self._connect_timeout is not None:
            self._connect_timeout.cancel()
            self._connect_timeout = None
        if not event.ok:
            return
        self._link.publish(self._topics.availability, AVAILABILITY_ONLINE, qos=1, retain=True)
        self._publish_announce()
        self._manager.publish_state()
        self._bridge_logger.info(
            'connected (connect %d): availability online, announce and state published',
            self._link.connects,
        )

    def _on_disconnect_event(self, event: DisconnectEvent) -> None:
        if event.exit_requested:
            self._request_exit(
                EXIT_TAKEOVER, self._link.exit_reason or 'MQTT session taken over repeatedly'
            )

    # ------------------------------------------------------------------ outbound (main thread)

    def _publish_json(
        self, topic: str, payload: Any, qos: int, retain: bool, expiry_s: int | None
    ) -> bool:
        """The task manager's publish: the link's guarded JSON publish.

        The retained state is republished in full on every connect, so a state
        change while disconnected is not handed to paho (it would only be
        logged as a failed publish); acks, feedback and results always are,
        because paho queues QoS 1 messages for the reconnect.
        """
        if topic == self._topics.state and not self._link.connected:
            self._bridge_logger.debug(
                'state changed while disconnected; it is republished on connect'
            )
            return False
        return self._link.publish_json(topic, payload, qos, retain, expiry_s)

    def _publish_announce(self) -> bool:
        payload = replace(self._announce, ts=self._bridge_clock.now_ms()).to_payload()
        return self._link.publish_json(self._topics.announce, payload, 1, True, None)

    def _publish_heartbeat(self) -> None:
        if not self._link.connected:
            self._bridge_logger.debug('heartbeat skipped: not connected')
            return
        self._heartbeat_seq += 1
        payload = HeartbeatPayload(
            robot_id=self.settings.robot.id,
            session=self.session,
            instance=self.instance,
            seq=self._heartbeat_seq,
            ts=self._bridge_clock.now_ms(),
            uptime_s=round(max(0.0, self._bridge_clock.monotonic() - self._started_monotonic), 3),
            counters=self._counters,
            ha_clock_offset_ms=self._watchdog.ha_clock_offset_ms,
        ).to_payload()
        self._link.publish_json(self._topics.heartbeat, payload, 0, False, HEARTBEAT_EXPIRY_S)

    def _publish_mqtt_in(self, text: str) -> None:
        message = String()
        message.data = text
        self._mqtt_in.publish(message)

    def _on_ha_link_change(self, up: bool) -> None:
        message = Bool()
        message.data = bool(up)
        self._ha_link_publisher.publish(message)

    # ------------------------------------------------------------------ helpers

    def _guarded(self, name: str, fn: Callable[[], object]) -> Callable[[], None]:
        """Wrap a timer callback so an exception is logged and never leaves the executor."""

        def run() -> None:
            if self._closed:
                return
            try:
                fn()
            except Exception:
                self._bridge_logger.exception('%s raised; ignored', name)

        return run


def _descriptor(spec: ParameterSpec) -> Any:
    """The rcl_interfaces descriptor for one row of the table.

    Every parameter is read-only (read once at start) except the two secrets,
    which the node undeclares as soon as it has read them; rclpy refuses to
    undeclare a read-only parameter.
    """
    dynamic = spec.kind in _DYNAMIC_KINDS
    descriptor = _ParameterDescriptor(
        name=spec.name,
        description=spec.description,
        read_only=spec.name not in SECRET_PARAMETERS,
        dynamic_typing=dynamic,
    )
    if not dynamic:
        descriptor.type = _PARAMETER_TYPES[spec.kind]
    return descriptor


# --------------------------------------------------------------------------- entry point


def main(args: list[str] | None = None) -> int:
    """``ros2 run ros2_ha_bridge bridge_node``: spin one node until it asks to exit."""
    rclpy.init(args=args)
    code = EXIT_OK
    try:
        try:
            node = BridgeNode()
        except ConfigError as error:
            get_logger(DEFAULT_NODE_NAME).fatal(f'configuration error: {error}')
            return EXIT_CONFIG
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            node.start()
            executor.spin_until_future_complete(node.exit_future)
            code = node.exit_code
        except (KeyboardInterrupt, ExternalShutdownException):
            code = EXIT_OK
        finally:
            node.shutdown()
            executor.remove_node(node)
            executor.shutdown(timeout_sec=1.0)
            node.destroy_node()
    finally:
        rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
