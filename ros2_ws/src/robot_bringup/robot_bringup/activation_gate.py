"""Activate and recover Nav2 after time-aware readiness checks."""

from functools import partial
import math
import threading
import time

from action_msgs.srv import CancelGoal
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ClearEntireCostmap, ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.clock import Clock as RclpyClock
from rclpy.clock import ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from robot_bringup.lifecycle_policy import duplicate_names
from robot_bringup.lifecycle_policy import lifecycle_decision
from robot_bringup.lifecycle_policy import LifecycleAction
from robot_bringup.lifecycle_policy import normalization_transition
from robot_bringup.lifecycle_policy import RetryPolicy
from robot_bringup.readiness import ReadinessConfig, ReadinessTracker
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


DEFAULT_MANAGED_NODES = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'velocity_smoother',
    'collision_monitor',
    'bt_navigator',
]


class Nav2ActivationGate(Node):
    """Own readiness policy while Nav2's manager owns lifecycle changes."""

    def __init__(self):
        super().__init__('nav2_activation_gate')
        self.declare_parameter('startup_timeout', 30.0)
        self.declare_parameter('recovery_timeout', 30.0)
        self.declare_parameter('recovery_service_timeout', 3.0)
        self.declare_parameter('check_period', 0.10)
        self.declare_parameter('freshness_timeout', 0.50)
        self.declare_parameter('tf_stable_duration', 1.00)
        self.declare_parameter('tf_translation_tolerance', 0.05)
        self.declare_parameter('tf_yaw_tolerance', 0.0523598776)
        self.declare_parameter('clock_jump_tolerance', 5.0)
        self.declare_parameter('max_attempts', 3)
        self.declare_parameter('retry_initial_backoff', 0.50)
        self.declare_parameter('retry_maximum_backoff', 2.00)
        self.declare_parameter(
            'lifecycle_service',
            '/lifecycle_manager_navigation/manage_nodes',
        )
        self.declare_parameter('managed_nodes', DEFAULT_MANAGED_NODES)
        self.declare_parameter('immutable_map_node', 'map_server')
        self.declare_parameter('initial_pose_source', 'auto')
        self.declare_parameter(
            'initial_pose_reseed_service', '/initial_pose/reseed')

        self._startup_timeout = self._positive_parameter('startup_timeout')
        self._recovery_timeout = self._positive_parameter(
            'recovery_timeout')
        self._recovery_service_timeout = self._positive_parameter(
            'recovery_service_timeout')
        check_period = self._positive_parameter('check_period')
        readiness_config = ReadinessConfig(
            freshness_timeout=self._positive_parameter(
                'freshness_timeout'),
            tf_stable_duration=self._positive_parameter(
                'tf_stable_duration'),
            tf_translation_tolerance=self._positive_parameter(
                'tf_translation_tolerance'),
            tf_yaw_tolerance=self._positive_parameter(
                'tf_yaw_tolerance'),
            clock_jump_tolerance=self._positive_parameter(
                'clock_jump_tolerance'),
        )
        self._retry_policy = RetryPolicy(
            max_attempts=self._positive_integer_parameter('max_attempts'),
            initial_backoff=self._positive_parameter(
                'retry_initial_backoff'),
            maximum_backoff=self._positive_parameter(
                'retry_maximum_backoff'),
        )
        self._managed_nodes = [
            str(value)
            for value in self.get_parameter('managed_nodes').value
        ]
        configured_duplicates = duplicate_names(self._managed_nodes)
        if configured_duplicates:
            raise ValueError(
                'managed_nodes contains duplicates: '
                + ', '.join(configured_duplicates))
        if not self._managed_nodes:
            raise ValueError('managed_nodes must not be empty')
        self._immutable_map_node = str(
            self.get_parameter('immutable_map_node').value).strip('/')
        if not self._immutable_map_node:
            raise ValueError('immutable_map_node must not be empty')
        if self._immutable_map_node in self._managed_nodes:
            raise ValueError(
                'immutable_map_node must be independent from managed_nodes')
        self._initial_pose_source = str(
            self.get_parameter('initial_pose_source').value)
        if self._initial_pose_source not in {'auto', 'rviz'}:
            raise ValueError('initial_pose_source must be auto or rviz')

        self._tracker = ReadinessTracker(readiness_config)
        self._started_at = time.monotonic()
        self._last_status_at = self._started_at
        self._next_attempt_at = self._started_at
        self._attempts = 0
        self._last_failure = ''
        self._fatal_error = None
        self._activated = False
        self._activation_verifying = False
        self._request_in_flight = False
        self._state_query_in_flight = False
        self._state_query = None
        # One lock protects generation changes and every async-operation
        # reservation/completion.  In a MultiThreadedExecutor a completed
        # state snapshot must transition atomically into a manager request;
        # otherwise the wall timer can start a second snapshot and issue a
        # duplicate lifecycle command.
        self._state_query_lock = threading.RLock()
        self._generation = 0
        self._snapshot_in_flight = False
        self._snapshot_generation = None
        self._manager_request_token = None
        self._normalization_attempts = 0
        self._map_operation_in_flight = False
        self._map_operation_token = None
        self._map_activation_attempts = 0
        self._next_map_attempt_at = self._started_at

        self._recovering = False
        self._recovery_started_at = None
        self._recovery_stage = None
        self._recovery_stage_started_at = None
        self._recovery_service_in_flight = False
        self._recovery_service_operation = None
        self._recovery_stage_attempts = 0
        self._recovery_pause_verifying = False
        self._recovery_resume_verifying = False

        best_effort = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        transient_local = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._clock_subscription = self.create_subscription(
            Clock, '/clock', self._clock_callback, best_effort)
        self._scan_subscription = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, best_effort)
        self._odom_subscription = self.create_subscription(
            Odometry, '/odom', self._odom_callback, reliable)
        self._map_subscription = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, transient_local)
        self._reset_subscription = self.create_subscription(
            Empty,
            '/simulation/reset_event',
            self._reset_callback,
            reliable,
        )

        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self)
        service_name = str(self.get_parameter('lifecycle_service').value)
        self._lifecycle_client = self.create_client(
            ManageLifecycleNodes, service_name)
        self._state_clients = {
            name: self.create_client(GetState, f'/{name}/get_state')
            for name in self._managed_nodes
        }
        self._change_state_clients = {
            name: self.create_client(ChangeState, f'/{name}/change_state')
            for name in self._managed_nodes
        }
        self._map_state_client = self.create_client(
            GetState, f'/{self._immutable_map_node}/get_state')
        self._map_change_state_client = self.create_client(
            ChangeState, f'/{self._immutable_map_node}/change_state')
        self._cancel_goal_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self._global_clear_client = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap',
        )
        self._local_clear_client = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap',
        )
        reseed_service = str(
            self.get_parameter('initial_pose_reseed_service').value)
        self._reseed_client = self.create_client(Trigger, reseed_service)

        # Readiness, retry, and recovery deadlines are wall-monotonic. A ROS
        # time timer can stall or execute unpredictably during /clock reset.
        self._steady_clock = RclpyClock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(
            check_period,
            self._check_readiness,
            clock=self._steady_clock,
        )

    def _positive_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    def _positive_integer_parameter(self, name):
        value = self.get_parameter(name).value
        if isinstance(value, bool) or not isinstance(value, int) or value < 1:
            raise ValueError(f'{name} must be a positive integer')
        return value

    def _clock_callback(self, message):
        stamp_s = message.clock.sec + message.clock.nanosec * 1.0e-9
        event = self._tracker.mark_clock(stamp_s, time.monotonic())
        if event is None:
            return
        self._handle_epoch_event(event)

    def _reset_callback(self, message):
        del message
        self._handle_epoch_event(self._tracker.mark_reset())

    def _handle_epoch_event(self, event):
        with self._state_query_lock:
            self.get_logger().warning(
                f'Simulation time {event.kind} detected: '
                f'{event.previous_stamp_s:.9f} -> {event.stamp_s:.9f}; '
                f'epoch={event.epoch}')
            # tf2 rejects lower-stamped transforms as TF_OLD_DATA until its
            # cache is cleared. The tracker alone cannot start a new epoch.
            self._tf_buffer.clear()
            if (self._activated or self._request_in_flight
                    or self._recovering):
                self._begin_recovery(event)
            else:
                self._invalidate_async_work()
                self._attempts = 0
                self._next_attempt_at = time.monotonic()

    def _scan_callback(self, message):
        stamp = message.header.stamp
        stamp_s = stamp.sec + stamp.nanosec * 1.0e-9
        self._tracker.mark_scan(stamp_s, time.monotonic())

    def _odom_callback(self, message):
        stamp = message.header.stamp
        stamp_s = stamp.sec + stamp.nanosec * 1.0e-9
        self._tracker.mark_odom(stamp_s, time.monotonic())

    def _map_callback(self, message):
        del message
        self._tracker.mark_map()

    def _check_readiness(self):
        now = time.monotonic()
        if self._fatal_error is not None:
            raise RuntimeError(self._fatal_error)

        self._observe_map_to_odom(now)
        if self._recovering:
            if now - self._recovery_started_at >= self._recovery_timeout:
                self._set_fatal(
                    'Nav2 recovery timed out; stage='
                    f'{self._recovery_stage}; last_failure='
                    f'{self._last_failure or "none"}')
                return
            self._advance_recovery(now)
            return
        if self._activated:
            return
        if now - self._started_at >= self._startup_timeout:
            missing = ', '.join(self._tracker.missing_requirements(now))
            self._set_fatal(
                f'Nav2 activation gate timed out after '
                f'{now - self._started_at:.1f}s; missing={missing or "none"}; '
                f'last_failure={self._last_failure or "none"}; '
                f'managed_nodes={self._managed_nodes}')
            return
        if (self._request_in_flight or self._state_query_in_flight
                or self._snapshot_in_flight):
            return
        if now < self._next_attempt_at:
            return

        missing = self._tracker.missing_requirements(now)
        if 'latched /map' in missing:
            missing.extend(self._ensure_immutable_map_active(now))
        service_missing = self._missing_lifecycle_services()
        duplicates = self._runtime_duplicate_managed_nodes()
        if duplicates:
            self._set_fatal(
                'duplicate managed node FQNs detected: '
                + ', '.join(duplicates))
            return
        if missing or service_missing:
            self._log_waiting(now, missing + service_missing)
            return
        self._start_state_query('activation')

    def _ensure_immutable_map_active(self, now):
        """Repair a missed launch transition before waiting indefinitely on /map."""
        with self._state_query_lock:
            if self._map_operation_in_flight:
                return ['immutable map lifecycle operation']
            if now < self._next_map_attempt_at:
                return ['immutable map lifecycle backoff']
            missing = []
            if not self._map_state_client.service_is_ready():
                missing.append(
                    f'/{self._immutable_map_node}/get_state')
            if not self._map_change_state_client.service_is_ready():
                missing.append(
                    f'/{self._immutable_map_node}/change_state')
            if missing:
                return missing
            token = object()
            generation = self._generation
            self._map_operation_in_flight = True
            self._map_operation_token = token
            try:
                future = self._map_state_client.call_async(GetState.Request())
            except Exception as exc:
                self._map_operation_in_flight = False
                self._map_operation_token = None
                self._record_map_activation_failure(
                    f'/{self._immutable_map_node}/get_state raised '
                    f'{type(exc).__name__}: {exc}',
                    now,
                )
                return ['immutable map lifecycle query failed']
            future.add_done_callback(partial(
                self._map_state_done,
                generation=generation,
                token=token,
            ))
            return ['immutable map lifecycle query']

    def _map_state_done(self, future, *, generation, token):
        try:
            response = future.result()
            if response is None:
                raise RuntimeError('empty response')
            state = str(response.current_state.label).lower()
            error = None
        except Exception as exc:
            state = ''
            error = exc
        with self._state_query_lock:
            if (generation != self._generation
                    or token is not self._map_operation_token):
                return
            self._map_operation_in_flight = False
            self._map_operation_token = None
            now = time.monotonic()
            if error is not None:
                self._record_map_activation_failure(
                    f'/{self._immutable_map_node}/get_state failed: '
                    f'{type(error).__name__}: {error}',
                    now,
                )
                return
            if state == 'active':
                self._map_activation_attempts = 0
                self._next_map_attempt_at = now + 0.1
                return
            if state in {
                    'configuring', 'activating', 'deactivating',
                    'cleaningup', 'shuttingdown'}:
                self._next_map_attempt_at = now + 0.1
                return
            transition = {
                'unconfigured': (
                    'configure', Transition.TRANSITION_CONFIGURE),
                'inactive': ('activate', Transition.TRANSITION_ACTIVATE),
            }.get(state)
            if transition is None:
                self._record_map_activation_failure(
                    f'/{self._immutable_map_node} has unsupported lifecycle '
                    f'state {state!r}',
                    now,
                )
                return
            self._send_map_transition(
                transition[0], transition[1], now)

    def _send_map_transition(self, label, transition_id, now):
        if self._map_operation_in_flight:
            return False
        if not self._map_change_state_client.service_is_ready():
            self._record_map_activation_failure(
                f'/{self._immutable_map_node}/change_state unavailable',
                now,
            )
            return False
        request = ChangeState.Request()
        request.transition.id = transition_id
        token = object()
        generation = self._generation
        self._map_operation_in_flight = True
        self._map_operation_token = token
        self.get_logger().warning(
            f'Repairing immutable map lifecycle: '
            f'node={self._immutable_map_node}, transition={label}')
        try:
            future = self._map_change_state_client.call_async(request)
        except Exception as exc:
            self._map_operation_in_flight = False
            self._map_operation_token = None
            self._record_map_activation_failure(
                f'/{self._immutable_map_node}/change_state {label} raised '
                f'{type(exc).__name__}: {exc}',
                now,
            )
            return False
        future.add_done_callback(partial(
            self._map_transition_done,
            label=label,
            generation=generation,
            token=token,
        ))
        return True

    def _map_transition_done(
            self, future, *, label, generation, token):
        try:
            response = future.result()
            error = None
        except Exception as exc:
            response = None
            error = exc
        with self._state_query_lock:
            if (generation != self._generation
                    or token is not self._map_operation_token):
                return
            self._map_operation_in_flight = False
            self._map_operation_token = None
            now = time.monotonic()
            if error is not None:
                self._record_map_activation_failure(
                    f'/{self._immutable_map_node}/change_state {label} '
                    f'raised {type(error).__name__}: {error}',
                    now,
                )
                return
            if response is None or not response.success:
                self._record_map_activation_failure(
                    f'/{self._immutable_map_node}/change_state {label} '
                    f'returned success={getattr(response, "success", None)}',
                    now,
                )
                return
            self._map_activation_attempts = 0
            self._next_map_attempt_at = now + 0.1
            self.get_logger().info(
                f'Immutable map lifecycle transition completed: '
                f'node={self._immutable_map_node}, transition={label}')

    def _record_map_activation_failure(self, reason, now):
        self._last_failure = reason
        self._map_activation_attempts += 1
        if not self._retry_policy.can_retry(self._map_activation_attempts):
            self._set_fatal(
                'Immutable map lifecycle activation failed after '
                f'{self._map_activation_attempts} attempts: {reason}')
            return
        delay = self._retry_policy.delay_after_failure(
            self._map_activation_attempts)
        self._next_map_attempt_at = now + delay
        self.get_logger().warning(
            f'Immutable map lifecycle attempt '
            f'{self._map_activation_attempts}/'
            f'{self._retry_policy.max_attempts} failed: {reason}; '
            f'retrying in {delay:.2f}s')

    def _observe_map_to_odom(self, now):
        try:
            transform = self._tf_buffer.lookup_transform(
                'map', 'odom', Time())
        except TransformException:
            return
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        translation = transform.transform.translation
        stamp = transform.header.stamp
        stamp_s = stamp.sec + stamp.nanosec * 1.0e-9
        self._tracker.observe_transform(
            translation.x, translation.y, yaw, stamp_s, now)

    def _missing_lifecycle_services(self):
        missing = []
        if not self._lifecycle_client.service_is_ready():
            missing.append('Nav2 lifecycle manager service')
        for name, client in self._state_clients.items():
            if not client.service_is_ready():
                missing.append(f'/{name}/get_state')
        return missing

    def _runtime_duplicate_managed_nodes(self):
        managed = {f'/{name}' for name in self._managed_nodes}
        discovered = []
        for name, namespace in self.get_node_names_and_namespaces():
            prefix = namespace.rstrip('/')
            fqn = f'{prefix}/{name}' if prefix else f'/{name}'
            if fqn in managed:
                discovered.append(fqn)
        return duplicate_names(discovered)

    def _log_waiting(self, now, missing):
        if now - self._last_status_at < 2.0:
            return
        self.get_logger().info(
            'Waiting to activate Nav2: ' + ', '.join(missing))
        self._last_status_at = now

    def _start_state_query(self, context):
        with self._state_query_lock:
            if (self._state_query_in_flight or self._snapshot_in_flight
                    or self._request_in_flight):
                return False
            generation = self._generation
            query = {
                'context': context,
                'generation': generation,
                'pending': set(self._managed_nodes),
                'states': {},
            }
            self._state_query = query
            self._state_query_in_flight = True
        for name, client in self._state_clients.items():
            try:
                future = client.call_async(GetState.Request())
            except Exception as exc:
                with self._state_query_lock:
                    if (generation != self._generation
                            or self._state_query is not query):
                        return False
                    self._state_query_in_flight = False
                    self._state_query = None
                    self._record_failure(
                        f'failed to query /{name}/get_state: {exc}',
                        time.monotonic(),
                    )
                return False
            future.add_done_callback(partial(
                self._state_query_done,
                name=name,
                generation=generation,
            ))
        return True

    def _state_query_done(self, future, *, name, generation):
        try:
            response = future.result()
            if response is None:
                raise RuntimeError('empty response')
            state = str(response.current_state.label).lower()
        except Exception as exc:
            with self._state_query_lock:
                if (generation != self._generation
                        or self._state_query is None
                        or self._state_query['generation'] != generation):
                    return
                self._state_query_in_flight = False
                self._state_query = None
                self._record_failure(
                    f'/{name}/get_state failed: '
                    f'{type(exc).__name__}: {exc}',
                    time.monotonic(),
                )
            return
        with self._state_query_lock:
            if generation != self._generation or self._state_query is None:
                return
            query = self._state_query
            if (query['generation'] != generation
                    or name not in query['pending']):
                return
            query['states'][name] = state
            query['pending'].discard(name)
            if query['pending']:
                return
            context = query['context']
            states = dict(query['states'])
            self._state_query_in_flight = False
            self._state_query = None
            self._snapshot_in_flight = True
            self._snapshot_generation = generation
            # Keep the reservation lock held through snapshot handling.  The
            # handler either consumes the snapshot or atomically reserves the
            # lifecycle manager request before the timer may query again.
            self._handle_state_snapshot(
                context, states, time.monotonic(), generation=generation)

    def _snapshot_is_current(self, generation):
        return (
            self._snapshot_in_flight
            and generation == self._generation
            and generation == self._snapshot_generation
        )

    def _consume_snapshot(self, generation):
        if generation is not None and not self._snapshot_is_current(
                generation):
            return False
        if self._snapshot_in_flight:
            self._snapshot_in_flight = False
            self._snapshot_generation = None
        return True

    def _handle_state_snapshot(
            self, context, states, now, *, generation=None):
        if generation is not None and not self._snapshot_is_current(
                generation):
            return
        decision = lifecycle_decision(states)
        summary = ', '.join(
            f'{name}={state}' for name, state in sorted(states.items()))
        self.get_logger().info(
            f'Nav2 lifecycle snapshot ({context}): {summary}; '
            f'decision={decision.action.value}')

        if decision.action is LifecycleAction.WAIT:
            if not self._consume_snapshot(generation):
                return
            self._next_attempt_at = now + 0.1
            return
        if decision.action is LifecycleAction.FAIL:
            if not self._consume_snapshot(generation):
                return
            self._record_failure(decision.reason, now)
            return

        if context == 'activation':
            self._handle_activation_decision(
                decision, states, now, generation)
        elif context == 'recovery_pause':
            self._handle_recovery_pause_decision(
                decision, states, now, generation)
        elif context == 'recovery_resume':
            self._handle_recovery_resume_decision(
                decision, states, now, generation)
        else:
            if not self._consume_snapshot(generation):
                return
            self._set_fatal(f'unknown lifecycle query context: {context}')

    def _handle_activation_decision(
            self, decision, states, now, generation):
        if decision.action is LifecycleAction.ALREADY_ACTIVE:
            if not self._consume_snapshot(generation):
                return
            self._mark_active(recovered=False)
            return
        if decision.action is LifecycleAction.NORMALIZE:
            self._send_normalization_transition(
                'activation',
                states,
                'active',
                now,
                generation,
            )
            return
        if self._activation_verifying:
            if not self._consume_snapshot(generation):
                return
            self._activation_verifying = False
            self._record_failure(
                'lifecycle manager returned success but nodes are not active: '
                + decision.reason,
                now,
            )
            return
        command = (
            ManageLifecycleNodes.Request.STARTUP
            if decision.action is LifecycleAction.STARTUP
            else ManageLifecycleNodes.Request.RESUME
        )
        self._send_manager_command(
            command, 'activation', snapshot_generation=generation)

    def _send_manager_command(
            self, command, context, *, snapshot_generation=None):
        with self._state_query_lock:
            if snapshot_generation is not None:
                if not self._snapshot_is_current(snapshot_generation):
                    return False
            elif self._snapshot_in_flight:
                return False
            if self._request_in_flight:
                return False
            if snapshot_generation is not None:
                self._consume_snapshot(snapshot_generation)
            request = ManageLifecycleNodes.Request()
            request.command = command
            token = object()
            self._request_in_flight = True
            self._manager_request_token = token
            self._attempts += 1
            generation = self._generation
            self.get_logger().info(
                f'Nav2 lifecycle command={self._command_name(command)} '
                f'context={context} attempt={self._attempts}/'
                f'{self._retry_policy.max_attempts}; '
                f'managed_nodes={self._managed_nodes}')
            try:
                future = self._lifecycle_client.call_async(request)
            except Exception as exc:
                if (generation == self._generation
                        and token is self._manager_request_token):
                    self._request_in_flight = False
                    self._manager_request_token = None
                    self._record_failure(
                        f'failed to call lifecycle manager: {exc}',
                        time.monotonic(),
                        attempt_already_counted=True,
                    )
                return False
            future.add_done_callback(partial(
                self._manager_command_done,
                context=context,
                command=command,
                generation=generation,
                token=token,
            ))
            return True

    def _manager_command_done(
            self, future, *, context, command, generation, token):
        try:
            response = future.result()
            error = None
        except Exception as exc:
            response = None
            error = exc
        with self._state_query_lock:
            if (generation != self._generation
                    or token is not self._manager_request_token):
                return
            self._request_in_flight = False
            self._manager_request_token = None
            if error is not None:
                self._record_failure(
                    f'lifecycle manager {self._command_name(command)} raised '
                    f'{type(error).__name__}: {error}',
                    time.monotonic(),
                    attempt_already_counted=True,
                )
                return
            if response is None or not response.success:
                self._record_failure(
                    f'lifecycle manager {self._command_name(command)} '
                    f'returned success={getattr(response, "success", None)}; '
                    f'managed_nodes={self._managed_nodes}',
                    time.monotonic(),
                    attempt_already_counted=True,
                )
                return

            self.get_logger().info(
                f'lifecycle manager {self._command_name(command)} accepted')
            self._next_attempt_at = time.monotonic() + 0.1
            if context == 'activation':
                self._activation_verifying = True
            elif context == 'recovery_pause':
                self._recovery_pause_verifying = True
                self._set_recovery_stage('pause_verify')
            elif context == 'recovery_resume':
                self._recovery_resume_verifying = True
                self._set_recovery_stage('resume_verify')
            else:
                self._set_fatal(
                    f'unknown lifecycle command context: {context}')

    def _send_normalization_transition(
            self, context, states, target, now, generation):
        try:
            transition = normalization_transition(
                states, self._managed_nodes, target)
        except ValueError as exc:
            if self._consume_snapshot(generation):
                self._set_fatal(str(exc))
            return False
        if transition is None:
            if self._consume_snapshot(generation):
                self._next_attempt_at = now + 0.1
            return True
        name, transition_name = transition
        transition_ids = {
            'configure': Transition.TRANSITION_CONFIGURE,
            'activate': Transition.TRANSITION_ACTIVATE,
            'deactivate': Transition.TRANSITION_DEACTIVATE,
        }
        with self._state_query_lock:
            if not self._snapshot_is_current(generation):
                return False
            if self._request_in_flight:
                return False
            client = self._change_state_clients[name]
            if not client.service_is_ready():
                self._consume_snapshot(generation)
                self._record_normalization_failure(
                    f'/{name}/change_state service unavailable',
                    now,
                )
                return False
            self._consume_snapshot(generation)
            request = ChangeState.Request()
            request.transition.id = transition_ids[transition_name]
            token = object()
            self._request_in_flight = True
            self._manager_request_token = token
            operation_generation = self._generation
            self.get_logger().warning(
                f'Normalizing mixed Nav2 lifecycle state: '
                f'node={name}, transition={transition_name}, '
                f'target={target}, context={context}')
            try:
                future = client.call_async(request)
            except Exception as exc:
                self._request_in_flight = False
                self._manager_request_token = None
                self._record_normalization_failure(
                    f'/{name}/change_state raised {type(exc).__name__}: {exc}',
                    now,
                )
                return False
            future.add_done_callback(partial(
                self._normalization_transition_done,
                context=context,
                name=name,
                transition_name=transition_name,
                generation=operation_generation,
                token=token,
            ))
            return True

    def _normalization_transition_done(
            self, future, *, context, name, transition_name,
            generation, token):
        try:
            response = future.result()
            error = None
        except Exception as exc:
            response = None
            error = exc
        with self._state_query_lock:
            if (generation != self._generation
                    or token is not self._manager_request_token):
                return
            self._request_in_flight = False
            self._manager_request_token = None
            now = time.monotonic()
            if error is not None:
                self._record_normalization_failure(
                    f'/{name}/change_state {transition_name} raised '
                    f'{type(error).__name__}: {error}',
                    now,
                )
                return
            if response is None or not response.success:
                self._record_normalization_failure(
                    f'/{name}/change_state {transition_name} returned '
                    f'success={getattr(response, "success", None)}',
                    now,
                )
                return
            self._normalization_attempts = 0
            self._next_attempt_at = now + 0.1
            self.get_logger().info(
                f'Nav2 lifecycle normalization step completed: '
                f'node={name}, transition={transition_name}, '
                f'context={context}')

    def _record_normalization_failure(self, reason, now):
        self._last_failure = reason
        self._normalization_attempts += 1
        if not self._retry_policy.can_retry(self._normalization_attempts):
            self._set_fatal(
                'Nav2 lifecycle normalization failed after '
                f'{self._normalization_attempts} attempts: {reason}')
            return
        delay = self._retry_policy.delay_after_failure(
            self._normalization_attempts)
        self._next_attempt_at = now + delay
        self.get_logger().warning(
            f'Nav2 lifecycle normalization attempt '
            f'{self._normalization_attempts}/'
            f'{self._retry_policy.max_attempts} failed: {reason}; '
            f'retrying in {delay:.2f}s')

    def _record_failure(
            self, reason, now, *, attempt_already_counted=False):
        with self._state_query_lock:
            if not attempt_already_counted:
                self._attempts += 1
            self._last_failure = reason
            if not self._retry_policy.can_retry(self._attempts):
                self._set_fatal(
                    f'Nav2 lifecycle failed after {self._attempts} attempts: '
                    f'{reason}; managed_nodes={self._managed_nodes}; '
                    f'duplicates={self._runtime_duplicate_managed_nodes()}')
                return
            delay = self._retry_policy.delay_after_failure(self._attempts)
            self._next_attempt_at = now + delay
            self.get_logger().warning(
                f'Nav2 lifecycle attempt {self._attempts}/'
                f'{self._retry_policy.max_attempts} failed: {reason}; '
                f'retrying in {delay:.2f}s')

    def _mark_active(self, *, recovered):
        self._activated = True
        self._recovering = False
        self._activation_verifying = False
        self._recovery_pause_verifying = False
        self._recovery_resume_verifying = False
        self._recovery_stage = None
        self._attempts = 0
        self._last_failure = ''
        self._normalization_attempts = 0
        if recovered:
            self.get_logger().info(
                'Nav2 lifecycle recovery completed on simulation epoch '
                f'{self._tracker.epoch}')
        else:
            self.get_logger().info('Nav2 lifecycle activation completed')

    def _begin_recovery(self, event):
        with self._state_query_lock:
            self._invalidate_async_work()
            self._activated = False
            self._recovering = True
            self._recovery_started_at = time.monotonic()
            self._attempts = 0
            self._last_failure = ''
            self._normalization_attempts = 0
            self._next_attempt_at = self._recovery_started_at
            self._set_recovery_stage('cancel_goal')
            self.get_logger().warning(
                'Starting Nav2 time-jump recovery: cancel goal, pause '
                'lifecycle, clear costmaps, reseed localization, wait for '
                'fresh epoch data, then resume; '
                f'event={event.kind}, epoch={event.epoch}')

    def _invalidate_async_work(self):
        with self._state_query_lock:
            self._generation += 1
            self._state_query_in_flight = False
            self._state_query = None
            self._snapshot_in_flight = False
            self._snapshot_generation = None
            self._request_in_flight = False
            self._manager_request_token = None
            self._map_operation_in_flight = False
            self._map_operation_token = None
            self._recovery_service_in_flight = False
            self._recovery_service_operation = None
            self._activation_verifying = False
            self._recovery_pause_verifying = False
            self._recovery_resume_verifying = False

    def _set_recovery_stage(self, stage):
        self._recovery_stage = stage
        self._recovery_stage_started_at = time.monotonic()
        self._recovery_service_in_flight = False
        self._recovery_service_operation = None
        self._recovery_stage_attempts = 0

    def _advance_recovery(self, now):
        with self._state_query_lock:
            if now < self._next_attempt_at:
                return
            if (self._request_in_flight or self._state_query_in_flight
                    or self._snapshot_in_flight):
                return
            if self._recovery_service_in_flight:
                operation = self._recovery_service_operation
                if (operation is None
                        or now - operation['started_at']
                        < self._recovery_service_timeout):
                    return
                self._recovery_service_in_flight = False
                self._recovery_service_operation = None
                self._handle_recovery_service_failure(
                    operation['label'],
                    f'timed out after {self._recovery_service_timeout:.1f}s',
                    operation['next_stage'],
                    operation['required'],
                    now,
                )
                return
            stage = self._recovery_stage
            if stage == 'cancel_goal':
                self._call_recovery_service(
                    self._cancel_goal_client,
                    CancelGoal.Request(),
                    'cancel NavigateToPose goals',
                    'pause_query',
                    now,
                    required=True,
                )
            elif stage in {'pause_query', 'pause_verify'}:
                if self._lifecycle_client.service_is_ready() and all(
                        client.service_is_ready()
                        for client in self._state_clients.values()):
                    self._start_state_query('recovery_pause')
                else:
                    self._recovery_service_wait_or_fail(
                        now, 'Nav2 lifecycle services before recovery pause')
            elif stage == 'clear_global':
                self._call_recovery_service(
                    self._global_clear_client,
                    ClearEntireCostmap.Request(),
                    'clear global costmap',
                    'clear_local',
                    now,
                    required=True,
                )
            elif stage == 'clear_local':
                self._call_recovery_service(
                    self._local_clear_client,
                    ClearEntireCostmap.Request(),
                    'clear local costmap',
                    'reseed',
                    now,
                    required=True,
                )
            elif stage == 'reseed':
                if self._initial_pose_source == 'auto':
                    self._call_recovery_service(
                        self._reseed_client,
                        Trigger.Request(),
                        'request calibrated initial-pose reseed',
                        'waiting_readiness',
                        now,
                        required=True,
                    )
                else:
                    self.get_logger().warning(
                        'initial_pose_source=rviz: waiting for a new RViz '
                        '2D Pose Estimate in the current simulation epoch')
                    self._set_recovery_stage('waiting_readiness')
            elif stage == 'waiting_readiness':
                missing = self._tracker.missing_requirements(now)
                if missing:
                    self._log_waiting(now, [
                        'recovery ' + item for item in missing])
                else:
                    self._set_recovery_stage('resume_query')
            elif stage in {'resume_query', 'resume_verify'}:
                if self._lifecycle_client.service_is_ready() and all(
                        client.service_is_ready()
                        for client in self._state_clients.values()):
                    self._start_state_query('recovery_resume')
                else:
                    self._recovery_service_wait_or_fail(
                        now, 'Nav2 lifecycle services before recovery resume')
            else:
                self._set_fatal(f'unknown recovery stage: {stage}')

    def _handle_recovery_pause_decision(
            self, decision, states, now, generation):
        if decision.action is LifecycleAction.NORMALIZE:
            self._send_normalization_transition(
                'recovery_pause',
                states,
                'inactive',
                now,
                generation,
            )
            return
        if decision.action in {
                LifecycleAction.RESUME, LifecycleAction.STARTUP}:
            if not self._consume_snapshot(generation):
                return
            self._recovery_pause_verifying = False
            self._attempts = 0
            self._set_recovery_stage('clear_global')
            return
        if decision.action is LifecycleAction.ALREADY_ACTIVE:
            if self._recovery_pause_verifying:
                if not self._consume_snapshot(generation):
                    return
                self._recovery_pause_verifying = False
                self._record_failure(
                    'lifecycle manager PAUSE returned success but managed '
                    'nodes remain active',
                    now,
                )
            else:
                self._send_manager_command(
                    ManageLifecycleNodes.Request.PAUSE,
                    'recovery_pause',
                    snapshot_generation=generation,
                )

    def _handle_recovery_resume_decision(
            self, decision, states, now, generation):
        if decision.action is LifecycleAction.NORMALIZE:
            self._send_normalization_transition(
                'recovery_resume',
                states,
                'active',
                now,
                generation,
            )
            return
        if decision.action is LifecycleAction.ALREADY_ACTIVE:
            if not self._consume_snapshot(generation):
                return
            self._mark_active(recovered=True)
            return
        if self._recovery_resume_verifying:
            if not self._consume_snapshot(generation):
                return
            self._recovery_resume_verifying = False
            self._record_failure(
                'lifecycle manager resume/startup returned success but '
                'managed nodes are not active: ' + decision.reason,
                now,
            )
            return
        command = (
            ManageLifecycleNodes.Request.STARTUP
            if decision.action is LifecycleAction.STARTUP
            else ManageLifecycleNodes.Request.RESUME
        )
        self._send_manager_command(
            command, 'recovery_resume', snapshot_generation=generation)

    def _call_recovery_service(
            self, client, request, label, next_stage, now, *, required):
        with self._state_query_lock:
            if not client.service_is_ready():
                if required:
                    self._recovery_service_wait_or_fail(now, label)
                else:
                    self._recovery_service_wait_or_skip(
                        now, label, next_stage)
                return False
            token = object()
            generation = self._generation
            operation = {
                'token': token,
                'generation': generation,
                'label': label,
                'next_stage': next_stage,
                'required': required,
                'started_at': now,
            }
            self._recovery_service_in_flight = True
            self._recovery_service_operation = operation
            self.get_logger().info(
                f'Nav2 recovery: {label}; '
                f'attempt={self._recovery_stage_attempts + 1}/'
                f'{self._retry_policy.max_attempts}')
            try:
                future = client.call_async(request)
            except Exception as exc:
                if (generation == self._generation
                        and operation is self._recovery_service_operation):
                    self._recovery_service_in_flight = False
                    self._recovery_service_operation = None
                    self._handle_recovery_service_failure(
                        label,
                        f'failed to queue: {type(exc).__name__}: {exc}',
                        next_stage,
                        required,
                        now,
                    )
                return False
            future.add_done_callback(partial(
                self._recovery_service_done,
                label=label,
                next_stage=next_stage,
                required=required,
                generation=generation,
                token=token,
            ))
            return True

    def _recovery_service_done(
            self, future, *, label, next_stage, required, generation, token):
        try:
            response = future.result()
            if response is None:
                raise RuntimeError('empty response')
            if hasattr(response, 'success') and not response.success:
                error = (
                    'reported failure: '
                    f'{getattr(response, "message", "") or "no detail"}'
                )
            elif (hasattr(response, 'return_code')
                    and response.return_code
                    != CancelGoal.Response.ERROR_NONE):
                error = (
                    'reported cancel return_code='
                    f'{response.return_code}'
                )
            else:
                error = None
        except Exception as exc:
            error = f'{type(exc).__name__}: {exc}'
        with self._state_query_lock:
            operation = self._recovery_service_operation
            if (generation != self._generation
                    or operation is None
                    or token is not operation['token']):
                return
            self._recovery_service_in_flight = False
            self._recovery_service_operation = None
            if error is not None:
                self._handle_recovery_service_failure(
                    label, error, next_stage, required, time.monotonic())
                return
            self._set_recovery_stage(next_stage)

    def _handle_recovery_service_failure(
            self, label, detail, next_stage, required, now):
        reason = f'Nav2 recovery service failed ({label}): {detail}'
        self._last_failure = reason
        if not required:
            self.get_logger().warning(reason + '; continuing')
            self._set_recovery_stage(next_stage)
            return

        self._recovery_stage_attempts += 1
        if not self._retry_policy.can_retry(self._recovery_stage_attempts):
            self._set_fatal(
                f'Required recovery step failed after '
                f'{self._recovery_stage_attempts} attempts: {label}; '
                f'last_failure={detail}')
            return
        delay = self._retry_policy.delay_after_failure(
            self._recovery_stage_attempts)
        self._next_attempt_at = now + delay
        self._recovery_stage_started_at = now
        self.get_logger().warning(
            f'{reason}; attempt={self._recovery_stage_attempts}/'
            f'{self._retry_policy.max_attempts}; retrying in {delay:.2f}s')

    def _recovery_service_wait_or_skip(self, now, label, next_stage):
        if now - self._recovery_stage_started_at \
                < self._recovery_service_timeout:
            self._log_waiting(now, [label + ' service'])
            return False
        self.get_logger().warning(
            f'Nav2 recovery service unavailable after '
            f'{self._recovery_service_timeout:.1f}s ({label}); continuing '
            'under the fresh-data readiness gate')
        self._set_recovery_stage(next_stage)
        return True

    def _recovery_service_wait_or_fail(self, now, label):
        if now - self._recovery_stage_started_at \
                < self._recovery_service_timeout:
            self._log_waiting(now, [label])
            return
        self._set_fatal(
            f'{label} unavailable for '
            f'{self._recovery_service_timeout:.1f}s')

    def _set_fatal(self, reason):
        if self._fatal_error is None:
            self._fatal_error = reason
            self.get_logger().error(reason)

    @staticmethod
    def _command_name(command):
        names = {
            ManageLifecycleNodes.Request.STARTUP: 'STARTUP',
            ManageLifecycleNodes.Request.PAUSE: 'PAUSE',
            ManageLifecycleNodes.Request.RESUME: 'RESUME',
        }
        return names.get(command, str(command))


def main(args=None):
    """Run the persistent Nav2 activation and time-recovery gate."""
    rclpy.init(args=args)
    node = None
    try:
        node = Nav2ActivationGate()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # The default rclpy SIGINT handler may invalidate the context before
        # spin() returns.  Always destroy timers, clients and pending futures.
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
