import time

from action_msgs.srv import CancelGoal
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ClearEntireCostmap
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid, Odometry
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from robot_bringup.activation_gate import DEFAULT_MANAGED_NODES
from robot_bringup.activation_gate import Nav2ActivationGate
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


def _stamp(value):
    seconds = int(value)
    return Time(
        sec=seconds,
        nanosec=int(round((value - seconds) * 1.0e9)),
    )


class _LifecycleFixture(Node):
    def __init__(self, *, partial_resume_once=False):
        super().__init__('activation_gate_fixture')
        self.states = {
            name: 'unconfigured' for name in DEFAULT_MANAGED_NODES
        }
        # Reproduce the observed launch race: the immutable map server reached
        # inactive, but the launch transition handler missed ACTIVATE.
        self.map_state = 'inactive'
        self.events = []
        self.sim_stamp = 10.0
        self._rollback_next = False
        self._partial_resume_once = partial_resume_once

        self._clock_publisher = self.create_publisher(Clock, '/clock', 10)
        self._scan = self.create_publisher(LaserScan, '/scan', 10)
        self._odom = self.create_publisher(Odometry, '/odom', 10)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map = self.create_publisher(
            OccupancyGrid, '/map', map_qos)
        self._reset_event = self.create_publisher(
            Empty, '/simulation/reset_event', 10)
        self._tf = TransformBroadcaster(self)

        self._state_services = [
            self.create_service(
                GetState,
                f'/{name}/get_state',
                lambda request, response, node_name=name:
                self._get_state(request, response, node_name),
            )
            for name in DEFAULT_MANAGED_NODES
        ]
        self._change_state_services = [
            self.create_service(
                ChangeState,
                f'/{name}/change_state',
                lambda request, response, node_name=name:
                self._change_state(request, response, node_name),
            )
            for name in DEFAULT_MANAGED_NODES
        ]
        self._map_state_service = self.create_service(
            GetState,
            '/map_server/get_state',
            self._get_map_state,
        )
        self._map_change_state_service = self.create_service(
            ChangeState,
            '/map_server/change_state',
            self._change_map_state,
        )
        self._manager = self.create_service(
            ManageLifecycleNodes,
            '/lifecycle_manager_navigation/manage_nodes',
            self._manage,
        )
        self._cancel = self.create_service(
            CancelGoal,
            '/navigate_to_pose/_action/cancel_goal',
            self._cancel_goals,
        )
        self._global_clear = self.create_service(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap',
            self._clear_global,
        )
        self._local_clear = self.create_service(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap',
            self._clear_local,
        )
        self._reseed = self.create_service(
            Trigger,
            '/initial_pose/reseed',
            self._trigger_reseed,
        )
        self._timer = self.create_timer(0.02, self._publish_inputs)

    def request_rollback(self):
        self._rollback_next = True

    def request_reset(self):
        self._reset_event.publish(Empty())

    def _get_state(self, request, response, node_name):
        del request
        response.current_state = State(
            id=State.PRIMARY_STATE_UNKNOWN,
            label=self.states[node_name],
        )
        return response

    def _manage(self, request, response):
        names = {
            ManageLifecycleNodes.Request.STARTUP: 'STARTUP',
            ManageLifecycleNodes.Request.PAUSE: 'PAUSE',
            ManageLifecycleNodes.Request.RESUME: 'RESUME',
        }
        self.events.append('manager:' + names[request.command])
        if (
            request.command == ManageLifecycleNodes.Request.RESUME
            and self._partial_resume_once
        ):
            self._partial_resume_once = False
            self.states.update({
                'controller_server': 'active',
                'planner_server': 'active',
            })
            response.success = True
            return response
        if request.command in {
                ManageLifecycleNodes.Request.STARTUP,
                ManageLifecycleNodes.Request.RESUME}:
            state = 'active'
        elif request.command == ManageLifecycleNodes.Request.PAUSE:
            state = 'inactive'
        else:
            response.success = False
            return response
        self.states = {name: state for name in self.states}
        response.success = True
        return response

    def _get_map_state(self, request, response):
        del request
        response.current_state = State(
            id=State.PRIMARY_STATE_UNKNOWN,
            label=self.map_state,
        )
        return response

    def _change_map_state(self, request, response):
        transitions = {
            Transition.TRANSITION_CONFIGURE: ('configure', 'inactive'),
            Transition.TRANSITION_ACTIVATE: ('activate', 'active'),
        }
        transition = transitions.get(request.transition.id)
        if transition is None:
            response.success = False
            return response
        label, target = transition
        self.events.append(f'direct:map_server:{label}')
        self.map_state = target
        response.success = True
        return response

    def _change_state(self, request, response, node_name):
        transitions = {
            Transition.TRANSITION_CONFIGURE: ('configure', 'inactive'),
            Transition.TRANSITION_ACTIVATE: ('activate', 'active'),
            Transition.TRANSITION_DEACTIVATE: ('deactivate', 'inactive'),
        }
        transition = transitions.get(request.transition.id)
        if transition is None:
            response.success = False
            return response
        label, target = transition
        self.events.append(f'direct:{node_name}:{label}')
        self.states[node_name] = target
        response.success = True
        return response

    def _cancel_goals(self, request, response):
        del request
        self.events.append('cancel')
        response.return_code = CancelGoal.Response.ERROR_NONE
        return response

    def _clear_global(self, request, response):
        del request
        self.events.append('clear_global')
        return response

    def _clear_local(self, request, response):
        del request
        self.events.append('clear_local')
        return response

    def _trigger_reseed(self, request, response):
        del request
        self.events.append('reseed')
        response.success = True
        response.message = 'reseeded'
        return response

    def _publish_inputs(self):
        if self._rollback_next:
            self.sim_stamp = 0.1
            self._rollback_next = False
        else:
            self.sim_stamp += 0.02
        stamp = _stamp(self.sim_stamp)

        clock = Clock()
        clock.clock = stamp
        self._clock_publisher.publish(clock)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'base_link'
        self._scan.publish(scan)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        self._odom.publish(odom)

        occupancy = OccupancyGrid()
        occupancy.header.stamp = stamp
        occupancy.header.frame_id = 'map'
        occupancy.info.width = 1
        occupancy.info.height = 1
        occupancy.info.resolution = 1.0
        occupancy.data = [0]
        if self.map_state == 'active':
            self._map.publish(occupancy)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.rotation.w = 1.0
        self._tf.sendTransform(transform)


def _spin_until(executor, predicate, timeout):
    expires = time.monotonic() + timeout
    while time.monotonic() < expires:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return True
    return False


@pytest.mark.ros
@pytest.mark.parametrize('event_kind', ['clock_rollback', 'reset_event'])
def test_gate_activates_once_then_recovers_in_order_after_epoch_change(
        tmp_path, event_kind):
    parameters = tmp_path / 'gate.yaml'
    parameters.write_text(
        """nav2_activation_gate:
  ros__parameters:
    use_sim_time: false
    startup_timeout: 5.0
    recovery_timeout: 5.0
    recovery_service_timeout: 2.0
    check_period: 0.02
    freshness_timeout: 0.30
    tf_stable_duration: 0.10
    tf_translation_tolerance: 0.05
    tf_yaw_tolerance: 0.05
    clock_jump_tolerance: 1.0
    max_attempts: 3
    retry_initial_backoff: 0.05
    retry_maximum_backoff: 0.10
    initial_pose_source: auto
""",
        encoding='utf-8',
    )
    rclpy.init(args=['--ros-args', '--params-file', str(parameters)])
    fixture = _LifecycleFixture()
    gate = Nav2ActivationGate()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(fixture)
    executor.add_node(gate)
    try:
        assert _spin_until(executor, lambda: gate._activated, 4.0)
        assert fixture.events == [
            'direct:map_server:activate',
            'manager:STARTUP',
        ]

        if event_kind == 'clock_rollback':
            fixture.request_rollback()
        else:
            fixture.request_reset()
        assert _spin_until(
            executor,
            lambda: gate._activated and not gate._recovering
            and 'manager:RESUME' in fixture.events,
            4.0,
        )
        assert fixture.events == [
            'direct:map_server:activate',
            'manager:STARTUP',
            'cancel',
            'manager:PAUSE',
            'clear_global',
            'clear_local',
            'reseed',
            'manager:RESUME',
        ]
        assert gate._tracker.epoch == 1
    finally:
        fixture._timer.cancel()
        gate._timer.cancel()
        gate._tf_listener.unregister()
        for _ in range(10):
            executor.spin_once(timeout_sec=0.01)
        executor.remove_node(gate)
        executor.remove_node(fixture)
        executor.shutdown(timeout_sec=1.0)
        gate.destroy_node()
        fixture.destroy_node()
        rclpy.shutdown()


@pytest.mark.ros
def test_gate_repairs_partial_resume_without_terminating(tmp_path):
    parameters = tmp_path / 'gate.yaml'
    parameters.write_text(
        """nav2_activation_gate:
  ros__parameters:
    use_sim_time: false
    startup_timeout: 5.0
    recovery_timeout: 5.0
    recovery_service_timeout: 2.0
    check_period: 0.02
    freshness_timeout: 0.30
    tf_stable_duration: 0.10
    tf_translation_tolerance: 0.05
    tf_yaw_tolerance: 0.05
    clock_jump_tolerance: 1.0
    max_attempts: 3
    retry_initial_backoff: 0.05
    retry_maximum_backoff: 0.10
    initial_pose_source: auto
""",
        encoding='utf-8',
    )
    rclpy.init(args=['--ros-args', '--params-file', str(parameters)])
    fixture = _LifecycleFixture(partial_resume_once=True)
    gate = Nav2ActivationGate()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(fixture)
    executor.add_node(gate)
    try:
        assert _spin_until(executor, lambda: gate._activated, 4.0)
        fixture.request_reset()
        assert _spin_until(
            executor,
            lambda: gate._tracker.epoch == 1 and gate._recovering,
            1.0,
        )
        assert _spin_until(
            executor,
            lambda: gate._activated and not gate._recovering
            and fixture.states
            == {name: 'active' for name in DEFAULT_MANAGED_NODES},
            4.0,
        )
        direct_events = [
            event for event in fixture.events if event.startswith('direct:')
        ]
        assert direct_events == [
            'direct:map_server:activate',
            'direct:behavior_server:activate',
            'direct:velocity_smoother:activate',
            'direct:collision_monitor:activate',
            'direct:bt_navigator:activate',
        ]
        assert gate._fatal_error is None
    finally:
        fixture._timer.cancel()
        gate._timer.cancel()
        gate._tf_listener.unregister()
        for _ in range(10):
            executor.spin_once(timeout_sec=0.01)
        executor.remove_node(gate)
        executor.remove_node(fixture)
        executor.shutdown(timeout_sec=1.0)
        gate.destroy_node()
        fixture.destroy_node()
        rclpy.shutdown()
