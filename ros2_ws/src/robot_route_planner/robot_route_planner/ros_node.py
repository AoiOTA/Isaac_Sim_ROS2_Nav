"""A21 Module3 coordinator for graph, official route search, and guidance."""

from __future__ import annotations

from pathlib import Path
import json
import math
import tempfile

import numpy as np

from .defaults import load_engineering_defaults
from .feasibility import apply_footprint_feasibility
from .gvg import build_gvg
from .map_io import OccupancyMap, load_occupancy_map
from .route_cost import edge_cost
from .route_support import export_route_support_graph, save_route_support
from .runtime_edges import RuntimeEdgeManager, RuntimeState
from .stable_ids import stabilize_graph_ids
from .structural_updates import StructuralChangeMonitor
from .tracking import RouteTracker


class RouteCoordinator:
    def __init__(self, node) -> None:
        from bio_nav_interfaces.msg import (
            CanonicalRoute,
            EdgePriorArray,
            NavigationGraph,
            RouteContext,
            RouteProgress,
            RuntimeEdgeObservation,
            RuntimeEdgeStateArray,
            StructuralGraphStatus,
        )
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import ComputeRoute, NavigateToPose
        from nav2_msgs.srv import DynamicEdges, SetRouteGraph
        from nav_msgs.msg import OccupancyGrid, Odometry
        from rclpy.action import ActionClient
        from rclpy.duration import Duration
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rclpy.time import Time
        from std_msgs.msg import Bool
        from tf2_ros import Buffer, TransformListener

        self.node = node
        for name, default in (
            ("engineering_defaults_file", ""),
            ("map_yaml", ""),
            ("frame_id", "map"),
            ("base_frame_id", "base_link"),
            ("module2_enabled", True),
            ("execute_navigation", True),
            ("route_guided_bt_xml", ""),
            ("route_goal_topic", "/bio_nav/route_goal"),
            ("edge_prior_topic", "/bio_nav/module2/edge_priors"),
            ("runtime_observation_topic", "/bio_nav/runtime_edge_observation"),
            ("structural_map_topic", "/bio_nav/structural_map"),
            ("goal_complete_topic", "/bio_nav/route_goal_complete"),
            ("odometry_topic", "/ground_truth/odom"),
            ("compute_route_action", "/compute_route"),
            ("navigate_to_pose_action", "/navigate_to_pose"),
            ("dynamic_edges_service", "/route_server/DynamicEdgesScorer/adjust_edges"),
            ("set_route_graph_service", "/route_server/set_route_graph"),
        ):
            node.declare_parameter(name, default)
        defaults_path = Path(str(node.get_parameter("engineering_defaults_file").value))
        map_path = Path(str(node.get_parameter("map_yaml").value))
        if not defaults_path.is_file() or not map_path.is_file():
            raise RuntimeError("engineering_defaults_file and map_yaml are required")
        self.defaults = load_engineering_defaults(defaults_path)
        self.map = load_occupancy_map(
            map_path,
            unknown_is_occupied=bool(self.defaults["graph"]["unknown_is_occupied"]),
        )
        self.graph = apply_footprint_feasibility(
            build_gvg(
                self.map,
                self.defaults["graph"],
                self.defaults["footprint"],
                self.defaults["route_cost"],
            ),
            self.map,
            self.defaults["footprint"],
        )
        self.support = export_route_support_graph(
            self.graph,
            support_spacing_m=float(self.defaults["graph"]["route_support_spacing_m"]),
        )
        self.runtime = RuntimeEdgeManager(
            self.defaults["runtime_edges"], self.defaults["route_cost"]
        )
        self.structural_monitor = StructuralChangeMonitor(
            self.map.free,
            self.map.resolution_m,
            self.defaults["structural_updates"],
        )
        self.pending_structural_map: OccupancyMap | None = None
        self.pending_goal = None
        self.pending_deadline_ns: int | None = None
        self.request_id = 0
        self.latest_priors: dict[int, tuple[float, float]] = {}
        self.tracker: RouteTracker | None = None
        self.latest_pose_xy: tuple[float, float] | None = None
        self.route_active = False
        self.navigation_goal_pending = False
        self.navigation_goal_handle = None
        self.navigation_failed = False
        self.frame_id = str(node.get_parameter("frame_id").value)
        self.base_frame_id = str(node.get_parameter("base_frame_id").value)
        self.module2_enabled = bool(node.get_parameter("module2_enabled").value)
        self.execute_navigation = bool(
            node.get_parameter("execute_navigation").value
        )
        self.route_guided_bt_xml = str(
            node.get_parameter("route_guided_bt_xml").value
        )
        self.Duration = Duration
        self.Time = Time
        self.ComputeRoute = ComputeRoute
        self.NavigateToPose = NavigateToPose
        self.DynamicEdges = DynamicEdges
        self.SetRouteGraph = SetRouteGraph
        self.CanonicalRoute = CanonicalRoute
        self.RouteContext = RouteContext
        self.RouteProgress = RouteProgress
        self.RuntimeEdgeStateArray = RuntimeEdgeStateArray
        self.StructuralGraphStatus = StructuralGraphStatus
        qos_latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.graph_pub = node.create_publisher(
            NavigationGraph, "/bio_nav/navigation_graph", qos_latched
        )
        self.context_pub = node.create_publisher(
            RouteContext, "/bio_nav/route_context", qos
        )
        self.route_pub = node.create_publisher(
            CanonicalRoute, "/bio_nav/canonical_route", qos_latched
        )
        self.progress_pub = node.create_publisher(
            RouteProgress, "/bio_nav/route_progress", qos
        )
        self.lookahead_pub = node.create_publisher(
            PoseStamped, "/bio_nav/route_lookahead_goal", qos
        )
        self.goal_update_pub = node.create_publisher(
            PoseStamped, "/goal_update", qos
        )
        self.goal_complete_pub = node.create_publisher(
            Bool,
            str(node.get_parameter("goal_complete_topic").value),
            qos,
        )
        self.runtime_pub = node.create_publisher(
            RuntimeEdgeStateArray, "/bio_nav/runtime_edge_states", qos_latched
        )
        self.status_pub = node.create_publisher(
            StructuralGraphStatus, "/bio_nav/structural_graph_status", qos_latched
        )
        node.create_subscription(
            PoseStamped,
            str(node.get_parameter("route_goal_topic").value),
            self._on_goal,
            qos,
        )
        node.create_subscription(
            EdgePriorArray,
            str(node.get_parameter("edge_prior_topic").value),
            self._on_priors,
            qos,
        )
        node.create_subscription(
            RuntimeEdgeObservation,
            str(node.get_parameter("runtime_observation_topic").value),
            self._on_runtime_observation,
            qos,
        )
        node.create_subscription(
            OccupancyGrid,
            str(node.get_parameter("structural_map_topic").value),
            self._on_structural_map,
            qos,
        )
        node.create_subscription(
            Bool,
            str(node.get_parameter("goal_complete_topic").value),
            self._on_goal_complete,
            qos,
        )
        node.create_subscription(
            Odometry,
            str(node.get_parameter("odometry_topic").value),
            self._on_odometry,
            qos,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.route_client = ActionClient(
            node,
            ComputeRoute,
            str(node.get_parameter("compute_route_action").value),
        )
        self.navigation_client = ActionClient(
            node,
            NavigateToPose,
            str(node.get_parameter("navigate_to_pose_action").value),
        )
        self.dynamic_client = node.create_client(
            DynamicEdges,
            str(node.get_parameter("dynamic_edges_service").value),
        )
        self.set_graph_client = node.create_client(
            SetRouteGraph,
            str(node.get_parameter("set_route_graph_service").value),
        )
        node.create_timer(0.02, self._check_prior_timeout)
        node.create_timer(
            1.0 / float(self.defaults["route_tracking"]["update_rate_hz"]),
            self._publish_progress,
        )
        node.create_timer(1.0, self._runtime_tick)
        self._publish_graph()
        self._publish_structural_status(StructuralGraphStatus.READY, "initial graph ready")

    def _now(self):
        return self.node.get_clock().now()

    def _on_odometry(self, message) -> None:
        self.latest_pose_xy = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )

    def _current_xy(self) -> tuple[float, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.base_frame_id,
                self.Time(),
                # This runs in the 5 Hz progress callback. Waiting for TF for
                # a full timer period starves goal/prior callbacks in rclpy's
                # default callback group. Read the latest cached transform and
                # immediately use the odometry fallback when it is absent.
                timeout=self.Duration(seconds=0.0),
            )
            return (
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
            )
        except Exception:
            return self.latest_pose_xy

    def _on_goal(self, goal) -> None:
        if self.navigation_goal_handle is not None:
            self.navigation_goal_handle.cancel_goal_async()
        self.navigation_goal_handle = None
        self.navigation_goal_pending = False
        self.navigation_failed = False
        self.request_id += 1
        self.pending_goal = goal
        self.route_active = True
        self.latest_priors = {}
        context = self.RouteContext()
        context.header.stamp = self._now().to_msg()
        context.header.frame_id = self.frame_id
        context.request_id = self.request_id
        context.graph_id = self.graph.graph_id
        context.graph_revision = self.graph.revision
        context.final_goal = goal
        context.module2_enabled = self.module2_enabled
        self.context_pub.publish(context)
        if self.module2_enabled:
            self.pending_deadline_ns = int(self._now().nanoseconds) + int(
                float(self.defaults["module2_edge_prior"]["response_timeout_s"]) * 1.0e9
            )
        else:
            self.pending_deadline_ns = None
            self._prepare_route({})

    def _on_priors(self, message) -> None:
        if (
            self.pending_goal is None
            or int(message.request_id) != self.request_id
            or str(message.graph_id) != self.graph.graph_id
            or int(message.graph_revision) != self.graph.revision
        ):
            return
        self.pending_deadline_ns = None
        priors = {
            int(item.edge_id): (float(item.cost_delta_m), float(item.confidence))
            for item in message.priors
        } if message.healthy else {}
        self.latest_priors = priors
        self._prepare_route(priors)

    def _check_prior_timeout(self) -> None:
        if (
            self.pending_goal is not None
            and self.pending_deadline_ns is not None
            and int(self._now().nanoseconds) >= self.pending_deadline_ns
        ):
            self.pending_deadline_ns = None
            self.node.get_logger().info("Module2 edge prior timed out; using geometry-only route")
            self._prepare_route({})

    def _nearest_node(self, xy: tuple[float, float]) -> int:
        return min(
            self.graph.nodes,
            key=lambda node: (math.dist(node.position_xy, xy), node.id),
        ).id

    def _prepare_route(self, priors: dict[int, tuple[float, float]]) -> None:
        current = self._current_xy()
        if current is None or self.pending_goal is None:
            self.node.get_logger().warning("route request has no map pose")
            return
        goal_xy = (
            float(self.pending_goal.pose.position.x),
            float(self.pending_goal.pose.position.y),
        )
        start_node = self._nearest_node(current)
        goal_node = self._nearest_node(goal_xy)
        request = self.DynamicEdges.Request()
        runtime_view = self.runtime.route_cost_view()
        edge_map = self.graph.edge_by_id()
        for canonical_id, support_ids in self.support.canonical_to_support_edges.items():
            edge = edge_map[canonical_id]
            prior = priors.get(canonical_id, (0.0, 0.0))
            runtime_penalty, blocked = runtime_view.get(canonical_id, (0.0, False))
            if blocked:
                request.closed_edges.extend(support_ids)
            else:
                request.opened_edges.extend(support_ids)
            total = edge_cost(
                edge,
                self.defaults["route_cost"],
                prior_cost_delta_m=prior[0],
                prior_confidence=prior[1],
                runtime_penalty_m=runtime_penalty,
            )
            extra = max(0.0, total - edge.length_m)
            for support_id in support_ids:
                adjustment = __import__("nav2_msgs.msg", fromlist=["EdgeCost"]).EdgeCost()
                adjustment.edgeid = int(support_id)
                adjustment.cost = float(extra / max(1, len(support_ids)))
                request.adjust_edges.append(adjustment)
        if not self.dynamic_client.service_is_ready():
            self.node.get_logger().warning("DynamicEdges service unavailable")
            return
        future = self.dynamic_client.call_async(request)
        future.add_done_callback(
            lambda completed, start=start_node, goal=goal_node: self._after_edge_update(
                completed, start, goal
            )
        )

    def _after_edge_update(self, future, start_node: int, goal_node: int) -> None:
        try:
            response = future.result()
        except Exception as error:
            self.node.get_logger().warning(f"edge update failed: {error}")
            return
        if response is None or not response.success or not self.route_client.server_is_ready():
            self.node.get_logger().warning("route services are not ready")
            return
        goal = self.ComputeRoute.Goal()
        goal.start_id = int(self.support.canonical_to_support_nodes[start_node])
        goal.goal_id = int(self.support.canonical_to_support_nodes[goal_node])
        goal.use_start = True
        goal.use_poses = False
        future = self.route_client.send_goal_async(goal)
        future.add_done_callback(self._on_route_goal_handle)

    def _on_route_goal_handle(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.node.get_logger().warning("ComputeRoute rejected")
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_route_result)

    def _on_route_result(self, future) -> None:
        wrapped = future.result()
        if wrapped is None or int(wrapped.result.error_code) != 0:
            code = -1 if wrapped is None else int(wrapped.result.error_code)
            self.node.get_logger().warning(f"ComputeRoute failed with error {code}")
            return
        canonical_ids = []
        for support_edge in wrapped.result.route.edges:
            canonical = self.support.support_to_canonical_edge.get(int(support_edge.edgeid))
            if canonical is not None and (not canonical_ids or canonical_ids[-1] != canonical):
                canonical_ids.append(canonical)
        if not canonical_ids:
            self.node.get_logger().warning("ComputeRoute returned no canonical edges")
            return
        edge_map = self.graph.edge_by_id()
        node_ids = [edge_map[canonical_ids[0]].from_node]
        node_ids.extend(edge_map[edge_id].to_node for edge_id in canonical_ids)
        message = self.CanonicalRoute()
        message.header.stamp = self._now().to_msg()
        message.header.frame_id = self.frame_id
        message.request_id = self.request_id
        message.graph_id = self.graph.graph_id
        message.graph_revision = self.graph.revision
        message.node_ids = node_ids
        message.edge_ids = canonical_ids
        message.total_cost_m = float(wrapped.result.route.route_cost)
        self.route_pub.publish(message)
        self.tracker = RouteTracker(
            self.graph, canonical_ids, self.defaults["route_tracking"]
        )
        self.navigation_failed = False

    def _publish_progress(self) -> None:
        if self.tracker is None or self.pending_goal is None:
            return
        current = self._current_xy()
        if current is None:
            return
        progress = self.tracker.update(current)
        message = self.RouteProgress()
        message.header.stamp = self._now().to_msg()
        message.header.frame_id = self.frame_id
        message.request_id = self.request_id
        message.edge_id = progress.edge_id
        message.edge_index = progress.edge_index
        message.arc_length_m = progress.arc_length_m
        message.lateral_error_m = progress.lateral_error_m
        message.remaining_m = progress.remaining_m
        message.projected_point.x = progress.projected_xy[0]
        message.projected_point.y = progress.projected_xy[1]
        if progress.use_final_goal:
            message.lookahead_goal = self.pending_goal
        else:
            message.lookahead_goal.header = message.header
            message.lookahead_goal.pose.position.x = progress.lookahead_xy[0]
            message.lookahead_goal.pose.position.y = progress.lookahead_xy[1]
            yaw = math.atan2(progress.lookahead_xy[1] - current[1], progress.lookahead_xy[0] - current[0])
            message.lookahead_goal.pose.orientation.z = math.sin(yaw * 0.5)
            message.lookahead_goal.pose.orientation.w = math.cos(yaw * 0.5)
        self.progress_pub.publish(message)
        self.lookahead_pub.publish(message.lookahead_goal)
        self.goal_update_pub.publish(message.lookahead_goal)
        if (
            self.execute_navigation
            and not self.navigation_failed
            and not self.navigation_goal_pending
            and self.navigation_goal_handle is None
        ):
            self._start_navigation(message.lookahead_goal)

    def _start_navigation(self, first_lookahead) -> None:
        if not self.navigation_client.server_is_ready():
            return
        goal = self.NavigateToPose.Goal()
        goal.pose = first_lookahead
        goal.behavior_tree = self.route_guided_bt_xml
        self.navigation_goal_pending = True
        future = self.navigation_client.send_goal_async(goal)
        future.add_done_callback(self._on_navigation_goal_handle)

    def _on_navigation_goal_handle(self, future) -> None:
        self.navigation_goal_pending = False
        handle = future.result()
        if handle is None or not handle.accepted:
            self.navigation_failed = True
            failed = __import__("std_msgs.msg", fromlist=["Bool"]).Bool()
            failed.data = False
            self.goal_complete_pub.publish(failed)
            self.node.get_logger().warning("route-guided NavigateToPose rejected")
            return
        self.navigation_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_navigation_result)

    def _on_navigation_result(self, future) -> None:
        wrapped = future.result()
        self.navigation_goal_handle = None
        result_code = -1 if wrapped is None else int(wrapped.result.error_code)
        if wrapped is not None and result_code == 0:
            completed = __import__("std_msgs.msg", fromlist=["Bool"]).Bool()
            completed.data = True
            self.goal_complete_pub.publish(completed)
            self.node.get_logger().info("route-guided navigation completed")
            return
        self.navigation_failed = True
        failed = __import__("std_msgs.msg", fromlist=["Bool"]).Bool()
        failed.data = False
        self.goal_complete_pub.publish(failed)
        self.node.get_logger().warning(
            f"route-guided navigation failed with error {result_code}"
        )

    def _on_runtime_observation(self, message) -> None:
        now_s = self._now().nanoseconds / 1.0e9
        edge_id = int(message.edge_id)
        previous = self.runtime.state(edge_id).state
        if message.observed_clear:
            state = self.runtime.observe_clear(edge_id, now_s)
        elif message.planning_failed:
            state = self.runtime.observe_failure(
                edge_id, now_s, occupied_ahead=bool(message.occupied_ahead)
            )
        else:
            return
        self._publish_runtime_states()
        if (
            state.state != previous
            and (
                state.state == RuntimeState.BLOCKED
                or previous == RuntimeState.BLOCKED
            )
            and self.pending_goal is not None
        ):
            self._prepare_route(self.latest_priors)

    def _runtime_tick(self) -> None:
        changed = self.runtime.tick(self._now().nanoseconds / 1.0e9)
        if changed:
            self._publish_runtime_states()
            if self.pending_goal is not None:
                # UNKNOWN is a different Route Server view than BLOCKED: the
                # edge becomes traversable again with the adjustable unknown
                # penalty. Keep DynamicEdges synchronized with every state
                # transition, not only the initial BLOCKED transition.
                self._prepare_route(self.latest_priors)

    def _publish_runtime_states(self) -> None:
        from bio_nav_interfaces.msg import RuntimeEdgeState

        message = self.RuntimeEdgeStateArray()
        message.header.stamp = self._now().to_msg()
        message.header.frame_id = self.frame_id
        message.graph_id = self.graph.graph_id
        message.graph_revision = self.graph.revision
        for state in sorted(self.runtime.edges.values(), key=lambda item: item.edge_id):
            item = RuntimeEdgeState()
            item.edge_id = state.edge_id
            item.state = int(state.state)
            item.penalty_m = state.penalty_m
            item.consecutive_failures = state.consecutive_failures
            item.state_changed_stamp = self._seconds_to_stamp(state.state_changed_s)
            if state.first_failure_s is not None:
                item.first_failure_stamp = self._seconds_to_stamp(state.first_failure_s)
            if state.last_observed_s is not None:
                item.last_observed_stamp = self._seconds_to_stamp(state.last_observed_s)
            message.states.append(item)
        self.runtime_pub.publish(message)

    @staticmethod
    def _seconds_to_stamp(value: float):
        from builtin_interfaces.msg import Time

        stamp = Time()
        stamp.sec = int(value)
        stamp.nanosec = int((value - int(value)) * 1.0e9)
        return stamp

    def _on_structural_map(self, message) -> None:
        values = np.asarray(message.data, dtype=np.int16).reshape(
            int(message.info.height), int(message.info.width)
        )
        maximum = int(self.defaults["structural_updates"]["ros_free_max_occupancy"])
        free = (values >= 0) & (values <= maximum)
        if free.shape != self.map.free.shape:
            self.node.get_logger().warning("structural map shape does not match baseline")
            return
        now_s = self._now().nanoseconds / 1.0e9
        if self.structural_monitor.observe(free, now_s):
            self.pending_structural_map = OccupancyMap(
                free=free,
                # Structural snapshots are required to match the current grid
                # shape. Reuse its canonical metric geometry as well: ROS
                # OccupancyGrid stores resolution/origin as float32, and a
                # round-trip 0.05 m value otherwise perturbs support-node
                # rounding and creates needless stable-ID churn.
                resolution_m=self.map.resolution_m,
                origin_xy_m=self.map.origin_xy_m,
                map_version=f"{self.map.map_version}:structural",
                yaml_path=self.map.yaml_path,
            )
            if not self.route_active:
                self._rebuild_structural_graph()

    def _on_goal_complete(self, message) -> None:
        if not message.data:
            return
        if self.navigation_goal_handle is not None:
            self.navigation_goal_handle.cancel_goal_async()
        self.route_active = False
        self.pending_goal = None
        self.tracker = None
        self.navigation_goal_pending = False
        self.navigation_goal_handle = None
        self.navigation_failed = False
        if self.pending_structural_map is not None:
            self._rebuild_structural_graph()

    def _rebuild_structural_graph(self) -> None:
        candidate_map = self.pending_structural_map
        if candidate_map is None:
            return
        self._publish_structural_status(
            self.StructuralGraphStatus.REBUILDING, "persistent structural update"
        )
        try:
            candidate = apply_footprint_feasibility(
                build_gvg(
                    candidate_map,
                    self.defaults["graph"],
                    self.defaults["footprint"],
                    self.defaults["route_cost"],
                    revision=self.graph.revision + 1,
                ),
                candidate_map,
                self.defaults["footprint"],
            )
            candidate = stabilize_graph_ids(candidate, self.graph, self.defaults["graph"])
            support = export_route_support_graph(
                candidate,
                support_spacing_m=float(self.defaults["graph"]["route_support_spacing_m"]),
            )
            directory = Path(tempfile.gettempdir()) / "bio_nav_attempt30_a21"
            directory.mkdir(parents=True, exist_ok=True)
            geojson = directory / "structural_graph.geojson"
            mapping = directory / "structural_graph_support_map.json"
            save_route_support(support, geojson, mapping)
        except Exception as error:
            self._publish_structural_status(
                self.StructuralGraphStatus.LAST_KNOWN_GOOD,
                f"rebuild failed: {error}",
            )
            return
        if not self.set_graph_client.service_is_ready():
            self._publish_structural_status(
                self.StructuralGraphStatus.LAST_KNOWN_GOOD,
                "SetRouteGraph service unavailable",
            )
            return
        request = self.SetRouteGraph.Request()
        request.graph_filepath = str(geojson)
        future = self.set_graph_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._finish_rebuild(
                completed, candidate, candidate_map, support
            )
        )

    def _finish_rebuild(self, future, graph, occupancy, support) -> None:
        try:
            response = future.result()
        except Exception as error:
            response = None
            self.node.get_logger().warning(f"SetRouteGraph failed: {error}")
        if response is None or not response.success:
            self._publish_structural_status(
                self.StructuralGraphStatus.LAST_KNOWN_GOOD,
                "Route Server rejected rebuilt graph",
            )
            return
        self.graph = graph
        self.map = occupancy
        self.support = support
        self.structural_monitor.accept_rebuild()
        self.pending_structural_map = None
        self._publish_graph()
        self._publish_structural_status(
            self.StructuralGraphStatus.READY, "rebuilt graph active"
        )

    def _publish_graph(self) -> None:
        from bio_nav_interfaces.msg import NavigationEdge, NavigationGraph, NavigationNode
        from geometry_msgs.msg import Point

        message = NavigationGraph()
        message.header.stamp = self._now().to_msg()
        message.header.frame_id = self.frame_id
        message.graph_id = self.graph.graph_id
        message.revision = self.graph.revision
        message.map_version = self.graph.map_version
        message.resolution_m = self.graph.resolution_m
        for node in self.graph.nodes:
            item = NavigationNode()
            item.id = node.id
            item.position.x, item.position.y = node.position_xy
            item.degree = node.degree
            item.node_type = int(node.node_type)
            item.clearance_m = node.clearance_m
            message.nodes.append(item)
        for edge in self.graph.edges:
            item = NavigationEdge()
            item.id = edge.id
            item.from_node = edge.from_node
            item.to_node = edge.to_node
            for x, y in edge.polyline_xy:
                point = Point()
                point.x = float(x)
                point.y = float(y)
                item.polyline.append(point)
            item.length_m = edge.length_m
            item.min_clearance_m = edge.min_clearance_m
            item.mean_clearance_m = edge.mean_clearance_m
            item.p05_clearance_m = edge.p05_clearance_m
            item.nominal_width_m = edge.nominal_width_m
            item.max_curvature_per_m = edge.max_curvature_per_m
            item.bottleneck = edge.bottleneck
            item.static_traversability = int(edge.static_traversability)
            item.predecessor_ids = list(edge.predecessor_ids)
            message.edges.append(item)
        self.graph_pub.publish(message)

    def _publish_structural_status(self, state: int, detail: str) -> None:
        message = self.StructuralGraphStatus()
        message.header.stamp = self._now().to_msg()
        message.header.frame_id = self.frame_id
        message.graph_id = self.graph.graph_id
        message.graph_revision = self.graph.revision
        message.state = int(state)
        message.detail = detail
        self.status_pub.publish(message)


def main() -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

    rclpy.init()
    node = rclpy.create_node("bio_nav_route_coordinator")
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        RouteCoordinator(node)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            # ros2 launch may deliver SIGINT while rclpy entities are already
            # being destroyed. The process is shutting down as requested.
            pass


if __name__ == "__main__":
    main()
