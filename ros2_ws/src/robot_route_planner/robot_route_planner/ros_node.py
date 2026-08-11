"""A21 Module3 coordinator for graph, official route search, and guidance."""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
import math
import tempfile

import numpy as np

from .defaults import load_engineering_defaults
from .feasibility import apply_footprint_feasibility, classify_edge
from .gvg import build_gvg
from .map_io import OccupancyMap, load_occupancy_map
from .route_cost import edge_cost
from .route_support import export_route_support_graph, save_route_support
from .runtime_edges import RuntimeEdgeManager, RuntimeState
from .stable_ids import stabilize_graph_ids
from .structural_updates import StructuralChangeMonitor
from .tracking import RouteTracker


def select_map_pose(
    map_frame_id: str,
    odometry_frame_id: str | None,
    odometry_xy: tuple[float, float] | None,
    tf_xy: tuple[float, float] | None,
) -> tuple[float, float] | None:
    """Prefer an explicit map-frame pose over a potentially transient TF."""
    if odometry_xy is not None and odometry_frame_id == map_frame_id:
        return odometry_xy
    return tf_xy


def populate_fresh_goal(target, source, header) -> None:
    """Copy a final goal pose while retaining the newest progress timestamp."""
    target.header = header
    target.pose = source.pose


@dataclass(frozen=True)
class CostmapSnapshot:
    """Immutable geometry and values from one live Nav2 global costmap."""

    values: np.ndarray
    resolution_m: float
    origin_xy: tuple[float, float]
    frame_id: str


def footprint_is_free(
    costmap: CostmapSnapshot,
    position_xy: tuple[float, float],
    yaw_rad: float,
    footprint_xy: np.ndarray,
    lethal_cost: int = 253,
) -> bool:
    """Check a posed footprint against live lethal/unknown costmap cells."""

    rotation = np.asarray(
        [
            [math.cos(yaw_rad), -math.sin(yaw_rad)],
            [math.sin(yaw_rad), math.cos(yaw_rad)],
        ],
        dtype=np.float64,
    )
    polygon = np.asarray(footprint_xy, dtype=np.float64) @ rotation.T
    polygon += np.asarray(position_xy, dtype=np.float64)
    resolution = float(costmap.resolution_m)
    origin = np.asarray(costmap.origin_xy, dtype=np.float64)
    height, width = costmap.values.shape
    lower = np.floor((polygon.min(axis=0) - origin) / resolution).astype(int)
    upper = np.floor((polygon.max(axis=0) - origin) / resolution).astype(int)
    if lower[0] < 0 or lower[1] < 0 or upper[0] >= width or upper[1] >= height:
        return False

    columns = np.arange(lower[0], upper[0] + 1)
    rows = np.arange(lower[1], upper[1] + 1)
    grid_x, grid_y = np.meshgrid(
        origin[0] + (columns + 0.5) * resolution,
        origin[1] + (rows + 0.5) * resolution,
    )
    points = np.column_stack((grid_x.ravel(), grid_y.ravel()))
    inside = np.zeros(len(points), dtype=bool)
    previous = polygon[-1]
    for current in polygon:
        crosses = (current[1] > points[:, 1]) != (previous[1] > points[:, 1])
        denominator = previous[1] - current[1]
        x_crossing = (
            (previous[0] - current[0])
            * (points[:, 1] - current[1])
            / (denominator if abs(denominator) > 1.0e-12 else 1.0e-12)
            + current[0]
        )
        inside ^= crosses & (points[:, 0] < x_crossing)
        previous = current
    row_grid, column_grid = np.meshgrid(rows, columns, indexing="ij")
    selected_rows = row_grid.ravel()[inside]
    selected_columns = column_grid.ravel()[inside]
    if np.any(costmap.values[selected_rows, selected_columns] >= lethal_cost):
        return False

    for start, end in zip(polygon, np.roll(polygon, -1, axis=0)):
        count = max(
            2,
            int(math.ceil(np.linalg.norm(end - start) / (0.5 * resolution))),
        )
        for fraction in np.linspace(0.0, 1.0, count):
            point = start + fraction * (end - start)
            column, row = np.floor((point - origin) / resolution).astype(int)
            if costmap.values[row, column] >= lethal_cost:
                return False
    return True


def select_live_feasible_lookahead(
    tracker: RouteTracker,
    current_xy: tuple[float, float],
    progress,
    costmap: CostmapSnapshot | None,
    footprint_xy: np.ndarray,
    nominal_distance_m: float,
    sample_spacing_m: float,
):
    """Advance only the metric target past a live obstacle on the same Route."""

    if progress.use_final_goal or costmap is None:
        return progress
    maximum = min(progress.remaining_m, 2.0 * nominal_distance_m)
    spacing = max(float(sample_spacing_m), float(costmap.resolution_m))
    distances = np.arange(nominal_distance_m, maximum + 0.5 * spacing, spacing)
    for distance in distances:
        candidate = tracker.point_at_distance_ahead(float(distance))
        yaw = math.atan2(
            candidate[1] - current_xy[1], candidate[0] - current_xy[0]
        )
        if footprint_is_free(costmap, candidate, yaw, footprint_xy):
            return replace(progress, lookahead_xy=candidate)
    return progress


def select_support_attachment(
    occupancy: OccupancyMap,
    support_nodes: dict[int, tuple[float, float]],
    position_xy: tuple[float, float],
    footprint_settings: dict,
    *,
    departing: bool,
) -> int:
    """Choose the nearest support point with a direct feasible connector."""

    ordered = sorted(
        support_nodes.items(),
        key=lambda item: (math.dist(position_xy, item[1]), item[0]),
    )
    if not ordered:
        raise ValueError("Route support graph contains no nodes")
    for node_id, node_xy in ordered:
        if math.dist(position_xy, node_xy) <= occupancy.resolution_m:
            return node_id
        endpoints = (
            np.asarray([position_xy, node_xy], dtype=np.float64)
            if departing
            else np.asarray([node_xy, position_xy], dtype=np.float64)
        )
        if classify_edge(
            occupancy,
            endpoints,
            footprint_polygon_m=np.asarray(
                footprint_settings["polygon_m"], dtype=np.float64
            ),
            footprint_padding_m=float(footprint_settings["padding_m"]),
            padded_inscribed_radius_m=float(
                footprint_settings["padded_inscribed_radius_m"]
            ),
            sweep_sample_spacing_m=float(
                footprint_settings["sweep_sample_spacing_m"]
            ),
        ).name == "FEASIBLE":
            return node_id
    return ordered[0][0]


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
        from nav2_msgs.msg import Costmap
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
        self.support_node_positions = {
            int(feature["properties"]["id"]): tuple(
                float(value) for value in feature["geometry"]["coordinates"]
            )
            for feature in self.support.geojson["features"]
            if feature["geometry"]["type"] == "Point"
        }
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
        self.latest_pose_frame_id: str | None = None
        self.latest_global_costmap: CostmapSnapshot | None = None
        footprint = np.asarray(
            self.defaults["footprint"]["polygon_m"], dtype=np.float64
        )
        padding = float(self.defaults["footprint"]["padding_m"])
        norms = np.linalg.norm(footprint, axis=1)
        self.guidance_footprint = footprint + (
            footprint / np.maximum(norms[:, None], 1.0e-12) * padding
        )
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
            Odometry,
            str(node.get_parameter("odometry_topic").value),
            self._on_odometry,
            qos,
        )
        node.create_subscription(
            Costmap,
            "/global_costmap/costmap_raw",
            self._on_global_costmap,
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
        self.latest_pose_frame_id = str(message.header.frame_id)

    def _on_global_costmap(self, message) -> None:
        width = int(message.metadata.size_x)
        height = int(message.metadata.size_y)
        values = np.asarray(message.data, dtype=np.uint8)
        if width <= 0 or height <= 0 or values.size != width * height:
            return
        self.latest_global_costmap = CostmapSnapshot(
            values=values.reshape(height, width),
            resolution_m=float(message.metadata.resolution),
            origin_xy=(
                float(message.metadata.origin.position.x),
                float(message.metadata.origin.position.y),
            ),
            frame_id=str(message.header.frame_id),
        )

    def _current_xy(self) -> tuple[float, float] | None:
        # Qualification and ideal-odometry launches provide a high-rate pose
        # already expressed in map. Prefer that explicit contract: a reset can
        # briefly leave a cached map->base_link transform internally
        # inconsistent with map-frame ground truth, which must not advance the
        # monotonic Route tracker to a distant edge. Localized /odom inputs are
        # not map-frame, so they continue through the normal TF path below.
        if (
            self.latest_pose_xy is not None
            and self.latest_pose_frame_id == self.frame_id
        ):
            return self.latest_pose_xy
        tf_xy = None
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
            tf_xy = (
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
            )
        except Exception:
            pass
        return select_map_pose(
            self.frame_id,
            self.latest_pose_frame_id,
            self.latest_pose_xy,
            tf_xy,
        )

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

    def _nearest_support_node(
        self, xy: tuple[float, float], *, departing: bool
    ) -> int:
        return select_support_attachment(
            self.map,
            self.support_node_positions,
            xy,
            self.defaults["footprint"],
            departing=departing,
        )

    def _prepare_route(self, priors: dict[int, tuple[float, float]]) -> None:
        current = self._current_xy()
        if current is None or self.pending_goal is None:
            self.node.get_logger().warning("route request has no map pose")
            return
        goal_xy = (
            float(self.pending_goal.pose.position.x),
            float(self.pending_goal.pose.position.y),
        )
        start_node = self._nearest_support_node(current, departing=True)
        goal_node = self._nearest_support_node(goal_xy, departing=False)
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
        goal.start_id = int(start_node)
        goal.goal_id = int(goal_node)
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
        route_segments: list[list[tuple[float, float]]] = []
        for support_edge in wrapped.result.route.edges:
            canonical = self.support.support_to_canonical_edge.get(
                int(support_edge.edgeid)
            )
            if canonical is None:
                continue
            start = (float(support_edge.start.x), float(support_edge.start.y))
            end = (float(support_edge.end.x), float(support_edge.end.y))
            if not canonical_ids or canonical_ids[-1] != canonical:
                canonical_ids.append(canonical)
                route_segments.append([start, end])
            else:
                route_segments[-1].append(end)
        if not canonical_ids:
            self.node.get_logger().warning("ComputeRoute returned no canonical edges")
            return
        edge_map = self.graph.edge_by_id()
        node_ids = []
        for index, (canonical, points) in enumerate(
            zip(canonical_ids, route_segments)
        ):
            edge = edge_map[canonical]
            starts_forward = math.dist(
                points[0], tuple(edge.polyline_xy[0])
            ) <= math.dist(points[0], tuple(edge.polyline_xy[-1]))
            source = edge.from_node if starts_forward else edge.to_node
            target = edge.to_node if starts_forward else edge.from_node
            if index == 0:
                node_ids.append(source)
            node_ids.append(target)
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
            self.graph,
            canonical_ids,
            self.defaults["route_tracking"],
            route_segments_xy=[
                np.asarray(points, dtype=np.float64) for points in route_segments
            ],
        )
        self.navigation_failed = False

    def _publish_progress(self) -> None:
        if self.tracker is None or self.pending_goal is None:
            return
        current = self._current_xy()
        if current is None:
            return
        progress = self.tracker.update(current)
        if (
            self.latest_global_costmap is not None
            and self.latest_global_costmap.frame_id == self.frame_id
        ):
            progress = select_live_feasible_lookahead(
                self.tracker,
                current,
                progress,
                self.latest_global_costmap,
                self.guidance_footprint,
                nominal_distance_m=float(
                    self.defaults["route_tracking"]["lookahead_m"]
                ),
                sample_spacing_m=float(
                    self.defaults["footprint"]["sweep_sample_spacing_m"]
                ),
            )
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
            # GoalUpdater rejects a goal whose stamp is older than the moving
            # lookahead it already accepted. Refresh only the header; preserve
            # the user's exact final pose and orientation.
            populate_fresh_goal(
                message.lookahead_goal, self.pending_goal, message.header
            )
        else:
            message.lookahead_goal.header = message.header
            message.lookahead_goal.pose.position.x = progress.lookahead_xy[0]
            message.lookahead_goal.pose.position.y = progress.lookahead_xy[1]
            yaw = math.atan2(
                progress.lookahead_xy[1] - current[1],
                progress.lookahead_xy[0] - current[0],
            )
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
        result_code = -1 if wrapped is None else int(wrapped.result.error_code)
        # Retire the completed leg before publishing its terminal event.  The
        # qualification runner can dispatch the next whole-house waypoint as
        # soon as it receives this Bool.  Clearing via a subscription to our
        # own publication races that new goal and can erase it before
        # ComputeRoute is requested.
        self._finish_active_route()
        if wrapped is not None and result_code == 0:
            completed = __import__("std_msgs.msg", fromlist=["Bool"]).Bool()
            completed.data = True
            self.goal_complete_pub.publish(completed)
            self.node.get_logger().info("route-guided navigation completed")
            return
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

    def _finish_active_route(self) -> None:
        """Synchronously retire one coordinator-owned Nav2 action."""

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
        self.support_node_positions = {
            int(feature["properties"]["id"]): tuple(
                float(value) for value in feature["geometry"]["coordinates"]
            )
            for feature in support.geojson["features"]
            if feature["geometry"]["type"] == "Point"
        }
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
