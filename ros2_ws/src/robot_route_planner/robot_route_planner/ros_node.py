"""A21 Module3 coordinator for graph, official route search, and guidance."""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
import math
import tempfile

import numpy as np

from .defaults import load_engineering_defaults
from .cognitive_constraints import (
    CognitiveConstraintsCache,
    build_cognitive_constraints,
    occupancy_grid_version,
)
from .feasibility import (
    apply_footprint_feasibility,
    classify_edge,
    retain_largest_feasible_component,
)
from .gvg import build_gvg
from .map_io import OccupancyMap, load_occupancy_map
from .route_cost import edge_cost_breakdown
from .route_support import export_route_support_graph, save_route_support
from .regions import RegionSelector, load_region_config
from .runtime_edges import RuntimeEdgeManager, RuntimeState
from .stable_ids import stabilize_graph_ids
from .structural_updates import StructuralChangeMonitor
from .tracking import RouteTracker


DEFAULT_ROUTE_ODOMETRY_TOPIC = "/odom"


def edge_prior_is_usable(
    *,
    healthy: bool,
    model_id: str,
    stamp_ns: int,
    now_ns: int,
    max_age_s: float,
    priors: list[tuple[int, float, float, float]],
) -> tuple[bool, str]:
    """Validate the bounded, fresh Module2 hint before it affects routing."""

    if not healthy:
        return False, "producer unhealthy"
    if not model_id.strip():
        return False, "model_id is empty"
    if stamp_ns <= 0:
        return False, "timestamp is invalid"
    age_s = (now_ns - stamp_ns) / 1.0e9
    if age_s < 0.0:
        return False, "timestamp is in the future"
    if age_s > max_age_s:
        return False, f"prior is stale ({age_s:.3f}s > {max_age_s:.3f}s)"
    for edge_id, cost_delta_m, learned_risk, confidence in priors:
        values = (cost_delta_m, learned_risk, confidence)
        if not all(math.isfinite(value) for value in values):
            return False, f"edge {edge_id} contains a non-finite value"
        if cost_delta_m < 0.0:
            return False, f"edge {edge_id} has a negative cost delta"
        if not 0.0 <= learned_risk <= 1.0:
            return False, f"edge {edge_id} learned_risk is outside [0, 1]"
        if not 0.0 <= confidence <= 1.0:
            return False, f"edge {edge_id} confidence is outside [0, 1]"
    return True, "fresh and healthy"


def validate_route_odometry_topic(topic: str) -> str:
    """Reject evaluation-only ground-truth inputs from online route tracking."""

    value = topic.strip()
    normalized_segments = value.lower().replace("-", "_").split("/")
    if any(
        segment == "groundtruth" or segment.startswith("ground_truth")
        for segment in normalized_segments
    ):
        raise ValueError(
            "RouteCoordinator odometry_topic must not use ground-truth data"
        )
    return value


def select_map_pose(
    map_frame_id: str,
    odometry_frame_id: str | None,
    odometry_xy: tuple[float, float] | None,
    odometry_age_s: float | None,
    odometry_max_age_s: float,
    tf_xy: tuple[float, float] | None,
) -> tuple[float, float] | None:
    """Prefer map->base TF; accept only a fresh, explicitly map-frame fallback."""

    if tf_xy is not None:
        return tf_xy
    if (
        odometry_xy is not None
        and odometry_frame_id == map_frame_id
        and odometry_age_s is not None
        and 0.0 <= odometry_age_s <= odometry_max_age_s
    ):
        return odometry_xy
    return None


def populate_fresh_goal(target, source, header) -> None:
    """Copy a final goal pose while retaining the newest progress timestamp."""
    target.header = header
    target.pose = source.pose


def navigation_result_succeeded(wrapped) -> bool:
    """Use the ROS action terminal status over a stale Nav2 error detail."""

    if wrapped is None:
        return False
    status = getattr(wrapped, "status", None)
    if status is not None:
        return int(status) == 4  # action_msgs/GoalStatus.STATUS_SUCCEEDED
    return int(wrapped.result.error_code) == 0


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
            CognitiveMapConstraints,
            CognitiveTransition,
            EdgePriorArray,
            NavigationGraph,
            RouteEdgeCostArray,
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
            ("module2_response_timeout_s", 0.0),
            ("module2_prior_ttl_s", 2.0),
            # Final route confirmation uses the campaign's 0.25 m waypoint
            # gate. Nav2 evaluates its 0.20 m goal checker in the local pose,
            # so requiring 0.20 m again can reject a valid final action due to
            # the small frame/pose sampling difference.
            ("route_goal_completion_tolerance_m", 0.25),
            ("feasible_only_largest_component", False),
            ("execute_navigation", True),
            ("route_guided_bt_xml", ""),
            ("route_goal_topic", "/bio_nav/route_goal"),
            ("edge_prior_topic", "/bio_nav/module2/edge_priors"),
            ("runtime_observation_topic", "/bio_nav/runtime_edge_observation"),
            ("structural_map_topic", "/bio_nav/structural_map"),
            ("occupancy_map_topic", "/map"),
            ("goal_complete_topic", "/bio_nav/route_goal_complete"),
            ("odometry_topic", DEFAULT_ROUTE_ODOMETRY_TOPIC),
            ("odometry_max_age_s", 0.5),
            ("region_config_file", ""),
            ("region_switch_min_dwell_s", 0.5),
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
        self.module2_response_timeout_s = float(
            node.get_parameter("module2_response_timeout_s").value
        )
        self.module2_prior_ttl_s = float(
            node.get_parameter("module2_prior_ttl_s").value
        )
        self.route_goal_completion_tolerance_m = float(
            node.get_parameter("route_goal_completion_tolerance_m").value
        )
        self.odometry_topic = validate_route_odometry_topic(
            str(node.get_parameter("odometry_topic").value)
        )
        validate_route_odometry_topic(
            str(node.resolve_topic_name(self.odometry_topic))
        )
        self.odometry_max_age_s = float(
            node.get_parameter("odometry_max_age_s").value
        )
        if self.module2_response_timeout_s < 0.0:
            raise ValueError("module2_response_timeout_s must be non-negative")
        if self.module2_prior_ttl_s <= 0.0:
            raise ValueError("module2_prior_ttl_s must be positive")
        if self.route_goal_completion_tolerance_m <= 0.0:
            raise ValueError("route_goal_completion_tolerance_m must be positive")
        if self.odometry_max_age_s <= 0.0:
            raise ValueError("odometry_max_age_s must be positive")
        self.map = load_occupancy_map(
            map_path,
            unknown_is_occupied=bool(self.defaults["graph"]["unknown_is_occupied"]),
        )
        self.feasible_only_largest_component = bool(
            node.get_parameter("feasible_only_largest_component").value
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
        if self.feasible_only_largest_component:
            retain_largest_feasible_component(self.graph)
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
        self.pending_prior_request_id: int | None = None
        self.pending_prior_graph_id: str | None = None
        self.pending_prior_graph_revision: int | None = None
        self.pending_prior_started_ns: int | None = None
        self.pending_prior_model_id: str | None = None
        self.request_id = 0
        self.last_context_publish_ns = 0
        self.latest_priors: dict[int, tuple[float, float]] = {}
        self.latest_priors_stamp_ns: int | None = None
        self.latest_prior_model_id: str | None = None
        self.tracker: RouteTracker | None = None
        self.latest_pose_xy: tuple[float, float] | None = None
        self.latest_pose_frame_id: str | None = None
        self.latest_pose_stamp_ns: int | None = None
        self.latest_global_costmap: CostmapSnapshot | None = None
        self.live_map_version: str | None = None
        self.cognitive_constraints_cache = CognitiveConstraintsCache()
        node.declare_parameter("cognitive_tile_cache_entries", 0)
        node.declare_parameter("cognitive_tile_cache_hits", 0)
        node.declare_parameter("cognitive_tile_cache_misses", 0)
        region_config_file = str(node.get_parameter("region_config_file").value).strip()
        self.region_selector = None
        if region_config_file:
            region_config = load_region_config(region_config_file)
            if region_config.map_frame != str(node.get_parameter("frame_id").value):
                raise ValueError("region config map_frame differs from frame_id")
            self.region_selector = RegionSelector(
                region_config,
                min_dwell_s=float(
                    node.get_parameter("region_switch_min_dwell_s").value
                ),
            )
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
        self.navigation_goal_targets_final = False
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
        self.CognitiveMapConstraints = CognitiveMapConstraints
        self.CognitiveTransition = CognitiveTransition
        self.RouteEdgeCostArray = RouteEdgeCostArray
        qos_latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        # This is a latest-state input, not an event stream.  Rivermark's
        # 1600x1600 full costmap is 2.56 MB per publication; a reliable depth
        # 10 queue can delay action-result callbacks behind stale grids and
        # stop the robot between route lookaheads.  One best-effort snapshot
        # preserves fail-closed footprint checks without creating a backlog.
        costmap_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
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
        self.cognitive_constraints_pub = node.create_publisher(
            CognitiveMapConstraints,
            "/bio_nav/cognitive_map/constraints",
            qos_latched,
        )
        self.route_edge_cost_pub = node.create_publisher(
            RouteEdgeCostArray,
            "/bio_nav/route_edge_costs",
            qos_latched,
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
            OccupancyGrid,
            str(node.get_parameter("occupancy_map_topic").value),
            self._on_occupancy_map,
            qos_latched,
        )
        node.create_subscription(
            Odometry,
            self.odometry_topic,
            self._on_odometry,
            qos,
        )
        node.create_subscription(
            Costmap,
            "/global_costmap/costmap_raw",
            self._on_global_costmap,
            costmap_qos,
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
        node.create_timer(0.2, self._region_tick)
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
        self.latest_pose_stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )

    def _region_tick(self) -> None:
        if self.region_selector is None:
            return
        current = self._current_xy()
        if current is None:
            return
        previous = self.region_selector.current
        selected = self.region_selector.select(
            current, self._now().nanoseconds / 1.0e9
        )
        if selected != previous:
            self.node.get_logger().info(
                f"active cognitive region: {selected.region_id}"
            )
            self._publish_cognitive_constraints()

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

    def _on_occupancy_map(self, message) -> None:
        """Bind cognitive constraints to the exact live ROS map bytes."""

        try:
            version = occupancy_grid_version(
                width=int(message.info.width),
                height=int(message.info.height),
                resolution=float(message.info.resolution),
                origin_x=float(message.info.origin.position.x),
                origin_y=float(message.info.origin.position.y),
                data=np.asarray(message.data, dtype=np.int8),
            )
            static_height, static_width = self.map.free.shape
            if (
                int(message.info.width) != static_width
                or int(message.info.height) != static_height
                or not math.isclose(
                    float(message.info.resolution),
                    self.map.resolution_m,
                    abs_tol=1.0e-6,
                )
                or not np.allclose(
                    (
                        float(message.info.origin.position.x),
                        float(message.info.origin.position.y),
                    ),
                    self.map.origin_xy_m,
                    atol=1.0e-5,
                )
            ):
                raise ValueError("live /map geometry differs from the frozen structural map")
            if version != self.live_map_version:
                self.live_map_version = version
                self.cognitive_constraints_cache.invalidate()
            self._publish_cognitive_constraints()
        except Exception as exc:
            self.live_map_version = None
            self.cognitive_constraints_cache.invalidate()
            self.node.get_logger().warning(f"cognitive map rejected: {exc}")

    def _publish_cognitive_constraints(self) -> None:
        if self.live_map_version is None:
            return
        region = None if self.region_selector is None else self.region_selector.current
        if self.region_selector is not None and region is None:
            return
        transform = (
            np.eye(3, dtype=np.float64)
            if region is None
            else region.t_map_canvas
        )
        tile_id = None if region is None else region.region_id
        cache_key = (
            self.live_map_version,
            int(self.graph.revision),
            tile_id,
        )
        value = self.cognitive_constraints_cache.get(cache_key)
        if value is None:
            value = build_cognitive_constraints(
                self.map,
                map_version=self.live_map_version,
                graph_revision=self.graph.revision,
                footprint_settings=self.defaults["footprint"],
                t_map_canvas=transform,
                stable_duration_s=0.0,
                persistent_confirmed=True,
                cognitive_tile_id=tile_id,
            )
            self.cognitive_constraints_cache.put(cache_key, value)
        self._publish_cognitive_cache_parameters()
        message = self.CognitiveMapConstraints()
        message.header.stamp = self._now().to_msg()
        message.header.frame_id = self.frame_id
        message.map_version = value.map_version
        message.cognitive_tile_id = value.cognitive_tile_id
        message.tile_revision = value.tile_revision
        message.graph_id = self.graph.graph_id
        message.graph_revision = value.graph_revision
        message.grid_width = 16
        message.grid_height = 16
        message.resolution_m = 1.0
        message.t_map_canvas = value.t_map_canvas.reshape(-1).tolist()
        message.reachable_state_mask = value.reachable_state_mask.tolist()
        for source, target in value.verified_transitions:
            transition = self.CognitiveTransition()
            transition.from_state = int(source)
            transition.to_state = int(target)
            message.verified_transitions.append(transition)
        message.structural_confidence = value.structural_confidence
        message.stable_duration_s = value.stable_duration_s
        message.persistent_confirmed = value.persistent_confirmed
        self.cognitive_constraints_pub.publish(message)

    def _publish_cognitive_cache_parameters(self) -> None:
        from rclpy.parameter import Parameter

        cache = self.cognitive_constraints_cache
        self.node.set_parameters(
            [
                Parameter(
                    "cognitive_tile_cache_entries",
                    value=len(cache.values),
                ),
                Parameter("cognitive_tile_cache_hits", value=cache.hits),
                Parameter("cognitive_tile_cache_misses", value=cache.misses),
            ]
        )

    def _current_xy(self) -> tuple[float, float] | None:
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
        odometry_age_s = None
        if self.latest_pose_stamp_ns is not None:
            odometry_age_s = (
                self._now().nanoseconds - self.latest_pose_stamp_ns
            ) / 1.0e9
        return select_map_pose(
            self.frame_id,
            self.latest_pose_frame_id,
            self.latest_pose_xy,
            odometry_age_s,
            self.odometry_max_age_s,
            tf_xy,
        )

    def _on_goal(self, goal) -> None:
        if self.navigation_goal_handle is not None:
            self.navigation_goal_handle.cancel_goal_async()
        self.navigation_goal_handle = None
        self.navigation_goal_pending = False
        self.navigation_goal_targets_final = False
        self.navigation_failed = False
        self.request_id += 1
        self.pending_goal = goal
        self.route_active = True
        self.latest_priors = {}
        self.latest_priors_stamp_ns = None
        self.latest_prior_model_id = None
        self.node.get_logger().info(
            "received route goal request "
            f"{self.request_id}: ({goal.pose.position.x:.3f}, "
            f"{goal.pose.position.y:.3f})"
        )
        if self.module2_enabled:
            self._arm_prior_request(int(self._now().nanoseconds))
        else:
            self._clear_pending_prior_request()
        self._publish_route_context()
        if not self.module2_enabled:
            self._prepare_route({})

    def _arm_prior_request(self, now_ns: int) -> None:
        timeout_s = self.module2_response_timeout_s or float(
            self.defaults["module2_edge_prior"]["response_timeout_s"]
        )
        self.pending_deadline_ns = now_ns + int(timeout_s * 1.0e9)
        self.pending_prior_request_id = self.request_id
        self.pending_prior_graph_id = self.graph.graph_id
        self.pending_prior_graph_revision = self.graph.revision
        self.pending_prior_started_ns = now_ns
        self.pending_prior_model_id = self.latest_prior_model_id

    def _clear_pending_prior_request(self) -> None:
        self.pending_deadline_ns = None
        self.pending_prior_request_id = None
        self.pending_prior_graph_id = None
        self.pending_prior_graph_revision = None
        self.pending_prior_started_ns = None
        self.pending_prior_model_id = None

    def _publish_route_context(self) -> None:
        if self.pending_goal is None:
            return
        context = self.RouteContext()
        context.header.stamp = self._now().to_msg()
        context.header.frame_id = self.frame_id
        context.request_id = self.request_id
        context.graph_id = self.graph.graph_id
        context.graph_revision = self.graph.revision
        context.final_goal = self.pending_goal
        context.module2_enabled = self.module2_enabled
        self.context_pub.publish(context)
        self.last_context_publish_ns = int(self._now().nanoseconds)

    def _on_priors(self, message) -> None:
        now_ns = int(self._now().nanoseconds)
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        if (
            self.pending_goal is None
            or self.pending_deadline_ns is None
            or now_ns >= self.pending_deadline_ns
            or self.pending_prior_request_id != self.request_id
            or self.pending_prior_graph_id != self.graph.graph_id
            or self.pending_prior_graph_revision != self.graph.revision
            or self.pending_prior_started_ns is None
            or stamp_ns < self.pending_prior_started_ns
            or int(message.request_id) != self.request_id
            or str(message.graph_id) != self.graph.graph_id
            or int(message.graph_revision) != self.graph.revision
            or (
                self.pending_prior_model_id is not None
                and str(message.model_id) != self.pending_prior_model_id
            )
        ):
            return
        self._clear_pending_prior_request()
        edge_ids = {int(edge.id) for edge in self.graph.edges}
        observed_ids = [int(item.edge_id) for item in message.priors]
        if len(observed_ids) != len(set(observed_ids)) or not set(observed_ids).issubset(
            edge_ids
        ):
            self.node.get_logger().warning(
                "Module2 prior contains duplicate or nonexistent graph edges; ignored"
            )
            self.latest_priors = {}
            self.latest_priors_stamp_ns = None
            self._prepare_route({})
            return
        rows = [
            (
                int(item.edge_id),
                float(item.cost_delta_m),
                float(item.learned_risk),
                float(item.confidence),
            )
            for item in message.priors
        ]
        usable, reason = edge_prior_is_usable(
            healthy=bool(message.healthy),
            model_id=str(message.model_id),
            stamp_ns=stamp_ns,
            now_ns=now_ns,
            max_age_s=self.module2_prior_ttl_s,
            priors=rows,
        )
        if not usable:
            self.latest_priors = {}
            self.latest_priors_stamp_ns = None
            self.node.get_logger().warning(
                f"Module2 edge prior rejected ({reason}); using geometry-only route"
            )
            self._prepare_route({})
            return
        priors = {
            edge_id: (cost_delta_m, confidence)
            for edge_id, cost_delta_m, _learned_risk, confidence in rows
        }
        self.latest_priors = priors
        self.latest_priors_stamp_ns = stamp_ns
        self.latest_prior_model_id = str(message.model_id)
        self._prepare_route(priors)

    def _check_prior_timeout(self) -> None:
        if (
            self.pending_goal is not None
            and self.pending_deadline_ns is not None
            and int(self._now().nanoseconds) >= self.pending_deadline_ns
        ):
            self._clear_pending_prior_request()
            self.latest_priors = {}
            self.latest_priors_stamp_ns = None
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
        self.node.get_logger().info(
            f"preparing route request {self.request_id}: "
            f"support {start_node}->{goal_node}"
        )
        request = self.DynamicEdges.Request()
        from bio_nav_interfaces.msg import RouteEdgeCost

        cost_message = self.RouteEdgeCostArray()
        cost_message.header.stamp = self._now().to_msg()
        cost_message.header.frame_id = self.frame_id
        cost_message.request_id = self.request_id
        cost_message.graph_id = self.graph.graph_id
        cost_message.graph_revision = self.graph.revision
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
            breakdown = edge_cost_breakdown(
                edge,
                self.defaults["route_cost"],
                prior_cost_delta_m=prior[0],
                prior_confidence=prior[1],
                runtime_penalty_m=runtime_penalty,
                blocked=blocked,
            )
            diagnostic = RouteEdgeCost()
            diagnostic.edge_id = int(edge.id)
            diagnostic.structural_cost_m = breakdown.structural_cost_m
            diagnostic.requested_module2_delta_m = (
                breakdown.requested_module2_delta_m
            )
            diagnostic.applied_module2_delta_m = breakdown.applied_module2_delta_m
            diagnostic.runtime_penalty_m = breakdown.runtime_penalty_m
            diagnostic.final_cost_m = breakdown.final_cost_m
            diagnostic.blocked = breakdown.blocked
            cost_message.costs.append(diagnostic)
            extra = (
                0.0
                if breakdown.blocked
                else max(0.0, breakdown.final_cost_m - edge.length_m)
            )
            for support_id in support_ids:
                adjustment = __import__("nav2_msgs.msg", fromlist=["EdgeCost"]).EdgeCost()
                adjustment.edgeid = int(support_id)
                adjustment.cost = float(extra / max(1, len(support_ids)))
                request.adjust_edges.append(adjustment)
        self.route_edge_cost_pub.publish(cost_message)
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
        self.node.get_logger().info(
            f"edge update accepted for route request {self.request_id}: "
            f"support {start_node}->{goal_node}"
        )
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
        returned_edges = wrapped.result.route.edges
        if returned_edges:
            first = returned_edges[0]
            last = returned_edges[-1]
            self.node.get_logger().info(
                "Route Server ordered support edges: "
                f"first={int(first.edgeid)} "
                f"({first.start.x:.3f},{first.start.y:.3f})->"
                f"({first.end.x:.3f},{first.end.y:.3f}), "
                f"last={int(last.edgeid)} "
                f"({last.start.x:.3f},{last.start.y:.3f})->"
                f"({last.end.x:.3f},{last.end.y:.3f})"
            )
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
        self.node.get_logger().info(
            f"canonical route ready for request {self.request_id}: "
            f"{len(canonical_ids)} edges, cost {message.total_cost_m:.3f} m"
        )
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
        # Bind a later action success to whether the last goal update carried
        # the user's exact final pose, not merely a nearby route lookahead.
        self.navigation_goal_targets_final = bool(progress.use_final_goal)
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
        if navigation_result_succeeded(wrapped):
            current = self._current_xy()
            completion_confirmed = False
            final_xy = None
            if current is not None and self.pending_goal is not None:
                final_xy = (
                    float(self.pending_goal.pose.position.x),
                    float(self.pending_goal.pose.position.y),
                )
                completion_confirmed = (
                    bool(getattr(self, "navigation_goal_targets_final", False))
                    and math.dist(current, final_xy)
                    <= self.route_goal_completion_tolerance_m
                )
            if not completion_confirmed:
                # A route lookahead is intentionally a short moving Nav2
                # goal. Reaching it advances the same leg; it is not a
                # waypoint completion. Retire only this child action so the
                # progress timer dispatches the next steer target. Missing
                # map pose also stays fail-closed and cannot complete a leg.
                self.navigation_goal_pending = False
                self.navigation_goal_handle = None
                self.navigation_failed = False
                suffix = (
                    "map pose unavailable"
                    if final_xy is None
                    else f"final goal is ({final_xy[0]:.3f}, {final_xy[1]:.3f})"
                )
                self.node.get_logger().info(
                    f"intermediate route lookahead reached; continuing because {suffix}"
                )
                return
        # Retire the completed leg before publishing its terminal event.  The
        # qualification runner can dispatch the next whole-house waypoint as
        # soon as it receives this Bool.  Clearing via a subscription to our
        # own publication races that new goal and can erase it before
        # ComputeRoute is requested.
        self._finish_active_route()
        if navigation_result_succeeded(wrapped):
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
        now_ns = int(self._now().nanoseconds)
        if self.latest_priors_stamp_ns is not None:
            age_s = (now_ns - self.latest_priors_stamp_ns) / 1.0e9
            if age_s < 0.0 or age_s > self.module2_prior_ttl_s:
                self.latest_priors = {}
                self.latest_priors_stamp_ns = None
                self.node.get_logger().warning(
                    "Module2 edge prior expired; restoring geometry-only route"
                )
                if self.pending_goal is not None:
                    self._prepare_route({})
        # Refresh Module2's learned risk field while the mission is active.
        # The request identity is unchanged, so a new healthy prior may update
        # Route Server costs without fabricating a new navigation request.
        refresh_period_ns = int(
            float(
                self.defaults["module2_edge_prior"].get(
                    "active_refresh_period_s", 5.0
                )
            )
            * 1.0e9
        )
        if (
            self.module2_enabled
            and self.pending_goal is not None
            and now_ns - self.last_context_publish_ns
            >= refresh_period_ns
        ):
            self._arm_prior_request(now_ns)
            self._publish_route_context()
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
        self._clear_pending_prior_request()
        self.latest_priors = {}
        self.latest_priors_stamp_ns = None
        self.latest_prior_model_id = None
        self.tracker = None
        self.navigation_goal_pending = False
        self.navigation_goal_handle = None
        self.navigation_goal_targets_final = False
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
            if self.feasible_only_largest_component:
                retain_largest_feasible_component(candidate)
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
        self.cognitive_constraints_cache.invalidate()
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
        self._publish_cognitive_constraints()
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
