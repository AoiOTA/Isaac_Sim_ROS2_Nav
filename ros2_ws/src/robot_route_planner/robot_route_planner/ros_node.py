"""A21 Module3 coordinator for graph, official route search, and guidance."""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
import json
import math
import tempfile
import threading
import time

import numpy as np

from .defaults import load_engineering_defaults
from .cognitive_constraints import (
    CognitiveConstraintsCache,
    build_cognitive_constraints,
    occupancy_grid_version,
)
from .cognitive_graph_adapter import (
    CognitiveGraphFeedback,
    CognitiveGraphIdentity,
    build_hybrid_graph,
    cognitive_graph_candidate_is_mature,
    cognitive_graph_feedback,
    validate_cognitive_graph_candidate,
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
class ResetStopGateStatus:
    """One validated generation-fenced ResetStopGate state snapshot."""

    generation: int
    held: bool
    eligible_generation: int | None
    reason: str


def parse_reset_stop_gate_status(payload: str) -> ResetStopGateStatus:
    """Strictly decode the transient-local ResetStopGate status contract."""

    try:
        document = json.loads(payload)
    except (TypeError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid reset stop gate JSON: {error}") from error
    if not isinstance(document, dict):
        raise ValueError("reset stop gate status must be a JSON object")
    if set(document) != {
        "generation", "held", "eligible_generation", "reason"
    }:
        raise ValueError("reset stop gate status fields do not match contract")
    generation = document.get("generation")
    held = document.get("held")
    eligible = document.get("eligible_generation")
    reason = document.get("reason")
    if (
        isinstance(generation, bool)
        or not isinstance(generation, int)
        or generation < 0
        or not isinstance(held, bool)
        or not isinstance(reason, str)
        or not reason
        or (
            eligible is not None
            and (
                isinstance(eligible, bool)
                or not isinstance(eligible, int)
                or eligible != generation
            )
        )
    ):
        raise ValueError("invalid reset stop gate status fields")
    valid_state = (
        (reason == "hold" and held and eligible is None)
        or (reason == "reset_complete" and held and eligible == generation)
        or (reason in {"initialized", "closed"} and held and eligible is None)
        or (
            reason.startswith("released:")
            and len(reason) > len("released:")
            and not held
            and eligible is None
        )
    )
    if not valid_state:
        raise ValueError("incoherent reset stop gate status state")
    return ResetStopGateStatus(generation, held, eligible, reason)


@dataclass(frozen=True)
class CostmapSnapshot:
    """Immutable geometry and values from one live Nav2 global costmap."""

    values: np.ndarray
    resolution_m: float
    origin_xy: tuple[float, float]
    frame_id: str


@dataclass(frozen=True)
class RouteCallbackGeneration:
    request_id: int
    graph_generation: int
    graph_id: str
    graph_revision: int


@dataclass(frozen=True)
class CognitiveValidationGeneration:
    request_id: int
    graph_generation: int
    reset_generation: int
    reset_epoch: int


@dataclass(frozen=True)
class GraphSwitchGeneration:
    switch_generation: int
    route_request_id: int | None
    base_graph_generation: int
    reset_generation: int = 0
    desired_generation: int = 0
    requested_graph_id: str = ""
    requested_graph_revision: int = 0


@dataclass(frozen=True)
class StructuralRebuildGeneration:
    request_id: int
    reset_generation: int
    structural_generation: int
    desired_generation: int
    requested_graph_id: str
    requested_graph_revision: int
    base_graph_generation: int
    candidate_generation: int
    candidate_identity: int


@dataclass(frozen=True)
class StructuralRebuildIntent:
    """Latest persistent-map candidate fenced to one idle route epoch."""

    candidate_generation: int
    candidate_identity: int
    request_id: int
    base_graph_generation: int
    reset_generation: int


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
            CognitiveEdgeOutcome,
            CognitiveGraphValidationAck,
            CognitiveMapConstraints,
            CognitivePlaceGraphCandidate,
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
        from rclpy.clock import Clock, ClockType
        from rclpy.duration import Duration
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rclpy.time import Time
        from std_msgs.msg import Bool, Empty, String
        from tf2_ros import Buffer, TransformListener

        self.node = node
        # MultiThreadedExecutor callbacks and ROS Future done callbacks may run
        # concurrently.  Keep route/graph authority changes atomic, but never
        # hold this lock while calling a service/action or publishing/cancelling.
        self._state_lock = threading.RLock()
        for name, default in (
            ("engineering_defaults_file", ""),
            ("map_yaml", ""),
            ("frame_id", "map"),
            ("base_frame_id", "base_link"),
            ("module2_enabled", True),
            ("module2_response_timeout_s", 0.0),
            ("module2_prior_ttl_s", 2.0),
            ("cognitive_goal_prior_wait_s", 4.0),
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
            ("cognitive_graph_mode", "gvg"),
            ("cognitive_graph_topic", "/bio_nav/module2/cognitive_place_graph"),
            ("cognitive_graph_reset_epoch", 0),
            ("cognitive_graph_session_id", ""),
            ("cognitive_graph_tile_id", ""),
            ("cognitive_graph_tile_revision", 0),
            ("cognitive_graph_model_id", ""),
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
        self.cognitive_goal_prior_wait_s = float(
            node.get_parameter("cognitive_goal_prior_wait_s").value
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
        self.cognitive_graph_mode = str(
            node.get_parameter("cognitive_graph_mode").value
        ).strip().lower()
        if self.cognitive_graph_mode not in {"gvg", "shadow", "hybrid", "primary"}:
            raise ValueError(
                "cognitive_graph_mode must be gvg, shadow, hybrid, or primary"
            )
        if self.module2_response_timeout_s < 0.0:
            raise ValueError("module2_response_timeout_s must be non-negative")
        if self.module2_prior_ttl_s <= 0.0:
            raise ValueError("module2_prior_ttl_s must be positive")
        if (
            self.cognitive_graph_mode in {"primary", "hybrid"}
            and not 3.5 <= self.cognitive_goal_prior_wait_s <= 4.0
        ):
            raise ValueError(
                "cognitive_goal_prior_wait_s must be in [3.5, 4.0] "
                "for primary/hybrid"
            )
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
        self.gvg_graph = self.graph
        self.gvg_support = self.support
        self.cognitive_graph_last_sequence = 0
        self.cognitive_graph_switch_pending = False
        self.cognitive_graph_feedback_active: CognitiveGraphFeedback | None = None
        self.cognitive_graph_feedback_pending: CognitiveGraphFeedback | None = None
        self.cognitive_feedback_sequences: dict[tuple[int, str, str], int] = {}
        self.cognitive_validation_terminal: set[tuple[int, str]] = set()
        self.cognitive_outcome_terminal: set[tuple[int, str]] = set()
        self.pending_reroute_outcome: tuple[CognitiveGraphFeedback, str, str] | None = None
        self.cognitive_reroute_revision = 0
        self.graph_generation = 0
        self.graph_switch_generation = 0
        self.reset_generation = 0
        # Navigation starts fail closed until the transient-local gate status
        # establishes a released baseline.  A higher-generation HOLD retires
        # route intent before the later Empty reset event can be delivered.
        self.reset_status_generation: int | None = None
        self.reset_status_snapshot: ResetStopGateStatus | None = None
        self.reset_intent_generation: int | None = None
        self.reset_event_completed_generation: int | None = None
        self.reset_release_seen_generation: int | None = None
        self.reset_hold_barrier = True
        self.structural_generation = 0
        self.desired_graph_generation = 0
        self.desired_graph = self.gvg_graph
        self.desired_support = self.gvg_support
        self.graph_coherent = True
        self.graph_reassert_required = False
        self.graph_transaction_generation: GraphSwitchGeneration | None = None
        self.graph_transaction_future = None
        self.graph_transaction_deadline_steady_s: float | None = None
        self.graph_transaction_kind: str | None = None
        self.graph_transaction_switch_context = None
        self.graph_retry_key = None
        self.graph_retry_attempt = 0
        self.graph_retry_due_steady_s: float | None = None
        self.graph_retry_reason = ""
        self.graph_retry_kind = "switch"
        self.graph_retry_switch_context = None
        self.primary_fallback_used = False
        self.cognitive_graph_identity = CognitiveGraphIdentity(
            int(node.get_parameter("cognitive_graph_reset_epoch").value),
            str(node.get_parameter("cognitive_graph_session_id").value),
            self.map.map_version,
            str(node.get_parameter("cognitive_graph_tile_id").value),
            int(node.get_parameter("cognitive_graph_tile_revision").value),
            self.gvg_graph.graph_id,
            self.gvg_graph.revision,
            str(node.get_parameter("cognitive_graph_model_id").value),
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
        self.structural_candidate_generation = 0
        self.pending_structural_intent: StructuralRebuildIntent | None = None
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
        self.latest_priors_request_id: int | None = None
        self.latest_priors_graph_id: str | None = None
        self.latest_priors_graph_revision: int | None = None
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
        self.CognitiveGraphValidationAck = CognitiveGraphValidationAck
        self.CognitiveEdgeOutcome = CognitiveEdgeOutcome
        self.CognitivePlaceGraphCandidate = CognitivePlaceGraphCandidate
        self.CognitiveTransition = CognitiveTransition
        self.RouteEdgeCostArray = RouteEdgeCostArray
        self.Bool = Bool
        self.String = String
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
        self.goal_result_pub = node.create_publisher(
            String, "/bio_nav/route_goal_result", qos
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
        self.cognitive_graph_validation_pub = node.create_publisher(
            CognitiveGraphValidationAck,
            "/bio_nav/module3/cognitive_graph_validation_ack",
            qos,
        )
        self.cognitive_edge_outcome_pub = node.create_publisher(
            CognitiveEdgeOutcome,
            "/bio_nav/module3/cognitive_edge_outcome",
            qos,
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
        node.create_subscription(
            Empty,
            "/simulation/reset_event",
            self._on_reset_event,
            qos,
        )
        node.create_subscription(
            String,
            "/simulation/reset_stop_gate/status",
            self._on_reset_stop_gate_status,
            qos_latched,
        )
        if self.cognitive_graph_mode != "gvg":
            node.create_subscription(
                CognitivePlaceGraphCandidate,
                str(node.get_parameter("cognitive_graph_topic").value),
                self._on_cognitive_graph,
                qos_latched,
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
        self._graph_reconciliation_clock = Clock(clock_type=ClockType.STEADY_TIME)
        node.create_timer(
            0.1,
            self._graph_reconciliation_tick,
            clock=self._graph_reconciliation_clock,
        )
        self._publish_graph()
        self._publish_structural_status(StructuralGraphStatus.READY, "initial graph ready")

    def _now(self):
        return self.node.get_clock().now()

    def _steady_now(self) -> float:
        return time.monotonic()

    def _route_state_lock(self):
        """Return the shared state lock, including for lightweight unit fixtures."""

        lock = getattr(self, "_state_lock", None)
        if lock is None:
            lock = threading.RLock()
            self._state_lock = lock
        return lock

    def _route_output_lock(self):
        """Serialize route terminals with reset without holding state locks."""

        lock = getattr(self, "_terminal_lock", None)
        if lock is None:
            lock = threading.RLock()
            self._terminal_lock = lock
        return lock

    def _reset_barrier_is_held(self) -> bool:
        """Return whether route intent is fenced by ResetStopGate authority."""

        return bool(getattr(self, "reset_hold_barrier", False))

    @staticmethod
    def _graph_identity(graph) -> tuple[str, int]:
        return str(graph.graph_id), int(graph.revision)

    def _set_desired_graph_locked(
        self, graph, support, *, require_reassert: bool = False
    ) -> None:
        requested = self._graph_identity(graph)
        current = getattr(self, "desired_graph", None)
        changed = current is None or self._graph_identity(current) != requested
        pending = getattr(self, "graph_transaction_generation", None)
        if changed:
            self.desired_graph_generation = int(
                getattr(self, "desired_graph_generation", 0)
            ) + 1
            self.graph_reassert_required = False
            self.graph_retry_key = None
            self.graph_retry_attempt = 0
            self.graph_retry_due_steady_s = None
            self.graph_retry_switch_context = None
        if require_reassert:
            self.graph_reassert_required = True
        if changed and pending is not None:
            self.graph_reassert_required = True
        self.desired_graph = graph
        self.desired_support = support
        self.graph_coherent = (
            self._graph_identity(self.graph) == requested
            and pending is None
            and not bool(getattr(self, "graph_reassert_required", False))
        )

    def _desired_graph_is_coherent_locked(self) -> bool:
        desired = getattr(self, "desired_graph", self.graph)
        return bool(getattr(self, "graph_coherent", True)) and (
            self._graph_identity(self.graph) == self._graph_identity(desired)
        ) and not bool(getattr(self, "graph_reassert_required", False))

    def _graph_retry_key_locked(self):
        graph = getattr(self, "desired_graph", None)
        if graph is None:
            graph = getattr(self, "gvg_graph", self.graph)
        route_request_id = (
            int(getattr(self, "request_id", 0))
            if getattr(self, "pending_goal", None) is not None
            else None
        )
        return (
            int(getattr(self, "reset_generation", 0)),
            int(getattr(self, "desired_graph_generation", 0)),
            route_request_id,
            *self._graph_identity(graph),
        )

    def _clear_graph_retry_locked(self) -> None:
        self.graph_retry_key = None
        self.graph_retry_attempt = 0
        self.graph_retry_due_steady_s = None
        self.graph_retry_reason = ""
        self.graph_retry_kind = "switch"
        self.graph_retry_switch_context = None

    def _refresh_structural_intent_locked(
        self,
    ) -> StructuralRebuildIntent | None:
        candidate = getattr(self, "pending_structural_map", None)
        if candidate is None:
            self.pending_structural_intent = None
            return None
        previous = getattr(self, "pending_structural_intent", None)
        candidate_generation = int(
            getattr(self, "structural_candidate_generation", 0)
        )
        if previous is None or previous.candidate_identity != id(candidate):
            candidate_generation += 1
            self.structural_candidate_generation = candidate_generation
        intent = StructuralRebuildIntent(
            candidate_generation,
            id(candidate),
            int(getattr(self, "request_id", 0)),
            int(getattr(self, "graph_generation", 0)),
            int(getattr(self, "reset_generation", 0)),
        )
        self.pending_structural_intent = intent
        return intent

    def _structural_intent_is_current_locked(
        self, intent: StructuralRebuildIntent
    ) -> bool:
        candidate = getattr(self, "pending_structural_map", None)
        return bool(
            candidate is not None
            and getattr(self, "pending_structural_intent", None) == intent
            and id(candidate) == intent.candidate_identity
            and int(getattr(self, "request_id", 0)) == intent.request_id
            and int(getattr(self, "graph_generation", 0))
            == intent.base_graph_generation
            and int(getattr(self, "reset_generation", 0))
            == intent.reset_generation
        )

    def _try_deferred_structural_rebuild(self) -> None:
        """Submit the latest candidate once graph authority and route are idle."""

        with self._route_state_lock():
            if (
                getattr(self, "pending_structural_map", None) is None
                or bool(getattr(self, "route_active", False))
                or getattr(self, "pending_goal", None) is not None
                or getattr(self, "graph_transaction_generation", None) is not None
                or getattr(self, "graph_retry_due_steady_s", None) is not None
            ):
                return
            self._refresh_structural_intent_locked()
        self._rebuild_structural_graph()

    def _schedule_graph_retry_locked(
        self,
        reason: str,
        *,
        kind: str = "switch",
        immediate: bool = False,
        now_steady_s: float | None = None,
        switch_context=None,
    ) -> None:
        now_s = self._steady_now() if now_steady_s is None else float(now_steady_s)
        key = self._graph_retry_key_locked()
        if getattr(self, "graph_retry_key", None) != key:
            attempt = 0
        else:
            attempt = int(getattr(self, "graph_retry_attempt", 0)) + (
                0 if immediate else 1
            )
        delay_s = 0.0 if immediate else min(0.25 * (2 ** attempt), 2.0)
        self.graph_retry_key = key
        self.graph_retry_attempt = attempt
        self.graph_retry_due_steady_s = now_s + delay_s
        self.graph_retry_reason = str(reason)
        self.graph_retry_kind = str(kind)
        self.graph_retry_switch_context = switch_context
        self.graph_reassert_required = True
        self.graph_coherent = False

    def _register_graph_transaction_future(
        self, generation: GraphSwitchGeneration, future, kind: str
    ) -> None:
        with self._route_state_lock():
            if getattr(self, "graph_transaction_generation", None) == generation:
                self.graph_transaction_future = future
                self.graph_transaction_deadline_steady_s = self._steady_now() + 2.0
                self.graph_transaction_kind = str(kind)

    def _clear_graph_transaction_future_locked(
        self, generation: GraphSwitchGeneration | None, future
    ) -> None:
        if (
            generation is None
            or (
                getattr(self, "graph_transaction_generation", None) == generation
                and getattr(self, "graph_transaction_future", None) is future
            )
        ):
            self.graph_transaction_future = None
            self.graph_transaction_deadline_steady_s = None
            self.graph_transaction_kind = None

    def _graph_reconciliation_tick(self) -> None:
        """Retry Route Server authority from a steady clock without request storms."""

        now_s = self._steady_now()
        graph = None
        reason = ""
        fallback = False
        retry_kind = "switch"
        switch_context = None
        deferred_structural = False
        with self._route_state_lock():
            transaction = getattr(self, "graph_transaction_generation", None)
            deadline = getattr(
                self, "graph_transaction_deadline_steady_s", None
            )
            if transaction is not None and deadline is not None and now_s >= deadline:
                timed_kind = getattr(self, "graph_transaction_kind", None) or "switch"
                timed_context = getattr(
                    self, "graph_transaction_switch_context", None
                )
                self.graph_transaction_generation = None
                self.cognitive_graph_switch_pending = False
                self.cognitive_graph_feedback_pending = None
                self.graph_transaction_future = None
                self.graph_transaction_deadline_steady_s = None
                self.graph_transaction_kind = None
                self.graph_transaction_switch_context = None
                retry_already_targets_desired = (
                    getattr(self, "graph_retry_key", None)
                    == self._graph_retry_key_locked()
                    and getattr(self, "graph_retry_due_steady_s", None) is not None
                )
                if not retry_already_targets_desired:
                    self._schedule_graph_retry_locked(
                        "SetRouteGraph request timed out",
                        kind=timed_kind,
                        now_steady_s=now_s,
                        switch_context=timed_context,
                    )
                transaction = None
            if transaction is not None:
                return
            due_s = getattr(self, "graph_retry_due_steady_s", None)
            if due_s is None:
                deferred_structural = bool(
                    getattr(self, "pending_structural_map", None) is not None
                    and not bool(getattr(self, "route_active", False))
                    and getattr(self, "pending_goal", None) is None
                )
            elif now_s < float(due_s):
                return
            if due_s is None:
                pass
            elif getattr(self, "graph_retry_key", None) != self._graph_retry_key_locked():
                self._schedule_graph_retry_locked(
                    "desired graph generation changed",
                    immediate=True,
                    now_steady_s=now_s,
                )
            if due_s is not None:
                retry_kind = str(getattr(self, "graph_retry_kind", "switch"))
                reason = str(
                    getattr(self, "graph_retry_reason", "graph reconciliation")
                )
                switch_context = getattr(self, "graph_retry_switch_context", None)
                self.graph_retry_due_steady_s = None
                if retry_kind != "structural":
                    graph = getattr(self, "desired_graph", self.gvg_graph)
                    fallback = self._graph_identity(graph) == self._graph_identity(
                        self.gvg_graph
                    )
        if deferred_structural or retry_kind == "structural":
            self._rebuild_structural_graph()
        elif graph is not None:
            if switch_context is None:
                self._request_graph_switch(graph, reason, fallback=fallback)
            else:
                saved_detail, saved_fallback, feedback, candidate, validation = (
                    switch_context
                )
                self._request_graph_switch(
                    graph,
                    saved_detail,
                    fallback=saved_fallback,
                    feedback=feedback,
                    candidate=candidate,
                    expected_validation=validation,
                )

    @staticmethod
    def _graph_transaction_paths(
        kind: str, generation: GraphSwitchGeneration
    ) -> tuple[Path, Path]:
        """Allocate immutable paths for exactly one Route Server transaction."""

        root = Path(tempfile.gettempdir()) / "bio_nav_v6_route_graph_transactions"
        root.mkdir(parents=True, exist_ok=True)
        prefix = (
            f"reset_{generation.reset_generation}_"
            f"desired_{generation.desired_generation}_"
            f"switch_{generation.switch_generation}_{kind}_"
        )
        directory = Path(tempfile.mkdtemp(prefix=prefix, dir=str(root)))
        return directory / "route_graph.geojson", directory / "support_map.json"

    def _feedback_sequence(
        self, feedback: CognitiveGraphFeedback, kind: str, candidate_edge_id: str
    ) -> int:
        key = (feedback.generation, kind, candidate_edge_id)
        sequences = getattr(self, "cognitive_feedback_sequences", {})
        value = int(sequences.get(key, 0)) + 1
        sequences[key] = value
        self.cognitive_feedback_sequences = sequences
        return value

    def _populate_graph_feedback(
        self, message, feedback: CognitiveGraphFeedback,
        candidate_edge_id: str, validated_edge_id: str,
    ) -> None:
        message.header.stamp = self._now().to_msg()
        message.header.frame_id = self.frame_id
        message.recurrent_session_id = feedback.recurrent_session_id
        message.reset_epoch = feedback.reset_epoch
        message.generation = feedback.generation
        message.candidate_graph_id = feedback.candidate_graph_id
        message.candidate_topology_revision = feedback.candidate_topology_revision
        message.candidate_value_sequence = feedback.candidate_value_sequence
        message.candidate_edge_id = candidate_edge_id
        message.validated_graph_id = feedback.validated_graph_id
        message.validated_graph_revision = feedback.validated_graph_revision
        message.validated_edge_id = validated_edge_id

    def _publish_graph_validation(
        self, feedback: CognitiveGraphFeedback, *, accepted: bool, reason: str
    ) -> None:
        terminal = getattr(self, "cognitive_validation_terminal", set())
        for candidate_edge_id in feedback.candidate_edges():
            key = (feedback.generation, candidate_edge_id)
            if key in terminal:
                continue
            message = self.CognitiveGraphValidationAck()
            validated_edge_id = (
                feedback.first_validated(candidate_edge_id) if accepted else ''
            )
            self._populate_graph_feedback(
                message, feedback, candidate_edge_id, validated_edge_id
            )
            message.event_sequence = self._feedback_sequence(
                feedback, "validation", candidate_edge_id
            )
            message.accepted = bool(accepted)
            message.reason = str(reason)
            message.reroute_revision = 0
            message.reroute_applied = False
            self.cognitive_graph_validation_pub.publish(message)
            terminal.add(key)
        self.cognitive_validation_terminal = terminal

    def _publish_edge_outcome(
        self, feedback: CognitiveGraphFeedback, validated_edge_id: str,
        candidate_edge_id: str, *, success: bool, reason: str,
        reroute_applied: bool = False,
    ) -> None:
        terminal = getattr(self, "cognitive_outcome_terminal", set())
        key = (feedback.generation, str(validated_edge_id))
        if key in terminal and not reroute_applied:
            return
        message = self.CognitiveEdgeOutcome()
        self._populate_graph_feedback(
            message, feedback, candidate_edge_id, str(validated_edge_id)
        )
        message.event_sequence = self._feedback_sequence(
            feedback, "outcome", candidate_edge_id
        )
        message.success = bool(success)
        message.failure = not bool(success)
        message.reason = str(reason)
        message.reroute_revision = (
            int(getattr(self, "cognitive_reroute_revision", 0))
            if reroute_applied else 0
        )
        message.reroute_applied = bool(reroute_applied)
        self.cognitive_edge_outcome_pub.publish(message)
        terminal.add(key)
        self.cognitive_outcome_terminal = terminal

    def _cognitive_route_edge(
        self, edge_index: int | None = None
    ) -> tuple[CognitiveGraphFeedback, str, str] | None:
        feedback = getattr(self, "cognitive_graph_feedback_active", None)
        tracker = getattr(self, "tracker", None)
        if feedback is None or tracker is None:
            return None
        if (
            str(self.graph.graph_id) != feedback.validated_graph_id
            or int(self.graph.revision) != feedback.validated_graph_revision
        ):
            return None
        index = tracker.edge_index if edge_index is None else int(edge_index)
        if index < 0 or index >= len(tracker.edges):
            return None
        validated_edge_id = str(tracker.edges[index].id)
        candidate_edge_id = feedback.candidate_for_validated(validated_edge_id)
        if candidate_edge_id is None:
            return None
        return feedback, validated_edge_id, candidate_edge_id

    def _publish_navigation_edge_failure(self, reason: str) -> None:
        edge = self._cognitive_route_edge()
        if edge is None:
            return
        feedback, validated_edge_id, candidate_edge_id = edge
        self._publish_edge_outcome(
            feedback, validated_edge_id, candidate_edge_id,
            success=False, reason=reason,
        )
        if self._primary_fallback_available():
            self.pending_reroute_outcome = edge

    def _publish_crossed_edge_outcomes(
        self, previous_edge_index: int, current_edge_index: int
    ) -> None:
        for edge_index in range(int(previous_edge_index), int(current_edge_index)):
            edge = self._cognitive_route_edge(edge_index)
            if edge is None:
                continue
            feedback, validated_edge_id, candidate_edge_id = edge
            self._publish_edge_outcome(
                feedback, validated_edge_id, candidate_edge_id,
                success=True, reason="route_tracker_edge_crossed",
            )

    def _route_callback_generation(self) -> RouteCallbackGeneration:
        return RouteCallbackGeneration(
            int(self.request_id),
            int(getattr(self, "graph_generation", 0)),
            str(self.graph.graph_id),
            int(self.graph.revision),
        )

    def _route_callback_is_current(
        self, generation: RouteCallbackGeneration | None
    ) -> bool:
        if generation is None:
            return not self._reset_barrier_is_held()
        return (
            not self._reset_barrier_is_held()
            and self.pending_goal is not None
            and generation == self._route_callback_generation()
        )

    def _cognitive_validation_generation_locked(
        self,
    ) -> CognitiveValidationGeneration:
        return CognitiveValidationGeneration(
            int(getattr(self, "request_id", 0)),
            int(getattr(self, "graph_generation", 0)),
            int(getattr(self, "reset_generation", 0)),
            int(self.cognitive_graph_identity.reset_epoch),
        )

    def _cognitive_validation_is_current_locked(
        self, generation: CognitiveValidationGeneration
    ) -> bool:
        return generation == self._cognitive_validation_generation_locked()

    def _graph_switch_callback_is_current(
        self, generation: GraphSwitchGeneration | None
    ) -> bool:
        if generation is None:
            return True
        return (
            generation.switch_generation
            == int(getattr(self, "graph_switch_generation", 0))
            and generation.base_graph_generation
            == int(getattr(self, "graph_generation", 0))
            and generation.route_request_id
            == (
                int(self.request_id)
                if getattr(self, "pending_goal", None) is not None
                else None
            )
        )

    def _graph_switch_request_is_current_locked(
        self, generation: GraphSwitchGeneration, graph
    ) -> bool:
        return (
            generation.reset_generation
            == int(getattr(self, "reset_generation", 0))
            and generation.desired_generation
            == int(getattr(self, "desired_graph_generation", 0))
            and generation.base_graph_generation
            == int(getattr(self, "graph_generation", 0))
            and self._graph_identity(getattr(self, "desired_graph", graph))
            == self._graph_identity(graph)
            and generation.route_request_id
            == (
                int(getattr(self, "request_id", 0))
                if getattr(self, "pending_goal", None) is not None
                else None
            )
        )

    def _primary_fallback_available(self) -> bool:
        return (
            getattr(self, "cognitive_graph_mode", "gvg") == "primary"
            and not getattr(self, "primary_fallback_used", False)
        )

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

    def _retire_route_state(self):
        """Clear coordinator-owned route intent without rebuilding the graph."""

        handle = getattr(self, "navigation_goal_handle", None)
        self.route_active = False
        self.pending_goal = None
        self._clear_pending_prior_request()
        self._clear_latest_priors()
        self.tracker = None
        self.navigation_goal_pending = False
        self.navigation_goal_handle = None
        self.navigation_goal_targets_final = False
        self.navigation_failed = False
        self.pending_reroute_outcome = None
        return handle

    def _cancel_navigation_handle(self, handle) -> None:
        if handle is None:
            return
        try:
            handle.cancel_goal_async()
        except Exception as error:
            self.node.get_logger().warning(
                f"NavigateToPose cancellation failed: {error}"
            )

    def _retire_active_route_for_reset(self):
        """Fence and synchronously retire all state owned by the old epoch."""

        was_active = bool(getattr(self, "route_active", False))
        old_request_id = int(getattr(self, "request_id", 0))
        old_goal = getattr(self, "pending_goal", None)
        old_handle = getattr(self, "navigation_goal_handle", None)

        # Advance every asynchronous route/graph fence before clearing state.
        self.request_id = old_request_id + 1
        self.graph_generation = int(getattr(self, "graph_generation", 0)) + 1
        self.graph_switch_generation = int(
            getattr(self, "graph_switch_generation", 0)
        ) + 1
        self._retire_route_state()

        runtime = getattr(self, "runtime", None)
        runtime_edges = None if runtime is None else getattr(runtime, "edges", None)
        if runtime_edges is not None:
            runtime_edges.clear()
        self.latest_pose_xy = None
        self.latest_pose_frame_id = None
        self.latest_pose_stamp_ns = None
        self.latest_global_costmap = None
        self.last_context_publish_ns = 0
        self.pending_structural_map = None
        self.pending_structural_intent = None
        self.structural_candidate_generation = int(
            getattr(self, "structural_candidate_generation", 0)
        ) + 1
        structural_monitor = getattr(self, "structural_monitor", None)
        if structural_monitor is not None:
            structural_monitor.last_candidate = None
            structural_monitor.first_stable_s = None
            structural_monitor.stable_count = 0
        cache = getattr(self, "cognitive_constraints_cache", None)
        if cache is not None:
            cache.invalidate()
        region_selector = getattr(self, "region_selector", None)
        if region_selector is not None:
            region_selector.current = None
            region_selector.last_switch_s = -math.inf
        tf_buffer = getattr(self, "tf_buffer", None)
        clear_tf = None if tf_buffer is None else getattr(tf_buffer, "clear", None)
        if clear_tf is not None:
            clear_tf()
        return was_active, old_request_id, old_goal, old_handle

    def _publish_route_terminal_pair(
        self,
        *,
        success: bool,
        request_id: int,
        status: str,
        reason: str,
        reset_epoch: int,
    ) -> None:
        bool_type = getattr(self, "Bool", None)
        string_type = getattr(self, "String", None)
        if bool_type is None or string_type is None:
            try:
                messages = __import__("std_msgs.msg", fromlist=["Bool", "String"])
                bool_type = messages.Bool
                string_type = messages.String
            except ModuleNotFoundError:
                class _Message:
                    data = None

                bool_type = string_type = _Message
        terminal = bool_type()
        terminal.data = bool(success)
        self.goal_complete_pub.publish(terminal)
        result = string_type()
        result.data = json.dumps(
            {
                "request_id": int(request_id),
                "status": str(status),
                "reason": str(reason),
                "reset_epoch": int(reset_epoch),
            },
            separators=(",", ":"),
            sort_keys=True,
        )
        result_pub = getattr(self, "goal_result_pub", None)
        if result_pub is not None:
            result_pub.publish(result)

    def _begin_simulation_reset_locked(self):
        """Retire old intent and advance the reset epoch exactly once."""

        identity = self.cognitive_graph_identity
        was_active, old_request_id, _old_goal, old_handle = (
            self._retire_active_route_for_reset()
        )
        self.reset_generation = int(getattr(self, "reset_generation", 0)) + 1
        self.structural_generation = int(
            getattr(self, "structural_generation", 0)
        ) + 1
        self.cognitive_graph_identity = CognitiveGraphIdentity(
            identity.reset_epoch + 1,
            '',
            identity.map_version,
            '',
            0,
            self.gvg_graph.graph_id,
            self.gvg_graph.revision,
            '',
        )
        self.cognitive_graph_last_sequence = 0
        self.cognitive_graph_feedback_active = None
        self.cognitive_graph_feedback_pending = None
        self.cognitive_feedback_sequences = {}
        self.cognitive_validation_terminal = set()
        self.cognitive_outcome_terminal = set()
        self.pending_reroute_outcome = None
        self.cognitive_reroute_revision = 0
        self.primary_fallback_used = False
        self._set_desired_graph_locked(
            self.gvg_graph,
            getattr(self, "gvg_support", getattr(self, "support", None)),
            require_reassert=True,
        )
        # Route Server authority is not coherent until completion dispatches
        # the generation-fenced GVG reassertion.
        self.graph_coherent = False
        return (
            was_active,
            old_request_id,
            old_handle,
            int(self.cognitive_graph_identity.reset_epoch),
        )

    def _publish_reset_completion(
        self, expected_generation: int | None
    ) -> None:
        """Publish fresh-epoch empty state and start GVG reconciliation."""

        with self._route_output_lock():
            if expected_generation is not None:
                with self._route_state_lock():
                    if (
                        getattr(self, "reset_intent_generation", None)
                        != expected_generation
                        or getattr(
                            self, "reset_event_completed_generation", None
                        )
                        != expected_generation
                    ):
                        return
            try:
                self._publish_runtime_states(graph=self.gvg_graph)
            except Exception as error:
                node = getattr(self, "node", None)
                if node is not None:
                    node.get_logger().warning(
                        f"reset runtime-edge empty snapshot failed: {error}"
                    )
                if (
                    hasattr(self, "StructuralGraphStatus")
                    and hasattr(self, "status_pub")
                ):
                    self._publish_structural_status(
                        self.StructuralGraphStatus.LAST_KNOWN_GOOD,
                        f"reset runtime-edge empty snapshot failed: {error}",
                    )
            self._ensure_desired_graph(
                "simulation reset requires Route Server GVG"
            )

    def _fail_closed_reset_status(self, detail: str) -> None:
        with self._route_state_lock():
            self.reset_hold_barrier = True
        self.node.get_logger().error(
            f"reset stop gate status rejected; route goals held: {detail}"
        )

    def _on_reset_stop_gate_status(self, message) -> None:
        """Retire active route intent at HOLD, before reset completion arrives."""

        try:
            status = parse_reset_stop_gate_status(str(message.data))
        except (AttributeError, ValueError) as error:
            self._fail_closed_reset_status(str(error))
            return

        old_handle = None
        terminal = None
        failure = None
        with self._route_output_lock():
            with self._route_state_lock():
                seen = getattr(self, "reset_status_generation", None)
                previous = getattr(self, "reset_status_snapshot", None)
                if seen is not None and status.generation < int(seen):
                    self.reset_hold_barrier = True
                    failure = (
                        "backward generation "
                        f"{status.generation} < {int(seen)}"
                    )
                elif seen is None and status.reason.startswith("released:"):
                    # A transient-local released snapshot is the normal startup
                    # baseline.  It carries no reset terminal of its own.
                    self.reset_status_generation = status.generation
                    self.reset_status_snapshot = status
                    self.reset_hold_barrier = False
                elif seen is None and status.reason in {"initialized", "closed"}:
                    self.reset_status_generation = status.generation
                    self.reset_status_snapshot = status
                    self.reset_hold_barrier = True
                elif seen is None or status.generation > int(seen):
                    if status.reason != "hold":
                        self.reset_status_generation = status.generation
                        self.reset_status_snapshot = status
                        self.reset_hold_barrier = True
                        failure = (
                            "higher generation did not begin with HOLD: "
                            f"generation={status.generation}, reason={status.reason}"
                        )
                    else:
                        self.reset_status_generation = status.generation
                        self.reset_status_snapshot = status
                        self.reset_intent_generation = status.generation
                        self.reset_event_completed_generation = None
                        self.reset_release_seen_generation = None
                        self.reset_hold_barrier = True
                        reset = self._begin_simulation_reset_locked()
                        was_active, old_request_id, old_handle, reset_epoch = reset
                        if was_active:
                            terminal = (old_request_id, reset_epoch)
                elif status == previous:
                    # Reliable transient-local delivery may repeat the same
                    # snapshot.  Exact duplicates are idempotent.
                    pass
                elif status.generation == int(seen):
                    intent = getattr(self, "reset_intent_generation", None)
                    if intent != status.generation:
                        self.reset_hold_barrier = True
                        failure = (
                            "same-generation transition without reset HOLD: "
                            f"generation={status.generation}, reason={status.reason}"
                        )
                    elif status.reason == "reset_complete":
                        self.reset_status_snapshot = status
                        self.reset_hold_barrier = True
                    elif status.reason.startswith("released:"):
                        self.reset_status_snapshot = status
                        self.reset_release_seen_generation = status.generation
                        self.reset_hold_barrier = (
                            getattr(self, "reset_event_completed_generation", None)
                            != status.generation
                        )
                    else:
                        self.reset_hold_barrier = True
                        failure = (
                            "conflicting same-generation reset status: "
                            f"generation={status.generation}, reason={status.reason}"
                        )
            if terminal is not None:
                old_request_id, reset_epoch = terminal
                self._publish_route_terminal_pair(
                    success=False,
                    request_id=old_request_id,
                    status="aborted",
                    reason="simulation_reset",
                    reset_epoch=reset_epoch,
                )
        self._cancel_navigation_handle(old_handle)
        if failure is not None:
            self.node.get_logger().error(
                f"reset stop gate status rejected; route goals held: {failure}"
            )

    def _on_reset_event(self, _message) -> None:
        old_handle = None
        terminal = None
        complete = False
        completion_generation = None
        with self._route_output_lock():
            with self._route_state_lock():
                intent = getattr(self, "reset_intent_generation", None)
                completed = getattr(
                    self, "reset_event_completed_generation", None
                )
                if intent is not None and completed != intent:
                    # HOLD already performed the retirement and epoch bump.
                    self.reset_event_completed_generation = intent
                    self.reset_hold_barrier = (
                        getattr(self, "reset_release_seen_generation", None)
                        != intent
                    )
                    complete = True
                    completion_generation = intent
                elif intent is not None:
                    # The Empty event has no generation field.  Once the
                    # current status-backed intent is complete, any further
                    # event before a higher HOLD is a duplicate even if the
                    # same-generation release has already opened the barrier.
                    return
                else:
                    # Legacy/no-status fallback retains the former event-only
                    # behavior and does not require an unavailable release.
                    reset = self._begin_simulation_reset_locked()
                    was_active, old_request_id, old_handle, reset_epoch = reset
                    self.reset_hold_barrier = False
                    if was_active:
                        terminal = (old_request_id, reset_epoch)
                    complete = True
            if terminal is not None:
                old_request_id, reset_epoch = terminal
                self._publish_route_terminal_pair(
                    success=False,
                    request_id=old_request_id,
                    status="aborted",
                    reason="simulation_reset",
                    reset_epoch=reset_epoch,
                )
        self._cancel_navigation_handle(old_handle)
        if complete:
            self._publish_reset_completion(completion_generation)

    def _on_cognitive_graph(self, message) -> None:
        if self._reset_barrier_is_held():
            return
        if (
            self.cognitive_graph_mode in {"primary", "hybrid"}
            and not cognitive_graph_candidate_is_mature(message)
        ):
            self._publish_structural_status(
                self.StructuralGraphStatus.READY,
                "cognitive_graph_immature_gvg_bootstrap",
            )
            return
        with self._route_state_lock():
            validation_generation = self._cognitive_validation_generation_locked()
            active_graph = self.graph
            feedback = replace(
                cognitive_graph_feedback(message),
                validated_graph_id=str(active_graph.graph_id),
                validated_graph_revision=int(active_graph.revision),
            )
            goal_locked_to_fallback = bool(
                self.pending_goal is not None and self.primary_fallback_used
            )
            switch_pending = bool(self.cognitive_graph_switch_pending)
            identity = self.cognitive_graph_identity
            bind_identity = not identity.recurrent_session_id
            last_source_sequence = int(self.cognitive_graph_last_sequence)
            occupancy = self.map
            gvg_graph = getattr(self, "gvg_graph", active_graph)
        if goal_locked_to_fallback:
            self._publish_structural_status(
                self.StructuralGraphStatus.LAST_KNOWN_GOOD,
                "cognitive graph rejected: current goal is locked to GVG fallback",
            )
            self._publish_graph_validation(
                feedback, accepted=False,
                reason="current_goal_locked_to_gvg_fallback",
            )
            return
        if switch_pending:
            self._publish_structural_status(
                self.StructuralGraphStatus.LAST_KNOWN_GOOD,
                "cognitive graph rejected: SetRouteGraph switch already pending",
            )
            self._publish_graph_validation(
                feedback, accepted=False, reason="set_route_graph_switch_pending"
            )
            return
        if bind_identity:
            identity = CognitiveGraphIdentity(
                identity.reset_epoch,
                str(message.recurrent_session_id),
                identity.map_version,
                str(message.cognitive_tile_id),
                int(message.tile_revision),
                identity.source_physical_graph_id,
                identity.source_physical_graph_revision,
                str(message.model_id),
            )
        try:
            candidate = validate_cognitive_graph_candidate(
                message,
                now_ns=int(self._now().nanoseconds),
                expected=identity,
                last_source_sequence=last_source_sequence,
                occupancy=occupancy,
                footprint=self.defaults["footprint"],
            )
            selected = candidate.graph
            if self.cognitive_graph_mode == "hybrid":
                selected = build_hybrid_graph(
                    gvg_graph,
                    candidate,
                    occupancy=occupancy,
                    footprint=self.defaults["footprint"],
                )
            feedback = cognitive_graph_feedback(message, selected)
        except Exception as error:
            with self._route_state_lock():
                if not self._cognitive_validation_is_current_locked(
                    validation_generation
                ):
                    return
            self._publish_structural_status(
                self.StructuralGraphStatus.LAST_KNOWN_GOOD,
                f"cognitive graph rejected: {error}",
            )
            self._publish_graph_validation(
                feedback, accepted=False,
                reason=f"physical_validation_rejected: {error}",
            )
            if self._primary_fallback_available():
                self._fallback_to_gvg_once(
                    f"candidate rejected: {error}",
                    request_id=validation_generation.request_id,
                    reset_generation=validation_generation.reset_generation,
                )
            return
        with self._route_state_lock():
            if not self._cognitive_validation_is_current_locked(
                validation_generation
            ):
                return
            if self.cognitive_graph_mode == "shadow":
                if bind_identity:
                    self.cognitive_graph_identity = candidate.identity
                self.cognitive_graph_last_sequence = candidate.source_sequence
                feedback = replace(
                    feedback,
                    validated_graph_id=str(self.graph.graph_id),
                    validated_graph_revision=int(self.graph.revision),
                )
        if self.cognitive_graph_mode == "shadow":
            self._publish_graph_validation(
                feedback, accepted=False,
                reason="physically_validated_shadow_not_selected",
            )
            self._publish_structural_status(
                self.StructuralGraphStatus.READY,
                "cognitive graph accepted in shadow; GVG remains selected",
            )
            return
        self._request_graph_switch(
            selected,
            f"cognitive graph selected mode={self.cognitive_graph_mode} "
            f"sequence={candidate.source_sequence} "
            f"tile={candidate.identity.cognitive_tile_id}:"
            f"{candidate.identity.tile_revision}",
            fallback=False,
            feedback=feedback,
            candidate=candidate,
            expected_validation=validation_generation,
        )

    def _request_graph_switch(
        self, graph, detail: str, *, fallback: bool,
        feedback: CognitiveGraphFeedback | None = None, candidate=None,
        expected_validation: CognitiveValidationGeneration | None = None,
    ) -> None:
        switch_context = (
            str(detail), bool(fallback), feedback, candidate, expected_validation
        )
        with self._route_state_lock():
            if (
                expected_validation is not None
                and not self._cognitive_validation_is_current_locked(
                    expected_validation
                )
            ):
                return
            invocation_reset_generation = int(
                getattr(self, "reset_generation", 0)
            )
            invocation_request_id = int(getattr(self, "request_id", 0))
            if (
                expected_validation is not None
                and getattr(self, "graph_transaction_generation", None) is not None
            ):
                return
            self._set_desired_graph_locked(graph, support=None)
            retry_waiting = (
                getattr(self, "graph_retry_key", None)
                == self._graph_retry_key_locked()
                and getattr(self, "graph_retry_due_steady_s", None) is not None
            )
            if retry_waiting:
                return
            if getattr(self, "graph_transaction_generation", None) is not None:
                # Consume and compensate the in-flight request before starting
                # another Route Server transaction.
                self.graph_coherent = False
                return
            self.graph_coherent = False
            self.graph_switch_generation = int(
                getattr(self, "graph_switch_generation", 0)
            ) + 1
            generation = GraphSwitchGeneration(
                self.graph_switch_generation,
                int(self.request_id) if self.pending_goal is not None else None,
                int(getattr(self, "graph_generation", 0)),
                invocation_reset_generation,
                int(getattr(self, "desired_graph_generation", 0)),
                str(graph.graph_id),
                int(graph.revision),
            )
            self.graph_transaction_generation = generation
            self.graph_transaction_future = None
            self.graph_transaction_deadline_steady_s = None
            self.graph_transaction_kind = "switch"
            self.graph_transaction_switch_context = switch_context
            self.cognitive_graph_switch_pending = True
            self.cognitive_graph_feedback_pending = feedback
        try:
            support = export_route_support_graph(
                graph,
                support_spacing_m=float(
                    self.defaults["graph"]["route_support_spacing_m"]
                ),
            )
            suffix = "gvg_fallback" if fallback else "selected"
            geojson, mapping = self._graph_transaction_paths(suffix, generation)
            save_route_support(support, geojson, mapping)
        except Exception as error:
            with self._route_state_lock():
                invalidated = not self._graph_switch_request_is_current_locked(
                    generation, graph
                )
                if self.graph_transaction_generation == generation:
                    self.graph_transaction_generation = None
                    self.graph_transaction_switch_context = None
                    self.cognitive_graph_switch_pending = False
                    self.cognitive_graph_feedback_pending = None
                self._schedule_graph_retry_locked(
                    "graph export invalidated" if invalidated
                    else f"graph export rejected: {error}",
                    switch_context=None if invalidated else switch_context,
                )
            if invalidated:
                return
            self._publish_structural_status(
                self.StructuralGraphStatus.LAST_KNOWN_GOOD,
                f"graph export rejected: {error}",
            )
            if feedback is not None:
                self._publish_graph_validation(
                    replace(
                        feedback,
                        validated_graph_id=str(self.graph.graph_id),
                        validated_graph_revision=int(self.graph.revision),
                    ), accepted=False,
                    reason=f"graph_export_rejected: {error}",
                )
            if not fallback and self._primary_fallback_available():
                self._fallback_to_gvg_once(
                    f"graph export rejected: {error}",
                    request_id=invocation_request_id,
                    reset_generation=invocation_reset_generation,
                )
            return
        if not self.set_graph_client.service_is_ready():
            with self._route_state_lock():
                invalidated = not self._graph_switch_request_is_current_locked(
                    generation, graph
                )
                if self.graph_transaction_generation == generation:
                    self.graph_transaction_generation = None
                    self.graph_transaction_switch_context = None
                    self.cognitive_graph_switch_pending = False
                    self.cognitive_graph_feedback_pending = None
                self._schedule_graph_retry_locked(
                    "graph switch invalidated while service unavailable"
                    if invalidated else "SetRouteGraph unavailable",
                    switch_context=None if invalidated else switch_context,
                )
            if invalidated:
                return
            self._publish_structural_status(
                self.StructuralGraphStatus.LAST_KNOWN_GOOD,
                "cognitive graph fallback: SetRouteGraph unavailable"
                if fallback else "cognitive graph rejected: SetRouteGraph unavailable",
            )
            if feedback is not None:
                self._publish_graph_validation(
                    replace(
                        feedback,
                        validated_graph_id=str(self.graph.graph_id),
                        validated_graph_revision=int(self.graph.revision),
                    ), accepted=False, reason="set_route_graph_unavailable"
                )
            if not fallback and self._primary_fallback_available():
                self._fallback_to_gvg_once(
                    "SetRouteGraph unavailable",
                    request_id=invocation_request_id,
                    reset_generation=invocation_reset_generation,
                )
            return
        with self._route_state_lock():
            transaction_current = (
                self.graph_transaction_generation == generation
                and self._graph_switch_request_is_current_locked(
                    generation, graph
                )
                and (
                    expected_validation is None
                    or self._cognitive_validation_is_current_locked(
                        expected_validation
                    )
                )
            )
            if not transaction_current:
                if self.graph_transaction_generation == generation:
                    self.graph_transaction_generation = None
                    self.graph_transaction_switch_context = None
                    self.cognitive_graph_switch_pending = False
                    self.cognitive_graph_feedback_pending = None
                self._schedule_graph_retry_locked(
                    "graph switch invalidated before submit",
                    switch_context=None,
                )
            else:
                self.desired_support = support
        if not transaction_current:
            return
        request = self.SetRouteGraph.Request()
        request.graph_filepath = str(geojson)
        try:
            future = self.set_graph_client.call_async(request)
        except Exception as error:
            with self._route_state_lock():
                invalidated = not self._graph_switch_request_is_current_locked(
                    generation, graph
                )
                if self.graph_transaction_generation == generation:
                    self.graph_transaction_generation = None
                    self.graph_transaction_switch_context = None
                    self.cognitive_graph_switch_pending = False
                    self.cognitive_graph_feedback_pending = None
                self._schedule_graph_retry_locked(
                    "graph switch invalidated during request failure"
                    if invalidated else f"SetRouteGraph request failed: {error}",
                    switch_context=None if invalidated else switch_context,
                )
            if invalidated:
                return
            self._publish_structural_status(
                self.StructuralGraphStatus.LAST_KNOWN_GOOD,
                f"SetRouteGraph request failed: {error}",
            )
            return
        self._register_graph_transaction_future(generation, future, "switch")
        future.add_done_callback(
            lambda completed: self._finish_cognitive_graph_switch(
                completed, graph, support, detail, fallback, generation,
                feedback, candidate,
            )
        )

    def _finish_cognitive_graph_switch(
        self, future, graph, support, detail: str, fallback: bool,
        generation: GraphSwitchGeneration | None = None,
        feedback: CognitiveGraphFeedback | None = None, candidate=None,
    ) -> None:
        # Always consume a completed SetRouteGraph response: success changes
        # Route Server state even when reset/preemption made this callback stale.
        try:
            response = future.result()
        except Exception as error:
            response = None
            detail = f"SetRouteGraph exception: {error}"
        succeeded = bool(response is not None and getattr(response, "success", False))
        requested_identity = self._graph_identity(graph)
        commit = False
        pending_outcome = None
        cancel_handle = None
        prepare_pending_goal = False
        current_failure = False
        with self._route_state_lock():
            transaction_matches = (
                generation is None
                or getattr(self, "graph_transaction_generation", None) == generation
            )
            transaction_context = (
                getattr(self, "graph_transaction_switch_context", None)
                if transaction_matches else None
            )
            self._clear_graph_transaction_future_locked(generation, future)
            if transaction_matches:
                self.graph_transaction_generation = None
                self.graph_transaction_switch_context = None
                self.cognitive_graph_switch_pending = False
                self.cognitive_graph_feedback_pending = None
            desired = getattr(self, "desired_graph", graph)
            desired_identity = self._graph_identity(desired)
            generation_current = generation is None or (
                generation.reset_generation
                == int(getattr(self, "reset_generation", 0))
                and generation.desired_generation
                == int(getattr(self, "desired_graph_generation", 0))
                and generation.base_graph_generation
                == int(getattr(self, "graph_generation", 0))
                and generation.route_request_id
                == (
                    int(self.request_id)
                    if getattr(self, "pending_goal", None) is not None
                    else None
                )
            )
            commit = bool(
                succeeded
                and transaction_matches
                and generation_current
                and requested_identity == desired_identity
            )
            if commit:
                self.graph = graph
                self.support = support
                self.graph_generation = int(
                    getattr(self, "graph_generation", 0)
                ) + 1
                self.support_node_positions = {
                    int(feature["properties"]["id"]): tuple(
                        float(value)
                        for value in feature["geometry"]["coordinates"]
                    )
                    for feature in support.geojson["features"]
                    if feature["geometry"]["type"] == "Point"
                }
                self._clear_latest_priors()
                if not fallback and feedback is not None:
                    if candidate is not None:
                        self.cognitive_graph_identity = candidate.identity
                        self.cognitive_graph_last_sequence = candidate.source_sequence
                    self.cognitive_graph_feedback_active = feedback
                elif fallback:
                    pending_outcome = getattr(self, "pending_reroute_outcome", None)
                    if pending_outcome is not None:
                        self.cognitive_reroute_revision = int(
                            getattr(self, "cognitive_reroute_revision", 0)
                        ) + 1
                    self.pending_reroute_outcome = None
                    self.cognitive_graph_feedback_active = None
                self.cognitive_constraints_cache.invalidate()
                self.graph_reassert_required = False
                self.graph_coherent = True
                self._clear_graph_retry_locked()
                if self.pending_goal is not None:
                    cancel_handle = self.navigation_goal_handle
                    self.navigation_goal_handle = None
                    self.navigation_goal_pending = False
                    self.navigation_goal_targets_final = False
                    self.navigation_failed = False
                    self.tracker = None
                    prepare_pending_goal = True
            else:
                if succeeded:
                    # A stale success may have changed Route Server state even
                    # though it cannot commit local state.
                    retry_context = (
                        getattr(self, "graph_retry_switch_context", None)
                        if getattr(self, "graph_retry_key", None)
                        == self._graph_retry_key_locked()
                        else None
                    )
                    self._schedule_graph_retry_locked(
                        "stale SetRouteGraph success",
                        switch_context=retry_context,
                    )
                else:
                    current_failure = bool(
                        transaction_matches
                        and generation_current
                        and requested_identity == desired_identity
                    )
                    if current_failure:
                        self._schedule_graph_retry_locked(
                            "current SetRouteGraph rejection",
                            switch_context=transaction_context,
                        )

        if not succeeded:
            if not current_failure:
                self._try_deferred_structural_rebuild()
                return
            self._publish_structural_status(
                self.StructuralGraphStatus.LAST_KNOWN_GOOD,
                f"cognitive graph {'fallback' if fallback else 'rejected'}: {detail}",
            )
            if feedback is not None and (generation is None or generation_current):
                self._publish_graph_validation(
                    replace(
                        feedback,
                        validated_graph_id=str(self.graph.graph_id),
                        validated_graph_revision=int(self.graph.revision),
                    ), accepted=False,
                    reason=f"set_route_graph_rejected: {detail}",
                )
            if current_failure and not fallback and self._primary_fallback_available():
                self._fallback_to_gvg_once(
                    detail,
                    request_id=(
                        None if generation is None else generation.route_request_id
                    ),
                    reset_generation=(
                        None if generation is None else generation.reset_generation
                    ),
                )
            self._try_deferred_structural_rebuild()
            return
        if not commit:
            self._publish_structural_status(
                self.StructuralGraphStatus.LAST_KNOWN_GOOD,
                "stale SetRouteGraph success; reconciling desired graph",
            )
            self._try_deferred_structural_rebuild()
            return
        if not fallback and feedback is not None:
            self._publish_graph_validation(
                feedback, accepted=True, reason="set_route_graph_accepted"
            )
        elif fallback and pending_outcome is not None:
            previous_feedback, validated_edge_id, candidate_edge_id = pending_outcome
            self._publish_edge_outcome(
                previous_feedback, validated_edge_id, candidate_edge_id,
                success=False, reason="whole_gvg_reroute_applied",
                reroute_applied=True,
            )
        self._cancel_navigation_handle(cancel_handle)
        self._publish_graph()
        self._publish_cognitive_constraints()
        self._publish_structural_status(
            self.StructuralGraphStatus.READY,
            f"cognitive graph {'fallback applied' if fallback else 'applied'}: {detail}",
        )
        if prepare_pending_goal:
            self._resume_pending_goal_after_graph_coherent()
        self._try_deferred_structural_rebuild()

    def _ensure_desired_graph(self, reason: str) -> None:
        with self._route_state_lock():
            if getattr(self, "graph_transaction_generation", None) is not None:
                self._schedule_graph_retry_locked(reason, immediate=True)
                return
            if (
                getattr(self, "graph_retry_key", None)
                == self._graph_retry_key_locked()
                and getattr(self, "graph_retry_due_steady_s", None) is not None
            ):
                return
            graph = getattr(self, "desired_graph", self.gvg_graph)
            fallback = self._graph_identity(graph) == self._graph_identity(
                self.gvg_graph
            )
        self._request_graph_switch(graph, reason, fallback=fallback)

    def _resume_pending_goal_after_graph_coherent(self) -> None:
        with self._route_state_lock():
            if (
                self._reset_barrier_is_held()
                or self.pending_goal is None
                or not self._desired_graph_is_coherent_locked()
            ):
                return
            module2_enabled = bool(self.module2_enabled)
            if module2_enabled:
                self._arm_prior_request(int(self._now().nanoseconds))
            else:
                self._clear_pending_prior_request()
        self._publish_route_context()
        if not module2_enabled:
            self._prepare_route({})

    def _fallback_to_gvg_once(
        self,
        reason: str,
        generation: RouteCallbackGeneration | None = None,
        *,
        request_id: int | None = None,
        reset_generation: int | None = None,
    ) -> None:
        with self._route_state_lock():
            stale = (
                (generation is not None and not self._route_callback_is_current(generation))
                or (
                    request_id is not None
                    and request_id != int(getattr(self, "request_id", 0))
                )
                or (
                    reset_generation is not None
                    and reset_generation
                    != int(getattr(self, "reset_generation", 0))
                )
            )
            if stale:
                return
            if self.primary_fallback_used:
                already_used = True
            else:
                already_used = False
                self.primary_fallback_used = True
                self._set_desired_graph_locked(
                    self.gvg_graph,
                    getattr(self, "gvg_support", getattr(self, "support", None)),
                    require_reassert=True,
                )
                retained = self._desired_graph_is_coherent_locked()
                pending_goal = getattr(self, "pending_goal", None) is not None
                if retained:
                    self.pending_reroute_outcome = None
                    self.cognitive_graph_feedback_active = None
        if already_used:
            self._publish_structural_status(
                self.StructuralGraphStatus.LAST_KNOWN_GOOD,
                f"cognitive graph fallback already used: {reason}",
            )
            return
        if retained:
            self._publish_structural_status(
                self.StructuralGraphStatus.READY,
                f"cognitive graph fallback retained GVG: {reason}",
            )
            if pending_goal:
                self._prepare_route({})
            return
        self._ensure_desired_graph(reason)

    def _region_tick(self) -> None:
        if self._reset_barrier_is_held() or self.region_selector is None:
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
        # Fence old action callbacks and remove its tracker before exposing the
        # new request to the prior/context path.
        with self._route_state_lock():
            if self._reset_barrier_is_held():
                self.node.get_logger().warning(
                    "route goal rejected while simulation reset HOLD is active"
                )
                return
            was_preemption = bool(
                getattr(self, "route_active", False)
                or getattr(self, "pending_goal", None) is not None
            )
            self.request_id += 1
            previous_handle = self._retire_route_state()
            if (
                getattr(self, "graph_retry_kind", "switch") == "structural"
                and getattr(self, "graph_retry_due_steady_s", None) is not None
            ):
                self._clear_graph_retry_locked()
            self.primary_fallback_used = False
            self.pending_reroute_outcome = None
            self.pending_goal = goal
            self.route_active = True
            graph_was_incoherent = not self._desired_graph_is_coherent_locked()
            if was_preemption or graph_was_incoherent:
                self._set_desired_graph_locked(
                    self.gvg_graph,
                    getattr(self, "gvg_support", getattr(self, "support", None)),
                    require_reassert=True,
                )
                # Reassert the server graph; a pending older switch will be
                # consumed and compensated before this new goal can route.
                self.graph_coherent = False
            coherent = self._desired_graph_is_coherent_locked()
            request_id = int(self.request_id)
        self._cancel_navigation_handle(previous_handle)
        self.node.get_logger().info(
            "received route goal request "
            f"{request_id}: ({goal.pose.position.x:.3f}, "
            f"{goal.pose.position.y:.3f})"
        )
        if was_preemption or graph_was_incoherent:
            self._ensure_desired_graph("new goal requires Route Server GVG")
        if not coherent:
            self._publish_structural_status(
                self.StructuralGraphStatus.LAST_KNOWN_GOOD,
                "route goal waiting for coherent Route Server graph",
            )
            return
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
        if getattr(self, "cognitive_graph_mode", "gvg") in {"primary", "hybrid"}:
            timeout_s = max(
                timeout_s,
                float(getattr(self, "cognitive_goal_prior_wait_s", 4.0)),
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

    def _clear_latest_priors(self) -> None:
        self.latest_priors = {}
        self.latest_priors_stamp_ns = None
        self.latest_prior_model_id = None
        self.latest_priors_request_id = None
        self.latest_priors_graph_id = None
        self.latest_priors_graph_revision = None

    def _priors_for_consumption(
        self, priors: dict[int, tuple[float, float]]
    ) -> dict[int, tuple[float, float]]:
        """Recheck TTL and route identity at the exact DynamicEdges use site."""

        if not priors:
            return {}
        now_ns = int(self._now().nanoseconds)
        stamp_ns = self.latest_priors_stamp_ns
        age_s = (
            math.inf if stamp_ns is None else (now_ns - int(stamp_ns)) / 1.0e9
        )
        request_id = getattr(self, "latest_priors_request_id", self.request_id)
        graph_id = getattr(
            self, "latest_priors_graph_id", self.graph.graph_id
        )
        graph_revision = getattr(
            self, "latest_priors_graph_revision", self.graph.revision
        )
        model_id = str(self.latest_prior_model_id or '')
        if (
            age_s < 0.0
            or age_s > self.module2_prior_ttl_s
            or request_id != self.request_id
            or graph_id != self.graph.graph_id
            or graph_revision != self.graph.revision
            or not model_id
        ):
            if priors == self.latest_priors:
                self._clear_latest_priors()
            self.node.get_logger().warning(
                "Module2 edge prior expired or changed identity at consumption; "
                "using geometry-only route"
            )
            return {}
        return priors

    def _publish_route_context(self) -> None:
        with self._route_output_lock():
            with self._route_state_lock():
                if self._reset_barrier_is_held() or self.pending_goal is None:
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
        if self._reset_barrier_is_held():
            return
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
            self._clear_latest_priors()
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
        out_of_distribution = any(
            bool(getattr(message, name, False))
            for name in ("out_of_distribution", "ood", "ood_detected")
        )
        trusted = bool(getattr(message, "trusted_write", True))
        rejection_mask = int(getattr(message, "rejection_mask", 0))
        usable, reason = edge_prior_is_usable(
            healthy=(
                bool(message.healthy)
                and trusted
                and rejection_mask == 0
                and not out_of_distribution
            ),
            model_id=str(message.model_id),
            stamp_ns=stamp_ns,
            now_ns=now_ns,
            max_age_s=self.module2_prior_ttl_s,
            priors=rows,
        )
        if not usable:
            self._clear_latest_priors()
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
        self.latest_priors_request_id = int(message.request_id)
        self.latest_priors_graph_id = str(message.graph_id)
        self.latest_priors_graph_revision = int(message.graph_revision)
        self._prepare_route(priors)

    def _check_prior_timeout(self) -> None:
        if (
            not self._reset_barrier_is_held()
            and self.pending_goal is not None
            and self.pending_deadline_ns is not None
            and int(self._now().nanoseconds) >= self.pending_deadline_ns
        ):
            self._clear_pending_prior_request()
            self._clear_latest_priors()
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
        with self._route_state_lock():
            if (
                self._reset_barrier_is_held()
                or self.pending_goal is None
                or not self._desired_graph_is_coherent_locked()
            ):
                return
            priors = self._priors_for_consumption(priors)
            generation = self._route_callback_generation()
        current = self._current_xy()
        if current is None:
            self.node.get_logger().warning("route request has no map pose")
            return
        with self._route_state_lock():
            if not self._route_callback_is_current(generation):
                return
            goal_xy = (
                float(self.pending_goal.pose.position.x),
                float(self.pending_goal.pose.position.y),
            )
            graph = self.graph
            support = self.support
            occupancy = self.map
            support_node_positions = dict(self.support_node_positions)
        start_node = select_support_attachment(
            occupancy,
            support_node_positions,
            current,
            self.defaults["footprint"],
            departing=True,
        )
        goal_node = select_support_attachment(
            occupancy,
            support_node_positions,
            goal_xy,
            self.defaults["footprint"],
            departing=False,
        )
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
        cost_message.graph_id = graph.graph_id
        cost_message.graph_revision = graph.revision
        runtime_view = self.runtime.route_cost_view()
        edge_map = graph.edge_by_id()
        for canonical_id, support_ids in support.canonical_to_support_edges.items():
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
        with self._route_output_lock():
            with self._route_state_lock():
                if not self._route_callback_is_current(generation):
                    return
            self.route_edge_cost_pub.publish(cost_message)
            if not self.dynamic_client.service_is_ready():
                self.node.get_logger().warning("DynamicEdges service unavailable")
                if self._primary_fallback_available():
                    self._fallback_to_gvg_once(
                        "DynamicEdges unavailable", generation
                    )
                return
            with self._route_state_lock():
                if not self._route_callback_is_current(generation):
                    return
            future = self.dynamic_client.call_async(request)
        future.add_done_callback(
            lambda completed, start=start_node, goal=goal_node: self._after_edge_update(
                completed, start, goal, generation
            )
        )

    def _after_edge_update(
        self,
        future,
        start_node: int,
        goal_node: int,
        generation: RouteCallbackGeneration | None = None,
    ) -> None:
        try:
            response = future.result()
        except Exception as error:
            response = None
            failure_detail = f"DynamicEdges failed: {error}"
        else:
            failure_detail = "DynamicEdges rejected or route unavailable"
        with self._route_state_lock():
            if not self._route_callback_is_current(generation):
                return
        route_ready = bool(
            response is not None
            and getattr(response, "success", False)
            and self.route_client.server_is_ready()
        )
        with self._route_state_lock():
            if not self._route_callback_is_current(generation):
                return
            failed = response is None or not response.success or not route_ready
            fallback = failed and self._primary_fallback_available()
            if not failed:
                goal = self.ComputeRoute.Goal()
                goal.start_id = int(start_node)
                goal.goal_id = int(goal_node)
                goal.use_start = True
                goal.use_poses = False
        if response is None:
            self.node.get_logger().warning(failure_detail)
            if fallback:
                self._fallback_to_gvg_once(failure_detail, generation)
            return
        if failed:
            self.node.get_logger().warning("route services are not ready")
            if fallback:
                self._fallback_to_gvg_once(failure_detail, generation)
            return
        with self._route_output_lock():
            with self._route_state_lock():
                if not self._route_callback_is_current(generation):
                    return
            self.node.get_logger().info(
                f"edge update accepted for route request {self.request_id}: "
                f"support {start_node}->{goal_node}"
            )
            future = self.route_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed: self._on_route_goal_handle(completed, generation)
        )

    def _on_route_goal_handle(
        self,
        future,
        generation: RouteCallbackGeneration | None = None,
    ) -> None:
        handle = future.result()
        with self._route_state_lock():
            current = self._route_callback_is_current(generation)
            fallback = current and (
                handle is None or not handle.accepted
            ) and self._primary_fallback_available()
        if not current:
            if handle is not None and handle.accepted:
                self._cancel_navigation_handle(handle)
            return
        if handle is None or not handle.accepted:
            self.node.get_logger().warning("ComputeRoute rejected")
            if fallback:
                self._fallback_to_gvg_once("ComputeRoute rejected", generation)
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda completed: self._on_route_result(completed, generation)
        )

    def _on_route_result(
        self,
        future,
        generation: RouteCallbackGeneration | None = None,
    ) -> None:
        wrapped = future.result()
        failure = None
        fallback = False
        message = None
        with self._route_state_lock():
            if not self._route_callback_is_current(generation):
                return
            if wrapped is None or int(wrapped.result.error_code) != 0:
                code = -1 if wrapped is None else int(wrapped.result.error_code)
                failure = f"ComputeRoute failed with error {code}"
            else:
                canonical_ids = []
                route_segments: list[list[tuple[float, float]]] = []
                for support_edge in wrapped.result.route.edges:
                    canonical = self.support.support_to_canonical_edge.get(
                        int(support_edge.edgeid)
                    )
                    if canonical is None:
                        continue
                    start = (
                        float(support_edge.start.x), float(support_edge.start.y)
                    )
                    end = (float(support_edge.end.x), float(support_edge.end.y))
                    if not canonical_ids or canonical_ids[-1] != canonical:
                        canonical_ids.append(canonical)
                        route_segments.append([start, end])
                    else:
                        route_segments[-1].append(end)
                if not canonical_ids:
                    failure = "ComputeRoute returned no canonical edges"
                else:
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
                    self.tracker = RouteTracker(
                        self.graph,
                        canonical_ids,
                        self.defaults["route_tracking"],
                        route_segments_xy=[
                            np.asarray(points, dtype=np.float64)
                            for points in route_segments
                        ],
                    )
                    self.navigation_failed = False
            fallback = failure is not None and self._primary_fallback_available()
        if failure is not None:
            self.node.get_logger().warning(failure)
            if fallback:
                self._fallback_to_gvg_once(failure, generation)
            return
        assert message is not None
        with self._route_output_lock():
            with self._route_state_lock():
                if not self._route_callback_is_current(generation):
                    return
            self.route_pub.publish(message)
            self.node.get_logger().info(
                f"canonical route ready for request {message.request_id}: "
                f"{len(message.edge_ids)} edges, cost {message.total_cost_m:.3f} m"
            )

    def _publish_progress(self) -> None:
        with self._route_state_lock():
            if (
                self._reset_barrier_is_held()
                or self.tracker is None
                or self.pending_goal is None
                or not self._desired_graph_is_coherent_locked()
            ):
                return
            generation = self._route_callback_generation()
            tracker = self.tracker
        current = self._current_xy()
        if current is None:
            return
        with self._route_state_lock():
            if (
                not self._route_callback_is_current(generation)
                or self.tracker is not tracker
                or not self._desired_graph_is_coherent_locked()
            ):
                return
            previous_edge_index = int(tracker.edge_index)
            progress = tracker.update(current)
            crossed = (
                (previous_edge_index, progress.edge_index)
                if progress.edge_index > previous_edge_index else None
            )
            if (
                self.latest_global_costmap is not None
                and self.latest_global_costmap.frame_id == self.frame_id
            ):
                progress = select_live_feasible_lookahead(
                    tracker,
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
            self.navigation_goal_targets_final = bool(progress.use_final_goal)
            start_navigation = bool(
                self.execute_navigation
                and not self.navigation_failed
                and not self.navigation_goal_pending
                and self.navigation_goal_handle is None
            )
        with self._route_output_lock():
            with self._route_state_lock():
                if (
                    not self._route_callback_is_current(generation)
                    or self.tracker is not tracker
                ):
                    return
            if crossed is not None:
                self._publish_crossed_edge_outcomes(*crossed)
            self.progress_pub.publish(message)
            self.lookahead_pub.publish(message.lookahead_goal)
            self.goal_update_pub.publish(message.lookahead_goal)
        if start_navigation:
            self._start_navigation(message.lookahead_goal)

    def _start_navigation(self, first_lookahead) -> None:
        if not self.navigation_client.server_is_ready():
            return
        with self._route_output_lock():
            with self._route_state_lock():
                if (
                    self._reset_barrier_is_held()
                    or self.pending_goal is None
                    or self.navigation_goal_pending
                    or self.navigation_goal_handle is not None
                    or not self._desired_graph_is_coherent_locked()
                ):
                    return
                goal = self.NavigateToPose.Goal()
                goal.pose = first_lookahead
                goal.behavior_tree = self.route_guided_bt_xml
                self.navigation_goal_pending = True
                generation = self._route_callback_generation()
            future = self.navigation_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed: self._on_navigation_goal_handle(completed, generation)
        )

    def _on_navigation_goal_handle(
        self,
        future,
        generation: RouteCallbackGeneration | None = None,
    ) -> None:
        handle = future.result()
        rejected = False
        fallback = False
        edge_failure = None
        late_handle = None
        terminal_snapshot = None
        rebuild = False
        with self._route_output_lock():
            with self._route_state_lock():
                current = (
                    self._route_callback_is_current(generation)
                    and bool(getattr(self, "navigation_goal_pending", False))
                )
                if current:
                    self.navigation_goal_pending = False
                    if handle is None or not handle.accepted:
                        rejected = True
                        edge_failure = self._cognitive_route_edge()
                        fallback = self._primary_fallback_available()
                        if edge_failure is not None and fallback:
                            self.pending_reroute_outcome = edge_failure
                        if fallback:
                            self.navigation_failed = False
                            self.tracker = None
                        else:
                            terminal_snapshot = (
                                int(getattr(self, "request_id", 0)),
                                int(getattr(
                                    getattr(self, "cognitive_graph_identity", None),
                                    "reset_epoch", 0,
                                )),
                            )
                            self._retire_route_state()
                            rebuild = (
                                getattr(self, "pending_structural_map", None)
                                is not None
                            )
                    else:
                        self.navigation_goal_handle = handle
            if not current:
                if handle is not None and handle.accepted:
                    late_handle = handle
            elif rejected:
                if edge_failure is not None:
                    feedback, validated_edge_id, candidate_edge_id = edge_failure
                    self._publish_edge_outcome(
                        feedback, validated_edge_id, candidate_edge_id,
                        success=False, reason="navigate_to_pose_rejected",
                    )
                if terminal_snapshot is not None:
                    request_id, reset_epoch = terminal_snapshot
                    self._publish_route_terminal_pair(
                        success=False,
                        request_id=request_id,
                        status="failed",
                        reason="navigate_to_pose_rejected",
                        reset_epoch=reset_epoch,
                    )
        if not current:
            self._cancel_navigation_handle(late_handle)
            return
        if rejected:
            if fallback:
                self._fallback_to_gvg_once(
                    "NavigateToPose rejected", generation
                )
            else:
                self.node.get_logger().warning(
                    "route-guided NavigateToPose rejected"
                )
                if rebuild:
                    self._rebuild_structural_graph()
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda completed: self._on_navigation_result(completed, generation)
        )

    def _on_navigation_result(
        self,
        future,
        generation: RouteCallbackGeneration | None = None,
    ) -> None:
        wrapped = future.result()
        succeeded = navigation_result_succeeded(wrapped)
        result_code = -1 if wrapped is None else int(wrapped.result.error_code)
        current_xy = self._current_xy() if succeeded else None
        intermediate_suffix = None
        fallback = False
        edge = None
        rebuild = False
        terminal_snapshot = None
        with self._route_output_lock():
            with self._route_state_lock():
                if (
                    not self._route_callback_is_current(generation)
                    or getattr(self, "navigation_goal_handle", None) is None
                ):
                    return
                completion_confirmed = False
                final_xy = None
                if current_xy is not None and self.pending_goal is not None:
                    final_xy = (
                        float(self.pending_goal.pose.position.x),
                        float(self.pending_goal.pose.position.y),
                    )
                    completion_confirmed = (
                        bool(getattr(self, "navigation_goal_targets_final", False))
                        and math.dist(current_xy, final_xy)
                        <= self.route_goal_completion_tolerance_m
                    )
                if succeeded and not completion_confirmed:
                    self.navigation_goal_pending = False
                    self.navigation_goal_handle = None
                    self.navigation_failed = False
                    intermediate_suffix = (
                        "map pose unavailable"
                        if final_xy is None
                        else f"final goal is ({final_xy[0]:.3f}, {final_xy[1]:.3f})"
                    )
                elif not succeeded and self._primary_fallback_available():
                    edge = self._cognitive_route_edge()
                    if edge is not None:
                        self.pending_reroute_outcome = edge
                    self.navigation_goal_pending = False
                    self.navigation_goal_handle = None
                    self.navigation_goal_targets_final = False
                    self.navigation_failed = False
                    self.tracker = None
                    fallback = True
                else:
                    edge = self._cognitive_route_edge()
                    terminal_snapshot = (
                        int(getattr(self, "request_id", 0)),
                        int(getattr(
                            getattr(self, "cognitive_graph_identity", None),
                            "reset_epoch", 0,
                        )),
                        bool(succeeded),
                        (
                            "final_goal_distance_confirmed"
                            if succeeded
                            else f"navigate_to_pose_failed_error_{result_code}"
                        ),
                    )
                    self._retire_route_state()
                    rebuild = (
                        getattr(self, "pending_structural_map", None) is not None
                    )
            if edge is not None:
                feedback, validated_edge_id, candidate_edge_id = edge
                self._publish_edge_outcome(
                    feedback, validated_edge_id, candidate_edge_id,
                    success=succeeded,
                    reason=(
                        "final_goal_distance_confirmed" if succeeded
                        else f"navigate_to_pose_failed_error_{result_code}"
                    ),
                )
            if terminal_snapshot is not None:
                request_id, reset_epoch, terminal_success, reason = terminal_snapshot
                self._publish_route_terminal_pair(
                    success=terminal_success,
                    request_id=request_id,
                    status="succeeded" if terminal_success else "failed",
                    reason=reason,
                    reset_epoch=reset_epoch,
                )
        if intermediate_suffix is not None:
            self.node.get_logger().info(
                "intermediate route lookahead reached; continuing because "
                f"{intermediate_suffix}"
            )
            return
        if fallback:
            self._fallback_to_gvg_once(
                f"NavigateToPose failed with error {result_code}", generation
            )
            return
        if rebuild:
            self._rebuild_structural_graph()
        if succeeded:
            self.node.get_logger().info("route-guided navigation completed")
            return
        self.node.get_logger().warning(
            f"route-guided navigation failed with error {result_code}"
        )

    def _on_runtime_observation(self, message) -> None:
        if self._reset_barrier_is_held():
            return
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
        if self._reset_barrier_is_held():
            return
        now_ns = int(self._now().nanoseconds)
        if self.latest_priors_stamp_ns is not None:
            age_s = (now_ns - self.latest_priors_stamp_ns) / 1.0e9
            if age_s < 0.0 or age_s > self.module2_prior_ttl_s:
                self._clear_latest_priors()
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

    def _publish_runtime_states(self, *, graph=None) -> None:
        graph = self.graph if graph is None else graph
        message = self.RuntimeEdgeStateArray()
        message.header.stamp = self._now().to_msg()
        message.header.frame_id = self.frame_id
        message.graph_id = graph.graph_id
        message.graph_revision = graph.revision
        RuntimeEdgeState = None
        for state in sorted(self.runtime.edges.values(), key=lambda item: item.edge_id):
            if RuntimeEdgeState is None:
                from bio_nav_interfaces.msg import RuntimeEdgeState
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
        if self._reset_barrier_is_held():
            return
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
            candidate_map = OccupancyMap(
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
            with self._route_state_lock():
                self.pending_structural_map = candidate_map
                self._refresh_structural_intent_locked()
            self._try_deferred_structural_rebuild()

    def _finish_active_route(self) -> None:
        """Synchronously retire one coordinator-owned Nav2 action."""

        with self._route_state_lock():
            self._retire_route_state()
            if getattr(self, "pending_structural_map", None) is not None:
                self._refresh_structural_intent_locked()
        self._try_deferred_structural_rebuild()

    def _rebuild_structural_graph(self) -> None:
        with self._route_state_lock():
            candidate_map = self.pending_structural_map
            if (
                candidate_map is None
                or self.route_active
                or self.pending_goal is not None
                or getattr(self, "graph_transaction_generation", None) is not None
                or getattr(self, "graph_retry_due_steady_s", None) is not None
            ):
                return
            intent = self._refresh_structural_intent_locked()
            if intent is None:
                return
            base_graph = self.graph
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
                    revision=base_graph.revision + 1,
                ),
                candidate_map,
                self.defaults["footprint"],
            )
            if self.feasible_only_largest_component:
                retain_largest_feasible_component(candidate)
            candidate = stabilize_graph_ids(
                candidate, base_graph, self.defaults["graph"]
            )
            support = export_route_support_graph(
                candidate,
                support_spacing_m=float(self.defaults["graph"]["route_support_spacing_m"]),
            )
        except Exception as error:
            with self._route_state_lock():
                if self._structural_intent_is_current_locked(intent):
                    self._schedule_graph_retry_locked(
                        f"structural rebuild failed: {error}", kind="structural"
                    )
            self._publish_structural_status(
                self.StructuralGraphStatus.LAST_KNOWN_GOOD,
                f"rebuild failed: {error}",
            )
            return
        if not self.set_graph_client.service_is_ready():
            with self._route_state_lock():
                if self._structural_intent_is_current_locked(intent):
                    self._schedule_graph_retry_locked(
                        "SetRouteGraph service unavailable", kind="structural"
                    )
            self._publish_structural_status(
                self.StructuralGraphStatus.LAST_KNOWN_GOOD,
                "SetRouteGraph service unavailable",
            )
            return
        with self._route_state_lock():
            if (
                not self._structural_intent_is_current_locked(intent)
                or self.route_active
                or self.pending_goal is not None
            ):
                return
            if getattr(self, "graph_transaction_generation", None) is not None:
                return
            self.structural_generation = int(
                getattr(self, "structural_generation", 0)
            ) + 1
            self._set_desired_graph_locked(candidate, support)
            self.graph_coherent = False
            rebuild_generation = StructuralRebuildGeneration(
                intent.request_id,
                intent.reset_generation,
                int(self.structural_generation),
                int(getattr(self, "desired_graph_generation", 0)),
                str(candidate.graph_id),
                int(candidate.revision),
                intent.base_graph_generation,
                intent.candidate_generation,
                intent.candidate_identity,
            )
            self.graph_switch_generation = int(
                getattr(self, "graph_switch_generation", 0)
            ) + 1
            transaction = GraphSwitchGeneration(
                self.graph_switch_generation,
                intent.request_id,
                int(getattr(self, "graph_generation", 0)),
                intent.reset_generation,
                int(getattr(self, "desired_graph_generation", 0)),
                str(candidate.graph_id),
                int(candidate.revision),
            )
            self.graph_transaction_generation = transaction
            self.graph_transaction_future = None
            self.graph_transaction_deadline_steady_s = None
            self.graph_transaction_kind = "structural"
            self.graph_transaction_switch_context = None
            self.cognitive_graph_switch_pending = True
        try:
            geojson, mapping = self._graph_transaction_paths(
                "structural", transaction
            )
            save_route_support(support, geojson, mapping)
        except Exception as error:
            with self._route_state_lock():
                invalidated = (
                    not self._structural_intent_is_current_locked(intent)
                    or int(getattr(self, "reset_generation", 0))
                    != intent.reset_generation
                    or self._graph_identity(
                        getattr(self, "desired_graph", candidate)
                    ) != self._graph_identity(candidate)
                )
                if self.graph_transaction_generation == transaction:
                    self.graph_transaction_generation = None
                    self.graph_transaction_switch_context = None
                    self.cognitive_graph_switch_pending = False
                self._schedule_graph_retry_locked(
                    "structural export invalidated" if invalidated
                    else f"rebuild export failed: {error}",
                    kind="switch" if invalidated else "structural",
                )
            if invalidated:
                return
            self._publish_structural_status(
                self.StructuralGraphStatus.LAST_KNOWN_GOOD,
                f"rebuild export failed: {error}",
            )
            return
        with self._route_state_lock():
            transaction_current = (
                self.graph_transaction_generation == transaction
                and self._structural_intent_is_current_locked(intent)
                and int(self.request_id) == intent.request_id
                and int(getattr(self, "reset_generation", 0))
                == intent.reset_generation
                and not self.route_active
                and self.pending_goal is None
                and self._graph_identity(getattr(self, "desired_graph", candidate))
                == self._graph_identity(candidate)
            )
            if not transaction_current:
                if self.graph_transaction_generation == transaction:
                    self.graph_transaction_generation = None
                    self.graph_transaction_switch_context = None
                    self.cognitive_graph_switch_pending = False
                self._schedule_graph_retry_locked(
                    "structural rebuild invalidated before submit",
                    kind="switch",
                )
        if not transaction_current:
            return
        request = self.SetRouteGraph.Request()
        request.graph_filepath = str(geojson)
        try:
            future = self.set_graph_client.call_async(request)
        except Exception as error:
            with self._route_state_lock():
                invalidated = (
                    not self._structural_intent_is_current_locked(intent)
                    or int(getattr(self, "reset_generation", 0))
                    != intent.reset_generation
                    or self._graph_identity(
                        getattr(self, "desired_graph", candidate)
                    ) != self._graph_identity(candidate)
                )
                if self.graph_transaction_generation == transaction:
                    self.graph_transaction_generation = None
                    self.graph_transaction_switch_context = None
                    self.cognitive_graph_switch_pending = False
                self._schedule_graph_retry_locked(
                    "structural rebuild invalidated during request failure"
                    if invalidated else f"SetRouteGraph rebuild request failed: {error}",
                    kind="switch" if invalidated else "structural",
                )
            if invalidated:
                return
            self._publish_structural_status(
                self.StructuralGraphStatus.LAST_KNOWN_GOOD,
                f"SetRouteGraph rebuild request failed: {error}",
            )
            return
        self._register_graph_transaction_future(transaction, future, "structural")
        future.add_done_callback(
            lambda completed: self._finish_rebuild(
                completed, candidate, candidate_map, support,
                rebuild_generation, transaction,
            )
        )

    def _finish_rebuild(
        self, future, graph, occupancy, support,
        generation: StructuralRebuildGeneration | None = None,
        transaction: GraphSwitchGeneration | None = None,
    ) -> None:
        try:
            response = future.result()
        except Exception as error:
            response = None
            self.node.get_logger().warning(f"SetRouteGraph failed: {error}")
        succeeded = bool(response is not None and response.success)
        commit = False
        current_failure = False
        with self._route_state_lock():
            transaction_matches = (
                transaction is None
                or getattr(self, "graph_transaction_generation", None) == transaction
            )
            self._clear_graph_transaction_future_locked(transaction, future)
            if transaction_matches:
                self.graph_transaction_generation = None
                self.graph_transaction_switch_context = None
                self.cognitive_graph_switch_pending = False
            desired = getattr(self, "desired_graph", graph)
            requested_identity = self._graph_identity(graph)
            desired_identity = self._graph_identity(desired)
            generation_current = generation is None or (
                generation.request_id == int(self.request_id)
                and generation.base_graph_generation
                == int(getattr(self, "graph_generation", 0))
                and generation.reset_generation
                == int(getattr(self, "reset_generation", 0))
                and generation.structural_generation
                == int(getattr(self, "structural_generation", 0))
                and generation.desired_generation
                == int(getattr(self, "desired_graph_generation", 0))
                and getattr(self, "pending_structural_intent", None) is not None
                and generation.candidate_generation
                == self.pending_structural_intent.candidate_generation
                and generation.candidate_identity
                == self.pending_structural_intent.candidate_identity
                and id(getattr(self, "pending_structural_map", None))
                == generation.candidate_identity
                and not self.route_active
                and self.pending_goal is None
            )
            commit = bool(
                succeeded
                and transaction_matches
                and generation_current
                and requested_identity == desired_identity
            )
            if commit:
                self.graph = graph
                self.map = occupancy
                self.support = support
                self.gvg_graph = graph
                self.gvg_support = support
                self.graph_generation = int(
                    getattr(self, "graph_generation", 0)
                ) + 1
                self.cognitive_graph_last_sequence = 0
                self.cognitive_graph_identity = CognitiveGraphIdentity(
                    self.cognitive_graph_identity.reset_epoch,
                    self.cognitive_graph_identity.recurrent_session_id,
                    occupancy.map_version,
                    self.cognitive_graph_identity.cognitive_tile_id,
                    self.cognitive_graph_identity.tile_revision,
                    graph.graph_id,
                    graph.revision,
                    self.cognitive_graph_identity.model_id,
                )
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
                self.pending_structural_intent = None
                self.graph_reassert_required = False
                self.graph_coherent = True
                self._clear_graph_retry_locked()
            else:
                if succeeded:
                    self._schedule_graph_retry_locked(
                        "stale structural rebuild success", kind="switch"
                    )
                else:
                    current_failure = bool(
                        transaction_matches
                        and generation_current
                        and requested_identity == desired_identity
                    )
                    if current_failure:
                        self._schedule_graph_retry_locked(
                            "Route Server rejected rebuilt graph",
                            kind="structural",
                        )
        if not succeeded:
            if not current_failure:
                self._try_deferred_structural_rebuild()
                return
            self._publish_structural_status(
                self.StructuralGraphStatus.LAST_KNOWN_GOOD,
                "Route Server rejected rebuilt graph",
            )
            self._try_deferred_structural_rebuild()
            return
        if not commit:
            self._publish_structural_status(
                self.StructuralGraphStatus.LAST_KNOWN_GOOD,
                "stale structural rebuild success; reconciling desired graph",
            )
            self._try_deferred_structural_rebuild()
            return
        self._publish_graph()
        self._publish_cognitive_constraints()
        self._publish_structural_status(
            self.StructuralGraphStatus.READY, "rebuilt graph active"
        )
        self._try_deferred_structural_rebuild()

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
