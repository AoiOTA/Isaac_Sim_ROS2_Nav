"""Automatic fixed-scene Module1 targeted teaching capture.

The targeted-teaching schema is intentionally separate from the exact five-leg
Phase-B manifest.  It reuses the Phase-B reset, route dispatch, collision, and
terminal-zero runtime, while allowing a bounded list of unique XY teaching
goals.  Ground Truth remains a passive recorder/evaluator stream and is never
used to dispatch a goal.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Mapping

import yaml

from robot_experiments.v6_formal import (
    ENGINEERING_PILOT,
    Episode,
    Manifest,
    MissionLeg,
    NOT_QUALIFIED,
    V6ContractError,
    V6FormalNode,
    _mapping,
    _pose,
    _require_exact_keys,
    _xy_goal,
)


SCHEMA_VERSION = "bio_nav_module1_targeted_teaching_manifest_v1"
MODE = "module1_targeted_teaching"
SCENE_ID = "kujiale_0026_A_to_B_door_open"
SCENE_REVISION = "original_usd_20260717"
ORIGINAL_USD = (
    "/home/lyb/kujiale_usd_rooms_20260717/kujiale_0026/"
    "kujiale_0026_A_to_B_door_open.usd"
)
STREAM_DROPOUT_TIMEOUT_SEC = 5.0

TARGETED_RUNTIME = {
    "odometry_mode": "mixed",
    "canonical_odom": {
        "topic": "/odom",
        "owner": "isaac_compute_odometry",
        "tf": "odom->base_link",
    },
    "global_localization": {
        "pose_topic": "/amcl_pose",
        "owner": "amcl",
        "tf": "map->odom",
    },
    "module1_odom": {
        "topic": "/bio_nav/module1/odom",
        "owner": "wheel_imu_ekf",
        "publish_tf": False,
    },
    "recovery_enabled": False,
    "module2_enabled": False,
    "module2_navigation_write_enabled": False,
    "module2_effect_scope": "off",
    "cognitive_place_graph_enabled": False,
    "route_backend": "gvg",
    "low_obstacles_enabled": False,
    "dynamic_actors_enabled": False,
    "goal_checker": "position_xy",
    "cognitive_profile": "M0",
    "module1_mode": "teaching_capture",
    "ground_truth_use": "evaluator_only",
}

TARGETED_ASSET_KEYS = {
    "scene_asset",
    "occupancy_map",
    "spawn_manifest",
    "route_graph",
    "navigation_config",
    "module1_scene_config",
}

REQUIRED_CAPTURE_STREAMS = (
    "/clock",
    "/camera/front/image_raw",
    "/camera/front/camera_info",
    "/ground_truth/odom",
    "/bio_nav/module1/odom",
    "/scan",
)


@dataclass(frozen=True)
class TargetedTeachingManifest:
    path: Path
    raw: Mapping[str, Any]
    route_id: str
    dataset: Mapping[str, str]
    runtime: Mapping[str, Any]
    assets: Mapping[str, str]
    valid_state_ids: frozenset[int]
    mission_legs: tuple[MissionLeg, ...]
    best_effort_leg_ids: frozenset[str]
    episode: Episode
    formal_manifest: Manifest


def state_id_for_map_xy(x: float, y: float) -> int:
    """Map and 16x16 canvas are identical for this fixed scene."""

    column = math.floor(float(x) + 8.0)
    row = math.floor(float(y) + 8.0)
    if not (0 <= row < 16 and 0 <= column < 16):
        raise V6ContractError(f"teaching goal ({x}, {y}) is outside the 16x16 canvas")
    return int(row * 16 + column)


def _validate_runtime(raw: Any) -> Mapping[str, Any]:
    runtime = _mapping(raw, "runtime")
    _require_exact_keys(runtime, set(TARGETED_RUNTIME), "runtime")
    for name, expected in TARGETED_RUNTIME.items():
        value = runtime.get(name)
        if isinstance(expected, Mapping):
            value = _mapping(value, f"runtime.{name}")
            _require_exact_keys(value, set(expected), f"runtime.{name}")
        if value != expected:
            raise V6ContractError(f"runtime.{name} must be {expected!r}")
    return runtime


def _load_scene_mask(path: str) -> frozenset[int]:
    config_path = Path(path).expanduser().resolve()
    try:
        document = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise V6ContractError(
            f"assets.module1_scene_config is unreadable: {config_path}"
        ) from exc
    document = _mapping(document, "module1_scene_config")
    scene = document.get("scene", document)
    scene = _mapping(scene, "module1_scene_config.scene")
    if scene.get("scene_id") != SCENE_ID:
        raise V6ContractError("Module1 scene config has the wrong scene_id")
    if scene.get("scene_revision") != SCENE_REVISION:
        raise V6ContractError("Module1 scene config is not the original USD revision")
    if scene.get("map_id") != "v6_kujiale_isaacgen_v1":
        raise V6ContractError("Module1 scene config has the wrong map_id")
    if scene.get("T_map_canvas") != [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]:
        raise V6ContractError("Module1 scene config T_map_canvas must be identity")
    state_mask = scene.get("valid_state_mask")
    state_ids = scene.get("valid_state_ids")
    if (
        not isinstance(state_mask, list)
        or len(state_mask) != 256
        or any(not isinstance(value, bool) for value in state_mask)
        or sum(state_mask) != 51
    ):
        raise V6ContractError("Module1 scene config must contain the 51/256 mask")
    expected_ids = [index for index, enabled in enumerate(state_mask) if enabled]
    if state_ids != expected_ids:
        raise V6ContractError("Module1 scene config state IDs disagree with its mask")
    return frozenset(expected_ids)


def _validate_assets(raw: Any) -> tuple[Mapping[str, str], frozenset[int]]:
    assets = _mapping(raw, "assets")
    _require_exact_keys(assets, TARGETED_ASSET_KEYS, "assets")
    for name, value in assets.items():
        if not isinstance(value, str) or not value:
            raise V6ContractError(f"assets.{name} must be a non-empty path")
    if assets["scene_asset"] != ORIGINAL_USD:
        raise V6ContractError("assets.scene_asset must be the original Kujiale USD")
    expected_suffixes = {
        "occupancy_map": "/data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml",
        "spawn_manifest": (
            "/isaac_sim/configs/environments/"
            "kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml"
        ),
        "route_graph": (
            "/ros2_ws/src/robot_route_planner/config/"
            "v6_kujiale_isaacgen_v1_gvg_v1.geojson"
        ),
        "navigation_config": "/ros2_ws/src/robot_navigation/config/nav2_params.yaml",
        "module1_scene_config": "/configs/kujiale_0026_module1_enrollment_v310.yaml",
    }
    for name, suffix in expected_suffixes.items():
        if not str(assets[name]).endswith(suffix):
            raise V6ContractError(f"assets.{name} is not the accepted fixed-scene asset")
    normalized = {name: str(value) for name, value in assets.items()}
    return normalized, _load_scene_mask(normalized["module1_scene_config"])


def _load_episode(raw: Any) -> Episode:
    rows = raw
    if not isinstance(rows, list) or len(rows) != 1:
        raise V6ContractError(
            "episodes must contain exactly one row so each reset has a separate bag"
        )
    row = _mapping(rows[0], "episodes[0]")
    _require_exact_keys(
        row,
        {"seed", "variant_id", "reset_pose_name", "dynamic_case_id"},
        "episodes[0]",
    )
    seed = row.get("seed")
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise V6ContractError("episodes[0].seed must be a non-negative int")
    if row.get("variant_id") != "baseline":
        raise V6ContractError("episodes[0].variant_id must be baseline")
    if row.get("reset_pose_name") != "long_route_start_g1":
        raise V6ContractError("episodes[0].reset_pose_name must be long_route_start_g1")
    if row.get("dynamic_case_id") != "static":
        raise V6ContractError("episodes[0].dynamic_case_id must be static")
    return Episode(
        seed=seed,
        variant_id="baseline",
        appearance_profile_id=None,
        reset_pose_name="long_route_start_g1",
        dynamic_case_id="static",
    )


def load_targeted_teaching_manifest(
    path: str | Path,
) -> TargetedTeachingManifest:
    manifest_path = Path(path).expanduser().resolve()
    raw = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    raw = _mapping(raw, "manifest")
    if raw.get("schema_version") != SCHEMA_VERSION:
        raise V6ContractError(f"schema_version must be {SCHEMA_VERSION}")
    _require_exact_keys(
        raw,
        {
            "schema_version",
            "mode",
            "intended_use",
            "scene",
            "runtime",
            "assets",
            "dataset",
            "mission",
            "episodes",
        },
        "manifest",
    )
    if raw.get("mode") != MODE:
        raise V6ContractError(f"mode must be {MODE}")
    if raw.get("intended_use") != "raw_teaching_capture":
        raise V6ContractError("intended_use must be raw_teaching_capture")

    scene = _mapping(raw.get("scene"), "scene")
    _require_exact_keys(scene, {"id", "revision", "world", "category"}, "scene")
    if scene != {
        "id": SCENE_ID,
        "revision": SCENE_REVISION,
        "world": "kujiale",
        "category": "static",
    }:
        raise V6ContractError("scene must bind the original fixed Kujiale USD")

    runtime = _validate_runtime(raw.get("runtime"))
    assets, valid_state_ids = _validate_assets(raw.get("assets"))

    dataset = _mapping(raw.get("dataset"), "dataset")
    _require_exact_keys(dataset, {"route_id", "role", "split", "status"}, "dataset")
    route_id = str(dataset.get("route_id", ""))
    if route_id not in {"EN", "SW"}:
        raise V6ContractError("dataset.route_id must be EN or SW")
    if dataset.get("role") != "train" or dataset.get("split") != "A_base":
        raise V6ContractError("dataset must be role=train and split=A_base")
    if dataset.get("status") != "raw_until_audit":
        raise V6ContractError("dataset.status must be raw_until_audit")

    mission = _mapping(raw.get("mission"), "mission")
    _require_exact_keys(
        mission, {"reset_pose", "legs", "best_effort_leg_ids"}, "mission"
    )
    reset_pose = _mapping(mission.get("reset_pose"), "mission.reset_pose")
    _require_exact_keys(
        reset_pose, {"id", "frame_id", "x", "y", "yaw_deg"}, "mission.reset_pose"
    )
    _, reset_x, reset_y, reset_yaw = _pose(reset_pose, "mission.reset_pose")
    if (
        reset_pose.get("id") != "G1"
        or not math.isclose(reset_x, 0.45, abs_tol=1.0e-12)
        or not math.isclose(reset_y, -5.35, abs_tol=1.0e-12)
        or not math.isclose(reset_yaw, 90.0, abs_tol=1.0e-12)
    ):
        raise V6ContractError("mission.reset_pose must be the accepted G1 pose")

    rows = mission.get("legs")
    if not isinstance(rows, list) or not rows:
        raise V6ContractError("mission.legs must contain at least one XY goal")
    legs: list[MissionLeg] = []
    seen_ids: set[str] = set()
    previous_xy = (reset_x, reset_y)
    for index, value in enumerate(rows):
        row = _mapping(value, f"mission.legs[{index}]")
        goal_id = str(row.get("id", ""))
        if not goal_id or goal_id in seen_ids:
            raise V6ContractError(
                f"mission.legs[{index}].id must be unique and non-empty"
            )
        frame_id, x, y = _xy_goal(row, f"mission.legs[{index}]")
        if math.hypot(x - previous_xy[0], y - previous_xy[1]) <= 1.0e-6:
            raise V6ContractError(f"mission.legs[{index}] is a zero-distance goal")
        state_id = state_id_for_map_xy(x, y)
        if state_id not in valid_state_ids:
            raise V6ContractError(
                f"mission.legs[{index}] maps to invalid state {state_id}"
            )
        legs.append(MissionLeg(goal_id, frame_id, x, y))
        seen_ids.add(goal_id)
        previous_xy = (x, y)

    best_effort_rows = mission.get("best_effort_leg_ids")
    if not isinstance(best_effort_rows, list) or any(
        not isinstance(value, str) for value in best_effort_rows
    ):
        raise V6ContractError("mission.best_effort_leg_ids must be a string list")
    best_effort = frozenset(best_effort_rows)
    if len(best_effort) != len(best_effort_rows) or not best_effort <= seen_ids:
        raise V6ContractError(
            "mission.best_effort_leg_ids must be unique mission leg IDs"
        )

    episode = _load_episode(raw.get("episodes"))
    formal = Manifest(
        path=manifest_path,
        raw=raw,
        scene_id=SCENE_ID,
        category="static",
        runtime=runtime,
        assets=assets,
        reset_pose=reset_pose,
        mission_legs=tuple(legs),
        dynamic_schedule=(),
        episodes=(episode,),
    )
    return TargetedTeachingManifest(
        path=manifest_path,
        raw=raw,
        route_id=route_id,
        dataset={name: str(value) for name, value in dataset.items()},
        runtime=runtime,
        assets=assets,
        valid_state_ids=valid_state_ids,
        mission_legs=tuple(legs),
        best_effort_leg_ids=best_effort,
        episode=episode,
        formal_manifest=formal,
    )


def stream_dropout_reason(
    last_seen: Mapping[str, float | None],
    *,
    now: float,
    timeout_sec: float = STREAM_DROPOUT_TIMEOUT_SEC,
) -> str:
    for topic in REQUIRED_CAPTURE_STREAMS:
        stamp = last_seen.get(topic)
        if stamp is None:
            return f"stream_missing:{topic}"
        if now - float(stamp) > timeout_sec:
            return f"stream_dropout:{topic}"
    return ""


# V6FormalNode is the current Phase-B pilot runtime.  Keep the descriptive
# alias local so targeted teaching clearly reuses that runtime rather than
# growing a second reset/navigation implementation.
V6FormalPilotRuntime = V6FormalNode


class Module1TargetedTeachingNode(V6FormalPilotRuntime):
    """Phase-B runtime plus passive capture health and one best-effort leg."""

    def __init__(
        self,
        manifest: TargetedTeachingManifest,
        output_jsonl: Path,
    ) -> None:
        super().__init__(
            manifest.formal_manifest,
            manifest.episode,
            output_jsonl,
            qualification=ENGINEERING_PILOT,
        )
        from bio_nav_interfaces.msg import PlanningPrior
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import CameraInfo, Image, LaserScan

        self.targeted_manifest = manifest
        self.best_effort_not_covered: list[str] = []
        self.planning_prior_messages = 0
        self.trusted_write_count = 0
        self._last_clock_stamp_ns: int | None = None
        self._stream_last_seen: dict[str, float | None] = {
            topic: None for topic in REQUIRED_CAPTURE_STREAMS
        }
        sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        # These subscriptions are passive presence/freshness checks only.
        # GT pose fields are never read and never enter the dispatch manifest.
        self.passive_evaluator_subscriptions = [
            self.node.create_subscription(
                Clock, "/clock", self._teaching_clock, sensor
            ),
            self.node.create_subscription(
                Image,
                "/camera/front/image_raw",
                self._stream_callback("/camera/front/image_raw"),
                sensor,
            ),
            self.node.create_subscription(
                CameraInfo,
                "/camera/front/camera_info",
                self._stream_callback("/camera/front/camera_info"),
                sensor,
            ),
            self.node.create_subscription(
                Odometry,
                "/ground_truth/odom",
                self._stream_callback("/ground_truth/odom"),
                sensor,
            ),
            self.node.create_subscription(
                Odometry,
                "/bio_nav/module1/odom",
                self._stream_callback("/bio_nav/module1/odom"),
                sensor,
            ),
            self.node.create_subscription(
                LaserScan, "/scan", self._stream_callback("/scan"), sensor
            ),
        ]
        self.isolation_subscriptions = [
            self.node.create_subscription(
                PlanningPrior,
                "/bio_nav/module2/planning_prior",
                self._planning_prior,
                sensor,
            )
        ]
        self.stream_health_timer = self.node.create_timer(
            0.5, self._check_capture_streams
        )

    def _stream_callback(self, topic: str):
        def callback(_message: Any) -> None:
            self._stream_last_seen[topic] = time.monotonic()

        return callback

    def _teaching_clock(self, message: Any) -> None:
        stamp_ns = int(message.clock.sec) * 1_000_000_000 + int(
            message.clock.nanosec
        )
        if (
            self._last_clock_stamp_ns is not None
            and stamp_ns < self._last_clock_stamp_ns
        ):
            self.guard.stop("clock_moved_backward")
            self._cancel_active_navigation_once("clock_moved_backward")
        self._last_clock_stamp_ns = stamp_ns
        self._stream_last_seen["/clock"] = time.monotonic()

    def _planning_prior(self, message: Any) -> None:
        self.planning_prior_messages += 1
        if bool(message.trusted_write):
            self.trusted_write_count += 1
            self.guard.stop("trusted_module2_write_forbidden")
            self._cancel_active_navigation_once("trusted_module2_write_forbidden")

    def _readiness_blockers(self) -> str:
        blockers = super()._readiness_blockers()
        capture = stream_dropout_reason(
            self._stream_last_seen, now=time.monotonic()
        )
        return ";".join(value for value in (blockers, capture) if value)

    def _pre_reset_ready(self) -> bool:
        ready = super()._pre_reset_ready()
        if stream_dropout_reason(self._stream_last_seen, now=time.monotonic()):
            self.pre_reset_quiet_since = None
            return False
        return ready

    def _check_capture_streams(self) -> None:
        if self.guard.state != "NAVIGATING":
            return
        reason = stream_dropout_reason(
            self._stream_last_seen, now=time.monotonic()
        )
        if reason:
            self.guard.stop(reason)
            self._cancel_active_navigation_once(reason)

    def _current_leg_id(self) -> str:
        index = self.guard.goal_publications - 1
        if 0 <= index < len(self.guard.mission_leg_ids):
            return self.guard.mission_leg_ids[index]
        return ""

    def _route_complete(self, message: Any) -> None:
        leg_id = self._current_leg_id()
        safe_best_effort_failure = bool(
            not bool(message.data)
            and leg_id in self.targeted_manifest.best_effort_leg_ids
            and self.guard.state == "NAVIGATING"
            and not self.guard.stop_reason
            and not self.collision
            and self.guard.reset_events == 1
            and self.trusted_write_count == 0
        )
        if not safe_best_effort_failure:
            super()._route_complete(message)
            return

        self._track_route_signal("route_goal_complete")
        self._navigation_terminal_observed = True
        self.guard.route_completion_messages += 1
        self.guard.route_succeeded = False
        final_leg = self.guard.goal_publications >= len(self.guard.mission_leg_ids)
        self.guard.state = "SUCCEEDED" if final_leg else "LEG_SUCCEEDED"
        self.best_effort_not_covered.append(leg_id)
        self._write(
            "BEST_EFFORT_NOT_COVERED",
            leg_id=leg_id,
            attempt_count=1,
            continuation="safe",
        )
        self._capture("/bio_nav/route_goal_complete", message)

    def result(self) -> dict[str, Any]:
        row = super().result()
        row.update(
            {
                "mode": MODE,
                "route_id": self.targeted_manifest.route_id,
                "dataset": dict(self.targeted_manifest.dataset),
                "best_effort_not_covered": list(self.best_effort_not_covered),
                "planning_prior_message_count": self.planning_prior_messages,
                "trusted_write_count": self.trusted_write_count,
            }
        )
        self._write("targeted_teaching_result", **row)
        return row


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--dispatch", action="store_true")
    parser.add_argument("--output-jsonl")
    parser.add_argument("--readiness-timeout-sec", type=float, default=120.0)
    parser.add_argument("--reset-timeout-sec", type=float, default=120.0)
    parser.add_argument("--navigation-timeout-sec", type=float, default=900.0)
    return parser


def cli(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.validate_only == args.dispatch:
            raise V6ContractError("select exactly one of --validate-only or --dispatch")
        manifest = load_targeted_teaching_manifest(args.manifest)
        if args.validate_only:
            print(
                json.dumps(
                    {
                        "mode": MODE,
                        "qualification": ENGINEERING_PILOT,
                        "formal_qualification": NOT_QUALIFIED,
                        "route_id": manifest.route_id,
                        "dataset": dict(manifest.dataset),
                        "leg_count": len(manifest.mission_legs),
                        "best_effort_leg_ids": sorted(
                            manifest.best_effort_leg_ids
                        ),
                        "dispatch": False,
                    },
                    sort_keys=True,
                )
            )
            return 0
        if args.output_jsonl is None:
            raise V6ContractError("--dispatch requires --output-jsonl")
        import rclpy

        rclpy.init(args=None)
        adapter = Module1TargetedTeachingNode(
            manifest, Path(args.output_jsonl).expanduser().resolve()
        )
        try:
            result = adapter.run(
                readiness_timeout_sec=args.readiness_timeout_sec,
                reset_timeout_sec=args.reset_timeout_sec,
                navigation_timeout_sec=args.navigation_timeout_sec,
            )
            print(json.dumps(result, sort_keys=True))
            passed = bool(
                result["state"] == "SUCCEEDED"
                and result["reset_calls"] == 1
                and result["reset_events"] == 1
                and not result["collision"]
                and result["terminal_zero_confirmed"]
                and result["trusted_write_count"] == 0
            )
            return 0 if passed else 2
        finally:
            adapter.destroy()
            rclpy.shutdown()
    except (OSError, V6ContractError, yaml.YAMLError) as exc:
        print(f"STOP: {exc}", file=sys.stderr)
        return 2


def main() -> None:
    raise SystemExit(cli())


if __name__ == "__main__":
    main()


__all__ = [
    "MODE",
    "Module1TargetedTeachingNode",
    "REQUIRED_CAPTURE_STREAMS",
    "SCHEMA_VERSION",
    "STREAM_DROPOUT_TIMEOUT_SEC",
    "TARGETED_RUNTIME",
    "TargetedTeachingManifest",
    "V6FormalPilotRuntime",
    "load_targeted_teaching_manifest",
    "state_id_for_map_xy",
    "stream_dropout_reason",
]
