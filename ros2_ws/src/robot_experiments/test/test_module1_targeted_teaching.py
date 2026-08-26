import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from robot_experiments.module1_targeted_teaching import (
    MODE,
    Module1TargetedTeachingNode,
    REQUIRED_CAPTURE_STREAMS,
    STREAM_DROPOUT_TIMEOUT_SEC,
    V6FormalPilotRuntime,
    cli,
    load_targeted_teaching_manifest,
    state_id_for_map_xy,
    stream_dropout_reason,
)
from robot_experiments.v6_formal import (
    EpisodeGuard,
    ReadinessFacts,
    V6ContractError,
    V6FormalNode,
    load_manifest,
)
from robot_route_planner.feasibility import classify_edge
from robot_route_planner.map_io import load_occupancy_map
from robot_route_planner.models import Traversability
from robot_route_planner.ros_node import select_support_attachment


PACKAGE = Path(__file__).resolve().parents[1]
REPO = Path(__file__).resolve().parents[4]
CONFIG = PACKAGE / "config"
EN = CONFIG / "module1_targeted_teaching_kujiale_en.yaml"
SW = CONFIG / "module1_targeted_teaching_kujiale_sw.yaml"
PHASE_B = CONFIG / "v6_r5_phase_b_kujiale_exact_baseline.yaml"


def _write(tmp_path: Path, document: dict) -> Path:
    path = tmp_path / "manifest.yaml"
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return path


def _ready_guard(*legs: str) -> EpisodeGuard:
    facts = ReadinessFacts(
        **{name: True for name in ReadinessFacts.__dataclass_fields__}
    )
    guard = EpisodeGuard(mission_leg_ids=legs)
    guard.arm_reset(facts)
    guard.record_reset_call()
    guard.record_reset_response(True)
    guard.record_reset_receipt_generation(1)
    guard.record_reset_event()
    guard.record_initialpose(100)
    guard.record_amcl(101)
    guard.record_navigation_ready(nav2_active=True, tf_active=True)
    guard.record_reset_gate_status(1, False)
    return guard


def test_targeted_manifests_bind_raw_training_role_and_effect_off_runtime():
    en = load_targeted_teaching_manifest(EN)
    sw = load_targeted_teaching_manifest(SW)

    assert en.route_id == "EN" and sw.route_id == "SW"
    assert en.dataset == {
        "route_id": "EN",
        "role": "train",
        "split": "A_base",
        "status": "raw_until_audit",
    }
    assert sw.dataset["status"] == "raw_until_audit"
    for manifest in (en, sw):
        assert manifest.runtime["odometry_mode"] == "mixed"
        assert manifest.runtime["cognitive_profile"] == "M0"
        assert manifest.runtime["module2_enabled"] is False
        assert manifest.runtime["module2_navigation_write_enabled"] is False
        assert manifest.runtime["module2_effect_scope"] == "off"
        assert manifest.runtime["dynamic_actors_enabled"] is False
        assert manifest.runtime["ground_truth_use"] == "evaluator_only"
        assert manifest.assets["scene_asset"].endswith(
            "/kujiale_0026/kujiale_0026_A_to_B_door_open.usd"
        )
        assert len(manifest.valid_state_ids) == 51
    assert en.best_effort_leg_ids == frozenset({"EN_S184"})
    assert sw.best_effort_leg_ids == frozenset()


def test_exact_routes_map_to_expected_valid_states():
    en = load_targeted_teaching_manifest(EN)
    sw = load_targeted_teaching_manifest(SW)

    assert [state_id_for_map_xy(leg.x, leg.y) for leg in en.mission_legs] == [
        122, 184, 202, 217, 215, 122, 39
    ]
    assert [state_id_for_map_xy(leg.x, leg.y) for leg in sw.mission_legs] == [
        70, 84, 101, 104, 182, 147, 117, 85, 40
    ]
    assert all(
        state_id_for_map_xy(leg.x, leg.y) in manifest.valid_state_ids
        for manifest in (en, sw)
        for leg in manifest.mission_legs
    )


def test_only_targeted_schema_allows_arbitrary_unique_xy_leg_count(tmp_path):
    targeted = yaml.safe_load(EN.read_text(encoding="utf-8"))
    targeted["mission"]["legs"] = targeted["mission"]["legs"][:2]
    targeted["mission"]["best_effort_leg_ids"] = ["EN_S184"]
    assert len(load_targeted_teaching_manifest(_write(tmp_path, targeted)).mission_legs) == 2

    phase_b = yaml.safe_load(PHASE_B.read_text(encoding="utf-8"))
    phase_b["mission"]["legs"] = phase_b["mission"]["legs"][:4]
    with pytest.raises(V6ContractError, match="exactly five rows"):
        load_manifest(_write(tmp_path, phase_b))


def test_targeted_goals_remain_xy_only(tmp_path):
    document = yaml.safe_load(EN.read_text(encoding="utf-8"))
    document["mission"]["legs"][0]["yaw_deg"] = 45.0
    with pytest.raises(V6ContractError, match="only id/frame_id/x/y"):
        load_targeted_teaching_manifest(_write(tmp_path, document))


def test_each_targeted_leg_has_footprint_feasible_gvg_connectors():
    occupancy = load_occupancy_map(
        REPO / "data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml",
        unknown_is_occupied=True,
    )
    graph = json.loads(
        (
            REPO
            / "ros2_ws/src/robot_route_planner/config/"
            "v6_kujiale_isaacgen_v1_gvg_v1.geojson"
        ).read_text(encoding="utf-8")
    )
    support = {
        int(feature["properties"]["id"]): tuple(
            feature["geometry"]["coordinates"]
        )
        for feature in graph["features"]
        if feature["geometry"]["type"] == "Point"
    }
    footprint = {
        "polygon_m": [
            [0.255, 0.21],
            [0.255, -0.21],
            [-0.23, -0.21],
            [-0.23, 0.21],
        ],
        "padding_m": 0.005,
        "padded_inscribed_radius_m": 0.215,
        "sweep_sample_spacing_m": 0.025,
    }

    for manifest in (
        load_targeted_teaching_manifest(EN),
        load_targeted_teaching_manifest(SW),
    ):
        points = [(0.45, -5.35)] + [(leg.x, leg.y) for leg in manifest.mission_legs]
        for start, goal in zip(points, points[1:]):
            start_node = select_support_attachment(
                occupancy, support, start, footprint, departing=True
            )
            goal_node = select_support_attachment(
                occupancy, support, goal, footprint, departing=False
            )
            common = {
                "footprint_polygon_m": np.asarray(footprint["polygon_m"]),
                "footprint_padding_m": footprint["padding_m"],
                "padded_inscribed_radius_m": footprint[
                    "padded_inscribed_radius_m"
                ],
                "sweep_sample_spacing_m": footprint["sweep_sample_spacing_m"],
            }
            assert classify_edge(
                occupancy,
                np.asarray([start, support[start_node]], dtype=np.float64),
                **common,
            ) == Traversability.FEASIBLE
            assert classify_edge(
                occupancy,
                np.asarray([support[goal_node], goal], dtype=np.float64),
                **common,
            ) == Traversability.FEASIBLE


def test_best_effort_184_failure_is_one_safe_noncovered_result():
    adapter = Module1TargetedTeachingNode.__new__(Module1TargetedTeachingNode)
    adapter.targeted_manifest = SimpleNamespace(
        best_effort_leg_ids=frozenset({"EN_S184"})
    )
    adapter.guard = _ready_guard("EN_S184", "EN_S202")
    adapter.guard.record_goal_publication("EN_S184")
    adapter.collision = False
    adapter.trusted_write_count = 0
    adapter.best_effort_not_covered = []
    adapter._navigation_terminal_observed = False
    events = []
    adapter._write = lambda event, **payload: events.append((event, payload))
    adapter._capture = lambda *_args, **_kwargs: None

    adapter._route_complete(SimpleNamespace(data=False))

    assert adapter.guard.state == "LEG_SUCCEEDED"
    assert adapter.guard.goal_publications == 1
    assert adapter.best_effort_not_covered == ["EN_S184"]
    assert events == [
        (
            "BEST_EFFORT_NOT_COVERED",
            {
                "leg_id": "EN_S184",
                "attempt_count": 1,
                "continuation": "safe",
            },
        )
    ]


def test_trusted_write_and_capture_stream_dropout_stop_dispatch():
    adapter = Module1TargetedTeachingNode.__new__(Module1TargetedTeachingNode)
    adapter.guard = _ready_guard("G")
    adapter.guard.record_goal_publication("G")
    adapter.planning_prior_messages = 0
    adapter.trusted_write_count = 0
    cancelled = []
    adapter._cancel_active_navigation_once = cancelled.append
    adapter._planning_prior(SimpleNamespace(trusted_write=True))
    assert adapter.guard.stop_reason == "trusted_module2_write_forbidden"
    assert adapter.trusted_write_count == 1
    assert cancelled == ["trusted_module2_write_forbidden"]

    fresh = {topic: 10.0 for topic in REQUIRED_CAPTURE_STREAMS}
    assert stream_dropout_reason(fresh, now=10.1) == ""
    fresh["/scan"] = 10.0 - STREAM_DROPOUT_TIMEOUT_SEC - 0.1
    assert stream_dropout_reason(fresh, now=10.0) == "stream_dropout:/scan"


def test_clock_rollback_and_non_best_effort_failure_stop():
    clock = Module1TargetedTeachingNode.__new__(Module1TargetedTeachingNode)
    clock.guard = _ready_guard("G")
    clock.guard.record_goal_publication("G")
    clock._last_clock_stamp_ns = 20
    clock._stream_last_seen = {topic: 1.0 for topic in REQUIRED_CAPTURE_STREAMS}
    cancelled = []
    clock._cancel_active_navigation_once = cancelled.append
    clock._teaching_clock(
        SimpleNamespace(clock=SimpleNamespace(sec=0, nanosec=19))
    )
    assert clock.guard.stop_reason == "clock_moved_backward"
    assert cancelled == ["clock_moved_backward"]

    failed = Module1TargetedTeachingNode.__new__(Module1TargetedTeachingNode)
    failed.targeted_manifest = SimpleNamespace(best_effort_leg_ids=frozenset())
    failed.guard = _ready_guard("REQUIRED")
    failed.guard.record_goal_publication("REQUIRED")
    failed.collision = False
    failed.trusted_write_count = 0
    failed._navigation_terminal_observed = False
    failed._terminal_started_monotonic = None
    failed._terminal_zero_reason = "not_required"
    failed._cmd_vel_sim_zero_stamps = []
    failed._terminal_cancel_requested = True
    failed._capture = lambda *_args, **_kwargs: None
    failed._route_complete(SimpleNamespace(data=False))
    assert failed.guard.state == "STOP"
    assert failed.guard.stop_reason == "route_completed_without_progress"


def test_targeted_runtime_reuses_formal_reset_and_terminal_zero_node():
    assert V6FormalPilotRuntime is V6FormalNode
    assert issubclass(Module1TargetedTeachingNode, V6FormalNode)
    assert Module1TargetedTeachingNode._settle_terminal_zero \
        is V6FormalNode._settle_terminal_zero
    assert Module1TargetedTeachingNode.run is V6FormalNode.run


def test_validate_only_reports_mode_without_dispatch(capsys):
    assert cli(["--manifest", str(EN), "--validate-only"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["mode"] == MODE
    assert payload["dispatch"] is False
    assert payload["route_id"] == "EN"
