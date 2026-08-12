from pathlib import Path

from robot_experiments.attempt30_a21_qualification import (
    APPEARANCE_PROFILES,
    EXPECTED,
    WHOLE_HOUSE_ROUTE,
    _route_projection_tracks,
    _route_valid,
    aggregate,
    write_outputs,
)
from robot_experiments.scenario import (
    load_scenario,
    validate_navigation_runner_scenario,
)
from robot_experiments.v310_evidence import _real_alternatives


CONFIG = Path(__file__).resolve().parents[1] / "config"


def test_exact_qualification_manifests_use_whole_house_route() -> None:
    scenarios = {
        group: load_scenario(CONFIG / f"{scenario_id}.yaml")
        for group, scenario_id in EXPECTED.items()
    }
    for scenario in scenarios.values():
        validate_navigation_runner_scenario(scenario)
        assert len(scenario.run_matrix) == 20
        assert scenario.spawn_pose_name == "long_route_start_g1"
        assert tuple(goal.goal_id for goal in scenario.route) == WHOLE_HOUSE_ROUTE
    assert len(scenarios["static"].obstacles["static"]) == 6
    assert len(scenarios["appearance"].obstacles["static"]) == 6
    assert scenarios["dynamic"].obstacles["static"] == []
    assert sorted(
        selection.appearance_profile_id
        for selection in scenarios["appearance"].run_matrix
    ) == sorted(APPEARANCE_PROFILES * 5)
    selected = {
        item["motion"]
        for item in scenarios["dynamic"].obstacle_trajectories
        if item["motion"] in {
            "local_bypass", "g2_g3_exit", "g5_g1_crossing"
        }
    }
    assert selected == {"local_bypass", "g2_g3_exit", "g5_g1_crossing"}
    assert {
        selection.variant_id for selection in scenarios["dynamic"].run_matrix
    } == {"v1", "v2", "v3", "v4", "v5"}


def _row(group: str, index: int) -> dict:
    profile = APPEARANCE_PROFILES[(index - 1) % 4] if group == "appearance" else None
    return {
        "task_success": True,
        "collision_free": True,
        "collision_free_success": True,
        "data_complete": True,
        "checksums_verified": True,
        "route_valid": True,
        "executed_deviation_percent": 10.0 if group == "static" else None,
        "planned_deviation_percent": 5.0 if group == "static" else None,
        "execution_time_sec": 100.0,
        "minimum_clearance_m": 0.2,
        "unexpected_abort": False,
        "dynamic_interaction_complete": group == "dynamic",
        "profile": profile,
        "localization_healthy": True,
        "module2_health": {"response_count": 5, "healthy_count": 5},
    }


def test_aggregate_applies_exact_19_and_18_of_20_thresholds() -> None:
    records = {
        group: [_row(group, index) for index in range(1, 21)]
        for group in EXPECTED
    }
    report = aggregate(records)
    assert report["passed"] is True
    assert report["total_run_count"] == 60

    records["static"][0]["collision_free"] = False
    records["static"][0]["collision_free_success"] = False
    assert aggregate(records)["groups"]["static"]["passed"] is True
    records["static"][1]["collision_free"] = False
    records["static"][1]["collision_free_success"] = False
    assert aggregate(records)["groups"]["static"]["passed"] is False

    records["static"] = [_row("static", index) for index in range(1, 21)]
    records["dynamic"][0]["task_success"] = False
    records["dynamic"][0]["dynamic_interaction_complete"] = False
    records["dynamic"][1]["task_success"] = False
    records["dynamic"][1]["dynamic_interaction_complete"] = False
    assert aggregate(records)["groups"]["dynamic"]["passed"] is True
    records["dynamic"][2]["task_success"] = False
    records["dynamic"][2]["dynamic_interaction_complete"] = False
    assert aggregate(records)["groups"]["dynamic"]["passed"] is False


def test_dynamic_success_still_requires_complete_actor_interactions() -> None:
    records = {
        group: [_row(group, index) for index in range(1, 21)]
        for group in EXPECTED
    }
    records["dynamic"][0]["dynamic_interaction_complete"] = False
    result = aggregate(records)["groups"]["dynamic"]
    assert result["successful_dynamic_interactions_complete"] is False
    assert result["passed"] is False


def test_route_valid_accepts_authorized_attempted_prefix_on_safe_abort() -> None:
    routes = [
        {"request_id": 41, "edge_ids": [1, 2]},
        {"request_id": 42, "edge_ids": [3, 4]},
    ]
    manifest = {
        "navigation_execution_backend": "route_guided",
        "route_poses": [{"id": item} for item in WHOLE_HOUSE_ROUTE],
        "legs": [
            {"id": "G2", "route_request_id": 41},
            {"id": "G3", "route_request_id": 42},
        ],
        "canonical_routes": routes,
    }
    assert _route_valid(manifest) is True
    manifest["legs"][1]["id"] = "G4"
    assert _route_valid(manifest) is False


def test_static_executed_deviation_is_a_separate_20_percent_gate() -> None:
    records = {
        group: [_row(group, index) for index in range(1, 21)]
        for group in EXPECTED
    }
    records["static"][7]["executed_deviation_percent"] = 20.01
    result = aggregate(records)["groups"]["static"]
    assert result["collision_free_success_count"] == 20
    assert result["executed_deviation_percent"]["passed"] is False
    assert result["passed"] is False


def test_route_projection_tracks_do_not_join_separate_requests() -> None:
    manifest = {
        "route_progress": [
            {"request_id": 41, "projected_point": [0.0, 0.0]},
            {"request_id": 41, "projected_point": [1.0, 0.0]},
            {"request_id": 42, "projected_point": [9.0, 9.0]},
            {"request_id": 42, "projected_point": [9.0, 10.0]},
        ]
    }
    assert _route_projection_tracks(manifest) == [
        (41, [[0.0, 0.0], [1.0, 0.0]]),
        (42, [[9.0, 9.0], [9.0, 10.0]]),
    ]


def test_qualification_csv_uses_lf_line_endings(tmp_path, monkeypatch) -> None:
    records = {
        group: [_row(group, index) for index in range(1, 21)]
        for group in EXPECTED
    }
    monkeypatch.setattr(
        "robot_experiments.attempt30_a21_qualification.write_visuals",
        lambda _records, _output: [],
    )
    monkeypatch.setattr(
        "robot_experiments.attempt30_a21_qualification.write_v310_evidence",
        lambda _records, _output: [],
    )
    write_outputs(records, aggregate(records), tmp_path)
    payload = (tmp_path / "runs.csv").read_bytes()
    assert b"\r\n" not in payload
    assert payload.count(b"\n") == 61


def test_v310_guidance_alternatives_use_only_recorded_graph_edges() -> None:
    manifest = {
        "navigation_graph": {
            "edges": [
                {"id": 1, "from_node": 1, "to_node": 2, "length_m": 1.0},
                {"id": 2, "from_node": 2, "to_node": 4, "length_m": 1.0},
                {"id": 3, "from_node": 1, "to_node": 3, "length_m": 1.2},
                {"id": 4, "from_node": 3, "to_node": 4, "length_m": 1.2},
            ]
        }
    }
    alternatives = _real_alternatives(
        manifest, 1, 4, {1: 1.0, 2: 1.0, 3: 1.2, 4: 1.2}
    )
    assert alternatives == [(2.0, (1, 2)), (2.4, (3, 4))]
