from pathlib import Path

from robot_experiments.attempt30_a21_qualification import (
    APPEARANCE_PROFILES,
    EXPECTED,
    WHOLE_HOUSE_ROUTE,
    aggregate,
)
from robot_experiments.scenario import (
    load_scenario,
    validate_navigation_runner_scenario,
)


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
    records["dynamic"][1]["task_success"] = False
    assert aggregate(records)["groups"]["dynamic"]["passed"] is True
    records["dynamic"][2]["task_success"] = False
    assert aggregate(records)["groups"]["dynamic"]["passed"] is False


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
