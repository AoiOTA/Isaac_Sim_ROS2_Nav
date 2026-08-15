import inspect

from robot_experiments.attempt31_rivermark_qualification import (
    _module2_runtime_consumption,
    _rate_group,
    summarize,
)


def _record(seed, arm="medium", profile=None):
    if arm == "off":
        diagnostics = []
        health = {"response_count": 0, "responses": []}
    else:
        sr = 0.2 if arm in {"sr_medium", "medium"} else 0.0
        dr = 0.3 if arm in {"dr_medium", "medium"} else 0.0
        diagnostics = [{"edges": [{"sr_penalty_m": sr, "dr_penalty_m": dr}]}]
        health = {
            "response_count": 1,
            "responses": [{
                "healthy": True,
                "total_cost_delta_m": sr + dr,
                "positive_cost_count": 1,
            }],
        }
    summary = {
        "strict_success": True,
        "physical_collision_free": True,
        "data_complete": True,
        "checksums_verified": True,
        "path_deviation_percent": 4.0,
        "dynamic_interaction_complete": True,
        "appearance_profile_id": profile,
        "module2_health": health,
        "legs": [
            {"id": f"G{index}", "route_edge_ids": [index, index + 1]}
            for index in range(1, 6)
        ],
    }
    return {
        "summary_path": f"/{arm}/{seed}/run_summary.json",
        "summary": summary,
        "manifest": {
            "experiment_arm": arm,
            "dynamic_selection": {
                "case_id": "full_route_four_stage", "variant_id": "v1",
            },
            "srdr_edge_diagnostics": diagnostics,
            "dynamic_interaction": {
                "expected_ids": ["a", "b", "c", "d"],
                "minimum_clearance_m_by_actor": {
                    "a": 0.2, "b": 0.4, "c": 0.6, "d": 0.8,
                },
            },
            "appearance": {
                "ready": True,
                "state": {"applied_counts": {"lights": 2, "material_color_inputs": 3}},
            },
            "planning_prior_samples": [
                {
                    "module2_healthy": True,
                    "cognitive_tile_id": "region_01",
                    "model_id": "module2_srdr_v310_test",
                },
                {
                    "module2_healthy": True,
                    "cognitive_tile_id": "region_02",
                    "model_id": "module2_srdr_v310_test",
                },
            ],
            "canonical_routes": [{"request_id": 1, "edge_ids": [1]}],
            "route_edge_costs": [{
                "request_id": 1,
                "edges": [{
                    "edge_id": 1,
                    "structural_cost_m": 1.0,
                    "runtime_penalty_m": 0.0,
                    "requested_module2_delta_m": 0.5,
                    "applied_module2_delta_m": 0.25,
                    "final_cost_m": 1.25,
                }],
            }],
        },
        "scenario_id": "test",
        "run_index": seed,
        "seed": seed,
    }


def test_rate_groups_enforce_static_dynamic_and_balanced_appearance_contracts():
    records = [_record(seed) for seed in range(20)]
    assert _rate_group(
        records, name="static", required_rate_percent=95.0,
        require_path_deviation=True,
    )["passed"]
    assert _rate_group(
        records, name="dynamic", required_rate_percent=90.0,
        require_dynamic_interaction=True,
    )["passed"]

    profiles = ("dim_warm", "dim_cool", "bright_warm", "bright_cool")
    appearance = [
        _record(seed, profile=profiles[seed % 4]) for seed in range(20)
    ]
    assert _rate_group(
        appearance, name="appearance", required_rate_percent=90.0,
        require_appearance=True,
    )["passed"]


def test_dynamic_gate_rejects_actor_that_never_pairs_with_robot():
    records = [_record(seed) for seed in range(20)]
    for index in (7, 8, 9):
        records[index]["manifest"]["dynamic_interaction"][
            "minimum_clearance_m_by_actor"
        ]["c"] = 3.3895
        records[index]["summary"]["dynamic_interaction_complete"] = False
        records[index]["summary"]["strict_success"] = False

    result = _rate_group(
        records, name="dynamic", required_rate_percent=90.0,
        require_dynamic_interaction=True,
    )

    assert result["gates"]["four_actor_close_interaction"] is False
    assert result["four_actor_close_interaction_runs"] == 17
    assert result["dynamic_interaction_complete_rate_percent"] == 85.0
    assert result["passed"] is False


def test_dynamic_gate_allows_two_pre_registered_failures_at_ninety_percent():
    records = [_record(seed) for seed in range(20)]
    for index in (7, 8):
        records[index]["manifest"]["dynamic_interaction"][
            "minimum_clearance_m_by_actor"
        ]["c"] = 3.3895
        records[index]["summary"]["dynamic_interaction_complete"] = False
        records[index]["summary"]["strict_success"] = False

    result = _rate_group(
        records, name="dynamic", required_rate_percent=90.0,
        require_dynamic_interaction=True,
    )

    assert result["strict_successes"] == 18
    assert result["dynamic_interaction_complete_runs"] == 18
    assert result["four_actor_close_interaction_runs"] == 18
    assert result["passed"] is True


def test_path_deviation_gate_applies_to_static_but_not_dynamic():
    records = [_record(seed) for seed in range(20)]
    records[4]["summary"]["path_deviation_percent"] = 21.0

    static = _rate_group(
        records, name="static", required_rate_percent=95.0,
        require_path_deviation=True,
    )
    dynamic = _rate_group(
        records, name="dynamic", required_rate_percent=90.0,
        require_dynamic_interaction=True,
    )

    assert static["gates"]["static_path_deviation"] is False
    assert static["passed"] is False
    assert "static_path_deviation" not in dynamic["gates"]
    assert dynamic["passed"] is True


def test_rate_gate_rejects_incomplete_five_waypoint_mission():
    records = [_record(seed) for seed in range(20)]
    records[9]["summary"]["legs"] = records[9]["summary"]["legs"][:-1]

    result = _rate_group(
        records, name="static", required_rate_percent=95.0,
        require_path_deviation=True,
    )

    assert result["five_waypoint_runs"] == 19
    assert result["gates"]["five_waypoint_contract"] is False
    assert result["passed"] is False


def test_module2_runtime_consumption_proves_bounded_route_cost_effect():
    records = [_record(seed) for seed in range(20)]

    result = _module2_runtime_consumption(records)

    assert result["runs_with_healthy_prior_samples"] == 20
    assert result["runs_with_region_switch"] == 20
    assert result["runs_with_positive_applied_delta"] == 20
    assert result["runs_with_selected_route_cost_affected"] == 20
    assert result["gates"]["bounded_cost_composition"] is True
    assert result["causal_claim"] is False
    assert result["passed"] is True


def test_module2_runtime_consumption_rejects_unapplied_prior():
    records = [_record(seed) for seed in range(20)]
    edge = records[7]["manifest"]["route_edge_costs"][0]["edges"][0]
    edge["applied_module2_delta_m"] = 0.0
    edge["final_cost_m"] = 1.0

    result = _module2_runtime_consumption(records)

    assert result["runs_with_positive_applied_delta"] == 19
    assert result["gates"]["positive_delta_applied_every_run"] is False
    assert result["gates"]["selected_route_cost_affected_every_run"] is False
    assert result["passed"] is False


def test_rivermark_summary_is_core_three_by_twenty_and_defers_causality_to_v4():
    parameters = inspect.signature(summarize).parameters
    assert set(parameters) == {
        "static_root", "dynamic_root", "appearance_root", "contract_summary",
    }

    source = inspect.getsource(summarize)
    assert '"module2_causality"' in source
    assert '"external_v4"' in source
    assert '"gating_for_rivermark": False' in source
    assert '"module2_runtime_consumption"' in source
    assert "module2_ablation" not in source
