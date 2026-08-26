from __future__ import annotations

from pathlib import Path
import subprocess

import pytest

from robot_experiments.v6_phase_g_causal import (
    LOOP_ROUTE_IDS,
    PhaseGConfigError,
    build_timeline,
    causal_contrast_status,
    evaluate_group,
    graph_kind,
    load_config,
    pareto_direction,
    phase_g_manifest,
    result_is_eligible,
    scoring_route_contrast_status,
)


PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[2]
CONFIG = PACKAGE / "config/v6_phase_g_causal.yaml"
WRAPPER = REPO / "scripts/run_v6_phase_g_causal.sh"


def _result(
    arm: str,
    *,
    path: float = 10.0,
    duration: float = 100.0,
    replans: int = 5,
    fallback_count: int = 0,
    obstacle_arm: str = "M3",
    eligible: bool = True,
) -> dict:
    return {
        "schema_version": "bio_nav_v6_phase_g_result_v1",
        "verdict": "RUN_COMPLETE" if eligible else "RUN_FAILED",
        "arm": arm,
        "obstacle_arm": obstacle_arm,
        "pair_identity": {
            "scene_id": "kujiale_0026_A_to_B_door_open",
            "seed": 7311,
            "route_ids": list(LOOP_ROUTE_IDS),
            "loops": ["warmup", "warmup", "scoring"],
            "reset_count": 1,
        },
        "state": "SUCCEEDED" if eligible else "FAILED",
        "reset_calls": 1,
        "reset_events": 1,
        "no_reset_between_loops": True,
        "scoring": {
            "completed": eligible,
            "completed_leg_ids": list(LOOP_ROUTE_IDS) if eligible else ["G2"],
            "route_success": eligible,
            "collision": False,
            "terminal_zero_confirmed": eligible,
            "path_length_m": path,
            "duration_s": duration,
            "replans": replans,
            "fallback_count": fallback_count,
        },
    }


def _group(**overrides: dict) -> dict[str, dict]:
    rows = {name: _result(name) for name in ("G0", "G1", "G2", "G3")}
    rows.update(overrides)
    return rows


def test_config_freezes_modes_route_loops_and_whole_group_fallback() -> None:
    config = load_config(CONFIG)
    assert [config.arms[name].graph_mode for name in config.arms] == [
        "gvg",
        "shadow",
        "hybrid",
        "primary",
    ]
    assert config.route_ids == LOOP_ROUTE_IDS
    assert (config.loops_total, config.warmup_loops, config.scoring_loop) == (3, 2, 3)
    assert config.reset_count == 1
    assert config.no_reset_between_loops is True
    assert (config.default_obstacle_arm, config.fallback_obstacle_arm) == ("M3", "M2")
    assert config.selection_thresholds == {
        "path_length_m": 0.0,
        "duration_s": 0.0,
        "replans": 0.0,
        "fallback_count": 0.0,
    }


def test_timeline_is_two_warmups_then_one_scoring_loop() -> None:
    timeline = build_timeline(LOOP_ROUTE_IDS)
    assert len(timeline) == 15
    assert [item.route_id for item in timeline] == list(LOOP_ROUTE_IDS) * 3
    assert [item.role for item in timeline[:10]] == ["warmup"] * 10
    assert [item.role for item in timeline[10:]] == ["scoring"] * 5
    assert [item.loop_number for item in timeline[::5]] == [1, 2, 3]


def test_phase_g_manifest_repeats_full_house_route_without_new_reset() -> None:
    config = load_config(CONFIG)
    manifest = phase_g_manifest(config, "G3", "M3")
    assert [item.goal_id for item in manifest.mission_legs] == list(LOOP_ROUTE_IDS) * 3
    assert len(manifest.episodes) == 1
    assert manifest.runtime["phase_g_three_loop_protocol"] == "warmup,warmup,scoring"
    assert manifest.runtime["cognitive_graph_mode"] == "primary"
    assert manifest.runtime["route_prior_enabled"] is True
    assert manifest.runtime["cognitive_place_graph_enabled"] is True
    assert manifest.runtime["low_obstacles_enabled"] is True
    assert manifest.runtime["cognitive_profile"] == "M3"


@pytest.mark.parametrize(
    ("graph_id", "kind"),
    [
        ("map:gvg_v1", "gvg"),
        ("map:gvg_v1:hybrid:4", "hybrid"),
        ("cpg-00112233445566778899aabb:primary", "primary"),
        ("", "unknown"),
    ],
)
def test_graph_kind(graph_id: str, kind: str) -> None:
    assert graph_kind(graph_id) == kind


def test_active_arms_require_expected_selected_graph_and_mature_cpg() -> None:
    candidate = {
        "node_count": 2,
        "edge_count": 1,
        "all_edge_ids_nonempty": True,
        "reset_epoch": 1,
        "module2_healthy": True,
        "observation_valid": True,
        "trusted_write": True,
        "rejection_mask": 0,
        "graph_id": "cpg-00112233445566778899aabb",
        "topology_revision": 4,
        "value_sequence": 9,
    }
    hybrid_ack = {
        "accepted": True,
        "reset_epoch": 1,
        "candidate_graph_id": candidate["graph_id"],
        "candidate_topology_revision": 4,
        "candidate_value_sequence": 9,
        "validated_graph_id": "map:gvg_v1:hybrid:4",
    }
    assert causal_contrast_status(
        "G2",
        "map:gvg_v1:hybrid:4",
        [candidate],
        [hybrid_ack],
        reset_epoch=1,
    ) == (True, ())
    valid, reasons = causal_contrast_status(
        "G3", "map:gvg_v1", [candidate], [hybrid_ack], reset_epoch=1
    )
    assert valid is False
    assert "selected_graph_kind:gvg!=primary" in reasons
    valid, reasons = causal_contrast_status(
        "G3",
        "cpg-00112233445566778899aabb:primary",
        [],
        [],
        reset_epoch=1,
    )
    assert valid is False
    assert reasons == ("current_trusted_mature_cognitive_graph_missing",)


def test_active_contrast_rejects_stale_untrusted_or_unacknowledged_candidate() -> None:
    candidate = {
        "node_count": 2,
        "edge_count": 1,
        "all_edge_ids_nonempty": True,
        "reset_epoch": 0,
        "module2_healthy": True,
        "observation_valid": True,
        "trusted_write": True,
        "rejection_mask": 0,
        "graph_id": "cpg-00112233445566778899aabb",
        "topology_revision": 4,
        "value_sequence": 9,
    }
    valid, reasons = causal_contrast_status(
        "G3",
        "cpg-00112233445566778899aabb:primary",
        [candidate],
        [],
        reset_epoch=1,
    )
    assert valid is False
    assert reasons == ("current_trusted_mature_cognitive_graph_missing",)
    candidate["reset_epoch"] = 1
    candidate["trusted_write"] = False
    valid, reasons = causal_contrast_status(
        "G3",
        "cpg-00112233445566778899aabb:primary",
        [candidate],
        [],
        reset_epoch=1,
    )
    assert valid is False
    assert reasons == ("current_trusted_mature_cognitive_graph_missing",)
    candidate["trusted_write"] = True
    valid, reasons = causal_contrast_status(
        "G3",
        "cpg-00112233445566778899aabb:primary",
        [candidate],
        [],
        reset_epoch=1,
    )
    assert valid is False
    assert reasons == ("matching_accepted_validation_missing",)


def test_g0_g1_require_selected_gvg_but_not_active_cpg_maturity() -> None:
    assert causal_contrast_status("G0", "map:gvg_v1", []) == (True, ())
    assert causal_contrast_status("G1", "map:gvg_v1", []) == (True, ())


def test_scoring_routes_must_keep_the_arm_selected_graph_kind() -> None:
    routes = [{"graph_id": "map:gvg_v1:hybrid:4"} for _ in LOOP_ROUTE_IDS]
    assert scoring_route_contrast_status("G2", routes) == (True, ())
    routes[-1] = {"graph_id": "map:gvg_v1"}
    valid, reasons = scoring_route_contrast_status("G2", routes)
    assert valid is False
    assert reasons == ("scoring_canonical_route_graph_kind:gvg!=hybrid",)


def test_result_eligibility_enforces_single_reset_three_loop_score_and_zero() -> None:
    row = _result("G0")
    assert result_is_eligible(row) is True
    row["reset_events"] = 2
    assert result_is_eligible(row) is False
    row["reset_events"] = 1
    row["scoring"]["terminal_zero_confirmed"] = False
    assert result_is_eligible(row) is False


def test_primary_selected_only_on_strict_raw_pareto_improvement() -> None:
    rows = _group(
        G2=_result("G2", path=9.5, duration=99.0, replans=5),
        G3=_result("G3", path=9.0, duration=95.0, replans=4),
    )
    summary = evaluate_group(rows)
    assert summary["verdict"] == "PRIMARY_CANDIDATE"
    assert summary["selected_graph_mode"] == "PRIMARY"
    assert summary["comparisons"]["G3_vs_G0"]["pareto_improves"] is True


def test_hybrid_selected_when_primary_does_not_dominate_but_hybrid_does() -> None:
    rows = _group(
        G2=_result("G2", path=9.5, duration=99.0, replans=5),
        G3=_result("G3", path=9.0, duration=101.0, replans=4),
    )
    summary = evaluate_group(rows)
    assert summary["verdict"] == "HYBRID_CANDIDATE"
    assert summary["selected_graph_mode"] == "HYBRID"


def test_mixed_tradeoff_is_ambiguous_and_keeps_gvg() -> None:
    rows = _group(
        G2=_result("G2", path=9.0, duration=101.0),
        G3=_result("G3", path=11.0, duration=99.0),
    )
    summary = evaluate_group(rows)
    assert summary["verdict"] == "AMBIGUOUS_KEEP_GVG"
    assert summary["selected_graph_mode"] == "GVG"


def test_equal_candidate_is_not_an_improvement() -> None:
    direction = pareto_direction(_result("G3"), _result("G0"))
    assert direction["comparable"] is True
    assert direction["pareto_improves"] is False
    summary = evaluate_group(_group())
    assert summary["verdict"] == "GVG_RETAINED"


def test_configured_threshold_can_keep_gvg_without_material_net_benefit() -> None:
    rows = _group(G3=_result("G3", path=9.5, duration=99.0, replans=5))
    thresholds = {
        "path_length_m": 1.0,
        "duration_s": 2.0,
        "replans": 1.0,
        "fallback_count": 1.0,
    }
    summary = evaluate_group(rows, thresholds)
    assert summary["comparisons"]["G3_vs_G0"]["pareto_improves"] is True
    assert summary["comparisons"]["G3_vs_G0"]["net_benefit"] is False
    assert summary["selected_graph_mode"] == "GVG"


def test_invalid_graph_contrast_is_not_treated_as_obstacle_fallback() -> None:
    invalid = _result("G3", eligible=False)
    invalid["verdict"] = "INVALID_NO_CAUSAL_CONTRAST"
    summary = evaluate_group(_group(G3=invalid))
    assert summary["verdict"] == "INVALID_NO_CAUSAL_CONTRAST"
    assert summary["selected_graph_mode"] == "GVG"
    assert summary["whole_group_m2_fallback_allowed"] is False


def test_incomplete_m3_group_requests_only_whole_group_m2_fallback() -> None:
    summary = evaluate_group(
        _group(G2=_result("G2", eligible=False), G3=_result("G3", eligible=False))
    )
    assert summary["verdict"] == "M3_GROUP_INCOMPLETE_TRY_WHOLE_GROUP_M2"
    assert summary["whole_group_m2_fallback_allowed"] is True
    assert summary["mixed_obstacle_arm_switch_forbidden"] is True


def test_invalid_g0_baseline_takes_precedence_over_candidate_tradeoffs() -> None:
    rows = _group(
        G0=_result("G0", eligible=False),
        G2=_result("G2", path=9.0, duration=101.0),
        G3=_result("G3", path=11.0, duration=99.0),
    )
    summary = evaluate_group(rows)
    assert summary["verdict"] == "M3_GROUP_INCOMPLETE_TRY_WHOLE_GROUP_M2"
    assert summary["selected_graph_mode"] == "GVG"


def test_ineligible_g1_shadow_control_forbids_primary_selection() -> None:
    rows = _group(
        G1=_result("G1", eligible=False),
        G3=_result("G3", path=9.0, duration=95.0, replans=4),
    )
    summary = evaluate_group(rows)
    assert summary["verdict"] == "M3_GROUP_INCOMPLETE_TRY_WHOLE_GROUP_M2"
    assert summary["selected_graph_mode"] == "GVG"


def test_mixed_m2_m3_group_is_rejected() -> None:
    rows = _group(G3=_result("G3", obstacle_arm="M2"))
    with pytest.raises(PhaseGConfigError, match="whole-group"):
        evaluate_group(rows)


def test_mismatched_pair_seed_is_rejected() -> None:
    rows = _group()
    rows["G3"]["pair_identity"]["seed"] = 7312
    with pytest.raises(PhaseGConfigError, match="pair identity"):
        evaluate_group(rows)


def test_wrapper_cli_matches_stack_contract_and_mcap_reuse() -> None:
    source = WRAPPER.read_text(encoding="utf-8")
    for fragment in (
        "run_v6_cognitive_graph_causal_stack.sh",
        '--arm "${arm}" --domain "${domain_id}"',
        '--run-root "${run_root}/stack/${arm,,}"',
        '--obstacle-arm "${obstacle_arm}"',
        "phase_b_observability --print-recorder-topics",
        "--storage mcap",
        "--qos-profile-overrides-path",
    ):
        assert fragment in source
    subprocess.run(["bash", "-n", str(WRAPPER)], check=True)


def test_setup_installs_phase_g_console_entry() -> None:
    setup_source = (PACKAGE / "setup.py").read_text(encoding="utf-8")
    assert "v6_phase_g_causal = robot_experiments.v6_phase_g_causal:main" in setup_source
