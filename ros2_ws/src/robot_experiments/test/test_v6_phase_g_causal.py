from __future__ import annotations

import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import time

import pytest

from robot_experiments.v6_phase_g_causal import (
    GRAPH_ONLY_SCOPE,
    LOOP_ROUTE_IDS,
    PhaseGConfigError,
    build_timeline,
    causal_contrast_status,
    evaluate_group,
    evaluate_graph_only_group,
    graph_only_g1_warmup_candidate_status,
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
STACK = REPO / "scripts/run_v6_cognitive_graph_causal_stack.sh"


def _fake_phase_g_isaac_wrapper(tmp_path: Path) -> Path:
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    wrapper = scripts / WRAPPER.name
    shutil.copy2(WRAPPER, wrapper)
    isaac = scripts / "run_v6_kujiale_low_obstacles.sh"
    isaac.write_text(
        "#!/usr/bin/env bash\n"
        "printf '%s|%s|%s|%s\\n' \"$ROS_DOMAIN_ID\" "
        "\"$ISAAC_NAV_EXPECTED_DOMAIN_ID\" "
        "\"$BIO_NAV_PHASE_B_DOMAIN_ID\" \"$*\"\n",
        encoding="utf-8",
    )
    isaac.chmod(0o755)
    return wrapper


def _phase_g_domain_clean_env() -> dict[str, str]:
    env = os.environ.copy()
    for name in (
        "ROS_DOMAIN_ID",
        "ISAAC_NAV_EXPECTED_DOMAIN_ID",
        "BIO_NAV_PHASE_B_DOMAIN_ID",
        "BIO_NAV_PHASE_G_DOMAIN_ID",
    ):
        env.pop(name, None)
    return env


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


def _graph_only_result(
    arm: str,
    *,
    path: float = 10.0,
    duration: float = 100.0,
    replans: int = 5,
) -> dict:
    row = _result(
        arm,
        path=path,
        duration=duration,
        replans=replans,
    )
    selected_graph_id = {
        "G1": "map:gvg_v1",
        "G2": "map:gvg_v1:hybrid:4",
        "G3": "cpg-00112233445566778899aabb:primary",
    }[arm]
    row.update(
        {
            "qualification": "ENGINEERING_GRAPH_ONLY_RUN",
            "formal_qualification": "NOT_QUALIFIED",
            "verdict": "ENGINEERING_GRAPH_ONLY_RUN_COMPLETE",
            "experiment_scope": GRAPH_ONLY_SCOPE,
            "graph_only_no_box": True,
            "box_obstacles_enabled": False,
            "cognitive_profile": "M0",
            "route_prior_enabled": False,
            "obstacle_arm": "NONE",
            "m3_safety_status": "DEFERRED",
            "route_prior_status": "DEFERRED",
            "graph_mode": {
                "G1": "shadow",
                "G2": "hybrid",
                "G3": "primary",
            }[arm],
            "candidate_graphs": [],
            "validation_acks": [],
            "reset_receipt": {"generation": 1},
        }
    )
    row["pair_identity"].update(
        {
            "experiment_scope": GRAPH_ONLY_SCOPE,
            "graph_only_no_box": True,
        }
    )
    row["scoring"]["selected_graph_ids"] = [selected_graph_id]
    candidate = {
        "node_count": 2,
        "edge_count": 1,
        "all_edge_ids_nonempty": True,
        "reset_epoch": 1,
        "recurrent_session_id": "session-current",
        "loop_index": 1,
        "module2_healthy": True,
        "observation_valid": True,
        "trusted_write": True,
        "rejection_mask": 0,
        "graph_id": "cpg-00112233445566778899aabb",
        "topology_revision": 4,
        "value_sequence": 9,
    }
    row["candidate_graphs"] = [candidate]
    if arm in {"G2", "G3"}:
        row["validation_acks"] = [
            {
                "accepted": True,
                "generation": 1,
                "reset_epoch": 1,
                "recurrent_session_id": "session-current",
                "candidate_graph_id": candidate["graph_id"],
                "candidate_topology_revision": 4,
                "candidate_value_sequence": 9,
                "validated_graph_id": selected_graph_id,
            }
        ]
    else:
        row["validation_acks"] = [
            {
                "accepted": False,
                "generation": 1,
                "reset_epoch": 1,
                "recurrent_session_id": "session-current",
            }
        ]
    return row


def test_graph_only_g1_accepts_current_warmup_candidate() -> None:
    row = _graph_only_result("G1")
    assert graph_only_g1_warmup_candidate_status(
        row["candidate_graphs"],
        row["validation_acks"],
        reset_generation=1,
    ) == (True, ())
    assert evaluate_graph_only_group(
        {name: _graph_only_result(name) for name in ("G1", "G2", "G3")}
    )["eligibility"]["G1"] is True


def test_graph_only_g1_rejects_pre_reset_or_unattributed_candidate() -> None:
    row = _graph_only_result("G1")
    row["candidate_graphs"][0]["loop_index"] = None
    valid, reasons = graph_only_g1_warmup_candidate_status(
        row["candidate_graphs"],
        row["validation_acks"],
        reset_generation=1,
    )
    assert valid is False
    assert reasons == ("current_warmup_trusted_mature_cognitive_graph_missing",)


def test_graph_only_g1_rejects_stale_reset_or_recurrent_session() -> None:
    row = _graph_only_result("G1")
    row["validation_acks"][-1].update(
        {
            "generation": 0,
            "reset_epoch": 0,
            "recurrent_session_id": "session-stale",
        }
    )
    assert graph_only_g1_warmup_candidate_status(
        row["candidate_graphs"],
        row["validation_acks"],
        reset_generation=1,
    ) == (False, ("current_reset_session_identity_missing",))

    row = _graph_only_result("G1")
    row["candidate_graphs"][0]["recurrent_session_id"] = "session-stale"
    assert graph_only_g1_warmup_candidate_status(
        row["candidate_graphs"],
        row["validation_acks"],
        reset_generation=1,
    ) == (False, ("current_warmup_trusted_mature_cognitive_graph_missing",))


def test_graph_only_g1_rejects_candidate_first_seen_in_scoring() -> None:
    row = _graph_only_result("G1")
    row["candidate_graphs"][0]["loop_index"] = 2
    assert graph_only_g1_warmup_candidate_status(
        row["candidate_graphs"],
        row["validation_acks"],
        reset_generation=1,
    ) == (False, ("current_warmup_trusted_mature_cognitive_graph_missing",))


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


def test_graph_only_manifest_is_no_box_m0_prior_off_without_changing_modes() -> None:
    config = load_config(CONFIG)
    manifest = phase_g_manifest(
        config, "G3", "M3", graph_only_no_box=True
    )
    assert manifest.runtime["experiment_scope"] == GRAPH_ONLY_SCOPE
    assert manifest.runtime["graph_only_no_box"] is True
    assert manifest.runtime["box_obstacles_enabled"] is False
    assert manifest.runtime["low_obstacles_enabled"] is False
    assert manifest.runtime["cognitive_profile"] == "M0"
    assert manifest.runtime["route_prior_enabled"] is False
    assert manifest.runtime["cognitive_graph_mode"] == "primary"
    assert manifest.runtime["m3_safety_status"] == "DEFERRED"
    assert manifest.runtime["route_prior_status"] == "DEFERRED"
    with pytest.raises(PhaseGConfigError, match="G1, G2, or G3"):
        phase_g_manifest(config, "G0", "M3", graph_only_no_box=True)


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


def test_graph_only_evaluator_uses_g1_control_and_can_select_primary() -> None:
    rows = {
        "G1": _graph_only_result("G1"),
        "G2": _graph_only_result("G2", path=9.5, duration=99.0),
        "G3": _graph_only_result(
            "G3", path=9.0, duration=95.0, replans=4
        ),
    }
    summary = evaluate_graph_only_group(rows)
    assert summary["qualification"] == "ENGINEERING_GRAPH_ONLY_CAUSAL"
    assert summary["formal_qualification"] == "NOT_QUALIFIED"
    assert summary["verdict"] == "ENGINEERING_GRAPH_ONLY_PRIMARY_CANDIDATE"
    assert summary["baseline_arm"] == "G1"
    assert summary["shadow_control"] == {
        "arm": "G1",
        "eligible": True,
        "selected_for_navigation": False,
    }
    assert summary["eligibility"] == {"G1": True, "G2": True, "G3": True}
    assert set(summary["comparisons"]) == {"G2_vs_G1", "G3_vs_G1"}
    assert summary["deferred"] == {
        "m3_safety": "DEFERRED",
        "route_prior": "DEFERRED",
    }


def test_graph_only_evaluator_rejects_mixed_scope_and_wrong_arm_set() -> None:
    rows = {name: _graph_only_result(name) for name in ("G1", "G2", "G3")}
    rows["G3"]["experiment_scope"] = "phase_g_full"
    with pytest.raises(PhaseGConfigError, match="scope"):
        evaluate_graph_only_group(rows)
    rows = {name: _graph_only_result(name) for name in ("G1", "G2", "G3")}
    rows["G0"] = _result("G0")
    with pytest.raises(PhaseGConfigError, match="exactly"):
        evaluate_graph_only_group(rows)

    rows = {name: _graph_only_result(name) for name in ("G1", "G2", "G3")}
    rows["G2"]["formal_qualification"] = "FORMAL_QUALIFICATION_PASS"
    with pytest.raises(PhaseGConfigError, match="NOT_QUALIFIED"):
        evaluate_graph_only_group(rows)

    rows = {name: _graph_only_result(name) for name in ("G1", "G2", "G3")}
    rows["G1"]["m3_safety_status"] = "IN_SCOPE"
    with pytest.raises(PhaseGConfigError, match="defer Module3 safety"):
        evaluate_graph_only_group(rows)


def test_graph_only_requires_g1_gvg_and_g2_g3_mature_accepted_selection() -> None:
    rows = {name: _graph_only_result(name) for name in ("G1", "G2", "G3")}
    rows["G1"]["scoring"]["selected_graph_ids"] = [
        "cpg-00112233445566778899aabb:primary"
    ]
    summary = evaluate_graph_only_group(rows)
    assert summary["eligibility"]["G1"] is False
    assert summary["verdict"] == "ENGINEERING_GRAPH_ONLY_GROUP_INCOMPLETE"

    rows = {name: _graph_only_result(name) for name in ("G1", "G2", "G3")}
    rows["G1"]["candidate_graphs"] = []
    rows["G2"]["validation_acks"] = []
    rows["G3"]["candidate_graphs"] = []
    summary = evaluate_graph_only_group(rows)
    assert summary["contrast_eligibility"] == {
        "G1": False,
        "G2": False,
        "G3": False,
    }
    assert summary["verdict"] == "ENGINEERING_GRAPH_ONLY_GROUP_INCOMPLETE"


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


def test_isaac_dry_run_exposes_selected_phase_b_domain_and_argv(
    tmp_path: Path,
) -> None:
    wrapper = _fake_phase_g_isaac_wrapper(tmp_path)
    result = subprocess.run(
        [
            "bash", str(wrapper), "--dry-run", "--domain", "226",
            "--arm", "G2", "isaac", "--probe", "value with space",
        ],
        check=True,
        capture_output=True,
        text=True,
        env=_phase_g_domain_clean_env(),
    )

    assert shlex.split(result.stdout) == [
        "env",
        "BIO_NAV_PHASE_B_DOMAIN_ID=226",
        str(wrapper.parent / "run_v6_kujiale_low_obstacles.sh"),
        "isaac",
        "--probe",
        "value with space",
    ]


def test_graph_only_isaac_appends_no_dynamic_obstacles_as_last_argument(
    tmp_path: Path,
) -> None:
    wrapper = _fake_phase_g_isaac_wrapper(tmp_path)
    result = subprocess.run(
        [
            "bash", str(wrapper), "--dry-run", "--domain", "226",
            "--arm", "G2", "--graph-only-no-box", "isaac",
            "--probe", "value with space",
        ],
        check=True,
        capture_output=True,
        text=True,
        env=_phase_g_domain_clean_env(),
    )
    argv = shlex.split(result.stdout)
    assert argv[-1] == "--no-dynamic-obstacles"
    assert argv[-3:-1] == ["--probe", "value with space"]


def test_isaac_child_receives_all_selected_domain_variables(tmp_path: Path) -> None:
    wrapper = _fake_phase_g_isaac_wrapper(tmp_path)
    result = subprocess.run(
        ["bash", str(wrapper), "--domain", "226", "--arm", "G2", "isaac"],
        check=True,
        capture_output=True,
        text=True,
        env=_phase_g_domain_clean_env(),
    )

    assert result.stdout == "226|226|226|isaac\n"


def test_isaac_child_receives_phase_g_default_domain(tmp_path: Path) -> None:
    wrapper = _fake_phase_g_isaac_wrapper(tmp_path)
    result = subprocess.run(
        ["bash", str(wrapper), "--arm", "G0", "isaac"],
        check=True,
        capture_output=True,
        text=True,
        env=_phase_g_domain_clean_env(),
    )

    assert result.stdout == "151|151|151|isaac\n"


def test_isaac_domain_forwarding_does_not_assign_readonly_parent_variable(
    tmp_path: Path,
) -> None:
    wrapper = _fake_phase_g_isaac_wrapper(tmp_path)
    result = subprocess.run(
        [
            "bash",
            "-c",
            "readonly BIO_NAV_PHASE_B_DOMAIN_ID=150; "
            "export BIO_NAV_PHASE_B_DOMAIN_ID; "
            "source \"$1\" --domain 226 --arm G1 isaac",
            "phase-g-readonly-test",
            str(wrapper),
        ],
        check=True,
        capture_output=True,
        text=True,
        env=_phase_g_domain_clean_env(),
    )

    assert result.stdout == "226|226|226|isaac\n"


def test_stack_waits_for_delayed_setsid_groups_and_cleans_up(tmp_path: Path) -> None:
    scripts = tmp_path / "scripts"
    (scripts / "lib").mkdir(parents=True)
    stack = scripts / STACK.name
    stack.write_text(STACK.read_text(encoding="utf-8"), encoding="utf-8")
    (scripts / "lib/common.sh").write_text(
        """#!/usr/bin/env bash
require_directory() { [[ -d "$1" ]]; }
require_file() { [[ -f "$1" ]]; }
source_ros() { :; }
""",
        encoding="utf-8",
    )
    long_running = """#!/usr/bin/env bash
trap 'exit 0' INT TERM HUP
while :; do sleep 0.05; done
"""
    module3 = scripts / "run_v6_kujiale_low_obstacles.sh"
    module3.write_text(long_running, encoding="utf-8")

    integration = tmp_path / "integration"
    integration_script = (
        integration / "scripts/run_v6_module2_graph_causal_server.sh"
    )
    integration_script.parent.mkdir(parents=True)
    integration_script.write_text(long_running, encoding="utf-8")
    candidate = integration / (
        "ros2_ws/src/bio_nav_ros_bridge/config/"
        "kujiale_0026_run4_read_only_shadow_candidate.json"
    )
    candidate.parent.mkdir(parents=True)
    candidate.touch()
    module2_config = tmp_path / (
        "module2/configs/kujiale_0026_module1_visual_shadow_v310.yaml"
    )
    module2_config.parent.mkdir(parents=True)
    module2_config.touch()

    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    ros2 = fake_bin / "ros2"
    ros2.write_text(long_running, encoding="utf-8")
    delayed_setsid = fake_bin / "setsid"
    delayed_setsid.write_text(
        """#!/usr/bin/env bash
sleep "${FAKE_SETSID_DELAY_SEC}"
/usr/bin/setsid "$@" &
wait "$!"
""",
        encoding="utf-8",
    )
    for executable in (stack, module3, integration_script, ros2, delayed_setsid):
        executable.chmod(0o755)

    run_dir = tmp_path / "run"
    socket_path = tmp_path / "socket/module2.sock"
    env = os.environ.copy()
    env.update(
        {
            "PATH": f"{fake_bin}:{env['PATH']}",
            "BIO_NAV_INTEGRATION_ROOT": str(integration),
            "BIO_NAV_MODULE2_V310_ROOT": str(module2_config.parents[1]),
            "FAKE_SETSID_DELAY_SEC": "0.08",
        }
    )
    process = subprocess.Popen(
        [
            str(stack),
            "--arm", "G3",
            "--domain", "150",
            "--run-dir", str(run_dir),
            "--socket", str(socket_path),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        env=env,
    )
    names = ("module3_ros", "module2_server", "integration_bridge")
    identities: dict[str, tuple[int, int]] = {}
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        if process.poll() is not None:
            break
        if all((run_dir / f"{name}.identity").is_file() for name in names):
            identities = {
                name: tuple(
                    int(value)
                    for value in (run_dir / f"{name}.identity")
                    .read_text(encoding="utf-8")
                    .split()
                )
                for name in names
            }
            break
        time.sleep(0.02)
    try:
        assert process.poll() is None, process.stdout.read() if process.stdout else ""
        assert set(identities) == set(names)
        assert len({pgid for _pid, pgid in identities.values()}) == len(names)
        for pid, pgid in identities.values():
            assert pid == pgid
            assert pgid != process.pid
            assert os.getpgid(pid) == pgid
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        output, _ = process.communicate(timeout=10.0)
    assert process.returncode == 130, output
    assert not list(run_dir.glob("*.identity"))
    for pid, _pgid in identities.values():
        with pytest.raises(ProcessLookupError):
            os.kill(pid, 0)


def test_setup_installs_phase_g_console_entry() -> None:
    setup_source = (PACKAGE / "setup.py").read_text(encoding="utf-8")
    assert "v6_phase_g_causal = robot_experiments.v6_phase_g_causal:main" in setup_source
