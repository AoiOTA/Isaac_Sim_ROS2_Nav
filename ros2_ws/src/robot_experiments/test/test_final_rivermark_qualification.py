import csv
import gzip
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from robot_experiments.experiment_runner import ExperimentRunner
from robot_experiments.final_rivermark_qualification import (
    _compute_only_metrics,
    _dynamic_threat_coverage,
    _trace_metrics,
    pilot_check,
)
from robot_experiments.scenario import load_scenario


PACKAGE_ROOT = Path(__file__).parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]


def _write_gzip_csv(path, fieldnames, rows):
    with gzip.open(path, "wt", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def test_trace_metrics_reports_speed_closing_ttc_and_exposure(tmp_path):
    _write_gzip_csv(
        tmp_path / "ground_truth.csv.gz",
        ("x", "y", "stamp_s"),
        [
            {"x": index * 0.1, "y": 0.0, "stamp_s": index * 0.1}
            for index in range(21)
        ],
    )
    _write_gzip_csv(
        tmp_path / "dynamic_obstacles.csv.gz",
        (
            "id", "state", "stamp_s", "position", "velocity_mps",
            "progress", "min_clearance_m",
        ),
        [
            {
                "id": "actor",
                "state": "moving",
                "stamp_s": index * 0.1,
                "position": str([2.0 - index * 0.05, 0.0, 0.5]),
                "velocity_mps": 0.5,
                "progress": index / 20.0,
                "min_clearance_m": 0.4,
            }
            for index in range(21)
        ],
    )

    actor = _trace_metrics(
        tmp_path, ("actor",), exposure_distance_m=2.5
    )["actor"]

    assert actor["active_sample_count"] == 21
    assert actor["peak_speed_mps"] == pytest.approx(0.5)
    assert actor["maximum_progress"] == pytest.approx(1.0)
    assert actor["minimum_center_distance_m"] == pytest.approx(0.05)
    assert actor["maximum_relative_closing_speed_mps"] == pytest.approx(1.5)
    assert actor["exposure_duration_sec"] == pytest.approx(2.0)
    assert actor["minimum_time_to_collision_sec"] is not None


def _threat_contract():
    contract_path = (
        REPOSITORY_ROOT
        / "data/rivermark_demo/final_rivermark_metric_contract.yaml"
    )
    return yaml.safe_load(contract_path.read_text(encoding="utf-8"))[
        "primary_navigation_metrics"
    ]["dynamic"]


def _passing_actor_metrics(contract):
    return {
        actor_id: {
            "aligned_sample_count": 100,
            "active_sample_count": 80,
            "states": ["moving", "clearing"],
            "minimum_center_distance_m": 1.0,
            "minimum_manager_clearance_m": 0.1,
            "peak_speed_mps": values["minimum_peak_speed_mps"],
            "maximum_progress": values["minimum_progress"],
            "maximum_relative_closing_speed_mps": 0.5,
            "minimum_time_to_collision_sec": 2.0,
            "exposure_distance_m": contract["exposure_distance_m"],
            "exposure_duration_sec": 1.0,
        }
        for actor_id, values in contract["actors"].items()
    }


def test_dynamic_threat_gate_rejects_a_slow_non_threatening_actor(monkeypatch):
    contract = _threat_contract()
    metrics = _passing_actor_metrics(contract)
    metrics["rivermark_crossing_cart"]["peak_speed_mps"] = 0.05
    monkeypatch.setattr(
        "robot_experiments.final_rivermark_qualification._trace_metrics",
        lambda *args, **kwargs: metrics,
    )
    records = [
        {
            "summary_path": f"/run-{index}/run_summary.json",
            "run_index": index,
            "seed": 19400 + index,
        }
        for index in range(1, 21)
    ]

    result = _dynamic_threat_coverage(records, contract)

    assert result["passed_runs"] == 0
    assert result["passed"] is False
    assert result["runs"][0]["actor_gates"][
        "rivermark_crossing_cart"
    ]["peak_speed"] is False


def test_compute_percentages_are_secondary_and_have_absolute_latencies():
    source = (
        REPOSITORY_ROOT
        / "data/rivermark_demo/benchmarks/module2_contracts/contract_benchmark_summary.json"
    )

    result = _compute_only_metrics(source)

    assert result["gating_for_navigation_qualification"] is False
    assert result["adaptation_compute_latency"]["sample_count"] == 20
    assert result["adaptation_compute_latency"][
        "baseline_latency_p50_ms"
    ] > result["adaptation_compute_latency"]["treatment_latency_p50_ms"]
    assert result["adaptation_compute_latency"]["median_speedup_ratio"] > 1.0
    assert result["prohibited_interpretation"] == (
        "navigation_success_or_end_to_end_speed_improvement"
    )


def test_final_scenarios_are_new_twenty_run_identities():
    for group, first_seed in (
        ("static", 19301),
        ("dynamic", 19401),
        ("appearance", 19501),
    ):
        scenario = load_scenario(
            PACKAGE_ROOT / "config" / f"final_rivermark_{group}.yaml"
        )
        assert scenario.scenario_id == f"final_rivermark_{group}"
        assert len(scenario.run_matrix) == 20
        assert scenario.run_matrix[0].seed == first_seed


def test_dispatch_receipt_is_created_once_before_navigation_write(tmp_path):
    runner = ExperimentRunner.__new__(ExperimentRunner)
    runner._goal_dispatch_recorded = False
    runner._record_evidence = True
    runner._active_evidence_root = tmp_path
    runner._active_run_index = 3
    runner._active_selection = SimpleNamespace(
        seed=19403,
        condition_id="final_rivermark_dynamic",
        case_id="crossing",
        variant_id="v3",
    )
    runner._scenario = SimpleNamespace(scenario_id="final_rivermark_dynamic")
    runner._experiment_arm = "medium"
    runner._navigation_execution_backend = "route_guided"
    runner._clock_seconds = lambda: 42.5
    lifecycle = []
    runner._lifecycle_event = lifecycle.append

    runner._record_trial_dispatched()

    receipt = json.loads((tmp_path / "TRIAL_DISPATCHED.json").read_text())
    assert receipt["run_index"] == 3
    assert receipt["seed"] == 19403
    assert receipt["experiment_arm"] == "medium"
    assert lifecycle == ["goal_dispatched"]
    with pytest.raises(Exception, match="already exists|goal"):
        # A second call is a no-op because the in-memory fence is already set.
        runner._goal_dispatch_recorded = False
        runner._record_trial_dispatched()


def test_pilot_check_recomputes_checksums_and_binds_metric_contract(tmp_path):
    contract = (
        REPOSITORY_ROOT
        / "data/rivermark_demo/final_rivermark_metric_contract.yaml"
    )
    contract_sha = hashlib.sha256(contract.read_bytes()).hexdigest()
    evidence = tmp_path / "final_rivermark_static/run-0001-seed-19301"
    evidence.mkdir(parents=True)
    (evidence / "run_manifest.json").write_text(
        json.dumps(
            {
                "scenario_id": "final_rivermark_static",
                "run_index": 1,
                "random_seed": 19301,
            }
        )
    )
    (evidence / "TRIAL_DISPATCHED.json").write_text("{}\n")
    (evidence / "FINAL_TRIAL_METRICS.json").write_text("{}\n")
    (evidence / "run_summary.json").write_text(
        json.dumps(
            {
                "strict_success": True,
                "physical_collision_free": True,
                "data_complete": True,
                "checksums_verified": True,
                "legs": [{"id": f"G{index}"} for index in range(1, 6)],
                "final_trial_metric_gate": {
                    "applicable": True,
                    "passed": True,
                    "contract_sha256": contract_sha,
                },
            }
        )
    )
    files = sorted(path for path in evidence.iterdir() if path.is_file())
    (evidence / "checksums.sha256").write_text(
        "\n".join(
            f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}"
            for path in files
        )
        + "\n"
    )

    result = pilot_check(
        group="static", root=tmp_path, metric_contract=contract
    )

    assert result["status"] == "PASS"
    assert result["gates"]["checksums_recomputed"] is True
    assert result["gates"]["final_metric_gate"] is True
