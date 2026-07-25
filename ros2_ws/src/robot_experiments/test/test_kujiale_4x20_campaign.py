import hashlib
import json
from pathlib import Path

from robot_experiments.kujiale_4x20_campaign import (
    summarize_4x20,
    write_4x20_report,
)
from robot_experiments.scenario import load_scenario


PACKAGE_ROOT = Path(__file__).parents[1]
CONFIG = PACKAGE_ROOT / "config"


def _write_checksums(root: Path) -> None:
    lines = []
    for path in sorted(item for item in root.rglob("*") if item.is_file()):
        lines.append(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.relative_to(root)}")
    (root / "checksums.sha256").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_campaign(root: Path, kinds: tuple[str, ...] = ("static", "dynamic")) -> None:
    for filename, kind in (
        ("kujiale_4x20_static_pair.yaml", "static"),
        ("kujiale_4x20_dynamic_pair.yaml", "dynamic"),
    ):
        if kind not in kinds:
            continue
        scenario = load_scenario(CONFIG / filename)
        for index, selection in enumerate(scenario.run_matrix, start=1):
            evidence = root / kind / scenario.scenario_id / f"run-{index:04d}-seed-{selection.seed}"
            evidence.mkdir(parents=True)
            manifest = {
                "condition_id": selection.condition_id,
                "appearance": {"profile_id": selection.appearance_profile_id},
                "nav2_profile": "dynamic_avoidance" if kind == "dynamic" else "stable",
                "dynamic_selection": {"case_id": selection.case_id, "variant_id": selection.variant_id},
                "metrics": {"ground_truth_path_length_m": 24.0},
                "legs": [{"duration_sec": 12.0}] * 5,
                "failure_reason": "",
                "dynamic_interaction": {"guard_aborted": False},
            }
            summary = {
                "campaign": "kujiale_long_range",
                "kind": kind,
                "seed": selection.seed,
                "condition_id": selection.condition_id,
                "appearance_profile_id": selection.appearance_profile_id,
                "nav2_profile": "dynamic_avoidance" if kind == "dynamic" else "stable",
                "strict_success": True,
                "physical_collision_free": True,
                "data_complete": True,
                "checksums_verified": True,
                "path_deviation_percent": 12.0 if kind == "static" else None,
                "dynamic_interaction_complete": kind != "dynamic" or True,
                "legs": [{"id": goal} for goal in ("G2", "G3", "G4", "G5", "G1")],
            }
            (evidence / "run_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
            (evidence / "run_summary.json").write_text(json.dumps(summary), encoding="utf-8")
            _write_checksums(evidence)


def test_4x20_summary_validates_all_four_conditions_and_writes_visual_report(tmp_path):
    run_root = tmp_path / "runs"
    _write_campaign(run_root)
    summary = summarize_4x20(run_root)
    assert summary["complete"] is True
    assert summary["passed"] is True
    assert {name: item["strict_success"]["numerator"] for name, item in summary["conditions"].items()} == {
        "static_baseline": 20,
        "static_appearance": 20,
        "dynamic_baseline": 20,
        "dynamic_appearance": 20,
    }
    output = write_4x20_report(summary, tmp_path / "report")
    assert (output / "index.html").is_file()
    assert (output / "benchmark.json").is_file()
    assert (output / "evidence_index.json").is_file()
    assert (output / "figures" / "condition_overview.png").is_file()
    assert (output / "report.pdf").read_bytes().startswith(b"%PDF")


def test_4x20_summary_marks_missing_evidence_incomplete(tmp_path):
    run_root = tmp_path / "runs"
    _write_campaign(run_root)
    next(run_root.rglob("run_summary.json")).unlink()
    summary = summarize_4x20(run_root)
    assert summary["complete"] is False
    assert summary["passed"] is False
    assert any(issue.startswith("missing_seeds:") for issue in summary["issues"])


def test_4x20_summary_ignores_pilot_evidence_with_duplicate_seed(tmp_path):
    run_root = tmp_path / "runs"
    _write_campaign(run_root)
    source = next(run_root.rglob("run-0002-seed-7201"))
    pilot = run_root / "pilot-static" / "kujiale_4x20_static_pair" / source.name
    pilot.mkdir(parents=True)
    for path in source.iterdir():
        (pilot / path.name).write_bytes(path.read_bytes())
    summary = summarize_4x20(run_root)
    assert summary["complete"] is True
    assert summary["passed"] is True
    assert len(summary["runs"]) == 80


def test_dynamic_2x20_report_is_complete_without_static_evidence(tmp_path):
    run_root = tmp_path / "runs"
    _write_campaign(run_root, kinds=("dynamic",))
    summary = summarize_4x20(run_root, scope="dynamic")
    assert summary["scope"] == "dynamic"
    assert summary["complete"] is True
    assert summary["passed"] is True
    assert set(summary["conditions"]) == {"dynamic_baseline", "dynamic_appearance"}
    output = write_4x20_report(summary, tmp_path / "dynamic-report")
    assert (output / "index.html").is_file()
    assert (output / "report.pdf").read_bytes().startswith(b"%PDF")
    assert not (output / "figures" / "static_path_deviation.png").exists()


def test_static_2x20_report_is_complete_without_dynamic_evidence(tmp_path):
    run_root = tmp_path / "runs"
    _write_campaign(run_root, kinds=("static",))
    summary = summarize_4x20(run_root, scope="static")
    assert summary["scope"] == "static"
    assert summary["complete"] is True
    assert summary["passed"] is True
    assert set(summary["conditions"]) == {"static_baseline", "static_appearance"}


def test_full_report_can_follow_retained_stage_subreports(tmp_path):
    run_root = tmp_path / "runs"
    report_root = tmp_path / "report"
    _write_campaign(run_root)
    write_4x20_report(summarize_4x20(run_root, scope="static"), report_root / "static_2x20")
    write_4x20_report(summarize_4x20(run_root, scope="dynamic"), report_root / "dynamic_2x20")
    output = write_4x20_report(summarize_4x20(run_root), report_root)
    assert (output / "index.html").is_file()
    assert (output / "static_2x20" / "index.html").is_file()
    assert (output / "dynamic_2x20" / "index.html").is_file()
