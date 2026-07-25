import hashlib
import gzip
import json
from pathlib import Path

from PIL import Image
from robot_experiments.kujiale_4x20_campaign import (
    main,
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


def _write_campaign(
    root: Path, kinds: tuple[str, ...] = ("static", "dynamic"), *, include_one_trajectory: bool = False
) -> None:
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
            if include_one_trajectory and kind == "static" and index == 1:
                with gzip.open(evidence / "ground_truth.csv.gz", "wt", encoding="utf-8", newline="") as stream:
                    stream.write("x,y,yaw_rad,linear_speed_mps,angular_speed_radps,stamp_s\n")
                    stream.write("0.45,-5.35,1.57,0.0,0.0,0.0\n")
                    stream.write("0.60,-5.10,1.40,0.2,0.0,1.0\n")
                    stream.write("0.80,4.80,-2.79,0.0,0.0,2.0\n")
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
    assert (output / "figures" / "dynamic_baseline_test_map.png").is_file()
    assert (output / "figures" / "dynamic_appearance_test_map.png").is_file()


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


def test_status_cli_supports_a_static_2x20_scope(tmp_path, capsys):
    run_root = tmp_path / "runs"
    _write_campaign(run_root, kinds=("static",))
    main(["--run-root", str(run_root), "--scope", "static", "--status"])
    status = json.loads(capsys.readouterr().out)
    assert status["scope"] == "static"
    assert status["complete"] is True
    assert status["passed"] is True


def test_report_embeds_a_filterable_actual_ground_truth_trajectory(tmp_path):
    run_root = tmp_path / "runs"
    _write_campaign(run_root, kinds=("static",), include_one_trajectory=True)
    output = write_4x20_report(summarize_4x20(run_root, scope="static"), tmp_path / "report")
    report = (output / "index.html").read_text(encoding="utf-8")
    assert "逐轮实际 GT 路径" in report
    assert "id='seed'" in report
    assert "id='trajectory'" in report
    assert (output / "figures" / "trajectories" / "static_baseline-seed-7201-baseline.png").is_file()
    assert (output / "figures" / "static_baseline_test_map.png").is_file()
    assert (output / "figures" / "static_appearance_test_map.png").is_file()
    assert not (output / "figures" / "kujiale_4x20_test_matrix_map.png").exists()
    with Image.open(output / "figures" / "static_baseline_test_map.png") as map_image:
        assert map_image.size == (1165, 820)
    benchmark = json.loads((output / "benchmark.json").read_text(encoding="utf-8"))
    assert any(row.get("trajectory_figure") for row in benchmark["runs"])
    legacy_map = output / "figures" / "kujiale_4x20_test_matrix_map.png"
    legacy_map.write_bytes(b"out-of-scope")
    assert write_4x20_report(
        summarize_4x20(run_root, scope="static"), output, replace_output=True
    ) == output
    assert not legacy_map.exists()
