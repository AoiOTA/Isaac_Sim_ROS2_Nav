import hashlib
import json
from pathlib import Path

import pytest

from robot_experiments.kujiale_campaign import (
    CampaignValidationError,
    load_campaign_definition,
    summarize_campaign,
    write_campaign_report,
)


def test_frozen_campaign_definition_has_the_required_v2_route_and_seeds():
    definition = load_campaign_definition(
        Path(__file__).resolve().parents[1]
        / "config" / "kujiale_long_range_campaign.yaml"
    )
    assert definition["route"][3]["id"] == "G4"


def _run(kind, seed, *, strict=True, collision_free=True, deviation=12.5):
    return {
        "campaign": "kujiale_long_range",
        "kind": kind,
        "seed": seed,
        "strict_success": strict,
        "physical_collision_free": collision_free,
        "data_complete": True,
        "checksums_verified": True,
        "path_deviation_percent": deviation,
        "legs": [{"id": f"G{index}"} for index in range(1, 9)],
    }


def _write_batch(root, kind, start, failures=0):
    for offset in range(20):
        run = _run(kind, start + offset, strict=offset >= failures, collision_free=offset >= failures)
        target = root / kind / f"{start + offset}" / "run_summary.json"
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(run), encoding="utf-8")


def test_campaign_requires_exact_seeds_and_writes_consistent_artifacts(tmp_path):
    _write_batch(tmp_path, "static", 7201, failures=1)
    _write_batch(tmp_path, "dynamic", 7301, failures=2)
    summary = summarize_campaign([tmp_path])
    assert summary == summarize_campaign([tmp_path])
    assert summary["passed"] is True
    assert summary["static"]["strict_success"]["numerator"] == 19
    assert summary["dynamic"]["strict_success"]["numerator"] == 18
    output = write_campaign_report(summary, tmp_path / "report")
    benchmark = json.loads((output / "benchmark.json").read_text())
    assert benchmark["passed"] is summary["passed"]
    assert all("_evidence_dir" not in row for row in benchmark["runs"])
    assert (output / "runs" / "static-7201" / "run_summary.json").is_file()
    assert (output / "index.html").is_file()
    assert (output / "report.pdf").read_bytes().startswith(b"%PDF")
    png = (output / "figures" / "campaign_overview.png").read_bytes()
    assert png.startswith(b"\x89PNG\r\n\x1a\n")
    assert b"\x00\x00\x06\x40\x00\x00\x03\x84" in png[:32]  # 1600 x 900
    checksums = (output / "checksums.sha256").read_text()
    assert hashlib.sha256((output / "benchmark.json").read_bytes()).hexdigest() in checksums


def test_campaign_rejects_missing_or_selectively_repeated_formal_runs(tmp_path):
    _write_batch(tmp_path, "static", 7201)
    _write_batch(tmp_path, "dynamic", 7301)
    (tmp_path / "static" / "7201" / "run_summary.json").unlink()
    with pytest.raises(CampaignValidationError, match="seeds must be exactly"):
        summarize_campaign([tmp_path])


def test_campaign_accepts_an_ordered_failed_leg_prefix_and_pads_the_heatmap(tmp_path):
    _write_batch(tmp_path, "static", 7201)
    _write_batch(tmp_path, "dynamic", 7301)
    failed = _run("dynamic", 7301, strict=False)
    failed["legs"] = [{"id": "G1", "timed_out": True}]
    (tmp_path / "dynamic" / "7301" / "run_summary.json").write_text(
        json.dumps(failed), encoding="utf-8"
    )
    summary = summarize_campaign([tmp_path])
    assert summary["dynamic"]["strict_success"]["numerator"] == 19
    output = write_campaign_report(summary, tmp_path / "report")
    assert (output / "figures" / "waypoint_heatmap_dynamic.png").is_file()
