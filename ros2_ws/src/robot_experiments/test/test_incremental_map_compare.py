"""Tests for deterministic offline incremental-map comparison."""

import json
from pathlib import Path

import pytest
import yaml

from robot_experiments.configuration import ConfigurationError
from robot_experiments.incremental_map_compare import (
    compare_incremental_map,
    load_comparison_spec,
    main,
)


def _write_map(directory: Path, name: str, rows: list[list[int]]) -> Path:
    image = directory / f"{name}.pgm"
    height = len(rows)
    width = len(rows[0])
    samples = "\n".join(" ".join(str(value) for value in row) for row in rows)
    image.write_text(f"P2\n{width} {height}\n255\n{samples}\n", encoding="ascii")
    metadata = directory / f"{name}.yaml"
    metadata.write_text(
        yaml.safe_dump(
            {
                "image": image.name,
                "resolution": 1.0,
                "origin": [0.0, 0.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            }
        ),
        encoding="utf-8",
    )
    return metadata


def _write_spec(
    directory: Path,
    baseline: Path,
    full: Path,
    incremental: Path,
    *,
    regions=None,
    thresholds=None,
) -> Path:
    comparison = {
        "baseline_map": baseline.name,
        "full_remap_map": full.name,
        "incremental_map": incremental.name,
        "timing_sec": {
            "full_mapping": 100.0,
            "incremental_mapping": 60.0,
        },
        "changed_regions": regions
        if regions is not None
        else [{"id": "rack", "bounds_m": [1.0, 1.0, 2.0, 2.0]}],
    }
    if thresholds is not None:
        comparison["thresholds"] = thresholds
    spec = directory / "comparison.yaml"
    spec.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "comparison": comparison,
            }
        ),
        encoding="utf-8",
    )
    return spec


def test_full_incremental_comparison_accepts_recovered_changed_cell(tmp_path):
    """A recovered authored change with no regression is accepted."""
    baseline = _write_map(tmp_path, "baseline", [[254] * 4 for _ in range(3)])
    changed = [[254] * 4 for _ in range(3)]
    changed[1][1] = 0
    full = _write_map(tmp_path, "full", changed)
    incremental = _write_map(tmp_path, "incremental", changed)
    spec = load_comparison_spec(
        _write_spec(tmp_path, baseline, full, incremental)
    )

    result = compare_incremental_map(spec)

    assert result.accepted is True
    assert result.report["metrics"]["time_improvement_percent"] == pytest.approx(40.0)
    assert result.report["metrics"]["changed_cell_recall_percent"] == 100.0
    assert result.report["metrics"]["old_region_regression_percent"] == 0.0
    assert result.report["counts"]["reference_changed_cells_in_declared_regions"] == 1


def test_comparison_rejects_missed_change_and_old_region_regression(tmp_path):
    """A missed target cell plus an old-region mutation fails both checks."""
    free = [[254] * 4 for _ in range(3)]
    baseline = _write_map(tmp_path, "baseline", free)
    full_rows = [row[:] for row in free]
    full_rows[1][1] = 0
    full = _write_map(tmp_path, "full", full_rows)
    incremental_rows = [row[:] for row in free]
    incremental_rows[2][3] = 0
    incremental = _write_map(tmp_path, "incremental", incremental_rows)
    spec = load_comparison_spec(
        _write_spec(tmp_path, baseline, full, incremental)
    )

    result = compare_incremental_map(spec)

    assert result.accepted is False
    assert result.report["checks"]["changed_cell_recall"] is False
    assert result.report["checks"]["old_region_regression"] is False


def test_spec_requires_geometric_changed_regions(tmp_path):
    """A label alone cannot define which map cells belong to a change."""
    baseline = _write_map(tmp_path, "baseline", [[254]])
    full = _write_map(tmp_path, "full", [[0]])
    incremental = _write_map(tmp_path, "incremental", [[0]])
    spec = _write_spec(tmp_path, baseline, full, incremental, regions=["label_only"])

    with pytest.raises(ConfigurationError, match="must be a mapping"):
        load_comparison_spec(spec)


def test_spec_accepts_explicit_old_region_regression_threshold(tmp_path):
    """Every documented threshold can be overridden without a parser error."""
    baseline = _write_map(tmp_path, "baseline", [[254]])
    full = _write_map(tmp_path, "full", [[0]])
    incremental = _write_map(tmp_path, "incremental", [[0]])
    spec = _write_spec(
        tmp_path,
        baseline,
        full,
        incremental,
        regions=[{"id": "cell", "bounds_m": [0.0, 0.0, 1.0, 1.0]}],
        thresholds={"maximum_old_region_regression_percent": 2.5},
    )

    loaded = load_comparison_spec(spec)

    assert loaded.thresholds.maximum_old_region_regression_percent == 2.5


def test_cli_writes_strict_report_and_returns_rejection_status(tmp_path):
    """The CLI preserves rejected evidence and returns a distinct status."""
    baseline = _write_map(tmp_path, "baseline", [[254]])
    full = _write_map(tmp_path, "full", [[0]])
    incremental = _write_map(tmp_path, "incremental", [[254]])
    spec = _write_spec(
        tmp_path,
        baseline,
        full,
        incremental,
        regions=[{"id": "cell", "bounds_m": [0.0, 0.0, 1.0, 1.0]}],
    )
    output = tmp_path / "report.json"

    assert main(["--spec", str(spec), "--output", str(output)]) == 2
    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["report_type"] == "incremental_occupancy_map_comparison"
    assert report["map_update_comparison_accepted"] is False
    assert "post-update Localization" in report["scope_note"]
