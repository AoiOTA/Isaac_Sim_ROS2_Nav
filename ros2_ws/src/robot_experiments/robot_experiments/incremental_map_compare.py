"""Compare a full remap with an incrementally updated occupancy map.

The comparison deliberately runs offline.  A changed environment must be
mapped twice: once from scratch and once after loading the baseline pose graph.
This module then evaluates the saved OccupancyGrid artifacts in world
coordinates and writes a strict JSON evidence report.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

from .configuration import ConfigurationError
from .metrics import incremental_time_improvement_percent
from .report import _atomic_text_write


CELL_UNKNOWN = -1
CELL_FREE = 0
CELL_OCCUPIED = 1


def _finite(value: Any, location: str) -> float:
    if isinstance(value, bool):
        raise ConfigurationError(f"{location} must be a finite number")
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigurationError(
            f"{location} must be a finite number"
        ) from exc
    if not math.isfinite(parsed):
        raise ConfigurationError(f"{location} must be a finite number")
    return parsed


def _mapping(value: Any, location: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ConfigurationError(f"{location} must be a mapping")
    return value


def _reject_unknown(
    values: Mapping[str, Any], allowed: set[str], location: str
) -> None:
    unknown = sorted(set(values) - allowed)
    if unknown:
        raise ConfigurationError(f"unknown {location} keys: {unknown}")


def _path(value: Any, location: str, base_directory: Path) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise ConfigurationError(f"{location} must be a non-empty path")
    candidate = Path(value).expanduser()
    if not candidate.is_absolute():
        candidate = base_directory / candidate
    resolved = candidate.resolve()
    if not resolved.is_file():
        raise ConfigurationError(f"{location} does not exist: {resolved}")
    return resolved


@dataclass(frozen=True)
class ChangedRegion:
    """Axis-aligned changed region in map/world metres."""

    region_id: str
    minimum_x: float
    minimum_y: float
    maximum_x: float
    maximum_y: float

    def contains(self, x: float, y: float) -> bool:
        """Return whether a world point is inside this half-open rectangle."""
        return (
            self.minimum_x <= x < self.maximum_x
            and self.minimum_y <= y < self.maximum_y
        )

    def as_dict(self) -> dict[str, object]:
        """Return the stable JSON representation used by evidence reports."""
        return {
            "id": self.region_id,
            "bounds_m": [
                self.minimum_x,
                self.minimum_y,
                self.maximum_x,
                self.maximum_y,
            ],
        }


@dataclass(frozen=True)
class ComparisonThresholds:
    """Explicit acceptance thresholds for one offline comparison."""

    minimum_time_improvement_percent: float = 30.0
    minimum_reference_overlap_percent: float = 95.0
    minimum_changed_cell_count: int = 1
    minimum_changed_cell_recall_percent: float = 95.0
    minimum_changed_region_agreement_percent: float = 95.0
    maximum_old_region_regression_percent: float = 1.0

    def __post_init__(self) -> None:
        """Reject thresholds that would make the comparison meaningless."""
        percentage_fields = (
            "minimum_time_improvement_percent",
            "minimum_reference_overlap_percent",
            "minimum_changed_cell_recall_percent",
            "minimum_changed_region_agreement_percent",
            "maximum_old_region_regression_percent",
        )
        for name in percentage_fields:
            value = _finite(getattr(self, name), f"thresholds.{name}")
            if not 0.0 <= value <= 100.0:
                raise ConfigurationError(
                    f"thresholds.{name} must be between 0 and 100"
                )
        if (
            isinstance(self.minimum_changed_cell_count, bool)
            or not isinstance(self.minimum_changed_cell_count, int)
            or self.minimum_changed_cell_count < 1
        ):
            raise ConfigurationError(
                "thresholds.minimum_changed_cell_count must be a positive integer"
            )


@dataclass(frozen=True)
class ComparisonSpec:
    """Resolved map artifacts, timings, regions, and thresholds."""

    source_path: Path
    baseline_map: Path
    full_remap_map: Path
    incremental_map: Path
    full_mapping_time_sec: float
    incremental_mapping_time_sec: float
    changed_regions: tuple[ChangedRegion, ...]
    thresholds: ComparisonThresholds


@dataclass(frozen=True)
class OccupancyMap:
    """A trinary ROS OccupancyGrid image with world-coordinate helpers."""

    source_path: Path
    image_path: Path
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    width: int
    height: int
    cells: tuple[int, ...]

    def value_at_world(self, world_x: float, world_y: float) -> int | None:
        """Sample a cell by world coordinates, or return ``None`` outside."""
        delta_x = world_x - self.origin_x
        delta_y = world_y - self.origin_y
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        local_x = cosine * delta_x + sine * delta_y
        local_y = -sine * delta_x + cosine * delta_y
        column = math.floor(local_x / self.resolution)
        grid_y = math.floor(local_y / self.resolution)
        if not (0 <= column < self.width and 0 <= grid_y < self.height):
            return None
        row = self.height - 1 - grid_y
        return self.cells[row * self.width + column]

    def world_cell_center(self, row: int, column: int) -> tuple[float, float]:
        """Convert one PGM row/column center to world coordinates."""
        local_x = (column + 0.5) * self.resolution
        local_y = (self.height - row - 0.5) * self.resolution
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        return (
            self.origin_x + cosine * local_x - sine * local_y,
            self.origin_y + sine * local_x + cosine * local_y,
        )


@dataclass(frozen=True)
class ComparisonResult:
    """Strict-JSON report and its aggregate map-comparison decision."""

    report: Mapping[str, Any]

    @property
    def accepted(self) -> bool:
        """Return the aggregate saved-map comparison decision."""
        return bool(self.report["map_update_comparison_accepted"])


def _parse_region(value: Any, index: int) -> ChangedRegion:
    location = f"comparison.changed_regions[{index}]"
    region = _mapping(value, location)
    _reject_unknown(region, {"id", "bounds_m"}, location)
    identifier = region.get("id")
    if not isinstance(identifier, str) or not identifier.strip():
        raise ConfigurationError(f"{location}.id must be a non-empty string")
    bounds = region.get("bounds_m")
    if not isinstance(bounds, Sequence) or isinstance(bounds, (str, bytes)):
        raise ConfigurationError(f"{location}.bounds_m must contain four numbers")
    if len(bounds) != 4:
        raise ConfigurationError(f"{location}.bounds_m must contain four numbers")
    parsed = tuple(
        _finite(component, f"{location}.bounds_m[{component_index}]")
        for component_index, component in enumerate(bounds)
    )
    if parsed[2] <= parsed[0] or parsed[3] <= parsed[1]:
        raise ConfigurationError(
            f"{location}.bounds_m must be [min_x, min_y, max_x, max_y]"
        )
    return ChangedRegion(identifier.strip(), *parsed)


def _parse_thresholds(value: Any) -> ComparisonThresholds:
    if value is None:
        return ComparisonThresholds()
    raw = _mapping(value, "comparison.thresholds")
    allowed = set(ComparisonThresholds.__dataclass_fields__)
    _reject_unknown(raw, allowed, "comparison.thresholds")
    defaults = asdict(ComparisonThresholds())
    defaults.update(raw)
    count = defaults["minimum_changed_cell_count"]
    if isinstance(count, bool) or not isinstance(count, int):
        raise ConfigurationError(
            "thresholds.minimum_changed_cell_count must be a positive integer"
        )
    return ComparisonThresholds(
        minimum_time_improvement_percent=_finite(
            defaults["minimum_time_improvement_percent"],
            "thresholds.minimum_time_improvement_percent",
        ),
        minimum_reference_overlap_percent=_finite(
            defaults["minimum_reference_overlap_percent"],
            "thresholds.minimum_reference_overlap_percent",
        ),
        minimum_changed_cell_count=count,
        minimum_changed_cell_recall_percent=_finite(
            defaults["minimum_changed_cell_recall_percent"],
            "thresholds.minimum_changed_cell_recall_percent",
        ),
        minimum_changed_region_agreement_percent=_finite(
            defaults["minimum_changed_region_agreement_percent"],
            "thresholds.minimum_changed_region_agreement_percent",
        ),
        maximum_old_region_regression_percent=_finite(
            defaults["maximum_old_region_regression_percent"],
            "thresholds.maximum_old_region_regression_percent",
        ),
    )


def load_comparison_spec(path: str | Path) -> ComparisonSpec:
    """Load and strictly validate a YAML comparison specification."""
    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise ConfigurationError(f"comparison spec does not exist: {source}")
    try:
        document = yaml.safe_load(source.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ConfigurationError(f"invalid YAML in comparison spec {source}") from exc
    root = _mapping(document, "comparison spec")
    _reject_unknown(root, {"schema_version", "comparison"}, "comparison spec")
    if root.get("schema_version") != 1:
        raise ConfigurationError("comparison spec schema_version must be 1")
    comparison = _mapping(root.get("comparison"), "comparison")
    _reject_unknown(
        comparison,
        {
            "baseline_map",
            "full_remap_map",
            "incremental_map",
            "timing_sec",
            "changed_regions",
            "thresholds",
        },
        "comparison",
    )
    base_directory = source.parent
    timing = _mapping(comparison.get("timing_sec"), "comparison.timing_sec")
    _reject_unknown(timing, {"full_mapping", "incremental_mapping"}, "timing_sec")
    full_time = _finite(timing.get("full_mapping"), "timing_sec.full_mapping")
    incremental_time = _finite(
        timing.get("incremental_mapping"), "timing_sec.incremental_mapping"
    )
    if full_time <= 0.0:
        raise ConfigurationError("timing_sec.full_mapping must be positive")
    if incremental_time < 0.0:
        raise ConfigurationError("timing_sec.incremental_mapping must be non-negative")
    raw_regions = comparison.get("changed_regions")
    if not isinstance(raw_regions, list) or not raw_regions:
        raise ConfigurationError("comparison.changed_regions must be non-empty")
    regions = tuple(_parse_region(value, index) for index, value in enumerate(raw_regions))
    identifiers = [region.region_id for region in regions]
    if len(set(identifiers)) != len(identifiers):
        raise ConfigurationError("comparison.changed_regions contains duplicate ids")
    return ComparisonSpec(
        source_path=source,
        baseline_map=_path(
            comparison.get("baseline_map"), "comparison.baseline_map", base_directory
        ),
        full_remap_map=_path(
            comparison.get("full_remap_map"),
            "comparison.full_remap_map",
            base_directory,
        ),
        incremental_map=_path(
            comparison.get("incremental_map"),
            "comparison.incremental_map",
            base_directory,
        ),
        full_mapping_time_sec=full_time,
        incremental_mapping_time_sec=incremental_time,
        changed_regions=regions,
        thresholds=_parse_thresholds(comparison.get("thresholds")),
    )


def _pgm_token(data: bytes, start: int) -> tuple[bytes, int]:
    cursor = start
    while cursor < len(data):
        if data[cursor] in b" \t\r\n":
            cursor += 1
            continue
        if data[cursor] == ord("#"):
            newline = data.find(b"\n", cursor)
            cursor = len(data) if newline < 0 else newline + 1
            continue
        break
    token_start = cursor
    while cursor < len(data) and data[cursor] not in b" \t\r\n#":
        cursor += 1
    if token_start == cursor:
        raise ConfigurationError("PGM header ended unexpectedly")
    return data[token_start:cursor], cursor


def _read_pgm(path: Path) -> tuple[int, int, int, tuple[int, ...]]:
    data = path.read_bytes()
    cursor = 0
    tokens: list[bytes] = []
    for _ in range(4):
        token, cursor = _pgm_token(data, cursor)
        tokens.append(token)
    magic = tokens[0]
    if magic not in {b"P2", b"P5"}:
        raise ConfigurationError(f"unsupported map image format in {path}: {magic!r}")
    try:
        width, height, maximum = (int(token) for token in tokens[1:])
    except ValueError as exc:
        raise ConfigurationError(f"invalid PGM header in {path}") from exc
    if width <= 0 or height <= 0 or not 1 <= maximum <= 65535:
        raise ConfigurationError(f"invalid PGM dimensions or max value in {path}")
    sample_count = width * height
    if magic == b"P2":
        samples: list[int] = []
        for _ in range(sample_count):
            token, cursor = _pgm_token(data, cursor)
            try:
                samples.append(int(token))
            except ValueError as exc:
                raise ConfigurationError(f"invalid PGM sample in {path}") from exc
    else:
        if cursor >= len(data) or data[cursor] not in b" \t\r\n":
            raise ConfigurationError(f"PGM header has no data separator in {path}")
        if data[cursor:cursor + 2] == b"\r\n":
            cursor += 2
        else:
            cursor += 1
        bytes_per_sample = 1 if maximum < 256 else 2
        expected_size = sample_count * bytes_per_sample
        payload = data[cursor:cursor + expected_size]
        if len(payload) != expected_size:
            raise ConfigurationError(f"PGM pixel payload is truncated in {path}")
        if bytes_per_sample == 1:
            samples = list(payload)
        else:
            samples = [
                int.from_bytes(payload[offset:offset + 2], "big")
                for offset in range(0, len(payload), 2)
            ]
    if any(sample < 0 or sample > maximum for sample in samples):
        raise ConfigurationError(f"PGM sample is outside declared range in {path}")
    return width, height, maximum, tuple(samples)


def load_occupancy_map(path: str | Path) -> OccupancyMap:
    """Load a ROS map-server YAML and its P2/P5 trinary PGM image."""
    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise ConfigurationError(f"occupancy map YAML does not exist: {source}")
    try:
        document = yaml.safe_load(source.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ConfigurationError(f"invalid occupancy map YAML: {source}") from exc
    values = _mapping(document, f"occupancy map {source}")
    image_value = values.get("image")
    if not isinstance(image_value, str) or not image_value.strip():
        raise ConfigurationError(f"occupancy map image is missing in {source}")
    image = Path(image_value).expanduser()
    if not image.is_absolute():
        image = source.parent / image
    image = image.resolve()
    if not image.is_file():
        raise ConfigurationError(f"occupancy map image does not exist: {image}")
    resolution = _finite(values.get("resolution"), f"{source}.resolution")
    if resolution <= 0.0:
        raise ConfigurationError(f"{source}.resolution must be positive")
    origin = values.get("origin")
    if not isinstance(origin, Sequence) or isinstance(origin, (str, bytes)):
        raise ConfigurationError(f"{source}.origin must contain x, y, and yaw")
    if len(origin) != 3:
        raise ConfigurationError(f"{source}.origin must contain x, y, and yaw")
    parsed_origin = tuple(
        _finite(component, f"{source}.origin[{index}]")
        for index, component in enumerate(origin)
    )
    negate = values.get("negate", 0)
    if negate not in {0, 1, False, True}:
        raise ConfigurationError(f"{source}.negate must be 0 or 1")
    occupied_threshold = _finite(
        values.get("occupied_thresh", 0.65), f"{source}.occupied_thresh"
    )
    free_threshold = _finite(
        values.get("free_thresh", 0.196), f"{source}.free_thresh"
    )
    if not 0.0 <= free_threshold < occupied_threshold <= 1.0:
        raise ConfigurationError(
            f"{source} requires 0 <= free_thresh < occupied_thresh <= 1"
        )
    mode = values.get("mode", "trinary")
    if mode != "trinary":
        raise ConfigurationError(
            f"{source}.mode={mode!r} is unsupported; comparison requires trinary"
        )
    width, height, maximum, pixels = _read_pgm(image)
    cells: list[int] = []
    for pixel in pixels:
        occupancy_probability = pixel / maximum if negate else (maximum - pixel) / maximum
        if occupancy_probability > occupied_threshold:
            cells.append(CELL_OCCUPIED)
        elif occupancy_probability < free_threshold:
            cells.append(CELL_FREE)
        else:
            cells.append(CELL_UNKNOWN)
    return OccupancyMap(
        source_path=source,
        image_path=image,
        resolution=resolution,
        origin_x=parsed_origin[0],
        origin_y=parsed_origin[1],
        origin_yaw=parsed_origin[2],
        width=width,
        height=height,
        cells=tuple(cells),
    )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _percent(numerator: int, denominator: int) -> float | None:
    if denominator == 0:
        return None
    return numerator / denominator * 100.0


def _map_record(occupancy_map: OccupancyMap) -> dict[str, object]:
    return {
        "yaml_path": str(occupancy_map.source_path),
        "yaml_sha256": _sha256(occupancy_map.source_path),
        "image_path": str(occupancy_map.image_path),
        "image_sha256": _sha256(occupancy_map.image_path),
        "resolution_m": occupancy_map.resolution,
        "origin": [
            occupancy_map.origin_x,
            occupancy_map.origin_y,
            occupancy_map.origin_yaw,
        ],
        "width": occupancy_map.width,
        "height": occupancy_map.height,
    }


def compare_incremental_map(spec: ComparisonSpec) -> ComparisonResult:
    """Compare three map artifacts over the full-remap world domain."""
    baseline = load_occupancy_map(spec.baseline_map)
    full = load_occupancy_map(spec.full_remap_map)
    incremental = load_occupancy_map(spec.incremental_map)

    total_reference_cells = full.width * full.height
    overlap_cells = 0
    declared_region_cells = 0
    changed_reference_cells = 0
    recovered_changed_cells = 0
    changed_region_agreement_cells = 0
    stable_old_region_cells = 0
    regressed_old_region_cells = 0
    reference_changes_outside_regions = 0
    candidate_changes_outside_regions = 0

    for row in range(full.height):
        for column in range(full.width):
            full_value = full.cells[row * full.width + column]
            world_x, world_y = full.world_cell_center(row, column)
            baseline_value = baseline.value_at_world(world_x, world_y)
            incremental_value = incremental.value_at_world(world_x, world_y)
            if baseline_value is None or incremental_value is None:
                continue
            overlap_cells += 1
            in_changed_region = any(
                region.contains(world_x, world_y) for region in spec.changed_regions
            )
            if in_changed_region:
                declared_region_cells += 1
                if incremental_value == full_value:
                    changed_region_agreement_cells += 1
                if baseline_value != full_value:
                    changed_reference_cells += 1
                    if incremental_value == full_value:
                        recovered_changed_cells += 1
            else:
                if baseline_value == full_value:
                    stable_old_region_cells += 1
                    if incremental_value != full_value:
                        regressed_old_region_cells += 1
                else:
                    reference_changes_outside_regions += 1
                if incremental_value != baseline_value:
                    candidate_changes_outside_regions += 1

    overlap_percent = _percent(overlap_cells, total_reference_cells)
    changed_recall_percent = _percent(
        recovered_changed_cells, changed_reference_cells
    )
    changed_agreement_percent = _percent(
        changed_region_agreement_cells, declared_region_cells
    )
    old_regression_percent = _percent(
        regressed_old_region_cells, stable_old_region_cells
    )
    time_improvement = incremental_time_improvement_percent(
        spec.full_mapping_time_sec, spec.incremental_mapping_time_sec
    )
    thresholds = spec.thresholds
    checks = {
        "time_improvement": (
            time_improvement >= thresholds.minimum_time_improvement_percent
        ),
        "reference_overlap": (
            overlap_percent is not None
            and overlap_percent >= thresholds.minimum_reference_overlap_percent
        ),
        "changed_cells_observed": (
            changed_reference_cells >= thresholds.minimum_changed_cell_count
        ),
        "changed_cell_recall": (
            changed_recall_percent is not None
            and changed_recall_percent
            >= thresholds.minimum_changed_cell_recall_percent
        ),
        "changed_region_agreement": (
            changed_agreement_percent is not None
            and changed_agreement_percent
            >= thresholds.minimum_changed_region_agreement_percent
        ),
        "old_region_regression": (
            old_regression_percent is not None
            and old_regression_percent
            <= thresholds.maximum_old_region_regression_percent
        ),
    }
    report: dict[str, Any] = {
        "schema_version": 1,
        "report_type": "incremental_occupancy_map_comparison",
        "scope_note": (
            "This report evaluates saved maps and mapping duration only; "
            "post-update Localization and Nav2 require separate runtime evidence."
        ),
        "source_spec": {
            "path": str(spec.source_path),
            "sha256": _sha256(spec.source_path),
        },
        "artifacts": {
            "baseline": _map_record(baseline),
            "full_remap": _map_record(full),
            "incremental": _map_record(incremental),
        },
        "timing_sec": {
            "full_mapping": spec.full_mapping_time_sec,
            "incremental_mapping": spec.incremental_mapping_time_sec,
        },
        "changed_regions": [region.as_dict() for region in spec.changed_regions],
        "counts": {
            "full_reference_cells": total_reference_cells,
            "three_map_overlap_cells": overlap_cells,
            "declared_region_cells": declared_region_cells,
            "reference_changed_cells_in_declared_regions": changed_reference_cells,
            "recovered_changed_cells": recovered_changed_cells,
            "stable_old_region_cells": stable_old_region_cells,
            "regressed_old_region_cells": regressed_old_region_cells,
            "reference_changes_outside_declared_regions": (
                reference_changes_outside_regions
            ),
            "candidate_changes_outside_declared_regions": (
                candidate_changes_outside_regions
            ),
        },
        "metrics": {
            "time_improvement_percent": time_improvement,
            "three_map_overlap_percent": overlap_percent,
            "changed_cell_recall_percent": changed_recall_percent,
            "changed_region_agreement_percent": changed_agreement_percent,
            "old_region_regression_percent": old_regression_percent,
        },
        "thresholds": asdict(thresholds),
        "checks": checks,
        "map_update_comparison_accepted": all(checks.values()),
    }
    return ComparisonResult(report=report)


def write_comparison_report(
    result: ComparisonResult, output_path: str | Path
) -> Path:
    """Atomically write one strict JSON comparison report."""
    output = Path(output_path).expanduser().resolve()

    def writer(stream) -> None:
        json.dump(result.report, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")

    _atomic_text_write(output, writer)
    return output


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Compare baseline, full-remap, and incremental OccupancyGrid artifacts"
        )
    )
    parser.add_argument("--spec", required=True, help="comparison YAML specification")
    parser.add_argument("--output", required=True, help="strict JSON report path")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run the installed CLI; return 0 accepted, 2 rejected, or 1 invalid."""
    arguments = _argument_parser().parse_args(argv)
    try:
        spec = load_comparison_spec(arguments.spec)
        result = compare_incremental_map(spec)
        output = write_comparison_report(result, arguments.output)
    except (ConfigurationError, OSError, ValueError) as exc:
        print(f"incremental map comparison failed: {exc}", file=sys.stderr)
        return 1
    print(output)
    if result.accepted:
        print("incremental map comparison accepted")
        return 0
    failed = [name for name, passed in result.report["checks"].items() if not passed]
    print(
        "incremental map comparison rejected; failed checks: " + ", ".join(failed),
        file=sys.stderr,
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
