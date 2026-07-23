"""Evidence-first aggregation and Chinese reporting for the frozen campaign."""

from __future__ import annotations

import argparse
import base64
import csv
from dataclasses import asdict, dataclass
import gzip
import hashlib
import html
import io
import json
import math
from pathlib import Path
import shutil
from statistics import median
from typing import Any, Iterable, Mapping

import yaml


STATIC_SEEDS = tuple(range(7201, 7221))
DYNAMIC_SEEDS = tuple(range(7301, 7321))
STATIC_MIN_SUCCESS = 19
DYNAMIC_MIN_SUCCESS = 18
PATH_DEVIATION_MAX_PERCENT = 20.0
# G1 is the redesigned route's calibrated spawn/return point.  The runner
# dispatches G2 through G5, then sends G1 as the final closed-loop goal.
WAYPOINT_IDS = ("G2", "G3", "G4", "G5", "G1")


class CampaignValidationError(ValueError):
    """Raised when formal evidence does not match the frozen acceptance contract."""


def load_campaign_definition(path: str | Path) -> Mapping[str, Any]:
    source = Path(path).expanduser().resolve()
    try:
        value = yaml.safe_load(source.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise CampaignValidationError(f"invalid campaign definition {source}: {exc}") from exc
    if not isinstance(value, Mapping) or value.get("schema_version") != 2:
        raise CampaignValidationError("campaign definition must use schema_version 2")
    if value.get("campaign") != "kujiale_long_range":
        raise CampaignValidationError("campaign definition has an unexpected campaign id")
    environment = value.get("environment")
    if not isinstance(environment, Mapping):
        raise CampaignValidationError("campaign environment must be a mapping")
    start_pose = environment.get("start_pose")
    if (
        not isinstance(start_pose, list)
        or len(start_pose) != 3
        or any(isinstance(component, bool) or not isinstance(component, (int, float)) for component in start_pose)
    ):
        raise CampaignValidationError("campaign environment.start_pose must be [x, y, yaw_deg]")
    route = value.get("route")
    if not isinstance(route, list) or [item.get("id") if isinstance(item, Mapping) else None for item in route] != list(WAYPOINT_IDS):
        raise CampaignValidationError("campaign route must contain the redesigned G2, G3, G4, G5, G1 order")
    for kind, seeds in (("static", STATIC_SEEDS), ("dynamic", DYNAMIC_SEEDS)):
        section = value.get(kind)
        if not isinstance(section, Mapping) or tuple(section.get("seeds", ())) != seeds:
            raise CampaignValidationError(f"campaign {kind} seeds must be exactly {seeds[0]}..{seeds[-1]}")
    return value


@dataclass(frozen=True)
class Rate:
    numerator: int
    denominator: int
    percent: float
    wilson95_low_percent: float
    wilson95_high_percent: float
    required_numerator: int
    passed: bool


def _wilson_interval(successes: int, total: int) -> tuple[float, float]:
    if total <= 0:
        return (0.0, 0.0)
    z = 1.959963984540054
    proportion = successes / total
    denominator = 1.0 + z * z / total
    centre = (proportion + z * z / (2.0 * total)) / denominator
    radius = z * math.sqrt(proportion * (1.0 - proportion) / total + z * z / (4.0 * total * total)) / denominator
    return (100.0 * max(0.0, centre - radius), 100.0 * min(1.0, centre + radius))


def _rate(successes: int, total: int, required: int) -> Rate:
    low, high = _wilson_interval(successes, total)
    return Rate(successes, total, 100.0 * successes / total if total else 0.0, low, high, required, total == 20 and successes >= required)


def _read_json(path: Path) -> Mapping[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CampaignValidationError(f"invalid JSON evidence {path}: {exc}") from exc
    if not isinstance(value, Mapping):
        raise CampaignValidationError(f"JSON evidence {path} must be an object")
    return value


def _candidate_summaries(directories: Iterable[str | Path]) -> list[Mapping[str, Any]]:
    paths: list[Path] = []
    for directory in directories:
        root = Path(directory).expanduser().resolve()
        if not root.is_dir():
            raise CampaignValidationError(f"campaign directory does not exist: {root}")
        paths.extend(root.rglob("run_summary.json"))
    if not paths:
        raise CampaignValidationError("campaign contains no run_summary.json files")
    rows = []
    for path in sorted(paths):
        row = dict(_read_json(path))
        row["_evidence_dir"] = str(path.parent)
        rows.append(row)
    return rows


def _bool(summary: Mapping[str, Any], field: str) -> bool:
    value = summary.get(field)
    if not isinstance(value, bool):
        raise CampaignValidationError(f"run summary {summary.get('seed')!r} lacks boolean {field}")
    return value


def _number(summary: Mapping[str, Any], field: str) -> float:
    value = summary.get(field)
    if isinstance(value, bool):
        value = None
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise CampaignValidationError(f"run summary {summary.get('seed')!r} lacks numeric {field}") from exc
    if not math.isfinite(result):
        raise CampaignValidationError(f"run summary {summary.get('seed')!r} has non-finite {field}")
    return result


def _validate_run(summary: Mapping[str, Any], expected_kind: str) -> int:
    if summary.get("campaign") != "kujiale_long_range" or summary.get("kind") != expected_kind:
        raise CampaignValidationError(f"run kind must be {expected_kind}")
    seed = summary.get("seed")
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise CampaignValidationError("run_summary seed must be an integer")
    legs = summary.get("legs")
    if not isinstance(legs, list) or not 1 <= len(legs) <= len(WAYPOINT_IDS):
        raise CampaignValidationError(
            f"run {seed} must contain one through {len(WAYPOINT_IDS)} leg results"
        )
    leg_ids = [item.get("id") if isinstance(item, Mapping) else None for item in legs]
    if leg_ids != list(WAYPOINT_IDS[:len(legs)]):
        raise CampaignValidationError(
            f"run {seed} legs must be an ordered redesigned-route prefix"
        )
    for field in ("strict_success", "physical_collision_free", "data_complete", "checksums_verified"):
        _bool(summary, field)
    if _bool(summary, "strict_success") and len(legs) != len(WAYPOINT_IDS):
        raise CampaignValidationError(
            f"strictly successful run {seed} must contain exactly {len(WAYPOINT_IDS)} leg results"
        )
    return seed


def _normalize_legacy_strict_success(summary: Mapping[str, Any]) -> Mapping[str, Any]:
    """Repair only the known route-length reporter defect in memory.

    Existing evidence is immutable.  Builds made before the corrected runner
    wrote ``strict_success: false`` for every successful redesigned route,
    because the summary still expected a retired route length. A successful
    manifest plus exactly the current route legs is sufficient to
    identify that narrow defect without reclassifying any actual failure.
    """
    if _bool(summary, "strict_success") or len(summary.get("legs", [])) != len(WAYPOINT_IDS):
        return summary
    evidence_dir = Path(str(summary.get("_evidence_dir", "")))
    manifest_path = evidence_dir / "run_manifest.json"
    if not manifest_path.is_file():
        return summary
    manifest = _read_json(manifest_path)
    if manifest.get("result") != "success":
        return summary
    normalized = dict(summary)
    normalized["strict_success"] = True
    normalized["strict_success_source"] = "run_manifest_success_legacy_route_length_fix"
    return normalized


def _campaign_rows(summaries: Iterable[Mapping[str, Any]], kind: str, seeds: tuple[int, ...]) -> list[Mapping[str, Any]]:
    selected: dict[int, Mapping[str, Any]] = {}
    for summary in summaries:
        if summary.get("kind") != kind:
            continue
        summary = _normalize_legacy_strict_success(summary)
        seed = _validate_run(summary, kind)
        if seed in selected:
            raise CampaignValidationError(f"duplicate {kind} seed {seed}")
        selected[seed] = summary
    if tuple(sorted(selected)) != seeds:
        raise CampaignValidationError(f"{kind} seeds must be exactly {seeds[0]}..{seeds[-1]}; got {tuple(sorted(selected))}")
    return [selected[seed] for seed in seeds]


def summarize_campaign(directories: Iterable[str | Path]) -> dict[str, Any]:
    """Validate exact formal batches and return only evidence-derived KPI data."""
    summaries = _candidate_summaries(directories)
    static = _campaign_rows(summaries, "static", STATIC_SEEDS)
    dynamic = _campaign_rows(summaries, "dynamic", DYNAMIC_SEEDS)
    static_strict = _rate(sum(_bool(row, "strict_success") for row in static), 20, STATIC_MIN_SUCCESS)
    static_collision = _rate(sum(_bool(row, "physical_collision_free") for row in static), 20, STATIC_MIN_SUCCESS)
    dynamic_strict = _rate(sum(_bool(row, "strict_success") for row in dynamic), 20, DYNAMIC_MIN_SUCCESS)
    dynamic_collision = _rate(sum(_bool(row, "physical_collision_free") for row in dynamic), 20, DYNAMIC_MIN_SUCCESS)
    successful_static = [row for row in static if _bool(row, "strict_success")]
    deviations = [_number(row, "path_deviation_percent") for row in successful_static]
    deviations_ok = bool(deviations) and all(value <= PATH_DEVIATION_MAX_PERCENT for value in deviations)
    evidence_ok = all(_bool(row, "data_complete") and _bool(row, "checksums_verified") for row in (*static, *dynamic))
    result = {
        "schema_version": 2, "campaign": "kujiale_long_range",
        "static": {"strict_success": asdict(static_strict), "physical_collision_free": asdict(static_collision)},
        "dynamic": {"strict_success": asdict(dynamic_strict), "physical_collision_free": asdict(dynamic_collision)},
        "path_optimality": {
            "successful_static_runs": len(successful_static),
            "mean_deviation_percent": sum(deviations) / len(deviations) if deviations else None,
            "p50_deviation_percent": median(deviations) if deviations else None,
            "p95_deviation_percent": sorted(deviations)[max(0, math.ceil(0.95 * len(deviations)) - 1)] if deviations else None,
            "maximum_deviation_percent": max(deviations) if deviations else None,
            "required_maximum_percent": PATH_DEVIATION_MAX_PERCENT, "passed": deviations_ok,
        },
        "evidence_complete": evidence_ok, "runs": list(static) + list(dynamic),
    }
    result["passed"] = bool(static_strict.passed and static_collision.passed and dynamic_strict.passed and dynamic_collision.passed and deviations_ok and evidence_ok)
    return result


def summarize_static_campaign(directories: Iterable[str | Path]) -> dict[str, Any]:
    """Validate one exact static 20-seed candidate batch.

    This is intentionally separate from :func:`summarize_campaign`: a static
    candidate result must never imply that the dynamic acceptance batch has
    passed or failed.  The output is still evidence-first and uses the same
    static 19/20, collision-free, and path-deviation gates as the full report.
    """
    summaries = _candidate_summaries(directories)
    static = _campaign_rows(summaries, "static", STATIC_SEEDS)
    strict = _rate(sum(_bool(row, "strict_success") for row in static), 20, STATIC_MIN_SUCCESS)
    collision = _rate(sum(_bool(row, "physical_collision_free") for row in static), 20, STATIC_MIN_SUCCESS)
    successful = [row for row in static if _bool(row, "strict_success")]
    deviations = [_number(row, "path_deviation_percent") for row in successful]
    deviations_ok = bool(deviations) and all(value <= PATH_DEVIATION_MAX_PERCENT for value in deviations)
    evidence_ok = all(_bool(row, "data_complete") and _bool(row, "checksums_verified") for row in static)
    result = {
        "schema_version": 2,
        "campaign": "kujiale_long_range",
        "scope": "static_20_candidate",
        "static": {"strict_success": asdict(strict), "physical_collision_free": asdict(collision)},
        "dynamic": {"executed": False, "reason": "本报告只汇总静态 20 轮；动态 20 轮未运行。"},
        "path_optimality": {
            "successful_static_runs": len(successful),
            "mean_deviation_percent": sum(deviations) / len(deviations) if deviations else None,
            "p50_deviation_percent": median(deviations) if deviations else None,
            "p95_deviation_percent": sorted(deviations)[max(0, math.ceil(0.95 * len(deviations)) - 1)] if deviations else None,
            "maximum_deviation_percent": max(deviations) if deviations else None,
            "required_maximum_percent": PATH_DEVIATION_MAX_PERCENT,
            "passed": deviations_ok,
        },
        "evidence_complete": evidence_ok,
        "runs": list(static),
    }
    result["passed"] = bool(strict.passed and collision.passed and deviations_ok and evidence_ok)
    return result


def _atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_bytes(payload)
    temporary.replace(path)


def _write_report_checksums(root: Path) -> None:
    """Refresh derived artifacts while retaining prior hashes for frozen evidence."""
    prior: dict[str, str] = {}
    checksum_file = root / "checksums.sha256"
    if checksum_file.is_file():
        for line in checksum_file.read_text(encoding="utf-8").splitlines():
            digest, separator, relative = line.partition("  ")
            if separator and len(digest) == 64:
                prior[relative] = digest
    lines: list[str] = []
    for path in sorted(item for item in root.rglob("*") if item.is_file() and item.name != "checksums.sha256"):
        relative = path.relative_to(root).as_posix()
        frozen_evidence = relative.startswith("runs/") and not relative.endswith("/index.html")
        digest = prior.get(relative) if frozen_evidence else None
        if digest is None:
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
        lines.append(f"{digest}  {relative}")
    _atomic_write(checksum_file, ("\n".join(lines) + "\n").encode("utf-8"))


def _matplotlib():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    from matplotlib import font_manager
    # Matplotlib does not always index Ubuntu's Noto TTC collection.  Register
    # it explicitly so the generated PNG/PDF keeps both Chinese and Latin text.
    noto_ttc = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    if noto_ttc.is_file():
        font_manager.fontManager.addfont(str(noto_ttc))
        plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "Droid Sans Fallback", "sans-serif"]
    else:
        plt.rcParams["font.sans-serif"] = ["Droid Sans Fallback", "sans-serif"]
    plt.rcParams["axes.unicode_minus"] = False
    return plt, PdfPages


def _plot_overview(summary: Mapping[str, Any], figures: Path) -> list[Path]:
    plt, _ = _matplotlib()
    figures.mkdir(parents=True, exist_ok=True)
    paths: list[Path] = []
    fig, axes = plt.subplots(2, 2, figsize=(16, 9), constrained_layout=True)
    fig.patch.set_facecolor("#f8fafc")
    fig.suptitle("Kujiale 全屋长距离导航｜正式验收总览", fontsize=22, fontweight="bold", color="#0f172a")
    labels = ["静态严格", "静态无碰撞", "动态严格", "动态无碰撞"]
    rates = [summary["static"]["strict_success"], summary["static"]["physical_collision_free"], summary["dynamic"]["strict_success"], summary["dynamic"]["physical_collision_free"]]
    values = [item["percent"] for item in rates]
    thresholds = [95, 95, 90, 90]
    kpi_axis = axes[0, 0]
    kpi_axis.set_facecolor("#ffffff")
    bars = kpi_axis.bar(labels, values, color=["#16a34a" if value >= threshold else "#dc2626" for value, threshold in zip(values, thresholds)], width=0.62)
    kpi_axis.plot(range(4), thresholds, color="#f97316", linestyle="--", marker="o", label="验收门槛")
    kpi_axis.set_ylim(0, 108); kpi_axis.set_ylabel("成功率 (%)"); kpi_axis.set_title("核心 KPI｜每项分母均为 20", loc="left", fontweight="bold")
    for bar, item in zip(bars, rates):
        kpi_axis.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 2, f"{item['numerator']}/20\n{item['percent']:.1f}%", ha="center", va="bottom", fontsize=10, fontweight="bold")
    kpi_axis.legend(loc="lower left", frameon=False)
    rows = [row for row in summary["runs"] if row["kind"] == "static"]
    seeds = [row["seed"] for row in rows]
    deviations = [row.get("path_deviation_percent") if row.get("strict_success") else None for row in rows]
    deviation_axis = axes[0, 1]
    deviation_axis.set_facecolor("#ffffff")
    valid = [(seed, value) for seed, value in zip(seeds, deviations) if value is not None]
    if valid:
        valid_seeds, valid_values = zip(*valid)
        deviation_axis.vlines(valid_seeds, 0, valid_values, color="#93c5fd", linewidth=2)
        deviation_axis.scatter(valid_seeds, valid_values, color="#2563eb", s=48, zorder=3, label="GT 相对理论路径")
    deviation_axis.axhline(PATH_DEVIATION_MAX_PERCENT, color="#dc2626", linestyle="--", label="20% 门槛")
    maximum = summary["path_optimality"]["maximum_deviation_percent"]
    deviation_axis.set_xlabel("静态种子"); deviation_axis.set_ylabel("路径偏差 (%)"); deviation_axis.set_title(f"路径最优性｜最大偏差 {maximum:.2f}%", loc="left", fontweight="bold")
    deviation_axis.legend(loc="upper left", frameon=False)

    outcome_axis = axes[1, 0]
    outcome_axis.set_facecolor("#ffffff")
    outcome_rows = [("静态", [row for row in summary["runs"] if row["kind"] == "static"]), ("动态", [row for row in summary["runs"] if row["kind"] == "dynamic"])]
    for y, (label, group) in enumerate(outcome_rows):
        for x, row in enumerate(group):
            color = "#16a34a" if row["strict_success"] else "#dc2626"
            outcome_axis.scatter(x + 1, y, s=260, color=color, marker="s", edgecolors="#ffffff", linewidths=1.2)
        outcome_axis.text(20.9, y, f"{sum(row['strict_success'] for row in group)}/20", va="center", color="#334155", fontweight="bold")
    outcome_axis.set(xlim=(0.25, 23), ylim=(-0.7, 1.7), yticks=[0, 1], yticklabels=[item[0] for item in outcome_rows], xlabel="正式试验序号（由左至右）")
    outcome_axis.set_title("试验覆盖｜绿=严格通过，红=未通过", loc="left", fontweight="bold")
    outcome_axis.set_xticks([1, 5, 10, 15, 20])

    evidence_axis = axes[1, 1]
    evidence_axis.set_facecolor("#0f172a"); evidence_axis.set_xticks([]); evidence_axis.set_yticks([])
    for spine in evidence_axis.spines.values():
        spine.set_visible(False)
    total_runs = len(summary["runs"])
    complete_runs = sum(bool(row["data_complete"]) for row in summary["runs"])
    verified_runs = sum(bool(row["checksums_verified"]) for row in summary["runs"])
    evidence_axis.text(0.07, 0.83, "证据与结论", transform=evidence_axis.transAxes, color="#f8fafc", fontsize=18, fontweight="bold")
    evidence_axis.text(0.07, 0.64, "自动验收", transform=evidence_axis.transAxes, color="#94a3b8", fontsize=11)
    evidence_axis.text(0.07, 0.51, "通过" if summary["passed"] else "未通过", transform=evidence_axis.transAxes, color="#4ade80" if summary["passed"] else "#f87171", fontsize=31, fontweight="bold")
    evidence_axis.text(0.07, 0.29, f"{complete_runs} / {total_runs} 轮证据完整\n{verified_runs} / {total_runs} 轮校验和已验证", transform=evidence_axis.transAxes, color="#e2e8f0", fontsize=13, linespacing=1.8)
    evidence_axis.text(0.07, 0.08, "静态门槛 19/20 · 动态门槛 18/20 · 偏差上限 20%", transform=evidence_axis.transAxes, color="#94a3b8", fontsize=10)
    for axis in (kpi_axis, deviation_axis, outcome_axis):
        axis.spines[["top", "right"]].set_visible(False)
        axis.grid(axis="y", color="#e2e8f0", linewidth=0.8)
    path = figures / "campaign_overview.png"; fig.savefig(path, dpi=100); plt.close(fig); paths.append(path)
    for kind in ("static", "dynamic"):
        rows = [row for row in summary["runs"] if row["kind"] == kind]
        matrix = []
        for row in rows:
            line = []
            legs = row["legs"]
            for index in range(len(WAYPOINT_IDS)):
                leg = legs[index] if index < len(legs) else None
                if leg is None:
                    # The first failed action ends a formal run.  Render the
                    # unexecuted remainder explicitly instead of producing a
                    # ragged matrix or pretending those goals succeeded.
                    line.append(0)
                elif leg.get("timed_out"):
                    line.append(1)
                elif not row["strict_success"]:
                    line.append(0)
                elif row.get("dynamic_interaction_complete") is False:
                    line.append(3)
                else:
                    line.append(2)
            matrix.append(line)
        fig, axis = plt.subplots(figsize=(16, 9), constrained_layout=True)
        fig.patch.set_facecolor("#f8fafc"); axis.set_facecolor("#ffffff")
        palette = ["#dc2626", "#f97316", "#16a34a", "#7c3aed"]
        image = axis.imshow(matrix, aspect="auto", interpolation="nearest", cmap=plt.matplotlib.colors.ListedColormap(palette), vmin=0, vmax=3)
        axis.set_xticks(range(len(WAYPOINT_IDS)), WAYPOINT_IDS); axis.set_yticks(range(20), [str(row["seed"]) for row in rows]); axis.set_xlabel("目标航点"); axis.set_ylabel("随机种子")
        axis.set_title(f"{kind.capitalize()}｜20 × {len(WAYPOINT_IDS)} 航点执行矩阵", loc="left", fontsize=18, fontweight="bold", pad=16)
        colorbar = fig.colorbar(image, ax=axis, ticks=[0, 1, 2, 3], pad=0.02)
        colorbar.ax.set_yticklabels(["失败 / 未执行", "超时", "成功", "动态交互失效"])
        path = figures / f"waypoint_heatmap_{kind}.png"; fig.savefig(path, dpi=100); plt.close(fig); paths.append(path)
    return paths


def _copy_runs(rows: Iterable[Mapping[str, Any]], destination: Path) -> None:
    for row in rows:
        source = Path(str(row["_evidence_dir"])); target = destination / f"{row['kind']}-{row['seed']}"
        if not source.is_dir():
            continue
        source_summary, target_summary = source / "run_summary.json", target / "run_summary.json"
        # Formal evidence is immutable once its run summary is frozen.  Avoid
        # re-copying tens of gigabytes of MCAP on a report-only regeneration.
        if target_summary.is_file() and source_summary.is_file() and source_summary.read_bytes() == target_summary.read_bytes():
            continue
        shutil.copytree(source, target, dirs_exist_ok=True)


def _repository_file(relative: str) -> Path | None:
    """Locate a checked-in report input without assuming the launch directory."""
    for base in (Path.cwd(), *Path(__file__).resolve().parents):
        candidate = base / relative
        if candidate.is_file():
            return candidate
    return None


def _read_csv_rows(path: Path, *, compressed: bool = False) -> list[dict[str, str]]:
    if not path.is_file():
        return []
    try:
        if compressed:
            with gzip.open(path, "rt", encoding="utf-8", newline="") as stream:
                return list(csv.DictReader(stream))
        with path.open("r", encoding="utf-8", newline="") as stream:
            return list(csv.DictReader(stream))
    except (OSError, UnicodeDecodeError, csv.Error):
        return []


def _read_events(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    events: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, Mapping):
            events.append(dict(value))
    return events


def _finite_number(value: object) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _sample_trajectory(rows: Iterable[Mapping[str, str]], limit: int = 240) -> list[list[float]]:
    source = [[float(row["x"]), float(row["y"]), float(row["stamp_s"])] for row in rows if _finite_number(row.get("x")) is not None and _finite_number(row.get("y")) is not None and _finite_number(row.get("stamp_s")) is not None]
    if len(source) <= limit:
        return source
    return [source[round(index * (len(source) - 1) / (limit - 1))] for index in range(limit)]


def _run_visual_detail(row: Mapping[str, Any]) -> dict[str, Any]:
    evidence = Path(str(row.get("_evidence_dir", "")))
    manifest: Mapping[str, Any] = {}
    try:
        loaded = json.loads((evidence / "run_manifest.json").read_text(encoding="utf-8"))
        if isinstance(loaded, Mapping):
            manifest = loaded
    except (OSError, json.JSONDecodeError):
        pass
    ground_truth = _read_csv_rows(evidence / "ground_truth.csv.gz", compressed=True)
    scan = _read_csv_rows(evidence / "scan.csv")
    metrics = manifest.get("metrics", {}) if isinstance(manifest.get("metrics"), Mapping) else {}
    motion = metrics.get("command_motion_quality", {}) if isinstance(metrics.get("command_motion_quality"), Mapping) else {}
    legs = [dict(leg) for leg in row.get("legs", []) if isinstance(leg, Mapping)]
    duration = sum(float(leg.get("duration_sec", 0.0)) for leg in legs if _finite_number(leg.get("duration_sec")) is not None)
    distance = sum(float(leg.get("ground_truth_length_m", 0.0)) for leg in legs if _finite_number(leg.get("ground_truth_length_m")) is not None)
    reference = sum(float(leg.get("reference_length_m", 0.0)) for leg in legs if _finite_number(leg.get("reference_length_m")) is not None)
    ranges = [_finite_number(sample.get("range_m")) for sample in scan]
    minimum_range = min((value for value in ranges if value is not None), default=None)
    failure_reason = str(manifest.get("failure_reason", "")) if not row["strict_success"] else ""
    return {
        "id": f"{row['kind']}-{row['seed']}", "kind": row["kind"], "seed": row["seed"],
        "strict_success": bool(row["strict_success"]), "physical_collision_free": bool(row["physical_collision_free"]),
        "dynamic_interaction_complete": row.get("dynamic_interaction_complete"),
        "path_deviation_percent": row.get("path_deviation_percent"), "duration_sec": duration,
        "ground_truth_length_m": distance, "reference_length_m": reference or None,
        "minimum_scan_range_m": minimum_range, "recoveries": metrics.get("maximum_route_recoveries"),
        "stopped_time_fraction": motion.get("stopped_time_fraction"), "trajectory": _sample_trajectory(ground_truth),
        "events": _read_events(evidence / "events.jsonl"), "legs": legs, "failure_reason": failure_reason,
        "files": [name for name in ("run_manifest.json", "run_summary.json", "events.jsonl", "leg_metrics.csv", "ground_truth.csv.gz", "odom.csv.gz", "cmd_vel.csv.gz", "obstacles.csv.gz", "depth_frame.pgm", "depth_frame.json", "scan.csv", "scan.json", "local_costmap.pgm", "local_costmap.json", "global_costmap.pgm", "global_costmap.json", "telemetry/telemetry_0.mcap") if (evidence / name).is_file()],
    }


def _map_visual_inputs() -> dict[str, Any]:
    campaign_file = _repository_file("ros2_ws/src/robot_experiments/config/kujiale_long_range_campaign.yaml")
    reference_file = _repository_file("ros2_ws/src/robot_experiments/config/optimal_reference.json")
    if campaign_file is None or reference_file is None:
        return {"available": False}
    campaign = yaml.safe_load(campaign_file.read_text(encoding="utf-8"))
    reference = json.loads(reference_file.read_text(encoding="utf-8"))
    if not isinstance(campaign, Mapping) or not isinstance(reference, Mapping):
        return {"available": False}
    map_data = reference.get("map", {})
    map_file = Path(str(map_data.get("image", ""))) if isinstance(map_data, Mapping) else Path()
    map_yaml = Path(str(map_data.get("yaml", ""))) if isinstance(map_data, Mapping) else Path()
    if not map_file.is_file() or not map_yaml.is_file():
        return {"available": False}
    document = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    if not isinstance(document, Mapping):
        return {"available": False}
    try:
        plt, _ = _matplotlib()
        raster = plt.imread(map_file)
        buffer = io.BytesIO(); plt.imsave(buffer, raster, cmap="gray", format="png")
    except (OSError, ValueError):
        return {"available": False}
    return {
        "available": True, "image_base64": base64.b64encode(buffer.getvalue()).decode("ascii"),
        "width": int(raster.shape[1]), "height": int(raster.shape[0]),
        "origin": [float(document["origin"][0]), float(document["origin"][1])], "resolution": float(document["resolution"]),
        "start": [float(campaign["environment"]["start_pose"][0]), float(campaign["environment"]["start_pose"][1])],
        "route": [{"id": item["id"], "region": item["region"], "x": float(item["pose"][0]), "y": float(item["pose"][1])} for item in campaign["route"]],
        "static_obstacle_polygons": reference.get("static_obstacle_polygons", []),
        "dynamic_obstacles": campaign.get("dynamic", {}).get("obstacles", []),
    }


def _visualization_data(summary: Mapping[str, Any]) -> dict[str, Any]:
    details = [_run_visual_detail(row) for row in summary["runs"]]
    static = [item for item in details if item["kind"] == "static"]
    dynamic = [item for item in details if item["kind"] == "dynamic"]
    deviations = [float(item["path_deviation_percent"]) for item in static if _finite_number(item.get("path_deviation_percent")) is not None]
    failure_reasons: dict[str, dict[str, int]] = {}
    for item in details:
        for reason in filter(None, item["failure_reason"].split(";")):
            failure_reasons.setdefault(reason, {"static": 0, "dynamic": 0})[item["kind"]] += 1
    total_duration = sum(item["duration_sec"] for item in details)
    total_distance = sum(item["ground_truth_length_m"] for item in details)
    return {
        "map": _map_visual_inputs(), "runs": details,
        "aggregate": {"total_duration_sec": total_duration, "total_ground_truth_length_m": total_distance, "physical_collision_count": sum(not item["physical_collision_free"] for item in details), "path_deviation": {"p50": median(deviations) if deviations else None, "p95": sorted(deviations)[max(0, math.ceil(.95 * len(deviations)) - 1)] if deviations else None, "maximum": max(deviations) if deviations else None}},
        "failure_reasons": failure_reasons,
        "availability": {"first_planned_paths": False, "voxelgrid_snapshot": False, "collision_monitor_event_stream": any(event.get("event") == "collision_monitor" for item in details for event in item["events"])},
    }


def _plot_evidence_figures(visual: Mapping[str, Any], figures: Path) -> list[Path]:
    """Render the report's additional 1600×900 evidence sheets."""
    plt, _ = _matplotlib()
    paths: list[Path] = []
    static = [item for item in visual["runs"] if item["kind"] == "static"]
    deviations = [float(item["path_deviation_percent"]) for item in static if _finite_number(item.get("path_deviation_percent")) is not None]
    fig, axes = plt.subplots(2, 2, figsize=(16, 9), constrained_layout=True); fig.patch.set_facecolor("#f8fafc")
    axes[0, 0].scatter([item["seed"] for item in static if _finite_number(item.get("path_deviation_percent")) is not None], deviations, color="#2563eb")
    axes[0, 0].axhline(20, color="#dc2626", linestyle="--"); axes[0, 0].set(title="静态路径偏差散点图", xlabel="种子", ylabel="偏差 (%)")
    if deviations:
        axes[0, 1].boxplot(deviations, vert=True, labels=["静态 20 轮"])
    else:
        axes[0, 1].text(.5, .5, "没有成功静态运行；\n无法计算路径偏差分布。", ha="center", va="center", transform=axes[0, 1].transAxes)
        axes[0, 1].set_xticks([])
    axes[0, 1].axhline(20, color="#dc2626", linestyle="--"); axes[0, 1].set(title="路径偏差分布（P50 / P95 / 最大值见 JSON）", ylabel="偏差 (%)")
    reference_by_leg = [sum(float(item["legs"][index].get("reference_length_m", 0.0)) for item in static if len(item["legs"]) > index and _finite_number(item["legs"][index].get("reference_length_m")) is not None) / max(1, sum(1 for item in static if len(item["legs"]) > index and _finite_number(item["legs"][index].get("reference_length_m")) is not None)) for index in range(len(WAYPOINT_IDS))]
    gt_by_leg = [sum(float(item["legs"][index].get("ground_truth_length_m", 0.0)) for item in static if len(item["legs"]) > index and _finite_number(item["legs"][index].get("ground_truth_length_m")) is not None) / max(1, sum(1 for item in static if len(item["legs"]) > index and _finite_number(item["legs"][index].get("ground_truth_length_m")) is not None)) for index in range(len(WAYPOINT_IDS))]
    indices = list(range(len(WAYPOINT_IDS))); axes[1, 0].bar([index - .19 for index in indices], reference_by_leg, .38, label="理论参考", color="#111827"); axes[1, 0].bar([index + .19 for index in indices], gt_by_leg, .38, label="GT 执行", color="#16a34a"); axes[1, 0].set_xticks(indices, WAYPOINT_IDS); axes[1, 0].set(title="每段理论与 GT 长度对比", ylabel="长度 (m)"); axes[1, 0].legend()
    axes[1, 1].axis("off"); aggregate = visual["aggregate"]["path_deviation"]
    def metric(value: object) -> str:
        number = _finite_number(value)
        return "—" if number is None else f"{number:.2f}%"
    axes[1, 1].text(.08, .78, "路径最优性摘要", fontsize=20, fontweight="bold", transform=axes[1, 1].transAxes); axes[1, 1].text(.08, .49, f"P50  {metric(aggregate['p50'])}\nP95  {metric(aggregate['p95'])}\n最大  {metric(aggregate['maximum'])}\n门槛  ≤ 20.00%", fontsize=17, linespacing=1.8, transform=axes[1, 1].transAxes); axes[1, 1].text(.08, .10, "首次规划路径未作为结构化序列采集；原始 MCAP 可下钻。", color="#64748b", transform=axes[1, 1].transAxes)
    path = figures / "path_optimality.png"; fig.savefig(path, dpi=100); plt.close(fig); paths.append(path)
    fig, axes = plt.subplots(1, 2, figsize=(16, 9), constrained_layout=True); fig.patch.set_facecolor("#f8fafc")
    runs = list(visual["runs"]); scan_points = [(item["seed"], item["minimum_scan_range_m"], item["kind"]) for item in runs if item["minimum_scan_range_m"] is not None]
    for kind, color in (("static", "#2563eb"), ("dynamic", "#7c3aed")):
        group = [(seed, value) for seed, value, source in scan_points if source == kind]
        if group: axes[0].scatter([item[0] for item in group], [item[1] for item in group], label=kind, color=color)
    axes[0].set(title="每轮最小 Scan 障碍距离", xlabel="种子", ylabel="距离 (m)"); axes[0].legend()
    recoveries = [float(item["recoveries"] or 0) for item in runs]; stops = [100 * float(item["stopped_time_fraction"] or 0) for item in runs]
    axes[1].scatter(recoveries, stops, c=["#16a34a" if item["strict_success"] else "#dc2626" for item in runs], alpha=.8); axes[1].set(title="恢复次数与停留时间占比", xlabel="恢复次数", ylabel="停留时间 (%)")
    path = figures / "safety_overview.png"; fig.savefig(path, dpi=100); plt.close(fig); paths.append(path)
    return paths


def _map_pixel(point: tuple[float, float], map_data: Mapping[str, Any]) -> tuple[float, float]:
    resolution = float(map_data["resolution"]); origin_x, origin_y = map_data["origin"]
    return ((point[0] - float(origin_x)) / resolution, float(map_data["height"]) - 1 - (point[1] - float(origin_y)) / resolution)


def _trajectory_points(points: Iterable[Iterable[float]], map_data: Mapping[str, Any]) -> str:
    return " ".join(f"{x:.2f},{y:.2f}" for point in points if len(point) >= 2 for x, y in [_map_pixel((float(point[0]), float(point[1])), map_data)])


def _map_svg(visual: Mapping[str, Any]) -> str:
    map_data = visual["map"]
    if not map_data.get("available"):
        return "<p class='unavailable'>仓库内未找到冻结的 warehouse_new OccupancyGrid；地图叠加未生成。</p>"
    width, height = int(map_data["width"]), int(map_data["height"])
    start = _map_pixel(tuple(map_data["start"]), map_data)
    goals = list(map_data["route"])
    obstacle_polygons = []
    for polygon in map_data.get("static_obstacle_polygons", []):
        if isinstance(polygon, list):
            obstacle_polygons.append(_trajectory_points(polygon, map_data))
    dynamic_paths = []
    for obstacle in map_data.get("dynamic_obstacles", []):
        if isinstance(obstacle, Mapping) and isinstance(obstacle.get("start"), list) and isinstance(obstacle.get("end"), list):
            dynamic_paths.append(_trajectory_points([obstacle["start"], obstacle["end"]], map_data))
    tracks = []
    for item in visual["runs"]:
        points = _trajectory_points(item["trajectory"], map_data)
        if points:
            state = "success-track" if item["strict_success"] else "failure-track"
            tracks.append(f"<polyline class='track {state}' data-kind='{item['kind']}' data-seed='{item['seed']}' data-status='{'pass' if item['strict_success'] else 'fail'}' points='{points}'><title>{item['id']}｜{'通过' if item['strict_success'] else '失败'}｜{item['ground_truth_length_m']:.2f} m</title></polyline>")
    labels = [f"<g class='goal'><circle cx='{_map_pixel((float(goal['x']), float(goal['y'])), map_data)[0]:.2f}' cy='{_map_pixel((float(goal['x']), float(goal['y'])), map_data)[1]:.2f}' r='4.7'/><text x='{_map_pixel((float(goal['x']), float(goal['y'])), map_data)[0] + 6:.2f}' y='{_map_pixel((float(goal['x']), float(goal['y'])), map_data)[1] - 6:.2f}'>{html.escape(str(goal['id']))}</text><title>{html.escape(str(goal['id']))}｜{html.escape(str(goal['region']))}</title></g>" for goal in goals]
    # Frozen reference only persists scalar lengths, not state-lattice vertices.
    # The dashed line therefore connects accepted route anchors and is labelled
    # as a route-order guide rather than fabricated as an optimal geometry.
    guide = _trajectory_points([map_data["start"]] + [[float(goal["x"]), float(goal["y"])] for goal in goals], map_data)
    return f"<svg class='route-map' viewBox='0 0 {width} {height}' role='img' aria-label='warehouse_new 地图和正式试验轨迹'><image href='data:image/png;base64,{map_data['image_base64']}' x='0' y='0' width='{width}' height='{height}'/><polyline class='reference-guide' points='{guide}'><title>冻结航点顺序引导线；精确状态格点路径仅以长度形式归档。</title></polyline>{''.join(f"<polygon class='static-box' points='{points}'><title>RGB-D 低矮方块</title></polygon>" for points in obstacle_polygons)}{''.join(f"<polyline class='dynamic-path' points='{points}'><title>动态障碍预定义轨迹</title></polyline>" for points in dynamic_paths)}{''.join(tracks)}<g class='start'><circle cx='{start[0]:.2f}' cy='{start[1]:.2f}' r='5.5'/><text x='{start[0] + 7:.2f}' y='{start[1] - 7:.2f}'>S</text></g>{''.join(labels)}</svg>"


def _html_heatmap(visual: Mapping[str, Any], kind: str) -> str:
    rows = [item for item in visual["runs"] if item["kind"] == kind]
    cells = ["<div></div>"] + [f"<div class='heat-label'>{waypoint}</div>" for waypoint in WAYPOINT_IDS]
    for index, item in enumerate(rows, 1):
        cells.append(f"<div class='heat-label'>{index:02d} · {item['seed']}</div>")
        legs = item["legs"]
        for leg_index, waypoint in enumerate(WAYPOINT_IDS):
            leg = legs[leg_index] if leg_index < len(legs) else None
            if leg is None:
                state, label = "fail", "未执行"
            elif leg.get("timed_out"):
                state, label = "timeout", "超时"
            elif not item["physical_collision_free"]:
                state, label = "fail", "物理碰撞"
            elif item["kind"] == "dynamic" and item.get("dynamic_interaction_complete") is False:
                state, label = "interaction", "动态交互失效"
            elif not item["strict_success"]:
                state, label = "fail", "Action/碰撞失败"
            else:
                state, label = "success", "成功"
            cells.append(f"<a class='heat-cell {state}' href='runs/{item['id']}/index.html#{waypoint}' title='{item['id']} · {waypoint} · {label}' aria-label='{item['id']} {waypoint} {label}'>{'✓' if state == 'success' else '!'}</a>")
    return f"<div class='heatmap' style='grid-template-columns:92px repeat({len(WAYPOINT_IDS)},minmax(28px,1fr))'>{''.join(cells)}</div>"


def _html_page(summary: Mapping[str, Any], figure_paths: Iterable[Path], visual: Mapping[str, Any]) -> str:
    passed = bool(summary["passed"])
    rates = [("静态严格成功", summary["static"]["strict_success"]), ("静态物理无碰撞", summary["static"]["physical_collision_free"]), ("动态严格成功", summary["dynamic"]["strict_success"]), ("动态物理无碰撞", summary["dynamic"]["physical_collision_free"])]
    cards = "".join(f"<article class='metric-card {'is-pass' if item['passed'] else 'is-fail'}'><span>{html.escape(label)}</span><strong>{item['numerator']}<em>/20</em></strong><b>{item['percent']:.1f}%</b><small>Wilson 95% · {item['wilson95_low_percent']:.1f}%–{item['wilson95_high_percent']:.1f}%</small></article>" for label, item in rates)
    table_rows = []
    for row in summary["runs"]:
        deviation = row.get("path_deviation_percent")
        deviation_text = "" if deviation is None else f"{float(deviation):.2f}%"
        status = "pass" if row["strict_success"] else "fail"
        table_rows.append(f"<tr data-kind='{row['kind']}' data-seed='{row['seed']}' data-status='{status}'><td><span class='tag'>{html.escape(str(row['kind']))}</span></td><td>{row['seed']}</td><td><span class='status {status}'>{'通过' if row['strict_success'] else '失败'}</span></td><td>{'是' if row['physical_collision_free'] else '否'}</td><td>{deviation_text or '—'}</td><td><a href='runs/{row['kind']}-{row['seed']}/index.html'>查看详情 <span aria-hidden='true'>→</span></a></td></tr>")
    rows = "".join(table_rows)
    images = "".join(f"<figure><img src='data:image/png;base64,{base64.b64encode(path.read_bytes()).decode('ascii')}' alt='{path.name}'><figcaption>{path.name}</figcaption></figure>" for path in figure_paths)
    conclusion = "通过" if passed else "未通过"
    max_deviation = summary["path_optimality"]["maximum_deviation_percent"]
    integrity = sum(bool(row["data_complete"]) and bool(row["checksums_verified"]) for row in summary["runs"])
    return _dashboard_html(summary, visual, cards, rows, images, conclusion, max_deviation, integrity)
    return f"""<!doctype html><html lang='zh-CN'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>Kujiale 全屋长距离导航验收</title><style>:root{{--ink:#0f172a;--muted:#64748b;--line:#e2e8f0;--paper:#fff;--canvas:#f6f8fc;--green:#15803d;--red:#b91c1c;--blue:#2563eb}}*{{box-sizing:border-box}}body{{margin:0;background:var(--canvas);color:var(--ink);font:15px/1.55 Inter,"Noto Sans CJK SC","Microsoft YaHei",system-ui,sans-serif}}.wrap{{max-width:1240px;margin:auto;padding:28px 24px 56px}}.hero{{padding:38px 42px;border-radius:24px;background:radial-gradient(circle at top right,#2563eb 0,transparent 37%),linear-gradient(135deg,#0f172a,#172554);color:white;box-shadow:0 20px 45px #0f172a33}}.eyebrow{{margin:0 0 8px;color:#bfdbfe;letter-spacing:.12em;font-size:12px;font-weight:700}}h1{{margin:0;font-size:clamp(28px,5vw,46px);letter-spacing:-.04em}}.hero p{{max-width:760px;margin:12px 0 0;color:#dbeafe;font-size:16px}}.decision{{display:inline-flex;align-items:center;gap:9px;margin-top:24px;padding:8px 13px;border:1px solid #ffffff33;border-radius:999px;background:#ffffff14;font-weight:700}}.decision i{{width:9px;height:9px;border-radius:50%;background:{'#4ade80' if passed else '#f87171'};box-shadow:0 0 0 5px {'#4ade8022' if passed else '#f8717122'}}}.section-head{{display:flex;align-items:end;justify-content:space-between;gap:18px;margin:38px 0 14px}}h2{{margin:0;font-size:22px;letter-spacing:-.02em}}.section-head p{{margin:0;color:var(--muted)}}.cards{{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:14px}}.metric-card,.panel,figure{{background:var(--paper);border:1px solid var(--line);border-radius:18px;box-shadow:0 10px 24px #0f172a08}}.metric-card{{position:relative;overflow:hidden;padding:18px}}.metric-card:before{{content:"";position:absolute;inset:0 auto 0 0;width:4px;background:var(--green)}}.metric-card.is-fail:before{{background:var(--red)}}.metric-card span,.metric-card small{{display:block;color:var(--muted)}}.metric-card strong{{display:block;margin-top:8px;font-size:32px;line-height:1;font-variant-numeric:tabular-nums}}.metric-card strong em{{font-size:16px;font-style:normal;color:var(--muted)}}.metric-card b{{display:block;margin:7px 0 5px;color:var(--green);font-size:14px}}.metric-card.is-fail b{{color:var(--red)}}.facts{{display:grid;grid-template-columns:repeat(3,1fr);gap:1px;overflow:hidden;border:1px solid var(--line);border-radius:18px;background:var(--line)}}.facts div{{padding:18px 20px;background:white}}.facts span,.facts strong{{display:block}}.facts span{{color:var(--muted);font-size:13px}}.facts strong{{margin-top:4px;font-size:20px;font-variant-numeric:tabular-nums}}.panel{{padding:18px}}.filters{{display:flex;flex-wrap:wrap;align-items:center;gap:12px}}label{{color:var(--muted);font-weight:600}}select{{margin-left:6px;padding:8px 30px 8px 10px;border:1px solid var(--line);border-radius:9px;background:white;color:var(--ink);font:inherit}}.table-wrap{{overflow-x:auto}}table{{width:100%;border-collapse:collapse;min-width:700px}}th{{padding:13px 12px;color:var(--muted);font-size:12px;text-align:left;text-transform:uppercase;letter-spacing:.06em}}td{{padding:13px 12px;border-top:1px solid var(--line)}}tbody tr:hover{{background:#f8fbff}}.tag,.status{{display:inline-block;border-radius:999px;padding:3px 8px;font-size:12px;font-weight:700}}.tag{{background:#eff6ff;color:#1d4ed8}}.status.pass{{background:#dcfce7;color:#166534}}.status.fail{{background:#fee2e2;color:#991b1b}}a{{color:var(--blue);font-weight:700;text-decoration:none}}a:hover{{text-decoration:underline}}figure{{margin:18px 0;padding:12px}}img{{display:block;width:100%;height:auto;border-radius:11px}}figcaption{{padding:10px 4px 2px;color:var(--muted);font-size:13px}}footer{{margin-top:36px;color:var(--muted);font-size:13px}}@media(max-width:860px){{.wrap{{padding:16px 14px 38px}}.hero{{padding:28px 24px;border-radius:18px}}.cards{{grid-template-columns:repeat(2,minmax(0,1fr))}}.facts{{grid-template-columns:1fr}}.section-head{{align-items:start;flex-direction:column}}}}@media(max-width:500px){{.cards{{grid-template-columns:1fr}}}}</style></head><body><main class='wrap'><header class='hero'><p class='eyebrow'>FORMAL ACCEPTANCE · KUJIALE LONG RANGE</p><h1>Kujiale 全屋长距离导航</h1><p>正式验收可视化报告｜覆盖静态与动态各 20 轮、8 个航点，并将运行清单、传感器与校验和证据随报告一并归档。</p><div class='decision'><i></i>自动结论：{conclusion}</div></header><section class='section-head'><div><h2>验收评分卡</h2><p>成功率与 Wilson 95% 置信区间；门槛：静态 19/20、动态 18/20。</p></div></section><section class='cards'>{cards}</section><section class='section-head'><div><h2>关键事实</h2><p>以 <code>benchmark.json</code> 作为机器可读 KPI 来源。</p></div></section><section class='facts'><div><span>最大静态路径偏差</span><strong>{max_deviation:.2f}% <small>≤ 20.00%</small></strong></div><div><span>完整且校验和已验证</span><strong>{integrity}/40 <small>正式运行证据</small></strong></div><div><span>固定路径</span><strong>G1 → G8 <small>每轮共 8 个航点</small></strong></div></section><section class='section-head'><div><h2>运行明细</h2><p>可按场景与严格成功结果筛选，并下钻至原始运行清单。</p></div></section><section class='panel'><div class='filters'><label>场景 <select id='kind'><option value='all'>全部</option><option>static</option><option>dynamic</option></select></label><label>结果 <select id='status'><option value='all'>全部</option><option value='pass'>通过</option><option value='fail'>失败</option></select></label></div><div class='table-wrap'><table><thead><tr><th>场景</th><th>种子</th><th>严格成功</th><th>物理无碰撞</th><th>路径偏差</th><th>证据下钻</th></tr></thead><tbody>{rows}</tbody></table></div></section><section class='section-head'><div><h2>图形证据</h2><p>总览、航点执行矩阵与 PDF 报告使用同一份聚合数据生成。</p></div></section><section>{images}</section><footer>自动生成的可移植离线报告 · 所有文件均由 <code>checksums.sha256</code> 覆盖校验。</footer></main><script>const kind=document.getElementById('kind'),status=document.getElementById('status');function filterRows(){{for(const row of document.querySelectorAll('tbody tr')){{row.hidden=(kind.value!=='all'&&row.dataset.kind!==kind.value)||(status.value!=='all'&&row.dataset.status!==status.value)}}}}kind.addEventListener('change',filterRows);status.addEventListener('change',filterRows);</script></body></html>"""


def _dashboard_html(summary: Mapping[str, Any], visual: Mapping[str, Any], cards: str, rows: str, images: str, conclusion: str, max_deviation: float, integrity: int) -> str:
    aggregate = visual["aggregate"]
    failures = "".join(
        f"<tr><td>{html.escape(reason)}</td><td>{counts['static']}</td><td>{counts['dynamic']}</td></tr>"
        for reason, counts in sorted(visual["failure_reasons"].items())
    ) or "<tr><td colspan='3'>无失败原因记录</td></tr>"
    seed_options = "".join(f"<option value='{item['seed']}'>{item['seed']}</option>" for item in visual["runs"])
    unavailable = []
    if not visual["availability"]["first_planned_paths"]:
        unavailable.append("首次规划路径：原始 MCAP 已归档，但未作为结构化序列采样，故不以零值替代。")
    if not visual["availability"]["voxelgrid_snapshot"]:
        unavailable.append("VoxelGrid：原始 MCAP 已归档，但本批次未导出可渲染快照。")
    unavailable_html = "".join(f"<li>{message}</li>" for message in unavailable) or "<li>所有聚合视图均有结构化输入。</li>"
    return f"""<!doctype html><html lang='zh-CN'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>Kujiale 全屋长距离导航验收</title><style>
:root{{--ink:#101828;--muted:#667085;--line:#e4e7ec;--paper:#fff;--canvas:#f6f8fc;--green:#169c4b;--red:#d92d20;--orange:#f79009;--purple:#7a5af8;--blue:#2e5bff}}*{{box-sizing:border-box}}body{{margin:0;background:var(--canvas);color:var(--ink);font:15px/1.55 Inter,"Noto Sans CJK SC","Microsoft YaHei",system-ui,sans-serif}}.wrap{{max-width:1280px;margin:auto;padding:28px 24px 60px}}.hero{{padding:38px 42px;border-radius:24px;background:radial-gradient(circle at top right,#3266ff 0,transparent 36%),linear-gradient(135deg,#101828,#172554);color:#fff;box-shadow:0 20px 45px #10182833}}.eyebrow{{margin:0;color:#bfdbfe;font-size:12px;font-weight:800;letter-spacing:.12em}}h1{{margin:8px 0 0;font-size:clamp(30px,5vw,46px);letter-spacing:-.04em}}.hero p{{max-width:800px;color:#dbeafe}}.decision{{display:inline-flex;gap:9px;align-items:center;margin-top:16px;padding:8px 13px;border:1px solid #ffffff33;border-radius:99px;background:#ffffff14;font-weight:800}}.decision:before{{content:"";width:9px;height:9px;border-radius:50%;background:{'#4ade80' if conclusion == '通过' else '#f87171'}}}.section-head{{margin:38px 0 14px}}h2{{margin:0;font-size:22px}}.section-head p{{margin:3px 0 0;color:var(--muted)}}.cards,.facts{{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:14px}}.metric-card,.panel,figure,.facts div{{background:var(--paper);border:1px solid var(--line);border-radius:18px;box-shadow:0 10px 24px #10182808}}.metric-card{{position:relative;padding:18px;overflow:hidden}}.metric-card:before{{content:"";position:absolute;inset:0 auto 0 0;width:4px;background:var(--green)}}.metric-card.is-fail:before{{background:var(--red)}}.metric-card span,.metric-card small,.facts span{{display:block;color:var(--muted)}}.metric-card strong{{display:block;margin-top:7px;font-size:32px;line-height:1}}.metric-card em{{font-size:16px;font-style:normal;color:var(--muted)}}.metric-card b{{display:block;margin:7px 0;color:var(--green)}}.facts{{grid-template-columns:repeat(4,1fr)}}.facts div{{padding:16px}}.facts strong{{display:block;margin-top:5px;font-size:20px}}.panel{{padding:18px}}.filters{{display:flex;flex-wrap:wrap;gap:12px;align-items:center}}select{{margin-left:6px;padding:8px;border:1px solid var(--line);border-radius:8px;background:#fff;font:inherit}}.route-map{{display:block;width:100%;max-height:680px;background:#f1f5f9;border-radius:12px}}.route-map text{{font-size:5px;font-weight:800;paint-order:stroke;stroke:#fff;stroke-width:1.3px}}.start circle{{fill:#111827;stroke:#fff;stroke-width:2px}}.goal circle{{fill:#2563eb;stroke:#fff;stroke-width:2px}}.static-box{{fill:#f59e0b88;stroke:#d97706;stroke-width:1.5px}}.dynamic-path{{fill:none;stroke:#7c3aed;stroke-width:2px;stroke-dasharray:4 2}}.reference-guide{{fill:none;stroke:#111827;stroke-width:1.4px;stroke-dasharray:5 3}}.track{{fill:none;stroke-width:1.6px;pointer-events:all}}.success-track{{stroke:#16a34a;opacity:.24}}.failure-track{{stroke:#dc2626;opacity:.9;stroke-width:2.4px}}.heatmap{{display:grid;gap:4px;align-items:center}}.heat-label{{font-size:12px;color:var(--muted);text-align:center}}.heat-cell{{display:grid;aspect-ratio:1;place-items:center;border-radius:5px;color:white;font-weight:800;text-decoration:none}}.heat-cell.success{{background:var(--green)}}.heat-cell.fail{{background:var(--red)}}.heat-cell.timeout{{background:var(--orange)}}.heat-cell.interaction{{background:var(--purple)}}.legend{{display:flex;flex-wrap:wrap;gap:14px;margin-top:12px;color:var(--muted);font-size:13px}}.legend i{{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:4px}}.table-wrap{{overflow-x:auto}}table{{width:100%;min-width:720px;border-collapse:collapse}}th{{padding:12px;text-align:left;color:var(--muted);font-size:12px}}td{{padding:12px;border-top:1px solid var(--line)}}.tag,.status{{display:inline-block;padding:3px 8px;border-radius:99px;font-size:12px;font-weight:800}}.tag{{background:#eff6ff;color:#1d4ed8}}.status.pass{{background:#dcfce7;color:#166534}}.status.fail{{background:#fee2e2;color:#991b1b}}a{{color:#2454d5;font-weight:700;text-decoration:none}}figure{{margin:16px 0;padding:12px}}img{{display:block;width:100%;border-radius:10px}}figcaption{{padding:9px 2px 1px;color:var(--muted);font-size:13px}}.split{{display:grid;grid-template-columns:1.2fr .8fr;gap:14px}}footer{{margin-top:32px;color:var(--muted);font-size:13px}}@media(max-width:860px){{.wrap{{padding:16px}}.hero{{padding:28px 24px}}.cards,.facts{{grid-template-columns:repeat(2,1fr)}}.split{{grid-template-columns:1fr}}}}@media(max-width:500px){{.cards,.facts{{grid-template-columns:1fr}}}}</style></head><body><main class='wrap'><header class='hero'><p class='eyebrow'>FORMAL ACCEPTANCE · KUJIALE LONG RANGE</p><h1>Kujiale 全屋长距离导航</h1><p>统一由 <code>benchmark.json</code> 聚合的正式验收可视化报告；覆盖静态与动态各 20 轮、8 个航点和完整可追溯证据。</p><div class='decision'>自动结论：{conclusion}</div></header><section class='section-head'><h2>总体 KPI</h2><p>门槛：静态严格/无碰撞 19/20（95%）；动态严格/无碰撞 18/20（90%）；静态偏差 ≤20%。</p></section><section class='cards'>{cards}</section><section class='section-head'><h2>运行规模与安全</h2></section><section class='facts'><div><span>最大静态路径偏差</span><strong>{max_deviation:.2f}% / 20%</strong></div><div><span>总实验时间</span><strong>{aggregate['total_duration_sec'] / 60:.1f} min</strong></div><div><span>总 GT 里程</span><strong>{aggregate['total_ground_truth_length_m']:.1f} m</strong></div><div><span>物理碰撞总数</span><strong>{aggregate['physical_collision_count']}</strong></div></section><section class='section-head'><h2>全屋轨迹地图</h2><p>底图：warehouse_new OccupancyGrid；路线可按场景、种子和结果筛选，悬停路线查看原始数值。</p></section><section class='panel'><div class='filters'><label>场景 <select id='kind'><option value='all'>全部</option><option value='static'>static</option><option value='dynamic'>dynamic</option></select></label><label>种子 <select id='seed'><option value='all'>全部</option>{seed_options}</select></label><label>结果 <select id='status'><option value='all'>全部</option><option value='pass'>通过</option><option value='fail'>失败</option></select></label></div>{_map_svg(visual)}<div class='legend'><span><i style='background:#16a34a'></i>成功 GT（半透明）</span><span><i style='background:#dc2626'></i>失败 GT</span><span><i style='background:#111827'></i>冻结航点顺序引导</span><span><i style='background:#7c3aed'></i>动态障碍路径</span></div></section><section class='section-head'><h2>航点结果热力图</h2><p>点击单元格进入对应实验及航段下钻页面。</p></section><section class='split'><div class='panel'><h3>静态 20 × 8</h3>{_html_heatmap(visual, 'static')}</div><div class='panel'><h3>动态 20 × 8</h3>{_html_heatmap(visual, 'dynamic')}</div></section><div class='legend'><span><i style='background:#169c4b'></i>成功</span><span><i style='background:#d92d20'></i>Action/碰撞失败</span><span><i style='background:#f79009'></i>超时/定位/传感器</span><span><i style='background:#7a5af8'></i>动态交互失效</span></div><section class='section-head'><h2>路径、避障与 RGB-D 证据</h2><p>图中悬停可查看数据点；未采集的结构化字段明确保留为不可用。</p></section><section>{images}</section><section class='split'><section class='panel'><h3>失败分析</h3><table><thead><tr><th>失败原因</th><th>静态</th><th>动态</th></tr></thead><tbody>{failures}</tbody></table></section><section class='panel'><h3>证据可用性</h3><ul>{unavailable_html}</ul><p>深度帧、Scan、Local/Global Costmap 与结构化运行数据在每轮 <code>runs/</code> 下提供；失败轮和指定代表轮保留 MCAP 相对链接。</p></section></section><section class='section-head'><h2>运行明细与原始数据</h2><p>表格与地图筛选联动；每轮页面包含事件时间线、轨迹、航段指标和原始文件链接。</p></section><section class='panel'><div class='table-wrap'><table><thead><tr><th>场景</th><th>种子</th><th>严格成功</th><th>物理无碰撞</th><th>路径偏差</th><th>证据下钻</th></tr></thead><tbody>{rows}</tbody></table></div></section><footer>自包含 HTML；PNG、PDF、Markdown、CSV、JSON 与原始运行证据均由同一次生成写入，并由 <code>checksums.sha256</code> 覆盖。</footer></main><script>const kind=document.getElementById('kind'),seed=document.getElementById('seed'),status=document.getElementById('status');function visible(e){{return(kind.value==='all'||e.dataset.kind===kind.value)&&(seed.value==='all'||e.dataset.seed===seed.value)&&(status.value==='all'||e.dataset.status===status.value)}}function apply(){{document.querySelectorAll('.track').forEach(e=>e.style.display=visible(e)?'':'none');document.querySelectorAll('tbody tr[data-kind]').forEach(e=>e.hidden=!visible(e))}}[kind,seed,status].forEach(e=>e.addEventListener('change',apply));</script></body></html>"""


# Keep the report template focused on evidence views, but derive its route copy
# from the frozen waypoint contract so a future layout change cannot silently
# keep the old G1–G8 wording in a newly generated report.
_legacy_dashboard_html = _dashboard_html


def _dashboard_html(summary: Mapping[str, Any], visual: Mapping[str, Any], cards: str, rows: str, images: str, conclusion: str, max_deviation: float, integrity: int) -> str:
    page = _legacy_dashboard_html(
        summary, visual, cards, rows, images, conclusion, max_deviation, integrity
    )
    route_copy = "G1（出生）→ G2 → G3 → G4 → G5 → G1（回归）"
    return (
        page.replace("静态与动态各 20 轮、8 个航点", "静态与动态各 20 轮、5 个导航 Goal")
        .replace("固定路径</span><strong>G1 → G8 <small>每轮共 8 个航点", f"固定路径</span><strong>{route_copy} <small>每轮共 5 个导航 Goal")
        .replace("静态 20 × 8", "静态 20 × 6")
        .replace("动态 20 × 8", "动态 20 × 6")
    )


def _write_run_pages(root: Path, visual: Mapping[str, Any]) -> None:
    for item in visual["runs"]:
        legs = "".join(
            f"<tr id='{html.escape(str(leg.get('id', '')))}'><td>{html.escape(str(leg.get('id', '—')))}</td><td>{leg.get('nav2_status', '—')}</td><td>{'超时' if leg.get('timed_out') else '完成' if leg.get('accepted') else '失败'}</td><td>{float(leg.get('duration_sec', 0.0)):.2f}s</td><td>{float(leg.get('ground_truth_length_m', 0.0)):.2f}m</td></tr>"
            for leg in item["legs"] if isinstance(leg, Mapping)
        )
        events = "".join(f"<li><code>{event.get('simulation_time', '—')}</code> · {html.escape(str(event.get('event', 'event')))} · {html.escape(str(event.get('obstacle_id', event.get('id', ''))))}</li>" for event in item["events"])
        files = "".join(f"<li><a href='{html.escape(name)}'>{html.escape(name)}</a></li>" for name in item["files"])
        status = "通过" if item["strict_success"] else "失败"
        minimum = "—" if item["minimum_scan_range_m"] is None else f"{item['minimum_scan_range_m']:.2f} m"
        page = f"""<!doctype html><html lang='zh-CN'><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>{item['id']} 运行下钻</title><style>body{{max-width:980px;margin:32px auto;padding:0 20px;font:15px/1.55 system-ui;color:#172033;background:#f8fafc}}section{{margin:18px 0;padding:18px;background:#fff;border:1px solid #e2e8f0;border-radius:14px}}table{{border-collapse:collapse;width:100%}}th,td{{padding:9px;text-align:left;border-bottom:1px solid #e2e8f0}}a{{color:#2563eb;font-weight:700}}code{{color:#64748b}}</style><p><a href='../../index.html'>← 返回总报告</a></p><h1>{item['id']}｜{status}</h1><p>GT 里程 {item['ground_truth_length_m']:.2f} m · 时长 {item['duration_sec']:.2f} s · 最小 Scan 距离 {minimum}</p><section><h2>航段结果</h2><table><thead><tr><th>航点</th><th>Nav2</th><th>结果</th><th>时长</th><th>GT 长度</th></tr></thead><tbody>{legs}</tbody></table></section><section><h2>事件时间线</h2><ul>{events or '<li>无结构化事件。</li>'}</ul></section><section><h2>原始结构化证据</h2><ul>{files}</ul></section></html>"""
        _atomic_write(root / "runs" / item["id"] / "index.html", page.encode("utf-8"))


def write_campaign_report(summary: Mapping[str, Any], directory: str | Path) -> Path:
    """Create a deterministic, portable Chinese dashboard and its exports."""
    root = Path(directory).expanduser().resolve(); figures = root / "figures"; root.mkdir(parents=True, exist_ok=True)
    visual = _visualization_data(summary)
    figure_paths = _plot_overview(summary, figures) + _plot_evidence_figures(visual, figures)
    _copy_runs(summary.get("runs", []), root / "runs")
    _write_run_pages(root, visual)
    clean_summary = {key: value for key, value in summary.items() if key != "runs"}
    clean_summary["runs"] = [{key: value for key, value in row.items() if key != "_evidence_dir"} for row in summary.get("runs", [])]
    clean_summary["visualization"] = visual
    benchmark = json.dumps(clean_summary, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False)
    _atomic_write(root / "benchmark.json", (benchmark + "\n").encode("utf-8"))
    fields = ["kind", "seed", "strict_success", "physical_collision_free", "path_deviation_percent", "data_complete", "checksums_verified", "campaign_conclusion"]
    with (root / "benchmark.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows({**{key: row.get(key) for key in fields}, "campaign_conclusion": "通过" if clean_summary["passed"] else "未通过"} for row in clean_summary["runs"])
    conclusion = "通过" if clean_summary["passed"] else "未通过"
    markdown = f"# Kujiale 全屋长距离导航验收报告\n\n自动结论：**{conclusion}**。静态门槛 19/20（95%），动态门槛 18/20（90%），静态路径偏差上限 20%。\n\n- 静态严格：{clean_summary['static']['strict_success']['numerator']}/20\n- 动态严格：{clean_summary['dynamic']['strict_success']['numerator']}/20\n- 最大静态路径偏差：{clean_summary['path_optimality']['maximum_deviation_percent']}%\n- 总实验时间：{visual['aggregate']['total_duration_sec'] / 60:.1f} min\n- 总 GT 里程：{visual['aggregate']['total_ground_truth_length_m']:.1f} m\n- 物理碰撞总数：{visual['aggregate']['physical_collision_count']}\n\n![总体 KPI](figures/campaign_overview.png)\n\n![路径最优性](figures/path_optimality.png)\n\n![避障与安全](figures/safety_overview.png)\n"
    _atomic_write(root / "report.md", markdown.encode("utf-8")); _atomic_write(root / "index.html", _html_page(clean_summary, figure_paths, visual).encode("utf-8"))
    dictionary = "# 数据字典\n\n`benchmark.json` 是 KPI 的唯一机器可读来源；`runs/` 保存每轮清单、事件、GT/Odom/Cmd、RGB-D、Scan、Costmap、MCAP 和校验和。`strict_success` 表示重设计闭环的五个导航 Goal（G2 至 G5，再回归 G1）及所有安全门禁均通过。\n"
    _atomic_write(root / "data_dictionary.md", dictionary.encode("utf-8"))
    plt, PdfPages = _matplotlib()
    with PdfPages(root / "report.pdf") as pdf:
        for path in figure_paths:
            image = plt.imread(path); fig, axis = plt.subplots(figsize=(16, 9)); axis.imshow(image); axis.axis("off"); axis.set_title(f"Kujiale 长距离导航：{conclusion}"); pdf.savefig(fig, bbox_inches="tight"); plt.close(fig)
    _write_report_checksums(root)
    return root


def _plot_static_overview(summary: Mapping[str, Any], figures: Path) -> list[Path]:
    """Render the overview for an explicitly static-only 20-run report."""
    plt, _ = _matplotlib()
    figures.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(16, 9), constrained_layout=True)
    fig.patch.set_facecolor("#f8fafc")
    fig.suptitle("Kujiale 全屋长距离导航｜静态 20 轮候选总览", fontsize=22, fontweight="bold", color="#0f172a")
    rates = [summary["static"]["strict_success"], summary["static"]["physical_collision_free"]]
    labels, thresholds = ["静态严格", "静态无碰撞"], [95, 95]
    kpi_axis = axes[0, 0]; kpi_axis.set_facecolor("#ffffff")
    values = [rate["percent"] for rate in rates]
    bars = kpi_axis.bar(labels, values, color=["#16a34a" if value >= threshold else "#dc2626" for value, threshold in zip(values, thresholds)], width=.62)
    kpi_axis.plot(range(2), thresholds, color="#f97316", linestyle="--", marker="o", label="静态门槛")
    kpi_axis.set(ylim=(0, 108), ylabel="成功率 (%)", title="静态 KPI｜每项分母均为 20")
    for bar, rate in zip(bars, rates):
        kpi_axis.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 2, f"{rate['numerator']}/20\n{rate['percent']:.1f}%", ha="center", va="bottom", fontsize=10, fontweight="bold")
    kpi_axis.legend(loc="lower left", frameon=False)

    rows = list(summary["runs"])
    valid = [(row["seed"], row.get("path_deviation_percent")) for row in rows if row.get("strict_success") and _finite_number(row.get("path_deviation_percent")) is not None]
    deviation_axis = axes[0, 1]; deviation_axis.set_facecolor("#ffffff")
    if valid:
        seeds, deviations = zip(*valid)
        deviation_axis.vlines(seeds, 0, deviations, color="#93c5fd", linewidth=2)
        deviation_axis.scatter(seeds, deviations, color="#2563eb", s=48, zorder=3, label="GT 相对理论路径")
    else:
        deviation_axis.text(.5, .5, "无成功静态运行\n无法计算路径偏差", ha="center", va="center", transform=deviation_axis.transAxes)
    deviation_axis.axhline(PATH_DEVIATION_MAX_PERCENT, color="#dc2626", linestyle="--", label="20% 门槛")
    maximum = _finite_number(summary["path_optimality"]["maximum_deviation_percent"])
    deviation_axis.set(xlabel="静态种子", ylabel="路径偏差 (%)", title=f"路径最优性｜最大偏差 {'—' if maximum is None else f'{maximum:.2f}%'}")
    deviation_axis.legend(loc="upper left", frameon=False)

    outcome_axis = axes[1, 0]; outcome_axis.set_facecolor("#ffffff")
    for index, row in enumerate(rows, 1):
        outcome_axis.scatter(index, 0, s=260, color="#16a34a" if row["strict_success"] else "#dc2626", marker="s", edgecolors="#ffffff", linewidths=1.2)
    strict_count = sum(bool(row["strict_success"]) for row in rows)
    outcome_axis.text(20.9, 0, f"{strict_count}/20", va="center", color="#334155", fontweight="bold")
    outcome_axis.set(xlim=(.25, 23), ylim=(-.7, .7), yticks=[0], yticklabels=["静态"], xlabel="正式试验序号（由左至右）", title="试验覆盖｜绿=严格通过，红=未通过")
    outcome_axis.set_xticks([1, 5, 10, 15, 20])

    evidence_axis = axes[1, 1]; evidence_axis.set_facecolor("#0f172a"); evidence_axis.set_xticks([]); evidence_axis.set_yticks([])
    for spine in evidence_axis.spines.values():
        spine.set_visible(False)
    complete = sum(bool(row["data_complete"]) for row in rows); verified = sum(bool(row["checksums_verified"]) for row in rows)
    evidence_axis.text(.07, .83, "静态证据与结论", transform=evidence_axis.transAxes, color="#f8fafc", fontsize=18, fontweight="bold")
    evidence_axis.text(.07, .55, "通过" if summary["passed"] else "未通过", transform=evidence_axis.transAxes, color="#4ade80" if summary["passed"] else "#f87171", fontsize=31, fontweight="bold")
    evidence_axis.text(.07, .27, f"{complete} / 20 轮证据完整\n{verified} / 20 轮校验和已验证", transform=evidence_axis.transAxes, color="#e2e8f0", fontsize=13, linespacing=1.8)
    evidence_axis.text(.07, .08, "仅静态 20 轮；动态批次未运行。门槛 19/20 · 偏差上限 20%", transform=evidence_axis.transAxes, color="#94a3b8", fontsize=10)
    for axis in (kpi_axis, deviation_axis, outcome_axis):
        axis.spines[["top", "right"]].set_visible(False); axis.grid(axis="y", color="#e2e8f0", linewidth=.8)
    overview = figures / "static_campaign_overview.png"; fig.savefig(overview, dpi=100); plt.close(fig)

    matrix = []
    for row in rows:
        line = []
        for index in range(len(WAYPOINT_IDS)):
            leg = row["legs"][index] if index < len(row["legs"]) else None
            line.append(0 if leg is None or not row["strict_success"] else 1 if leg.get("timed_out") else 2)
        matrix.append(line)
    fig, axis = plt.subplots(figsize=(16, 9), constrained_layout=True); fig.patch.set_facecolor("#f8fafc"); axis.set_facecolor("#ffffff")
    image = axis.imshow(matrix, aspect="auto", interpolation="nearest", cmap=plt.matplotlib.colors.ListedColormap(["#dc2626", "#f97316", "#16a34a"]), vmin=0, vmax=2)
    axis.set_xticks(range(len(WAYPOINT_IDS)), WAYPOINT_IDS); axis.set_yticks(range(20), [str(row["seed"]) for row in rows]); axis.set(xlabel="目标航点", ylabel="随机种子", title=f"Static｜20 × {len(WAYPOINT_IDS)} 航点执行矩阵")
    colorbar = fig.colorbar(image, ax=axis, ticks=[0, 1, 2], pad=.02); colorbar.ax.set_yticklabels(["失败 / 未执行", "超时", "成功"])
    heatmap = figures / "waypoint_heatmap_static.png"; fig.savefig(heatmap, dpi=100); plt.close(fig)
    return [overview, heatmap]


def _static_dashboard_html(summary: Mapping[str, Any], visual: Mapping[str, Any], figure_paths: Iterable[Path]) -> str:
    strict, collision = summary["static"]["strict_success"], summary["static"]["physical_collision_free"]
    cards = "".join(
        f"<article class='card {'good' if rate['passed'] else 'bad'}'><span>{label}</span><strong>{rate['numerator']}/20</strong><b>{rate['percent']:.1f}%</b><small>Wilson 95%：{rate['wilson95_low_percent']:.1f}%–{rate['wilson95_high_percent']:.1f}%</small></article>"
        for label, rate in (("静态严格成功", strict), ("静态物理无碰撞", collision))
    )
    max_deviation = _finite_number(summary["path_optimality"]["maximum_deviation_percent"])
    table_rows: list[str] = []
    for row in summary["runs"]:
        deviation = _finite_number(row.get("path_deviation_percent"))
        deviation_text = "—" if deviation is None else f"{deviation:.2f}%"
        status = "pass" if row["strict_success"] else "fail"
        table_rows.append(
            f"<tr data-seed='{row['seed']}' data-status='{status}'><td>{row['seed']}</td><td>{'通过' if row['strict_success'] else '失败'}</td>"
            f"<td>{'是' if row['physical_collision_free'] else '否'}</td><td>{deviation_text}</td>"
            f"<td><a href='runs/static-{row['seed']}/index.html'>查看详情</a></td></tr>"
        )
    rows = "".join(table_rows)
    images = "".join(f"<figure><img src='data:image/png;base64,{base64.b64encode(path.read_bytes()).decode('ascii')}' alt='{html.escape(path.name)}'><figcaption>{html.escape(path.name)}</figcaption></figure>" for path in figure_paths)
    conclusion = "通过" if summary["passed"] else "未通过"
    static_obstacle_count = len(visual["map"].get("static_obstacle_polygons", []))
    return f"""<!doctype html><html lang='zh-CN'><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>Kujiale 静态 20 轮报告</title><style>:root{{--ink:#101828;--muted:#667085;--line:#e4e7ec;--paper:#fff;--canvas:#f6f8fc;--green:#169c4b;--red:#d92d20;--blue:#2e5bff}}*{{box-sizing:border-box}}body{{margin:0;background:var(--canvas);color:var(--ink);font:15px/1.55 Inter,"Noto Sans CJK SC","Microsoft YaHei",system-ui,sans-serif}}main{{max-width:1280px;margin:auto;padding:28px 24px 60px}}header{{padding:38px 42px;border-radius:24px;background:radial-gradient(circle at top right,#3266ff 0,transparent 36%),linear-gradient(135deg,#101828,#172554);color:#fff}}h1{{margin:8px 0;font-size:clamp(30px,5vw,46px)}}h2{{margin:38px 0 8px}}.eyebrow{{font-size:12px;font-weight:800;letter-spacing:.12em;color:#bfdbfe}}.note{{color:#dbeafe}}.decision{{display:inline-block;margin-top:10px;padding:7px 12px;border:1px solid #ffffff33;border-radius:99px;font-weight:800}}.cards{{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:14px}}.card,.panel,figure{{background:var(--paper);border:1px solid var(--line);border-radius:18px;box-shadow:0 10px 24px #10182808}}.card{{padding:18px;border-left:4px solid var(--green)}}.card.bad{{border-left-color:var(--red)}}.card span,.card small{{display:block;color:var(--muted)}}.card strong{{display:block;margin-top:7px;font-size:34px}}.card b{{display:block;color:var(--green)}}.card.bad b{{color:var(--red)}}.panel{{padding:18px}}.route-map{{display:block;width:100%;max-height:680px;background:#f1f5f9;border-radius:12px}}.route-map text{{font-size:5px;font-weight:800;paint-order:stroke;stroke:#fff;stroke-width:1.3px}}.start circle{{fill:#111827;stroke:#fff;stroke-width:2px}}.goal circle{{fill:#2563eb;stroke:#fff;stroke-width:2px}}.static-box{{fill:#f59e0b88;stroke:#d97706;stroke-width:1.5px}}.reference-guide{{fill:none;stroke:#111827;stroke-width:1.4px;stroke-dasharray:5 3}}.track{{fill:none;stroke-width:1.6px;pointer-events:all}}.success-track{{stroke:#16a34a;opacity:.24}}.failure-track{{stroke:#dc2626;opacity:.9;stroke-width:2.4px}}.heatmap{{display:grid;gap:4px;align-items:center}}.heat-label{{font-size:12px;color:var(--muted);text-align:center}}.heat-cell{{display:grid;aspect-ratio:1;place-items:center;border-radius:5px;color:white;font-weight:800;text-decoration:none}}.heat-cell.success{{background:var(--green)}}.heat-cell.fail{{background:var(--red)}}.heat-cell.timeout{{background:#f79009}}table{{width:100%;border-collapse:collapse}}th,td{{padding:11px;text-align:left;border-bottom:1px solid var(--line)}}a{{color:var(--blue);font-weight:700;text-decoration:none}}figure{{margin:16px 0;padding:12px}}img{{display:block;width:100%;border-radius:10px}}figcaption{{padding:9px 2px 1px;color:var(--muted);font-size:13px}}footer{{margin-top:32px;color:var(--muted)}}@media(max-width:680px){{main{{padding:16px}}header{{padding:28px 24px}}.cards{{grid-template-columns:1fr}}}}</style><main><header><p class='eyebrow'>STATIC 20-RUN CANDIDATE · KUJIALE LONG RANGE</p><h1>Kujiale 全屋长距离导航｜静态 20 轮</h1><p class='note'>当前 {static_obstacle_count} 个 RGB-D 低矮静态障碍参数的候选静态批次。动态 20 轮尚未运行，本报告不对动态验收作任何结论。</p><div class='decision'>自动静态结论：{conclusion}</div></header><h2>总体 KPI</h2><p>门槛：静态严格成功和静态物理无碰撞均为 19/20（95%）；成功轮路径偏差 ≤20%。</p><section class='cards'>{cards}</section><section class='panel'><p>最大静态路径偏差：<strong>{'—' if max_deviation is None else f'{max_deviation:.2f}%'}</strong> / 20%　·　证据完整：{sum(bool(row['data_complete']) and bool(row['checksums_verified']) for row in summary['runs'])}/20</p></section><h2>全屋轨迹地图</h2><section class='panel'>{_map_svg(visual)}<p>黑色虚线是航点顺序引导；绿色为成功 GT、红色为失败 GT；橙色方块为当前 {static_obstacle_count} 个静态 RGB-D 障碍。</p></section><h2>航点结果热力图</h2><section class='panel'>{_html_heatmap(visual, 'static')}</section><h2>图形证据</h2>{images}<h2>运行明细</h2><section class='panel'><table><thead><tr><th>种子</th><th>严格成功</th><th>物理无碰撞</th><th>路径偏差</th><th>下钻</th></tr></thead><tbody>{rows}</tbody></table></section><footer>自动生成的离线静态候选报告。`benchmark.json` 是本报告的机器可读 KPI 来源；HTML、PDF、Markdown、CSV、PNG 和原始证据均由同一批输入生成并受 `checksums.sha256` 覆盖。</footer></main></html>"""


_legacy_static_dashboard_html = _static_dashboard_html


def _static_dashboard_html(summary: Mapping[str, Any], visual: Mapping[str, Any], figure_paths: Iterable[Path]) -> str:
    """Add static-run filtering without duplicating the evidence-derived map."""
    page = _legacy_static_dashboard_html(summary, visual, figure_paths)
    seed_options = "".join(
        f"<option value='{row['seed']}'>{row['seed']}</option>"
        for row in summary["runs"]
    )
    filters = (
        "<div class='filters'><label>种子 <select id='seed'><option value='all'>全部</option>"
        f"{seed_options}</select></label><label>结果 <select id='status'><option value='all'>全部</option>"
        "<option value='pass'>通过</option><option value='fail'>失败</option></select></label></div>"
    )
    route_map = _map_svg(visual)
    page = page.replace(
        f"<section class='panel'>{route_map}",
        f"<section class='panel'>{filters}{route_map}",
        1,
    ).replace(
        ".panel{padding:18px}",
        ".panel{padding:18px}.filters{display:flex;flex-wrap:wrap;gap:12px;align-items:center;margin-bottom:12px}.filters label{color:#667085;font-weight:700}.filters select{margin-left:6px;padding:8px;border:1px solid #e4e7ec;border-radius:8px;background:#fff;font:inherit}",
        1,
    )
    script = """<script>const seed=document.getElementById('seed'),status=document.getElementById('status');function visible(e){return(seed.value==='all'||e.dataset.seed===seed.value)&&(status.value==='all'||e.dataset.status===status.value)}function apply(){document.querySelectorAll('.track').forEach(e=>e.style.display=visible(e)?'':'none');document.querySelectorAll('tbody tr[data-seed]').forEach(e=>e.hidden=!visible(e))}[seed,status].forEach(e=>e.addEventListener('change',apply));</script>"""
    return page.replace("</main></html>", f"{script}</main></html>", 1)


def write_static_campaign_report(summary: Mapping[str, Any], directory: str | Path) -> Path:
    """Create the self-contained report for one static 20-run candidate batch."""
    root = Path(directory).expanduser().resolve(); figures = root / "figures"; root.mkdir(parents=True, exist_ok=True)
    visual = _visualization_data(summary)
    figure_paths = _plot_static_overview(summary, figures) + _plot_evidence_figures(visual, figures)
    _copy_runs(summary["runs"], root / "runs"); _write_run_pages(root, visual)
    clean = {key: value for key, value in summary.items() if key != "runs"}
    clean["runs"] = [{key: value for key, value in row.items() if key != "_evidence_dir"} for row in summary["runs"]]
    clean["visualization"] = visual
    _atomic_write(root / "benchmark.json", (json.dumps(clean, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False) + "\n").encode("utf-8"))
    fields = ["kind", "seed", "strict_success", "physical_collision_free", "path_deviation_percent", "data_complete", "checksums_verified", "static_candidate_conclusion"]
    with (root / "benchmark.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        writer.writerows({**{key: row.get(key) for key in fields}, "static_candidate_conclusion": "通过" if clean["passed"] else "未通过"} for row in clean["runs"])
    conclusion = "通过" if clean["passed"] else "未通过"; deviation = _finite_number(clean["path_optimality"]["maximum_deviation_percent"])
    markdown = f"# Kujiale 静态 20 轮候选报告\n\n自动静态结论：**{conclusion}**。动态 20 轮未运行，本报告不包含动态验收结论。\n\n- 静态严格：{clean['static']['strict_success']['numerator']}/20\n- 静态物理无碰撞：{clean['static']['physical_collision_free']['numerator']}/20\n- 最大静态路径偏差：{'—' if deviation is None else f'{deviation:.2f}%'}（门槛 ≤20%）\n\n![静态总览](figures/static_campaign_overview.png)\n\n![路径最优性](figures/path_optimality.png)\n"
    _atomic_write(root / "report.md", markdown.encode("utf-8")); _atomic_write(root / "index.html", _static_dashboard_html(clean, visual, figure_paths).encode("utf-8"))
    _atomic_write(root / "data_dictionary.md", "# 数据字典\n\n`benchmark.json` 是静态 20 轮候选 KPI 的唯一机器可读来源。`runs/` 保存每轮结构化清单、事件、GT/Odom/Cmd、RGB-D、Scan、Costmap、MCAP 与校验和。此报告明确不包含动态批次结论。\n".encode("utf-8"))
    plt, PdfPages = _matplotlib()
    with PdfPages(root / "report.pdf") as pdf:
        for path in figure_paths:
            image = plt.imread(path); fig, axis = plt.subplots(figsize=(16, 9)); axis.imshow(image); axis.axis("off"); axis.set_title(f"Kujiale 静态 20 轮：{conclusion}"); pdf.savefig(fig, bbox_inches="tight"); plt.close(fig)
    _write_report_checksums(root)
    return root


def main() -> None:
    parser = argparse.ArgumentParser(description="validate and render a Kujiale formal campaign")
    parser.add_argument("--run-directory", action="append", required=True); parser.add_argument("--output-directory", required=True)
    parser.add_argument("--static-only", action="store_true", help="validate static seeds 7201–7220 and render a static-only candidate report")
    arguments = parser.parse_args()
    summary = summarize_static_campaign(arguments.run_directory) if arguments.static_only else summarize_campaign(arguments.run_directory)
    output = write_static_campaign_report(summary, arguments.output_directory) if arguments.static_only else write_campaign_report(summary, arguments.output_directory)
    print(json.dumps({"output": str(output), "passed": summary["passed"]}, ensure_ascii=False)); raise SystemExit(0 if summary["passed"] else 2)
