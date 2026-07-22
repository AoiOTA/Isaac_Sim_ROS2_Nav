"""Evidence-first aggregation and Chinese reporting for the frozen campaign."""

from __future__ import annotations

import argparse
import base64
import csv
from dataclasses import asdict, dataclass
import gzip
import hashlib
import html
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
WAYPOINT_IDS = tuple(f"G{index}" for index in range(1, 9))


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
    route = value.get("route")
    if not isinstance(route, list) or [item.get("id") if isinstance(item, Mapping) else None for item in route] != list(WAYPOINT_IDS):
        raise CampaignValidationError("campaign route must contain G1 through G8 in order")
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
            f"run {seed} legs must be an ordered G1-through-G8 prefix"
        )
    for field in ("strict_success", "physical_collision_free", "data_complete", "checksums_verified"):
        _bool(summary, field)
    if _bool(summary, "strict_success") and len(legs) != len(WAYPOINT_IDS):
        raise CampaignValidationError(
            f"strictly successful run {seed} must contain exactly 8 leg results"
        )
    return seed


def _campaign_rows(summaries: Iterable[Mapping[str, Any]], kind: str, seeds: tuple[int, ...]) -> list[Mapping[str, Any]]:
    selected: dict[int, Mapping[str, Any]] = {}
    for summary in summaries:
        if summary.get("kind") != kind:
            continue
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


def _atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_bytes(payload)
    temporary.replace(path)


def _matplotlib():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "Droid Sans Fallback", "sans-serif"]
    plt.rcParams["axes.unicode_minus"] = False
    return plt, PdfPages


def _plot_overview(summary: Mapping[str, Any], figures: Path) -> list[Path]:
    plt, _ = _matplotlib()
    figures.mkdir(parents=True, exist_ok=True)
    paths: list[Path] = []
    fig, axes = plt.subplots(1, 2, figsize=(16, 9), constrained_layout=True)
    labels = ["静态严格", "静态无碰撞", "动态严格", "动态无碰撞"]
    rates = [summary["static"]["strict_success"], summary["static"]["physical_collision_free"], summary["dynamic"]["strict_success"], summary["dynamic"]["physical_collision_free"]]
    values = [item["percent"] for item in rates]
    thresholds = [95, 95, 90, 90]
    axes[0].bar(labels, values, color=["#22c55e" if value >= threshold else "#ef4444" for value, threshold in zip(values, thresholds)])
    axes[0].plot(range(4), thresholds, "r--", label="验收门槛")
    axes[0].set_ylim(0, 105); axes[0].set_ylabel("百分比 (%)"); axes[0].set_title("正式验收 KPI（分母均为 20）")
    for index, item in enumerate(rates): axes[0].text(index, values[index] + 2, f"{item['numerator']}/20\n{values[index]:.1f}%", ha="center")
    axes[0].legend()
    rows = [row for row in summary["runs"] if row["kind"] == "static"]
    seeds = [row["seed"] for row in rows]
    deviations = [row.get("path_deviation_percent") if row.get("strict_success") else None for row in rows]
    axes[1].scatter([seed for seed, value in zip(seeds, deviations) if value is not None], [value for value in deviations if value is not None], color="#2563eb", label="GT 相对理论路径")
    axes[1].axhline(20, color="#dc2626", linestyle="--", label="20% 门槛")
    axes[1].set_xlabel("静态种子"); axes[1].set_ylabel("路径偏差 (%)"); axes[1].set_title("静态路径最优性"); axes[1].legend()
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
        image = axis.imshow(matrix, aspect="auto", interpolation="nearest", cmap=plt.matplotlib.colors.ListedColormap(["#ef4444", "#f97316", "#22c55e", "#a855f7"]), vmin=0, vmax=3)
        del image
        axis.set_xticks(range(8), WAYPOINT_IDS); axis.set_yticks(range(20), [str(row["seed"]) for row in rows]); axis.set_title(f"{kind}：20 × 8 航点结果热力图（绿=成功，红=失败，橙=超时，紫=动态交互失效）")
        path = figures / f"waypoint_heatmap_{kind}.png"; fig.savefig(path, dpi=100); plt.close(fig); paths.append(path)
    return paths


def _copy_runs(rows: Iterable[Mapping[str, Any]], destination: Path) -> None:
    for row in rows:
        source = Path(str(row["_evidence_dir"])); target = destination / f"{row['kind']}-{row['seed']}"
        if source.is_dir(): shutil.copytree(source, target, dirs_exist_ok=True)


def _html_page(summary: Mapping[str, Any], figure_paths: Iterable[Path]) -> str:
    passed = bool(summary["passed"])
    rates = [("静态严格成功", summary["static"]["strict_success"]), ("静态物理无碰撞", summary["static"]["physical_collision_free"]), ("动态严格成功", summary["dynamic"]["strict_success"]), ("动态物理无碰撞", summary["dynamic"]["physical_collision_free"])]
    cards = "".join(f"<article><b>{html.escape(label)}</b><strong>{item['numerator']}/20 ({item['percent']:.1f}%)</strong><small>Wilson 95%: {item['wilson95_low_percent']:.1f}%–{item['wilson95_high_percent']:.1f}%</small></article>" for label, item in rates)
    table_rows = []
    for row in summary["runs"]:
        deviation = row.get("path_deviation_percent")
        deviation_text = "" if deviation is None else f"{float(deviation):.2f}%"
        status = "pass" if row["strict_success"] else "fail"
        table_rows.append(
            f"<tr data-kind='{row['kind']}' data-status='{status}'><td>{row['kind']}</td>"
            f"<td>{row['seed']}</td><td>{'通过' if row['strict_success'] else '失败'}</td>"
            f"<td>{'是' if row['physical_collision_free'] else '否'}</td><td>{deviation_text}</td>"
            f"<td><a href='runs/{row['kind']}-{row['seed']}/run_manifest.json'>清单</a></td></tr>"
        )
    rows = "".join(table_rows)
    images = "".join(f"<figure><img src='data:image/png;base64,{base64.b64encode(path.read_bytes()).decode('ascii')}' alt='{path.name}'><figcaption>{path.name}</figcaption></figure>" for path in figure_paths)
    conclusion = "通过" if passed else "未通过"
    return f"""<!doctype html><html lang='zh-CN'><meta charset='utf-8'><title>Kujiale 全屋长距离导航验收</title><style>body{{font:15px system-ui;margin:24px;background:#f8fafc;color:#172033}}h1{{margin-bottom:4px}}.pass{{color:#15803d}}.fail{{color:#b91c1c}}.cards{{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:12px}}article,table,figure{{background:white;border-radius:10px;padding:14px;box-shadow:0 1px 3px #0001}}strong,small{{display:block;margin-top:7px}}strong{{font-size:22px}}table{{border-collapse:collapse;width:100%;margin-top:12px}}td,th{{padding:8px;border-bottom:1px solid #e2e8f0;text-align:left}}img{{max-width:100%;height:auto}}figure{{margin:18px 0}}select{{padding:5px}}</style><h1>Kujiale 全屋长距离导航验收</h1><p class='{'pass' if passed else 'fail'}'>自动结论：<b>{conclusion}</b>；门槛：静态 19/20（95%）、动态 18/20（90%）、静态路径偏差 ≤20%。</p><section class='cards'>{cards}</section><h2>运行筛选</h2><label>场景 <select id='kind'><option value='all'>全部</option><option>static</option><option>dynamic</option></select></label> <label>结果 <select id='status'><option value='all'>全部</option><option value='pass'>通过</option><option value='fail'>失败</option></select></label><table><thead><tr><th>场景</th><th>种子</th><th>严格成功</th><th>物理无碰撞</th><th>路径偏差</th><th>下钻</th></tr></thead><tbody>{rows}</tbody></table><section>{images}</section><script>for(const x of document.querySelectorAll('select'))x.onchange=()=>{{for(const r of document.querySelectorAll('tbody tr'))r.hidden=(kind.value!='all'&&r.dataset.kind!=kind.value)||(status.value!='all'&&r.dataset.status!=status.value)}}</script></html>"""


def write_campaign_report(summary: Mapping[str, Any], directory: str | Path) -> Path:
    """Create a deterministic, portable Chinese dashboard and its exports."""
    root = Path(directory).expanduser().resolve(); figures = root / "figures"; root.mkdir(parents=True, exist_ok=True)
    figure_paths = _plot_overview(summary, figures)
    _copy_runs(summary.get("runs", []), root / "runs")
    clean_summary = {key: value for key, value in summary.items() if key != "runs"}
    clean_summary["runs"] = [{key: value for key, value in row.items() if key != "_evidence_dir"} for row in summary.get("runs", [])]
    benchmark = json.dumps(clean_summary, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False)
    _atomic_write(root / "benchmark.json", (benchmark + "\n").encode("utf-8"))
    fields = ["kind", "seed", "strict_success", "physical_collision_free", "path_deviation_percent", "data_complete", "checksums_verified"]
    with (root / "benchmark.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows({key: row.get(key) for key in fields} for row in clean_summary["runs"])
    conclusion = "通过" if clean_summary["passed"] else "未通过"
    markdown = f"# Kujiale 全屋长距离导航验收报告\n\n自动结论：**{conclusion}**。静态门槛 19/20（95%），动态门槛 18/20（90%），静态路径偏差上限 20%。\n\n- 静态严格：{clean_summary['static']['strict_success']['numerator']}/20\n- 动态严格：{clean_summary['dynamic']['strict_success']['numerator']}/20\n- 最大静态路径偏差：{clean_summary['path_optimality']['maximum_deviation_percent']}%\n\n![总体 KPI](figures/campaign_overview.png)\n"
    _atomic_write(root / "report.md", markdown.encode("utf-8")); _atomic_write(root / "index.html", _html_page(summary, figure_paths).encode("utf-8"))
    dictionary = "# 数据字典\n\n`benchmark.json` 是 KPI 的唯一机器可读来源；`runs/` 保存每轮清单、事件、GT/Odom/Cmd、RGB-D、Scan、Costmap、MCAP 和校验和。`strict_success` 表示八个航点及所有安全门禁均通过。\n"
    _atomic_write(root / "data_dictionary.md", dictionary.encode("utf-8"))
    plt, PdfPages = _matplotlib()
    with PdfPages(root / "report.pdf") as pdf:
        for path in figure_paths:
            image = plt.imread(path); fig, axis = plt.subplots(figsize=(16, 9)); axis.imshow(image); axis.axis("off"); axis.set_title(f"Kujiale 长距离导航：{conclusion}"); pdf.savefig(fig, bbox_inches="tight"); plt.close(fig)
    checksum_lines = [f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.relative_to(root)}" for path in sorted(path for path in root.rglob("*") if path.is_file() and path.name != "checksums.sha256")]
    _atomic_write(root / "checksums.sha256", ("\n".join(checksum_lines) + "\n").encode("utf-8"))
    return root


def main() -> None:
    parser = argparse.ArgumentParser(description="validate and render a Kujiale formal campaign")
    parser.add_argument("--run-directory", action="append", required=True); parser.add_argument("--output-directory", required=True)
    arguments = parser.parse_args(); summary = summarize_campaign(arguments.run_directory); output = write_campaign_report(summary, arguments.output_directory)
    print(json.dumps({"output": str(output), "passed": summary["passed"]}, ensure_ascii=False)); raise SystemExit(0 if summary["passed"] else 2)
