"""Evidence-first reporting for the Kujiale static/dynamic appearance 4x20 campaign."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import html
import json
import math
from pathlib import Path
import shutil
from statistics import mean
from typing import Any, Iterable, Mapping


PROJECT_ROOT = Path(__file__).resolve().parents[4]
STATIC_SEEDS = tuple(range(7201, 7221))
DYNAMIC_SEEDS = tuple(range(7301, 7321))
APPEARANCE_PROFILES = ("dim_warm", "dim_cool", "bright_warm", "bright_cool")
CONDITIONS = {
    "static_baseline": {"kind": "static", "seeds": STATIC_SEEDS, "profile": "baseline", "required": 19},
    "static_appearance": {"kind": "static", "seeds": STATIC_SEEDS, "profile": None, "required": 19},
    "dynamic_baseline": {"kind": "dynamic", "seeds": DYNAMIC_SEEDS, "profile": "baseline", "required": 18},
    "dynamic_appearance": {"kind": "dynamic", "seeds": DYNAMIC_SEEDS, "profile": None, "required": 18},
}


class Campaign4x20Error(ValueError):
    """Raised for malformed campaign input rather than an acceptance failure."""


@dataclass(frozen=True)
class Rate:
    numerator: int
    denominator: int
    percent: float
    wilson95_low_percent: float
    wilson95_high_percent: float
    required_numerator: int
    passed: bool


def _read_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise Campaign4x20Error(f"invalid JSON evidence {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise Campaign4x20Error(f"JSON evidence must be an object: {path}")
    return value


def _checksums_verified(root: Path) -> bool:
    checksum_file = root / "checksums.sha256"
    if not checksum_file.is_file():
        return False
    try:
        rows = [line.split("  ", 1) for line in checksum_file.read_text(encoding="utf-8").splitlines() if line]
    except OSError:
        return False
    if not rows or any(len(row) != 2 or len(row[0]) != 64 for row in rows):
        return False
    for digest, relative in rows:
        candidate = root / relative
        try:
            candidate.resolve().relative_to(root.resolve())
        except ValueError:
            return False
        if not candidate.is_file() or hashlib.sha256(candidate.read_bytes()).hexdigest() != digest:
            return False
    return True


def _wilson(successes: int, total: int) -> tuple[float, float]:
    if total <= 0:
        return 0.0, 0.0
    z = 1.959963984540054
    ratio = successes / total
    denominator = 1.0 + z * z / total
    centre = (ratio + z * z / (2.0 * total)) / denominator
    radius = z * math.sqrt(ratio * (1.0 - ratio) / total + z * z / (4.0 * total * total)) / denominator
    return 100.0 * max(0.0, centre - radius), 100.0 * min(1.0, centre + radius)


def _rate(successes: int, required: int) -> Rate:
    low, high = _wilson(successes, 20)
    return Rate(successes, 20, successes * 5.0, low, high, required, successes >= required)


def _finite(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _duration(row: Mapping[str, Any]) -> float | None:
    legs = row.get("legs")
    if not isinstance(legs, list):
        return None
    values = [_finite(item.get("duration_sec")) for item in legs if isinstance(item, Mapping)]
    return sum(value for value in values if value is not None) if values else None


def _run_rows(run_root: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for summary_path in sorted(run_root.rglob("run_summary.json")):
        relative_parts = summary_path.relative_to(run_root).parts
        # Pilots prove the RGB/Session-Layer evidence before a formal stage.
        # They share a seed with the formal matrix, so including them would
        # create duplicate identities and invalidate the 80-run report.
        if any(part.startswith("pilot-") or part.startswith(".incomplete-") for part in relative_parts):
            continue
        root = summary_path.parent
        summary = _read_object(summary_path)
        manifest_path = root / "run_manifest.json"
        manifest = _read_object(manifest_path) if manifest_path.is_file() else {}
        condition = summary.get("condition_id")
        profile = summary.get("appearance_profile_id")
        manifest_condition = manifest.get("condition_id")
        manifest_profile = manifest.get("appearance", {}).get("profile_id") if isinstance(manifest.get("appearance"), Mapping) else None
        nav2_profile = summary.get("nav2_profile")
        manifest_nav2_profile = manifest.get("nav2_profile")
        metrics = manifest.get("metrics", {}) if isinstance(manifest.get("metrics"), Mapping) else {}
        interaction = manifest.get("dynamic_interaction", {}) if isinstance(manifest.get("dynamic_interaction"), Mapping) else {}
        selection = manifest.get("dynamic_selection", {}) if isinstance(manifest.get("dynamic_selection"), Mapping) else {}
        rows.append({
            "condition_id": condition,
            "kind": summary.get("kind"),
            "seed": summary.get("seed"),
            "appearance_profile_id": profile,
            "nav2_profile": nav2_profile,
            "strict_success": summary.get("strict_success") is True,
            "physical_collision_free": summary.get("physical_collision_free") is True,
            "data_complete": summary.get("data_complete") is True,
            "checksums_verified": summary.get("checksums_verified") is True and _checksums_verified(root),
            "path_deviation_percent": _finite(summary.get("path_deviation_percent")),
            "dynamic_interaction_complete": summary.get("dynamic_interaction_complete") is True,
            "variant_id": selection.get("variant_id"),
            "case_id": selection.get("case_id"),
            "duration_sec": _duration(manifest),
            "ground_truth_path_length_m": _finite(metrics.get("ground_truth_path_length_m")),
            "maximum_route_recoveries": _finite(metrics.get("maximum_route_recoveries")),
            "failure_reason": str(manifest.get("failure_reason", "")),
            "manifest_condition_id": manifest_condition,
            "manifest_appearance_profile_id": manifest_profile,
            "manifest_nav2_profile": manifest_nav2_profile,
            "manifest_path": str(manifest_path),
            "evidence_dir": str(root),
            "dynamic_guard_aborted": interaction.get("guard_aborted") is True,
        })
    return rows


def summarize_4x20(run_root: str | Path) -> dict[str, Any]:
    """Validate all 80 expected evidence rows and calculate four independent gates."""
    root = Path(run_root).expanduser().resolve()
    if not root.is_dir():
        raise Campaign4x20Error(f"campaign run root does not exist: {root}")
    rows = _run_rows(root)
    by_condition: dict[str, list[dict[str, Any]]] = {name: [] for name in CONDITIONS}
    issues: list[str] = []
    for row in rows:
        condition = row["condition_id"]
        if condition not in CONDITIONS:
            issues.append(f"unknown_or_missing_condition:{condition!r}:{row['evidence_dir']}")
            continue
        by_condition[condition].append(row)
    condition_summaries: dict[str, Any] = {}
    all_rows: list[dict[str, Any]] = []
    for condition, specification in CONDITIONS.items():
        selected = by_condition[condition]
        expected_seeds = set(specification["seeds"])
        seen: set[int] = set()
        valid_rows: list[dict[str, Any]] = []
        for row in selected:
            seed = row["seed"]
            if not isinstance(seed, int) or seed not in expected_seeds or seed in seen:
                issues.append(f"invalid_or_duplicate_seed:{condition}:{seed}")
                continue
            seen.add(seed)
            if row["kind"] != specification["kind"]:
                issues.append(f"kind_mismatch:{condition}:{seed}:{row['kind']}")
            if row["manifest_condition_id"] != condition:
                issues.append(f"manifest_condition_mismatch:{condition}:{seed}")
            if row["manifest_appearance_profile_id"] != row["appearance_profile_id"]:
                issues.append(f"manifest_appearance_mismatch:{condition}:{seed}")
            expected_nav2_profile = "dynamic_avoidance" if specification["kind"] == "dynamic" else "stable"
            if row["nav2_profile"] != expected_nav2_profile or row["manifest_nav2_profile"] != expected_nav2_profile:
                issues.append(f"nav2_profile_mismatch:{condition}:{seed}")
            expected_profile = specification["profile"]
            if expected_profile is not None and row["appearance_profile_id"] != expected_profile:
                issues.append(f"baseline_profile_mismatch:{condition}:{seed}")
            if expected_profile is None and row["appearance_profile_id"] not in APPEARANCE_PROFILES:
                issues.append(f"appearance_profile_mismatch:{condition}:{seed}")
            valid_rows.append(row)
        missing = sorted(expected_seeds - seen)
        if missing:
            issues.append(f"missing_seeds:{condition}:{','.join(str(seed) for seed in missing)}")
        strict = sum(
            row["strict_success"]
            and (row["kind"] != "dynamic" or row["dynamic_interaction_complete"])
            and not row["dynamic_guard_aborted"]
            for row in valid_rows
        )
        collision = sum(row["physical_collision_free"] for row in valid_rows)
        successful_static_deviations = [
            row["path_deviation_percent"]
            for row in valid_rows
            if row["kind"] == "static" and row["strict_success"] and row["path_deviation_percent"] is not None
        ]
        deviations_ok = (
            specification["kind"] != "static"
            or (
                len(successful_static_deviations) == strict
                and all(value <= 20.0 for value in successful_static_deviations)
            )
        )
        evidence_ok = len(valid_rows) == 20 and all(
            row["data_complete"] and row["checksums_verified"] for row in valid_rows
        )
        strict_rate = _rate(strict, int(specification["required"]))
        collision_rate = _rate(collision, int(specification["required"]))
        condition_summaries[condition] = {
            "kind": specification["kind"],
            "expected_runs": 20,
            "observed_runs": len(valid_rows),
            "strict_success": asdict(strict_rate),
            "physical_collision_free": asdict(collision_rate),
            "evidence_complete": evidence_ok,
            "path_deviation": {
                "maximum_percent": max(successful_static_deviations) if successful_static_deviations else None,
                "mean_percent": mean(successful_static_deviations) if successful_static_deviations else None,
                "required_maximum_percent": 20.0 if specification["kind"] == "static" else None,
                "passed": deviations_ok,
            },
            "passed": strict_rate.passed and collision_rate.passed and evidence_ok and deviations_ok,
        }
        if not evidence_ok:
            issues.append(f"evidence_incomplete:{condition}")
        all_rows.extend(valid_rows)

    for base, varied in (("static_baseline", "static_appearance"), ("dynamic_baseline", "dynamic_appearance")):
        base_seeds = {row["seed"] for row in by_condition[base] if isinstance(row["seed"], int)}
        varied_seeds = {row["seed"] for row in by_condition[varied] if isinstance(row["seed"], int)}
        if base_seeds != varied_seeds:
            issues.append(f"unpaired_seeds:{base}:{varied}")
    profiles = [row["appearance_profile_id"] for row in by_condition["static_appearance"]]
    if {profile: profiles.count(profile) for profile in APPEARANCE_PROFILES} != {profile: 5 for profile in APPEARANCE_PROFILES}:
        issues.append("static_appearance_profile_distribution_invalid")
    dynamic_profiles: dict[str, set[str]] = {}
    for row in by_condition["dynamic_appearance"]:
        if isinstance(row["variant_id"], str) and isinstance(row["appearance_profile_id"], str):
            dynamic_profiles.setdefault(row["variant_id"], set()).add(row["appearance_profile_id"])
    if dynamic_profiles != {variant: set(APPEARANCE_PROFILES) for variant in ("v1", "v2", "v3", "v4", "v5")}:
        issues.append("dynamic_variant_profile_crossing_invalid")

    pairs = []
    for base, varied in (("static_baseline", "static_appearance"), ("dynamic_baseline", "dynamic_appearance")):
        first = {row["seed"]: row for row in by_condition[base] if isinstance(row["seed"], int)}
        second = {row["seed"]: row for row in by_condition[varied] if isinstance(row["seed"], int)}
        for seed in sorted(set(first) & set(second)):
            pairs.append({
                "kind": CONDITIONS[base]["kind"], "seed": seed,
                "baseline_condition": base, "appearance_condition": varied,
                "baseline_strict_success": first[seed]["strict_success"],
                "appearance_strict_success": second[seed]["strict_success"],
                "duration_delta_sec": (
                    None if first[seed]["duration_sec"] is None or second[seed]["duration_sec"] is None
                    else second[seed]["duration_sec"] - first[seed]["duration_sec"]
                ),
            })
    complete = (
        len(all_rows) == 80
        and not issues
        and all(item["evidence_complete"] for item in condition_summaries.values())
    )
    passed = complete and all(item["passed"] for item in condition_summaries.values())
    return {
        "schema_version": 1,
        "campaign": "kujiale_4x20_appearance",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "run_root": str(root),
        "complete": complete,
        "passed": passed,
        "issues": issues,
        "conditions": condition_summaries,
        "pairs": pairs,
        "runs": all_rows,
    }


def _matplotlib():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    from matplotlib import font_manager
    noto = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    if noto.is_file():
        font_manager.fontManager.addfont(str(noto))
        plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "Droid Sans Fallback", "sans-serif"]
    plt.rcParams["axes.unicode_minus"] = False
    return plt, PdfPages


def _plot_figures(summary: Mapping[str, Any], figures: Path) -> list[Path]:
    plt, _ = _matplotlib()
    figures.mkdir(parents=True, exist_ok=True)
    paths: list[Path] = []
    conditions = list(CONDITIONS)
    labels = ["静态\n基准", "静态\n外观", "动态\n基准", "动态\n外观"]
    strict = [summary["conditions"][item]["strict_success"] for item in conditions]
    collision = [summary["conditions"][item]["physical_collision_free"] for item in conditions]
    thresholds = [95, 95, 90, 90]
    fig, axis = plt.subplots(figsize=(13, 7), constrained_layout=True)
    x = list(range(4)); width = 0.36
    strict_values = [item["percent"] for item in strict]; collision_values = [item["percent"] for item in collision]
    first = axis.bar([value - width / 2 for value in x], strict_values, width, label="严格成功", color="#2563eb")
    second = axis.bar([value + width / 2 for value in x], collision_values, width, label="物理无碰撞", color="#059669")
    axis.plot(x, thresholds, "o--", color="#ea580c", label="分组门槛")
    axis.set_xticks(x, labels); axis.set_ylim(0, 108); axis.set_ylabel("比例 (%)")
    axis.set_title("Kujiale 4×20｜四组独立验收", loc="left", fontweight="bold")
    axis.legend(frameon=False, ncol=3)
    for bars, entries in ((first, strict), (second, collision)):
        for bar, entry in zip(bars, entries):
            axis.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 1.5, f"{entry['numerator']}/20", ha="center", fontsize=10)
    path = figures / "condition_overview.png"; fig.savefig(path, dpi=180, facecolor="#f8fafc"); plt.close(fig); paths.append(path)

    fig, axis = plt.subplots(figsize=(13, 6), constrained_layout=True)
    rows = summary["runs"]
    colors = {"static_baseline": "#2563eb", "static_appearance": "#7c3aed", "dynamic_baseline": "#059669", "dynamic_appearance": "#db2777"}
    for condition in conditions:
        values = [row["duration_sec"] for row in rows if row["condition_id"] == condition and row["duration_sec"] is not None]
        if values:
            axis.scatter([condition] * len(values), values, s=28, alpha=0.75, color=colors[condition], label=condition)
    axis.set_ylabel("累计航段时长 (s)"); axis.set_title("每轮导航时长分布", loc="left", fontweight="bold")
    axis.tick_params(axis="x", rotation=15); axis.legend(frameon=False, ncol=2)
    path = figures / "duration_distribution.png"; fig.savefig(path, dpi=180, facecolor="#f8fafc"); plt.close(fig); paths.append(path)

    fig, axis = plt.subplots(figsize=(13, 6), constrained_layout=True)
    static_rows = [row for row in rows if row["kind"] == "static" and row["strict_success"] and row["path_deviation_percent"] is not None]
    for condition in ("static_baseline", "static_appearance"):
        values = [row for row in static_rows if row["condition_id"] == condition]
        axis.scatter([row["seed"] for row in values], [row["path_deviation_percent"] for row in values], label=condition, color=colors[condition])
    axis.axhline(20.0, linestyle="--", color="#dc2626", label="20% 门槛")
    axis.set_xlabel("seed"); axis.set_ylabel("GT路径偏差 (%)"); axis.set_title("静态成功轮次的路径偏差", loc="left", fontweight="bold")
    axis.legend(frameon=False)
    path = figures / "static_path_deviation.png"; fig.savefig(path, dpi=180, facecolor="#f8fafc"); plt.close(fig); paths.append(path)
    return paths


def _copy_map_figure(figures: Path) -> Path | None:
    source = PROJECT_ROOT / "docs/figures/kujiale_4x20_test_matrix_map.png"
    if not source.is_file():
        return None
    target = figures / source.name
    shutil.copy2(source, target)
    return target


def _write_checksums(root: Path) -> None:
    rows = []
    for path in sorted(item for item in root.rglob("*") if item.is_file() and item.name != "checksums.sha256"):
        rows.append(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.relative_to(root)}")
    (root / "checksums.sha256").write_text("\n".join(rows) + "\n", encoding="utf-8")


def _clean(summary: Mapping[str, Any]) -> dict[str, Any]:
    result = dict(summary)
    result["runs"] = [{key: value for key, value in row.items() if key != "evidence_dir"} for row in summary["runs"]]
    return result


def _dashboard(summary: Mapping[str, Any], figures: Iterable[Path]) -> str:
    cards = "".join(
        f"<article><h3>{html.escape(condition)}</h3><strong>{entry['strict_success']['numerator']}/20</strong><span>严格成功</span><p>无碰撞 {entry['physical_collision_free']['numerator']}/20 · {'通过' if entry['passed'] else '未通过'}</p></article>"
        for condition, entry in summary["conditions"].items()
    )
    rows = "".join(
        f"<tr data-condition='{html.escape(str(row['condition_id']))}' data-profile='{html.escape(str(row['appearance_profile_id']))}' data-result={'pass' if row['strict_success'] else 'fail'}><td>{html.escape(str(row['condition_id']))}</td><td>{row['seed']}</td><td>{html.escape(str(row['appearance_profile_id']))}</td><td>{html.escape(str(row['variant_id'] or '—'))}</td><td>{'通过' if row['strict_success'] else '失败'}</td><td>{'是' if row['physical_collision_free'] else '否'}</td><td>{'—' if row['duration_sec'] is None else f"{row['duration_sec']:.1f}"}</td><td>{html.escape(row['failure_reason'] or '—')}</td></tr>"
        for row in summary["runs"]
    )
    images = "".join(f"<figure><img src='figures/{html.escape(path.name)}' alt='{html.escape(path.stem)}'><figcaption>{html.escape(path.stem)}</figcaption></figure>" for path in figures)
    issue_text = "无" if not summary["issues"] else "<br>".join(html.escape(item) for item in summary["issues"])
    status = "通过" if summary["passed"] else "未通过"
    return f"""<!doctype html><html lang='zh-CN'><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>Kujiale 4×20 外观鲁棒性报告</title><style>body{{margin:0;background:#f6f8fb;color:#172033;font:15px/1.5 system-ui,sans-serif}}main{{max-width:1440px;margin:auto;padding:30px}}header,.panel,article{{background:#fff;border:1px solid #e2e8f0;border-radius:16px;padding:20px;margin-bottom:18px}}h1,h2,h3{{margin:.1em 0 .55em}}.cards{{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:14px}}article strong{{font-size:32px;color:#2563eb;display:block}}article span{{color:#64748b}}.filters{{display:flex;gap:12px;flex-wrap:wrap;margin:12px 0}}select{{padding:7px;border:1px solid #cbd5e1;border-radius:8px;background:#fff}}table{{width:100%;border-collapse:collapse;font-size:13px}}th,td{{padding:9px;border-bottom:1px solid #e2e8f0;text-align:left;vertical-align:top}}figure{{margin:20px 0;background:#fff;border:1px solid #e2e8f0;border-radius:12px;padding:12px}}img{{display:block;max-width:100%;height:auto;margin:auto}}figcaption{{color:#64748b;margin-top:8px}}.bad{{color:#b91c1c;font-weight:700}}</style><main><header><h1>Kujiale 4×20 光照/颜色鲁棒性报告：{status}</h1><p>自动生成；四组各20轮，报告以每轮的manifest、summary、文件校验和为唯一输入。完整性：{'完整' if summary['complete'] else '不完整'}。</p></header><section class='cards'>{cards}</section><section class='panel'><h2>完整性与问题</h2><p class='bad'>{issue_text}</p></section><section class='panel'><h2>可视化</h2>{images}</section><section class='panel'><h2>运行筛选</h2><div class='filters'><label>条件 <select id='condition'><option value='all'>全部</option>{''.join(f"<option>{name}</option>" for name in CONDITIONS)}</select></label><label>外观 <select id='profile'><option value='all'>全部</option><option>baseline</option>{''.join(f'<option>{name}</option>' for name in APPEARANCE_PROFILES)}</select></label><label>结果 <select id='result'><option value='all'>全部</option><option value='pass'>通过</option><option value='fail'>失败</option></select></label></div><table><thead><tr><th>条件</th><th>seed</th><th>外观</th><th>变体</th><th>严格</th><th>无碰撞</th><th>时长(s)</th><th>失败原因</th></tr></thead><tbody>{rows}</tbody></table></section><footer><p>机器可读结果：benchmark.json / benchmark.csv；证据索引：evidence_index.json；不复制MCAP。</p></footer></main><script>for(const e of document.querySelectorAll('select'))e.onchange=()=>{{const c=condition.value,p=profile.value,r=result.value;document.querySelectorAll('tbody tr').forEach(x=>x.hidden=!((c==='all'||x.dataset.condition===c)&&(p==='all'||x.dataset.profile===p)&&(r==='all'||x.dataset.result===r)))}}</script></html>"""


def write_4x20_report(summary: Mapping[str, Any], output_directory: str | Path) -> Path:
    root = Path(output_directory).expanduser().resolve()
    if root.exists():
        raise Campaign4x20Error(f"refusing to overwrite report directory: {root}")
    figures = root / "figures"; root.mkdir(parents=True)
    figures_written = _plot_figures(summary, figures)
    map_figure = _copy_map_figure(figures)
    if map_figure is not None:
        figures_written.insert(0, map_figure)
    clean = _clean(summary)
    (root / "benchmark.json").write_text(json.dumps(clean, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")
    fields = ["condition_id", "kind", "seed", "appearance_profile_id", "nav2_profile", "variant_id", "strict_success", "physical_collision_free", "data_complete", "checksums_verified", "dynamic_interaction_complete", "path_deviation_percent", "ground_truth_path_length_m", "duration_sec", "maximum_route_recoveries", "failure_reason"]
    with (root / "benchmark.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows({key: row.get(key) for key in fields} for row in clean["runs"])
    evidence = [{"condition_id": row["condition_id"], "seed": row["seed"], "appearance_profile_id": row["appearance_profile_id"], "nav2_profile": row["nav2_profile"], "manifest_path": row["manifest_path"], "evidence_dir": row["evidence_dir"]} for row in summary["runs"]]
    (root / "evidence_index.json").write_text(json.dumps(evidence, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    markdown = "# Kujiale 4×20 光照/颜色鲁棒性报告\n\n"
    markdown += f"结论：**{'通过' if summary['passed'] else '未通过'}**；证据完整：{'是' if summary['complete'] else '否'}。\n\n"
    for condition, entry in summary["conditions"].items():
        markdown += f"- {condition}: 严格 {entry['strict_success']['numerator']}/20，无碰撞 {entry['physical_collision_free']['numerator']}/20，{'通过' if entry['passed'] else '未通过'}。\n"
    markdown += "\n![四组测试地图](figures/kujiale_4x20_test_matrix_map.png)\n\n![条件总览](figures/condition_overview.png)\n"
    (root / "report.md").write_text(markdown, encoding="utf-8")
    (root / "data_dictionary.md").write_text("# 数据字典\n\n`benchmark.json` 是四组验收、完整性和逐轮指标的机器可读来源。`evidence_index.json` 只索引原始证据目录，不复制MCAP。`condition_id` 为四组实验条件，`appearance_profile_id` 是本轮固定的Session Layer配置；`nav2_profile` 记录静态的 `stable` 或动态的 `dynamic_avoidance` 导航参数配置。\n", encoding="utf-8")
    (root / "index.html").write_text(_dashboard(clean, figures_written), encoding="utf-8")
    plt, PdfPages = _matplotlib()
    with PdfPages(root / "report.pdf") as pdf:
        for path in figures_written:
            image = plt.imread(path); fig, axis = plt.subplots(figsize=(16, 9)); axis.imshow(image); axis.axis("off"); axis.set_title("Kujiale 4×20 光照/颜色鲁棒性报告"); pdf.savefig(fig, bbox_inches="tight"); plt.close(fig)
    _write_checksums(root)
    return root


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="validate and report the Kujiale 4x20 appearance campaign")
    parser.add_argument("--run-root", required=True)
    parser.add_argument("--output-directory")
    parser.add_argument("--status", action="store_true")
    parsed = parser.parse_args(args)
    if bool(parsed.output_directory) == bool(parsed.status):
        parser.error("provide exactly one of --output-directory or --status")
    summary = summarize_4x20(parsed.run_root)
    if parsed.status:
        print(json.dumps({"complete": summary["complete"], "passed": summary["passed"], "issues": summary["issues"], "conditions": summary["conditions"]}, ensure_ascii=False))
        return
    output = write_4x20_report(summary, parsed.output_directory)
    print(json.dumps({"output": str(output), "complete": summary["complete"], "passed": summary["passed"]}, ensure_ascii=False))
    raise SystemExit(0 if summary["passed"] else 2)
