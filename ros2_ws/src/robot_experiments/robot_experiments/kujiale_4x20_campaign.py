"""Evidence-first reporting for the Kujiale static/dynamic appearance 4x20 campaign."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import gzip
import hashlib
import html
import json
import math
from pathlib import Path
from statistics import mean
import textwrap
from typing import Any, Iterable, Mapping

from PIL import Image
import yaml


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
SCOPES = {
    "full": tuple(CONDITIONS),
    "static": ("static_baseline", "static_appearance"),
    "dynamic": ("dynamic_baseline", "dynamic_appearance"),
}
SCOPE_TITLES = {
    "full": "Kujiale 4×20 光照/颜色鲁棒性报告",
    "static": "Kujiale 静态 2×20 光照/颜色鲁棒性报告",
    "dynamic": "Kujiale 动态 2×20 光照/颜色鲁棒性报告",
}
CONDITION_LABELS = {
    "static_baseline": "静态\n基准",
    "static_appearance": "静态\n外观",
    "dynamic_baseline": "动态\n基准",
    "dynamic_appearance": "动态\n外观",
}
CONDITION_COLORS = {
    "static_baseline": "#2563eb",
    "static_appearance": "#7c3aed",
    "dynamic_baseline": "#059669",
    "dynamic_appearance": "#db2777",
}
DYNAMIC_ACTOR_COLORS = {
    "local_bypass_actor": "#f97316",
    "g2_g3_exit_actor": "#0891b2",
    "g5_g1_crossing_actor": "#eab308",
}
DYNAMIC_ACTOR_LABELS = {
    "local_bypass_actor": "G1→G2 局部绕行 actor",
    "g2_g3_exit_actor": "G2→G3 出口 actor",
    "g5_g1_crossing_actor": "G5→G1 横穿 actor",
}
DEMO_VIDEOS = {
    "static": (
        "静态避障演示（4×速）",
        "https://github.com/user-attachments/assets/39970d48-47df-428b-8d7d-276d2fd7db9d",
        "https://raw.githubusercontent.com/AoiOTA/Isaac_Sim_ROS2_Nav/main/docs/videos/%E9%9D%99%E6%80%81%E9%81%BF%E9%9A%9C%E6%BC%94%E7%A4%BA_4x_10MB.mp4",
    ),
    "dynamic": (
        "动态避障演示（4×速）",
        "https://github.com/user-attachments/assets/0fc1c31f-ace7-4b53-a463-b525a2521f4d",
        "https://raw.githubusercontent.com/AoiOTA/Isaac_Sim_ROS2_Nav/main/docs/videos/%E5%8A%A8%E6%80%81%E9%81%BF%E9%9A%9C%E6%BC%94%E7%A4%BA_4x_10MB.mp4",
    ),
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
        clearance_values = [
            float(value)
            for value in interaction.get(
                "minimum_clearance_m_by_actor", {}
            ).values()
            if isinstance(value, (int, float)) and math.isfinite(float(value))
        ] if isinstance(interaction.get("minimum_clearance_m_by_actor"), Mapping) else []
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
            "warning_reason": str(manifest.get("warning_reason", "")),
            "manifest_condition_id": manifest_condition,
            "manifest_appearance_profile_id": manifest_profile,
            "manifest_nav2_profile": manifest_nav2_profile,
            "manifest_path": str(manifest_path),
            "evidence_dir": str(root),
            "dynamic_guard_aborted": interaction.get("guard_aborted") is True,
            "dynamic_safety_yield": interaction.get("safety_yield") is True,
            "minimum_actor_clearance_m": (
                min(clearance_values) if clearance_values else None
            ),
        })
    return rows


def summarize_4x20(run_root: str | Path, *, scope: str = "full") -> dict[str, Any]:
    """Validate the full 4x20 evidence or an independently reportable 2x20 slice."""
    root = Path(run_root).expanduser().resolve()
    if not root.is_dir():
        raise Campaign4x20Error(f"campaign run root does not exist: {root}")
    if scope not in SCOPES:
        raise Campaign4x20Error(f"unknown report scope: {scope}")
    selected_conditions = SCOPES[scope]
    rows = _run_rows(root)
    by_condition: dict[str, list[dict[str, Any]]] = {name: [] for name in selected_conditions}
    issues: list[str] = []
    for row in rows:
        condition = row["condition_id"]
        if condition not in CONDITIONS:
            issues.append(f"unknown_or_missing_condition:{condition!r}:{row['evidence_dir']}")
        elif condition in by_condition:
            by_condition[condition].append(row)
    condition_summaries: dict[str, Any] = {}
    all_rows: list[dict[str, Any]] = []
    for condition in selected_conditions:
        specification = CONDITIONS[condition]
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
        deviations_ok = specification["kind"] != "static" or (
            len(successful_static_deviations) == strict
            and all(value <= 20.0 for value in successful_static_deviations)
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

    selected_pairs = [
        (base, varied)
        for base, varied in (("static_baseline", "static_appearance"), ("dynamic_baseline", "dynamic_appearance"))
        if base in selected_conditions and varied in selected_conditions
    ]
    for base, varied in selected_pairs:
        base_seeds = {row["seed"] for row in by_condition[base] if isinstance(row["seed"], int)}
        varied_seeds = {row["seed"] for row in by_condition[varied] if isinstance(row["seed"], int)}
        if base_seeds != varied_seeds:
            issues.append(f"unpaired_seeds:{base}:{varied}")
    if "static_appearance" in selected_conditions:
        profiles = [row["appearance_profile_id"] for row in by_condition["static_appearance"]]
        if {profile: profiles.count(profile) for profile in APPEARANCE_PROFILES} != {profile: 5 for profile in APPEARANCE_PROFILES}:
            issues.append("static_appearance_profile_distribution_invalid")
    if "dynamic_appearance" in selected_conditions:
        dynamic_profiles: dict[str, set[str]] = {}
        for row in by_condition["dynamic_appearance"]:
            if isinstance(row["variant_id"], str) and isinstance(row["appearance_profile_id"], str):
                dynamic_profiles.setdefault(row["variant_id"], set()).add(row["appearance_profile_id"])
        if dynamic_profiles != {variant: set(APPEARANCE_PROFILES) for variant in ("v1", "v2", "v3", "v4", "v5")}:
            issues.append("dynamic_variant_profile_crossing_invalid")

    pairs = []
    for base, varied in selected_pairs:
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
        len(all_rows) == 20 * len(selected_conditions)
        and not issues
        and all(item["evidence_complete"] for item in condition_summaries.values())
    )
    passed = complete and all(item["passed"] for item in condition_summaries.values())
    return {
        "schema_version": 1,
        "campaign": "kujiale_4x20_appearance",
        "scope": scope,
        "title": SCOPE_TITLES[scope],
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
    conditions = list(summary["conditions"])
    labels = [CONDITION_LABELS[item] for item in conditions]
    strict = [summary["conditions"][item]["strict_success"] for item in conditions]
    collision = [summary["conditions"][item]["physical_collision_free"] for item in conditions]
    thresholds = [summary["conditions"][item]["strict_success"]["required_numerator"] * 5 for item in conditions]
    fig, axis = plt.subplots(figsize=(13, 7), constrained_layout=True)
    x = list(range(len(conditions))); width = 0.36
    strict_values = [item["percent"] for item in strict]; collision_values = [item["percent"] for item in collision]
    first = axis.bar([value - width / 2 for value in x], strict_values, width, label="严格成功", color="#2563eb")
    second = axis.bar([value + width / 2 for value in x], collision_values, width, label="物理无碰撞", color="#059669")
    axis.plot(x, thresholds, "o--", color="#ea580c", label="分组门槛")
    axis.set_xticks(x, labels); axis.set_ylim(0, 108); axis.set_ylabel("比例 (%)")
    axis.set_title(f"{summary['title']}｜分组独立验收", loc="left", fontweight="bold")
    axis.legend(frameon=False, ncol=3)
    for bars, entries in ((first, strict), (second, collision)):
        for bar, entry in zip(bars, entries):
            axis.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 1.5, f"{entry['numerator']}/20", ha="center", fontsize=10)
    path = figures / "condition_overview.png"; fig.savefig(path, dpi=180, facecolor="#f8fafc"); plt.close(fig); paths.append(path)

    fig, axis = plt.subplots(figsize=(13, 6), constrained_layout=True)
    rows = summary["runs"]
    for condition in conditions:
        values = [row["duration_sec"] for row in rows if row["condition_id"] == condition and row["duration_sec"] is not None]
        if values:
            axis.scatter([condition] * len(values), values, s=28, alpha=0.75, color=CONDITION_COLORS[condition], label=condition)
    axis.set_ylabel("累计航段时长 (s)"); axis.set_title("每轮导航时长分布", loc="left", fontweight="bold")
    axis.tick_params(axis="x", rotation=15); axis.legend(frameon=False, ncol=2)
    path = figures / "duration_distribution.png"; fig.savefig(path, dpi=180, facecolor="#f8fafc"); plt.close(fig); paths.append(path)

    static_conditions = [item for item in conditions if CONDITIONS[item]["kind"] == "static"]
    if static_conditions:
        fig, axis = plt.subplots(figsize=(13, 6), constrained_layout=True)
        static_rows = [row for row in rows if row["kind"] == "static" and row["strict_success"] and row["path_deviation_percent"] is not None]
        for condition in static_conditions:
            values = [row for row in static_rows if row["condition_id"] == condition]
            axis.scatter([row["seed"] for row in values], [row["path_deviation_percent"] for row in values], label=condition, color=CONDITION_COLORS[condition])
        axis.axhline(20.0, linestyle="--", color="#dc2626", label="20% 门槛")
        axis.set_xlabel("seed"); axis.set_ylabel("GT路径偏差 (%)"); axis.set_title("静态成功轮次的路径偏差", loc="left", fontweight="bold")
        axis.legend(frameon=False)
        path = figures / "static_path_deviation.png"; fig.savefig(path, dpi=180, facecolor="#f8fafc"); plt.close(fig); paths.append(path)
    return paths


def _ground_truth_points(evidence_dir: str) -> list[tuple[float, float]]:
    path = Path(evidence_dir) / "ground_truth.csv.gz"
    if not path.is_file():
        return []
    try:
        with gzip.open(path, "rt", encoding="utf-8", newline="") as stream:
            points = [
                (x, y)
                for item in csv.DictReader(stream)
                if (x := _finite(item.get("x"))) is not None and (y := _finite(item.get("y"))) is not None
            ]
    except (OSError, UnicodeDecodeError, csv.Error):
        return []
    if len(points) <= 4000:
        return points
    stride = math.ceil(len(points) / 4000)
    return points[::stride] + [points[-1]]


def _occupancy_image():
    map_yaml = PROJECT_ROOT / "data/maps/occupancy/warehouse_new.yaml"
    try:
        payload = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            return None
        image_name = payload.get("image")
        resolution = _finite(payload.get("resolution"))
        origin = payload.get("origin")
        if not isinstance(image_name, str) or resolution is None or not isinstance(origin, list) or len(origin) < 2:
            return None
        origin_x, origin_y = _finite(origin[0]), _finite(origin[1])
        if origin_x is None or origin_y is None:
            return None
        plt, _ = _matplotlib()
        image = plt.imread(map_yaml.parent / image_name)
        height, width = image.shape[:2]
        return image, (origin_x, origin_x + width * resolution, origin_y, origin_y + height * resolution)
    except (OSError, ValueError, yaml.YAMLError):
        return None


def _trajectory_filename(row: Mapping[str, Any]) -> str:
    condition = str(row["condition_id"])
    seed = row["seed"]
    profile = str(row["appearance_profile_id"])
    return f"{condition}-seed-{seed}-{profile}.png"


def _static_obstacle_rectangles() -> list[tuple[float, float, float, float]]:
    """Read the same versioned physics layout used by the static 4×20 scenario."""
    source = PROJECT_ROOT / "isaac_sim/configs/experiments/kujiale_long_range_static.yaml"
    try:
        payload = yaml.safe_load(source.read_text(encoding="utf-8"))
        obstacles = payload.get("obstacles") if isinstance(payload, Mapping) else None
        if not isinstance(obstacles, list):
            return []
        rectangles = []
        for item in obstacles:
            if not isinstance(item, Mapping) or item.get("mode") != "stationary":
                return []
            center, size = item.get("start"), item.get("size")
            if not isinstance(center, list) or not isinstance(size, list) or len(center) < 2 or len(size) < 2:
                return []
            x, y, width, height = (_finite(center[0]), _finite(center[1]), _finite(size[0]), _finite(size[1]))
            if None in (x, y, width, height) or width <= 0.0 or height <= 0.0:
                return []
            rectangles.append((x - width / 2.0, y - height / 2.0, width, height))
        return rectangles
    except (OSError, yaml.YAMLError):
        return []


def _dynamic_obstacle_tracks(
    evidence_directory: str | Path,
) -> dict[str, list[tuple[float, float]]]:
    """Read actual tracks for actors activated during one dynamic run.

    The evidence stream also contains actors that remain in ``waiting`` for
    the entire run.  They are runtime-contract inventory rather than active
    obstacles, so report figures deliberately omit them.
    """
    source = Path(evidence_directory) / "dynamic_obstacles.csv.gz"
    samples: dict[str, list[tuple[str, float, float]]] = {}
    activated_ids: set[str] = set()
    try:
        with gzip.open(source, "rt", encoding="utf-8", newline="") as stream:
            for row in csv.DictReader(stream):
                actor_id = str(row.get("id", "")).strip()
                state = str(row.get("state", "")).strip()
                if not actor_id:
                    continue
                try:
                    position = json.loads(str(row.get("position", "")))
                except (json.JSONDecodeError, TypeError):
                    continue
                if not isinstance(position, list) or len(position) < 2:
                    continue
                x, y = _finite(position[0]), _finite(position[1])
                if x is None or y is None:
                    continue
                samples.setdefault(actor_id, []).append((state, x, y))
                if state and state != "waiting":
                    activated_ids.add(actor_id)
    except (OSError, EOFError):
        return {}

    tracks: dict[str, list[tuple[float, float]]] = {}
    for actor_id in sorted(activated_ids):
        points: list[tuple[float, float]] = []
        for state, x, y in samples.get(actor_id, []):
            if state == "waiting":
                continue
            point = (x, y)
            if not points or math.dist(points[-1], point) > 1.0e-6:
                points.append(point)
        if points:
            tracks[actor_id] = points
    return tracks


def _draw_dynamic_obstacle_tracks(axis: Any, tracks: Mapping[str, list[tuple[float, float]]]) -> None:
    """Overlay activated actor tracks, endpoints, and direction on one axis."""
    fallback_colors = ("#f97316", "#0891b2", "#eab308", "#9333ea")
    for index, (actor_id, points) in enumerate(sorted(tracks.items())):
        color = DYNAMIC_ACTOR_COLORS.get(actor_id, fallback_colors[index % len(fallback_colors)])
        label = DYNAMIC_ACTOR_LABELS.get(actor_id, actor_id)
        x_values = [point[0] for point in points]
        y_values = [point[1] for point in points]
        axis.plot(
            x_values,
            y_values,
            color=color,
            linestyle="--",
            linewidth=2.3,
            zorder=6,
            label=label,
        )
        axis.scatter(
            x_values[0],
            y_values[0],
            marker="s",
            color=color,
            edgecolors="white",
            linewidths=0.8,
            s=58,
            zorder=7,
        )
        axis.scatter(
            x_values[-1],
            y_values[-1],
            marker="X",
            color=color,
            edgecolors="white",
            linewidths=0.8,
            s=72,
            zorder=7,
        )
        if len(points) >= 2:
            arrow_index = max(1, len(points) // 2)
            axis.annotate(
                "",
                xy=points[arrow_index],
                xytext=points[arrow_index - 1],
                arrowprops={"arrowstyle": "-|>", "color": color, "lw": 2.0},
                zorder=8,
            )


def _plot_trajectory_figures(summary: Mapping[str, Any], figures: Path) -> None:
    """Render each available GT trace on the actual occupancy-grid map."""
    map_data = _occupancy_image()
    static_obstacles = _static_obstacle_rectangles()
    trajectory_root = figures / "trajectories"
    trajectory_root.mkdir(parents=True, exist_ok=True)
    for row in summary["runs"]:
        points = _ground_truth_points(str(row["evidence_dir"]))
        if not points:
            row["trajectory_figure"] = None
            continue
        filename = _trajectory_filename(row)
        relative = Path("figures") / "trajectories" / filename
        row["trajectory_figure"] = str(relative)
        plt, _ = _matplotlib()
        figure, axis = plt.subplots(figsize=(10, 8), constrained_layout=True)
        if map_data is not None:
            image, extent = map_data
            axis.imshow(image, cmap="gray", origin="upper", extent=extent, interpolation="nearest")
        if row["kind"] == "static":
            from matplotlib.patches import Rectangle

            for index, (left, bottom, width, height) in enumerate(static_obstacles):
                axis.add_patch(Rectangle(
                    (left, bottom), width, height,
                    facecolor="#fb923c", edgecolor="#ea580c", alpha=0.70, linewidth=1.4,
                    zorder=3, label="静态 RGB-D 障碍" if index == 0 else "_nolegend_",
                ))
        elif row["kind"] == "dynamic":
            _draw_dynamic_obstacle_tracks(
                axis,
                _dynamic_obstacle_tracks(str(row["evidence_dir"])),
            )
        x_values = [point[0] for point in points]
        y_values = [point[1] for point in points]
        color = CONDITION_COLORS.get(str(row["condition_id"]), "#2563eb")
        axis.plot(x_values, y_values, color=color, linewidth=1.8, zorder=4, label="实际 GT 路径")
        axis.scatter(x_values[0], y_values[0], color="#16a34a", edgecolors="white", linewidths=0.8, s=54, zorder=3, label="起点")
        axis.scatter(x_values[-1], y_values[-1], color="#dc2626", edgecolors="white", linewidths=0.8, s=54, zorder=3, label="终点")
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("map x (m)")
        axis.set_ylabel("map y (m)")
        axis.set_title(
            f"{row['condition_id']} · seed {row['seed']} · {row['appearance_profile_id']}",
            loc="left",
            fontweight="bold",
        )
        axis.grid(alpha=0.18)
        axis.legend(frameon=True, loc="best")
        figure.savefig(trajectory_root / filename, dpi=170, facecolor="#f8fafc")
        plt.close(figure)


def _copy_map_figures(summary: Mapping[str, Any], figures: Path) -> list[Path]:
    source = PROJECT_ROOT / "docs/figures/kujiale_4x20_test_matrix_map.png"
    if not source.is_file():
        return []
    scope = str(summary["scope"])
    # These bounds mirror the panel geometry in generate_kujiale_long_route_maps.py.
    # Every report uses individual panels: embedding the full 2×2 source map
    # makes labels and obstacle details unreadable at dashboard size.
    panels_by_scope = {
        "full": (
            ("static_baseline", (105, 220, 1270, 1040)),
            ("static_appearance", (1330, 220, 2495, 1040)),
            ("dynamic_baseline", (105, 1085, 1270, 1905)),
            ("dynamic_appearance", (1330, 1085, 2495, 1905)),
        ),
        "static": (
            ("static_baseline", (105, 220, 1270, 1040)),
            ("static_appearance", (1330, 220, 2495, 1040)),
        ),
        "dynamic": (
            ("dynamic_baseline", (105, 1085, 1270, 1905)),
            ("dynamic_appearance", (1330, 1085, 2495, 1905)),
        ),
    }
    with Image.open(source) as image:
        width, height = image.size
        scale_x, scale_y = width / 2600.0, height / 2580.0
        panels = []
        for condition, bounds in panels_by_scope[scope]:
            left, top, right, bottom = bounds
            target = figures / f"{condition}_test_map.png"
            image.crop((left * scale_x, top * scale_y, right * scale_x, bottom * scale_y)).save(target)
            panels.append(target)
    return panels


def _write_checksums(root: Path) -> None:
    rows = []
    for path in sorted(item for item in root.rglob("*") if item.is_file() and item.name != "checksums.sha256"):
        rows.append(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.relative_to(root)}")
    (root / "checksums.sha256").write_text("\n".join(rows) + "\n", encoding="utf-8")


def _clean(summary: Mapping[str, Any]) -> dict[str, Any]:
    result = dict(summary)
    result["runs"] = [{key: value for key, value in row.items() if key != "evidence_dir"} for row in summary["runs"]]
    return result


def _complete_navigation(row: Mapping[str, Any]) -> bool:
    """Return the published, evidence-complete definition of task success."""
    return bool(
        row.get("strict_success")
        and row.get("physical_collision_free")
        and row.get("data_complete")
        and row.get("checksums_verified")
        and (
            row.get("kind") != "dynamic"
            or (row.get("dynamic_interaction_complete") and not row.get("dynamic_guard_aborted"))
        )
    )


def _metric_summary(summary: Mapping[str, Any]) -> dict[str, Any]:
    """Calculate publication metrics without excluding failed planned rounds."""
    rows = list(summary["runs"])
    result: dict[str, Any] = {"overall": {}}
    for kind in ("static", "dynamic"):
        conditions = [
            condition for condition in summary["conditions"] if CONDITIONS[condition]["kind"] == kind
        ]
        planned = 20 * len(conditions)
        selected = [row for row in rows if row["kind"] == kind]
        result[kind] = {
            "planned": planned,
            "avoidance_success": sum(_complete_navigation(row) for row in selected),
            "navigation_success": sum(bool(row["strict_success"]) for row in selected),
        }
    total_planned = 20 * len(summary["conditions"])
    result["overall"] = {
        "planned": total_planned,
        "navigation_success": sum(_complete_navigation(row) for row in rows),
    }
    static_deviations = [
        row["path_deviation_percent"]
        for row in rows
        if row["kind"] == "static"
        and _complete_navigation(row)
        and row["path_deviation_percent"] is not None
    ]
    result["static"]["path_deviation"] = {
        "mean": mean(static_deviations) if static_deviations else None,
        "maximum": max(static_deviations) if static_deviations else None,
        "count": len(static_deviations),
    }
    return result


def _ratio_text(numerator: int, denominator: int) -> str:
    if denominator <= 0:
        return "不适用"
    return f"{numerator}/{denominator} = {100.0 * numerator / denominator:.1f}%"


def _methodology_html(summary: Mapping[str, Any]) -> str:
    """Describe the fixed experiment contract and its honest metric boundaries."""
    metrics = _metric_summary(summary)
    static = metrics["static"]
    dynamic = metrics["dynamic"]
    overall = metrics["overall"]
    deviation = static["path_deviation"]
    deviation_text = (
        f"{deviation['count']}个静态完整导航轮：均值 {deviation['mean']:.4f}%，"
        f"最大 {deviation['maximum']:.4f}%（门槛 ≤20%）。"
        if deviation["mean"] is not None
        else "本报告范围不含可计算的静态路径偏差。"
    )
    return (
        "<section class='panel methodology'><h2>实验设计与指标口径</h2>"
        "<h3>实验如何执行</h3>"
        "<p>固定 <code>warehouse_new</code> OccupancyGrid、Ideal 定位、G1 出生点和 "
        "<code>G1 → G2 → G3 → G4 → G5 → G1</code> 全屋闭环路线。每个条件20轮："
        "静态两组使用六个低矮 RGB-D 障碍；动态两组使用 <code>full_route_three_stage</code> 的 "
        "G1→G2、G2→G3、G5→G1 三阶段 actor。外观组在不改变几何、碰撞、地图或 actor 运动学的前提下，"
        "轮换四种固定光照/材质颜色 Session Layer 配置。</p>"
        "<p>每轮保留 manifest、summary、校验和、Ground Truth、传感器及动态交互证据。"
        "静态使用 <code>stable</code>，动态使用 <code>dynamic_avoidance</code>；两阶段之间重启 Isaac/Nav2，"
        "避免跨阶段调参或进程状态影响。</p>"
        "<h3>推荐交付指标与本次结果</h3>"
        "<table><thead><tr><th>指标</th><th>定义</th><th>本报告范围结果</th></tr></thead><tbody>"
        f"<tr><td>静态避障成功率</td><td><code>ASR_s = N_s^succ / N_s × 100%</code>。<code>N_s^succ</code> 为在静态障碍场景中完成规定路线且未发生碰撞的轮数。</td><td>{_ratio_text(static['avoidance_success'], static['planned'])}</td></tr>"
        f"<tr><td>动态避障成功率</td><td><code>ASR_d = N_d^succ / N_d × 100%</code>。<code>N_d^succ</code> 为完成动态障碍交互、规定路线且未发生碰撞的轮数。</td><td>{_ratio_text(dynamic['avoidance_success'], dynamic['planned'])}</td></tr>"
        f"<tr><td>规划相对理论最优路径偏差</td><td>单轮 <code>δ_i = (L_i − L_i*) / L_i* × 100%</code>；报告均值 <code>δ̄ = (1/n)Σδ_i</code> 和最大值 <code>max(δ_i)</code>。<code>L_i*</code> 为同约束下的理论最短可行路径长度。</td><td>{html.escape(deviation_text)}</td></tr>"
        f"<tr><td>导航成功率</td><td><code>NSR = N_goal / N × 100%</code>。<code>N_goal</code> 为在规定时间内完成全部必经航点的轮数。</td><td>静态 {_ratio_text(static['navigation_success'], static['planned'])}；动态 {_ratio_text(dynamic['navigation_success'], dynamic['planned'])}；总体 {_ratio_text(overall['navigation_success'], overall['planned'])}</td></tr>"
        "</tbody></table>"
        "<p class='muted'>这些是通用的任务级定义。当前实验中，目标完成由导航动作结果判定，碰撞由场景接触检测判定，"
        "动态交互由 actor 状态机判定；证据和校验和仅用于保证统计数据可追溯，不属于公式的一部分。动态 actor 的时间相关运动改变可行空间，"
        "因此不将动态实际路径与静态固定参考强行比较。</p></section>"
    )


def _methodology_markdown(summary: Mapping[str, Any]) -> str:
    metrics = _metric_summary(summary)
    static = metrics["static"]
    dynamic = metrics["dynamic"]
    overall = metrics["overall"]
    deviation = static["path_deviation"]
    deviation_text = (
        f"均值 {deviation['mean']:.4f}%；最大 {deviation['maximum']:.4f}%（{deviation['count']}轮，门槛≤20%）"
        if deviation["mean"] is not None else "不适用"
    )
    return (
        "## 实验如何执行\n\n"
        "固定 `warehouse_new`、Ideal 定位、G1出生点及 `G1 → G2 → G3 → G4 → G5 → G1` 全屋闭环路线。"
        "四个条件各20轮：静态两组使用六个低矮 RGB-D 障碍；动态两组使用三阶段 actor；外观组轮换四种固定"
        "光照/材质颜色 Session Layer 配置，不改变几何、碰撞、地图或 actor 运动学。静态使用 `stable`，动态使用 "
        "`dynamic_avoidance`，两个阶段之间重启 Isaac/Nav2。\n\n"
        "## 指标定义与本次结果\n\n"
        "| 指标 | 定义 | 结果 |\n| --- | --- | --- |\n"
        f"| 静态避障成功率 | `ASR_s = N_s^succ / N_s × 100%`；`N_s^succ` 为完成规定路线且未碰撞的静态轮数 | {_ratio_text(static['avoidance_success'], static['planned'])} |\n"
        f"| 动态避障成功率 | `ASR_d = N_d^succ / N_d × 100%`；`N_d^succ` 为完成动态交互、规定路线且未碰撞的动态轮数 | {_ratio_text(dynamic['avoidance_success'], dynamic['planned'])} |\n"
        f"| 规划相对理论最优路径偏差 | 单轮 `δ_i = (L_i − L_i*) / L_i* × 100%`；汇总 `δ̄ = (1/n)Σδ_i` 与 `max(δ_i)` | {deviation_text} |\n"
        f"| 导航成功率 | `NSR = N_goal / N × 100%`；`N_goal` 为在规定时间内完成全部必经航点的轮数 | 静态 {_ratio_text(static['navigation_success'], static['planned'])}；动态 {_ratio_text(dynamic['navigation_success'], dynamic['planned'])}；总体 {_ratio_text(overall['navigation_success'], overall['planned'])} |\n\n"
        "当前实验将目标完成、碰撞和动态交互分别从导航动作、接触检测和 actor 状态机测量；证据完整性只用于保证数据可追溯，不属于上述通用公式。动态场景不使用静态固定路径参考比较偏差，因为 actor 时序会改变可行空间。\n\n"
    )


def _write_methodology_pdf_page(plt: Any, pdf: Any, summary: Mapping[str, Any]) -> None:
    """Put the method and published metric results into the PDF, not only HTML."""
    metrics = _metric_summary(summary)
    static = metrics["static"]
    dynamic = metrics["dynamic"]
    overall = metrics["overall"]
    deviation = static["path_deviation"]
    deviation_text = (
        f"静态路径偏差：均值 {deviation['mean']:.4f}%，最大 {deviation['maximum']:.4f}%（门槛≤20%）。"
        if deviation["mean"] is not None else "静态路径偏差：本报告范围不适用。"
    )
    paragraphs = [
        "实验如何执行：固定 warehouse_new、Ideal 定位、G1出生点和 G1→G2→G3→G4→G5→G1 全屋闭环路线。四组各20轮；静态组使用六个低矮 RGB-D 障碍，动态组使用三阶段 actor；外观组轮换四种固定光照/材质颜色 Session Layer，不改变几何、碰撞、地图或 actor 运动学。静态和动态分别使用 stable、dynamic_avoidance，并在两阶段间重启 Isaac/Nav2。",
        f"静态避障成功率 ASR_s=N_s^succ/N_s×100%：{_ratio_text(static['avoidance_success'], static['planned'])}。动态避障成功率 ASR_d=N_d^succ/N_d×100%：{_ratio_text(dynamic['avoidance_success'], dynamic['planned'])}。",
        deviation_text,
        f"导航成功率：静态 {_ratio_text(static['navigation_success'], static['planned'])}；动态 {_ratio_text(dynamic['navigation_success'], dynamic['planned'])}；总体 {_ratio_text(overall['navigation_success'], overall['planned'])}。",
        "口径：避障成功表示完成相应场景规定路线且未碰撞；导航成功率 NSR=N_goal/N×100%，表示规定时间内完成全部必经航点。当前实验以导航动作、接触检测和 actor 状态机测量这些事件；证据校验只保证数据可追溯。动态 actor 时序会改变可行空间，因此不将动态实际路径与静态固定理论参考比较。",
    ]
    figure, axis = plt.subplots(figsize=(16, 9))
    axis.axis("off")
    axis.set_title(f"{summary['title']}｜实验设计与指标口径", loc="left", fontsize=18, fontweight="bold")
    y = 0.88
    for paragraph in paragraphs:
        lines = textwrap.wrap(paragraph, width=54, break_long_words=True)
        axis.text(0.05, y, "\n".join(lines), transform=axis.transAxes, va="top", fontsize=13)
        y -= 0.055 * (len(lines) + 1)
    pdf.savefig(figure, bbox_inches="tight")
    plt.close(figure)


def _dashboard(
    summary: Mapping[str, Any], statistics_figures: Iterable[Path], map_figures: Iterable[Path]
) -> str:
    title = str(summary["title"])
    scope = str(summary["scope"])
    run_count = 20 * len(summary["conditions"])
    scope_text = (
        "四组各20轮；仅当同一批次的静态和动态均完成时，才可作为完整4×20结论。"
        if scope == "full"
        else f"本报告仅覆盖{'静态' if scope == 'static' else '动态'} 2×20；不能替代或自动合并为完整4×20结论。"
    )
    cards = "".join(
        f"<article><h3>{html.escape(condition)}</h3><strong>{entry['strict_success']['numerator']}/20</strong><span>严格成功</span><p>无碰撞 {entry['physical_collision_free']['numerator']}/20 · {'通过' if entry['passed'] else '未通过'}</p></article>"
        for condition, entry in summary["conditions"].items()
    )
    methodology = _methodology_html(summary)
    trajectory_records = [
        {
            "condition": row["condition_id"],
            "seed": row["seed"],
            "profile": row["appearance_profile_id"],
            "result": "pass" if row["strict_success"] else "fail",
            "path": row.get("trajectory_figure"),
            "label": f"{row['condition_id']} · seed {row['seed']} · {row['appearance_profile_id']}",
        }
        for row in summary["runs"]
    ]
    rows = "".join(
        f"<tr data-condition='{html.escape(str(row['condition_id']))}' data-seed='{row['seed']}' data-profile='{html.escape(str(row['appearance_profile_id']))}' data-result={'pass' if row['strict_success'] else 'fail'}><td>{html.escape(str(row['condition_id']))}</td><td>{row['seed']}</td><td>{html.escape(str(row['appearance_profile_id']))}</td><td>{html.escape(str(row['variant_id'] or '—'))}</td><td>{'通过' if row['strict_success'] else '失败'}</td><td>{'是' if row['physical_collision_free'] else '否'}</td><td>{'—' if row['minimum_actor_clearance_m'] is None else f"{row['minimum_actor_clearance_m']:.3f}"}</td><td>{'是' if row['dynamic_safety_yield'] else '否'}</td><td>{'—' if row['duration_sec'] is None else f"{row['duration_sec']:.1f}"}</td><td>{html.escape(row['failure_reason'] or '—')}</td><td>{html.escape(row['warning_reason'] or '—')}</td><td>{f"<a href='{html.escape(str(row['trajectory_figure']))}' target='_blank'>打开</a>" if row.get('trajectory_figure') else '缺失'}</td></tr>"
        for row in summary["runs"]
    )
    statistics_images = "".join(
        f"<figure><a href='figures/{html.escape(path.name)}' target='_blank' rel='noopener'><img src='figures/{html.escape(path.name)}' alt='{html.escape(path.stem)}'></a><figcaption>{html.escape(path.stem)}（点击放大）</figcaption></figure>"
        for path in statistics_figures
    )
    maps_by_kind = {"static": [], "dynamic": []}
    for path in map_figures:
        kind = "dynamic" if path.name.startswith("dynamic_") else "static"
        maps_by_kind[kind].append(path)
    map_sections = "".join(
        f"<section class='map-pair'><h3>{'静态两组对比' if kind == 'static' else '动态两组对比'}</h3><div class='map-grid'>"
        + "".join(
            f"<figure><a href='figures/{html.escape(path.name)}' target='_blank' rel='noopener'><img src='figures/{html.escape(path.name)}' alt='{html.escape(path.stem)}'></a><figcaption>{html.escape(path.stem)}（点击打开原尺寸）</figcaption></figure>"
            for path in maps_by_kind[kind]
        )
        + "</div></section>"
        for kind in ("static", "dynamic") if maps_by_kind[kind]
    )
    demo_kinds = ("static", "dynamic") if scope == "full" else (scope,)
    demo_videos = "".join(
        f"<figure class='video-card'><figcaption>{html.escape(DEMO_VIDEOS[kind][0])}</figcaption><video controls preload='metadata' playsinline><source src='{html.escape(DEMO_VIDEOS[kind][1])}' type='video/mp4'><source src='{html.escape(DEMO_VIDEOS[kind][2])}' type='video/mp4'>你的浏览器不支持视频播放；<a href='{html.escape(DEMO_VIDEOS[kind][2])}' target='_blank' rel='noopener'>打开演示视频</a>。</video><p class='muted'>优先使用 GitHub attachment；若其不可用，播放器自动回退到仓库内同一视频的原始文件。</p></figure>"
        for kind in demo_kinds
    )
    issue_text = "无" if not summary["issues"] else "<br>".join(html.escape(item) for item in summary["issues"])
    status = "通过" if summary["passed"] else "未通过"
    trajectory_note = (
        "动态轮同时读取 <code>dynamic_obstacles.csv.gz</code>：彩色虚线为本轮实际触发 actor "
        "的运动轨迹，方形为起点，X为终点，箭头为运动方向。"
        if scope in {"full", "dynamic"}
        else "静态轮同时叠加六个正式 RGB-D 障碍物。"
    )
    records_json = json.dumps(trajectory_records, ensure_ascii=False).replace("</", "<\\/")
    seed_options = "".join(f"<option>{seed}</option>" for seed in sorted({row["seed"] for row in summary["runs"] if isinstance(row["seed"], int)}))
    condition_options = "".join(f"<option>{name}</option>" for name in summary["conditions"])
    profile_options = "".join(f"<option>{name}</option>" for name in ("baseline", *APPEARANCE_PROFILES))
    return f"""<!doctype html><html lang='zh-CN'><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>{html.escape(title)}</title><style>body{{margin:0;background:#f6f8fb;color:#172033;font:15px/1.5 system-ui,sans-serif}}main{{max-width:1440px;margin:auto;padding:30px}}header,.panel,article{{background:#fff;border:1px solid #e2e8f0;border-radius:16px;padding:20px;margin-bottom:18px}}h1,h2,h3{{margin:.1em 0 .55em}}.cards{{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:14px}}article strong{{font-size:32px;color:#2563eb;display:block}}article span{{color:#64748b}}.filters{{display:flex;gap:12px;flex-wrap:wrap;margin:12px 0}}select{{padding:7px;border:1px solid #cbd5e1;border-radius:8px;background:#fff}}table{{width:100%;border-collapse:collapse;font-size:13px}}th,td{{padding:9px;border-bottom:1px solid #e2e8f0;text-align:left;vertical-align:top}}figure{{margin:20px 0;background:#fff;border:1px solid #e2e8f0;border-radius:12px;padding:12px}}img,video{{display:block;max-width:100%;height:auto;margin:auto}}figcaption,.muted{{color:#64748b;margin-top:8px}}.bad{{color:#b91c1c;font-weight:700}}.map-grid{{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:18px}}.map-grid figure,.video-card{{margin:0}}.map-grid img{{width:100%;cursor:zoom-in}}.video-card video{{width:min(100%,1000px);background:#020617}}.statistics figure a{{display:block}}.methodology table{{margin-top:12px}}#trajectory-image{{max-height:760px;border:1px solid #cbd5e1;border-radius:12px;background:#fff;padding:4px;cursor:zoom-in}}#trajectory-image[hidden]{{display:none}}@media(max-width:800px){{main{{padding:14px}}.map-grid{{grid-template-columns:1fr}}}}</style><main><header><h1>{html.escape(title)}：{status}</h1><p>自动生成；{run_count}轮，报告以每轮的manifest、summary、文件校验和为唯一输入。{html.escape(scope_text)} 完整性：{'完整' if summary['complete'] else '不完整'}。动态验收以真实物理碰撞为距离门槛；小于0.10 m和actor安全让停保留为警告。</p></header><section class='cards'>{cards}</section>{methodology}<section class='panel'><h2>完整性与问题</h2><p class='bad'>{issue_text}</p></section><section class='panel'><h2>避障演示视频</h2><p class='muted'>4×速录制；可在此页面直接播放，也可使用视频控件打开链接。</p>{demo_videos}</section><section class='panel'><h2>测试地图</h2><p class='muted'>每次只两组并排对比；点击任意地图可在新标签页打开原尺寸查看。</p>{map_sections}</section><section class='panel'><h2>逐轮实际 GT 路径</h2><p class='muted'>路径来自该轮 <code>ground_truth.csv.gz</code>，叠加在 <code>warehouse_new</code> OccupancyGrid 上；绿点为起点，红点为终点。点击路径图可打开原尺寸。{trajectory_note}</p><section class='filters'><label>条件 <select id='condition'><option value='all'>全部</option>{condition_options}</select></label><label>seed <select id='seed'><option value='all'>全部</option>{seed_options}</select></label><label>外观 <select id='profile'><option value='all'>全部</option>{profile_options}</select></label><label>结果 <select id='result'><option value='all'>全部</option><option value='pass'>通过</option><option value='fail'>失败</option></select></label></section><label>匹配轮次 <select id='trajectory'></select></label><p id='trajectory-empty' class='muted'></p><a id='trajectory-link' target='_blank' rel='noopener'><img id='trajectory-image' alt='实际 GT 路径' hidden></a></section><section class='panel statistics'><h2>统计可视化</h2><p class='muted'>点击统计图可打开原尺寸。</p>{statistics_images}</section><section class='panel'><h2>运行明细</h2><table><thead><tr><th>条件</th><th>seed</th><th>外观</th><th>变体</th><th>严格</th><th>无碰撞</th><th>最小净距(m)</th><th>actor让停</th><th>时长(s)</th><th>失败原因</th><th>警告</th><th>路径</th></tr></thead><tbody>{rows}</tbody></table></section><footer><p>机器可读结果：benchmark.json / benchmark.csv；证据索引：evidence_index.json；不复制MCAP。</p></footer></main><script id='trajectory-data' type='application/json'>{records_json}</script><script>const records=JSON.parse(document.getElementById('trajectory-data').textContent),condition=document.getElementById('condition'),seed=document.getElementById('seed'),profile=document.getElementById('profile'),result=document.getElementById('result'),trajectory=document.getElementById('trajectory'),image=document.getElementById('trajectory-image'),link=document.getElementById('trajectory-link'),empty=document.getElementById('trajectory-empty');function matches(x){{return(condition.value==='all'||x.condition===condition.value)&&(seed.value==='all'||String(x.seed)===seed.value)&&(profile.value==='all'||x.profile===profile.value)&&(result.value==='all'||x.result===result.value)}}function showTrajectory(){{const item=records.find(x=>x.path&&x.path===trajectory.value);image.hidden=!item;link.hidden=!item;empty.textContent=item?'':(records.some(matches)?'匹配的轮次缺少 ground_truth.csv.gz，无法绘制实际路径。':'当前筛选没有匹配轮次。');if(item){{image.src=item.path;image.alt=item.label;link.href=item.path}}}}function apply(){{const options=records.filter(matches).filter(x=>x.path);const prior=trajectory.value;trajectory.replaceChildren();for(const item of options){{const option=document.createElement('option');option.value=item.path;option.textContent=item.label;trajectory.appendChild(option)}}if(options.some(x=>x.path===prior))trajectory.value=prior;document.querySelectorAll('tbody tr').forEach(x=>x.hidden=!matches({{condition:x.dataset.condition,seed:x.dataset.seed,profile:x.dataset.profile,result:x.dataset.result}}));showTrajectory()}}for(const item of [condition,seed,profile,result])item.onchange=apply;trajectory.onchange=showTrajectory;apply();</script></html>"""


def write_4x20_report(
    summary: Mapping[str, Any], output_directory: str | Path, *, replace_output: bool = False
) -> Path:
    root = Path(output_directory).expanduser().resolve()
    if root.exists():
        if not replace_output:
            # A full report shares its campaign container with previously emitted
            # static/dynamic subreports.  Those immutable child reports are safe
            # to retain; any other content means the requested report could be
            # overwritten and must be rejected.
            allowed_subreports = {"static_2x20", "dynamic_2x20"}
            unexpected = [item.name for item in root.iterdir() if item.name not in allowed_subreports]
            if unexpected:
                raise Campaign4x20Error(f"refusing to overwrite report directory: {root}")
    else:
        root.mkdir(parents=True)
    figures = root / "figures"
    # Older report versions embedded the unreadable, composite 2×2 map.  It is
    # a generated artifact, safe to discard on refresh in every report scope.
    (figures / "kujiale_4x20_test_matrix_map.png").unlink(missing_ok=True)
    figures_written = _plot_figures(summary, figures)
    _plot_trajectory_figures(summary, figures)
    map_figures = _copy_map_figures(summary, figures)
    clean = _clean(summary)
    (root / "benchmark.json").write_text(json.dumps(clean, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")
    fields = ["condition_id", "kind", "seed", "appearance_profile_id", "nav2_profile", "variant_id", "strict_success", "physical_collision_free", "data_complete", "checksums_verified", "dynamic_interaction_complete", "minimum_actor_clearance_m", "dynamic_safety_yield", "path_deviation_percent", "ground_truth_path_length_m", "duration_sec", "maximum_route_recoveries", "failure_reason", "warning_reason"]
    with (root / "benchmark.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows({key: row.get(key) for key in fields} for row in clean["runs"])
    evidence = [{"condition_id": row["condition_id"], "seed": row["seed"], "appearance_profile_id": row["appearance_profile_id"], "nav2_profile": row["nav2_profile"], "manifest_path": row["manifest_path"], "evidence_dir": row["evidence_dir"]} for row in summary["runs"]]
    (root / "evidence_index.json").write_text(json.dumps(evidence, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    title = str(summary["title"])
    scope = str(summary["scope"])
    markdown = f"# {title}\n\n"
    markdown += f"结论：**{'通过' if summary['passed'] else '未通过'}**；证据完整：{'是' if summary['complete'] else '否'}。\n\n"
    if scope != "full":
        markdown += "本报告只覆盖本次静态或动态 2×20 证据，不能单独作为完整 4×20 验收结论，也不会自动合并不同批次。\n\n"
    markdown += _methodology_markdown(summary)
    for condition, entry in summary["conditions"].items():
        markdown += f"- {condition}: 严格 {entry['strict_success']['numerator']}/20，无碰撞 {entry['physical_collision_free']['numerator']}/20，{'通过' if entry['passed'] else '未通过'}。\n"
    for figure in map_figures:
        markdown += f"\n![测试地图](figures/{figure.name})\n"
    markdown += "\n![条件总览](figures/condition_overview.png)\n"
    (root / "report.md").write_text(markdown, encoding="utf-8")
    dictionary = "# 数据字典\n\n"
    dictionary += "`benchmark.json` 是本报告范围内的验收、完整性和逐轮指标的机器可读来源。`evidence_index.json` 只索引原始证据目录，不复制MCAP。`condition_id` 为实验条件，`appearance_profile_id` 是本轮固定的Session Layer配置；`nav2_profile` 记录静态的 `stable` 或动态的 `dynamic_avoidance` 导航参数配置。`minimum_actor_clearance_m` 是本轮所有动态 actor 的保守最小净距，`dynamic_safety_yield` 表示 actor 是否执行保护让停；两者在无真实物理碰撞时属于风险警告，写入 `warning_reason` 而不是 `failure_reason`。报告首页和 `report.md` 使用通用公式 `ASR_s=N_s^succ/N_s`、`ASR_d=N_d^succ/N_d`、`NSR=N_goal/N`；本实验分别以导航动作、接触检测和 actor 状态机测量公式中的事件，证据校验仅保证结果可追溯。\n"
    if scope != "full":
        dictionary += "\n本报告是独立的 2×20 子报告，不会与其他批次自动合并。\n"
    (root / "data_dictionary.md").write_text(dictionary, encoding="utf-8")
    (root / "index.html").write_text(
        _dashboard(clean, figures_written, map_figures), encoding="utf-8"
    )
    plt, PdfPages = _matplotlib()
    with PdfPages(root / "report.pdf") as pdf:
        _write_methodology_pdf_page(plt, pdf, clean)
        for path in [*map_figures, *figures_written]:
            image = plt.imread(path); fig, axis = plt.subplots(figsize=(16, 9)); axis.imshow(image); axis.axis("off"); axis.set_title(title); pdf.savefig(fig, bbox_inches="tight"); plt.close(fig)
    _write_checksums(root)
    return root


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="validate and report the Kujiale 4x20 appearance campaign")
    parser.add_argument("--run-root", required=True)
    parser.add_argument("--output-directory")
    parser.add_argument("--status", action="store_true")
    parser.add_argument("--scope", choices=tuple(SCOPES), default="full")
    parser.add_argument("--replace-output", action="store_true")
    parsed = parser.parse_args(args)
    if bool(parsed.output_directory) == bool(parsed.status):
        parser.error("provide exactly one of --output-directory or --status")
    summary = summarize_4x20(parsed.run_root, scope=parsed.scope)
    if parsed.status:
        print(json.dumps({"scope": summary["scope"], "complete": summary["complete"], "passed": summary["passed"], "issues": summary["issues"], "conditions": summary["conditions"]}, ensure_ascii=False))
        return
    output = write_4x20_report(summary, parsed.output_directory, replace_output=parsed.replace_output)
    print(json.dumps({"output": str(output), "scope": summary["scope"], "complete": summary["complete"], "passed": summary["passed"]}, ensure_ascii=False))
    raise SystemExit(0 if summary["passed"] else 2)


if __name__ == "__main__":
    main()
