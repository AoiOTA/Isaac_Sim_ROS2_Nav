"""Aggregate the exact Attempt30/A21 3x20 Final Qualification campaign."""

from __future__ import annotations

import argparse
import ast
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
from statistics import mean
from typing import Any, Iterable, Mapping


EXPECTED = {
    "static": "attempt30_a21_qualification_static",
    "dynamic": "attempt30_a21_qualification_dynamic",
    "appearance": "attempt30_a21_qualification_appearance",
}
WHOLE_HOUSE_ROUTE = ("G2", "G3", "G4", "G5", "G1")
APPEARANCE_PROFILES = ("dim_warm", "dim_cool", "bright_warm", "bright_cool")


class QualificationError(ValueError):
    """Evidence is incomplete or does not match the authorized campaign."""


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise QualificationError(f"invalid JSON: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise QualificationError(f"JSON is not an object: {path}")
    return value


def _checksums_verified(root: Path) -> bool:
    inventory = root / "checksums.sha256"
    if not inventory.is_file():
        return False
    for line in inventory.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            digest, relative = line.split("  ", 1)
            candidate = (root / relative).resolve()
            candidate.relative_to(root.resolve())
        except (ValueError, OSError):
            return False
        if (
            len(digest) != 64
            or not candidate.is_file()
            or hashlib.sha256(candidate.read_bytes()).hexdigest() != digest
        ):
            return False
    return True


def _task_success(manifest: Mapping[str, Any]) -> bool:
    legs = manifest.get("legs")
    metrics = manifest.get("metrics")
    if not isinstance(legs, list) or not isinstance(metrics, Mapping):
        return False
    return bool(
        len(legs) == 5
        and tuple(str(leg.get("id")) for leg in legs) == WHOLE_HOUSE_ROUTE
        and all(int(leg.get("nav2_status", -1)) == 4 for leg in legs)
        and int(manifest.get("nav2_status", -1)) == 4
        and float(metrics.get("ground_truth_position_error_m", math.inf)) <= 0.25
        and abs(float(metrics.get("ground_truth_orientation_error_rad", math.inf)))
        <= math.radians(10.0)
        and metrics.get("final_still_duration_met") is True
    )


def _route_valid(manifest: Mapping[str, Any]) -> bool:
    poses = manifest.get("route_poses")
    legs = manifest.get("legs")
    routes = manifest.get("canonical_routes")
    if not isinstance(legs, list) or not 1 <= len(legs) <= len(WHOLE_HOUSE_ROUTE):
        return False
    if not isinstance(routes, list) or not routes:
        return False
    route_request_ids = {
        item.get("request_id") for item in routes if isinstance(item, Mapping)
    }
    return bool(
        manifest.get("navigation_execution_backend") == "route_guided"
        and isinstance(poses, list)
        and tuple(str(item.get("id")) for item in poses) == WHOLE_HOUSE_ROUTE
        and tuple(str(item.get("id")) for item in legs)
        == WHOLE_HOUSE_ROUTE[: len(legs)]
        and all(
            isinstance(item, Mapping)
            and isinstance(item.get("edge_ids"), list)
            and item.get("edge_ids")
            for item in routes
        )
        and all(
            item.get("route_request_id") in route_request_ids for item in legs
        )
    )


def _record(root: Path) -> dict[str, Any]:
    manifest = _read_json(root / "run_manifest.json")
    summary = _read_json(root / "run_summary.json")
    metrics = manifest.get("metrics", {})
    collision_free = bool(summary.get("physical_collision_free"))
    return {
        "root": root,
        "manifest": manifest,
        "summary": summary,
        "scenario_id": str(manifest.get("scenario_id", "")),
        "run_index": int(manifest.get("run_index", 0)),
        "seed": int(manifest.get("random_seed", 0)),
        "profile": manifest.get("appearance", {}).get("profile_id"),
        "task_success": _task_success(manifest),
        "collision_free": collision_free,
        "collision_free_success": _task_success(manifest) and collision_free,
        "route_valid": _route_valid(manifest),
        "data_complete": bool(summary.get("data_complete")),
        "checksums_verified": _checksums_verified(root),
        "executed_deviation_percent": metrics.get("path_deviation_percent"),
        "planned_deviation_percent": metrics.get(
            "planned_path_deviation_percent"
        ),
        "execution_time_sec": metrics.get("execution_time_sec"),
        "minimum_clearance_m": metrics.get("minimum_clearance_m"),
        "localization_healthy": bool(
            manifest.get("observability", {}).get("localization_healthy")
        ),
        "module2_health": manifest.get("module2_health", {}),
        "unexpected_abort": int(manifest.get("nav2_status", -1)) != 4,
        "dynamic_interaction_complete": bool(
            manifest.get("dynamic_interaction", {}).get("complete")
        ),
    }


def load_records(input_root: Path) -> dict[str, list[dict[str, Any]]]:
    result: dict[str, list[dict[str, Any]]] = {}
    for group, scenario_id in EXPECTED.items():
        scenario_root = input_root / scenario_id
        roots = sorted(
            path.parent for path in scenario_root.glob("run-*/run_manifest.json")
        )
        if len(roots) != 20:
            raise QualificationError(
                f"{group} requires exactly 20 run directories, found {len(roots)}"
            )
        records = [_record(root) for root in roots]
        if any(item["scenario_id"] != scenario_id for item in records):
            raise QualificationError(f"{group} contains a foreign scenario")
        if sorted(item["run_index"] for item in records) != list(range(1, 21)):
            raise QualificationError(f"{group} run indices must be exactly 1..20")
        result[group] = records
    return result


def _finite(values: Iterable[Any]) -> list[float]:
    return [
        float(value)
        for value in values
        if isinstance(value, (int, float)) and math.isfinite(float(value))
    ]


def aggregate(records: Mapping[str, list[dict[str, Any]]]) -> dict[str, Any]:
    groups: dict[str, Any] = {}
    for group, rows in records.items():
        task_successes = sum(item["task_success"] for item in rows)
        collision_free = sum(item["collision_free"] for item in rows)
        collision_free_success = sum(item["collision_free_success"] for item in rows)
        evidence_complete = all(
            item["data_complete"]
            and item["checksums_verified"]
            and item["route_valid"]
            for item in rows
        )
        times = _finite(item["execution_time_sec"] for item in rows)
        clearances = _finite(item["minimum_clearance_m"] for item in rows)
        group_result: dict[str, Any] = {
            "run_count": len(rows),
            "task_success_count": task_successes,
            "collision_free_count": collision_free,
            "collision_free_success_count": collision_free_success,
            "evidence_complete": evidence_complete,
            "route": "G1 -> G2 -> G3 -> G4 -> G5 -> G1",
            "execution_time_sec": {
                "mean": mean(times) if times else None,
                "min": min(times) if times else None,
                "max": max(times) if times else None,
            },
            "minimum_clearance_m": min(clearances) if clearances else None,
            "unexpected_abort_count": sum(item["unexpected_abort"] for item in rows),
        }
        if group == "static":
            executed = _finite(
                item["executed_deviation_percent"]
                for item in rows
                if item["task_success"]
            )
            planned = _finite(
                item["planned_deviation_percent"]
                for item in rows
                if item["task_success"]
            )
            deviation_pass = bool(
                len(executed) == task_successes
                and executed
                and max(executed) <= 20.0
            )
            group_result.update(
                {
                    "required_collision_free_success_count": 19,
                    "executed_deviation_percent": {
                        "count": len(executed),
                        "mean": mean(executed) if executed else None,
                        "max": max(executed) if executed else None,
                        "limit": 20.0,
                        "passed": deviation_pass,
                    },
                    "planned_deviation_percent": {
                        "count": len(planned),
                        "mean": mean(planned) if planned else None,
                        "max": max(planned) if planned else None,
                    },
                    "passed": bool(
                        evidence_complete
                        and collision_free_success >= 19
                        and deviation_pass
                    ),
                }
            )
        elif group == "dynamic":
            interaction_count = sum(
                item["dynamic_interaction_complete"] for item in rows
            )
            successful_interaction_count = sum(
                item["task_success"] and item["dynamic_interaction_complete"]
                for item in rows
            )
            successful_interactions_complete = bool(
                successful_interaction_count == task_successes
            )
            group_result.update(
                {
                    "required_task_success_count": 18,
                    "required_collision_free_count": 18,
                    "dynamic_interaction_complete_count": interaction_count,
                    "successful_dynamic_interaction_complete_count": (
                        successful_interaction_count
                    ),
                    "successful_dynamic_interactions_complete": (
                        successful_interactions_complete
                    ),
                    "passed": bool(
                        evidence_complete
                        and successful_interactions_complete
                        and task_successes >= 18
                        and collision_free >= 18
                    ),
                }
            )
        else:
            profile_counts = {
                profile: sum(item["profile"] == profile for item in rows)
                for profile in APPEARANCE_PROFILES
            }
            response_count = sum(
                int(item["module2_health"].get("response_count", 0))
                for item in rows
                if isinstance(item["module2_health"], Mapping)
            )
            healthy_count = sum(
                int(item["module2_health"].get("healthy_count", 0))
                for item in rows
                if isinstance(item["module2_health"], Mapping)
            )
            group_result.update(
                {
                    "required_task_success_count": 18,
                    "required_collision_free_count": 18,
                    "profile_counts": profile_counts,
                    "localization_healthy_count": sum(
                        item["localization_healthy"] for item in rows
                    ),
                    "module2_health": {
                        "response_count": response_count,
                        "healthy_count": healthy_count,
                        "healthy_fraction": (
                            healthy_count / response_count if response_count else None
                        ),
                    },
                    "passed": bool(
                        evidence_complete
                        and all(value == 5 for value in profile_counts.values())
                        and task_successes >= 18
                        and collision_free >= 18
                    ),
                }
            )
        groups[group] = group_result
    return {
        "schema": "attempt30_a21_final_qualification_v1",
        "campaign": "Attempt30/A21 Final Qualification",
        "authorized_route": "G1 -> G2 -> G3 -> G4 -> G5 -> G1",
        "total_run_count": sum(value["run_count"] for value in groups.values()),
        "groups": groups,
        "passed": bool(
            sum(value["run_count"] for value in groups.values()) == 60
            and all(value["passed"] for value in groups.values())
        ),
    }


def _read_gt(root: Path) -> tuple[list[float], list[float]]:
    with gzip.open(root / "ground_truth.csv.gz", "rt", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    return ([float(row["x"]) for row in rows], [float(row["y"]) for row in rows])


def _draw_context(axis, first: Mapping[str, Any]) -> None:
    root = first["root"]
    pgm, metadata = root / "global_costmap.pgm", root / "global_costmap.json"
    if pgm.is_file() and metadata.is_file():
        import matplotlib.pyplot as plt

        image = plt.imread(pgm)
        info = _read_json(metadata)
        origin = info["origin"]
        resolution = float(info["resolution_m"])
        axis.imshow(
            image,
            cmap="gray",
            origin="lower",
            extent=[
                float(origin["x"]),
                float(origin["x"]) + image.shape[1] * resolution,
                float(origin["y"]),
                float(origin["y"]) + image.shape[0] * resolution,
            ],
            alpha=0.55,
            zorder=0,
        )
    manifest = first["manifest"]
    graph = manifest.get("navigation_graph") or {}
    for edge in graph.get("edges", []):
        points = edge.get("polyline", [])
        if len(points) >= 2:
            axis.plot(
                [point[0] for point in points],
                [point[1] for point in points],
                color="#64748b",
                linewidth=0.45,
                alpha=0.35,
                zorder=1,
            )
    edge_map = {int(edge["id"]): edge for edge in graph.get("edges", [])}
    for route in manifest.get("canonical_routes", []):
        for edge_id in route.get("edge_ids", []):
            edge = edge_map.get(int(edge_id))
            if edge and len(edge.get("polyline", [])) >= 2:
                points = edge["polyline"]
                axis.plot(
                    [point[0] for point in points],
                    [point[1] for point in points],
                    color="#2563eb",
                    linewidth=1.4,
                    alpha=0.7,
                    zorder=3,
                )
    for plan in manifest.get("smac_plans", [])[::max(1, len(manifest.get("smac_plans", [])) // 30 or 1)]:
        points = plan.get("points", [])
        if len(points) >= 2:
            axis.plot(
                [point[0] for point in points],
                [point[1] for point in points],
                color="#22c55e",
                linewidth=0.7,
                alpha=0.28,
                zorder=2,
            )
    progress = manifest.get("route_progress", [])
    if progress:
        projected = [item["projected_point"] for item in progress]
        lookahead = [item["lookahead"] for item in progress]
        axis.scatter(
            [item[0] for item in projected[::10]],
            [item[1] for item in projected[::10]],
            s=4,
            color="#0ea5e9",
            alpha=0.5,
            zorder=4,
            label="Route projection",
        )
        axis.scatter(
            [item[0] for item in lookahead[::10]],
            [item[1] for item in lookahead[::10]],
            s=5,
            color="#f59e0b",
            alpha=0.5,
            zorder=4,
            label="Lookahead",
        )
    poses = manifest.get("route_poses", [])
    for item in poses:
        position = item.get("position", [0.0, 0.0])
        axis.scatter(position[0], position[1], marker="*", s=80, color="#dc2626", zorder=7)
        axis.text(position[0] + 0.08, position[1] + 0.08, str(item.get("id")), fontsize=8)


def write_visuals(records: Mapping[str, list[dict[str, Any]]], output: Path) -> list[str]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output.mkdir(parents=True, exist_ok=True)
    written: list[str] = []
    colors = {"static": "#f97316", "dynamic": "#db2777", "appearance": "#7c3aed"}
    for group, rows in records.items():
        figure, axis = plt.subplots(figsize=(9, 10), constrained_layout=True)
        _draw_context(axis, rows[0])
        for row in rows:
            x, y = _read_gt(row["root"])
            axis.plot(x, y, color=colors[group], linewidth=0.8, alpha=0.25, zorder=5)
            if x:
                axis.scatter(x[0], y[0], s=12, color="#16a34a", zorder=6)
        axis.set_title(f"Attempt30/A21 {group}: 20 whole-house trajectories")
        axis.set_xlabel("map x (m)")
        axis.set_ylabel("map y (m)")
        axis.set_aspect("equal", adjustable="box")
        axis.grid(alpha=0.15)
        target = output / f"qualification_{group}_route_smac_overlay.png"
        figure.savefig(target, dpi=180)
        plt.close(figure)
        written.append(target.name)

    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for axis, profile in zip(axes.flat, APPEARANCE_PROFILES):
        row = next(item for item in records["appearance"] if item["profile"] == profile)
        image = plt.imread(row["root"] / "appearance_rgb_before_goal.ppm")
        axis.imshow(image)
        axis.set_title(profile)
        axis.axis("off")
    target = output / "qualification_appearance_profiles.png"
    figure.savefig(target, dpi=180)
    plt.close(figure)
    written.append(target.name)
    return written


def write_outputs(
    records: Mapping[str, list[dict[str, Any]]], report: dict[str, Any], output: Path
) -> None:
    output.mkdir(parents=True, exist_ok=True)
    report["visual_evidence"] = write_visuals(records, output)
    (output / "qualification.json").write_text(
        json.dumps(report, indent=2, sort_keys=True), encoding="utf-8"
    )
    with (output / "runs.csv").open("w", newline="", encoding="utf-8") as stream:
        fields = [
            "group", "run_index", "seed", "profile", "task_success",
            "collision_free", "collision_free_success", "route_valid",
            "data_complete", "checksums_verified", "executed_deviation_percent",
            "planned_deviation_percent", "execution_time_sec", "minimum_clearance_m",
            "localization_healthy", "unexpected_abort", "dynamic_interaction_complete",
        ]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for group, rows in records.items():
            for item in rows:
                writer.writerow({key: item.get(key) for key in fields} | {"group": group})
    lines = [
        "# Attempt30/A21 Final Qualification",
        "",
        f"Overall: **{'PASS' if report['passed'] else 'FAIL'}**",
        "",
        "Every run used `G1 -> G2 -> G3 -> G4 -> G5 -> G1`.",
        "",
        "| Group | Success | Collision-free | Collision-free success | Result |",
        "|---|---:|---:|---:|---|",
    ]
    for group, value in report["groups"].items():
        lines.append(
            f"| {group} | {value['task_success_count']}/20 | "
            f"{value['collision_free_count']}/20 | "
            f"{value['collision_free_success_count']}/20 | "
            f"{'PASS' if value['passed'] else 'FAIL'} |"
        )
    (output / "qualification.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    arguments = parser.parse_args()
    records = load_records(arguments.input_root.resolve())
    report = aggregate(records)
    write_outputs(records, report, arguments.output_dir.resolve())
    print(json.dumps({"passed": report["passed"], "groups": report["groups"]}, indent=2))


if __name__ == "__main__":
    main()
