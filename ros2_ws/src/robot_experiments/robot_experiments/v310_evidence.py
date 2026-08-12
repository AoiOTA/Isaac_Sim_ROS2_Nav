"""Attempt30/A21 V3.10 research evidence derived from recorded manifests.

This module is deliberately offline-only.  It reads immutable run artifacts and
never publishes ROS messages or participates in route selection.
"""

from __future__ import annotations

import csv
import gzip
import heapq
import math
from pathlib import Path
from statistics import mean
from typing import Any, Iterable, Mapping, Sequence


def _finite(value: Any) -> float | None:
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return float(value)
    return None


def _mean(values: Iterable[Any]) -> float | None:
    finite = [number for value in values if (number := _finite(value)) is not None]
    return mean(finite) if finite else None


def _maximum(values: Iterable[Any]) -> float | None:
    finite = [number for value in values if (number := _finite(value)) is not None]
    return max(finite) if finite else None


def _write_csv(path: Path, fields: Sequence[str], rows: Iterable[Mapping[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fields})


def _selected_edge_ids(manifest: Mapping[str, Any]) -> str:
    return ";".join(
        ",".join(str(value) for value in route.get("edge_ids", []))
        for route in manifest.get("canonical_routes", [])
        if isinstance(route, Mapping)
    )


def _risk_exposure(manifest: Mapping[str, Any]) -> float | None:
    return _mean(
        sample.get("risk_exposure_rate")
        for sample in manifest.get("planning_prior_samples", [])
        if isinstance(sample, Mapping)
    )


def _navigation_rows(group: str, records: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    rows = []
    for record in records:
        manifest = record["manifest"]
        row = {
            "run": record["run_index"],
            "seed": record["seed"],
            "success": record["task_success"],
            "collision_free": record["collision_free"],
            "path_length_m": manifest.get("metrics", {}).get("ground_truth_path_length_m"),
            "path_deviation_percent": record["executed_deviation_percent"],
            "execution_time_sec": record["execution_time_sec"],
            "selected_route_edge_ids": _selected_edge_ids(manifest),
            "nav2_profile": manifest.get("nav2_profile"),
            "module2_healthy_fraction": manifest.get("module2_health", {}).get(
                "healthy_fraction"
            ),
        }
        if group == "dynamic":
            selection = manifest.get("dynamic_selection", {})
            row.update(
                {
                    "actor_configuration": selection.get("variant_id"),
                    "dynamic_interaction_complete": record[
                        "dynamic_interaction_complete"
                    ],
                    "dr_guidance_healthy_count": manifest.get(
                        "module2_health", {}
                    ).get("healthy_count"),
                    "mean_risk_exposure_rate": _risk_exposure(manifest),
                }
            )
        rows.append(row)
    return rows


def _graph_adjacency(manifest: Mapping[str, Any], costs: Mapping[int, float]):
    adjacency: dict[int, list[tuple[int, int, float]]] = {}
    for edge in manifest.get("navigation_graph", {}).get("edges", []):
        edge_id = int(edge["id"])
        weight = costs.get(edge_id, _finite(edge.get("length_m")) or math.inf)
        adjacency.setdefault(int(edge["from_node"]), []).append(
            (int(edge["to_node"]), edge_id, float(weight))
        )
    return adjacency


def _shortest_path(
    adjacency: Mapping[int, Sequence[tuple[int, int, float]]],
    start: int,
    goal: int,
    excluded: frozenset[int] = frozenset(),
) -> tuple[float, tuple[int, ...]] | None:
    queue: list[tuple[float, int, tuple[int, ...]]] = [(0.0, start, ())]
    best: dict[int, float] = {}
    while queue:
        cost, node, path = heapq.heappop(queue)
        if cost >= best.get(node, math.inf):
            continue
        best[node] = cost
        if node == goal:
            return cost, path
        for target, edge_id, weight in adjacency.get(node, ()):
            if edge_id not in excluded and math.isfinite(weight):
                heapq.heappush(queue, (cost + weight, target, path + (edge_id,)))
    return None


def _real_alternatives(
    manifest: Mapping[str, Any], start: int, goal: int, costs: Mapping[int, float]
) -> list[tuple[float, tuple[int, ...]]]:
    adjacency = _graph_adjacency(manifest, costs)
    first = _shortest_path(adjacency, start, goal)
    if first is None:
        return []
    candidates = {first}
    for edge_id in first[1]:
        candidate = _shortest_path(adjacency, start, goal, frozenset({edge_id}))
        if candidate is not None:
            candidates.add(candidate)
    return sorted(candidates)[:8]


def _guidance_rows(records: Mapping[str, Sequence[Mapping[str, Any]]]):
    rows: list[dict[str, Any]] = []
    for group, group_records in records.items():
        for record in group_records:
            manifest = record["manifest"]
            diagnostics = {
                int(item["request_id"]): item
                for item in manifest.get("srdr_edge_diagnostics", [])
            }
            route_costs = {
                int(item["request_id"]): item
                for item in manifest.get("route_edge_costs", [])
            }
            health = {
                int(item["request_id"]): item
                for item in manifest.get("module2_health", {}).get("responses", [])
            }
            for route in manifest.get("canonical_routes", []):
                request_id = int(route["request_id"])
                nodes = tuple(int(value) for value in route.get("node_ids", []))
                selected = tuple(int(value) for value in route.get("edge_ids", []))
                cost_record = route_costs.get(request_id, {})
                cost_edges = cost_record.get("edges", [])
                structural = {
                    int(edge["edge_id"]): float(edge["structural_cost_m"])
                    for edge in cost_edges
                    if _finite(edge.get("structural_cost_m")) is not None
                }
                final = {
                    int(edge["edge_id"]): float(edge["final_cost_m"])
                    for edge in cost_edges
                    if _finite(edge.get("final_cost_m")) is not None
                }
                structural_paths = (
                    _real_alternatives(manifest, nodes[0], nodes[-1], structural)
                    if len(nodes) >= 2
                    else []
                )
                final_paths = (
                    _real_alternatives(manifest, nodes[0], nodes[-1], final)
                    if len(nodes) >= 2
                    else []
                )
                diag_edges = diagnostics.get(request_id, {}).get("edges", [])
                usable = [edge for edge in diag_edges if edge.get("usable") is True]
                selected_delta = sum(
                    float(edge.get("applied_module2_delta_m", 0.0))
                    for edge in cost_edges
                    if int(edge.get("edge_id", -1)) in selected
                )
                response = health.get(request_id, {})
                rows.append(
                    {
                        "group": group,
                        "run": record["run_index"],
                        "request_id": request_id,
                        "start_node": nodes[0] if nodes else None,
                        "goal_node": nodes[-1] if nodes else None,
                        "real_alternative_route_count": len(structural_paths),
                        "structural_best_edge_ids": ",".join(
                            str(value) for value in structural_paths[0][1]
                        )
                        if structural_paths
                        else "",
                        "srdr_best_edge_ids": ",".join(
                            str(value) for value in final_paths[0][1]
                        )
                        if final_paths
                        else "",
                        "selected_edge_ids": ",".join(str(value) for value in selected),
                        "route_changed_by_srdr": bool(
                            structural_paths
                            and final_paths
                            and structural_paths[0][1] != final_paths[0][1]
                        ),
                        "module2_healthy": response.get("healthy", False),
                        "model_id": response.get("model_id", ""),
                        "usable_edge_count": len(usable),
                        "total_edge_count": len(diag_edges),
                        "selected_applied_module2_delta_m": selected_delta,
                        "maximum_cost_delta_m": response.get("maximum_cost_delta_m"),
                    }
                )
    return rows


def _full_prior_samples(manifest: Mapping[str, Any]):
    return [
        sample
        for sample in manifest.get("planning_prior_samples", [])
        if isinstance(sample, Mapping)
        and isinstance(sample.get("place_belief"), list)
        and len(sample["place_belief"]) == 256
    ]


def _representation_rows(records: Mapping[str, Sequence[Mapping[str, Any]]]):
    rows = []
    for group, group_records in records.items():
        for record in group_records:
            samples = _full_prior_samples(record["manifest"])
            rows.append(
                {
                    "condition": group,
                    "run": record["run_index"],
                    "profile": record.get("profile"),
                    "sample_count": len(samples),
                    "mean_p_corr_peak": _mean(item.get("place_peak") for item in samples),
                    "mean_p_corr_entropy": _mean(
                        item.get("place_entropy_normalized") for item in samples
                    ),
                    "mean_context_uncertainty": _mean(
                        item.get("context_uncertainty") for item in samples
                    ),
                    "maximum_sr": _maximum(
                        value for item in samples for value in item.get("value_sr", [])
                    ),
                    "maximum_dr": _maximum(
                        value
                        for item in samples
                        for value in item.get("future_cost_dr", [])
                    ),
                    "maximum_dynamic_cost": _maximum(
                        value for item in samples for value in item.get("dynamic_cost", [])
                    ),
                    "mean_risk_exposure_rate": _mean(
                        item.get("risk_exposure_rate") for item in samples
                    ),
                    "trusted_write_fraction": (
                        sum(item.get("trusted_write") is True for item in samples)
                        / len(samples)
                        if samples
                        else None
                    ),
                }
            )
    return rows


def _cosine(left, right) -> float | None:
    import numpy as np

    a = np.asarray(left, dtype=float)
    b = np.asarray(right, dtype=float)
    denominator = float(np.linalg.norm(a) * np.linalg.norm(b))
    if not denominator or not np.isfinite(denominator):
        return None
    return float(np.dot(a, b) / denominator)


def _appearance_stability(records: Sequence[Mapping[str, Any]]):
    import numpy as np

    fields = ("place_belief", "value_sr", "future_cost_dr")
    by_profile: dict[str, list[Mapping[str, Any]]] = {}
    for record in records:
        by_profile.setdefault(str(record.get("profile")), []).extend(
            _full_prior_samples(record["manifest"])
        )
    averages = {
        profile: {
            field: np.mean(
                np.asarray([sample[field] for sample in samples], dtype=float), axis=0
            )
            for field in fields
        }
        for profile, samples in by_profile.items()
        if samples
    }
    if not averages:
        return []
    reference_name = "dim_warm" if "dim_warm" in averages else sorted(averages)[0]
    reference = averages[reference_name]
    return [
        {
            "profile": profile,
            "reference_profile": reference_name,
            "sample_count": len(by_profile[profile]),
            "p_corr_cosine_similarity": _cosine(
                values["place_belief"], reference["place_belief"]
            ),
            "sr_cosine_similarity": _cosine(
                values["value_sr"], reference["value_sr"]
            ),
            "dr_cosine_similarity": _cosine(
                values["future_cost_dr"], reference["future_cost_dr"]
            ),
        }
        for profile, values in sorted(averages.items())
    ]


def _read_ground_truth(root: Path):
    with gzip.open(root / "ground_truth.csv.gz", "rt", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    return [float(row["x"]) for row in rows], [float(row["y"]) for row in rows]


def _plot_navigation(records: Mapping[str, Sequence[Mapping[str, Any]]], output: Path):
    import matplotlib.pyplot as plt

    static = records["static"]
    figure, axis = plt.subplots(figsize=(10, 4.8), constrained_layout=True)
    values = [
        value if (value := _finite(record["executed_deviation_percent"])) is not None else math.nan
        for record in static
    ]
    axis.bar(range(1, 21), values, color="#2563eb")
    axis.axhline(20.0, color="#dc2626", linestyle="--", label="20% limit")
    axis.set(xlabel="run", ylabel="executed path deviation (%)", title="Static path deviation")
    axis.set_xticks(range(1, 21))
    axis.legend()
    figure.savefig(output / "static_path_deviation.png", dpi=180)
    plt.close(figure)

    dynamic = records["dynamic"]
    figure, axes = plt.subplots(1, 2, figsize=(11, 4.8), constrained_layout=True)
    colors = ["#16a34a" if row["task_success"] else "#dc2626" for row in dynamic]
    axes[0].bar(range(1, 21), [_risk_exposure(row["manifest"]) or 0.0 for row in dynamic], color=colors)
    axes[0].set(xlabel="run", ylabel="mean p_corr dot dynamic_cost", title="Dynamic risk exposure")
    for row in dynamic:
        x, y = _read_ground_truth(row["root"])
        axes[1].plot(x, y, linewidth=0.7, alpha=0.3)
    axes[1].set(xlabel="map x (m)", ylabel="map y (m)", title="Dynamic executed trajectories")
    axes[1].set_aspect("equal", adjustable="box")
    figure.savefig(output / "dynamic_summary.png", dpi=180)
    plt.close(figure)


def _plot_fields(records: Mapping[str, Sequence[Mapping[str, Any]]], output: Path):
    import matplotlib.pyplot as plt
    import numpy as np

    selected = []
    for group in ("static", "dynamic", "appearance"):
        samples = _full_prior_samples(records[group][0]["manifest"])
        if samples:
            selected.append((group, max(samples, key=lambda item: item.get("risk_exposure_rate") or 0.0)))
    if not selected:
        return
    fields = (("place_belief", "p_corr"), ("value_sr", "SR"), ("future_cost_dr", "DR"), ("dynamic_cost", "dynamic cost"))
    figure, axes = plt.subplots(len(selected), 4, figsize=(14, 3.2 * len(selected)), constrained_layout=True)
    axes = np.asarray(axes).reshape(len(selected), 4)
    for row_index, (group, sample) in enumerate(selected):
        for column, (field, title) in enumerate(fields):
            values = np.asarray(sample[field], dtype=float).reshape(16, 16)
            image = axes[row_index, column].imshow(values, origin="lower", cmap="viridis")
            axes[row_index, column].set_title(f"{group}: {title}")
            figure.colorbar(image, ax=axes[row_index, column], fraction=0.046)
    figure.savefig(output / "representation_static_dynamic_appearance.png", dpi=180)
    plt.close(figure)


def _plot_dynamic_temporal(
    records: Sequence[Mapping[str, Any]], output: Path
) -> None:
    import matplotlib.pyplot as plt
    import numpy as np

    samples = _full_prior_samples(records[0]["manifest"])
    if len(samples) < 3:
        return
    peak_index = max(
        range(len(samples)),
        key=lambda index: samples[index].get("risk_exposure_rate") or 0.0,
    )
    selected = (samples[0], samples[peak_index], samples[-1])
    figure, axes = plt.subplots(2, 3, figsize=(11, 7), constrained_layout=True)
    for column, (sample, phase) in enumerate(zip(selected, ("before", "peak", "after"))):
        for row, (field, name) in enumerate(
            (("future_cost_dr", "DR"), ("dynamic_cost", "dynamic cost"))
        ):
            values = np.asarray(sample[field], dtype=float).reshape(16, 16)
            image = axes[row, column].imshow(values, origin="lower", cmap="magma")
            axes[row, column].set_title(
                f"{phase}: {name}\nt={sample.get('stamp_s')} s"
            )
            figure.colorbar(image, ax=axes[row, column], fraction=0.046)
    figure.savefig(output / "dynamic_dr_before_peak_after.png", dpi=180)
    plt.close(figure)


def _plot_appearance_stability(rows: Sequence[Mapping[str, Any]], output: Path) -> None:
    if not rows:
        return
    import matplotlib.pyplot as plt
    import numpy as np

    profiles = [str(row["profile"]) for row in rows]
    x = np.arange(len(profiles), dtype=float)
    figure, axis = plt.subplots(figsize=(9, 4.8), constrained_layout=True)
    for offset, field, label in (
        (-0.25, "p_corr_cosine_similarity", "p_corr"),
        (0.0, "sr_cosine_similarity", "SR"),
        (0.25, "dr_cosine_similarity", "DR"),
    ):
        axis.bar(
            x + offset,
            [row.get(field) if row.get(field) is not None else math.nan for row in rows],
            width=0.24,
            label=label,
        )
    axis.set_xticks(x, profiles)
    axis.set_ylim(0.0, 1.05)
    axis.set_ylabel("cosine similarity to dim_warm aggregate")
    axis.set_title("V3.10 appearance representation stability")
    axis.legend()
    figure.savefig(output / "appearance_representation_stability.png", dpi=180)
    plt.close(figure)


def write_v310_evidence(
    records: Mapping[str, Sequence[Mapping[str, Any]]], output: Path
) -> list[str]:
    """Write the Plan's V3.10 CSV and PNG deliverables."""
    output.mkdir(parents=True, exist_ok=True)
    static_rows = _navigation_rows("static", records["static"])
    dynamic_rows = _navigation_rows("dynamic", records["dynamic"])
    static_fields = list(static_rows[0])
    dynamic_fields = list(dynamic_rows[0])
    _write_csv(output / "static_20run_summary.csv", static_fields, static_rows)
    _write_csv(output / "dynamic_20run_summary.csv", dynamic_fields, dynamic_rows)

    guidance = _guidance_rows(records)
    _write_csv(output / "module2_guidance_summary.csv", list(guidance[0]), guidance)
    representation = _representation_rows(records)
    _write_csv(output / "representation_summary.csv", list(representation[0]), representation)
    appearance = _appearance_stability(records["appearance"])
    if appearance:
        _write_csv(
            output / "appearance_stability_summary.csv",
            list(appearance[0]),
            appearance,
        )

    _plot_navigation(records, output)
    _plot_fields(records, output)
    _plot_dynamic_temporal(records["dynamic"], output)
    _plot_appearance_stability(appearance, output)
    return [
        "static_20run_summary.csv",
        "static_path_deviation.png",
        "dynamic_20run_summary.csv",
        "dynamic_summary.png",
        "module2_guidance_summary.csv",
        "representation_summary.csv",
        "representation_static_dynamic_appearance.png",
        "dynamic_dr_before_peak_after.png",
        "appearance_stability_summary.csv",
        "appearance_representation_stability.png",
    ]


__all__ = ["write_v310_evidence"]
