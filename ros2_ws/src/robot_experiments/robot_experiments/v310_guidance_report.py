"""Report paired real-GVG V3.10 guidance navigation runs."""

from __future__ import annotations

import argparse
import csv
import gzip
import json
from pathlib import Path


def _read(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def _trajectory(root: Path):
    with gzip.open(root / "ground_truth.csv.gz", "rt", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    return [float(row["x"]) for row in rows], [float(row["y"]) for row in rows]


def write_actual_pair(
    structural_root: Path,
    medium_root: Path,
    candidate_json: Path,
    output: Path,
    guided_label: str = "medium",
):
    candidate = _read(candidate_json)
    if not candidate.get("eligible") or not candidate.get("no_fake_edges"):
        raise ValueError("offline candidate is not an eligible real-GVG query")
    records = []
    graph_reference = None
    for profile, root in (("structural", structural_root), (guided_label, medium_root)):
        manifest = _read(root / "run_manifest.json")
        summary = _read(root / "run_summary.json")
        graph = manifest["navigation_graph"]
        graph_identity = json.dumps(graph, sort_keys=True, separators=(",", ":"))
        if graph_reference is None:
            graph_reference = graph_identity
        elif graph_identity != graph_reference:
            raise ValueError("paired runs did not use an identical physical GVG")
        matching_routes = [
            route
            for route in manifest["canonical_routes"]
            if route.get("node_ids")
            and int(route["node_ids"][0]) == int(candidate["start_node"])
            and int(route["node_ids"][-1]) == int(candidate["goal_node"])
        ]
        if len(matching_routes) != 1:
            raise ValueError("paired run has no unique candidate start-to-goal route")
        route = matching_routes[0]
        request_id = int(route["request_id"])
        graph_edges = {int(edge["id"]) for edge in graph["edges"]}
        selected = tuple(int(value) for value in route["edge_ids"])
        if not selected or not set(selected).issubset(graph_edges):
            raise ValueError("selected route contains an edge absent from the graph")
        responses = [
            item for item in manifest.get("module2_health", {}).get("responses", [])
            if int(item.get("request_id", -1)) == request_id
        ]
        if len(responses) != 1 or responses[0].get("healthy") is not True:
            raise ValueError("candidate route did not use one healthy Module2 response")
        target_response = responses[0]
        diagnostics = [
            item for item in manifest.get("srdr_edge_diagnostics", [])
            if int(item.get("request_id", -1)) == request_id
        ]
        target_diagnostics = diagnostics[0].get("edges", []) if diagnostics else []
        route_costs = [
            item for item in manifest.get("route_edge_costs", [])
            if int(item.get("request_id", -1)) == request_id
        ]
        target_costs = route_costs[0].get("edges", []) if route_costs else []
        records.append(
            {
                "profile": profile,
                "strict_success": bool(summary.get("strict_success")),
                "collision_free": bool(summary.get("physical_collision_free")),
                "data_complete": bool(summary.get("data_complete")),
                "selected_edge_ids": ",".join(str(value) for value in selected),
                "execution_time_sec": summary.get("execution_time_sec"),
                "path_length_m": manifest.get("metrics", {}).get(
                    "ground_truth_path_length_m"
                ),
                "target_request_id": request_id,
                "target_request_module2_healthy": target_response.get("healthy"),
                "target_request_model_id": target_response.get("model_id"),
                "usable_edge_count": sum(
                    item.get("usable") is True for item in target_diagnostics
                ),
                "maximum_cost_delta_m": target_response.get("maximum_cost_delta_m"),
                "selected_sr_penalty_m": sum(
                    float(item.get("sr_penalty_m", 0.0))
                    for item in target_diagnostics
                    if int(item.get("edge_id", -1)) in selected
                ),
                "selected_dr_penalty_m": sum(
                    float(item.get("dr_penalty_m", 0.0))
                    for item in target_diagnostics
                    if int(item.get("edge_id", -1)) in selected
                ),
                "selected_applied_module2_delta_m": sum(
                    float(item.get("applied_module2_delta_m", 0.0))
                    for item in target_costs
                    if int(item.get("edge_id", -1)) in selected
                ),
                "root": root,
                "manifest": manifest,
            }
        )
    route_changed = records[0]["selected_edge_ids"] != records[1]["selected_edge_ids"]
    passed = bool(
        route_changed
        and all(
            row["strict_success"] and row["collision_free"] and row["data_complete"]
            for row in records
        )
    )
    output.mkdir(parents=True, exist_ok=True)
    fields = [
        "profile",
        "strict_success",
        "collision_free",
        "data_complete",
        "selected_edge_ids",
        "execution_time_sec",
        "path_length_m",
        "target_request_id",
        "target_request_module2_healthy",
        "target_request_model_id",
        "usable_edge_count",
        "maximum_cost_delta_m",
        "selected_sr_penalty_m",
        "selected_dr_penalty_m",
        "selected_applied_module2_delta_m",
    ]
    with (output / "sr_guidance_actual.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows({key: row.get(key) for key in fields} for row in records)

    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    figure, axis = plt.subplots(figsize=(8, 8), constrained_layout=True)
    graph = records[0]["manifest"]["navigation_graph"]
    by_id = {int(edge["id"]): edge for edge in graph["edges"]}
    for edge in graph["edges"]:
        points = edge["polyline"]
        axis.plot(
            [point[0] for point in points],
            [point[1] for point in points],
            color="#cbd5e1",
            linewidth=0.5,
        )
    for row, color in zip(records, ("#2563eb", "#dc2626")):
        for index, edge_id in enumerate(
            int(value) for value in row["selected_edge_ids"].split(",")
        ):
            points = by_id[edge_id]["polyline"]
            axis.plot(
                [point[0] for point in points],
                [point[1] for point in points],
                color=color,
                linewidth=2.7,
                label=f"{row['profile']} canonical route" if index == 0 else None,
            )
        x, y = _trajectory(row["root"])
        axis.plot(
            x,
            y,
            color=color,
            linewidth=1.1,
            linestyle="--",
            alpha=0.75,
            label=f"{row['profile']} actual trajectory",
        )
    axis.set_title("V3.10 SR guidance: real GVG route and actual navigation")
    axis.set_xlabel("map x (m)")
    axis.set_ylabel("map y (m)")
    axis.set_aspect("equal", adjustable="box")
    axis.legend(fontsize=8)
    axis.grid(alpha=0.15)
    figure.savefig(output / "sr_guidance_actual.png", dpi=180)
    plt.close(figure)

    payload = {
        "schema": "bio_nav.v310.real_gvg_sr_guidance_actual.v1",
        "candidate": candidate,
        "profiles": [{key: row.get(key) for key in fields} for row in records],
        "identical_physical_graph": True,
        "route_changed": route_changed,
        "actual_navigation_changed": route_changed,
        "engineering_pass": passed,
    }
    (output / "sr_guidance_actual.json").write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return payload


def _single_root(path: Path) -> Path:
    roots = list(path.glob("**/run_manifest.json"))
    if len(roots) != 1:
        raise ValueError(f"expected exactly one run under {path}, found {len(roots)}")
    return roots[0].parent


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--structural", type=Path, required=True)
    parser.add_argument("--medium", type=Path, required=True)
    parser.add_argument("--candidate-json", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--guided-label", default="medium")
    arguments = parser.parse_args()
    print(
        json.dumps(
            write_actual_pair(
                _single_root(arguments.structural),
                _single_root(arguments.medium),
                arguments.candidate_json,
                arguments.output,
                arguments.guided_label,
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
