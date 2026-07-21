import json
from pathlib import Path

import pytest

from robot_experiments.navigation_benchmark import (
    summarize_navigation_benchmark,
)
from robot_experiments.optimal_path import (
    OptimalPathError,
    load_occupancy_grid_reference,
)


def _write_map(tmp_path: Path, rows: list[list[int]]) -> Path:
    tmp_path.mkdir(parents=True, exist_ok=True)
    image = tmp_path / "map.pgm"
    image.write_text(
        "P2\n"
        f"{len(rows[0])} {len(rows)}\n"
        "255\n"
        + "\n".join(" ".join(str(value) for value in row) for row in rows)
        + "\n",
        encoding="ascii",
    )
    map_file = tmp_path / "map.yaml"
    map_file.write_text(
        "image: map.pgm\n"
        "resolution: 1.0\n"
        "origin: [0.0, 0.0, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n",
        encoding="utf-8",
    )
    return map_file


def test_occupancy_reference_returns_straight_and_detoured_lengths(tmp_path):
    clear = _write_map(tmp_path / "clear", [[254] * 5 for _ in range(5)])
    reference = load_occupancy_grid_reference(clear, clearance_m=0.0)
    assert reference.shortest_path_length((0.5, 0.5), (4.5, 0.5)) \
        == pytest.approx(4.0)

    detour_root = tmp_path / "detour"
    detour_root.mkdir()
    detour = _write_map(
        detour_root,
        [
            [254, 254, 254, 254, 254],
            [254, 254, 0, 254, 254],
            [254, 254, 0, 254, 254],
            [254, 254, 0, 254, 254],
            [254, 254, 0, 254, 254],
        ],
    )
    detoured = load_occupancy_grid_reference(detour, clearance_m=0.0)
    assert detoured.shortest_path_length((0.5, 0.5), (4.5, 0.5)) > 8.0


def test_occupancy_reference_rejects_blocked_goal(tmp_path):
    map_file = _write_map(
        tmp_path,
        [
            [254, 254],
            [254, 0],
        ],
    )
    reference = load_occupancy_grid_reference(map_file, clearance_m=0.0)
    with pytest.raises(OptimalPathError, match="goal is blocked"):
        reference.shortest_path_length((0.5, 0.5), (1.5, 0.5))


def _manifest(scenario_type, index, success, executed):
    return {
        "scenario_id": f"{scenario_type}_benchmark",
        "random_seed": 1000 + index,
        "map_version": "test_map",
        "posegraph_version": "test_map",
        "robot_config_hash": "a" * 64,
        "nav2_config_hash": "b" * 64,
        "dynamic_runtime_contract": {
            "verified": True,
            "enabled": scenario_type == "dynamic",
            "config_sha256": "c" * 64,
            "obstacle_ids": [],
        },
        "spawn_pose_name": "start",
        "usd_start_pose": {"position": [0.0, 0.0, 0.0], "yaw_deg": 0.0},
        "map_start_pose": {"position": [0.5, 0.5], "yaw_deg": 0.0},
        "goal_pose": {
            "frame_id": "map",
            "position": [4.5, 0.5],
            "yaw_deg": 0.0,
            "require_orientation": True,
        },
        "obstacle_trajectories": [],
        "physics_dt": 1.0 / 60.0,
        "rtf": 1.0,
        "result": "success" if success else "failure",
        "failure_reason": "" if success else "timed_out",
        "run_index": index,
        "scenario_type": scenario_type,
        "metrics": {"ground_truth_path_length_m": executed},
    }


def test_navigation_benchmark_enforces_rates_and_path_deviation(tmp_path):
    static_directory = tmp_path / "static"
    dynamic_directory = tmp_path / "dynamic"
    static_directory.mkdir()
    dynamic_directory.mkdir()
    for index in range(1, 21):
        (static_directory / f"{index}.json").write_text(
            json.dumps(_manifest("static", index, index != 20, 4.4)),
            encoding="utf-8",
        )
        (dynamic_directory / f"{index}.json").write_text(
            json.dumps(_manifest("dynamic", index, index > 2, 4.4)),
            encoding="utf-8",
        )
    map_file = _write_map(tmp_path / "benchmark_map", [[254] * 5 for _ in range(5)])
    summary = summarize_navigation_benchmark(
        static_directories=[static_directory],
        dynamic_directories=[dynamic_directory],
        map_file=map_file,
        clearance_m=0.0,
    )
    assert summary["static_avoidance"]["rate_percent"] == pytest.approx(95.0)
    assert summary["dynamic_avoidance"]["rate_percent"] == pytest.approx(90.0)
    assert summary["path_optimality"]["maximum_deviation_percent"] \
        == pytest.approx(10.0)
    assert summary["passed"] is True
