import json
import math
from pathlib import Path
import subprocess

import cv2
import numpy as np
import pytest
import yaml

from robot_experiments.module1_targeted_teaching import (
    PAIRED_BASELINE_TOPIC,
    PAIRED_STATE_TOPIC,
    PAIRED_VARIANT_TOPIC,
    cli,
    load_targeted_teaching_manifest,
    paired_stamp_summary,
    paired_state_error,
    state_id_for_map_xy,
)
from robot_experiments.v6_formal import V6ContractError
from robot_route_planner.feasibility import classify_edge
from robot_route_planner.map_io import load_occupancy_map
from robot_route_planner.models import Traversability
from robot_route_planner.ros_node import select_support_attachment


PACKAGE = Path(__file__).resolve().parents[1]
REPO = Path(__file__).resolve().parents[4]
CONFIG = PACKAGE / "config"
V1 = CONFIG / "module1_targeted_teaching_kujiale_v1.yaml"
T1 = CONFIG / "module1_targeted_teaching_kujiale_t1.yaml"
V2 = CONFIG / "module1_targeted_teaching_kujiale_v2.yaml"
T2 = CONFIG / "module1_targeted_teaching_kujiale_t2.yaml"
WRAPPER = REPO / "scripts/run_module1_targeted_teaching_kujiale.sh"
RECORDER = REPO / "scripts/record_module1_kujiale_scene.sh"


def _write(tmp_path: Path, document: dict) -> Path:
    path = tmp_path / "manifest.yaml"
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return path


def test_independent_validation_manifests_are_read_only_and_exact():
    v1 = load_targeted_teaching_manifest(V1)
    t1 = load_targeted_teaching_manifest(T1)

    assert v1.dataset == {
        "route_id": "V1",
        "role": "validation",
        "split": "validation",
        "status": "raw_until_audit",
        "evaluation_read_only": True,
    }
    assert t1.dataset == {
        "route_id": "T1",
        "role": "read_only_test",
        "split": "test",
        "status": "raw_until_audit",
        "evaluation_read_only": True,
    }
    assert v1.episode.seed == 7811
    assert t1.episode.seed == 7812
    assert v1.paired_appearance == {
        "baseline_profile_id": "baseline",
        "variant_profile_id": "dim_cool",
        "same_stamp_required": True,
        "simulation_time_advanced_during_capture": False,
    }
    assert t1.paired_appearance["variant_profile_id"] == "bright_warm"
    assert [leg.goal_id for leg in v1.mission_legs] == [
        "G4",
        "G2",
        "G5",
        "G3",
        "G1",
    ]
    assert [leg.goal_id for leg in t1.mission_legs] == [
        "G3",
        "G5",
        "G2",
        "G4",
        "G1",
    ]
    assert [state_id_for_map_xy(leg.x, leg.y) for leg in v1.mission_legs] == [
        117,
        200,
        85,
        181,
        40,
    ]
    assert [state_id_for_map_xy(leg.x, leg.y) for leg in t1.mission_legs] == [
        181,
        85,
        200,
        117,
        40,
    ]
    for manifest in (v1, t1):
        assert manifest.runtime["cognitive_profile"] == "M0"
        assert manifest.runtime["module2_effect_scope"] == "off"
        assert manifest.runtime["module2_enabled"] is False
        assert manifest.runtime["cognitive_place_graph_enabled"] is False
        assert manifest.runtime["low_obstacles_enabled"] is False
        assert manifest.runtime["dynamic_actors_enabled"] is False
        assert manifest.runtime["ground_truth_use"] == "evaluator_only"
        assert manifest.episode.variant_id == "baseline"
        assert manifest.episode.appearance_profile_id is None


def test_post_run4b_manifests_are_new_read_only_splits_with_strict_routes():
    v2 = load_targeted_teaching_manifest(V2)
    t2 = load_targeted_teaching_manifest(T2)

    assert v2.dataset == {
        "route_id": "V2",
        "role": "validation",
        "split": "validation",
        "status": "raw_until_audit",
        "evaluation_read_only": True,
    }
    assert t2.dataset == {
        "route_id": "T2",
        "role": "read_only_test",
        "split": "test",
        "status": "raw_until_audit",
        "evaluation_read_only": True,
    }
    assert (v2.episode.seed, t2.episode.seed) == (7913, 7912)
    assert v2.paired_appearance["variant_profile_id"] == "dim_warm"
    assert t2.paired_appearance["variant_profile_id"] == "bright_cool"
    assert (v2.mission_legs[0].x, v2.mission_legs[0].y) == pytest.approx(
        (-3.475, -2.475)
    )
    assert [state_id_for_map_xy(leg.x, leg.y) for leg in v2.mission_legs] == [
        84,
        147,
        118,
        167,
        215,
        40,
    ]
    assert [state_id_for_map_xy(leg.x, leg.y) for leg in t2.mission_legs] == [
        202,
        183,
        119,
        85,
        149,
        40,
    ]
    for manifest in (v2, t2):
        states = [state_id_for_map_xy(leg.x, leg.y) for leg in manifest.mission_legs]
        assert len(manifest.mission_legs) == 6  # Five strict legs, then G1 return.
        assert len(set(states)) == 6
        assert manifest.mission_legs[-1].goal_id.endswith("G1_S40")
        assert manifest.best_effort_leg_ids == frozenset()
        assert manifest.runtime["cognitive_profile"] == "M0"
        assert manifest.runtime["module2_enabled"] is False
        assert manifest.runtime["module2_navigation_write_enabled"] is False
        assert manifest.runtime["module2_effect_scope"] == "off"
        assert manifest.runtime["cognitive_place_graph_enabled"] is False
        assert manifest.runtime["low_obstacles_enabled"] is False
        assert manifest.runtime["dynamic_actors_enabled"] is False
        assert manifest.runtime["ground_truth_use"] == "evaluator_only"


def test_post_run4b_routes_have_feasible_unique_canonical_gvg_connectors():
    occupancy = load_occupancy_map(
        REPO / "data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml",
        unknown_is_occupied=True,
    )
    graph = json.loads(
        (
            REPO
            / "ros2_ws/src/robot_route_planner/config/"
            "v6_kujiale_isaacgen_v1_gvg_v1.geojson"
        ).read_text(encoding="utf-8")
    )
    support = {
        int(feature["properties"]["id"]): tuple(
            feature["geometry"]["coordinates"]
        )
        for feature in graph["features"]
        if feature["geometry"]["type"] == "Point"
    }
    adjacency: dict[int, list[int]] = {}
    for feature in graph["features"]:
        if feature["geometry"]["type"] != "MultiLineString":
            continue
        properties = feature["properties"]
        adjacency.setdefault(int(properties["startid"]), []).append(
            int(properties["endid"])
        )
    footprint = {
        "polygon_m": [
            [0.255, 0.21],
            [0.255, -0.21],
            [-0.23, -0.21],
            [-0.23, 0.21],
        ],
        "padding_m": 0.005,
        "padded_inscribed_radius_m": 0.215,
        "sweep_sample_spacing_m": 0.025,
    }
    v2_s84 = load_targeted_teaching_manifest(V2).mission_legs[0]
    v2_s84_pixel = occupancy.world_to_pixel(v2_s84.x, v2_s84.y)
    static_clearance_m = (
        cv2.distanceTransform(
            np.where(occupancy.free, 255, 0).astype(np.uint8),
            cv2.DIST_L2,
            cv2.DIST_MASK_PRECISE,
        )[v2_s84_pixel]
        * occupancy.resolution_m
    )
    assert static_clearance_m == pytest.approx(math.sqrt(0.1), abs=1e-6)
    assert static_clearance_m > footprint["padded_inscribed_radius_m"]
    assert math.dist(
        (v2_s84.x, v2_s84.y),
        (-3.387832736968995, -2.5402230739593485),
    ) < 0.20

    route_pairs: dict[str, tuple[tuple[int, int], ...]] = {}
    for manifest in (
        load_targeted_teaching_manifest(V2),
        load_targeted_teaching_manifest(T2),
    ):
        points = [(0.45, -5.35)] + [
            (leg.x, leg.y) for leg in manifest.mission_legs
        ]
        pairs: list[tuple[int, int]] = []
        for start_xy, goal_xy in zip(points, points[1:]):
            start = select_support_attachment(
                occupancy, support, start_xy, footprint, departing=True
            )
            goal = select_support_attachment(
                occupancy, support, goal_xy, footprint, departing=False
            )
            pairs.append((start, goal))

            for connector in ((start_xy, support[start]), (support[goal], goal_xy)):
                if math.dist(*connector) <= occupancy.resolution_m:
                    continue
                assert classify_edge(
                    occupancy,
                    np.asarray(connector, dtype=np.float64),
                    footprint_polygon_m=np.asarray(
                        footprint["polygon_m"], dtype=np.float64
                    ),
                    footprint_padding_m=footprint["padding_m"],
                    padded_inscribed_radius_m=footprint[
                        "padded_inscribed_radius_m"
                    ],
                    sweep_sample_spacing_m=footprint["sweep_sample_spacing_m"],
                ) == Traversability.FEASIBLE

            reached = {start}
            frontier = [start]
            while frontier and goal not in reached:
                node = frontier.pop(0)
                for following in adjacency.get(node, []):
                    if following not in reached:
                        reached.add(following)
                        frontier.append(following)
            assert goal in reached

        assert len(set(pairs)) == len(pairs)
        route_pairs[manifest.route_id] = tuple(pairs)
    assert route_pairs["V2"] != route_pairs["T2"]


def test_validation_role_cannot_be_admitted_as_training(tmp_path):
    document = yaml.safe_load(V1.read_text(encoding="utf-8"))
    document["dataset"]["role"] = "train"
    with pytest.raises(V6ContractError, match="dataset keys|training dataset"):
        load_targeted_teaching_manifest(_write(tmp_path, document))


def test_paired_capture_audit_requires_identical_nonempty_stamp_multisets():
    assert paired_stamp_summary({10: 1, 20: 2}, {10: 1, 20: 2}) == {
        "baseline_count": 3,
        "variant_count": 3,
        "matched_count": 3,
        "same_stamp": True,
    }
    mismatch = paired_stamp_summary({10: 1, 20: 1}, {10: 1, 21: 1})
    assert mismatch["matched_count"] == 1
    assert mismatch["same_stamp"] is False
    assert paired_stamp_summary({}, {})["same_stamp"] is False


def test_paired_state_requires_baseline_authority_and_frozen_simulation_time():
    for profile in ("dim_cool", "bright_warm", "dim_warm", "bright_cool"):
        state = {
            "schema": "bio_nav_paired_appearance_capture_v1",
            "baseline_profile_id": "baseline",
            "variant_profile_id": profile,
            "simulation_time_advanced_during_capture": False,
        }
        assert paired_state_error(json.dumps(state), profile) == ""
    state["simulation_time_advanced_during_capture"] = True
    assert paired_state_error(json.dumps(state), "bright_cool") == (
        "paired_state_mismatch:simulation_time_advanced_during_capture"
    )


def test_validation_validate_only_reports_read_only_eligibility(capsys):
    assert cli(["--manifest", str(V1), "--validate-only"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["route_id"] == "V1"
    assert payload["training_eligible"] is False
    assert payload["head_eligible"] is False
    assert payload["read_only_eligible"] is True
    assert payload["paired_appearance"]["variant_profile_id"] == "dim_cool"
    assert payload["dispatch"] is False

    assert cli(["--manifest", str(V2), "--validate-only"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["route_id"] == "V2"
    assert payload["training_eligible"] is False
    assert payload["head_eligible"] is False
    assert payload["read_only_eligible"] is True
    assert payload["paired_appearance"]["variant_profile_id"] == "dim_warm"
    assert payload["dispatch"] is False

    assert cli(["--manifest", str(T2), "--validate-only"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["route_id"] == "T2"
    assert payload["training_eligible"] is False
    assert payload["head_eligible"] is False
    assert payload["read_only_eligible"] is True
    assert payload["paired_appearance"]["variant_profile_id"] == "bright_cool"
    assert payload["dispatch"] is False


def test_wrapper_reuses_targeted_runner_and_records_each_validation_route():
    wrapper = WRAPPER.read_text(encoding="utf-8")
    recorder = RECORDER.read_text(encoding="utf-8")

    assert 'run_episode v1 "$@"' in wrapper
    assert 'run_episode t1 "$@"' in wrapper
    assert 'run_episode v2 "$@"' in wrapper
    assert 'run_episode t2 "$@"' in wrapper
    assert "collect-validation-v2" in wrapper
    assert "v2_validation" in wrapper
    assert "t2_read_only_test" in wrapper
    assert 'printf \'%s_read_only_test\\n\'' in wrapper
    assert "run_v6_r5_phase_b_kujiale.sh" in wrapper
    assert "--dispatch" in wrapper
    for topic in (
        PAIRED_BASELINE_TOPIC,
        PAIRED_VARIANT_TOPIC,
        PAIRED_STATE_TOPIC,
    ):
        assert topic in recorder
    subprocess.run(["bash", "-n", str(WRAPPER)], check=True)
    subprocess.run(["bash", "-n", str(RECORDER)], check=True)
