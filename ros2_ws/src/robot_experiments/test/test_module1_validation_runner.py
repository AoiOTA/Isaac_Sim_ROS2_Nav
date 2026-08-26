import json
from pathlib import Path
import subprocess

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


PACKAGE = Path(__file__).resolve().parents[1]
REPO = Path(__file__).resolve().parents[4]
CONFIG = PACKAGE / "config"
V1 = CONFIG / "module1_targeted_teaching_kujiale_v1.yaml"
T1 = CONFIG / "module1_targeted_teaching_kujiale_t1.yaml"
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
    state = {
        "schema": "bio_nav_paired_appearance_capture_v1",
        "baseline_profile_id": "baseline",
        "variant_profile_id": "dim_cool",
        "simulation_time_advanced_during_capture": False,
    }
    assert paired_state_error(json.dumps(state), "dim_cool") == ""
    state["simulation_time_advanced_during_capture"] = True
    assert paired_state_error(json.dumps(state), "dim_cool") == (
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


def test_wrapper_reuses_targeted_runner_and_records_each_validation_route():
    wrapper = WRAPPER.read_text(encoding="utf-8")
    recorder = RECORDER.read_text(encoding="utf-8")

    assert 'run_episode v1 "$@"' in wrapper
    assert 'run_episode t1 "$@"' in wrapper
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
