from pathlib import Path

import pytest
import yaml

from isaac_sim.src.experiment.appearance import (
    PROFILE_IDS,
    is_material_color_input,
    load_appearance_profiles,
    rotate_rgb_hue,
)


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "isaac_sim/configs/experiments/kujiale_appearance_profiles.yaml"


def test_fixed_appearance_profile_contract_is_valid_and_hashed():
    profiles = load_appearance_profiles(SOURCE)
    assert tuple(profiles.profiles) == PROFILE_IDS
    assert profiles.require("baseline").light_intensity_scale == 1.0
    assert profiles.require("dim_warm").color_temperature_k == 3000
    assert profiles.require("bright_cool").material_hue_shift_deg == -35.0
    assert len(profiles.sha256) == 64


def test_appearance_parser_rejects_missing_or_changed_baseline(tmp_path):
    document = yaml.safe_load(SOURCE.read_text(encoding="utf-8"))
    del document["profiles"]["dim_cool"]
    target = tmp_path / "profiles.yaml"
    target.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError, match="exactly"):
        load_appearance_profiles(target)

    document = yaml.safe_load(SOURCE.read_text(encoding="utf-8"))
    document["profiles"]["baseline"]["light_intensity_scale"] = 0.9
    target.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError, match="baseline"):
        load_appearance_profiles(target)


def test_hue_rotation_and_supported_material_input_detection_are_deterministic():
    shifted = rotate_rgb_hue((1.0, 0.0, 0.0), 120.0)
    assert shifted[0] == pytest.approx(0.0, abs=1.0e-7)
    assert shifted[1] == pytest.approx(1.0, abs=1.0e-7)
    assert shifted[2] == pytest.approx(0.0, abs=1.0e-7)
    assert is_material_color_input("inputs:base_color")
    assert is_material_color_input("inputs:diffuseColor")
    assert is_material_color_input("inputs:ColorColor_2")
    assert is_material_color_input("inputs:diffuse_color_constant")
    assert not is_material_color_input("inputs:roughness")
    assert not is_material_color_input("outputs:base_color")
