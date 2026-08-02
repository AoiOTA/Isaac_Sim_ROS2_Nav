from importlib.util import module_from_spec, spec_from_file_location
import json
from pathlib import Path
import subprocess
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[2]
SCRIPT = WORKSPACE_ROOT / "scripts" / "generate_attempt21_rgbd_risk_profile.py"
STABLE = (
    PACKAGE_ROOT.parent / "robot_navigation" / "config" / "nav2_stable.yaml"
)
STATIC_BASELINE = (
    PACKAGE_ROOT.parent
    / "robot_navigation"
    / "config"
    / "nav2_attempt21_static_collection.yaml"
)
SPEC = spec_from_file_location("attempt21_profile", SCRIPT)
MODULE = module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def layer(profile):
    return profile["global_costmap"]["global_costmap"]["ros__parameters"]


def test_shadow_profile_is_nonwriting_and_identity_bound():
    profile = MODULE.build_profile(
        STABLE,
        risk_model_sha256="1" * 64,
        risk_qualification_sha256="2" * 64,
        shadow_only=True,
    )
    parameters = layer(profile)
    assert parameters["plugins"] == [
        "static_layer",
        "obstacle_layer",
        "local_rgbd_risk_layer",
        "inflation_layer",
    ]
    risk = parameters["local_rgbd_risk_layer"]
    assert risk["shadow_only"] is True
    assert risk["expected_model_sha256"] == "1" * 64
    assert risk["expected_qualification_sha256"] == "2" * 64
    assert risk["maximum_cost"] == 80
    assert risk["minimum_projection_range_m"] == 0.0


def test_controlled_ab_changes_only_global_overlay_and_stays_nonlethal():
    import yaml

    stable = yaml.safe_load(STABLE.read_text(encoding="utf-8"))
    profile = MODULE.build_profile(
        STABLE,
        risk_model_sha256="3" * 64,
        risk_qualification_sha256="4" * 64,
        shadow_only=False,
        minimum_projection_range_m=1.0,
        maximum_cost=20,
    )
    risk = layer(profile)["local_rgbd_risk_layer"]
    assert risk["shadow_only"] is False
    assert risk["maximum_cost"] == 20
    assert risk["minimum_projection_range_m"] == 1.0
    assert profile["local_costmap"] == stable["local_costmap"]
    assert profile["controller_server"] == stable["controller_server"]


def test_profile_rejects_unbound_identity():
    try:
        MODULE.build_profile(
            STABLE,
            risk_model_sha256="invalid",
            risk_qualification_sha256="4" * 64,
            shadow_only=True,
        )
    except ValueError as error:
        assert "risk_model_sha256" in str(error)
    else:
        raise AssertionError("profile accepted an invalid model identity")


def test_profile_rejects_out_of_bounds_projection_range():
    try:
        MODULE.build_profile(
            STABLE,
            risk_model_sha256="1" * 64,
            risk_qualification_sha256="2" * 64,
            shadow_only=False,
            minimum_projection_range_m=8.1,
        )
    except ValueError as error:
        assert "minimum_projection_range_m" in str(error)
    else:
        raise AssertionError("profile accepted an out-of-bounds projection range")


def test_profile_rejects_lethal_or_zero_cost_ceiling():
    for value in (0, 81):
        try:
            MODULE.build_profile(
                STABLE,
                risk_model_sha256="1" * 64,
                risk_qualification_sha256="2" * 64,
                shadow_only=False,
                maximum_cost=value,
            )
        except ValueError as error:
            assert "maximum_cost" in str(error)
        else:
            raise AssertionError("profile accepted an unsafe cost ceiling")


def test_static_opt_in_cli_requires_and_binds_final_delivery_sha(tmp_path):
    output = tmp_path / "static-opt-in.yaml"
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--risk-model-sha256",
            "1" * 64,
            "--risk-qualification-sha256",
            "2" * 64,
            "--static-opt-in",
            "--authorization-sha256",
            "3" * 64,
            "--output",
            str(output),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    receipt = json.loads(
        output.with_suffix(".yaml.receipt.json").read_text(encoding="utf-8")
    )
    assert receipt["qualification_scope"] == "static_hazard_opt_in_active"
    assert receipt["static_opt_in_costmap_write_enabled"] is True
    assert receipt["static_delivery_sha256"] == "3" * 64
    assert receipt["general_active_fusion_authorized"] is False
    assert receipt["base_profile"] == str(STATIC_BASELINE.resolve())
    profile = __import__("yaml").safe_load(output.read_text(encoding="utf-8"))
    baseline = __import__("yaml").safe_load(
        STATIC_BASELINE.read_text(encoding="utf-8")
    )
    assert profile["bt_navigator"] == baseline["bt_navigator"]
    assert profile["collision_monitor"] == baseline["collision_monitor"]


def test_controlled_static_cli_defaults_to_complete_static_baseline(tmp_path):
    output = tmp_path / "controlled.yaml"
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--risk-model-sha256",
            "1" * 64,
            "--risk-qualification-sha256",
            "2" * 64,
            "--controlled-static-ab",
            "--authorization-sha256",
            "3" * 64,
            "--output",
            str(output),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    profile = __import__("yaml").safe_load(output.read_text(encoding="utf-8"))
    baseline = __import__("yaml").safe_load(
        STATIC_BASELINE.read_text(encoding="utf-8")
    )
    assert profile["bt_navigator"] == baseline["bt_navigator"]
    assert profile["collision_monitor"] == baseline["collision_monitor"]
    assert profile["controller_server"] == baseline["controller_server"]
    assert profile["local_costmap"] == baseline["local_costmap"]
