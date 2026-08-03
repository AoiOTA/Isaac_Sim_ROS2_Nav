from pathlib import Path

import pytest
import yaml

from robot_bringup.mode_contract import validate_nav2_profile_params_file


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NAVIGATION_ROOT = PACKAGE_ROOT.parent / "robot_navigation"


def _write_profile(path, *, frequency=10.0, model_dt=0.1,
                   time_steps=20, batch_size=750):
    path.write_text(
        yaml.safe_dump({
            "controller_server": {
                "ros__parameters": {
                    "controller_frequency": frequency,
                    "FollowPath": {
                        "model_dt": model_dt,
                        "time_steps": time_steps,
                        "batch_size": batch_size,
                    },
                },
            },
        }),
        encoding="utf-8",
    )
    return path


@pytest.mark.parametrize(
    "profile_name",
    [
        "stable",
        "performance",
        "dynamic_avoidance",
        "bio_nav_planning_only",
        "bio_nav_risk_only",
        "bio_nav_tiebreak_risk",
        "attempt21_static_collection",
        "bio_nav_rgbd_risk_shadow",
    ],
)
def test_shipped_nav2_profiles_satisfy_mppi_timing_contract(profile_name):
    profile = validate_nav2_profile_params_file(
        NAVIGATION_ROOT / "config" / f"nav2_{profile_name}.yaml"
    )

    assert profile.controller_period <= profile.model_dt
    assert profile.time_steps > 0
    assert profile.batch_size > 0


def test_eight_hz_with_point_one_model_dt_fails_before_node_start(tmp_path):
    path = _write_profile(tmp_path / "invalid.yaml", frequency=8.0, model_dt=0.1)

    with pytest.raises(
        ValueError,
        match=(
            r"Nav2 1\.3\.12 MPPI timing constraint violated: .*"
            r"0\.125000s exceeds .*0\.100000s.*at least 10Hz"
        ),
    ):
        validate_nav2_profile_params_file(path)


def test_boundary_frequency_equal_to_model_step_is_valid(tmp_path):
    profile = validate_nav2_profile_params_file(
        _write_profile(tmp_path / "boundary.yaml", frequency=10, model_dt=0.1)
    )

    assert profile.controller_period == pytest.approx(profile.model_dt)


@pytest.mark.parametrize(
    ("field", "value", "expected"),
    [
        ("frequency", 0.0, "controller_frequency must be a finite positive"),
        ("frequency", float("nan"), "controller_frequency must be a finite positive"),
        ("model_dt", float("inf"), "model_dt must be a finite positive"),
        ("time_steps", 20.0, "time_steps must be a positive integer"),
        ("time_steps", True, "time_steps must be a positive integer"),
        ("batch_size", 0, "batch_size must be a positive integer"),
    ],
)
def test_profile_scalars_are_strict_positive_and_finite(
    tmp_path, field, value, expected
):
    values = {
        "frequency": 10.0,
        "model_dt": 0.1,
        "time_steps": 20,
        "batch_size": 750,
    }
    values[field] = value
    path = _write_profile(tmp_path / "invalid.yaml", **values)

    with pytest.raises(ValueError, match=expected):
        validate_nav2_profile_params_file(path)


@pytest.mark.parametrize(
    ("document", "expected"),
    [
        ([], "nav2 profile root must be a YAML mapping"),
        ({}, "controller_server must be a YAML mapping"),
        ({"controller_server": []}, "controller_server must be a YAML mapping"),
        (
            {"controller_server": {"ros__parameters": []}},
            "controller_server.ros__parameters must be a YAML mapping",
        ),
        (
            {"controller_server": {"ros__parameters": {}}},
            "controller_server.ros__parameters.FollowPath must be a YAML mapping",
        ),
    ],
)
def test_profile_yaml_structure_is_strict(tmp_path, document, expected):
    path = tmp_path / "invalid.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")

    with pytest.raises(ValueError, match=expected):
        validate_nav2_profile_params_file(path)


def test_launch_validates_profile_before_constructing_node_actions():
    source = (
        PACKAGE_ROOT / "launch" / "ros_stack.launch.py"
    ).read_text(encoding="utf-8")

    validation = source.index("validate_nav2_profile_params_file(")
    first_action = source.index("actions = [LogInfo", validation)
    first_node = source.index("actions.append(Node(", first_action)
    assert validation < first_action < first_node


def test_bio_nav_planner_helper_does_not_inherit_planner_server_name():
    source = (
        PACKAGE_ROOT.parent
        / "bio_nav_fusion"
        / "src"
        / "bio_nav_grid_based.cpp"
    ).read_text(encoding="utf-8")

    assert "rclcpp::NodeOptions().use_global_arguments(false)" in source
    assert '"bio_nav_goal_prior_client"' in source
    assert "clock_ = node->get_clock();" in source
    assert "client_node_->get_clock()" not in source


def test_cognitive_risk_layer_latches_bridge_absolute_reset_epoch():
    source = (
        PACKAGE_ROOT.parent
        / "bio_nav_fusion"
        / "src"
        / "cognitive_risk_layer.cpp"
    ).read_text(encoding="utf-8")

    assert "if (!reset_epoch_initialized_)" in source
    assert "reset_epoch_ = message->reset_epoch;" in source
    assert '"/simulation/reset_event"' in (
        PACKAGE_ROOT.parent
        / "bio_nav_fusion"
        / "include"
        / "bio_nav_fusion"
        / "cognitive_risk_layer.hpp"
    ).read_text(encoding="utf-8")
