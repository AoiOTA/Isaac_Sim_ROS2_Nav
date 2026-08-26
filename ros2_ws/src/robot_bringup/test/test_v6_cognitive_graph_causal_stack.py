import subprocess
from pathlib import Path

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
STACK = REPOSITORY_ROOT / "scripts/run_v6_cognitive_graph_causal_stack.sh"


def _dry_run(tmp_path: Path, arm: str, *, obstacle_arm: str = "M3"):
    result = subprocess.run(
        [
            str(STACK),
            "--arm", arm,
            "--domain", "151",
            "--run-dir", str(tmp_path / "run"),
            "--socket", str(tmp_path / "module2.sock"),
            "--module2-root", str(tmp_path / "module2"),
            "--obstacle-arm", obstacle_arm,
            "--dry-run",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return dict(
        line.split("=", 1)
        for line in result.stdout.splitlines()
        if "=" in line and not line.startswith(("module3:", "module2:", "bridge:"))
    ), result.stdout


@pytest.mark.parametrize(
    ("arm", "mode", "route_prior", "profile", "scope"),
    (
        ("G0", "gvg", "false", "module2_causal_obstacle_active", "obstacle_only"),
        (
            "G1", "shadow", "false", "cognitive_graph_causal_shadow",
            "obstacle_and_graph",
        ),
        ("G2", "hybrid", "true", "cognitive_graph_causal_hybrid", "all"),
        ("G3", "primary", "true", "cognitive_graph_causal_primary", "all"),
    ),
)
def test_graph_arms_have_one_exact_stack_contract(
    tmp_path, arm, mode, route_prior, profile, scope
):
    values, argv = _dry_run(tmp_path, arm)
    assert values == {
        "arm": arm,
        "graph_mode": mode,
        "route_prior_enabled": route_prior,
        "obstacle_arm": "M3",
        "startup_profile": profile,
        "active_effect_scope": scope,
        "localization_supervisor_mode": "shadow",
    }
    assert f"cognitive_graph_mode:={mode}" in argv
    assert f"route_prior_enabled:={route_prior}" in argv
    assert f"--startup-profile {profile}" in argv
    assert f"--active-effect-scope {scope}" in argv
    assert f"--cognitive-graph-mode {mode}" in argv
    assert "stop-producer" not in argv
    assert "--shadow-config" not in argv
    assert "--candidate-manifest" in argv
    assert "kujiale_0026_run4_read_only_shadow_candidate.json" in argv
    assert "localization_candidate_manifest:=" in argv
    assert "localization_supervisor_mode:=shadow" in argv


def test_obstacle_arm_is_held_m3_or_whole_group_m2_fallback(tmp_path):
    _values, m3 = _dry_run(tmp_path, "G3")
    values, m2 = _dry_run(tmp_path, "G3", obstacle_arm="M2")
    assert "V6_COGNITIVE_PROFILE=M3" in m3
    assert "V6_COGNITIVE_PROFILE=M2" in m2
    assert " ros-d primary route_prior_enabled:=true" in m3
    assert " ros-d primary route_prior_enabled:=true" in m2
    assert " primary M3 " not in m3
    assert " primary M2 " not in m2
    assert values["obstacle_arm"] == "M2"


def test_graph_stack_rejects_partial_or_out_of_contract_arms(tmp_path):
    result = subprocess.run(
        [
            str(STACK),
            "--arm", "G4",
            "--domain", "151",
            "--run-dir", str(tmp_path / "run"),
            "--socket", str(tmp_path / "module2.sock"),
            "--dry-run",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 2

    result = subprocess.run(
        [
            str(STACK),
            "--arm", "G0",
            "--domain", "151",
            "--run-dir", str(tmp_path / "run"),
            "--socket", str(tmp_path / "module2.sock"),
            "--obstacle-arm", "M1",
            "--dry-run",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 2
    assert "M3 or M2" in result.stderr

    result = subprocess.run(
        [
            str(STACK),
            "--arm", "G0",
            "--domain", "151",
            "--run-dir", str(tmp_path / "run"),
            "--socket", str(tmp_path / "module2.sock"),
            "--localization-supervisor-mode", "active",
            "--dry-run",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 2
    assert "shadow or startup" in result.stderr


def test_new_axis_does_not_change_phase_f_or_legacy_defaults():
    phase_f = (
        REPOSITORY_ROOT / "scripts/run_v6_low_obstacle_phase_f_stack.sh"
    ).read_text(encoding="utf-8")
    core = (
        REPOSITORY_ROOT
        / "ros2_ws/src/robot_bringup/launch/ros_stack.launch.py"
    ).read_text(encoding="utf-8")
    assert "route_prior_enabled" not in phase_f
    assert "'route_prior_enabled', default_value='auto'" in core
    assert "resolve_route_prior_enabled(" in core
