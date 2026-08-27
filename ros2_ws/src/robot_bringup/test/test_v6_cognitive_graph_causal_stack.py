import os
import socket
import subprocess
from pathlib import Path

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
STACK = REPOSITORY_ROOT / "scripts/run_v6_cognitive_graph_causal_stack.sh"


def _dry_run(
    tmp_path: Path,
    arm: str,
    *,
    obstacle_arm: str = "M3",
    graph_only_no_box: bool = False,
):
    command = [
        str(STACK),
        "--arm", arm,
        "--domain", "151",
        "--run-dir", str(tmp_path / "run"),
        "--socket", str(tmp_path / "module2.sock"),
        "--module2-root", str(tmp_path / "module2"),
        "--obstacle-arm", obstacle_arm,
    ]
    if graph_only_no_box:
        command.append("--graph-only-no-box")
    command.append("--dry-run")
    result = subprocess.run(
        command,
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


@pytest.mark.parametrize(
    ("arm", "mode"),
    (("G1", "shadow"), ("G2", "hybrid"), ("G3", "primary")),
)
def test_graph_only_no_box_keeps_graph_modes_but_disables_safety_and_prior(
    tmp_path, arm, mode
):
    values, argv = _dry_run(
        tmp_path, arm, graph_only_no_box=True
    )
    assert values["experiment_scope"] == "graph_only_no_box"
    assert values["no_box"] == "true"
    assert values["graph_mode"] == mode
    assert values["cognitive_profile"] == "M0"
    assert values["route_prior_enabled"] == "false"
    assert values["integration_graph_mode"] == "shadow"
    assert values["m3_safety_status"] == "DEFERRED"
    assert values["route_prior_status"] == "DEFERRED"
    assert "V6_COGNITIVE_PROFILE=M0" in argv
    assert f" ros-d {mode} route_prior_enabled:=false" in argv
    assert "--cognitive-graph-mode shadow" in argv
    assert "cognitive_graph_mode:=shadow" in argv


def test_graph_only_no_box_rejects_g0(tmp_path):
    result = subprocess.run(
        [
            str(STACK),
            "--arm", "G0",
            "--domain", "151",
            "--run-dir", str(tmp_path / "run"),
            "--socket", str(tmp_path / "module2.sock"),
            "--graph-only-no-box",
            "--dry-run",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 2
    assert "requires G1, G2, or G3" in result.stderr


def test_nondefault_domain_reaches_active_socket_check_without_readonly_error(
    tmp_path,
):
    integration_root = tmp_path / "integration"
    module2_root = tmp_path / "module2"
    (integration_root / "scripts").mkdir(parents=True)
    (integration_root / "scripts/run_v6_module2_graph_causal_server.sh").touch()
    candidate_manifest = (
        integration_root
        / "ros2_ws/src/bio_nav_ros_bridge/config"
        / "kujiale_0026_run4_read_only_shadow_candidate.json"
    )
    candidate_manifest.parent.mkdir(parents=True)
    candidate_manifest.touch()
    (module2_root / "configs").mkdir(parents=True)
    (
        module2_root / "configs/kujiale_0026_module1_visual_shadow_v310.yaml"
    ).touch()

    socket_path = tmp_path / "active.sock"
    env = os.environ.copy()
    env["BIO_NAV_INTEGRATION_ROOT"] = str(integration_root)
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as active_socket:
        active_socket.bind(str(socket_path))
        active_socket.listen()
        result = subprocess.run(
            [
                str(STACK),
                "--arm", "G0",
                "--domain", "151",
                "--run-dir", str(tmp_path / "run"),
                "--socket", str(socket_path),
                "--module2-root", str(module2_root),
            ],
            check=False,
            capture_output=True,
            text=True,
            env=env,
        )

    assert result.returncode == 1
    assert "refusing to replace active Module2 socket" in result.stderr
    assert "readonly variable" not in result.stderr


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


def test_phase_f_disables_route_prior_without_changing_legacy_default():
    phase_f = (
        REPOSITORY_ROOT / "scripts/run_v6_low_obstacle_phase_f_stack.sh"
    ).read_text(encoding="utf-8")
    core = (
        REPOSITORY_ROOT
        / "ros2_ws/src/robot_bringup/launch/ros_stack.launch.py"
    ).read_text(encoding="utf-8")
    assert "route_prior_enabled:=false" in phase_f
    assert "'route_prior_enabled', default_value='auto'" in core
    assert "resolve_route_prior_enabled(" in core
