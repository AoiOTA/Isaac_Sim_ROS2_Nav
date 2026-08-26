from pathlib import Path
import subprocess

import yaml


REPO = Path(__file__).resolve().parents[4]
WRAPPER = REPO / "scripts/run_v6_run4_shadow_probe.sh"
CONFIG = (
    REPO
    / "ros2_ws/src/robot_experiments/config/v6_run4_shadow_probe.yaml"
)


def _run(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["bash", str(WRAPPER), *args],
        check=True,
        capture_output=True,
        text=True,
    )


def test_plan_freezes_run4_identity_and_shadow_only_policy():
    result = _run("--dry-run", "plan")
    text = result.stdout

    assert "status=T2_FAIL_KEEP_SHADOW_ONLY" in text
    assert "qualification=NOT_QUALIFIED" in text
    assert "expected_manifest_status=READ_ONLY_CAUSAL_CANDIDATE_STARTUP_ONLY" in text
    assert "expected_model_id=kujiale_0026_visual_heads_run4_v310" in text
    assert "80f0b104c68899f1865a4369f091a16631ed8f178b895c84b8d072cbe10a7821" in text
    assert "72d5ba175b97ae12d55881d0f7ad73e025b5971d23bae9857c6da76fc532d1da" in text
    assert "startup_profile=estimated_shadow" in text
    assert "localization_supervisor_mode=shadow" in text
    assert "active_effect_scope=none" in text
    assert "navigation_dispatch=false" in text
    assert "expected_initialpose_writes=0" in text


def test_server_and_bridge_dry_run_pass_the_same_candidate_manifest(tmp_path):
    integration = tmp_path / "integration"
    module2 = tmp_path / "module2"
    manifest = (
        integration
        / "ros2_ws/src/bio_nav_ros_bridge/config/kujiale_0026_run4_read_only_shadow_candidate.json"
    )
    manifest.parent.mkdir(parents=True)
    manifest.write_text("{}\n", encoding="utf-8")
    server = integration / "scripts/run_module2_v310_server.sh"
    server.parent.mkdir(parents=True)
    server.write_text("#!/usr/bin/env bash\n", encoding="utf-8")

    common = (
        "--dry-run",
        "--integration-root",
        str(integration),
        "--module2-root",
        str(module2),
    )
    server_result = _run(*common, "server", "--extra-server-arg")
    bridge_result = _run(*common, "bridge", "extra_bridge_arg:=value")

    assert str(server) in server_result.stdout
    assert f"--candidate-manifest {manifest}" in server_result.stdout
    assert "--extra-server-arg" in server_result.stdout
    assert "startup_profile:=estimated_shadow" in bridge_result.stdout
    assert "localization_supervisor_mode:=shadow" in bridge_result.stdout
    assert f"localization_candidate_manifest:={manifest}" in bridge_result.stdout
    assert "extra_bridge_arg:=value" in bridge_result.stdout


def test_bounded_recorder_contains_required_diagnostics_and_no_goal_topic(tmp_path):
    result = _run(
        "--dry-run",
        "--run-root",
        str(tmp_path / "probe"),
        "--duration",
        "17",
        "record",
    )
    text = result.stdout

    assert "timeout --signal=INT --kill-after=5s 17s" in text
    for topic in (
        "/bio_nav/module2/planning_prior",
        "/bio_nav/localization/candidates",
        "/diagnostics",
        "/initialpose",
        "/cmd_vel_sim",
    ):
        assert topic in text
    assert "/goal_pose" not in text
    assert "runner" not in text


def test_config_matches_read_only_candidate_interface():
    value = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))

    assert value["status"] == "T2_FAIL_KEEP_SHADOW_ONLY"
    assert value["qualification"] == "NOT_QUALIFIED"
    assert value["runtime_policy"] == {
        "startup_profile": "estimated_shadow",
        "localization_supervisor_mode": "shadow",
        "active_effect_scope": "none",
        "navigation_dispatch": False,
        "isaac_launch": False,
        "module3_ros_nav_launch": False,
        "expected_initialpose_writes": 0,
    }
    assert value["integration_interface"] == {
        "candidate_manifest": "ros2_ws/src/bio_nav_ros_bridge/config/kujiale_0026_run4_read_only_shadow_candidate.json",
        "server_argument": "--candidate-manifest",
        "bridge_launch_argument": "localization_candidate_manifest",
    }


def test_wrapper_has_valid_bash_syntax_and_no_active_arm_entrypoint():
    subprocess.run(["bash", "-n", str(WRAPPER)], check=True)
    source = WRAPPER.read_text(encoding="utf-8")

    assert "run_v6_localization_causal.sh" not in source
    assert "run_v6_r5_phase_b_kujiale.sh" not in source
    assert "localization_supervisor_mode:=startup" not in source
    assert "localization_supervisor_mode:=active" not in source
    assert "run_isaac.sh" not in source
    assert "run_ros.sh" not in source
    assert "goal runner" in source
