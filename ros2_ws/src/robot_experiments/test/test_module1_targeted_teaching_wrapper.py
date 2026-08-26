from pathlib import Path
import subprocess

import yaml


REPO = Path(__file__).resolve().parents[4]
WRAPPER = REPO / "scripts/run_module1_targeted_teaching_kujiale.sh"
RECORDER = REPO / "scripts/record_module1_kujiale_scene.sh"
QOS = (
    REPO
    / "ros2_ws/src/robot_experiments/config/"
    "module1_targeted_teaching_rosbag_qos.yaml"
)


def test_wrapper_reuses_phase_b_stack_and_dispatches_separate_route_bags():
    source = WRAPPER.read_text(encoding="utf-8")

    assert "run_v6_r5_phase_b_kujiale.sh" in source
    assert '"${PHASE_B_WRAPPER}"' in source
    assert "record_module1_kujiale_scene.sh" in source
    assert 'run_episode en "$@"' in source
    assert 'run_episode sw "$@"' in source
    assert 'printf \'%s_A_base\\n\'' in source
    assert '--manifest "${manifest}"' in source
    assert "--dispatch" in source
    assert "--output-jsonl" in source
    assert "cleanup_recorder" in source


def test_wrapper_and_recorder_pin_effect_off_raw_capture_topics():
    wrapper = WRAPPER.read_text(encoding="utf-8")
    recorder = RECORDER.read_text(encoding="utf-8")
    phase_b = (REPO / "scripts/run_v6_r5_phase_b_kujiale.sh").read_text(
        encoding="utf-8"
    )

    assert "--mode mixed" in phase_b
    assert "--no-dynamic-obstacles" in phase_b
    assert "cognitive_profile:=M0" in phase_b
    assert "module2_enabled:=false" in phase_b
    assert "Module2/CPG/dynamic effects off" in wrapper
    for topic in (
        "/clock",
        "/camera/front/image_raw",
        "/camera/front/camera_info",
        "/ground_truth/odom",
        "/bio_nav/module1/odom",
        "/cmd_vel",
        "/cmd_vel_sim",
        "/simulation/reset_event",
        "/simulation/collision",
        "/scan",
    ):
        assert topic in recorder
    assert "/bio_nav/module2" not in recorder
    assert "planning_prior" not in recorder.lower()


def test_recorder_uses_explicit_best_effort_sensor_qos():
    recorder = RECORDER.read_text(encoding="utf-8")
    document = yaml.safe_load(QOS.read_text(encoding="utf-8"))

    assert "--qos-profile-overrides-path" in recorder
    for topic in (
        "/camera/front/image_raw",
        "/camera/front/camera_info",
        "/scan",
    ):
        assert document[topic]["reliability"] == "best_effort"
        assert document[topic]["durability"] == "volatile"


def test_targeted_wrapper_help_and_bash_syntax():
    result = subprocess.run(
        ["bash", str(WRAPPER), "--help"],
        check=True,
        capture_output=True,
        text=True,
    )
    assert "one MCAP bag" in result.stdout
    assert "one exactly-once reset per episode" in result.stdout
    assert "GT" in result.stdout and "never a goal input" in result.stdout
    subprocess.run(["bash", "-n", str(WRAPPER)], check=True)
    subprocess.run(["bash", "-n", str(RECORDER)], check=True)
