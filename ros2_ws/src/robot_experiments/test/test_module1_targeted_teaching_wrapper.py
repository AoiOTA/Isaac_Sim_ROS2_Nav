from pathlib import Path
import os
import shutil
import subprocess
import textwrap

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


def test_episode_stops_sigint_ignoring_rosbag_with_term_and_writes_metadata(
    tmp_path,
):
    project = tmp_path / "project"
    scripts = project / "scripts"
    config = project / "ros2_ws/src/robot_experiments/config"
    install = project / "ros2_ws/install"
    fake_bin = tmp_path / "bin"
    for directory in (scripts / "lib", config, install, fake_bin):
        directory.mkdir(parents=True, exist_ok=True)

    for source, target in (
        (WRAPPER, scripts / WRAPPER.name),
        (RECORDER, scripts / RECORDER.name),
        (REPO / "scripts/lib/common.sh", scripts / "lib/common.sh"),
        (
            REPO
            / "ros2_ws/src/robot_experiments/config/"
            "module1_targeted_teaching_kujiale_en.yaml",
            config / "module1_targeted_teaching_kujiale_en.yaml",
        ),
        (QOS, config / QOS.name),
    ):
        shutil.copy2(source, target)

    setup = tmp_path / "ros_setup.bash"
    setup.write_text("export ROS_DISTRO=jazzy\n", encoding="utf-8")
    for name in ("setup.bash", "local_setup.bash"):
        (install / name).write_text("export ROS_DISTRO=jazzy\n", encoding="utf-8")
    fastdds = tmp_path / "fastdds.xml"
    fastdds.write_text("<profiles/>\n", encoding="utf-8")
    trace = tmp_path / "rosbag_signal.txt"

    fake_ros2 = fake_bin / "ros2"
    fake_ros2.write_text(
        textwrap.dedent(
            """\
            #!/usr/bin/env python3
            import os
            from pathlib import Path
            import signal
            import sys
            import time

            argv = sys.argv[1:]
            if argv[:2] == ["pkg", "prefix"]:
                root = Path(os.environ["FAKE_INTEGRATION_INSTALL"])
                print(root / argv[2])
                raise SystemExit(0)
            if argv[:2] == ["run", "robot_experiments"]:
                raise SystemExit(0)
            if argv[:2] != ["bag", "record"]:
                raise SystemExit(f"unexpected ros2 invocation: {argv}")

            output = Path(argv[argv.index("--output") + 1])
            output.mkdir(parents=True)
            signal.signal(signal.SIGINT, signal.SIG_IGN)

            def stop(_signum, _frame):
                (output / "metadata.yaml").write_text("recording: stopped\\n")
                Path(os.environ["FAKE_ROSBAG_SIGNAL_TRACE"]).write_text("SIGTERM\\n")
                raise SystemExit(0)

            signal.signal(signal.SIGTERM, stop)
            while True:
                time.sleep(1)
            """
        ),
        encoding="utf-8",
    )
    fake_ros2.chmod(0o755)

    integration_install = Path(
        "/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/"
        "bio_nav_integration/ros2_ws/install"
    )
    run_root = tmp_path / "run"
    env = os.environ.copy()
    env.update(
        {
            "PATH": f"{fake_bin}:{env['PATH']}",
            "ROS_SETUP": str(setup),
            "ISAAC_NAV_FASTDDS_PROFILE": str(fastdds),
            "FAKE_INTEGRATION_INSTALL": str(integration_install),
            "FAKE_ROSBAG_SIGNAL_TRACE": str(trace),
        }
    )
    env.pop("RMW_IMPLEMENTATION", None)
    result = subprocess.run(
        [
            "bash",
            str(scripts / WRAPPER.name),
            "--run-root",
            str(run_root),
            "--domain",
            "150",
            "episode",
            "en",
        ],
        env=env,
        capture_output=True,
        text=True,
        timeout=10,
    )

    assert result.returncode == 0, result.stderr
    assert trace.read_text(encoding="utf-8") == "SIGTERM\n"
    assert (run_root / "raw_mcap/en_A_base/metadata.yaml").is_file()
    source = WRAPPER.read_text(encoding="utf-8")
    assert 'kill -TERM "${recorder_pid}"' in source
    assert 'kill -KILL "${recorder_pid}"' in source
    assert 'kill -INT "${recorder_pid}"' not in source
