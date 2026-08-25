import os
from pathlib import Path
import subprocess


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
RECORDER = REPOSITORY_ROOT / 'scripts' / 'record_module1_kujiale_scene.sh'
RUN_TELEOP = REPOSITORY_ROOT / 'scripts' / 'run_teleop.sh'

REQUIRED_TOPICS = {
    '/clock',
    '/camera/front/image_raw',
    '/camera/front/camera_info',
    '/joint_states',
    '/wheel/odom',
    '/imu/data_raw',
    '/imu/data',
    '/bio_nav/module1/odom',
    '/odom',
    '/amcl_pose',
    '/ground_truth/odom',
    '/map',
    '/scan',
    '/tf',
    '/tf_static',
    '/cmd_vel',
    '/cmd_vel_sim',
    '/initialpose',
    '/simulation/reset_event',
    '/simulation/localization_seeded',
    '/simulation/reset_stop_gate/status',
    '/simulation/collision',
}


def test_recorder_help_documents_episode_and_default_root(tmp_path):
    result = subprocess.run(
        [str(RECORDER), '--help'],
        cwd=tmp_path,
        text=True,
        capture_output=True,
        check=False,
    )

    assert os.access(RECORDER, os.X_OK)
    assert result.returncode == 0, result.stderr
    assert '--root PATH' in result.stdout
    assert '--episode ID' in result.stdout
    assert 'module1_kujiale_scene_registration_20260825/raw_mcap' \
        in result.stdout


def test_recorder_uses_mcap_sim_time_fast_compression_and_exact_topics():
    source = RECORDER.read_text(encoding='utf-8')

    for argument in (
        '--use-sim-time',
        '--storage mcap',
        '--storage-preset-profile zstd_fast',
    ):
        assert argument in source
    assert REQUIRED_TOPICS == {
        line.strip()
        for line in source.splitlines()
        if line.strip().startswith('/')
    }


def test_recorder_excludes_depth_module2_and_cognitive_graph_topics():
    source = RECORDER.read_text(encoding='utf-8').lower()

    assert '/camera/front/depth' not in source
    assert '/bio_nav/module2' not in source
    assert 'cognitiveplacegraph' not in source
    assert 'cognitive_place_graph' not in source


def test_recorder_refuses_existing_episode_before_ros_setup(tmp_path):
    episode = 'already-exists'
    (tmp_path / episode).mkdir()

    result = subprocess.run(
        [str(RECORDER), '--root', str(tmp_path), '--episode', episode],
        stdin=subprocess.DEVNULL,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode != 0
    assert 'refusing to overwrite existing episode' in result.stderr


def test_collection_mode_is_explicit_and_navigation_checks_remain():
    source = RUN_TELEOP.read_text(encoding='utf-8')

    assert '--localization-collection' in source
    for node in ('controller_server', 'planner_server', 'collision_monitor'):
        assert node in source
    assert 'simulation-only localization collection teleop' in source
