"""Package, configuration, and terminal-adapter contract tests."""

from pathlib import Path

from robot_teleop.keyboard_teleop import decode_keypresses
from robot_teleop.keyboard_teleop import format_speed_feedback
from robot_teleop.safety import SpeedFeedback
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]


def test_terminal_arrow_sequences_and_control_keys_are_decoded():
    """Raw terminal escape sequences retain all required controls."""
    assert decode_keypresses(b'w\x1b[A\x1b[B\x1b[C\x1b[D\x03\x04') == [
        'w', 'up', 'down', 'right', 'left', '\x03', '\x04']


def test_terminal_decoder_preserves_every_runtime_speed_key():
    assert decode_keypresses(b'+=-][.,0hH?') == list('+=-][.,0hH?')


def test_safe_defaults_and_package_install_contract():
    """Installed YAML defaults stay inside project safety limits."""
    document = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'teleop.yaml').read_text())
    params = document['keyboard_teleop']['ros__parameters']

    assert params['cmd_vel_topic'] == '/cmd_vel'
    assert params['command_timeout_sec'] <= 0.20
    assert params['publish_rate_hz'] >= 10.0
    assert params['linear_speed'] == 0.50
    assert params['angular_speed'] == 0.80
    assert params['min_linear_speed'] == 0.10
    assert params['min_angular_speed'] == 0.20
    assert params['max_linear_speed'] == 1.00
    assert params['max_angular_speed'] == 1.50
    assert params['linear_speed_step'] == 0.05
    assert params['angular_speed_step'] == 0.10

    setup_source = (PACKAGE_ROOT / 'setup.py').read_text()
    assert "glob('config/*.yaml')" in setup_source
    assert 'keyboard_teleop = robot_teleop.keyboard_teleop:main' \
        in setup_source


def test_speed_feedback_mentions_targets_and_reached_limits():
    message = format_speed_feedback(SpeedFeedback(
        kind='speed',
        linear_speed=1.0,
        angular_speed=1.5,
        limits=('linear maximum', 'angular maximum'),
    ))
    assert 'linear=1.00 m/s' in message
    assert 'angular=1.50 rad/s' in message
    assert 'limit reached' in message


def test_node_uses_wall_monotonic_time_and_finally_stops():
    """The ROS adapter is independent of `/clock` and always closes safely."""
    source = (
        PACKAGE_ROOT / 'robot_teleop' / 'keyboard_teleop.py').read_text()

    assert 'time.monotonic()' in source
    assert 'node.runtime.close()' in source
    assert "topic != '/cmd_vel'" in source
    assert 'create_timer' not in source
    assert 'This window must have keyboard focus' in source
    for key_description in ('+ / =', '] / [', '. / ,', 'H / ?'):
        assert key_description in source


def test_launcher_whitelists_speed_overrides_and_passes_ros_parameters():
    source = (
        REPOSITORY_ROOT / 'scripts' / 'run_teleop.sh'
    ).read_text(encoding='utf-8')
    for name in (
        'linear_speed',
        'angular_speed',
        'linear_speed_step',
        'angular_speed_step',
        'min_linear_speed',
        'min_angular_speed',
        'max_linear_speed',
        'max_angular_speed',
    ):
        assert f'{name}:=*' in source
    assert 'parameter_arguments+=(--param "${override}")' in source
    assert 'unsupported teleop speed override' in source
