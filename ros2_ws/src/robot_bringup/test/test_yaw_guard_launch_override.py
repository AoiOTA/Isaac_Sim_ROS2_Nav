from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_stack_exposes_one_default_off_wheel_yaw_guard_override():
    source = (
        PACKAGE_ROOT / 'launch' / 'ros_stack.launch.py'
    ).read_text(encoding='utf-8')
    declaration = (
        'DeclareLaunchArgument(\n            '
        "'yaw_disagreement_guard_enabled'")
    assert source.count(declaration) == 1
    assert "'yaw_disagreement_guard_enabled',\n            default_value='false'" \
        in source
    assert "'yaw_disagreement_guard_enabled': LaunchConfiguration(" \
        in source
