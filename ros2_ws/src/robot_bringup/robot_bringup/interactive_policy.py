"""Pure policy for RViz and keyboard interaction in each operation."""

from dataclasses import dataclass
from pathlib import Path
import shutil


_OPERATIONS = {
    'mapping',
    'incremental_mapping',
    'localization',
    'navigation',
}
_MAPPING_OPERATIONS = {'mapping', 'incremental_mapping'}
_RVIZ_CONFIGS = {
    'mapping': 'mapping.rviz',
    'incremental_mapping': 'mapping.rviz',
    'localization': 'localization.rviz',
    'navigation': 'navigation.rviz',
}
_TRUE = {'1', 'true', 'yes', 'on'}
_FALSE = {'0', 'false', 'no', 'off'}


@dataclass(frozen=True)
class InteractiveSelection:
    """Resolved interactive processes for one ROS operation."""

    use_rviz: bool
    rviz_config: str
    use_teleop: bool


def parse_bool(value, name):
    """Parse the launch-style boolean values accepted by this project."""
    normalized = str(value).strip().lower()
    if normalized in _TRUE:
        return True
    if normalized in _FALSE:
        return False
    raise ValueError(f'{name} must be true or false; got {value!r}')


def resolve_interactive_selection(
        *, operation, interactive, use_rviz, rviz_config, use_teleop,
        robot_description_share):
    """Validate and resolve RViz/teleop choices before nodes are launched."""
    operation = str(operation).strip().lower()
    if operation not in _OPERATIONS:
        raise ValueError(f'unsupported operation: {operation!r}')

    if not parse_bool(interactive, 'interactive'):
        return InteractiveSelection(
            use_rviz=False,
            rviz_config='',
            use_teleop=False,
        )

    rviz_enabled = parse_bool(use_rviz, 'use_rviz')
    teleop_value = str(use_teleop).strip().lower()
    if teleop_value == 'auto':
        teleop_enabled = operation in _MAPPING_OPERATIONS
    else:
        teleop_enabled = parse_bool(teleop_value, 'use_teleop')

    if teleop_enabled and operation not in _MAPPING_OPERATIONS:
        raise ValueError(
            f'use_teleop=true is unsafe for {operation}; mapping teleop '
            'may only own /cmd_vel in mapping or incremental_mapping')

    resolved_config = ''
    if rviz_enabled:
        config_value = str(rviz_config).strip()
        if not config_value or config_value.lower() == 'auto':
            resolved = (
                Path(robot_description_share)
                / 'rviz'
                / _RVIZ_CONFIGS[operation]
            )
        else:
            resolved = Path(config_value).expanduser()
            if not resolved.is_absolute():
                resolved = resolved.resolve()
        if not resolved.is_file():
            raise ValueError(f'RViz config does not exist: {resolved}')
        resolved_config = str(resolved)

    return InteractiveSelection(
        use_rviz=rviz_enabled,
        rviz_config=resolved_config,
        use_teleop=teleop_enabled,
    )


def teleop_terminal_command(
        run_teleop, arguments=(), find_executable=shutil.which):
    """Return a foreground terminal command that owns the teleop child."""
    run_teleop = str(Path(run_teleop))
    arguments = tuple(str(argument) for argument in arguments)
    if any(not argument or '\x00' in argument for argument in arguments):
        raise ValueError('teleop terminal arguments must be non-empty strings')
    teleop_command = [run_teleop, *arguments]
    candidates = (
        (
            'gnome-terminal',
            lambda path: [
                path,
                '--wait',
                '--title=Isaac Nav Mapping Teleop',
                '--',
                *teleop_command,
            ],
        ),
        (
            'xterm',
            lambda path: [
                path,
                '-T',
                'Isaac Nav Mapping Teleop',
                '-e',
                *teleop_command,
            ],
        ),
        (
            'konsole',
            lambda path: [
                path,
                '--nofork',
                '-p',
                'tabtitle=Isaac Nav Mapping Teleop',
                '-e',
                *teleop_command,
            ],
        ),
    )
    for executable, build in candidates:
        path = find_executable(executable)
        if path:
            return build(path)
    raise ValueError(
        'interactive mapping teleop requires gnome-terminal, xterm, or '
        'konsole; install one or launch with use_teleop:=false')
