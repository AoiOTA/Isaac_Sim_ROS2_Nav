from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
# The resolver is a ROS-free helper module colocated with the launch files.
sys.path.insert(0, str(PACKAGE_ROOT / 'launch'))

from localization_backend import (  # noqa: E402
    AMCL_LOCALIZATION_BACKEND,
    IDEAL_LOCALIZATION_BACKEND,
    resolve_localization_backend,
    SLAM_TOOLBOX_LOCALIZATION_BACKEND,
)

import pytest  # noqa: E402


@pytest.mark.parametrize('backend', [
    IDEAL_LOCALIZATION_BACKEND,
    AMCL_LOCALIZATION_BACKEND,
    SLAM_TOOLBOX_LOCALIZATION_BACKEND,
])
def test_explicit_backend_is_selected(backend):
    assert resolve_localization_backend(backend, 'true') == backend
    assert resolve_localization_backend(backend, 'false') == backend


def test_empty_backend_falls_back_to_posegraph_boolean():
    assert resolve_localization_backend('', 'true') == (
        SLAM_TOOLBOX_LOCALIZATION_BACKEND)
    assert resolve_localization_backend('', 'false') == (
        IDEAL_LOCALIZATION_BACKEND)


def test_explicit_backend_overrides_posegraph_boolean():
    assert resolve_localization_backend('ideal', 'true') == (
        IDEAL_LOCALIZATION_BACKEND)
    assert resolve_localization_backend('slam_toolbox', 'false') == (
        SLAM_TOOLBOX_LOCALIZATION_BACKEND)
    assert resolve_localization_backend('amcl', 'false') == (
        AMCL_LOCALIZATION_BACKEND)


def test_backend_value_is_normalized():
    assert resolve_localization_backend(' AMCL ', 'false') == (
        AMCL_LOCALIZATION_BACKEND)
    assert resolve_localization_backend('Slam_Toolbox', 'false') == (
        SLAM_TOOLBOX_LOCALIZATION_BACKEND)


def test_invalid_backend_is_rejected():
    with pytest.raises(ValueError):
        resolve_localization_backend('cartographer', 'false')


def test_empty_backend_still_validates_legacy_boolean():
    with pytest.raises(ValueError):
        resolve_localization_backend('', 'yes')


def test_amcl_branch_wires_clear_localization_buffer_shim():
    launch_source = (PACKAGE_ROOT / 'launch' / 'localization.launch.py').read_text()

    assert "executable='amcl_clear_localization_buffer'" in launch_source
    assert launch_source.index("executable='amcl_clear_localization_buffer'") > (
        launch_source.index('elif backend == AMCL_LOCALIZATION_BACKEND:'))
