from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _amcl_params(name):
    document = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / name).read_text(encoding='utf-8'))
    return document['amcl']['ros__parameters']


def test_isaac_odom_variant_differs_only_in_odometry_alphas():
    baseline = _amcl_params('amcl.yaml')
    isaac = _amcl_params('amcl_isaac_odom.yaml')

    # Isaac ideal/compute odometry is near-noiseless; the variant keeps small
    # non-zero alphas so the particle filter cannot collapse.
    assert isaac['alpha1'] == 0.05
    assert isaac['alpha2'] == 0.05
    assert isaac['alpha3'] == 0.02
    assert isaac['alpha4'] == 0.02
    for key in ('alpha1', 'alpha2', 'alpha3', 'alpha4'):
        assert isaac[key] > 0.0
        assert isaac[key] < baseline[key]
    for key in baseline:
        if key.startswith('alpha'):
            continue
        assert isaac[key] == baseline[key], key
    assert set(isaac) == set(baseline)


def test_amcl_params_file_defaults_to_baseline_and_is_overridable():
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'localization.launch.py').read_text(
            encoding='utf-8')

    assert "default_amcl_config = package_share / 'config' / 'amcl.yaml'" \
        in launch_source
    assert "DeclareLaunchArgument(\n            'amcl_params_file', " \
        'default_value=str(default_amcl_config))' in launch_source
    assert "LaunchConfiguration('amcl_params_file')" in launch_source
