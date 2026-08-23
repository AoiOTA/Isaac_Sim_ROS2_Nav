from __future__ import annotations

from pathlib import Path

from isaac_sim.apps.navigation_sim import effective_reset_seed


ROOT = Path(__file__).resolve().parents[2]


def test_cli_seed_is_the_effective_startup_and_first_trigger_default():
    effective = effective_reset_seed(8601, 17)
    startup_seed = effective
    reset_service_default_seed = effective
    assert startup_seed == reset_service_default_seed == 8601
    source = (ROOT / "isaac_sim/apps/navigation_sim.py").read_text(
        encoding="utf-8"
    )
    assert "default_reset_seed=effective_seed" in source
    assert "random_seed=(\n                    effective_seed" in source


def test_scenario_seed_is_the_fallback_when_cli_omits_override():
    assert effective_reset_seed(None, 90210) == 90210


def test_v6_grid_reset_bridge_has_no_global_pose_publication_path():
    source = (ROOT / "isaac_sim/src/bridge/reset_service.py").read_text(
        encoding="utf-8"
    )
    assert '"/initialpose"' not in source
    assert '"/simulation/localization_seeded"' not in source
    assert "_initial_pose_publisher" not in source
    assert "_localization_seeded_publisher" not in source
