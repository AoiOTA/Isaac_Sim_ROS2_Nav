from pathlib import Path


REPO = Path(__file__).resolve().parents[4]
WRAPPER = REPO / "scripts/run_v6_r5_phase_b_kujiale.sh"


def test_phase_b_wrapper_pins_exact_scene_and_effect_off_navigation():
    source = WRAPPER.read_text(encoding="utf-8")

    assert "/kujiale_0026/kujiale_0026_A_to_B_door_open.usd" in source
    assert "kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml" in source
    assert "v6_kujiale_isaacgen_v1.yaml" in source
    assert "v6_kujiale_isaacgen_v1_gvg_v1.geojson" in source
    assert "--mode mixed" in source
    assert "--no-dynamic-obstacles" in source
    assert "cognitive_profile:=M0" in source
    assert "module2_enabled:=false" in source
    assert "cognitive_graph_mode:=gvg" in source
    assert "cognitive_constraints_override_file:=${SHADOW_CONFIG_ABS}" in source


def test_phase_b_wrapper_uses_canonical_shadow_server_and_planning_prior_record():
    source = WRAPPER.read_text(encoding="utf-8")
    observability = (
        REPO
        / "ros2_ws/src/robot_experiments/robot_experiments/phase_b_observability.py"
    ).read_text(encoding="utf-8")

    assert "run_module2_v310_server.sh" in source
    assert "--shadow-config \"${SHADOW_CONFIG}\"" in source
    assert "startup_profile:=estimated_shadow" in source
    assert "phase_b_observability" in source
    assert '"/bio_nav/module2/planning_prior"' in observability
    assert "--pilot --dispatch-pilot" in source
