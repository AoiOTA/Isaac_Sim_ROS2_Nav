from pathlib import Path
import subprocess


REPO = Path(__file__).resolve().parents[4]
WRAPPER = REPO / "scripts/run_v6_r5_phase_b_kujiale.sh"
COMMON = REPO / "scripts/lib/common.sh"


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


def test_phase_b_integration_mode_sources_current_module3_overlay():
    source = COMMON.read_text(encoding="utf-8")

    integration_branch = source.index(
        'workspace_setup="${PROJECT_ROOT}/ros2_ws/install/local_setup.bash"'
    )
    overlay_source = source.index('source "${workspace_setup}"', integration_branch)
    validation = source.index("validate_v6_integration_underlay", overlay_source)

    assert overlay_source < validation
    assert 'if [[ -f "${workspace_setup}" ]]; then' in source
    assert '"${require_workspace}" == true || "${require_integration}" == true' \
        in source


def test_phase_b_help_declares_ros_first_and_isaac_readiness_order():
    result = subprocess.run(
        ["bash", str(WRAPPER), "--help"],
        check=True,
        capture_output=True,
        text=True,
    )
    help_text = result.stdout

    assert help_text.index("1. ros") < help_text.index("2. isaac")
    assert "/wheel_odometry/reset and /set_pose" in help_text
    assert "manifest command validates" in help_text

    source = WRAPPER.read_text(encoding="utf-8")
    isaac_case = source[source.index("  isaac)"):source.index("  ros)")]
    assert isaac_case.index("source_ros --require-integration-underlay") \
        < isaac_case.index("wait_for_ros_reset_services") \
        < isaac_case.index('exec "${SCRIPT_DIR}/run_isaac.sh"')


def test_phase_b_socket_defaults_local_and_preserves_absolute_override():
    source = WRAPPER.read_text(encoding="utf-8")

    assert '${XDG_RUNTIME_DIR}/bio_nav_phase_b_${UID}' in source
    assert '/tmp/bio_nav_phase_b_${UID}' in source
    assert '/domain_${domain_id}/${run_id}/module2.sock' in source
    assert '${run_root}/runtime/module2.sock' not in source
    assert 'socket_path="${BIO_NAV_PHASE_B_SOCKET_PATH}"' in source
    assert '[[ "${socket_path}" == /* ]]' in source
    assert 'mkdir -p -m 700 "${socket_directory}"' in source


def test_phase_b_wrapper_has_valid_bash_syntax():
    subprocess.run(["bash", "-n", str(WRAPPER)], check=True)
