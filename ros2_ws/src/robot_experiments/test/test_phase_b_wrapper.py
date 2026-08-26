import os
from pathlib import Path
import shutil
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
    assert "waits for /wheel_odometry/reset before starting" in help_text
    assert "requires /set_pose in its bounded startup reset" in help_text
    assert "manifest command validates" in help_text

    source = WRAPPER.read_text(encoding="utf-8")
    isaac_case = source[source.index("  isaac)"):source.index("  ros)")]
    assert isaac_case.index("source_ros --require-integration-underlay") \
        < isaac_case.index("wait_for_ros_reset_services") \
        < isaac_case.index('exec "${SCRIPT_DIR}/run_isaac.sh"')


def _run_fake_isaac_startup(tmp_path, ready_services):
    project = tmp_path / "project"
    scripts = project / "scripts"
    (scripts / "lib").mkdir(parents=True)
    shutil.copy2(WRAPPER, scripts / WRAPPER.name)
    (scripts / "lib/common.sh").write_text(
        """#!/usr/bin/env bash
export PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
die() { printf '%s\\n' "$*" >&2; exit 1; }
require_file() { [[ -f "$1" ]] || die "missing: $1"; }
source_ros() { :; }
log_info() { printf '%s\\n' "$*"; }
""",
        encoding="utf-8",
    )
    run_isaac = scripts / "run_isaac.sh"
    run_isaac.write_text(
        "#!/usr/bin/env bash\nprintf '%s\\n' \"$@\"\n", encoding="utf-8"
    )
    run_isaac.chmod(0o755)
    for relative in (
        "ros2_ws/src/robot_experiments/config/v6_r5_phase_b_kujiale_exact_baseline.yaml",
        "data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml",
        "isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml",
        "ros2_ws/src/robot_route_planner/config/v6_kujiale_isaacgen_v1_gvg_v1.geojson",
    ):
        path = project / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
    module2 = tmp_path / "module2"
    shadow = module2 / "configs/kujiale_0026_module1_visual_shadow_v310.yaml"
    shadow.parent.mkdir(parents=True)
    shadow.touch()
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    ros2 = fake_bin / "ros2"
    ros2.write_text(
        """#!/usr/bin/env bash
if [[ "$1" == service && "$2" == type ]]; then
  case ":${FAKE_READY_SERVICES:-}:" in
    *":$3:"*) printf '%s\\n' std_srvs/srv/Empty ;;
  esac
fi
""",
        encoding="utf-8",
    )
    ros2.chmod(0o755)
    env = os.environ.copy()
    env.update(
        {
            "PATH": f"{fake_bin}:{env['PATH']}",
            "BIO_NAV_MODULE2_V310_ROOT": str(module2),
            "BIO_NAV_PHASE_B_ROS_READY_TIMEOUT_SEC": "1",
            "FAKE_READY_SERVICES": ":".join(ready_services),
        }
    )
    return subprocess.run(
        [str(scripts / WRAPPER.name), "isaac"],
        capture_output=True,
        text=True,
        timeout=5,
        env=env,
    )


def test_phase_b_isaac_starts_before_mixed_ekf_set_pose_exists(tmp_path):
    result = _run_fake_isaac_startup(tmp_path, ["/wheel_odometry/reset"])

    assert result.returncode == 0, result.stderr
    assert "pre-Isaac ROS reset service is ready" in result.stdout
    assert "--spawn-pose\nlong_route_start_g1" in result.stdout


def test_phase_b_isaac_still_fails_when_wheel_reset_is_missing(tmp_path):
    result = _run_fake_isaac_startup(tmp_path, ["/set_pose"])

    assert result.returncode == 1
    assert "/wheel_odometry/reset" in result.stderr
    assert "start the ros component first" in result.stderr


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
