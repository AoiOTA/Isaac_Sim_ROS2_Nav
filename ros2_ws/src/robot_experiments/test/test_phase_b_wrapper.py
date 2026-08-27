import os
from pathlib import Path
import shutil
import select
import subprocess
import time
import uuid


REPO = Path(__file__).resolve().parents[4]
WRAPPER = REPO / "scripts/run_v6_r5_phase_b_kujiale.sh"
COMMON = REPO / "scripts/lib/common.sh"
BUILD_ROS2 = REPO / "scripts/build_ros2.sh"
WAIT_HELPER = REPO / "scripts/wait_for_empty_service.py"


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


def _run_common_shell(command, env=None):
    return subprocess.run(
        ["bash", "-c", f'source "{COMMON}"\n{command}'],
        capture_output=True,
        text=True,
        env=env,
    )


def test_module3_install_defaults_to_current_worktree_and_allows_override(tmp_path):
    default_env = os.environ.copy()
    default_env.pop("BIO_NAV_MODULE3_INSTALL", None)
    default = _run_common_shell('printf "%s" "${BIO_NAV_MODULE3_INSTALL}"', default_env)

    override_path = tmp_path / "selected-module3-install"
    override_env = {**default_env, "BIO_NAV_MODULE3_INSTALL": str(override_path)}
    override = _run_common_shell('printf "%s" "${BIO_NAV_MODULE3_INSTALL}"', override_env)

    assert default.returncode == 0, default.stderr
    assert default.stdout == str(REPO / "ros2_ws/install")
    assert override.returncode == 0, override.stderr
    assert override.stdout == str(override_path)


def test_build_sources_integration_underlay_without_module3_overlay(tmp_path):
    source = COMMON.read_text(encoding="utf-8")
    build_source = BUILD_ROS2.read_text(encoding="utf-8")
    ros_setup = tmp_path / "ros_setup.bash"
    ros_setup.write_text("export ROS_DISTRO=jazzy\n", encoding="utf-8")
    missing_install = tmp_path / "not-built-yet"

    result = _run_common_shell(
        """
source_v6_integration_underlay() { printf 'INTEGRATION\n'; }
validate_v6_integration_underlay() { :; }
validate_runtime_environment() { :; }
source_ros --require-integration-underlay --skip-module3-overlay
""",
        {
            **os.environ,
            "ROS_SETUP": str(ros_setup),
            "BIO_NAV_MODULE3_INSTALL": str(missing_install),
        },
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout == "INTEGRATION\n"
    assert (
        "source_ros --require-integration-underlay --skip-module3-overlay"
        in build_source
    )
    assert '--skip-module3-overlay)' in source


def test_runtime_integration_mode_requires_selected_module3_overlay(tmp_path):
    ros_setup = tmp_path / "ros_setup.bash"
    ros_setup.write_text("export ROS_DISTRO=jazzy\n", encoding="utf-8")
    missing_install = tmp_path / "selected-but-missing"

    result = _run_common_shell(
        """
source_v6_integration_underlay() { :; }
validate_v6_integration_underlay() { :; }
validate_runtime_environment() { :; }
source_ros --require-integration-underlay
""",
        {
            **os.environ,
            "ROS_SETUP": str(ros_setup),
            "BIO_NAV_MODULE3_INSTALL": str(missing_install),
        },
    )

    assert result.returncode == 1
    assert f"required file not found: {missing_install}/local_setup.bash" in result.stderr


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
    assert "ros2 service list" not in source
    assert '/usr/bin/python3 "${SCRIPT_DIR}/wait_for_empty_service.py"' in source
    assert isaac_case.index("source_ros --require-integration-underlay") \
        < isaac_case.index("wait_for_ros_reset_services") \
        < isaac_case.index('exec "${SCRIPT_DIR}/run_isaac.sh"')


def _run_fake_isaac_startup(tmp_path, *, helper_result="0", helper_sleep_sec="0"):
    project = tmp_path / "project"
    scripts = project / "scripts"
    (scripts / "lib").mkdir(parents=True)
    shutil.copy2(WRAPPER, scripts / WRAPPER.name)
    (scripts / WAIT_HELPER.name).write_text(
        """import os
import time
time.sleep(float(os.environ.get("FAKE_HELPER_SLEEP_SEC", "0")))
raise SystemExit(int(os.environ.get("FAKE_HELPER_RESULT", "0")))
""",
        encoding="utf-8",
    )
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
    env = os.environ.copy()
    env.update(
        {
            "BIO_NAV_MODULE2_V310_ROOT": str(module2),
            "BIO_NAV_PHASE_B_ROS_READY_TIMEOUT_SEC": "1",
            "FAKE_HELPER_RESULT": helper_result,
            "FAKE_HELPER_SLEEP_SEC": helper_sleep_sec,
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
    result = _run_fake_isaac_startup(tmp_path)

    assert result.returncode == 0, result.stderr
    assert "pre-Isaac ROS reset service is ready" in result.stdout
    assert "--spawn-pose\nlong_route_start_g1" in result.stdout


def test_phase_b_isaac_still_fails_when_wheel_reset_is_missing(tmp_path):
    result = _run_fake_isaac_startup(tmp_path, helper_result="1")

    assert result.returncode == 1
    assert "/wheel_odometry/reset" in result.stderr
    assert "start the ros component first" in result.stderr


def test_phase_b_reset_service_discovery_remains_bounded(tmp_path):
    started = time.monotonic()
    result = _run_fake_isaac_startup(
        tmp_path,
        helper_result="1",
        helper_sleep_sec="0.5",
    )
    elapsed = time.monotonic() - started

    assert result.returncode == 1
    assert elapsed < 2.0
    assert "not ready after 1s" in result.stderr


def _start_service(service_name, service_type, domain_id):
    server = subprocess.Popen(
        [
            "/usr/bin/python3",
            "-c",
            f"""import rclpy
from std_srvs.srv import {service_type}
rclpy.init()
node = rclpy.create_node('phase_b_test_service')
node.create_service({service_type}, {service_name!r}, lambda request, response: response)
print('READY', flush=True)
rclpy.spin(node)
""",
        ],
        env={**os.environ, "ROS_DOMAIN_ID": str(domain_id)},
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    assert server.stdout is not None
    readable, _, _ = select.select([server.stdout], [], [], 5.0)
    assert readable, "temporary rclpy service did not start"
    assert server.stdout.readline().strip() == "READY"
    return server


def _stop_service(server):
    server.terminate()
    try:
        server.wait(timeout=5)
    except subprocess.TimeoutExpired:
        server.kill()
        server.wait(timeout=5)


def _run_wait_helper(service_name, domain_id, timeout="1"):
    return subprocess.run(
        [
            "/usr/bin/python3",
            str(WAIT_HELPER),
            "--service",
            service_name,
            "--timeout",
            timeout,
        ],
        env={**os.environ, "ROS_DOMAIN_ID": str(domain_id)},
        capture_output=True,
        text=True,
        timeout=5,
    )


def test_typed_wait_helper_finds_empty_service_only_in_matching_domain():
    service_name = f"/phase_b_test/t_{uuid.uuid4().hex}/reset"
    domain_id = 60 + os.getpid() % 80
    server = _start_service(service_name, "Empty", domain_id)
    try:
        found = _run_wait_helper(service_name, domain_id)
        isolated = _run_wait_helper(service_name, domain_id + 1, timeout="0.25")
    finally:
        _stop_service(server)

    assert found.returncode == 0, found.stderr
    assert "[std_srvs/srv/Empty]" in found.stdout
    assert f"ROS_DOMAIN_ID={domain_id}" in found.stdout
    assert isolated.returncode == 1
    assert f"ROS_DOMAIN_ID={domain_id + 1}" in isolated.stderr


def test_typed_wait_helper_rejects_same_name_with_wrong_type():
    service_name = f"/phase_b_test/t_{uuid.uuid4().hex}/reset"
    domain_id = 140 + os.getpid() % 60
    server = _start_service(service_name, "Trigger", domain_id)
    try:
        result = _run_wait_helper(service_name, domain_id, timeout="0.25")
    finally:
        _stop_service(server)

    assert result.returncode == 1
    assert service_name in result.stderr
    assert "[std_srvs/srv/Empty]" in result.stderr


def test_typed_wait_helper_missing_service_timeout_is_bounded():
    service_name = f"/phase_b_test/t_{uuid.uuid4().hex}/missing"
    domain_id = 200 + os.getpid() % 30
    started = time.monotonic()
    result = _run_wait_helper(service_name, domain_id, timeout="0.25")
    elapsed = time.monotonic() - started

    assert result.returncode == 1
    assert elapsed < 2.0
    assert "not ready after 0.25s" in result.stderr


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
