import os
from pathlib import Path
import shutil
import subprocess


REPO = Path(__file__).resolve().parents[4]
WRAPPER = REPO / "scripts/run_v6_single_dynamic_low_obstacle.sh"
STACK = REPO / "scripts/run_v6_low_obstacle_phase_f_stack.sh"
STARTUP = REPO / "scripts/lib/v6_dynamic_startup.sh"


def _fake_startup(tmp_path: Path, *, has_candidate=True, asset_available=True):
    project = tmp_path / "module3"
    scripts = project / "scripts"
    (scripts / "lib").mkdir(parents=True)
    shutil.copy2(WRAPPER, scripts / WRAPPER.name)
    shutil.copy2(STACK, scripts / STACK.name)
    shutil.copy2(STARTUP, scripts / "lib" / STARTUP.name)
    (project / "isaac_sim/tools").mkdir(parents=True)
    (project / "isaac_sim/tools/import_assets.py").touch()

    integration = tmp_path / "integration"
    prefix = integration / "ros2_ws/install_run4_candidate"
    prefix.mkdir(parents=True)
    setup = prefix / "setup.bash"
    setup.write_text(
        f"export FAKE_INTEGRATION_PREFIX={prefix}\n"
        f"export FAKE_HAS_CANDIDATE={'1' if has_candidate else '0'}\n"
        'export OVERLAY_ORDER="${OVERLAY_ORDER},integration"\n',
        encoding="utf-8",
    )
    ros_setup = tmp_path / "jazzy_setup.bash"
    ros_setup.write_text("export OVERLAY_ORDER=jazzy\n", encoding="utf-8")
    local_setup = project / "ros2_ws/install/local_setup.bash"
    local_setup.parent.mkdir(parents=True)
    local_setup.write_text(
        'export OVERLAY_ORDER="${OVERLAY_ORDER},module3"\n', encoding="utf-8"
    )
    (scripts / "lib/common.sh").write_text(
        """#!/usr/bin/env bash
export PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ISAAC_PYTHON="${ISAAC_PYTHON}"
export ISAAC_ASSET_ROOT="${ISAAC_ASSET_ROOT}"
require_directory() { [[ -d "$1" ]]; }
require_file() { [[ -f "$1" ]]; }
source_ros() {
  source "${ROS_SETUP}"
  source "${BIO_NAV_INTEGRATION_SETUP}"
  source "${PROJECT_ROOT}/ros2_ws/install/local_setup.bash"
}
""",
        encoding="utf-8",
    )

    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    dispatch_log = tmp_path / "dispatch.log"
    ros2 = fake_bin / "ros2"
    ros2.write_text(
        """#!/usr/bin/env bash
if [[ "${1:-}" == pkg && "${2:-}" == prefix ]]; then
  printf '%s/%s\n' "${FAKE_INTEGRATION_PREFIX}" "$3"
  exit 0
fi
printf '%s|%s\n' "${OVERLAY_ORDER:-unsourced}" "$*" >>"${FAKE_DISPATCH_LOG}"
""",
        encoding="utf-8",
    )
    ros2.chmod(0o755)
    python3 = fake_bin / "python3"
    python3.write_text(
        """#!/usr/bin/env bash
[[ "$*" == *CognitivePoseModeCandidate* ]] || exit 3
[[ "${FAKE_HAS_CANDIDATE:-0}" == 1 ]]
""",
        encoding="utf-8",
    )
    python3.chmod(0o755)
    asset_python = fake_bin / "asset-python"
    asset_python.write_text(
        """#!/usr/bin/env bash
printf '%s|%s\n' "${OVERLAY_ORDER:-unsourced}" "$*" >>"${FAKE_ASSET_LOG}"
if [[ "$*" == *--check* ]]; then
  [[ -f "${FAKE_IMPORTED_ASSET}" ]]
else
  [[ "${FAKE_ASSET_AVAILABLE}" == 1 ]] || exit 2
  : >"${FAKE_IMPORTED_ASSET}"
fi
""",
        encoding="utf-8",
    )
    asset_python.chmod(0o755)

    env = os.environ.copy()
    env.update({
        "PATH": f"{fake_bin}:{env['PATH']}",
        "ROS_SETUP": str(ros_setup),
        "BIO_NAV_INTEGRATION_ROOT": str(integration),
        "ISAAC_PYTHON": str(asset_python),
        "ISAAC_ASSET_ROOT": str(tmp_path / "assets"),
        "FAKE_ASSET_AVAILABLE": "1" if asset_available else "0",
        "FAKE_IMPORTED_ASSET": str(tmp_path / "jackal_original.usd"),
        "FAKE_ASSET_LOG": str(tmp_path / "asset.log"),
        "FAKE_DISPATCH_LOG": str(dispatch_log),
    })
    env.pop("BIO_NAV_INTEGRATION_SETUP", None)
    return scripts, integration, env


def _run_wrapper(scripts: Path, env: dict[str, str], command: str):
    return subprocess.run(
        ["bash", str(scripts / WRAPPER.name), command, "M3", "--output-root", "/tmp/fake"],
        capture_output=True, text=True, env=env,
    )


def test_plan_does_not_import_assets(tmp_path):
    scripts, _, env = _fake_startup(tmp_path)
    result = _run_wrapper(scripts, env, "plan")

    assert result.returncode == 0
    assert not Path(env["FAKE_ASSET_LOG"]).exists()


def test_run_imports_then_checks_assets_after_current_overlay(tmp_path):
    scripts, integration, env = _fake_startup(tmp_path)
    result = _run_wrapper(scripts, env, "run")

    assert result.returncode == 0, result.stderr
    calls = Path(env["FAKE_ASSET_LOG"]).read_text(encoding="utf-8").splitlines()
    assert len(calls) == 2
    assert all(row.startswith("jazzy,integration,module3|") for row in calls)
    assert "--check" not in calls[0]
    assert "--check" in calls[1]
    assert os.environ.get("BIO_NAV_INTEGRATION_SETUP") is None
    assert (integration / "ros2_ws/install_run4_candidate/setup.bash").is_file()
    dispatch = Path(env["FAKE_DISPATCH_LOG"]).read_text(encoding="utf-8")
    assert "jazzy,integration,module3|run robot_experiments" in dispatch


def test_missing_asset_fails_before_ros2_dispatch(tmp_path):
    scripts, _, env = _fake_startup(tmp_path, asset_available=False)
    result = _run_wrapper(scripts, env, "run")

    assert result.returncode == 2
    assert not Path(env["FAKE_DISPATCH_LOG"]).exists()


def test_stale_overlay_fails_before_asset_or_dispatch(tmp_path):
    scripts, _, env = _fake_startup(tmp_path, has_candidate=False)
    result = _run_wrapper(scripts, env, "run")

    assert result.returncode == 2
    assert "does not provide CognitivePoseModeCandidate" in result.stderr
    assert not Path(env["FAKE_ASSET_LOG"]).exists()
    assert not Path(env["FAKE_DISPATCH_LOG"]).exists()


def test_phase_f_stale_overlay_fails_before_run_directory(tmp_path):
    scripts, integration, env = _fake_startup(tmp_path, has_candidate=False)
    module2 = tmp_path / "module2"
    constraints = module2 / "configs/kujiale_0026_module1_visual_shadow_v310.yaml"
    constraints.parent.mkdir(parents=True)
    constraints.touch()
    integration_scripts = integration / "scripts"
    integration_scripts.mkdir()
    for name in ("run_module2_v310_server.sh", "run_v6_module2_causal_obstacle_server.sh"):
        (integration_scripts / name).touch()
    run_dir = tmp_path / "must_not_exist"
    env["BIO_NAV_MODULE2_V310_ROOT"] = str(module2)
    result = subprocess.run(
        [str(scripts / STACK.name), "M3", "--domain", "150", "--run-dir", str(run_dir),
         "--socket", str(tmp_path / "socket/module2.sock")],
        capture_output=True, text=True, env=env,
    )

    assert result.returncode == 2
    assert "does not provide CognitivePoseModeCandidate" in result.stderr
    assert not run_dir.exists()
