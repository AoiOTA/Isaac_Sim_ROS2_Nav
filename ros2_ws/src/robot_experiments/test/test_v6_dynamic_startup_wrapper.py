import os
from pathlib import Path
import shlex
import shutil
import subprocess


REPO = Path(__file__).resolve().parents[4]
STACK = REPO / "scripts/run_v6_low_obstacle_phase_f_stack.sh"
STARTUP = REPO / "scripts/lib/v6_dynamic_startup.sh"


def _fake_startup(tmp_path: Path, *, has_candidate=True, asset_available=True):
    project = tmp_path / "bio_nav_module3"
    (tmp_path / "module2-assets").mkdir(parents=True)
    scripts = project / "scripts"
    (scripts / "lib").mkdir(parents=True)
    shutil.copy2(STACK, scripts / STACK.name)
    shutil.copy2(STARTUP, scripts / "lib" / STARTUP.name)
    (project / "isaac_sim/tools").mkdir(parents=True)
    (project / "isaac_sim/tools/import_assets.py").touch()

    integration = tmp_path / "bio_nav_integration"
    prefix = integration / "ros2_ws/install"
    prefix.mkdir(parents=True)
    setup = prefix / "local_setup.bash"
    setup.write_text(
        f"export FAKE_INTEGRATION_PREFIX={shlex.quote(str(prefix))}\n"
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
require_directory() {
  if [[ -n "${FAKE_REQUIRED_DIRS_LOG:-}" ]]; then
    printf '%s\n' "$1" >>"${FAKE_REQUIRED_DIRS_LOG}"
  fi
  [[ -d "$1" ]]
}
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


def test_phase_f_default_uses_sibling_integration_with_spaces(tmp_path):
    checkout = tmp_path / "checkout with spaces"
    scripts, integration, env = _fake_startup(checkout, has_candidate=False)
    module2 = checkout / "module2"
    constraints = module2 / "configs/kujiale_0026_module1_visual_shadow_v310.yaml"
    constraints.parent.mkdir(parents=True)
    constraints.touch()
    integration_scripts = integration / "scripts"
    integration_scripts.mkdir()
    for name in ("run_module2_v310_server.sh", "run_v6_module2_causal_obstacle_server.sh"):
        (integration_scripts / name).touch()
    run_dir = checkout / "must_not_exist"
    required_dirs = checkout / "required-dirs.log"
    env.pop("BIO_NAV_INTEGRATION_ROOT")
    env["BIO_NAV_MODULE2_V310_ROOT"] = str(module2)
    env["FAKE_REQUIRED_DIRS_LOG"] = str(required_dirs)
    result = subprocess.run(
        [str(scripts / STACK.name), "M3", "--domain", "150", "--run-dir", str(run_dir),
         "--socket", str(checkout / "socket/module2.sock"),
         "--module2-asset-root", str(checkout / "module2-assets")],
        capture_output=True, text=True, env=env,
    )

    assert result.returncode == 2
    assert "does not provide CognitivePoseModeCandidate" in result.stderr
    assert required_dirs.read_text(encoding="utf-8").splitlines()[0] == str(
        integration
    )
    assert not run_dir.exists()


def test_phase_f_preserves_explicit_integration_root_with_spaces(tmp_path):
    checkout = tmp_path / "checkout"
    scripts, integration, env = _fake_startup(checkout, has_candidate=False)
    explicit = tmp_path / "explicit integration with spaces"
    shutil.copytree(integration, explicit)
    prefix = explicit / "ros2_ws/install"
    (prefix / "local_setup.bash").write_text(
        f"export FAKE_INTEGRATION_PREFIX={shlex.quote(str(prefix))}\n"
        "export FAKE_HAS_CANDIDATE=0\n"
        'export OVERLAY_ORDER="${OVERLAY_ORDER},integration-explicit"\n',
        encoding="utf-8",
    )
    integration_scripts = explicit / "scripts"
    integration_scripts.mkdir()
    for name in (
        "run_module2_v310_server.sh",
        "run_v6_module2_causal_obstacle_server.sh",
    ):
        (integration_scripts / name).touch()
    module2 = checkout / "module2"
    constraints = module2 / "configs/kujiale_0026_module1_visual_shadow_v310.yaml"
    constraints.parent.mkdir(parents=True)
    constraints.touch()
    required_dirs = checkout / "required-dirs.log"
    env["BIO_NAV_INTEGRATION_ROOT"] = str(explicit)
    env["BIO_NAV_MODULE2_V310_ROOT"] = str(module2)
    env["FAKE_REQUIRED_DIRS_LOG"] = str(required_dirs)
    result = subprocess.run(
        [str(scripts / STACK.name), "M3", "--domain", "150",
         "--run-dir", str(checkout / "must_not_exist"),
         "--socket", str(checkout / "socket/module2.sock"),
         "--module2-asset-root", str(checkout / "module2-assets")],
        capture_output=True, text=True, env=env,
    )

    assert result.returncode == 2
    assert "does not provide CognitivePoseModeCandidate" in result.stderr
    assert required_dirs.read_text(encoding="utf-8").splitlines()[0] == str(explicit)
