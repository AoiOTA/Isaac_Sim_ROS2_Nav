#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

require_executable "${ISAAC_PYTHON}"
require_file "${ISAAC_ASSET_ROOT}/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
require_file "${ISAAC_ASSET_ROOT}/Isaac/Robots/Clearpath/Jackal/jackal.usd"
require_file "${ISAAC_ASSET_ROOT}/Isaac/Robots/Clearpath/Jackal/configuration/jackal_robot_schema.usd"
require_command colcon
require_command flock
require_command nvidia-smi
require_command python3
require_command realpath
require_command timeout
require_executable "${PROJECT_ROOT}/scripts/run_rviz.sh"
require_executable "${PROJECT_ROOT}/scripts/run_teleop.sh"
require_executable "${PROJECT_ROOT}/scripts/run_teleop_terminal.sh"
require_executable "${PROJECT_ROOT}/scripts/run_camera_view.sh"
require_executable "${PROJECT_ROOT}/scripts/profile_runtime.sh"
require_executable "${PROJECT_ROOT}/scripts/performance_mode.sh"
require_executable "${PROJECT_ROOT}/scripts/setup_ros_env.sh"

"${ISAAC_PYTHON}" -c 'from importlib.metadata import version; assert version("isaacsim") == "6.0.1.0"; print("Isaac Sim", version("isaacsim"))'

source_ros --require-workspace
for package in nav2_bringup nav2_mppi_controller nav2_smac_planner \
  nav2_velocity_smoother nav2_collision_monitor slam_toolbox \
  nav2_rviz_plugins pointcloud_to_laserscan robot_localization rviz2 xacro \
  robot_bringup robot_description robot_experiments \
  robot_localization_config robot_mapping robot_navigation robot_odometry \
  robot_perception robot_teleop; do
  ros2 pkg prefix "${package}" >/dev/null
done

nav2_rviz_prefix="$(ros2 pkg prefix nav2_rviz_plugins)"
nav2_plugin_xml="${nav2_rviz_prefix}/share/nav2_rviz_plugins/plugins_description.xml"
require_file "${nav2_plugin_xml}"
grep -q 'name="nav2_rviz_plugins/GoalTool"' "${nav2_plugin_xml}" \
  || die "Jazzy Nav2 GoalTool plugin is unavailable: ${nav2_plugin_xml}"
grep -q 'name="nav2_rviz_plugins/Navigation 2"' "${nav2_plugin_xml}" \
  || die "Jazzy Nav2 Navigation 2 panel is unavailable: ${nav2_plugin_xml}"

description_prefix="$(ros2 pkg prefix robot_description)"
for config_name in mapping.rviz localization.rviz navigation.rviz; do
  require_file "${description_prefix}/share/robot_description/rviz/${config_name}"
done
teleop_prefix="$(ros2 pkg prefix robot_teleop)"
require_file "${teleop_prefix}/share/robot_teleop/config/teleop.yaml"

MAP_MANIFEST="${PROJECT_ROOT}/data/maps/manifests/warehouse_v1.yaml"
require_file "${MAP_MANIFEST}"
python3 - "${PROJECT_ROOT}" "${MAP_MANIFEST}" <<'PY'
import hashlib
from pathlib import Path
import sys

import yaml

project_root = Path(sys.argv[1])
manifest = yaml.safe_load(Path(sys.argv[2]).read_text(encoding="utf-8"))
entries = [
    *manifest["occupancy_grid"]["files"],
    *manifest["pose_graph"]["files"],
]
for entry in entries:
    path = project_root / entry["path"]
    if not path.is_file():
        raise SystemExit(f"missing curated map artifact: {path}")
    if path.read_bytes().startswith(b"version https://git-lfs.github.com/spec/v1"):
        raise SystemExit(f"Git LFS artifact is not hydrated: {path}; run git lfs pull")
    if path.stat().st_size != entry["bytes"]:
        raise SystemExit(f"map artifact size mismatch: {path}")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != entry["sha256"]:
        raise SystemExit(f"map artifact SHA256 mismatch: {path}")
print(f"map baseline: {manifest['map_version']} (integrity verified)")
PY

nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader

terminal=""
for candidate in gnome-terminal xterm konsole; do
  if command -v "${candidate}" >/dev/null 2>&1; then
    terminal="${candidate}"
    break
  fi
done
if [[ -n "${terminal}" ]]; then
  log_info "mapping teleop terminal: ${terminal}"
else
  log_warn "no supported terminal emulator found; interactive mapping teleop will require manual startup"
fi

if [[ -d "${ISAAC_NAV_RUNTIME_DIR}" ]]; then
  [[ ! -L "${ISAAC_NAV_RUNTIME_DIR}" ]] \
    || die "runtime directory must not be a symlink: ${ISAAC_NAV_RUNTIME_DIR}"
  runtime_owner="$(stat -c '%u' "${ISAAC_NAV_RUNTIME_DIR}")"
  [[ "${runtime_owner}" == "${UID}" ]] \
    || die "runtime directory is owned by uid ${runtime_owner}: ${ISAAC_NAV_RUNTIME_DIR}"
  for component in isaac ros rviz teleop; do
    lock_file="${ISAAC_NAV_RUNTIME_DIR}/${component}.lock"
    [[ -e "${lock_file}" ]] || continue
    if ! (
      exec 9<>"${lock_file}"
      flock -n 9
    ); then
      die "${component} runtime lock is active; stop the existing stack before startup"
    fi
  done
fi

nodes="$(timeout 3 ros2 node list --no-daemon --spin-time 0.5 2>/dev/null || true)"
duplicates="$(printf '%s\n' "${nodes}" | sed '/^$/d' | sort | uniq -d)"
[[ -z "${duplicates}" ]] \
  || die "duplicate ROS node names detected: $(printf '%s' "${duplicates}" | tr '\n' ' ')"

shopt -s nullglob
shm_files=(
  /dev/shm/fastrtps_*
  /dev/shm/fastdds_*
  /dev/shm/sem.fastrtps_*
  /dev/shm/sem.fastdds_*
)
shopt -u nullglob
if ((${#shm_files[@]})); then
  root_owned=()
  for shm_file in "${shm_files[@]}"; do
    owner="$(stat -c '%u' "${shm_file}" 2>/dev/null || printf unknown)"
    [[ "${owner}" != "0" ]] || root_owned+=("${shm_file}")
  done
  ((${#root_owned[@]} == 0)) \
    || die "root-owned Fast DDS SHM artifacts require manual review: ${root_owned[*]}"
  log_warn "found ${#shm_files[@]} Fast DDS SHM artifacts; inspect with scripts/diagnose.sh and clean only while DDS is inactive"
fi

governors=(/sys/devices/system/cpu/cpu*/cpufreq/scaling_governor)
if [[ -e "${governors[0]}" ]]; then
  non_performance=()
  for governor in "${governors[@]}"; do
    [[ "$(<"${governor}")" == "performance" ]] \
      || non_performance+=("${governor}=$(<"${governor}")")
  done
  ((${#non_performance[@]} == 0)) \
    || log_warn "CPU governor is not performance on ${#non_performance[@]} cores; record this for MPPI benchmarks"
fi

echo "ROS 2: ${ROS_DISTRO:-unknown}; RMW: ${RMW_IMPLEMENTATION}; domain: ${ROS_DOMAIN_ID}"
echo "asset root: ${ISAAC_ASSET_ROOT}"
echo "preflight: PASS"
