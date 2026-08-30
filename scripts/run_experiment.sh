#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

scenario_file="${1:-}"
output_directory="${2:-}"
[[ -n "${scenario_file}" && -n "${output_directory}" ]] \
  || die "usage: $0 SCENARIO_FILE OUTPUT_DIRECTORY [experiment.launch.py arguments...]"
shift 2

source_ros --require-workspace --require-integration-underlay
scenario_file="$(realpath -e "${scenario_file}")"
# Navigation experiments default to the active Kujiale calibration, matching
# scripts/run_ros.sh.  Warehouse callers can still select their own profile
# explicitly through ISAAC_NAV_SPAWN_POSES.
default_spawn_poses_file="${ISAAC_NAV_SPAWN_POSES:-${PROJECT_ROOT}/isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.spawn.yaml}"
spawn_poses_file="${default_spawn_poses_file}"
spawn_override_count=0
for argument in "$@"; do
  case "${argument}" in
    spawn_poses_file:=*)
      spawn_override_count=$((spawn_override_count + 1))
      spawn_poses_file="${argument#spawn_poses_file:=}"
      ;;
  esac
done
((spawn_override_count <= 1)) \
  || die "spawn_poses_file may be supplied at most once"
require_file "${scenario_file}"
require_file "${spawn_poses_file}"
mkdir -p "${output_directory}"
output_directory="$(realpath -e "${output_directory}")"

spawn_launch_argument=()
if ((spawn_override_count == 0)); then
  spawn_launch_argument=("spawn_poses_file:=${spawn_poses_file}")
fi

exec ros2 launch robot_experiments experiment.launch.py \
  "scenario_file:=${scenario_file}" \
  "${spawn_launch_argument[@]}" \
  "output_directory:=${output_directory}" \
  "$@"
