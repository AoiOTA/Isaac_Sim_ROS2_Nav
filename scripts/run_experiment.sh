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

source_ros --require-workspace
scenario_file="$(realpath -e "${scenario_file}")"
spawn_poses_file="${ISAAC_NAV_SPAWN_POSES:-${PROJECT_ROOT}/isaac_sim/configs/spawn_poses.yaml}"
require_file "${scenario_file}"
require_file "${spawn_poses_file}"
mkdir -p "${output_directory}"
output_directory="$(realpath -e "${output_directory}")"

exec ros2 launch robot_experiments experiment.launch.py \
  "scenario_file:=${scenario_file}" \
  "spawn_poses_file:=${spawn_poses_file}" \
  "output_directory:=${output_directory}" \
  "$@"
