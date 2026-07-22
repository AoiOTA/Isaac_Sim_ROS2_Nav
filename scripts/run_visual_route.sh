#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

kind="${1:-}"
case "${kind}" in
  static) scenario_name="kujiale_static_visual.yaml" ;;
  dynamic) scenario_name="kujiale_dynamic_visual.yaml" ;;
  *) die "usage: $0 static|dynamic" ;;
esac

source_ros --require-workspace
scenario_file="${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/${scenario_name}"
spawn_poses_file="${ISAAC_NAV_SPAWN_POSES:-${PROJECT_ROOT}/isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.spawn.yaml}"
require_file "${scenario_file}"
require_file "${spawn_poses_file}"

# This is intentionally not run_experiment.sh: visual mode sends the same
# route but creates no project evidence directory, bag, JSON, CSV, or report.
exec ros2 launch robot_experiments experiment.launch.py \
  "scenario_file:=${scenario_file}" \
  "spawn_poses_file:=${spawn_poses_file}" \
  "record_evidence:=false"
