#!/usr/bin/env bash
# Explicit entrypoints for the frozen V6 low-obstacle profile. Existing
# static/dynamic campaigns remain untouched and keep their original layouts.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

profile="${1:-}"
[[ -n "${profile}" ]] \
  || die "usage: $0 isaac|ros|runner [M0|M1|M2|M3] [arguments...]"
shift

scenario_file="${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/v6_kujiale_low_obstacles_static.yaml"
require_file "${scenario_file}"

case "${profile}" in
  isaac)
    exec "${SCRIPT_DIR}/run_kujiale_4x20_isaac.sh" v6-low-obstacles "$@"
    ;;
  ros)
    cognitive_profile="${V6_COGNITIVE_PROFILE:-M3}"
    if [[ "${1:-}" =~ ^M[0-3]$ ]]; then
      cognitive_profile="$1"
      shift
    fi
    [[ "${cognitive_profile}" =~ ^M[0-3]$ ]] \
      || die "V6 cognitive profile must be M0, M1, M2, or M3; got: ${cognitive_profile}"
    export ISAAC_NAV_REQUIRE_V6_INTEGRATION=1
    exec "${SCRIPT_DIR}/run_ros.sh" navigation \
      odometry_mode:=estimated localization_profile:=kujiale \
      nav2_profile:=v6_low_obstacle_isolation \
      cognitive_profile:="${cognitive_profile}" "$@"
    ;;
  runner)
    output_directory="${1:-${PROJECT_ROOT}/data/experiment_runs/v6_kujiale_low_obstacles}"
    [[ $# -eq 0 ]] || shift
    source_ros --require-workspace
    exec ros2 launch robot_experiments experiment.launch.py \
      scenario_file:="${scenario_file}" \
      output_directory:="${output_directory}" "$@"
    ;;
  *) die "profile must be isaac, ros, or runner; got: ${profile}" ;;
esac
