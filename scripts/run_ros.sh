#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

operation="${1:-}"
[[ -n "${operation}" ]] || die "usage: $0 mapping|incremental_mapping|localization|navigation [launch arguments...]"
shift

case "${operation}" in
  mapping|incremental_mapping|localization|navigation) ;;
  *) die "unsupported operation: ${operation}" ;;
esac

source_ros --require-workspace
acquire_instance_lock ros "ROS stack"
export ISAAC_NAV_SPAWN_POSES="${ISAAC_NAV_SPAWN_POSES:-${PROJECT_ROOT}/isaac_sim/configs/spawn_poses.yaml}"

launch_args=("$@")
if [[ "${operation}" == "localization" || "${operation}" == "navigation" ]]; then
  posegraph_file=""
  map_file=""
  for argument in "${launch_args[@]}"; do
    case "${argument}" in
      posegraph_file:=*) posegraph_file="${argument#posegraph_file:=}" ;;
      map_file:=*) map_file="${argument#map_file:=}" ;;
    esac
  done
  if [[ -z "${map_file}" && -n "${posegraph_file}" ]]; then
    posegraph_prefix="${posegraph_file%.posegraph}"
    posegraph_prefix="${posegraph_prefix%.data}"
    map_file="${PROJECT_ROOT}/data/maps/occupancy/$(basename "${posegraph_prefix}").yaml"
    require_file "${map_file}"
    launch_args+=("map_file:=${map_file}")
  fi
fi

exec ros2 launch robot_bringup "${operation}_bringup.launch.py" "${launch_args[@]}"
