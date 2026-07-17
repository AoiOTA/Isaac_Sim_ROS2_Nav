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
if [[ "${operation}" == "localization" || "${operation}" == "navigation" ]]; then
  if runtime_lock_is_held teleop; then
    die "stop the Mapping teleop before starting ${operation}; it must not publish /cmd_vel in this mode"
  fi
fi
acquire_instance_lock ros "ROS stack"
export ISAAC_NAV_SPAWN_POSES="${ISAAC_NAV_SPAWN_POSES:-${PROJECT_ROOT}/isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.spawn.yaml}"

launch_args=("$@")
if [[ "${operation}" == "localization" || "${operation}" == "navigation" ]]; then
  # This custom-scene branch promotes the complete Kujiale map bundle while
  # the original warehouse branch keeps warehouse_v2 as its own default.
  default_map_version="warehouse_new"
  posegraph_file=""
  map_file=""
  odometry_mode="ideal"
  posegraph_calibration="false"
  for argument in "${launch_args[@]}"; do
    case "${argument}" in
      posegraph_file:=*) posegraph_file="${argument#posegraph_file:=}" ;;
      map_file:=*) map_file="${argument#map_file:=}" ;;
      odometry_mode:=*) odometry_mode="${argument#odometry_mode:=}" ;;
      posegraph_calibration:=*) posegraph_calibration="${argument#posegraph_calibration:=}" ;;
    esac
  done
  if [[ -z "${posegraph_file}" ]]; then
    if [[ -n "${map_file}" ]]; then
      map_prefix="${map_file%.yaml}"
      posegraph_file="${PROJECT_ROOT}/data/maps/posegraphs/$(basename "${map_prefix}")"
    else
      posegraph_file="${PROJECT_ROOT}/data/maps/posegraphs/${default_map_version}"
    fi
    require_file "${posegraph_file}.posegraph"
    require_file "${posegraph_file}.data"
    launch_args+=("posegraph_file:=${posegraph_file}")
  fi
  if [[ -z "${map_file}" ]]; then
    posegraph_prefix="${posegraph_file%.posegraph}"
    posegraph_prefix="${posegraph_prefix%.data}"
    map_file="${PROJECT_ROOT}/data/maps/occupancy/$(basename "${posegraph_prefix}").yaml"
    require_file "${map_file}"
    launch_args+=("map_file:=${map_file}")
  fi
  posegraph_version="$(basename "${posegraph_file%.posegraph}")"
  posegraph_version="${posegraph_version%.data}"
  if [[ "${posegraph_version}" == "warehouse_new" ]] \
      && [[ "${odometry_mode}" != "ideal" \
            || "${posegraph_calibration}" == "true" ]]; then
    die "warehouse_new is calibrated for normal Ideal localization/navigation only; rebuild it with scan matching before Realistic or Pose Graph localization"
  fi
fi

exec ros2 launch robot_bringup "${operation}_bringup.launch.py" "${launch_args[@]}"
