#!/usr/bin/env bash
# Explicit entrypoints for the frozen V6 low-obstacle profile. Existing
# static/dynamic campaigns remain untouched and keep their original layouts.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

profile="${1:-}"
[[ -n "${profile}" ]] || die \
  "usage: $0 isaac|ros|shadow|ros-d|runner [mode] [arguments...]"
shift

scenario_file="${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/v6_kujiale_low_obstacles_static.yaml"
require_file "${scenario_file}"

reject_graph_override() {
  local argument
  for argument in "$@"; do
    [[ "${argument}" != cognitive_graph_mode:=* ]] || die \
      "C/shadow entrypoints fix cognitive_graph_mode=gvg; use ros-d for D experiments"
  done
}

reject_final_estimated_policy_override() {
  local argument
  for argument in "$@"; do
    case "${argument}" in
      ekf_profile:=*|imu_calibration_params_file:=*|\
      lidar_odometry_backend:=*|lidar_odometry_validated:=*)
        die "V6 PRIMARY fixes wheel+calibrated-IMU with RF2O off; rejected override: ${argument}"
        ;;
    esac
  done
}

run_ros_profile() {
  local graph_mode="$1"
  local activation_policy="$2"
  local initial_pose_source="$3"
  local default_cognitive_profile="$4"
  local odometry_defaults="$5"
  shift 5
  local cognitive_profile="${V6_COGNITIVE_PROFILE:-${default_cognitive_profile}}"
  local -a odometry_args=()
  if [[ "${1:-}" =~ ^M[0-3]$ ]]; then
    cognitive_profile="$1"
    shift
  fi
  [[ "${cognitive_profile}" =~ ^M[0-3]$ ]] || die \
    "V6 cognitive profile must be M0, M1, M2, or M3; got: ${cognitive_profile}"
  if [[ "${odometry_defaults}" == "rf2o-shadow" ]]; then
    odometry_args=(
      ekf_profile:=wheel_imu
      lidar_odometry_backend:=rf2o
      lidar_odometry_validated:=false
    )
  elif [[ "${odometry_defaults}" == "final" ]]; then
    odometry_args=(
      ekf_profile:=wheel_imu
      imu_calibration_params_file:="${PROJECT_ROOT}/ros2_ws/src/robot_odometry/config/imu_calibration.yaml"
      lidar_odometry_backend:=off
      lidar_odometry_validated:=false
    )
  fi
  export ISAAC_NAV_REQUIRE_V6_INTEGRATION=1
  # V6 chain pins the Isaac-regenerated map bundle explicitly; run_ros.sh keeps
  # its warehouse_new default for the historical flows.
  exec "${SCRIPT_DIR}/run_ros.sh" navigation \
    odometry_mode:=estimated localization_profile:=kujiale \
    nav2_profile:=v6_low_obstacle_isolation \
    cognitive_profile:="${cognitive_profile}" \
    cognitive_graph_mode:="${graph_mode}" \
    initial_pose_source:="${initial_pose_source}" \
    activation_startup_policy:="${activation_policy}" \
    "spawn_poses_file:=${PROJECT_ROOT}/isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml" \
    "posegraph_file:=${PROJECT_ROOT}/data/maps/posegraphs/v6_kujiale_isaacgen_v1" \
    "map_file:=${PROJECT_ROOT}/data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml" \
    "route_graph_file:=${PROJECT_ROOT}/ros2_ws/src/robot_route_planner/config/v6_kujiale_isaacgen_v1_gvg_v1.geojson" \
    "${odometry_args[@]}" "$@"
}

case "${profile}" in
  isaac)
    exec "${SCRIPT_DIR}/run_kujiale_4x20_isaac.sh" v6-low-obstacles "$@"
    ;;
  ros)
    # C experiment: only M0--M3 changes; the physical graph stays GVG.
    reject_graph_override "$@"
    run_ros_profile gvg fail_closed auto M3 final "$@"
    ;;
  shadow)
    # Reproducible zero-seed enrollment: retain the full localization stack,
    # keep the local Module2 arm shadow-only, and never activate Nav2 until a
    # valid RViz initial-pose seed satisfies the normal readiness contract.
    reject_graph_override "$@"
    run_ros_profile gvg wait_for_seed rviz M1 rf2o-shadow "$@"
    ;;
  ros-d)
    graph_mode="${1:-}"
    [[ "${graph_mode}" =~ ^(shadow|hybrid|primary)$ ]] || die \
      "V6 D graph mode must be shadow, hybrid, or primary; got: ${graph_mode:-empty}"
    shift
    if [[ "${graph_mode}" == "primary" ]]; then
      reject_final_estimated_policy_override "$@"
    fi
    run_ros_profile "${graph_mode}" fail_closed auto M3 final "$@"
    ;;
  runner)
    output_directory="${1:-${PROJECT_ROOT}/data/experiment_runs/v6_kujiale_low_obstacles}"
    [[ $# -eq 0 ]] || shift
    source_ros --require-workspace
    exec ros2 launch robot_experiments experiment.launch.py \
      scenario_file:="${scenario_file}" \
      output_directory:="${output_directory}" "$@"
    ;;
  *) die "profile must be isaac, ros, shadow, ros-d, or runner; got: ${profile}" ;;
esac
