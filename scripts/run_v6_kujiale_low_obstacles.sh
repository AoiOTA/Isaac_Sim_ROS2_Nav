#!/usr/bin/env bash
# Retained path for the canonical V6-GRID Phase-1 empty-room entrypoints.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

profile="${1:-}"
[[ -n "${profile}" ]] || die \
  "usage: $0 isaac|ros|validate|session [arguments...]"
shift

manifest="${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/v6_final_kujiale_static.yaml"
require_file "${manifest}"
export ISAAC_NAV_REQUIRE_V6_INTEGRATION=1

reject_phase1_override() {
  local argument
  for argument in "$@"; do
    case "${argument}" in
      localization_owner:=*|nav2_profile:=*|cognitive_profile:=*|\
      module2_enabled:=*|cognitive_graph_mode:=*|ekf_profile:=*|\
      lidar_odometry_backend:=*|lidar_odometry_validated:=*)
        die "Phase 1 fixes grid+stable+M0+module2=false+gvg+RF2O-off; rejected override: ${argument}"
        ;;
    esac
  done
}

case "${profile}" in
  isaac)
    exec "${SCRIPT_DIR}/run_kujiale_4x20_isaac.sh" \
      v6-phase1-empty-room "$@"
    ;;
  ros)
    reject_phase1_override "$@"
    source_ros --require-workspace
    exec ros2 launch robot_bringup ros_stack.launch.py \
      operation:=navigation \
      odometry_mode:=estimated \
      structure_tf_source:=isaac \
      localization_map_contract:=occupancy_only \
      localization_profile:=kujiale \
      localization_owner:=grid \
      nav2_profile:=stable \
      cognitive_profile:=M0 \
      module2_enabled:=false \
      cognitive_graph_mode:=gvg \
      activation_startup_policy:=fail_closed \
      ekf_profile:=wheel_imu \
      "imu_calibration_params_file:=${PROJECT_ROOT}/ros2_ws/src/robot_odometry/config/imu_calibration.yaml" \
      lidar_odometry_backend:=off \
      lidar_odometry_validated:=false \
      spawn_pose_name:=long_route_start_g1 \
      "spawn_poses_file:=${PROJECT_ROOT}/isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml" \
      "map_file:=${PROJECT_ROOT}/data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml" \
      "route_graph_file:=${PROJECT_ROOT}/ros2_ws/src/robot_route_planner/config/v6_kujiale_isaacgen_v1_gvg_v1.geojson" \
      interactive:=false \
      use_rviz:=false \
      use_teleop:=false \
      "project_root:=${PROJECT_ROOT}" \
      "$@"
    ;;
  validate)
    source_ros --require-workspace
    exec ros2 run robot_experiments v6_formal_episode \
      --manifest "${manifest}" --pilot
    ;;
  session)
    [[ $# -eq 2 ]] || die \
      "usage: $0 session NAS_RUN_DIR SNAPSHOT_ROOT"
    exec "${SCRIPT_DIR}/v6_reset_cold_boundary_r5_session.sh" "$1" "$2"
    ;;
  *) die "profile must be isaac, ros, validate, or session; got: ${profile}" ;;
esac
