#!/usr/bin/env bash
# Paired final-runtime entrypoints for the three Rivermark scene classes.
# Isaac and ROS remain separate processes so a cold stage can be started once
# and inspected before the estimated-state navigation stack joins the graph.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

entrypoint="${1:-}"
scenario="${2:-}"
[[ -n "${entrypoint}" && -n "${scenario}" ]] || die \
  "usage: $0 isaac|ros static|dynamic|appearance [appearance-profile] [arguments...]"
shift 2

case "${entrypoint}" in
  isaac|ros) ;;
  *) die "entrypoint must be isaac or ros; got: ${entrypoint}" ;;
esac
case "${scenario}" in
  static|dynamic|appearance) ;;
  *) die "scenario must be static, dynamic, or appearance; got: ${scenario}" ;;
esac

appearance_profile="baseline"
if [[ "${scenario}" == "appearance" ]]; then
  appearance_profile="bright_warm"
  if [[ -n "${1:-}" && "${1}" != --* && "${1}" != *:=* ]]; then
    appearance_profile="$1"
    shift
  fi
  case "${appearance_profile}" in
    dim_warm|dim_cool|bright_warm|bright_cool) ;;
    *) die "unknown Rivermark appearance profile: ${appearance_profile}" ;;
  esac
fi

demo_dir="${RIVERMARK_DEMO_DIR:-${PROJECT_ROOT}/data/rivermark_demo}"
[[ -n "${RIVERMARK_USD:-}" ]] || die \
  "RIVERMARK_USD must name the frozen Rivermark USD"
environment_usd="${RIVERMARK_USD}"
[[ -r "${environment_usd}" ]] || die \
  "RIVERMARK_USD is not readable: ${environment_usd}"
spawn_poses_file="${demo_dir}/rivermark.spawn.yaml"
occupancy_map="${demo_dir}/rivermark_selected.yaml"
route_graph="${demo_dir}/rivermark_selected.geojson"
goals_file="${demo_dir}/rivermark_demo_goals.yaml"
static_config="${demo_dir}/final_rivermark_static_obstacles.yaml"
dynamic_config="${demo_dir}/final_rivermark_dynamic.yaml"
appearance_config="${demo_dir}/rivermark_appearance_profiles.yaml"

for required in \
  "${environment_usd}" \
  "${spawn_poses_file}" \
  "${occupancy_map}" \
  "${route_graph}" \
  "${goals_file}" \
  "${static_config}" \
  "${dynamic_config}" \
  "${appearance_config}"; do
  require_file "${required}"
done

export V6_RIVERMARK_SCENARIO="${scenario}"
export V6_RIVERMARK_GOALS_FILE="${goals_file}"
export V6_RIVERMARK_APPEARANCE_PROFILE="${appearance_profile}"

if [[ "${entrypoint}" == "isaac" ]]; then
  for argument in "$@"; do
    case "${argument}" in
      --environment-usd|--environment-usd=*|--spawn-poses-file|--spawn-poses-file=*|\
      --spawn-pose|--spawn-pose=*|--mode|--mode=*|\
      --structure-tf-source|--structure-tf-source=*|\
      --camera-profile|--camera-profile=*|\
      --dynamic-obstacle-config|--dynamic-obstacle-config=*|\
      --appearance-config|--appearance-config=*|\
      --appearance-profile|--appearance-profile=*)
        die "V6 Rivermark fixes the scene/estimated-state contract; rejected override: ${argument}"
        ;;
    esac
  done
  "${SCRIPT_DIR}/import_assets.sh"
  "${SCRIPT_DIR}/import_assets.sh" --check
  obstacle_config="${dynamic_config}"
  dynamic_args=(--no-dynamic-obstacles)
  if [[ "${scenario}" == "static" ]]; then
    obstacle_config="${static_config}"
    dynamic_args=(--dynamic-obstacles)
  elif [[ "${scenario}" == "dynamic" ]]; then
    dynamic_args=(
      --dynamic-obstacles
      --dynamic-case-id full_route_four_stage
      --dynamic-variant-id v3
    )
  fi
  export ISAAC_NAV__GROUND_TRUTH__ENABLED=true
  exec "${SCRIPT_DIR}/run_isaac.sh" \
    --environment-usd "${environment_usd}" \
    --spawn-poses-file "${spawn_poses_file}" \
    --spawn-pose rivermark_start \
    --navigation-mode localization \
    --mode realistic \
    --structure-tf-source isaac \
    --camera-profile rgbd_navigation \
    --dynamic-obstacle-config "${obstacle_config}" \
    "${dynamic_args[@]}" \
    --appearance-config "${appearance_config}" \
    --appearance-profile "${appearance_profile}" \
    "$@"
fi

for argument in "$@"; do
  case "${argument}" in
    odometry_mode:=*|structure_tf_source:=*|posegraph_file:=*|\
    localization_map_contract:=*|localization_owner:=*|\
    map_file:=*|route_graph_file:=*|localization_profile:=*|\
    ekf_profile:=*|imu_calibration_params_file:=*|\
    lidar_odometry_backend:=*|\
    lidar_odometry_validated:=*|nav2_profile:=*|\
    nav2_profile_params_file:=*|nav2_params_file:=*|\
    cognitive_profile:=*|cognitive_graph_mode:=*|route_prior_enabled:=*|\
    module2_enabled:=*|\
    interactive:=*|use_rviz:=*)
      die "V6 Rivermark fixes the estimated-navigation contract; rejected override: ${argument}"
      ;;
  esac
done

export ISAAC_NAV_REQUIRE_V6_INTEGRATION=1
export ISAAC_NAV_SPAWN_POSES="${spawn_poses_file}"
exec "${SCRIPT_DIR}/run_ros.sh" navigation \
  odometry_mode:=estimated \
  structure_tf_source:=isaac \
  localization_map_contract:=occupancy_only \
  localization_owner:=amcl \
  map_file:="${occupancy_map}" \
  route_graph_file:="${route_graph}" \
  localization_profile:=rivermark \
  ekf_profile:=wheel_imu \
  imu_calibration_params_file:="${PROJECT_ROOT}/ros2_ws/src/robot_odometry/config/imu_calibration.yaml" \
  lidar_odometry_backend:=off \
  lidar_odometry_validated:=false \
  nav2_profile:=stable \
  cognitive_profile:=M0 \
  cognitive_graph_mode:=gvg \
  route_prior_enabled:=false \
  module2_enabled:=false \
  spawn_poses_file:="${spawn_poses_file}" \
  spawn_pose_name:=rivermark_start \
  initial_pose_source:=auto \
  interactive:=false \
  use_rviz:=false \
  use_teleop:=false \
  use_self_filter:=true \
  "$@"
