#!/usr/bin/env bash
# Explicit entrypoints for the frozen V6 low-obstacle profile. Existing
# static/dynamic campaigns remain untouched and keep their original layouts.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

condition="static"
if [[ "${1:-}" == "--condition" ]]; then
  (($# >= 2)) || die \
    "usage: $0 [--condition static|dynamic|appearance] isaac|ros|shadow|ros-d|runner [mode] [arguments...]"
  condition="$2"
  shift 2
fi
case "${condition}" in
  static|dynamic|appearance) ;;
  *) die "condition must be static, dynamic, or appearance; got: ${condition}" ;;
esac

profile="${1:-}"
[[ -n "${profile}" ]] || die \
  "usage: $0 [--condition static|dynamic|appearance] isaac|ros|shadow|ros-d|runner [mode] [arguments...]"
shift

case "${condition}" in
  static)
    scenario_file="${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/v6_final_kujiale_static.yaml"
    ;;
  dynamic)
    scenario_file="${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/v6_final_kujiale_dynamic.yaml"
    ;;
  appearance)
    scenario_file="${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/v6_final_kujiale_appearance.yaml"
    ;;
esac
nav2_config_file="${PROJECT_ROOT}/ros2_ws/src/robot_navigation/config/nav2_v6_low_obstacle_isolation.yaml"

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

reject_phase_f_substrate_override() {
  local argument
  for argument in "$@"; do
    case "${argument}" in
      odometry_mode:=*|structure_tf_source:=*|localization_map_contract:=*|\
      localization_owner:=*|map_file:=*|spawn_poses_file:=*|\
      route_graph_file:=*|module1_amcl_prior_enabled:=*)
        die "Phase F fixes the Phase-B mixed/AMCL/GVG substrate; rejected override: ${argument}"
        ;;
    esac
  done
}

reject_isaac_condition_override() {
  local argument
  for argument in "$@"; do
    case "${argument}" in
      --dynamic-obstacle-config|--dynamic-obstacle-config=*|\
      --dynamic-obstacles|--dynamic-obstacles=*|\
      --no-dynamic-obstacles|--no-dynamic-obstacles=*|\
      --dynamic-case-id|--dynamic-case-id=*|\
      --dynamic-variant-id|--dynamic-variant-id=*|\
      --dynamic-seed|--dynamic-seed=*|\
      --appearance-config|--appearance-config=*|\
      --appearance-profile|--appearance-profile=*|\
      --environment-usd|--environment-usd=*|\
      --environment-root|--environment-root=*|\
      --spawn-poses-file|--spawn-poses-file=*|\
      --spawn-pose|--spawn-pose=*)
        die "V6 condition fixes Isaac scene/obstacle/appearance identity; rejected override: ${argument}"
        ;;
    esac
  done
}

reject_runner_condition_override() {
  local argument
  for argument in "$@"; do
    case "${argument}" in
      scenario_file:=*|spawn_poses_file:=*|nav2_profile:=*|nav2_config_file:=*|\
      dynamic_case_id:=*|dynamic_variant_id:=*|dynamic_seed:=*|robot_config_file:=*)
        die "V6 condition fixes runner scenario/spawn/Nav2 identity; rejected override: ${argument}"
        ;;
    esac
  done
}

run_ros_profile() {
  local graph_mode="$1"
  local activation_policy="$2"
  local initial_pose_source="$3"
  local default_cognitive_profile="$4"
  local substrate_odometry_mode="$5"
  local odometry_defaults="$6"
  shift 6
  local cognitive_profile="${V6_COGNITIVE_PROFILE:-${default_cognitive_profile}}"
  local -a localization_args=()
  local -a module1_args=()
  if [[ "${1:-}" =~ ^M[0-3]$ ]]; then
    cognitive_profile="$1"
    shift
  fi
  reject_phase_f_substrate_override "$@"
  if [[ "${substrate_odometry_mode}" == "mixed" ]]; then
    local argument
    for argument in "$@"; do
      case "${argument}" in
        spawn_pose_name:=*)
          die "Phase F fixes the Phase-B G1 spawn; rejected override: ${argument}"
          ;;
        cognitive_constraints_override_file:=*)
          die "Phase F fixes the canonical map context; rejected override: ${argument}"
          ;;
      esac
    done
  fi
  [[ "${cognitive_profile}" =~ ^M[0-3]$ ]] || die \
    "V6 cognitive profile must be M0, M1, M2, or M3; got: ${cognitive_profile}"
  if [[ "${odometry_defaults}" == "rf2o-shadow" ]]; then
    module1_args=(
      ekf_profile:=wheel_imu lidar_odometry_backend:=rf2o
      lidar_odometry_validated:=false
    )
  elif [[ "${odometry_defaults}" == "final" ]]; then
    module1_args=(
      ekf_profile:=wheel_imu
      imu_calibration_params_file:="${PROJECT_ROOT}/ros2_ws/src/robot_odometry/config/imu_calibration.yaml"
      lidar_odometry_backend:=off
      lidar_odometry_validated:=false
    )
  fi
  if [[ "${substrate_odometry_mode}" == "mixed" ]]; then
    local module2_root="${BIO_NAV_MODULE2_V310_ROOT:-}"
    local canonical_constraints_file
    [[ -n "${module2_root}" ]] || die \
      "BIO_NAV_MODULE2_V310_ROOT is required for the Phase-F map context"
    require_directory "${module2_root}"
    module2_root="$(cd "${module2_root}" && pwd -P)"
    canonical_constraints_file="${module2_root}/configs/kujiale_0026_module1_visual_shadow_v310.yaml"
    require_file "${canonical_constraints_file}"
    localization_args=(
      structure_tf_source:=isaac
      localization_map_contract:=occupancy_only
      localization_owner:=amcl
      "spawn_poses_file:=${PROJECT_ROOT}/isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml"
      spawn_pose_name:=long_route_start_g1
      "map_file:=${PROJECT_ROOT}/data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml"
      "route_graph_file:=${PROJECT_ROOT}/ros2_ws/src/robot_route_planner/config/v6_kujiale_isaacgen_v1_gvg_v1.geojson"
      "cognitive_constraints_override_file:=${canonical_constraints_file}"
    )
  else
    localization_args=(
      structure_tf_source:=isaac
      localization_map_contract:=posegraph_bundle
      localization_owner:=auto
      "spawn_poses_file:=${PROJECT_ROOT}/isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml"
      "posegraph_file:=${PROJECT_ROOT}/data/maps/posegraphs/v6_kujiale_isaacgen_v1"
      "map_file:=${PROJECT_ROOT}/data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml"
      "route_graph_file:=${PROJECT_ROOT}/ros2_ws/src/robot_route_planner/config/v6_kujiale_isaacgen_v1_gvg_v1.geojson"
    )
  fi
  export ISAAC_NAV_REQUIRE_V6_INTEGRATION=1
  # Phase F callers pass the exact Phase-B mixed substrate; the legacy shadow
  # entrypoint keeps its estimated RF2O/wheel+IMU substrate.
  exec "${SCRIPT_DIR}/run_ros.sh" navigation \
    "odometry_mode:=${substrate_odometry_mode}" \
    localization_profile:=kujiale \
    nav2_profile:=v6_low_obstacle_isolation \
    cognitive_profile:="${cognitive_profile}" \
    cognitive_graph_mode:="${graph_mode}" \
    initial_pose_source:="${initial_pose_source}" \
    activation_startup_policy:="${activation_policy}" \
    "${localization_args[@]}" \
    interactive:=false use_rviz:=false use_teleop:=false \
    "${module1_args[@]}" "$@"
}

case "${profile}" in
  isaac)
    # Reuse the Phase-B mixed Compute-Odom + AMCL launcher, then explicitly
    # enable the selected physical condition.  Both physical YAMLs are
    # default-off; --dynamic-obstacles is the explicit activation switch.
    reject_isaac_condition_override "$@"
    dynamic_obstacle_config="${PROJECT_ROOT}/isaac_sim/configs/experiments/v6_kujiale_low_obstacles_frozen.yaml"
    condition_args=(--appearance-profile baseline)
    if [[ "${condition}" == "dynamic" ]]; then
      dynamic_obstacle_config="${PROJECT_ROOT}/isaac_sim/configs/experiments/v6_single_dynamic_low_obstacle.yaml"
      condition_args=()
    elif [[ "${condition}" == "appearance" ]]; then
      condition_args=(
        --appearance-config
        "${PROJECT_ROOT}/isaac_sim/configs/experiments/kujiale_appearance_profiles.yaml"
        --appearance-profile baseline
      )
    fi
    exec "${SCRIPT_DIR}/run_v6_r5_phase_b_kujiale.sh" isaac \
      --dynamic-obstacle-config \
      "${dynamic_obstacle_config}" \
      --dynamic-obstacles \
      "${condition_args[@]}" \
      "$@"
    ;;
  ros)
    # C experiment: only M0--M3 changes; the physical graph stays GVG.
    reject_graph_override "$@"
    run_ros_profile gvg fail_closed auto M3 mixed final "$@"
    ;;
  shadow)
    # Reproducible zero-seed enrollment: retain the full localization stack,
    # keep the local Module2 arm shadow-only, and never activate Nav2 until a
    # valid RViz initial-pose seed satisfies the normal readiness contract.
    reject_graph_override "$@"
    run_ros_profile gvg wait_for_seed rviz M1 estimated rf2o-shadow "$@"
    ;;
  ros-d)
    graph_mode="${1:-}"
    [[ "${graph_mode}" =~ ^(shadow|hybrid|primary)$ ]] || die \
      "V6 D graph mode must be shadow, hybrid, or primary; got: ${graph_mode:-empty}"
    shift
    if [[ "${graph_mode}" == "primary" ]]; then
      reject_final_estimated_policy_override "$@"
    fi
    run_ros_profile "${graph_mode}" fail_closed auto M3 mixed final "$@"
    ;;
  runner)
    output_directory="${1:-${PROJECT_ROOT}/data/experiment_runs/v6_kujiale_low_obstacles}"
    [[ $# -eq 0 ]] || shift
    reject_runner_condition_override "$@"
    require_file "${scenario_file}"
    require_file "${nav2_config_file}"
    source_ros --require-workspace
    exec ros2 launch robot_experiments experiment.launch.py \
      scenario_file:="${scenario_file}" \
      spawn_poses_file:="${PROJECT_ROOT}/isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml" \
      output_directory:="${output_directory}" \
      nav2_profile:=v6_low_obstacle_isolation \
      nav2_config_file:="${nav2_config_file}" \
      "$@"
    ;;
  *) die "profile must be isaac, ros, shadow, ros-d, or runner; got: ${profile}" ;;
esac
