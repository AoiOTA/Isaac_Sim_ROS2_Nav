#!/usr/bin/env bash
# Paired final-runtime entrypoints for the three Rivermark scene classes.
# Isaac and ROS remain separate processes so a cold stage can be started once
# and inspected before the mixed-odometry navigation stack joins the graph.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

entrypoint="${1:-}"
scenario="${2:-}"
[[ -n "${entrypoint}" && -n "${scenario}" ]] || die \
  "usage: $0 isaac|ros static|dynamic|appearance [appearance-profile] [arguments...]"
shift 2

viewport_arm="A"
viewport_arm_selected=false
viewport_runtime_attestation=""
viewport_winner_manifest=""
viewport_run_root=""
forwarded_arguments=()
while (( $# )); do
  case "$1" in
    --viewport-arm)
      (( $# >= 2 )) || die "--viewport-arm requires A or B"
      viewport_arm="$2"
      viewport_arm_selected=true
      shift 2
      ;;
    --viewport-arm=*)
      die "use --viewport-arm A or --viewport-arm B"
      ;;
    --viewport-runtime-attestation)
      (( $# >= 2 )) || die "--viewport-runtime-attestation requires a path"
      viewport_runtime_attestation="$2"
      shift 2
      ;;
    --viewport-winner-manifest)
      (( $# >= 2 )) || die "--viewport-winner-manifest requires a path"
      viewport_winner_manifest="$2"
      shift 2
      ;;
    --viewport-run-root)
      (( $# >= 2 )) || die "--viewport-run-root requires a path"
      viewport_run_root="$2"
      shift 2
      ;;
    *)
      forwarded_arguments+=("$1")
      shift
      ;;
  esac
done
set -- "${forwarded_arguments[@]}"

case "${viewport_arm}" in
  A|B) ;;
  *) die "--viewport-arm must be A or B; got: ${viewport_arm}" ;;
esac
if [[ "${entrypoint}" != "isaac" && "${viewport_arm_selected}" == true ]]; then
  die "--viewport-arm is valid only for the Isaac entrypoint"
fi
if [[ "${entrypoint}" != "isaac" && -n "${viewport_runtime_attestation}${viewport_winner_manifest}${viewport_run_root}" ]]; then
  die "viewport runtime attestation options are valid only for the Isaac entrypoint"
fi

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
region_config_file="${demo_dir}/rivermark_regions.yaml"
goals_file="${demo_dir}/rivermark_demo_goals.yaml"
static_config="${demo_dir}/final_rivermark_static_obstacles.yaml"
dynamic_config="${demo_dir}/final_rivermark_dynamic.yaml"
appearance_config="${demo_dir}/rivermark_appearance_profiles.yaml"

for required in \
  "${environment_usd}" \
  "${spawn_poses_file}" \
  "${occupancy_map}" \
  "${route_graph}" \
  "${region_config_file}" \
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
  effective_headless=false
  for argument in "$@"; do
    case "${argument}" in
      --headless) effective_headless=true ;;
      --no-headless) effective_headless=false ;;
    esac
    case "${argument}" in
      --environment-usd|--environment-usd=*|--spawn-poses-file|--spawn-poses-file=*|\
      --spawn-pose|--spawn-pose=*|--mode|--mode=*|\
      --structure-tf-source|--structure-tf-source=*|\
      --localization-owner|--localization-owner=*|\
      --camera-profile|--camera-profile=*|\
      --rtx-descriptor*|\
      --disable-dlss|--disable-dlss=*|\
      --no-disable-dlss|--no-disable-dlss=*|\
      --disable-viewport-updates|--disable-viewport-updates=*|\
      --no-disable-viewport-updates|--no-disable-viewport-updates=*|\
      --dynamic-obstacle-config|--dynamic-obstacle-config=*|\
      --appearance-config|--appearance-config=*|\
      --appearance-profile|--appearance-profile=*)
        die "V6 Rivermark fixes the scene/runtime contract; rejected override: ${argument}"
        ;;
    esac
  done
  viewport_arguments=(--no-disable-viewport-updates)
  if [[ "${viewport_arm}" == "B" ]]; then
    [[ "${effective_headless}" == true ]] || die \
      "--viewport-arm B requires the effective Isaac mode to be --headless"
    [[ "${viewport_runtime_attestation}" == /* \
        && "${viewport_winner_manifest}" == /* \
        && "${viewport_run_root}" == /* ]] || die \
      "--viewport-arm B requires absolute runtime-attestation, winner-manifest, and run-root paths"
    [[ -d "${viewport_run_root}" ]] || die \
      "viewport run root does not exist: ${viewport_run_root}"
    viewport_run_root="$(cd "${viewport_run_root}" && pwd -P)"
    viewport_runtime_attestation="$(readlink -m -- "${viewport_runtime_attestation}")"
    [[ "${viewport_runtime_attestation}" == \
        "${viewport_run_root}/viewport_runtime_attestation.json" ]] || die \
      "viewport runtime attestation must be RUN_ROOT/viewport_runtime_attestation.json"
    [[ ! -e "${viewport_runtime_attestation}" && ! -L "${viewport_runtime_attestation}" ]] || die \
      "viewport runtime attestation path already exists"
    require_file "${viewport_winner_manifest}"
    viewport_winner_manifest="$(readlink -f -- "${viewport_winner_manifest}")"
    viewport_winner_manifest_sha256="$(sha256sum "${viewport_winner_manifest}" | awk '{print $1}')"
    viewport_arguments=(
      --disable-viewport-updates
      --viewport-arm-identity B
      --viewport-runtime-attestation "${viewport_runtime_attestation}"
      --viewport-winner-manifest "${viewport_winner_manifest}"
      --viewport-winner-manifest-sha256 "${viewport_winner_manifest_sha256}"
      --viewport-run-root "${viewport_run_root}"
      --viewport-scene "rivermark:${scenario}"
      --viewport-launcher "$(readlink -f -- "$0")"
    )
  elif [[ -n "${viewport_runtime_attestation}${viewport_winner_manifest}${viewport_run_root}" ]]; then
    die "viewport runtime attestation options require --viewport-arm B"
  fi
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
      --dynamic-case-id crossing
      --dynamic-variant-id v3
    )
  else
    obstacle_config="${static_config}"
    dynamic_args=(--dynamic-obstacles)
  fi
  export ISAAC_NAV__GROUND_TRUTH__ENABLED=true
  exec "${SCRIPT_DIR}/run_isaac.sh" \
    --environment-usd "${environment_usd}" \
    --spawn-poses-file "${spawn_poses_file}" \
    --spawn-pose rivermark_start \
    --navigation-mode localization \
    --mode mixed \
    --structure-tf-source isaac \
    --localization-owner ideal \
    --camera-profile rgbd_navigation \
    --rtx-descriptor-sets 20000 \
    --disable-dlss \
    "${viewport_arguments[@]}" \
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
    region_config_file:=*|\
    initial_pose_source:=*|\
    activation_startup_timeout:=*|activation_startup_policy:=*|\
    interactive:=*|use_rviz:=*)
      die "V6 Rivermark fixes the navigation contract; rejected override: ${argument}"
      ;;
  esac
done

export ISAAC_NAV_REQUIRE_V6_INTEGRATION=1
export ISAAC_NAV_SPAWN_POSES="${spawn_poses_file}"
exec "${SCRIPT_DIR}/run_ros.sh" navigation \
  odometry_mode:=mixed \
  structure_tf_source:=isaac \
  localization_map_contract:=occupancy_only \
  localization_owner:=ideal \
  map_file:="${occupancy_map}" \
  route_graph_file:="${route_graph}" \
  localization_profile:=rivermark \
  ekf_profile:=wheel_imu \
  imu_calibration_params_file:="${PROJECT_ROOT}/ros2_ws/src/robot_odometry/config/imu_calibration.yaml" \
  lidar_odometry_backend:=off \
  lidar_odometry_validated:=false \
  nav2_profile:=v6_low_obstacle_isolation \
  cognitive_profile:=M3 \
  cognitive_graph_mode:=gvg \
  route_prior_enabled:=true \
  module2_enabled:=true \
  region_config_file:="${region_config_file}" \
  spawn_poses_file:="${spawn_poses_file}" \
  spawn_pose_name:=rivermark_start \
  initial_pose_source:=isaac \
  activation_startup_timeout:=240.0 \
  interactive:=false \
  use_rviz:=false \
  use_teleop:=false \
  use_self_filter:=true \
  "$@"
