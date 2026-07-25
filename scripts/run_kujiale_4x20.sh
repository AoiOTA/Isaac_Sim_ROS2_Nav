#!/usr/bin/env bash
# User-operated 4x20 campaign controller.  Isaac and Nav2 remain visible in
# their own terminals; this script owns only the formal runner/report process.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

usage() {
  cat <<'USAGE'
usage:
  run_kujiale_4x20.sh preflight static|dynamic
  run_kujiale_4x20.sh pilot static|dynamic CAMPAIGN_ID [--resume]
  run_kujiale_4x20.sh static-pair CAMPAIGN_ID [--resume]
  run_kujiale_4x20.sh dynamic-pair CAMPAIGN_ID [--resume]
  run_kujiale_4x20.sh status CAMPAIGN_ID
  run_kujiale_4x20.sh report CAMPAIGN_ID
USAGE
}

[[ $# -ge 1 ]] || { usage >&2; exit 2; }
command_name="$1"; shift

require_campaign_id() {
  local value="$1"
  [[ "${value}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || die "invalid CAMPAIGN_ID: ${value}"
}

parse_resume() {
  local value="${1:-}"
  [[ -z "${value}" || "${value}" == "--resume" ]] || die "expected optional --resume"
  [[ "${value}" == "--resume" ]] && printf 'true' || printf 'false'
}

preflight() {
  local mode="$1"
  local nav2_profile nav2_batch nav2_frequency environment_root scene_file
  [[ "${mode}" == "static" || "${mode}" == "dynamic" ]] || die "preflight mode must be static or dynamic"
  source_ros --require-workspace
  local available_gib
  available_gib="$(df -Pk "${PROJECT_ROOT}" | awk 'NR == 2 { print int($4 / 1024 / 1024) }')"
  [[ "${available_gib}" =~ ^[0-9]+$ && "${available_gib}" -ge 120 ]] \
    || die "4x20 campaign requires at least 120 GiB free; available=${available_gib:-unknown} GiB"
  require_file "${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/kujiale_4x20_${mode}_pair.yaml"
  require_file "${PROJECT_ROOT}/isaac_sim/configs/experiments/kujiale_appearance_profiles.yaml"
  require_file "${PROJECT_ROOT}/data/maps/occupancy/warehouse_new.yaml"
  require_file "${PROJECT_ROOT}/data/maps/occupancy/warehouse_new.pgm"
  require_file "${PROJECT_ROOT}/data/maps/posegraphs/warehouse_new.posegraph"
  require_file "${PROJECT_ROOT}/data/maps/posegraphs/warehouse_new.data"
  require_file "${PROJECT_ROOT}/isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.spawn.yaml"
  environment_root="${KUJIALE_ENVIRONMENT_ROOT:-/home/lyb/kujiale_usd_rooms_20260717}"
  require_directory "${environment_root}"
  mapfile -t scene_matches < <(rg --files "${environment_root}" -g 'kujiale_0026_A_to_B_door_open.usd')
  [[ "${#scene_matches[@]}" -eq 1 ]] || die "expected exactly one Kujiale scene USD below ${environment_root}; found ${#scene_matches[@]}"
  scene_file="${scene_matches[0]}"
  nav2_profile="stable"; nav2_batch="700"; nav2_frequency="10.0"
  if [[ "${mode}" == "dynamic" ]]; then
    nav2_profile="dynamic_avoidance"; nav2_batch="500"; nav2_frequency="15.0"
  fi
  require_file "${PROJECT_ROOT}/ros2_ws/src/robot_navigation/config/nav2_${nav2_profile}.yaml"
  if [[ "${mode}" == "dynamic" ]] && ! ros2 pkg prefix spatio_temporal_voxel_layer >/dev/null 2>&1; then
    die "dynamic stage requires STVL: sudo apt install ros-jazzy-spatio-temporal-voxel-layer"
  fi
  local topics
  topics="$(ros2 topic list 2>/dev/null || true)"
  for topic in /clock /ground_truth/odom /odom /camera/front/image_raw /camera/front/depth/points /experiment/appearance/state; do
    grep -qx "${topic}" <<<"${topics}" || die "required live topic is absent: ${topic}; start Isaac/Nav2 and wait for readiness"
  done
  ros2 param get /isaac_navigation_sim appearance_config_sha256 >/dev/null \
    || die "Isaac appearance contract is unavailable; start run_kujiale_4x20_isaac.sh and wait for ready log"
  ros2 param get /controller_server FollowPath.batch_size 2>/dev/null | grep -Eq "${nav2_batch}" \
    || die "Nav2 profile mismatch: expected ${nav2_profile} (FollowPath.batch_size=${nav2_batch})"
  ros2 param get /controller_server controller_frequency 2>/dev/null | grep -Eq "${nav2_frequency}" \
    || die "Nav2 profile mismatch: expected ${nav2_profile} (controller_frequency=${nav2_frequency})"
  timeout 8s bash -c 'ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -qm 1 "Translation"' \
    || die "map -> base_link TF is unavailable; wait for localization before starting the campaign"
  local map_hash scene_hash
  map_hash="$(sha256sum "${PROJECT_ROOT}/data/maps/occupancy/warehouse_new.yaml" | awk '{print $1}')"
  scene_hash="$(sha256sum "${scene_file}" | awk '{print $1}')"
  log_info "preflight passed for ${mode}; nav2=${nav2_profile}; free=${available_gib} GiB; map_sha256=${map_hash:0:12}; scene_sha256=${scene_hash:0:12}"
}

run_stage() {
  local mode="$1" output="$2" indices="$3" resume="$4"
  local scenario="${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/kujiale_4x20_${mode}_pair.yaml"
  local nav2_profile="stable"
  [[ "${mode}" == "dynamic" ]] && nav2_profile="dynamic_avoidance"
  [[ ! -e "${output}" || "${resume}" == true ]] || die "refusing to overwrite ${output}; use --resume only after an interrupted run"
  mkdir -p "${output}"
  local arguments=("${SCRIPT_DIR}/run_experiment.sh" "${scenario}" "${output}" "resume:=${resume}" "nav2_profile:=${nav2_profile}")
  [[ -n "${indices}" ]] && arguments+=("run_indices:=${indices}")
  "${arguments[@]}"
}

case "${command_name}" in
  preflight)
    [[ $# -eq 1 ]] || { usage >&2; exit 2; }
    preflight "$1"
    ;;
  pilot)
    [[ $# -ge 2 && $# -le 3 ]] || { usage >&2; exit 2; }
    mode="$1"; campaign_id="$2"; resume="$(parse_resume "${3:-}")"; require_campaign_id "${campaign_id}"
    [[ "${mode}" == "static" || "${mode}" == "dynamic" ]] || die "pilot mode must be static or dynamic"
    preflight "${mode}"
    # Matrix row 2 is the first non-baseline profile (dim_warm) for both modes.
    run_stage "${mode}" "${PROJECT_ROOT}/data/experiment_runs/kujiale_4x20_${campaign_id}/pilot-${mode}" "2" "${resume}"
    ;;
  static-pair|dynamic-pair)
    [[ $# -ge 1 && $# -le 2 ]] || { usage >&2; exit 2; }
    campaign_id="$1"; resume="$(parse_resume "${2:-}")"; require_campaign_id "${campaign_id}"
    mode="${command_name%-pair}"
    preflight "${mode}"
    run_stage "${mode}" "${PROJECT_ROOT}/data/experiment_runs/kujiale_4x20_${campaign_id}/${mode}" "" "${resume}"
    ;;
  status)
    [[ $# -eq 1 ]] || { usage >&2; exit 2; }
    campaign_id="$1"; require_campaign_id "${campaign_id}"; source_ros --require-workspace
    ros2 run robot_experiments kujiale_4x20_campaign --run-root "${PROJECT_ROOT}/data/experiment_runs/kujiale_4x20_${campaign_id}" --status
    ;;
  report)
    [[ $# -eq 1 ]] || { usage >&2; exit 2; }
    campaign_id="$1"; require_campaign_id "${campaign_id}"; source_ros --require-workspace
    run_root="${PROJECT_ROOT}/data/experiment_runs/kujiale_4x20_${campaign_id}"
    report_root="${PROJECT_ROOT}/data/reports/kujiale_4x20_${campaign_id}"
    [[ ! -e "${report_root}" ]] || die "refusing to overwrite existing report: ${report_root}"
    ros2 run robot_experiments kujiale_4x20_campaign --run-root "${run_root}" --output-directory "${report_root}"
    ;;
  *) usage >&2; exit 2 ;;
esac
