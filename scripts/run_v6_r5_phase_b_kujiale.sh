#!/usr/bin/env bash
# Start one component of the exact-scene R5 Phase B baseline in its own terminal.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<'USAGE'
usage: run_v6_r5_phase_b_kujiale.sh [--run-root PATH] [--domain ID]
       ros|isaac|module1-shadow|bridge|record|runner|manifest [arguments...]

Run the components in separate terminals in this order:
  1. ros
  2. isaac (waits for /wheel_odometry/reset before starting; Isaac then
     bootstraps /clock and requires /set_pose in its bounded startup reset)
  3. module1-shadow
  4. bridge
  5. record
  6. runner

Module1 is observable shadow only; Nav2 remains M0/GVG and cannot consume
Module2 output. The manifest command validates the manifest without starting
ROS, Isaac, recording, or navigation dispatch.
USAGE
}

run_root="${BIO_NAV_PHASE_B_RUN_ROOT:-/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6r5_phase_b_kujiale_current}"
domain_id="${BIO_NAV_PHASE_B_DOMAIN_ID:-150}"
while (($# > 0)); do
  case "$1" in
    --run-root)
      (($# >= 2)) || { usage >&2; exit 2; }
      run_root="$2"
      shift 2
      ;;
    --domain)
      (($# >= 2)) || { usage >&2; exit 2; }
      domain_id="$2"
      shift 2
      ;;
    *) break ;;
  esac
done

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi
component="${1:-}"
[[ -n "${component}" ]] || { usage >&2; exit 2; }
shift
[[ "${domain_id}" =~ ^[0-9]+$ && "${domain_id}" -le 232 ]] || {
  echo "domain must be an integer in [0,232]" >&2
  exit 2
}

export ISAAC_NAV_EXPECTED_DOMAIN_ID="${domain_id}"
export ROS_DOMAIN_ID="${domain_id}"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

readonly INTEGRATION_ROOT="${BIO_NAV_INTEGRATION_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_integration}"
readonly MODULE2_ROOT="${BIO_NAV_MODULE2_V310_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_module2}"
readonly MANIFEST="${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/v6_r5_phase_b_kujiale_exact_baseline.yaml"
readonly ORIGINAL_USD="/home/lyb/kujiale_usd_rooms_20260717/kujiale_0026/kujiale_0026_A_to_B_door_open.usd"
readonly MAP="${PROJECT_ROOT}/data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml"
readonly SPAWN="${PROJECT_ROOT}/isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml"
readonly GVG="${PROJECT_ROOT}/ros2_ws/src/robot_route_planner/config/v6_kujiale_isaacgen_v1_gvg_v1.geojson"
readonly SHADOW_CONFIG="${BIO_NAV_MODULE1_SHADOW_CONFIG:-configs/kujiale_0026_module1_visual_shadow_v310.yaml}"
readonly SHADOW_CONFIG_ABS="$([[ "${SHADOW_CONFIG}" = /* ]] && printf '%s' "${SHADOW_CONFIG}" || printf '%s/%s' "${MODULE2_ROOT}" "${SHADOW_CONFIG}")"

run_id="${BIO_NAV_PHASE_B_RUN_ID:-$(basename "${run_root%/}")}"
run_id="${run_id//[^A-Za-z0-9_.-]/_}"
[[ -n "${run_id}" ]] || die "Phase B run id must not be empty"
if [[ -n "${BIO_NAV_PHASE_B_SOCKET_PATH:-}" ]]; then
  socket_path="${BIO_NAV_PHASE_B_SOCKET_PATH}"
else
  if [[ -n "${XDG_RUNTIME_DIR:-}" ]]; then
    socket_runtime_root="${XDG_RUNTIME_DIR}/bio_nav_phase_b_${UID}"
  else
    socket_runtime_root="/tmp/bio_nav_phase_b_${UID}"
  fi
  socket_path="${socket_runtime_root}/domain_${domain_id}/${run_id}/module2.sock"
fi
[[ "${socket_path}" == /* ]] || die "Phase B socket path must be absolute: ${socket_path}"
readonly SOCKET_PATH="${socket_path}"

prepare_socket_directory() {
  local socket_directory
  socket_directory="$(dirname "${SOCKET_PATH}")"
  if [[ -L "${socket_directory}" ]]; then
    die "Phase B socket directory must not be a symlink: ${socket_directory}"
  fi
  mkdir -p -m 700 "${socket_directory}"
  [[ -d "${socket_directory}" && ! -L "${socket_directory}" ]] \
    || die "could not create a safe Phase B socket directory: ${socket_directory}"
}

wait_for_ros_reset_services() {
  local timeout_sec deadline service service_list
  local -a required_services missing
  timeout_sec="${BIO_NAV_PHASE_B_ROS_READY_TIMEOUT_SEC:-120}"
  [[ "${timeout_sec}" =~ ^[1-9][0-9]*$ ]] \
    || die "BIO_NAV_PHASE_B_ROS_READY_TIMEOUT_SEC must be a positive integer"
  # The mixed EKF waits for Isaac simulation time before advertising
  # /set_pose.  Pre-Isaac readiness therefore stops at the wheel reset
  # service; Isaac's bootstrap-clock startup reset performs the bounded,
  # required discovery of both wheel and EKF reset services.
  required_services=(/wheel_odometry/reset)
  deadline=$((SECONDS + timeout_sec))
  while true; do
    service_list="$(
      timeout 2 ros2 service list -t --no-daemon --spin-time 1 2>/dev/null \
        || true
    )"
    missing=()
    for service in "${required_services[@]}"; do
      awk -v service="${service}" \
        '$1 == service && $2 == "[std_srvs/srv/Empty]" && NF == 2 { found = 1 } END { exit !found }' \
        <<<"${service_list}" \
        || missing+=("${service}")
    done
    if ((${#missing[@]} == 0)); then
      log_info "Phase B pre-Isaac ROS reset service is ready; starting Isaac"
      return 0
    fi
    ((SECONDS < deadline)) || die \
      "Phase B ROS reset services not ready after ${timeout_sec}s: ${missing[*]}; start the ros component first"
    sleep 1
  done
}

require_file "${MANIFEST}"
require_file "${ORIGINAL_USD}"
require_file "${MAP}"
require_file "${SPAWN}"
require_file "${GVG}"
require_file "${SHADOW_CONFIG_ABS}"

case "${component}" in
  isaac)
    source_ros --require-integration-underlay
    wait_for_ros_reset_services
    export ISAAC_NAV__GROUND_TRUTH__ENABLED=true
    exec "${SCRIPT_DIR}/run_isaac.sh" \
      --environment-usd "${ORIGINAL_USD}" \
      --spawn-poses-file "${SPAWN}" \
      --spawn-pose long_route_start_g1 \
      --navigation-mode localization \
      --mode mixed \
      --camera-profile rgbd_navigation \
      --no-dynamic-obstacles \
      --appearance-profile baseline \
      "$@"
    ;;
  ros)
    export ISAAC_NAV_REQUIRE_V6_INTEGRATION=1
    exec "${SCRIPT_DIR}/run_ros.sh" navigation \
      odometry_mode:=mixed \
      structure_tf_source:=isaac \
      localization_map_contract:=occupancy_only \
      localization_owner:=amcl \
      "map_file:=${MAP}" \
      "spawn_poses_file:=${SPAWN}" \
      spawn_pose_name:=long_route_start_g1 \
      "route_graph_file:=${GVG}" \
      "cognitive_constraints_override_file:=${SHADOW_CONFIG_ABS}" \
      nav2_profile:=stable \
      cognitive_profile:=M0 \
      module2_enabled:=false \
      cognitive_graph_mode:=gvg \
      interactive:=false \
      use_rviz:=false \
      use_teleop:=false \
      "$@"
    ;;
  module1-shadow)
    prepare_socket_directory
    require_file "${INTEGRATION_ROOT}/scripts/run_module2_v310_server.sh"
    export BIO_NAV_MODULE2_V310_ROOT="${MODULE2_ROOT}"
    exec "${INTEGRATION_ROOT}/scripts/run_module2_v310_server.sh" \
      --shadow-config "${SHADOW_CONFIG}" \
      --socket "${SOCKET_PATH}" \
      --device "${BIO_NAV_PHASE_B_DEVICE:-cuda}" \
      "$@"
    ;;
  bridge)
    source_ros --require-integration-underlay
    prepare_socket_directory
    exec ros2 launch bio_nav_ros_bridge v6_cognitive_navigation.launch.py \
      startup_profile:=estimated_shadow \
      "socket_path:=${SOCKET_PATH}" \
      use_sim_time:=true \
      "$@"
    ;;
  record)
    source_ros --require-integration-underlay
    mkdir -p "${run_root}/rosbag"
    bag_path="${run_root}/rosbag/phase_b"
    [[ ! -e "${bag_path}" ]] || die "refusing to overwrite ${bag_path}"
    mapfile -t topics < <(
      python3 -m robot_experiments.phase_b_observability \
        --print-recorder-topics
    )
    exec ros2 bag record \
      --use-sim-time \
      --storage mcap \
      --storage-preset-profile zstd_fast \
      --output "${bag_path}" \
      "${topics[@]}" "$@"
    ;;
  runner)
    source_ros --require-integration-underlay
    mkdir -p "${run_root}/episodes"
    exec "${SCRIPT_DIR}/run_v6_formal_episode.sh" \
      --pilot --dispatch-pilot "${MANIFEST}" \
      --output-jsonl "${run_root}/episodes/phase_b.jsonl" \
      "$@"
    ;;
  manifest)
    source_ros --require-integration-underlay
    exec "${SCRIPT_DIR}/run_v6_formal_episode.sh" --pilot "${MANIFEST}" "$@"
    ;;
  *) usage >&2; die "unknown component: ${component}" ;;
esac
