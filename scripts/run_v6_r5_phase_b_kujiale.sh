#!/usr/bin/env bash
# Start one component of the exact-scene R5 Phase B baseline in its own terminal.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<'USAGE'
usage: run_v6_r5_phase_b_kujiale.sh [--run-root PATH] [--domain ID]
       isaac|ros|module1-shadow|bridge|record|runner|manifest [arguments...]

Run isaac, ros, module1-shadow, bridge, record, then runner in separate
terminals. Module1 is observable shadow only; Nav2 remains M0/GVG and cannot
consume Module2 output.
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
readonly SOCKET_PATH="${BIO_NAV_PHASE_B_SOCKET_PATH:-${run_root}/runtime/module2.sock}"

require_file "${MANIFEST}"
require_file "${ORIGINAL_USD}"
require_file "${MAP}"
require_file "${SPAWN}"
require_file "${GVG}"
require_file "${SHADOW_CONFIG_ABS}"
mkdir -p "${run_root}/runtime" "${run_root}/episodes" "${run_root}/rosbag"

case "${component}" in
  isaac)
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
    exec ros2 launch bio_nav_ros_bridge v6_cognitive_navigation.launch.py \
      startup_profile:=estimated_shadow \
      "socket_path:=${SOCKET_PATH}" \
      use_sim_time:=true \
      "$@"
    ;;
  record)
    source_ros --require-integration-underlay
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
