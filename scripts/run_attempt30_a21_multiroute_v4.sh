#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# The desktop shell may already contain another Module3 checkout.  Clear only
# ROS/colcon discovery variables so this runner is identity-bound to the
# Attempt30 Integration underlay recorded by this worktree's install/setup.bash.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
export ISAAC_NAV_EXPECTED_DOMAIN_ID="${ISAAC_NAV_EXPECTED_DOMAIN_ID:-151}"
export ISAAC_NAV_RUNTIME_DIR="${ISAAC_NAV_RUNTIME_DIR:-/tmp/isaac_sim_ros2_nav_${UID}_a21_v310_q151}"
export ROS_DOMAIN_ID="${ISAAC_NAV_EXPECTED_DOMAIN_ID}"
# Evaluation-only map-frame Ground Truth is required by probe_closed_loop.
# It remains a publication side channel and is never connected to TF, Nav2,
# Route Server, Collision Monitor, or command generation.
export ISAAC_NAV__GROUND_TRUTH__ENABLED=true
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

mode="${1:-}"
query_id="${2:-}"
arm="${3:-${ATTEMPT30_A21_ARM:-}}"
[[ "${mode}" == "isaac" || "${mode}" == "ros" \
    || "${mode}" == "module2" || "${mode}" == "prior" ]] \
  || die "usage: $0 isaac|ros|module2|prior QUERY_ID [baseline|sr_only|dr_only|srdr]"
[[ -n "${query_id}" ]] || die "QUERY_ID is required"

if [[ -n "${arm}" ]]; then
  case "${arm}" in
    baseline)
      arm_module2=false
      arm_profile=structural
      ;;
    sr_only)
      arm_module2=true
      arm_profile=sr_medium
      ;;
    dr_only)
      arm_module2=true
      arm_profile=dr_medium
      ;;
    srdr)
      arm_module2=true
      arm_profile=medium
      ;;
    *) die "unknown experiment arm '${arm}'" ;;
  esac
else
  # Backward-compatible engineering mode used by the existing hand-run probes.
  arm_module2="${ATTEMPT30_A21_MODULE2_ENABLED:-false}"
  arm_profile="${ATTEMPT30_A21_GUIDANCE_PROFILE:-sr_medium}"
fi

integration_root="${BIO_NAV_ATTEMPT30_V310_INTEGRATION_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/integration/attempt30-a21-v310-srdr-rviz}"
integration_setup="${BIO_NAV_ATTEMPT30_V310_INTEGRATION_SETUP:-${integration_root}/install/setup.bash}"
if [[ ! -f "${integration_setup}" \
    && -f "${integration_root}/ros2_ws/install/setup.bash" ]]; then
  integration_setup="${integration_root}/ros2_ws/install/setup.bash"
fi
socket_path="${ISAAC_NAV_RUNTIME_DIR}/module2-v310.sock"

if [[ "${mode}" == "module2" ]]; then
  require_file "${integration_root}/scripts/run_module2_v310_server.sh"
  mkdir -p "${ISAAC_NAV_RUNTIME_DIR}"
  acquire_instance_lock module2_v310 "V3.10 Module2 server"
  rm -f "${socket_path}"
  export BIO_NAV_SOCKET_PATH="${socket_path}"
  exec "${integration_root}/scripts/run_module2_v310_server.sh"
fi

if [[ "${mode}" == "prior" ]]; then
  [[ "${arm}" != "baseline" ]] \
    || die "Baseline is SR=false, DR=false and must not launch the edge-prior bridge"
  require_file "${integration_setup}"
  require_file "${integration_root}/ros2_ws/src/bio_nav_ros_bridge/bio_nav_common/v310.py"
  export BIO_NAV_ATTEMPT30_V310_INTEGRATION_ROOT="${integration_root}"
  export BIO_NAV_ATTEMPT30_V310_INTEGRATION_SETUP="${integration_setup}"
  prior_arguments=(
    "socket_path:=${socket_path}" \
    use_sim_time:=true \
    "guidance_profile:=${arm_profile}" \
    "adaptation_method:=${ATTEMPT30_A21_ADAPTATION_METHOD:-cognitive}" \
    goal_prior_retry_window_s:=4.5
  )
  [[ -z "${ATTEMPT30_A21_AUDIT_JSONL_PATH:-}" ]] \
    || prior_arguments+=("audit_jsonl_path:=${ATTEMPT30_A21_AUDIT_JSONL_PATH}")
  [[ -z "${ATTEMPT30_A21_INCREMENTAL_PARENT_NPZ:-}" ]] \
    || prior_arguments+=("incremental_parent_npz:=${ATTEMPT30_A21_INCREMENTAL_PARENT_NPZ}")
  exec "${SCRIPT_DIR}/run_attempt30_a21_edge_prior.sh" "${prior_arguments[@]}"
fi

evidence_root="${ATTEMPT30_A21_V4_EVIDENCE_ROOT:-${PROJECT_ROOT}/../../integration/attempt30-a21-v310-srdr-rviz/docs/evidence/attempt30_a21_v310/multiroute_benchmark_v4}"
evidence_root="$(realpath -e "${evidence_root}")"
benchmark_stem="${ATTEMPT30_A21_BENCHMARK_STEM:-attempt30_a21_multiroute_v4}"
asset="${evidence_root}/${benchmark_stem}.usda"
map_file="${evidence_root}/${benchmark_stem}.yaml"
graph_file="${evidence_root}/${benchmark_stem}.geojson"
spawn_file="${evidence_root}/${benchmark_stem}.spawn.yaml"
candidates="${evidence_root}/${benchmark_stem}_execution_candidates.json"
for path in "${asset}" "${map_file}" "${graph_file}" "${spawn_file}" "${candidates}"; do
  require_file "${path}"
done

query_json="$({
  python3 - "${candidates}" "${query_id}" <<'PY'
import json, sys
records = json.load(open(sys.argv[1], encoding="utf-8"))
matches = [item for item in records if item["query_id"] == sys.argv[2]]
if len(matches) != 1:
    raise SystemExit(f"query {sys.argv[2]!r} not found exactly once")
print(json.dumps(matches[0]))
PY
} )"
spawn_name="$(python3 -c 'import json,sys; print(json.load(sys.stdin)["spawn_pose_name"])' <<<"${query_json}")"
map_x="$(python3 -c 'import json,sys; print(json.load(sys.stdin)["start_xy_yaw_deg"][0])' <<<"${query_json}")"
map_y="$(python3 -c 'import json,sys; print(json.load(sys.stdin)["start_xy_yaw_deg"][1])' <<<"${query_json}")"
map_yaw="$(python3 -c 'import json,sys; print(json.load(sys.stdin)["start_xy_yaw_deg"][2])' <<<"${query_json}")"

export ISAAC_NAV_SPAWN_POSES="${spawn_file}"

if [[ "${mode}" == "isaac" ]]; then
  isaac_arguments=(
    --environment-usd "${asset}"
    --spawn-poses-file "${spawn_file}"
    --spawn-pose "${spawn_name}"
    # All four causal arms use the identical RGB-D observation surface.  The
    # camera is required for a live Module2 PlanningPrior; Baseline simply
    # leaves the edge-prior consumer disabled.
    --camera-profile "${ATTEMPT30_A21_CAMERA_PROFILE:-rgbd_navigation}"
    --headless
  )
  if [[ -n "${ATTEMPT30_A21_DYNAMIC_CONFIG:-}" ]]; then
    dynamic_config="$(realpath -e "${ATTEMPT30_A21_DYNAMIC_CONFIG}")"
    isaac_arguments+=(--dynamic-obstacle-config "${dynamic_config}" --dynamic-obstacles)
    [[ -z "${ATTEMPT30_A21_DYNAMIC_CASE:-}" ]] \
      || isaac_arguments+=(--dynamic-case-id "${ATTEMPT30_A21_DYNAMIC_CASE}")
    [[ -z "${ATTEMPT30_A21_DYNAMIC_VARIANT:-}" ]] \
      || isaac_arguments+=(--dynamic-variant-id "${ATTEMPT30_A21_DYNAMIC_VARIANT}")
    [[ -z "${ATTEMPT30_A21_DYNAMIC_SEED:-}" ]] \
      || isaac_arguments+=(--dynamic-seed "${ATTEMPT30_A21_DYNAMIC_SEED}")
  else
    isaac_arguments+=(--no-dynamic-obstacles)
  fi
  exec "${SCRIPT_DIR}/run_isaac.sh" "${isaac_arguments[@]}"
fi

source_ros --require-workspace
# The generated Module3 setup records whichever Integration underlay was active
# at build time. Reassert the Attempt30 pair with local_setup files so the
# experiment cannot resolve engineering defaults or interfaces from main.
set +u
# shellcheck disable=SC1091
source "${integration_root}/install/local_setup.bash"
# shellcheck disable=SC1091
source "${PROJECT_ROOT}/ros2_ws/install/local_setup.bash"
set -u
exec ros2 launch robot_bringup multiroute_benchmark_navigation.launch.py \
  "map_file:=${map_file}" \
  "route_graph_file:=${graph_file}" \
  "spawn_poses_file:=${spawn_file}" \
  "spawn_pose_name:=${spawn_name}" \
  "map_to_odom_x:=${map_x}" \
  "map_to_odom_y:=${map_y}" \
  "map_to_odom_yaw_deg:=${map_yaw}" \
  "module2_enabled:=${arm_module2}"
