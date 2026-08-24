#!/usr/bin/env bash
# V6 reset cold-boundary R5 — live multi-episode re-arm session driver.
#
# usage:
#   v6_reset_cold_boundary_r5_session.sh RUN_DIR SNAPSHOT_ROOT
#
# One persistent Kujiale empty-room stack (Isaac + ROS/Nav2 + the Integration
# Grid relocalization coordinator), then three consecutive v6_formal_episode
# engineering-pilot dispatches (episode indices 0/1/2 = seeds 7201/7202/7203
# of the Kujiale static manifest), each on the same warm stack (Option A
# in-place re-arm).  One session-long MCAP plus one runner JSONL and one
# boundary-ownership probe per episode are written under RUN_DIR.
#
# The driver only orchestrates existing locked entry points; it changes no
# runtime behavior.  Everything runs from the immutable SNAPSHOT_ROOT source
# archive with its isolated build/install trees.

set -Eeuo pipefail

RUN_DIR="${1:-}"
SNAP_INPUT="${2:-}"
if [[ -z "${RUN_DIR}" || -z "${SNAP_INPUT}" ]]; then
  echo "usage: $0 RUN_DIR SNAPSHOT_ROOT" >&2
  exit 64
fi
[[ -d "${SNAP_INPUT}" ]] || {
  echo "snapshot root not found: ${SNAP_INPUT}" >&2
  exit 66
}
SNAP="$(readlink -f -- "${SNAP_INPUT}")"
ASSET_ROOT_INPUT="${ISAAC_ASSET_ROOT:-}"
if [[ -z "${ASSET_ROOT_INPUT}" \
    || "${ISAAC_ASSET_ROOT_DEFAULTED:-0}" == 1 ]]; then
  echo "ISAAC_ASSET_ROOT must be set to an absolute authorized asset root" >&2
  exit 64
fi
[[ "${ASSET_ROOT_INPUT}" == /* ]] || {
  echo "ISAAC_ASSET_ROOT must be absolute: ${ASSET_ROOT_INPUT}" >&2
  exit 64
}
[[ -d "${ASSET_ROOT_INPUT}" ]] || {
  echo "ISAAC_ASSET_ROOT is not a directory: ${ASSET_ROOT_INPUT}" >&2
  exit 66
}
ISAAC_ASSET_ROOT="$(readlink -f -- "${ASSET_ROOT_INPUT}")"

DOMAIN_ID="${R5_DOMAIN_ID:-173}"
EPISODE_INDICES="${R5_EPISODE_INDICES:-0 1 2}"
EPISODE_SEEDS="${R5_EPISODE_SEEDS:-7201 7202 7203}"
READINESS_TIMEOUT="${R5_READINESS_TIMEOUT_SEC:-240}"
RESET_TIMEOUT="${R5_RESET_TIMEOUT_SEC:-240}"
NAVIGATION_TIMEOUT="${R5_NAVIGATION_TIMEOUT_SEC:-300}"
V6_COGNITIVE_PROFILE="${V6_COGNITIVE_PROFILE:-M0}"
V6_LOCALIZATION_BACKEND="${V6_LOCALIZATION_BACKEND:-grid}"
V6_NAV2_PROFILE="${V6_NAV2_PROFILE:-stable}"
V6_MODULE2_ENABLED="${V6_MODULE2_ENABLED:-false}"
V6_COGNITIVE_GRAPH_MODE="${V6_COGNITIVE_GRAPH_MODE:-gvg}"
V6_LOW_OBSTACLES_ENABLED="${V6_LOW_OBSTACLES_ENABLED:-false}"
V6_DYNAMIC_ACTORS_ENABLED="${V6_DYNAMIC_ACTORS_ENABLED:-false}"
V6_VISUAL_ODOMETRY_SHADOW_ENABLED="${V6_VISUAL_ODOMETRY_SHADOW_ENABLED:-false}"
M3="${SNAP}/m3_src"
I_SRC="${SNAP}/i_src"
M3_INSTALL="${M3}/ros2_ws/install_r5"
I_INSTALL="${I_SRC}/ros2_ws/install_r5"
I_SETUP="${I_INSTALL}/setup.bash"
M3_LOCAL_SETUP="${M3_INSTALL}/local_setup.bash"
I_OBSTACLE_HEADER="${I_INSTALL}/bio_nav_interfaces/include/bio_nav_interfaces/bio_nav_interfaces/msg/detail/cognitive_obstacle_array__struct.hpp"
ASSET_IMPORTER="${M3}/scripts/import_assets.sh"
ASSET_MANIFEST="${M3}/isaac_sim/assets/robots/jackal/asset_manifest.json"
KUJIALE_SOURCE_USD="/home/lyb/kujiale_usd_rooms_20260717/kujiale_0026/kujiale_0026_A_to_B_door_open.usd"
KUJIALE_ENVIRONMENT_ROOT="${M3}/isaac_sim/assets/environments/v6_kujiale_clearance_r2"
LOGS="${RUN_DIR}/logs"
PROV="${RUN_DIR}/provenance"
EPISODES_DIR="${RUN_DIR}/episodes"
LOCK_DIR="${ISAAC_NAV_RUNTIME_DIR:-/tmp/isaac_sim_ros2_nav_$(id -u)}"
MANIFEST="${M3}/ros2_ws/src/robot_experiments/config/v6_final_kujiale_static.yaml"

export ROS_DOMAIN_ID="${DOMAIN_ID}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ISAAC_NAV_EXPECTED_DOMAIN_ID="${DOMAIN_ID}"
export ROS_LOG_DIR="${SNAP}/ros_log"
export BIO_NAV_INTEGRATION_ROOT="${I_SRC}"
export BIO_NAV_INTEGRATION_INSTALL="${I_INSTALL}"
export BIO_NAV_INTEGRATION_SETUP="${I_SETUP}"
export ISAAC_NAV_WORKSPACE_SETUP="${M3_LOCAL_SETUP}"
export ISAAC_ASSET_ROOT
export KUJIALE_ENVIRONMENT_ROOT

for snapshot_file in \
    "${ASSET_IMPORTER}" \
    "${ASSET_MANIFEST}" \
    "${I_SETUP}" \
    "${I_OBSTACLE_HEADER}" \
    "${M3_INSTALL}/setup.bash" \
    "${M3_LOCAL_SETUP}" \
    "${KUJIALE_SOURCE_USD}" \
    "${KUJIALE_ENVIRONMENT_ROOT}/kujiale_0026_A_to_B_door_open.usd"; do
  [[ -f "${snapshot_file}" ]] || {
    echo "required snapshot file not found: ${snapshot_file}" >&2
    exit 66
  }
done

[[ "${RUN_DIR}" == /mnt/nas_home/Bio_Nav_Data/* ]] || {
  echo "RUN_DIR must be under /mnt/nas_home/Bio_Nav_Data" >&2
  exit 64
}

mkdir -p "${LOGS}" "${PROV}" "${EPISODES_DIR}"
[[ "${V6_COGNITIVE_PROFILE}" == M0 ]] || {
  echo "Phase 1 requires V6_COGNITIVE_PROFILE=M0" >&2
  exit 64
}
[[ "${V6_LOCALIZATION_BACKEND}" == grid ]] || {
  echo "Phase 1 requires V6_LOCALIZATION_BACKEND=grid" >&2
  exit 64
}
[[ "${V6_NAV2_PROFILE}" == stable ]] || {
  echo "Phase 1 requires V6_NAV2_PROFILE=stable" >&2
  exit 64
}
[[ "${V6_MODULE2_ENABLED}" == false \
  && "${V6_COGNITIVE_GRAPH_MODE}" == gvg \
  && "${V6_LOW_OBSTACLES_ENABLED}" == false \
  && "${V6_DYNAMIC_ACTORS_ENABLED}" == false ]] || {
  echo "Phase 1 requires module2=false, gvg, low-obstacles=false, dynamic-actors=false" >&2
  exit 64
}
[[ "${V6_VISUAL_ODOMETRY_SHADOW_ENABLED}" == false \
  || "${V6_VISUAL_ODOMETRY_SHADOW_ENABLED}" == true ]] || {
  echo "V6_VISUAL_ODOMETRY_SHADOW_ENABLED must be true or false" >&2
  exit 64
}

if [[ -e "${RUN_DIR}/STOP.md" || -e "${RUN_DIR}/driver_summary.json" ]]; then
  echo "run directory already contains a terminal artifact; refusing to reuse it" >&2
  exit 65
fi

declare -A CHILD_PGIDS=()
STAGE="init"

_log_stage() {
  printf '{"stage":"%s","at":"%s","detail":"%s"}\n' \
    "$1" "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "${2:-}" >> "${PROV}/stages.jsonl"
}

_cleanup_children() {
  local name pid
  ((${#CHILD_PGIDS[@]})) || return 0
  for name in "${!CHILD_PGIDS[@]}"; do
    pid="${CHILD_PGIDS[$name]}"
    kill -INT -- "-${pid}" 2>/dev/null || true
  done
  sleep 8
  for name in "${!CHILD_PGIDS[@]}"; do
    pid="${CHILD_PGIDS[$name]}"
    kill -TERM -- "-${pid}" 2>/dev/null || true
  done
  sleep 4
  for name in "${!CHILD_PGIDS[@]}"; do
    pid="${CHILD_PGIDS[$name]}"
    kill -KILL -- "-${pid}" 2>/dev/null || true
  done
}

_stop() {
  local reason="$1"
  _log_stage "STOP" "${reason}"
  _cleanup_children
  {
    echo "# V6 reset cold-boundary R5 — STOP"
    echo
    echo "- run_dir: ${RUN_DIR}"
    echo "- domain: ${DOMAIN_ID}"
    echo "- stage: ${STAGE}"
    echo "- reason: ${reason}"
    echo "- isaac_asset_root: ${ISAAC_ASSET_ROOT}"
    echo "- kujiale_source_usd: ${KUJIALE_SOURCE_USD}"
    echo "- kujiale_environment_root: ${KUJIALE_ENVIRONMENT_ROOT}"
    echo "- layout_id: v6_kujiale_clearance_r2"
    echo "- asset_materialization_status: ${ASSET_MATERIALIZATION_STATUS:-not_started}"
    echo "- stopped_at: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  } > "${RUN_DIR}/STOP.md"
  exit 2
}

_on_err() {
  _stop "driver error at line $1 during stage ${STAGE}; see ${LOGS}"
}
trap '_on_err ${LINENO}' ERR

_write_run_metadata() {
  cat > "${RUN_DIR}/run.yaml" <<EOF
snapshot_root: ${SNAP}
isaac_asset_root: ${ISAAC_ASSET_ROOT}
kujiale_source_usd: ${KUJIALE_SOURCE_USD}
kujiale_environment_root: ${KUJIALE_ENVIRONMENT_ROOT}
layout_id: v6_kujiale_clearance_r2
asset_materialization_status: ${ASSET_MATERIALIZATION_STATUS}
asset_manifest: isaac_sim/assets/robots/jackal/asset_manifest.json
git_contains_runtime_asset_binaries: false
EOF
}

start_bg() { # name command...
  local name="$1"; shift
  setsid "$@" > "${LOGS}/${name}.log" 2>&1 &
  local pid=$!
  CHILD_PGIDS[$name]="${pid}"
  echo "${pid}" > "${PROV}/${name}.pid"
  _log_stage "start" "${name} pgid=${pid}"
}

stop_bg() { # name
  local name="$1"
  local pid="${CHILD_PGIDS[$name]:-}"
  [[ -n "${pid}" ]] || return 0
  kill -INT -- "-${pid}" 2>/dev/null || true
  local waited=0
  while kill -0 "${pid}" 2>/dev/null && (( waited < 90 )); do
    sleep 2; waited=$((waited + 2))
  done
  kill -TERM -- "-${pid}" 2>/dev/null || true
  sleep 3
  kill -KILL -- "-${pid}" 2>/dev/null || true
  unset 'CHILD_PGIDS[$name]'
  _log_stage "stop" "${name}"
}

assert_alive() { # name
  local pid="${CHILD_PGIDS[$1]:-}"
  [[ -n "${pid}" ]] || _stop "$1 was never started"
  if ! kill -0 "${pid}" 2>/dev/null; then
    _stop "$1 exited unexpectedly; see ${LOGS}/$1.log"
  fi
}

wait_service() { # service timeout_sec
  local deadline=$((SECONDS + $2))
  while (( SECONDS < deadline )); do
    if ros2 service list 2>/dev/null | grep -qx "$1"; then return 0; fi
    sleep 4
  done
  return 1
}

wait_topic() { # topic timeout_sec
  local deadline=$((SECONDS + $2))
  while (( SECONDS < deadline )); do
    if timeout 10 ros2 topic echo "$1" --once --qos-reliability best_effort \
        >/dev/null 2>&1; then
      return 0
    fi
    sleep 4
  done
  return 1
}

wait_file() { # path timeout_sec
  local deadline=$((SECONDS + $2))
  while (( SECONDS < deadline )); do
    [[ -e "$1" ]] && return 0
    sleep 3
  done
  return 1
}

# ---------------------------------------------------- snapshot asset closure
STAGE="asset_materialization"
ASSET_MATERIALIZATION_STATUS="pending"
export ASSET_MATERIALIZATION_STATUS
_write_run_metadata
{
  echo "isaac_asset_root=${ISAAC_ASSET_ROOT}"
  echo "kujiale_source_usd=${KUJIALE_SOURCE_USD}"
  echo "kujiale_environment_root=${KUJIALE_ENVIRONMENT_ROOT}"
  echo "layout_id=v6_kujiale_clearance_r2"
  echo "asset_manifest=${ASSET_MANIFEST}"
  echo "status=importing"
} > "${LOGS}/asset_materialization.log"
if ! (cd "${M3}" && "${ASSET_IMPORTER}") \
    >> "${LOGS}/asset_materialization.log" 2>&1; then
  ASSET_MATERIALIZATION_STATUS="import_failed"
  export ASSET_MATERIALIZATION_STATUS
  _write_run_metadata
  _stop "Jackal asset import failed; see ${LOGS}/asset_materialization.log"
fi
if ! (cd "${M3}" && "${ASSET_IMPORTER}" --check) \
    >> "${LOGS}/asset_materialization.log" 2>&1; then
  ASSET_MATERIALIZATION_STATUS="check_failed"
  export ASSET_MATERIALIZATION_STATUS
  _write_run_metadata
  _stop "Jackal asset check failed; see ${LOGS}/asset_materialization.log"
fi
ASSET_MATERIALIZATION_STATUS="verified"
export ASSET_MATERIALIZATION_STATUS
echo "status=${ASSET_MATERIALIZATION_STATUS}" \
  >> "${LOGS}/asset_materialization.log"
_write_run_metadata
_log_stage "asset_materialization" "verified in snapshot"

# ---------------------------------------------------------------- env setup
STAGE="env"
# Drop any pre-existing ROS overlay from the caller environment: the session
# must resolve packages only from this snapshot (mirrors common.sh
# reset_ros_overlay_environment, plus Python/library path pollution).
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
unset PYTHONPATH LD_LIBRARY_PATH
# ROS-generated setup scripts reference optional variables before defining
# them, so nounset must be suspended while they are sourced.
set +u
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1090
source "${I_INSTALL}/setup.bash"
# shellcheck disable=SC1090
source "${M3_INSTALL}/setup.bash"
set -u

ROBOT_EXPERIMENTS_PREFIX="$(ros2 pkg prefix robot_experiments)"
echo "${ROBOT_EXPERIMENTS_PREFIX}" > "${PROV}/robot_experiments_prefix.txt"
[[ "${ROBOT_EXPERIMENTS_PREFIX}" == "${M3_INSTALL}/robot_experiments" ]] || \
  _stop "robot_experiments resolves outside the snapshot install: ${ROBOT_EXPERIMENTS_PREFIX}"

{
  echo "run_dir=${RUN_DIR}"
  echo "snapshot=${SNAP}"
  echo "integration_root=${BIO_NAV_INTEGRATION_ROOT}"
  echo "integration_install=${BIO_NAV_INTEGRATION_INSTALL}"
  echo "integration_setup=${BIO_NAV_INTEGRATION_SETUP}"
  echo "module3_workspace_setup=${ISAAC_NAV_WORKSPACE_SETUP}"
  echo "isaac_asset_root=${ISAAC_ASSET_ROOT}"
  echo "asset_materialization_status=${ASSET_MATERIALIZATION_STATUS}"
  echo "git_contains_runtime_asset_binaries=false"
  echo "domain_id=${DOMAIN_ID}"
  echo "rmw=${RMW_IMPLEMENTATION}"
  echo "episode_indices=${EPISODE_INDICES}"
  echo "episode_seeds=${EPISODE_SEEDS}"
  echo "manifest=${MANIFEST}"
  echo "yaw_scale=0.9294"
  echo "rf2o=off"
  echo "ekf_profile=wheel_imu"
  echo "lidar_odometry_backend=off"
  echo "cognitive_profile=${V6_COGNITIVE_PROFILE}"
  echo "localization_backend=${V6_LOCALIZATION_BACKEND}"
  echo "nav2_profile=${V6_NAV2_PROFILE}"
  echo "module2_enabled=${V6_MODULE2_ENABLED}"
  echo "cognitive_graph_mode=${V6_COGNITIVE_GRAPH_MODE}"
  echo "low_obstacles_enabled=${V6_LOW_OBSTACLES_ENABLED}"
  echo "dynamic_actors_enabled=${V6_DYNAMIC_ACTORS_ENABLED}"
  echo "visual_odometry_shadow_enabled=${V6_VISUAL_ODOMETRY_SHADOW_ENABLED}"
  echo "mission=G1->G2->G3->G4->G5->G1"
  if [[ -f "${SNAP}/SNAPSHOT_SHAS.txt" ]]; then
    cat "${SNAP}/SNAPSHOT_SHAS.txt"
  fi
} > "${PROV}/pins_and_run_contract.txt"

if [[ -e "${LOCK_DIR}/isaac.lock" ]]; then
  if ! flock -n "${LOCK_DIR}/isaac.lock" -c true 2>/dev/null; then
    _stop "Isaac instance lock is held by another process"
  fi
fi

QOS_FILE="${PROV}/rosbag_qos_overrides.yaml"
cat > "${QOS_FILE}" <<'QOS'
/clock:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 100
/imu/data_raw:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 100
/imu/data:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 100
/ground_truth/odom:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 100
/odom:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 100
/wheel/odom:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 100
/camera/front/image_raw:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 10
/camera/front/camera_info:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 10
/camera/front/depth/image_raw:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 10
/scan:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 50
/scan_safety:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 50
QOS

# ---------------------------------------------------------------- isaac
STAGE="isaac"
start_bg isaac "${M3}/scripts/run_kujiale_4x20_isaac.sh" \
  v6-phase1-empty-room --headless
wait_topic /clock 600 || _stop "Kujiale /clock did not appear; see logs/isaac.log"
assert_alive isaac
wait_service /simulation/reset 300 || _stop "reset service did not appear"
assert_alive isaac

# ---------------------------------------------------------------- navigation
STAGE="navigation_stack"
start_bg navigation ros2 launch robot_bringup ros_stack.launch.py \
  operation:=navigation \
  odometry_mode:=estimated \
  structure_tf_source:=isaac \
  localization_map_contract:=occupancy_only \
  localization_profile:=kujiale \
  localization_owner:=${V6_LOCALIZATION_BACKEND} \
  nav2_profile:=${V6_NAV2_PROFILE} \
  cognitive_profile:=${V6_COGNITIVE_PROFILE} \
  module2_enabled:=${V6_MODULE2_ENABLED} \
  cognitive_graph_mode:=${V6_COGNITIVE_GRAPH_MODE} \
  activation_startup_policy:=fail_closed \
  activation_startup_timeout:=120.0 \
  ekf_profile:=wheel_imu \
  "imu_calibration_params_file:=${M3}/ros2_ws/src/robot_odometry/config/imu_calibration.yaml" \
  lidar_odometry_backend:=off \
  lidar_odometry_validated:=false \
  visual_odometry_shadow_enabled:=${V6_VISUAL_ODOMETRY_SHADOW_ENABLED} \
  spawn_pose_name:=long_route_start_g1 \
  "spawn_poses_file:=${M3}/isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.v6_clearance_r2.spawn.yaml" \
  "map_file:=${M3}/data/maps/occupancy/v6_kujiale_clearance_r2.yaml" \
  "route_graph_file:=${M3}/ros2_ws/src/robot_route_planner/config/v6_kujiale_clearance_r2_gvg_v1.geojson" \
  interactive:=false \
  use_rviz:=false \
  use_teleop:=false \
  "project_root:=${M3}"
nav_deadline=$((SECONDS + 420))
nav_ready=0
while (( SECONDS < nav_deadline )); do
  assert_alive navigation
  if ros2 topic list 2>/dev/null | grep -qx "/bio_nav/route_goal_complete"; then
    nav_ready=1
    break
  fi
  sleep 5
done
[[ "${nav_ready}" == 1 ]] || \
  _stop "navigation stack did not expose /bio_nav/route_goal_complete; see logs/navigation.log"

# ---------------------------------------------------------------- bridge
STAGE="bridge"
start_bg bridge ros2 launch bio_nav_ros_bridge v6_cognitive_navigation.launch.py \
  startup_profile:=estimated_shadow \
  localization_backend:=grid \
  runtime_profile:=estimated_m0 \
  "audit_jsonl_path:=${LOGS}/bridge_audit.jsonl" \
  use_sim_time:=true
sleep 15
assert_alive bridge

# ---------------------------------------------------------------- recorder
STAGE="record"
RECORD_TOPICS=(
  /clock /joint_states /imu/data_raw /imu/data /wheel/odom /odom
  /scan /flatscan /localization_result /bio_nav/localization/status
  /tf /tf_static /ground_truth/odom /simulation/reset_event
  /simulation/collision /simulation/collision_diagnostics
  /cmd_vel /cmd_vel_nav /cmd_vel_smoothed /cmd_vel_sim
  /plan /local_plan /local_costmap/costmap_raw
  /global_costmap/costmap_raw /planner_server/transition_event
  /controller_server/transition_event /velocity_smoother/transition_event
  /collision_monitor_state /scan_safety
  /bio_nav/navigation_graph /bio_nav/canonical_route /bio_nav/route_progress
  /bio_nav/route_goal_complete /bio_nav/route_goal /bio_nav/route_goal_result
  /bio_nav/route_edge_costs /diagnostics /rosout
)
if [[ "${V6_VISUAL_ODOMETRY_SHADOW_ENABLED}" == true ]]; then
  RECORD_TOPICS+=(
    /camera/front/image_raw
    /camera/front/depth/image_raw
    /camera/front/camera_info
    /visual/odom_shadow
    /visual/status
  )
fi
start_bg rosbag ros2 bag record \
  --storage mcap \
  --node-name r5_session_recorder \
  --qos-profile-overrides-path "${QOS_FILE}" \
  -o "${RUN_DIR}/rosbag/r5_session" \
  "${RECORD_TOPICS[@]}"
sleep 8
grep -q "Subscribed to topic '/ground_truth/odom'" "${LOGS}/rosbag.log" || \
  _stop "recorder did not subscribe to /ground_truth/odom; see logs/rosbag.log"
grep -q "Subscribed to topic '/cmd_vel_sim'" "${LOGS}/rosbag.log" || \
  _stop "recorder did not subscribe to /cmd_vel_sim; see logs/rosbag.log"
grep -q "Subscribed to topic '/rosout'" "${LOGS}/rosbag.log" || \
  _stop "recorder did not subscribe to /rosout; see logs/rosbag.log"
for required_topic in /flatscan /localization_result \
    /bio_nav/localization/status /odom /tf /tf_static; do
  grep -q "Subscribed to topic '${required_topic}'" "${LOGS}/rosbag.log" || \
    _stop "recorder did not subscribe to ${required_topic}; see logs/rosbag.log"
done
if [[ "${V6_VISUAL_ODOMETRY_SHADOW_ENABLED}" == true ]]; then
  for required_topic in /camera/front/image_raw \
      /camera/front/depth/image_raw /camera/front/camera_info \
      /visual/odom_shadow /visual/status; do
    grep -q "Subscribed to topic '${required_topic}'" "${LOGS}/rosbag.log" || \
      _stop "recorder did not subscribe to ${required_topic}; see logs/rosbag.log"
  done
fi

# ---------------------------------------------------------------- episodes
STAGE="episodes"
read -r -a index_rows <<< "${EPISODE_INDICES}"
read -r -a seed_rows <<< "${EPISODE_SEEDS}"
[[ "${#index_rows[@]}" == "${#seed_rows[@]}" ]] || \
  _stop "episode index/seed lists differ in length"
episode_results=()
for position in "${!index_rows[@]}"; do
  index="${index_rows[$position]}"
  seed="${seed_rows[$position]}"
  episode_jsonl="${EPISODES_DIR}/episode_seed${seed}.jsonl"
  episode_result="${EPISODES_DIR}/episode_seed${seed}.result.json"
  episode_status=0
  _log_stage "episode_start" "seed=${seed} index=${index}"
  timeout $((READINESS_TIMEOUT + RESET_TIMEOUT + 5 * NAVIGATION_TIMEOUT + 120)) \
    "${M3}/scripts/run_v6_formal_episode.sh" --pilot --dispatch-pilot \
    "${MANIFEST}" \
    --episode-index "${index}" \
    --output-jsonl "${episode_jsonl}" \
    --readiness-timeout-sec "${READINESS_TIMEOUT}" \
    --reset-timeout-sec "${RESET_TIMEOUT}" \
    --navigation-timeout-sec "${NAVIGATION_TIMEOUT}" \
    > "${episode_result}" 2> "${LOGS}/episode_seed${seed}.stderr.log" \
    || episode_status=$?
  echo "${episode_status}" > "${PROV}/episode_seed${seed}.exit_status.txt"
  _log_stage "episode_end" "seed=${seed} exit=${episode_status}"
  if [[ "${episode_status}" != 0 ]]; then
    _log_stage "episode_terminal_stop" \
      "seed=${seed} exit=${episode_status}; stopping owned navigation pgid"
    stop_bg navigation
  fi
  assert_alive isaac
  if [[ "${episode_status}" == 0 ]]; then
    assert_alive navigation
  fi
  assert_alive bridge

  # Read-only boundary probe after any failed episode has already stopped the
  # run-owned navigation process group and command chain.
  boundary_status=0
  python3 - "${EPISODES_DIR}/boundary_seed${seed}.json" <<'PYEOF' || boundary_status=$?
import json
import sys
import time

import rclpy

EXPECTED_PUBLISHERS = {
    "/odom": 1,
    "/cmd_vel": 1,
    "/cmd_vel_sim": 1,
    "/bio_nav/localization/status": 1,
}
rclpy.init()
node = rclpy.create_node("r5_boundary_probe")
max_publishers = {topic: 0 for topic in EXPECTED_PUBLISHERS}
max_gt_subscribers = 0
deadline = time.monotonic() + 20.0
while time.monotonic() < deadline:
    for topic in EXPECTED_PUBLISHERS:
        max_publishers[topic] = max(max_publishers[topic], node.count_publishers(topic))
    max_gt_subscribers = max(
        max_gt_subscribers, node.count_subscribers("/ground_truth/odom")
    )
    time.sleep(0.5)
result = {
    "max_publishers": max_publishers,
    "expected_publishers": EXPECTED_PUBLISHERS,
    "ground_truth_odom_max_subscribers": max_gt_subscribers,
    "publisher_ownership_pass": max_publishers == EXPECTED_PUBLISHERS,
    "ground_truth_firewall_pass": max_gt_subscribers == 1,
}
with open(sys.argv[1], "w", encoding="utf-8") as stream:
    json.dump(result, stream, indent=2, sort_keys=True)
    stream.write("\n")
sys.exit(0 if (result["publisher_ownership_pass"] and result["ground_truth_firewall_pass"]) else 2)
PYEOF
  echo "${boundary_status}" > "${PROV}/boundary_seed${seed}.exit_status.txt"
  episode_results+=("${seed}:${episode_status}:boundary=${boundary_status}")
  if [[ "${episode_status}" != 0 ]]; then
    _stop "episode seed ${seed} failed with exit ${episode_status}; evidence kept"
  fi
done

# ---------------------------------------------------------------- finalize
STAGE="finalize"
stop_bg rosbag
ros2 bag info "${RUN_DIR}/rosbag/r5_session" \
  > "${PROV}/r5_session_bag_info.txt" 2>&1 || true
_cleanup_children
CHILD_PGIDS=()
sleep 4
ros2 node list > "${PROV}/node_list_postcleanup.txt" 2>&1 || true
{
  echo "{"
  echo "  \"session\": \"r5_reset_cold_boundary\","
  echo "  \"run_dir\": \"${RUN_DIR}\","
  echo "  \"domain_id\": ${DOMAIN_ID},"
  echo "  \"episode_results\": [$(printf '"%s", ' "${episode_results[@]}" | sed 's/, $//')],"
  echo "  \"completed_at\": \"$(date -u +%Y-%m-%dT%H:%M:%SZ)\""
  echo "}"
} > "${RUN_DIR}/driver_summary.json"
_log_stage "DONE" "r5 session complete"
exit 0
