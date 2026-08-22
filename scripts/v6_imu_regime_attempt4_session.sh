#!/usr/bin/env bash
# V6 IMU regime Attempt4 — locked live capture session driver.
#
# usage:
#   v6_imu_regime_attempt4_session.sh flat20 RUN_DIR SNAPSHOT_ROOT
#   v6_imu_regime_attempt4_session.sh goal   RUN_DIR SNAPSHOT_ROOT
#
# flat20: locked flat20 runner + LiDAR preflight + stationary seed 8609 +
#         nine primitives (seeds 8610..8618) + schema-2 MotionBenchmark report
#         + MCAP + phase JSONL, all evidence written under RUN_DIR.
# goal:   the separately provenance-bearing Kujiale Estimated goal capture
#         (replicates the v6_estimated_dynamic_smoke session shape) producing
#         the goal MCAP plus goal_mcap_outcome_metadata JSON.
#
# The driver only orchestrates existing locked entry points; it changes no
# runtime behavior.  Everything runs from the immutable SNAPSHOT_ROOT source
# archive with its isolated build/install trees.

set -Eeuo pipefail

SESSION="${1:-}"
RUN_DIR="${2:-}"
SNAP="${3:-}"
if [[ "${SESSION}" != flat20 && "${SESSION}" != goal ]] || [[ -z "${RUN_DIR}" || -z "${SNAP}" ]]; then
  echo "usage: $0 flat20|goal RUN_DIR SNAPSHOT_ROOT" >&2
  exit 64
fi

DOMAIN_ID="${ATTEMPT4_DOMAIN_ID:-171}"
GOAL_SEED="${ATTEMPT4_GOAL_SEED:-8619}"
M3="${SNAP}/m3_src"
I_SRC="${SNAP}/i_src"
M2="${SNAP}/m2_src"
M3_INSTALL="${M3}/ros2_ws/install_attempt4"
I_INSTALL="${I_SRC}/ros2_ws/install_attempt4"
LOGS="${RUN_DIR}/logs"
PROV="${RUN_DIR}/provenance"
ANALYSIS="${RUN_DIR}/analysis"
LOCK_DIR="${ISAAC_NAV_RUNTIME_DIR:-/tmp/isaac_sim_ros2_nav_$(id -u)}"

export ROS_DOMAIN_ID="${DOMAIN_ID}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ISAAC_NAV_EXPECTED_DOMAIN_ID="${DOMAIN_ID}"
export ROS_LOG_DIR="${SNAP}/ros_log"

mkdir -p "${LOGS}" "${PROV}" "${ANALYSIS}"

if [[ -f "${RUN_DIR}/STOP.md" || -f "${RUN_DIR}/driver_summary.json" ]]; then
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
    echo "# V6 IMU regime Attempt4 ${SESSION} — STOP"
    echo
    echo "- run_dir: ${RUN_DIR}"
    echo "- domain: ${DOMAIN_ID}"
    echo "- stage: ${STAGE}"
    echo "- reason: ${reason}"
    echo "- stopped_at: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  } > "${RUN_DIR}/STOP.md"
  exit 2
}

_on_err() {
  _stop "driver error at line $1 during stage ${STAGE}; see ${LOGS}"
}
trap '_on_err ${LINENO}' ERR

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
SHARE="${ROBOT_EXPERIMENTS_PREFIX}/share/robot_experiments"

{
  echo "session=${SESSION}"
  echo "run_dir=${RUN_DIR}"
  echo "snapshot=${SNAP}"
  echo "domain_id=${DOMAIN_ID}"
  echo "rmw=${RMW_IMPLEMENTATION}"
  echo "module3_snapshot=$(git -C "${M3}" rev-parse HEAD 2>/dev/null || echo archive)"
  echo "yaw_scale=0.9294"
  echo "rf2o=off"
  echo "cognitive_profile=M0"
  echo "goal_seed=${GOAL_SEED}"
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
/lidar/points_raw:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 10
/lidar/points_scan:
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

# ================================================================ flat20
if [[ "${SESSION}" == flat20 ]]; then
  STAGE="isaac_flat20"
  start_bg isaac "${M3}/scripts/run_v6_imu_regime_diagnostic_isaac.sh" \
    "${RUN_DIR}/imu_regime_phase.jsonl"
  wait_topic /clock 420 || _stop "Isaac /clock did not appear; see logs/isaac.log"
  assert_alive isaac
  wait_service /simulation/reset 300 || _stop "reset service did not appear"
  assert_alive isaac

  STAGE="mapping_stack"
  start_bg mapping ros2 launch robot_bringup ros_stack.launch.py \
    operation:=mapping \
    odometry_mode:=estimated \
    structure_tf_source:=isaac \
    "spawn_poses_file:=${SHARE}/environments/v6_calibration_flat_20m.spawn.yaml" \
    spawn_pose_name:=flat20_start \
    "imu_calibration_params_file:=${M3}/ros2_ws/src/robot_odometry/config/imu_calibration.yaml" \
    ekf_profile:=wheel_imu \
    lidar_odometry_backend:=off \
    lidar_odometry_validated:=false \
    nav2_profile:=stable \
    cognitive_profile:=M0 \
    module2_enabled:=false \
    cognitive_graph_mode:=gvg \
    interactive:=false \
    use_rviz:=false \
    use_teleop:=false \
    "project_root:=${M3}"
  wait_topic /scan 300 || _stop "mapping /scan did not appear; see logs/mapping.log"

  STAGE="safety_chain"
  PERCEPTION_SHARE="$(ros2 pkg prefix robot_perception)/share/robot_perception"
  start_bg self_filter ros2 run robot_perception lidar_self_filter --ros-args \
    --params-file "${PERCEPTION_SHARE}/config/self_filter_optional.yaml" \
    -p use_sim_time:=true \
    -p input_topic:=/lidar/points_raw \
    -p output_topic:=/lidar/points_scan
  NAV_CONFIG="$(ros2 pkg prefix robot_navigation)/share/robot_navigation/config"
  start_bg safety_scan ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
    --ros-args \
    -r __node:=pointcloud_to_laserscan_safety \
    --params-file "${PERCEPTION_SHARE}/config/pointcloud_to_laserscan_safety.yaml" \
    -p use_sim_time:=true \
    -r cloud_in:=/lidar/points_scan \
    -r scan:=/scan_safety
  start_bg velocity_smoother ros2 run nav2_velocity_smoother velocity_smoother \
    --ros-args \
    --params-file "${NAV_CONFIG}/nav2_params.yaml" \
    --params-file "${NAV_CONFIG}/nav2_stable.yaml" \
    -p use_sim_time:=true \
    -r cmd_vel:=/cmd_vel_nav
  # Diagnostic-only override: keep the monitor republishing zeros through the
  # 10 s stationary window and every settle window (default stop_pub_timeout
  # 1.0 s would go silent and starve the schema-2 zero-coverage evidence).
  start_bg collision_monitor ros2 run nav2_collision_monitor collision_monitor \
    --ros-args \
    --params-file "${NAV_CONFIG}/nav2_params.yaml" \
    --params-file "${NAV_CONFIG}/nav2_stable.yaml" \
    -p use_sim_time:=true \
    -p stop_pub_timeout:=30.0
  sleep 6
  ros2 lifecycle set /velocity_smoother configure > "${PROV}/velocity_smoother_configure.txt" 2>&1
  ros2 lifecycle set /collision_monitor configure > "${PROV}/collision_monitor_configure.txt" 2>&1
  ros2 lifecycle set /velocity_smoother activate > "${PROV}/velocity_smoother_activate.txt" 2>&1
  ros2 lifecycle set /collision_monitor activate > "${PROV}/collision_monitor_activate.txt" 2>&1
  ros2 lifecycle get /velocity_smoother > "${PROV}/velocity_smoother_state.txt" 2>&1 || true
  ros2 lifecycle get /collision_monitor > "${PROV}/collision_monitor_state.txt" 2>&1 || true
  grep -q "active" "${PROV}/velocity_smoother_state.txt" || _stop "velocity_smoother not active"
  grep -q "active" "${PROV}/collision_monitor_state.txt" || _stop "collision_monitor not active"
  wait_topic /scan_safety 120 || _stop "/scan_safety did not appear"

  STAGE="preflight"
  ros2 node list > "${PROV}/node_list_preflight.txt" 2>&1 || true
  ros2 topic list > "${PROV}/topic_list_preflight.txt" 2>&1 || true
  for topic in /cmd_vel /cmd_vel_sim /cmd_vel_smoothed /cmd_vel_nav \
      /imu/data_raw /imu/data /ground_truth/odom /scan /scan_safety \
      /lidar/points_raw /lidar/points_scan; do
    ros2 topic info "${topic}" > "${PROV}/topic_info_$(echo "${topic}" | tr '/' '_').txt" 2>&1 || true
  done
  python3 - "${PROV}/command_authority_preflight_rclpy.json" <<'PYEOF'
import json
import sys
import time

import rclpy

EXPECTED = {
    "/cmd_vel": 1,
    "/cmd_vel_sim": 1,
    "/cmd_vel_smoothed": 1,
    "/cmd_vel_nav": 0,
}
rclpy.init()
node = rclpy.create_node("attempt4_authority_preflight")
max_publishers = {topic: 0 for topic in EXPECTED}
samples = 0
deadline = time.monotonic() + 60.0
while time.monotonic() < deadline and samples < 40:
    for topic in EXPECTED:
        count = node.count_publishers(topic)
        max_publishers[topic] = max(max_publishers[topic], count)
    samples += 1
    time.sleep(0.5)
last = {topic: node.count_publishers(topic) for topic in EXPECTED}
node.destroy_node()
rclpy.shutdown()
result = {
    "samples": samples,
    "last": last,
    "max_publishers": max_publishers,
    "expected_publishers": EXPECTED,
    "pass": max_publishers == EXPECTED,
}
with open(sys.argv[1], "w", encoding="utf-8") as stream:
    json.dump(result, stream, indent=2, sort_keys=True)
    stream.write("\n")
sys.exit(0 if result["pass"] else 2)
PYEOF
  ros2 run robot_experiments v6_imu_lidar_preflight \
    --output "${PROV}/lidar_readiness.json" > "${LOGS}/lidar_preflight.log" 2>&1
  echo "$?" > "${PROV}/lidar_preflight_exit_status.txt"
  [[ "$(cat "${PROV}/lidar_preflight_exit_status.txt")" == "0" ]] || \
    _stop "v6_imu_lidar_preflight failed; see logs/lidar_preflight.log"

  STAGE="record"
  start_bg rosbag ros2 bag record \
    --storage mcap \
    --node-name attempt4_mcap_recorder \
    --qos-profile-overrides-path "${QOS_FILE}" \
    -o "${RUN_DIR}/rosbag/flat20_motion" \
    -a
  sleep 6
  grep -q "Subscribed to topic '/imu/data_raw'" "${LOGS}/rosbag.log" || \
    _stop "MCAP recorder did not subscribe to /imu/data_raw; see logs/rosbag.log"
  start_bg safety_monitor python3 "${M3}/scripts/v6_imu_regime_attempt4_monitor.py" \
    --output "${LOGS}/safety_monitor.jsonl" \
    --summary "${ANALYSIS}/safety_monitor_summary.json"
  sleep 2

  STAGE="benchmark"
  benchmark_status=0
  timeout 900 ros2 run robot_experiments motion_benchmark \
    --config "${SHARE}/config/v6_imu_regime_diagnostic.yaml" \
    --output "${ANALYSIS}/motion_report.json" \
    > "${LOGS}/motion_benchmark.log" 2>&1 || benchmark_status=$?
  echo "${benchmark_status}" > "${PROV}/motion_benchmark_exit_status.txt"
  if [[ "${benchmark_status}" != 0 && "${benchmark_status}" != 2 ]]; then
    _stop "motion_benchmark exit ${benchmark_status}; see logs/motion_benchmark.log"
  fi
  [[ -s "${ANALYSIS}/motion_report.json" ]] || _stop "motion benchmark report missing"

  STAGE="finalize"
  stop_bg safety_monitor
  stop_bg rosbag
  ros2 bag info "${RUN_DIR}/rosbag/flat20_motion" \
    > "${PROV}/flat20_motion_bag_info.txt" 2>&1 || true
  _cleanup_children
  CHILD_PGIDS=()
  sleep 4
  ros2 node list > "${PROV}/node_list_postcleanup.txt" 2>&1 || true
  cat > "${RUN_DIR}/driver_summary.json" <<JSON
{
  "session": "flat20",
  "run_dir": "${RUN_DIR}",
  "domain_id": ${DOMAIN_ID},
  "benchmark_exit_status": ${benchmark_status},
  "lidar_preflight_exit_status": $(cat "${PROV}/lidar_preflight_exit_status.txt"),
  "completed_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
JSON
  _log_stage "DONE" "flat20 capture complete"
  exit 0
fi

# ================================================================ goal
STAGE="isaac_kujiale"
mkdir -p "${SNAP}/runtime" "${RUN_DIR}/probe" "${RUN_DIR}/evaluator"
pushd "${I_SRC}/module2_runtime/bio_nav_module2_server" > /dev/null
start_bg module2 /home/lyb/miniconda3/envs/bionav-module2/bin/python -u server.py \
  --runtime-version v310 \
  --module2-root "${M2}" \
  --bridge-source "${I_SRC}/ros2_ws/src/bio_nav_ros_bridge" \
  --socket "${SNAP}/runtime/module2.sock" \
  --device cuda
popd > /dev/null
wait_file "${SNAP}/runtime/module2.sock" 180 || _stop "module2 socket did not appear"
start_bg isaac "${M3}/scripts/run_kujiale_4x20_isaac.sh" static \
  --headless --no-dynamic-obstacles --mode realistic --dynamic-seed "${GOAL_SEED}"
wait_topic /clock 600 || _stop "Kujiale /clock did not appear; see logs/isaac.log"
assert_alive isaac
wait_service /simulation/reset 300 || _stop "reset service did not appear"
assert_alive isaac

STAGE="navigation_stack"
start_bg navigation ros2 launch robot_bringup ros_stack.launch.py \
  operation:=navigation \
  odometry_mode:=estimated \
  localization_profile:=kujiale \
  nav2_profile:=v6_low_obstacle_isolation \
  cognitive_profile:=M0 \
  cognitive_graph_mode:=gvg \
  initial_pose_source:=auto \
  activation_startup_policy:=fail_closed \
  activation_startup_timeout:=120.0 \
  ekf_profile:=wheel_imu \
  "imu_calibration_params_file:=${M3}/ros2_ws/src/robot_odometry/config/imu_calibration.yaml" \
  lidar_odometry_backend:=off \
  lidar_odometry_validated:=false \
  spawn_pose_name:=long_route_start_g1 \
  "spawn_poses_file:=${M3}/isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.spawn.yaml" \
  "posegraph_file:=${M3}/data/maps/posegraphs/warehouse_new" \
  "map_file:=${M3}/data/maps/occupancy/warehouse_new.yaml" \
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

STAGE="bridge"
start_bg bridge ros2 launch bio_nav_ros_bridge v6_cognitive_navigation.launch.py \
  startup_profile:=estimated_shadow \
  "socket_path:=${SNAP}/runtime/module2.sock" \
  "audit_jsonl_path:=${LOGS}/bridge_audit.jsonl" \
  localization_supervisor_mode:=shadow \
  use_sim_time:=true
sleep 15
assert_alive bridge

STAGE="evaluator_and_record"
start_bg evaluator ros2 run robot_experiments estimated_state_evaluator --ros-args \
  -p "output_dir:=${RUN_DIR}/evaluator" \
  -p episode_id:=attempt4_kujiale_estimated_g1_g2_seed${GOAL_SEED} \
  -p arm:=M0_gvg_wheel_imu_lidar_off \
  -p use_sim_time:=true \
  -p report_period_sec:=5.0
start_bg rosbag ros2 bag record \
  --storage mcap \
  --node-name attempt4_goal_recorder \
  --qos-profile-overrides-path "${QOS_FILE}" \
  -o "${RUN_DIR}/rosbag/kujiale_goal" \
  /clock /joint_states /imu/data_raw /imu/data /wheel/odom /odom /amcl_pose \
  /tf /tf_static /ground_truth/odom /simulation/reset_event \
  /simulation/collision /simulation/collision_diagnostics \
  /cmd_vel /cmd_vel_nav /cmd_vel_smoothed \
  /bio_nav/navigation_graph /bio_nav/canonical_route /bio_nav/route_progress \
  /bio_nav/route_goal_complete /bio_nav/route_goal /plan /scan /scan_safety \
  /bio_nav/module2/planning_prior /bio_nav/module2/edge_priors \
  /bio_nav/module2/srdr_edge_diagnostics /bio_nav/route_edge_costs /rosout
sleep 8
grep -q "Subscribed to topic '/imu/data_raw'" "${LOGS}/rosbag.log" || \
  _stop "goal recorder did not subscribe to /imu/data_raw; see logs/rosbag.log"
grep -q "Subscribed to topic '/rosout'" "${LOGS}/rosbag.log" || \
  _stop "goal recorder did not subscribe to /rosout; see logs/rosbag.log"

STAGE="reset_and_goal"
reset_status=0
ros2 service call /simulation/reset std_srvs/srv/Trigger '{}' \
  > "${PROV}/reset_call.log" 2>&1 || reset_status=$?
echo "${reset_status}" > "${PROV}/reset_call_exit_status.txt"
[[ "${reset_status}" == 0 ]] || _stop "reset service call failed"
grep -q "success=True" "${PROV}/reset_call.log" || \
  _stop "reset transaction was not successful; see provenance/reset_call.log"
grep -o 'reset_receipt={[^}]*}' "${PROV}/reset_call.log" \
  > "${PROV}/reset_receipt.txt" || _stop "reset receipt missing from response"
grep -q "\"seed\":${GOAL_SEED}" "${PROV}/reset_receipt.txt" || \
  _stop "reset receipt seed != requested ${GOAL_SEED}; see provenance/reset_receipt.txt"
start_bg safety_monitor python3 "${M3}/scripts/v6_imu_regime_attempt4_monitor.py" \
  --output "${LOGS}/safety_monitor.jsonl" \
  --summary "${ANALYSIS}/safety_monitor_summary_goal.json"
probe_status=0
timeout 420 ros2 run robot_route_planner probe_closed_loop -- \
  --goal 0.80 4.80 -2.792526803 \
  --map "${M3}/data/maps/occupancy/warehouse_new.yaml" \
  --defaults "${I_SRC}/ros2_ws/src/bio_nav_ros_bridge/config/engineering_defaults.yaml" \
  --output-json "${RUN_DIR}/probe/closed_loop.json" \
  --output-image "${RUN_DIR}/probe/closed_loop.png" \
  --timeout 180 \
  --experiment-arm baseline \
  --query-id "G1_to_G2_estimated_M0_gvg_attempt4_seed${GOAL_SEED}" \
  > "${LOGS}/probe.log" 2>&1 || probe_status=$?
echo "${probe_status}" > "${PROV}/probe_exit_status.txt"
[[ -s "${RUN_DIR}/probe/closed_loop.json" ]] || \
  _stop "probe produced no closed_loop.json (exit ${probe_status}); see logs/probe.log"

STAGE="finalize_goal"
stop_bg safety_monitor
stop_bg rosbag
ros2 bag info "${RUN_DIR}/rosbag/kujiale_goal" \
  > "${PROV}/kujiale_goal_bag_info.txt" 2>&1 || true
python3 "${M3}/scripts/v6_imu_regime_attempt4_goal_metadata.py" \
  --goal-mcap "${RUN_DIR}/rosbag/kujiale_goal" \
  --output "${ANALYSIS}/goal_metadata.json" \
  --requested-seed "${GOAL_SEED}" || \
  _stop "goal metadata extraction failed; bag kept for inspection"
_cleanup_children
CHILD_PGIDS=()
sleep 4
ros2 node list > "${PROV}/node_list_postcleanup.txt" 2>&1 || true
cat > "${RUN_DIR}/driver_summary.json" <<JSON
{
  "session": "goal",
  "run_dir": "${RUN_DIR}",
  "domain_id": ${DOMAIN_ID},
  "goal_seed": ${GOAL_SEED},
  "probe_exit_status": ${probe_status},
  "completed_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
JSON
_log_stage "DONE" "goal capture complete"
exit 0
