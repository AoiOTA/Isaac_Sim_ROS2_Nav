#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
INTEGRATION_ROOT="/home/lyb/Workspace/Bio_Nav/worktrees/integration/attempt30-a21-v310-srdr-rviz"
EVIDENCE_ROOT="${INTEGRATION_ROOT}/docs/evidence/attempt30_a21_v310/multiroute_benchmark_v4"
DEFAULTS="${INTEGRATION_ROOT}/ros2_ws/src/bio_nav_ros_bridge/config/engineering_defaults.yaml"
MAP_FILE="${EVIDENCE_ROOT}/attempt30_a21_multiroute_v4.yaml"
CANDIDATES="${EVIDENCE_ROOT}/attempt30_a21_multiroute_v4_execution_candidates.json"
RUNNER="${SCRIPT_DIR}/run_attempt30_a21_multiroute_v4.sh"

query_id="${1:-}"
arm="${2:-}"
variant="${3:-}"
output_root="${4:-}"
[[ -n "${query_id}" && -n "${arm}" && -n "${variant}" && -n "${output_root}" ]] || {
  echo "usage: $0 QUERY_ID baseline|sr_only|dr_only|srdr v1..v5 OUTPUT_ROOT" >&2
  exit 2
}
case "${arm}" in baseline|sr_only|dr_only|srdr) ;; *) echo "invalid arm: ${arm}" >&2; exit 2 ;; esac
case "${variant}" in v1|v2|v3|v4|v5) ;; *) echo "invalid variant: ${variant}" >&2; exit 2 ;; esac

case "${query_id}" in
  Q02_58) dynamic_stem="attempt30_a21_q02_58_dynamic"; case_id="q02_58_crossing"; obstacle_group="FOCUS" ;;
  Q01_50) dynamic_stem="attempt30_a21_q01_50_dynamic"; case_id="q01_50_crossing"; obstacle_group="FOCUS" ;;
  Q36_04) dynamic_stem="attempt30_a21_q36_04_dynamic"; case_id="q36_04_crossing"; obstacle_group="FOCUS" ;;
  Q14_45) dynamic_stem="attempt30_a21_q14_45_dynamic"; case_id="q14_45_crossing"; obstacle_group="FOCUS" ;;
  Q36_51) dynamic_stem="attempt30_a21_multiroute_v4_benefit_dynamic"; case_id="outer_east_crossing"; obstacle_group="BENEFIT" ;;
  *) echo "unsupported query: ${query_id}" >&2; exit 2 ;;
esac
dynamic_config="${PROJECT_ROOT}/isaac_sim/configs/benchmarks/${dynamic_stem}.yaml"
for required in "${RUNNER}" "${DEFAULTS}" "${MAP_FILE}" "${CANDIDATES}" "${dynamic_config}"; do
  [[ -f "${required}" ]] || { echo "required file is missing: ${required}" >&2; exit 2; }
done

trial_dir="$(realpath -m "${output_root}")/${query_id}/${arm}/${variant}"
trial_timeout_s="${ATTEMPT30_A21_TRIAL_TIMEOUT_S:-110}"
trial_domain_id="${ATTEMPT30_A21_TRIAL_DOMAIN_ID:-151}"
[[ "${trial_timeout_s}" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
  echo "ATTEMPT30_A21_TRIAL_TIMEOUT_S must be numeric" >&2
  exit 2
}
[[ "${trial_domain_id}" =~ ^[0-9]+$ ]] \
  && (( trial_domain_id >= 0 && trial_domain_id <= 232 )) || {
  echo "ATTEMPT30_A21_TRIAL_DOMAIN_ID must be an integer from 0 through 232" >&2
  exit 2
}
trial_runtime_dir="/tmp/isaac_sim_ros2_nav_${UID}_a21_v310_q${trial_domain_id}"
closed_loop_json="${trial_dir}/closed_loop.json"
closed_loop_png="${trial_dir}/closed_loop.png"
if [[ -e "${closed_loop_json}" || -e "${closed_loop_png}" ]]; then
  echo "refusing to overwrite existing trial: ${trial_dir}" >&2
  exit 2
fi
mkdir -p "${trial_dir}/logs"

read -r goal_x goal_y goal_yaw < <(
  python3 - "${CANDIDATES}" "${query_id}" <<'PY'
import json
import sys

records = json.load(open(sys.argv[1], encoding="utf-8"))
matches = [item for item in records if item["query_id"] == sys.argv[2]]
if len(matches) != 1:
    raise SystemExit(f"query {sys.argv[2]!r} not found exactly once")
print(*matches[0]["goal_xy_yaw_deg"])
PY
)

declare -a process_groups=()
cleanup() {
  local pid
  set +e
  for ((index=${#process_groups[@]}-1; index>=0; index--)); do
    pid="${process_groups[index]}"
    kill -INT -- "-${pid}" 2>/dev/null || true
  done
  sleep 1
  for pid in "${process_groups[@]}"; do
    kill -TERM -- "-${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
on_signal() {
  trap - INT TERM
  cleanup
  exit 130
}
trap cleanup EXIT
trap on_signal INT TERM

start_group() {
  local log_file="$1"
  shift
  setsid "$@" >"${log_file}" 2>&1 &
  STARTED_PID=$!
  process_groups+=("${STARTED_PID}")
}

wait_for_log() {
  local pid="$1" log_file="$2" pattern="$3" timeout_s="$4"
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    grep -Fq -- "${pattern}" "${log_file}" 2>/dev/null && return 0
    kill -0 "${pid}" 2>/dev/null || {
      echo "component exited before readiness pattern '${pattern}': ${log_file}" >&2
      tail -80 "${log_file}" >&2 || true
      return 1
    }
    sleep 1
  done
  echo "readiness timeout for '${pattern}': ${log_file}" >&2
  tail -80 "${log_file}" >&2 || true
  return 1
}

wait_for_socket() {
  local pid="$1" socket_path="$2" timeout_s="$3"
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    [[ -S "${socket_path}" ]] && return 0
    kill -0 "${pid}" 2>/dev/null || {
      echo "Module2 exited before creating socket: ${socket_path}" >&2
      return 1
    }
    sleep 1
  done
  echo "Module2 socket readiness timeout: ${socket_path}" >&2
  return 1
}

wait_for_planning_prior_input() {
  local pid="$1" timeout_s="$2"
  local deadline=$((SECONDS + timeout_s))
  local sample
  while (( SECONDS < deadline )); do
    kill -0 "${pid}" 2>/dev/null || {
      echo "edge-prior stack exited before PlanningPrior readiness" >&2
      return 1
    }
    sample="$({
      export ROS_DOMAIN_ID="${trial_domain_id}"
      set +u
      source /opt/ros/jazzy/setup.bash
      source "${PROJECT_ROOT}/install/setup.bash"
      set -u
      timeout 4 ros2 topic echo --once /bio_nav/module2/planning_prior 2>/dev/null
    } || true)"
    grep -Fq "input_healthy: true" <<<"${sample}" && return 0
    sleep 1
  done
  echo "input-healthy PlanningPrior readiness timeout" >&2
  return 1
}

cd "${PROJECT_ROOT}"
if [[ "${arm}" != "baseline" ]]; then
  start_group "${trial_dir}/logs/module2.log" env PYTHONUNBUFFERED=1 \
    ISAAC_NAV_EXPECTED_DOMAIN_ID="${trial_domain_id}" \
    ISAAC_NAV_RUNTIME_DIR="${trial_runtime_dir}" \
    "${RUNNER}" module2 "${query_id}" "${arm}"
  module2_pid="${STARTED_PID}"
  wait_for_socket "${module2_pid}" \
    "${trial_runtime_dir}/module2-v310.sock" 90
fi

start_group "${trial_dir}/logs/isaac.log" env \
  ISAAC_NAV_EXPECTED_DOMAIN_ID="${trial_domain_id}" \
  ISAAC_NAV_RUNTIME_DIR="${trial_runtime_dir}" \
  ATTEMPT30_A21_DYNAMIC_CONFIG="${dynamic_config}" \
  ATTEMPT30_A21_DYNAMIC_CASE="${case_id}" \
  ATTEMPT30_A21_DYNAMIC_VARIANT="${variant}" \
  "${RUNNER}" isaac "${query_id}" "${arm}"
isaac_pid="${STARTED_PID}"

wait_for_log "${isaac_pid}" "${trial_dir}/logs/isaac.log" "Isaac navigation simulation ready" 90

if [[ "${arm}" != "baseline" ]]; then
  # Start before the structural graph so this stack receives the one-shot
  # cognitive-map constraints publication used by PlanningPrior inference.
  start_group "${trial_dir}/logs/prior.log" env \
    ISAAC_NAV_EXPECTED_DOMAIN_ID="${trial_domain_id}" \
    ISAAC_NAV_RUNTIME_DIR="${trial_runtime_dir}" \
    "${RUNNER}" prior "${query_id}" "${arm}"
  prior_pid="${STARTED_PID}"
  wait_for_log "${prior_pid}" "${trial_dir}/logs/prior.log" "process started with pid" 30
fi

start_group "${trial_dir}/logs/ros.log" env \
  ISAAC_NAV_EXPECTED_DOMAIN_ID="${trial_domain_id}" \
  ISAAC_NAV_RUNTIME_DIR="${trial_runtime_dir}" \
  "${RUNNER}" ros "${query_id}" "${arm}"
ros_pid="${STARTED_PID}"
wait_for_log "${ros_pid}" "${trial_dir}/logs/ros.log" "Managed nodes are active" 90

if [[ "${arm}" != "baseline" ]]; then
  wait_for_planning_prior_input "${prior_pid}" 240
fi

set +e
(
  export ROS_DOMAIN_ID="${trial_domain_id}"
  export ATTEMPT30_A21_TRIAL_DOMAIN_ID="${trial_domain_id}"
  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
  set +u
  source /opt/ros/jazzy/setup.bash
  source "${PROJECT_ROOT}/install/setup.bash"
  set -u
  ros2 run robot_route_planner probe_closed_loop -- \
    --goal "${goal_x}" "${goal_y}" "${goal_yaw}" \
    --map "${MAP_FILE}" \
    --defaults "${DEFAULTS}" \
    --output-json "${closed_loop_json}" \
    --output-image "${closed_loop_png}" \
    --timeout "${trial_timeout_s}" \
    --obstacle-group "${obstacle_group}" \
    --experiment-arm "${arm}" \
    --query-id "${query_id}" \
    --dynamic-case-id "${case_id}" \
    --dynamic-variant-id "${variant}"
) >"${trial_dir}/logs/probe.log" 2>&1
probe_status=$?
set -e
cat "${trial_dir}/logs/probe.log"
[[ ${probe_status} -eq 0 ]] || exit "${probe_status}"

python3 - "${closed_loop_json}" <<'PY'
import json
import sys

record = json.load(open(sys.argv[1], encoding="utf-8"))
summary = {
    key: record.get(key)
    for key in (
        "experiment_arm", "query_id", "dynamic_variant_id", "completed",
        "failed", "elapsed_s", "travelled_distance_m", "physical_collision",
        "sampled_static_footprint_collisions", "obstacle_center_min_distance_m",
    )
}
print(json.dumps(summary, sort_keys=True))
if not record.get("completed") or record.get("failed"):
    raise SystemExit(1)
if record.get("physical_collision") or record.get("sampled_static_footprint_collisions"):
    raise SystemExit(1)
arm = record.get("experiment_arm")
if arm != "baseline":
    if not any(item.get("healthy") for item in record.get("edge_prior_history", [])):
        raise SystemExit("no healthy Module2 edge prior was captured")
    diagnostics = record.get("srdr_edge_diagnostic_history", [])
    if arm == "sr_only" and not any(
        item.get("positive_sr_count", 0) > 0 and item.get("positive_dr_count", 0) == 0
        for item in diagnostics
    ):
        raise SystemExit("SR-only trial did not capture an SR-only positive edge penalty")
    if arm == "dr_only" and not any(
        item.get("positive_dr_count", 0) > 0 and item.get("positive_sr_count", 0) == 0
        for item in diagnostics
    ):
        raise SystemExit("DR-only trial did not capture a DR-only positive edge penalty")
    if arm == "srdr" and not any(
        item.get("positive_sr_count", 0) > 0 and item.get("positive_dr_count", 0) > 0
        for item in diagnostics
    ):
        raise SystemExit("SRDR trial did not capture simultaneous SR and DR penalties")
PY

# Allow DDS participants and the GPU runtime to fully leave before a campaign
# driver starts the next arm with the same domain and topic names.
cleanup
trap - EXIT
sleep 5
