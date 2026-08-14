#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
INTEGRATION_ROOT="${BIO_NAV_ATTEMPT30_V310_INTEGRATION_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/integration/attempt30-a21-v310-srdr-rviz}"
EVIDENCE_ROOT="${ATTEMPT30_A21_V4_EVIDENCE_ROOT:-${INTEGRATION_ROOT}/docs/evidence/attempt30_a21_v310/multiroute_benchmark_v4}"
BENCHMARK_STEM="${ATTEMPT30_A21_BENCHMARK_STEM:-attempt30_a21_multiroute_v4}"
DEFAULTS="${INTEGRATION_ROOT}/ros2_ws/src/bio_nav_ros_bridge/config/engineering_defaults.yaml"
MAP_FILE="${EVIDENCE_ROOT}/${BENCHMARK_STEM}.yaml"
CANDIDATES="${EVIDENCE_ROOT}/${BENCHMARK_STEM}_execution_candidates.json"
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

if [[ -n "${ATTEMPT30_A21_DYNAMIC_CONFIG_OVERRIDE:-}" \
    && -n "${ATTEMPT30_A21_DYNAMIC_CASE_OVERRIDE:-}" \
    && -n "${ATTEMPT30_A21_OBSTACLE_GROUP_OVERRIDE:-}" ]]; then
  dynamic_config="$(realpath -e "${ATTEMPT30_A21_DYNAMIC_CONFIG_OVERRIDE}")"
  case_id="${ATTEMPT30_A21_DYNAMIC_CASE_OVERRIDE}"
  obstacle_group="${ATTEMPT30_A21_OBSTACLE_GROUP_OVERRIDE}"
else
  case "${query_id}" in
    Q02_58) dynamic_stem="attempt30_a21_q02_58_dynamic"; case_id="q02_58_crossing"; obstacle_group="FOCUS" ;;
    Q01_50) dynamic_stem="attempt30_a21_q01_50_dynamic"; case_id="q01_50_crossing"; obstacle_group="FOCUS" ;;
    Q36_04) dynamic_stem="attempt30_a21_q36_04_dynamic"; case_id="q36_04_crossing"; obstacle_group="FOCUS" ;;
    Q14_45) dynamic_stem="attempt30_a21_q14_45_dynamic"; case_id="q14_45_crossing"; obstacle_group="FOCUS" ;;
    Q36_51) dynamic_stem="attempt30_a21_multiroute_v4_benefit_dynamic"; case_id="outer_east_crossing"; obstacle_group="BENEFIT" ;;
    *) echo "unsupported query without explicit dynamic overrides: ${query_id}" >&2; exit 2 ;;
  esac
  dynamic_config="${PROJECT_ROOT}/isaac_sim/configs/benchmarks/${dynamic_stem}.yaml"
fi
for required in "${RUNNER}" "${DEFAULTS}" "${MAP_FILE}" "${CANDIDATES}" "${dynamic_config}"; do
  [[ -f "${required}" ]] || { echo "required file is missing: ${required}" >&2; exit 2; }
done

source_attempt30_ros() {
  # A colcon rebuild can regenerate PROJECT_ROOT/install/setup.bash with the
  # desktop shell's unrelated Integration underlay. Bind every CLI probe to
  # the same Attempt30 Integration/Module3 pair used by the launched stack.
  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  # shellcheck disable=SC1091
  source "${INTEGRATION_ROOT}/install/local_setup.bash"
  # shellcheck disable=SC1091
  source "${PROJECT_ROOT}/ros2_ws/install/local_setup.bash"
  set -u
}

read -r expected_map_width expected_map_height < <(
  python3 - "${MAP_FILE}" <<'PY'
from pathlib import Path
import sys
import yaml

map_yaml = Path(sys.argv[1]).resolve()
metadata = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
image = (map_yaml.parent / str(metadata["image"])).resolve()
with image.open("rb") as stream:
    if stream.readline().strip() not in {b"P2", b"P5"}:
        raise SystemExit(f"unsupported occupancy image: {image}")
    dimensions = stream.readline()
    while dimensions.lstrip().startswith(b"#"):
        dimensions = stream.readline()
    width, height = (int(value) for value in dimensions.split())
print(width, height)
PY
)

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
  local env_file pid runtime_value
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

  # Some Nav2 lifecycle children can outlive an already-exited ros2 launch
  # parent while retaining the trial's environment and orphaned process group.
  # The ROS domain and runtime directory are unique per row, so terminate only
  # processes carrying this exact pair; never match another campaign by name.
  for env_file in /proc/[0-9]*/environ; do
    [[ -r "${env_file}" ]] || continue
    runtime_value="$(tr '\0' '\n' <"${env_file}" 2>/dev/null \
      | sed -n 's/^ISAAC_NAV_RUNTIME_DIR=//p')"
    [[ "${runtime_value}" == "${trial_runtime_dir}" ]] || continue
    grep -Fqz "ROS_DOMAIN_ID=${trial_domain_id}" "${env_file}" 2>/dev/null \
      || continue
    pid="${env_file#/proc/}"
    pid="${pid%/environ}"
    [[ "${pid}" == "$$" ]] || kill -TERM "${pid}" 2>/dev/null || true
  done
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
      source_attempt30_ros
      timeout 4 ros2 topic echo --once --no-daemon \
        /bio_nav/module2/planning_prior 2>/dev/null
    } || true)"
    grep -Fq "input_healthy: true" <<<"${sample}" && return 0
    sleep 1
  done
  echo "input-healthy PlanningPrior readiness timeout" >&2
  return 1
}

wait_for_global_costmap() {
  local pid="$1" log_file="$2" timeout_s="$3"
  local deadline=$((SECONDS + timeout_s))
  local sample
  while (( SECONDS < deadline )); do
    kill -0 "${pid}" 2>/dev/null || {
      echo "navigation stack exited before global costmap readiness" >&2
      return 1
    }
    sample="$({
      export ROS_DOMAIN_ID="${trial_domain_id}"
      source_attempt30_ros
      ros2 topic echo /global_costmap/costmap_raw nav2_msgs/msg/Costmap \
        --field metadata --once --timeout 4 --no-daemon 2>/dev/null
    } || true)"
    # Do not use a separate `ros2 lifecycle get` process as an active-state
    # oracle here: short-lived CLI discovery can report "Node not found" even
    # while the launch-owned node is healthy. The map_server activation log,
    # followed by StaticLayer receipt of this exact map and live 320-cell
    # metadata, is the launch-local causal chain required before dispatch.
    if grep -Fq '[map_server]: Activating' "${log_file}" \
        && grep -Fq \
          "StaticLayer: Resizing costmap to ${expected_map_width} X ${expected_map_height}" \
          "${log_file}" \
        && grep -Eq "^[[:space:]]*size_x: ${expected_map_width}[[:space:]]*$" <<<"${sample}" \
        && grep -Eq "^[[:space:]]*size_y: ${expected_map_height}[[:space:]]*$" <<<"${sample}"; then
      return 0
    fi
    sleep 1
  done
  echo "global costmap did not materialize the ${expected_map_width}x${expected_map_height} occupancy map" >&2
  return 1
}

reactivate_map_server() {
  local log_file="$1"
  {
    export ROS_DOMAIN_ID="${trial_domain_id}"
    source_attempt30_ros
    echo "MAP_RECOVERY_START domain=${trial_domain_id}"
    map_state="$(timeout 12 ros2 lifecycle get /map_server | awk '{print $1}')"
    echo "MAP_RECOVERY_STATE state=${map_state}"
    case "${map_state}" in
      active)
        timeout 12 ros2 lifecycle set /map_server deactivate
        timeout 12 ros2 lifecycle set /map_server activate
        ;;
      inactive)
        timeout 12 ros2 lifecycle set /map_server activate
        ;;
      unconfigured)
        timeout 12 ros2 lifecycle set /map_server configure
        timeout 12 ros2 lifecycle set /map_server activate
        ;;
      *)
        echo "unsupported map_server lifecycle state: ${map_state}" >&2
        return 1
        ;;
    esac
    echo "MAP_RECOVERY_COMPLETE domain=${trial_domain_id}"
  } >>"${log_file}" 2>&1
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
    ATTEMPT30_A21_AUDIT_JSONL_PATH="${trial_dir}/bridge_audit.jsonl" \
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

# Autostart can occasionally activate the static layer before DDS delivers the
# transient-local /map sample.  A merely active lifecycle state is therefore
# insufficient: dispatch only after the published global costmap has the same
# dimensions as the benchmark occupancy image.  One map-server reactivation is
# allowed as startup recovery; a second miss fails this fresh trial closed.
if ! wait_for_global_costmap "${ros_pid}" "${trial_dir}/logs/ros.log" 20; then
  echo "global costmap readiness missed; reactivating map_server once" \
    | tee -a "${trial_dir}/logs/map_recovery.log" >&2
  reactivate_map_server "${trial_dir}/logs/map_recovery.log"
  wait_for_global_costmap "${ros_pid}" "${trial_dir}/logs/ros.log" 40
fi

if [[ "${arm}" != "baseline" ]]; then
  wait_for_planning_prior_input "${prior_pid}" 240
fi

if [[ "${ATTEMPT30_A21_PREFLIGHT_ONLY:-false}" == "true" ]]; then
  python3 - "${trial_dir}/PREFLIGHT_COMPLETE.json" "${query_id}" "${arm}" \
    "${variant}" "${trial_domain_id}" "${expected_map_width}" \
    "${expected_map_height}" <<'PY'
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

path = Path(sys.argv[1])
payload = {
    "schema": "attempt30_a21_trial_preflight_v1",
    "classification": "engineering_startup_preflight_not_navigation_evidence",
    "status": "COMPLETE",
    "query_id": sys.argv[2],
    "experiment_arm": sys.argv[3],
    "dynamic_variant_id": sys.argv[4],
    "ros_domain_id": int(sys.argv[5]),
    "expected_map_size_cells": [int(sys.argv[6]), int(sys.argv[7])],
    "goal_dispatched": False,
    "timestamp_utc": datetime.now(timezone.utc).isoformat(),
}
path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
print(json.dumps(payload, sort_keys=True))
PY
  exit 0
fi

python3 - "${trial_dir}/TRIAL_DISPATCHED.json" "${query_id}" "${arm}" \
  "${variant}" "${trial_domain_id}" <<'PY'
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

Path(sys.argv[1]).write_text(json.dumps({
    "schema": "attempt30_a21_trial_dispatch_v1",
    "query_id": sys.argv[2],
    "experiment_arm": sys.argv[3],
    "dynamic_variant_id": sys.argv[4],
    "ros_domain_id": int(sys.argv[5]),
    "goal_dispatched": True,
    "timestamp_utc": datetime.now(timezone.utc).isoformat(),
}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY

set +e
(
  export ROS_DOMAIN_ID="${trial_domain_id}"
  export ATTEMPT30_A21_TRIAL_DOMAIN_ID="${trial_domain_id}"
  source_attempt30_ros
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
