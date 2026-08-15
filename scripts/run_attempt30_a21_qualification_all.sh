#!/usr/bin/env bash
# One-command V3.10 engineering supervisor for pilots and 3x20 A21 runs.
set -Eeuo pipefail

export ISAAC_NAV_EXPECTED_DOMAIN_ID="${ISAAC_NAV_EXPECTED_DOMAIN_ID:-151}"
export ROS_DOMAIN_ID="${ISAAC_NAV_EXPECTED_DOMAIN_ID}"
export ISAAC_NAV_RUNTIME_DIR="${ISAAC_NAV_RUNTIME_DIR:-/tmp/isaac_sim_ros2_nav_${UID}_a21_v310_q${ISAAC_NAV_EXPECTED_DOMAIN_ID}}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

campaign="${1:-$(date +%Y%m%d-%H%M%S)}"
[[ "${campaign}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || die "invalid campaign id"
orchestration_mode="${2:-full}"
[[ "${orchestration_mode}" == "full" \
    || "${orchestration_mode}" == "pilot-static" \
    || "${orchestration_mode}" == "diagnostic-direct-static" \
    || "${orchestration_mode}" == "diagnostic-dynamic-profile-static" \
    || "${orchestration_mode}" == "diagnostic-static-repeat" \
    || "${orchestration_mode}" == "diagnostic-dynamic-repeat" \
    || "${orchestration_mode}" == "pilot-dynamic" \
    || "${orchestration_mode}" == "run-dynamic" \
    || "${orchestration_mode}" == "sr-demo-structural" \
    || "${orchestration_mode}" == "sr-demo-medium" \
    || "${orchestration_mode}" == "sr-demo-sr-medium" ]] \
  || die "unsupported orchestration mode: ${orchestration_mode}"
control="${PROJECT_ROOT}/data/experiment_runs/attempt30_a21_${campaign}/orchestrator"
attempt30_integration_root="/home/lyb/Workspace/Bio_Nav/worktrees/integration/attempt30-a21-v310-srdr-rviz"
mkdir -p "${control}"
isaac_pid=""; ros_pid=""; module2_pid=""; prior_pid=""

stop_stack() {
  local component fallback pid pgid deadline
  for component in edge_prior module2 ros isaac; do
    case "${component}" in
      edge_prior) fallback="${prior_pid}" ;;
      module2) fallback="${module2_pid}" ;;
      ros) fallback="${ros_pid}" ;;
      isaac) fallback="${isaac_pid}" ;;
    esac
    pid="${fallback}"
    if [[ -r "$(runtime_pid_file "${component}")" ]]; then
      local recorded
      recorded="$(sed -n 's/^pid=//p' "$(runtime_pid_file "${component}")" | head -n 1)"
      [[ "${recorded}" =~ ^[0-9]+$ ]] && pid="${recorded}"
    fi
    [[ "${pid}" =~ ^[0-9]+$ ]] || continue
    pgid="$(current_process_group "${pid}" 2>/dev/null || true)"
    if [[ "${pgid}" == "${pid}" ]]; then
      kill -TERM -- "-${pgid}" 2>/dev/null || true
    else
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done

  deadline=$((SECONDS + 30))
  while ((SECONDS < deadline)); do
    local any_alive=0
    for pid in "${prior_pid}" "${module2_pid}" "${ros_pid}" "${isaac_pid}"; do
      if [[ "${pid}" =~ ^[0-9]+$ ]] \
          && { kill -0 "${pid}" 2>/dev/null || kill -0 -- "-${pid}" 2>/dev/null; }; then
        any_alive=1
      fi
    done
    ((any_alive == 0)) && break
    sleep 1
  done

  # The launchers run in dedicated groups. Escalation remains scoped to the
  # three exact leaders started by this supervisor.
  for pid in "${prior_pid}" "${module2_pid}" "${ros_pid}" "${isaac_pid}"; do
    [[ "${pid}" =~ ^[0-9]+$ ]] || continue
    if kill -0 "${pid}" 2>/dev/null || kill -0 -- "-${pid}" 2>/dev/null; then
      pgid="$(current_process_group "${pid}" 2>/dev/null || true)"
      if kill -0 -- "-${pid}" 2>/dev/null; then
        pgid="${pid}"
        kill -KILL -- "-${pgid}" 2>/dev/null || true
      elif [[ "${pgid}" == "${pid}" ]]; then
        kill -KILL -- "-${pgid}" 2>/dev/null || true
      else
        kill -KILL "${pid}" 2>/dev/null || true
      fi
    fi
    wait "${pid}" 2>/dev/null || true
  done
  isaac_pid=""; ros_pid=""; module2_pid=""; prior_pid=""
}
trap 'stop_stack' EXIT INT TERM HUP

start_stack() {
  local mode="$1" profile="${2:-}" spawn_pose="${3:-long_route_start_g1}"
  local guidance_profile="${4:-}"
  local module2_response_timeout_s="0.0"
  local nav2_profile_params_file=""
  [[ -z "${guidance_profile}" ]] || module2_response_timeout_s="5.0"
  [[ -z "${guidance_profile}" ]] || nav2_profile_params_file="${PROJECT_ROOT}/ros2_ws/src/robot_navigation/config/nav2_sr_guidance.yaml"
  local session_index=1 session_prefix
  local integration_root="${attempt30_integration_root}"
  local socket_path="${ISAAC_NAV_RUNTIME_DIR}/module2-v310.sock"
  while compgen -G "${control}/${mode}-session-${session_index}-*" >/dev/null; do
    session_index=$((session_index + 1))
  done
  session_prefix="${control}/${mode}-session-${session_index}"
  if [[ -z "${profile}" ]]; then
    profile="stable"
    [[ "${mode}" == "dynamic" ]] && profile="dynamic_avoidance"
  fi
  require_file "${integration_root}/scripts/run_module2_v310_server.sh"
  require_file "${integration_root}/ros2_ws/src/bio_nav_ros_bridge/launch/attempt30_a21_v310.launch.py"
  require_file "/home/lyb/Workspace/Bio_Nav/repos/MODULE2_SRDR_V310_MODULE3_HANDOFF_20260812/weights/module2_srdr_v310_seed20260822.pt"
  mkdir -p "${ISAAC_NAV_RUNTIME_DIR}"
  rm -f "${socket_path}"
  ISAAC_NAV_ATTEMPT30_SPAWN_POSE="${spawn_pose}" \
    ISAAC_NAV_STATIC_OBSTACLE_CONFIG="${guidance_profile:+${PROJECT_ROOT}/isaac_sim/configs/experiments/attempt30_a21_sr_guidance_static.yaml}" \
    "${SCRIPT_DIR}/run_kujiale_4x20_isaac.sh" "${mode}" --headless \
    >"${session_prefix}-isaac.log" 2>&1 & isaac_pid=$!
  "${SCRIPT_DIR}/run_ros.sh" navigation odometry_mode:=ideal \
    spawn_pose_name:="${spawn_pose}" nav2_profile:="${profile}" \
    nav2_profile_params_file:="${nav2_profile_params_file}" \
    module2_response_timeout_s:="${module2_response_timeout_s}" \
    interactive:=false use_rviz:=false \
    >"${session_prefix}-nav2.log" 2>&1 & ros_pid=$!
  BIO_NAV_SOCKET_PATH="${socket_path}" \
    "${integration_root}/scripts/run_module2_v310_server.sh" \
    >"${session_prefix}-module2.log" 2>&1 & module2_pid=$!
  local socket_deadline=$((SECONDS + 120))
  while ((SECONDS < socket_deadline)) && [[ ! -S "${socket_path}" ]]; do
    kill -0 "${module2_pid}" 2>/dev/null || die "Module2 exited; inspect ${session_prefix}-module2.log"
    sleep 1
  done
  [[ -S "${socket_path}" ]] || die "Module2 socket timed out: ${socket_path}"
  local -a edge_prior_arguments=(
    "socket_path:=${socket_path}"
    use_sim_time:=true
  )
  [[ -z "${guidance_profile}" ]] \
    || edge_prior_arguments+=(
      "guidance_profile:=${guidance_profile}"
      "goal_prior_retry_window_s:=4.5"
    )
  BIO_NAV_ATTEMPT30_V310_INTEGRATION_ROOT="${integration_root}" \
    "${SCRIPT_DIR}/run_attempt30_a21_edge_prior.sh" \
    "${edge_prior_arguments[@]}" \
    >"${session_prefix}-edge-prior.log" 2>&1 & prior_pid=$!
  local deadline=$((SECONDS + 900))
  while ((SECONDS < deadline)); do
    kill -0 "${isaac_pid}" 2>/dev/null || die "Isaac exited; inspect ${session_prefix}-isaac.log"
    kill -0 "${ros_pid}" 2>/dev/null || die "Nav2 exited; inspect ${session_prefix}-nav2.log"
    kill -0 "${module2_pid}" 2>/dev/null || die "Module2 exited; inspect ${session_prefix}-module2.log"
    kill -0 "${prior_pid}" 2>/dev/null || die "V3.10 bridge/edge/visualizer launch exited; inspect ${session_prefix}-edge-prior.log"
    if "${SCRIPT_DIR}/run_attempt30_a21_qualification.sh" preflight "${mode}" \
      >"${session_prefix}-preflight.log" 2>&1; then
      return 0
    fi
    sleep 5
  done
  tail -n 80 "${session_prefix}-preflight.log" >&2 || true
  die "${mode} stack preflight timed out"
}

# Pin ROS package discovery to the same Integration worktree used below for
# Module2 launchers.  Without this explicit underlay, a caller's ambient shell
# can resolve bio_nav_ros_bridge from another checkout and silently load a
# missing or stale A21 engineering-defaults file.
"${attempt30_integration_root}/scripts/build_ros_bridge.sh"
require_file "${attempt30_integration_root}/install/local_setup.bash"
require_file "${attempt30_integration_root}/install/bio_nav_ros_bridge/share/bio_nav_ros_bridge/config/engineering_defaults.yaml"
# Source the base ROS and Module3 workspace first.  Then place the exact
# Attempt30 Integration overlay last; otherwise the Module3-generated prefix
# chain can re-promote another bio_nav_ros_bridge checkout after provenance was
# apparently pinned.
source_ros --require-workspace
set +u
# shellcheck disable=SC1091
source "${attempt30_integration_root}/install/local_setup.bash"
set -u
[[ "$(ros2 pkg prefix bio_nav_ros_bridge 2>/dev/null)" == \
    "${attempt30_integration_root}/install/bio_nav_ros_bridge" ]] \
  || die "bio_nav_ros_bridge did not resolve from the Attempt30 Integration worktree"
(
  cd "${PROJECT_ROOT}/ros2_ws"
  colcon build --symlink-install \
    --packages-select robot_experiments robot_route_planner robot_navigation robot_bringup \
    --allow-overriding robot_experiments robot_route_planner robot_navigation robot_bringup
)
if [[ "${orchestration_mode}" == "sr-demo-structural" \
    || "${orchestration_mode}" == "sr-demo-medium" \
    || "${orchestration_mode}" == "sr-demo-sr-medium" ]]; then
  guidance_profile="structural"
  [[ "${orchestration_mode}" == "sr-demo-medium" ]] \
    && guidance_profile="medium"
  [[ "${orchestration_mode}" == "sr-demo-sr-medium" ]] \
    && guidance_profile="sr_medium"
  start_stack static stable long_route_start_g2 "${guidance_profile}"
  "${SCRIPT_DIR}/run_experiment.sh" \
    "${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/attempt30_a21_v310_sr_guidance_demo.yaml" \
    "${PROJECT_ROOT}/data/experiment_runs/attempt30_a21_guidance_${campaign}/${guidance_profile}" \
    navigation_execution_backend:=route_guided \
    record_bag:=false record_evidence:=true nav2_profile:=stable \
    require_module2_planning_ready:=true \
    module2_planning_ready_timeout_sec:=30.0 \
    resume:=false run_indices:=1
  demo_root="${PROJECT_ROOT}/data/experiment_runs/attempt30_a21_guidance_${campaign}/${guidance_profile}/attempt30_a21_v310_sr_guidance_demo"
  python3 - "${demo_root}" "${guidance_profile}" <<'PY'
import json
import pathlib
import sys

summaries = sorted(pathlib.Path(sys.argv[1]).glob("run-*/run_summary.json"))
if len(summaries) != 1:
    raise SystemExit(f"SR guidance demo expected one summary, found {len(summaries)}")
summary = summaries[0]
value = json.loads(summary.read_text())
profile = sys.argv[2]
required = (
    value.get("strict_success"),
    value.get("physical_collision_free"),
    value.get("data_complete"),
    value.get("localization_healthy"),
)
if not all(required):
    raise SystemExit(f"SR guidance demo failed strict evidence gates: {summary}")
responses = value.get("module2_health", {}).get("responses", [])
target_legs = [leg for leg in value.get("legs", []) if leg.get("id") == "SR_GOAL_NODE_11"]
if len(target_legs) != 1:
    raise SystemExit(f"SR guidance demo target leg is missing or duplicated: {summary}")
target_request = int(target_legs[0].get("route_request_id", -1))
target_responses = [
    item for item in responses if int(item.get("request_id", -2)) == target_request
]
if len(target_responses) != 1 or target_responses[0].get("healthy") is not True:
    raise SystemExit(f"SR guidance target did not use a healthy prior: {summary}")
if profile != "structural" and int(target_responses[0].get("positive_cost_count", 0)) <= 0:
    raise SystemExit(f"SR guidance demo active profile applied no positive edge costs: {summary}")
PY
  stop_stack
  exit 0
fi
if [[ "${orchestration_mode}" == "diagnostic-dynamic-profile-static" ]]; then
  start_stack static dynamic_avoidance
  "${SCRIPT_DIR}/run_experiment.sh" \
    "${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/attempt30_a21_qualification_static.yaml" \
    "${PROJECT_ROOT}/data/experiment_runs/attempt30_a21_diagnostic_${campaign}/dynamic_profile_static" \
    navigation_execution_backend:=route_guided \
    record_bag:=false record_evidence:=true nav2_profile:=dynamic_avoidance \
    resume:=false run_indices:=1
  stop_stack
  exit 0
fi
if [[ "${orchestration_mode}" == "pilot-dynamic" \
    || "${orchestration_mode}" == "run-dynamic" ]]; then
  start_stack dynamic
  if [[ "${orchestration_mode}" == "pilot-dynamic" ]]; then
    "${SCRIPT_DIR}/run_attempt30_a21_qualification.sh" pilot dynamic "${campaign}"
  else
    "${SCRIPT_DIR}/run_attempt30_a21_qualification.sh" run dynamic "${campaign}"
  fi
  stop_stack
  exit 0
fi
if [[ "${orchestration_mode}" == "diagnostic-dynamic-repeat" ]]; then
  # Exercise five consecutive resets in one long-lived stack.  This is
  # deliberately stored outside the formal campaign tree: it diagnoses
  # temporal voxel decay and reset isolation without creating qualification
  # rows or weakening the immutable fail-stop campaign.
  start_stack dynamic
  "${SCRIPT_DIR}/run_experiment.sh" \
    "${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/attempt30_a21_qualification_dynamic.yaml" \
    "${PROJECT_ROOT}/data/experiment_runs/attempt30_a21_diagnostic_${campaign}/dynamic_repeat" \
    navigation_execution_backend:=route_guided \
    record_bag:=false record_evidence:=true nav2_profile:=dynamic_avoidance \
    resume:=false run_indices:=1,2,3,4,5
  stop_stack
  exit 0
fi
start_stack static
if [[ "${orchestration_mode}" == "diagnostic-static-repeat" ]]; then
  # Match the formal stack history: its pilot consumes one mission/reset
  # before formal indices 1--8.  Keep the warmup and repeats non-formal while
  # preserving the original full-house route and six static obstacles.
  "${SCRIPT_DIR}/run_experiment.sh" \
    "${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/attempt30_a21_qualification_static.yaml" \
    "${PROJECT_ROOT}/data/experiment_runs/attempt30_a21_diagnostic_${campaign}/static_warmup" \
    navigation_execution_backend:=route_guided \
    record_bag:=false record_evidence:=true nav2_profile:=stable \
    resume:=false run_indices:=1
  "${SCRIPT_DIR}/run_experiment.sh" \
    "${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/attempt30_a21_qualification_static.yaml" \
    "${PROJECT_ROOT}/data/experiment_runs/attempt30_a21_diagnostic_${campaign}/static_repeat" \
    navigation_execution_backend:=route_guided \
    record_bag:=false record_evidence:=true nav2_profile:=stable \
    resume:=false run_indices:=1,2,3,4,5,6,7,8
  stop_stack
  exit 0
fi
if [[ "${orchestration_mode}" == "diagnostic-direct-static" ]]; then
  "${SCRIPT_DIR}/run_attempt30_a21_qualification.sh" preflight static
  "${SCRIPT_DIR}/run_experiment.sh" \
    "${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/attempt30_a21_qualification_static.yaml" \
    "${PROJECT_ROOT}/data/experiment_runs/attempt30_a21_diagnostic_direct_${campaign}/static" \
    navigation_execution_backend:=navigate_to_pose \
    record_bag:=false record_evidence:=true nav2_profile:=stable \
    resume:=false run_indices:=1
  stop_stack
  exit 0
fi
"${SCRIPT_DIR}/run_attempt30_a21_qualification.sh" pilot static "${campaign}"
if [[ "${orchestration_mode}" == "pilot-static" ]]; then
  stop_stack
  exit 0
fi
"${SCRIPT_DIR}/run_attempt30_a21_qualification.sh" run static "${campaign}"
"${SCRIPT_DIR}/run_attempt30_a21_qualification.sh" pilot appearance "${campaign}"
"${SCRIPT_DIR}/run_attempt30_a21_qualification.sh" run appearance "${campaign}"
stop_stack

start_stack dynamic
"${SCRIPT_DIR}/run_attempt30_a21_qualification.sh" pilot dynamic "${campaign}"
"${SCRIPT_DIR}/run_attempt30_a21_qualification.sh" run dynamic "${campaign}"
stop_stack

"${SCRIPT_DIR}/run_attempt30_a21_qualification.sh" report "${campaign}"
