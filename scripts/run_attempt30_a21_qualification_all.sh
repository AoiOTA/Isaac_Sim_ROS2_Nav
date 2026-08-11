#!/usr/bin/env bash
# One-command supervisor for pilot + exact 60-run A21 Final Qualification.
set -Eeuo pipefail

export ISAAC_NAV_EXPECTED_DOMAIN_ID="${ISAAC_NAV_EXPECTED_DOMAIN_ID:-150}"
export ROS_DOMAIN_ID="${ISAAC_NAV_EXPECTED_DOMAIN_ID}"
export ISAAC_NAV_RUNTIME_DIR="${ISAAC_NAV_RUNTIME_DIR:-/tmp/isaac_sim_ros2_nav_${UID}_a21_q${ISAAC_NAV_EXPECTED_DOMAIN_ID}}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

campaign="${1:-$(date +%Y%m%d-%H%M%S)}"
[[ "${campaign}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || die "invalid campaign id"
orchestration_mode="${2:-full}"
[[ "${orchestration_mode}" == "full" \
    || "${orchestration_mode}" == "pilot-static" \
    || "${orchestration_mode}" == "diagnostic-direct-static" \
    || "${orchestration_mode}" == "pilot-dynamic" \
    || "${orchestration_mode}" == "run-dynamic" ]] \
  || die "mode must be full, pilot-static, diagnostic-direct-static, pilot-dynamic or run-dynamic"
control="${PROJECT_ROOT}/data/experiment_runs/attempt30_a21_${campaign}/orchestrator"
mkdir -p "${control}"
isaac_pid=""; ros_pid=""; module2_pid=""; bridge_pid=""; goal_prior_pid=""; prior_pid=""

stop_stack() {
  local component fallback pid pgid deadline
  for component in edge_prior goal_prior module2_bridge module2 ros isaac; do
    case "${component}" in
      edge_prior) fallback="${prior_pid}" ;;
      goal_prior) fallback="${goal_prior_pid}" ;;
      module2_bridge) fallback="${bridge_pid}" ;;
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
    for pid in "${prior_pid}" "${goal_prior_pid}" "${bridge_pid}" "${module2_pid}" "${ros_pid}" "${isaac_pid}"; do
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
  for pid in "${prior_pid}" "${goal_prior_pid}" "${bridge_pid}" "${module2_pid}" "${ros_pid}" "${isaac_pid}"; do
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
  isaac_pid=""; ros_pid=""; module2_pid=""; bridge_pid=""; goal_prior_pid=""; prior_pid=""
}
trap 'stop_stack' EXIT INT TERM HUP

start_stack() {
  local mode="$1" profile="stable"
  local integration_root="/home/lyb/Workspace/Bio_Nav/worktrees/integration/attempt30-a21-gvg-route"
  local module2_runtime_root="/home/lyb/Workspace/Bio_Nav/worktrees/module2/attempt30-a21-runtime-fbfdc8d"
  local asset_root="/home/lyb/Workspace/Bio_Nav/artifacts/releases/isaac-nav-fusion-v0.1.0-engineering"
  local socket_path="${ISAAC_NAV_RUNTIME_DIR}/module2.sock"
  local snapshot_map_version
  [[ "${mode}" == "dynamic" ]] && profile="dynamic_avoidance"
  require_file "${asset_root}/HYDRATION_COMPLETE.json"
  require_file "${asset_root}/sr-snapshot/manifest.json"
  require_file "${integration_root}/scripts/run_module2_server.sh"
  require_file "${integration_root}/scripts/run_ros_bridge.sh"
  require_file "${integration_root}/scripts/run_goal_prior_bridge.sh"
  require_file "${module2_runtime_root}/src/module2_srdr_pdf_v30/isaac_head_adapter.py"
  snapshot_map_version="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["map_version"])' "${asset_root}/sr-snapshot/manifest.json")"
  mkdir -p "${ISAAC_NAV_RUNTIME_DIR}"
  rm -f "${socket_path}"
  "${SCRIPT_DIR}/run_kujiale_4x20_isaac.sh" "${mode}" --headless \
    >"${control}/${mode}-isaac.log" 2>&1 & isaac_pid=$!
  "${SCRIPT_DIR}/run_ros.sh" navigation odometry_mode:=ideal \
    spawn_pose_name:=long_route_start_g1 nav2_profile:="${profile}" \
    interactive:=false use_rviz:=false \
    >"${control}/${mode}-nav2.log" 2>&1 & ros_pid=$!
  BIO_NAV_SOCKET_PATH="${socket_path}" \
    "${integration_root}/scripts/run_module2_server.sh" \
    --module2-root "${module2_runtime_root}" \
    --isaac-head-adapter "${asset_root}/isaac-risk-adapter/isaac_head.pt" \
    --risk-adapter "${asset_root}/isaac-risk-adapter/risk_calibration.json" \
    --allow-synthetic-isaac-adapter --force-reference-mamba \
    >"${control}/${mode}-module2.log" 2>&1 & module2_pid=$!
  local socket_deadline=$((SECONDS + 120))
  while ((SECONDS < socket_deadline)) && [[ ! -S "${socket_path}" ]]; do
    kill -0 "${module2_pid}" 2>/dev/null || die "Module2 exited; inspect ${control}/${mode}-module2.log"
    sleep 1
  done
  [[ -S "${socket_path}" ]] || die "Module2 socket timed out: ${socket_path}"
  "${integration_root}/scripts/run_ros_bridge.sh" \
    -p use_sim_time:=true -p "socket_path:=${socket_path}" \
    -p motion_source:=pose_delta -p suppress_unhealthy_frames:=true \
    -p "audit_jsonl_path:=${control}/${mode}-module2-audit.jsonl" \
    >"${control}/${mode}-module2-bridge.log" 2>&1 & bridge_pid=$!
  "${integration_root}/scripts/run_goal_prior_bridge.sh" \
    -p use_sim_time:=true -p "snapshot_path:=${asset_root}/sr-snapshot" \
    -p max_prior_age_s:=0.75 \
    >"${control}/${mode}-goal-prior.log" 2>&1 & goal_prior_pid=$!
  "${SCRIPT_DIR}/run_attempt30_a21_edge_prior.sh" \
    "goal_prior_map_version:=${snapshot_map_version}" \
    >"${control}/${mode}-edge-prior.log" 2>&1 & prior_pid=$!
  local deadline=$((SECONDS + 900))
  while ((SECONDS < deadline)); do
    kill -0 "${isaac_pid}" 2>/dev/null || die "Isaac exited; inspect ${control}/${mode}-isaac.log"
    kill -0 "${ros_pid}" 2>/dev/null || die "Nav2 exited; inspect ${control}/${mode}-nav2.log"
    kill -0 "${module2_pid}" 2>/dev/null || die "Module2 exited; inspect ${control}/${mode}-module2.log"
    kill -0 "${bridge_pid}" 2>/dev/null || die "Module2 ROS bridge exited; inspect ${control}/${mode}-module2-bridge.log"
    kill -0 "${goal_prior_pid}" 2>/dev/null || die "goal prior exited; inspect ${control}/${mode}-goal-prior.log"
    kill -0 "${prior_pid}" 2>/dev/null || die "edge prior bridge exited; inspect ${control}/${mode}-edge-prior.log"
    if "${SCRIPT_DIR}/run_attempt30_a21_qualification.sh" preflight "${mode}" \
      >"${control}/${mode}-preflight.log" 2>&1; then
      return 0
    fi
    sleep 5
  done
  tail -n 80 "${control}/${mode}-preflight.log" >&2 || true
  die "${mode} stack preflight timed out"
}

source_ros --require-workspace
(
  cd "${PROJECT_ROOT}/ros2_ws"
  colcon build --symlink-install \
    --packages-select robot_experiments robot_route_planner robot_navigation robot_bringup \
    --allow-overriding robot_experiments robot_route_planner robot_navigation robot_bringup
)
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
start_stack static
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
