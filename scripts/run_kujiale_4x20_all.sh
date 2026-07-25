#!/usr/bin/env bash
# One-command supervisor for the full Kujiale 4x20 campaign.  It owns only the
# two background stack supervisors; each stage's formal runner stays evidence
# first and the static/dynamic stacks are never reused across the boundary.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

usage() {
  cat <<'USAGE'
usage: run_kujiale_4x20_all.sh [CAMPAIGN_ID] [--resume] [--skip-build] [--startup-timeout-sec SECONDS]

Runs, in order: static pilot, static 40 rounds, controlled shutdown, dynamic
pilot, dynamic 40 rounds, controlled shutdown, then the visual report.  The
same CAMPAIGN_ID is used throughout; omit it to generate YYYYMMDD-HHMMSS.
USAGE
}

campaign_id="${CAMPAIGN_ID:-$(date +%Y%m%d-%H%M%S)}"
resume=false
build_workspace=true
startup_timeout_sec=900

if [[ $# -gt 0 && "${1}" != --* ]]; then
  campaign_id="$1"
  shift
fi
while (($#)); do
  case "$1" in
    --resume) resume=true ;;
    --skip-build) build_workspace=false ;;
    --startup-timeout-sec)
      shift
      [[ $# -gt 0 ]] || die "--startup-timeout-sec requires a positive integer"
      startup_timeout_sec="$1"
      ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; die "unknown argument: $1" ;;
  esac
  shift
done

[[ "${campaign_id}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] \
  || die "invalid CAMPAIGN_ID: ${campaign_id}"
[[ "${startup_timeout_sec}" =~ ^[1-9][0-9]*$ ]] \
  || die "startup timeout must be a positive integer"

run_root="${PROJECT_ROOT}/data/experiment_runs/kujiale_4x20_${campaign_id}"
report_root="${PROJECT_ROOT}/data/reports/kujiale_4x20_${campaign_id}"
control_root="${run_root}/orchestrator"
isaac_pid=""
ros_pid=""
active_mode=""

pid_is_running() {
  local pid="$1"
  [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null
}

dedicated_process_group_for() {
  local pid="$1" process_group
  process_group="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
  # run_isaac.sh and run_ros.sh promise to re-exec into their own sessions.
  # Refuse to signal a shared group if that invariant was not established.
  [[ "${process_group}" == "${pid}" ]] && printf '%s' "${process_group}"
}

ros_launch_process_group_for() {
  local supervisor_pid="$1"
  # run_ros.sh places `ros2 launch` in a child session.  Signal that group,
  # rather than only the shell waiting on it, so Nav2 receives SIGINT too.
  ps -eo pid=,ppid=,pgid=,stat= | awk -v parent="${supervisor_pid}" '
    $2 == parent && $1 == $3 && $4 !~ /^Z/ { print $3; exit }
  '
}

stop_stage() {
  local process_group status=0
  [[ -n "${active_mode}" ]] || return 0
  log_info "stopping ${active_mode} Nav2 supervisor"
  if pid_is_running "${ros_pid}"; then
    process_group="$(ros_launch_process_group_for "${ros_pid}" || true)"
    if [[ "${process_group}" =~ ^[1-9][0-9]*$ ]]; then
      log_info "stopping ${active_mode} ROS launch process group ${process_group}"
      kill -INT -- "-${process_group}" 2>/dev/null || true
    else
      log_warn "${active_mode} ROS launch group was unavailable; signaling supervisor ${ros_pid}"
      kill -INT "${ros_pid}" 2>/dev/null || true
    fi
    wait "${ros_pid}" || status=$?
  fi
  log_info "stopping ${active_mode} Isaac supervisor"
  if pid_is_running "${isaac_pid}"; then
    process_group="$(dedicated_process_group_for "${isaac_pid}" || true)"
    if [[ "${process_group}" =~ ^[1-9][0-9]*$ ]]; then
      log_info "stopping ${active_mode} Isaac process group ${process_group}"
      kill -INT -- "-${process_group}" 2>/dev/null || true
    else
      log_warn "${active_mode} Isaac process group was unavailable; signaling supervisor ${isaac_pid}"
      kill -INT "${isaac_pid}" 2>/dev/null || true
    fi
    wait "${isaac_pid}" || status=$?
  fi
  isaac_pid=""
  ros_pid=""
  active_mode=""
  return 0
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM HUP
  stop_stage || true
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

wait_for_preflight() {
  local mode="$1" deadline log_file
  deadline=$((SECONDS + startup_timeout_sec))
  log_file="${control_root}/${mode}-preflight.log"
  while (( SECONDS < deadline )); do
    pid_is_running "${isaac_pid}" || die "${mode} Isaac supervisor exited; inspect ${control_root}/${mode}-isaac.log"
    pid_is_running "${ros_pid}" || die "${mode} Nav2 supervisor exited; inspect ${control_root}/${mode}-nav2.log"
    if "${SCRIPT_DIR}/run_kujiale_4x20.sh" preflight "${mode}" >"${log_file}" 2>&1; then
      cat "${log_file}"
      return 0
    fi
    sleep 5
  done
  tail -n 80 "${log_file}" >&2 || true
  die "${mode} stage did not satisfy preflight within ${startup_timeout_sec}s"
}

start_stage() {
  local mode="$1" nav2_profile
  [[ -z "${active_mode}" ]] || die "cannot start ${mode}; ${active_mode} stage is still active"
  nav2_profile="stable"
  [[ "${mode}" == "dynamic" ]] && nav2_profile="dynamic_avoidance"
  log_info "starting ${mode} Isaac supervisor; log=${control_root}/${mode}-isaac.log"
  # These launchers establish and supervise their own dedicated process groups.
  # Keep them as direct children so their PIDs remain waitable by stop_stage;
  # wrapping them in a second `setsid` can make $! refer to a short-lived
  # helper and leave the actual Isaac/ROS stack orphaned after Ctrl+C.
  "${SCRIPT_DIR}/run_kujiale_4x20_isaac.sh" "${mode}" --headless \
    >"${control_root}/${mode}-isaac.log" 2>&1 &
  isaac_pid=$!
  log_info "starting ${mode} Nav2 supervisor; log=${control_root}/${mode}-nav2.log"
  "${SCRIPT_DIR}/run_ros.sh" navigation \
    odometry_mode:=ideal spawn_pose_name:=long_route_start_g1 \
    nav2_profile:="${nav2_profile}" interactive:=false use_rviz:=false \
    >"${control_root}/${mode}-nav2.log" 2>&1 &
  ros_pid=$!
  active_mode="${mode}"
  wait_for_preflight "${mode}"
}

run_campaign() {
  local command_name="$1" mode=""
  shift
  if [[ "${command_name}" == "pilot" ]]; then
    [[ $# -eq 1 ]] || die "pilot requires static or dynamic mode"
    mode="$1"
  elif [[ $# -ne 0 ]]; then
    die "${command_name} does not accept a mode"
  fi
  if [[ "${resume}" == true ]]; then
    if [[ -n "${mode}" ]]; then
      "${SCRIPT_DIR}/run_kujiale_4x20.sh" "${command_name}" "${mode}" "${campaign_id}" --resume
    else
      "${SCRIPT_DIR}/run_kujiale_4x20.sh" "${command_name}" "${campaign_id}" --resume
    fi
  else
    if [[ -n "${mode}" ]]; then
      "${SCRIPT_DIR}/run_kujiale_4x20.sh" "${command_name}" "${mode}" "${campaign_id}"
    else
      "${SCRIPT_DIR}/run_kujiale_4x20.sh" "${command_name}" "${campaign_id}"
    fi
  fi
}

[[ ! -e "${run_root}" || "${resume}" == true ]] \
  || die "campaign directory already exists: ${run_root}; use --resume or a new CAMPAIGN_ID"
[[ ! -e "${report_root}" ]] \
  || die "report directory already exists: ${report_root}; choose a new CAMPAIGN_ID"
mkdir -p "${control_root}"

if [[ "${build_workspace}" == true ]]; then
  log_info "building ROS workspace before campaign"
  "${SCRIPT_DIR}/build_ros2.sh"
fi

log_info "starting one-command Kujiale 4x20 campaign=${campaign_id}"
start_stage static
run_campaign pilot static
run_campaign static-pair
stop_stage

start_stage dynamic
run_campaign pilot dynamic
run_campaign dynamic-pair
stop_stage

"${SCRIPT_DIR}/run_kujiale_4x20.sh" status "${campaign_id}"
set +e
"${SCRIPT_DIR}/run_kujiale_4x20.sh" report "${campaign_id}"
report_status=$?
set -e
if [[ "${report_status}" -eq 0 ]]; then
  log_info "4x20 campaign passed; open ${report_root}/index.html"
elif [[ "${report_status}" -eq 2 ]]; then
  log_warn "4x20 campaign completed but did not satisfy every gate; report: ${report_root}/index.html"
else
  die "4x20 report generation failed (exit ${report_status}); inspect ${run_root}"
fi
exit "${report_status}"
