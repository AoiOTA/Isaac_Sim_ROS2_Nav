#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

original_args=("$@")

operation="${1:-}"
[[ -n "${operation}" ]] || die "usage: $0 mapping|incremental_mapping|localization|navigation [launch arguments...]"
shift

case "${operation}" in
  mapping|incremental_mapping|localization|navigation) ;;
  *) die "unsupported operation: ${operation}" ;;
esac

source_ros --require-workspace
if [[ "${operation}" == "localization" || "${operation}" == "navigation" ]]; then
  if runtime_lock_is_held teleop; then
    die "stop the Mapping teleop before starting ${operation}; it must not publish /cmd_vel in this mode"
  fi
fi
ensure_dedicated_process_group "${original_args[@]}"
start_runtime_session
acquire_instance_lock ros "ROS stack"
trap 'release_instance_lock ros || true' EXIT
export ISAAC_NAV_SPAWN_POSES="${ISAAC_NAV_SPAWN_POSES:-${PROJECT_ROOT}/isaac_sim/configs/spawn_poses.yaml}"

launch_args=("$@")
if [[ "${operation}" != "mapping" ]]; then
  posegraph_file=""
  map_file=""
  map_manifest_file=""
  for argument in "${launch_args[@]}"; do
    case "${argument}" in
      posegraph_file:=*) posegraph_file="${argument#posegraph_file:=}" ;;
      map_file:=*) map_file="${argument#map_file:=}" ;;
      map_manifest_file:=*) map_manifest_file="${argument#map_manifest_file:=}" ;;
    esac
  done
  if [[ ("${operation}" == "localization" || "${operation}" == "navigation") \
      && -z "${map_file}" && -n "${posegraph_file}" ]]; then
    posegraph_prefix="${posegraph_file%.posegraph}"
    posegraph_prefix="${posegraph_prefix%.data}"
    map_file="${PROJECT_ROOT}/data/maps/occupancy/$(basename "${posegraph_prefix}").yaml"
    require_file "${map_file}"
    launch_args+=("map_file:=${map_file}")
  fi
  if [[ -z "${map_manifest_file}" && -n "${posegraph_file}" ]]; then
    posegraph_prefix="${posegraph_file%.posegraph}"
    posegraph_prefix="${posegraph_prefix%.data}"
    map_manifest_file="${PROJECT_ROOT}/data/maps/manifests/$(basename "${posegraph_prefix}").yaml"
    require_file "${map_manifest_file}"
    launch_args+=("map_manifest_file:=${map_manifest_file}")
  fi
fi

require_command env
require_command timeout

launch_pid=""
launch_start_ticks=""
lifecycle_pid=""
lifecycle_start_ticks=""
shutdown_signal=""
shutdown_deadline_ms=0
force_requested=false
shutdown_timeout_seconds="${ISAAC_NAV_SHUTDOWN_TIMEOUT_SECONDS:-20}"
lifecycle_timeout_seconds="${ISAAC_NAV_LIFECYCLE_SHUTDOWN_SECONDS:-10}"
shutdown_int_checks="${ISAAC_NAV_SHUTDOWN_INT_CHECKS:-50}"
shutdown_term_checks="${ISAAC_NAV_SHUTDOWN_TERM_CHECKS:-30}"
shutdown_kill_checks="${ISAAC_NAV_SHUTDOWN_KILL_CHECKS:-20}"
for numeric_setting in \
  shutdown_timeout_seconds lifecycle_timeout_seconds \
  shutdown_int_checks shutdown_term_checks shutdown_kill_checks; do
  numeric_value="${!numeric_setting}"
  [[ "${numeric_value}" =~ ^[1-9][0-9]*$ ]] \
    || die "${numeric_setting} must be a positive integer"
done

declare -Ag managed_process_groups=()
declare -Ag managed_process_group_starts=()

monotonic_millis() {
  local uptime whole fraction
  read -r uptime _ </proc/uptime
  whole="${uptime%%.*}"
  fraction="${uptime#*.}000"
  fraction="${fraction:0:3}"
  printf '%s\n' "$((10#${whole} * 1000 + 10#${fraction}))"
}

ensure_shutdown_deadline() {
  if ((shutdown_deadline_ms == 0)); then
    shutdown_deadline_ms="$(monotonic_millis)"
    shutdown_deadline_ms=$((
      shutdown_deadline_ms + shutdown_timeout_seconds * 1000
    ))
    log_info "shutdown deadline: ${shutdown_timeout_seconds}s total"
  fi
}

remaining_shutdown_timeout() {
  local cap_seconds="$1"
  local now remaining cap_ms
  ensure_shutdown_deadline
  now="$(monotonic_millis)"
  # Reserve the final second for process-group escalation and metadata cleanup.
  remaining=$((shutdown_deadline_ms - now - 1000))
  ((remaining > 0)) || return 1
  cap_ms=$((cap_seconds * 1000))
  ((remaining <= cap_ms)) || remaining="${cap_ms}"
  printf '%d.%03d\n' "$((remaining / 1000))" "$((remaining % 1000))"
}

shutdown_deadline_has_time() {
  ((shutdown_deadline_ms == 0)) && return 0
  (($(monotonic_millis) < shutdown_deadline_ms))
}

collect_managed_process_groups() {
  local component process_group pid_file leader_start_ticks
  for component in rviz teleop; do
    [[ -z "${managed_process_groups[${component}]:-}" ]] || continue
    process_group="$(
      runtime_registered_process_group \
        "${component}" "${ISAAC_NAV_SESSION_ID}" || true
    )"
    if [[ -n "${process_group}" ]]; then
      pid_file="$(runtime_pid_file "${component}")"
      leader_start_ticks="$(
        runtime_metadata_value "${pid_file}" leader_start_ticks || true
      )"
      [[ "${leader_start_ticks}" =~ ^[0-9]+$ ]] || {
        log_warn "refusing cached ${component} group without a start identity"
        continue
      }
      managed_process_groups["${component}"]="${process_group}"
      managed_process_group_starts["${component}"]="${leader_start_ticks}"
      log_info "registered managed ${component} process group ${process_group}"
    fi
  done
}

owned_group_is_running() {
  local process_group="$1"
  local expected_start_ticks="${2:-}"
  local actual_start_ticks actual_group
  runtime_process_group_is_running "${process_group}" || return 1
  # If the original leader is still present, pin the cached PGID to its
  # immutable /proc start time.  Descendant-only groups remain controllable
  # through the per-member project/session authentication below.
  if [[ -n "${expected_start_ticks}" ]] \
      && runtime_process_is_running "${process_group}"; then
    actual_start_ticks="$(
      runtime_process_start_ticks "${process_group}" || true
    )"
    actual_group="$(current_process_group "${process_group}" || true)"
    if [[ "${actual_start_ticks}" != "${expected_start_ticks}" \
          || "${actual_group}" != "${process_group}" ]]; then
      log_warn "refusing reused process group identity ${process_group}"
      return 1
    fi
  fi
  runtime_process_group_is_owned_by_session \
    "${process_group}" "${PROJECT_ROOT}" "${ISAAC_NAV_SESSION_ID}"
}

launch_group_is_running() {
  [[ -n "${launch_pid}" ]] || return 1
  owned_group_is_running "${launch_pid}" "${launch_start_ticks}"
}

signal_owned_group() {
  local label="$1"
  local process_group="$2"
  local signal_name="$3"
  local expected_start_ticks="${4:-}"
  owned_group_is_running \
    "${process_group}" "${expected_start_ticks}" || return 0
  log_info "stopping ${label} process group ${process_group} with SIG${signal_name}"
  kill "-${signal_name}" -- "-${process_group}" 2>/dev/null || true
}

signal_launch_group() {
  local signal_name="$1"
  [[ -n "${launch_pid}" ]] || return 0
  signal_owned_group \
    "ROS launch" "${launch_pid}" "${signal_name}" "${launch_start_ticks}"
}

signal_lifecycle_group() {
  local signal_name="$1"
  [[ -n "${lifecycle_pid}" ]] || return 0
  signal_owned_group \
    "ordered lifecycle helper" "${lifecycle_pid}" "${signal_name}" \
    "${lifecycle_start_ticks}"
}

signal_all_owned_groups() {
  local signal_name="$1"
  local component process_group
  collect_managed_process_groups
  for component in rviz teleop; do
    process_group="${managed_process_groups[${component}]:-}"
    [[ -n "${process_group}" ]] || continue
    signal_owned_group \
      "managed ${component}" "${process_group}" "${signal_name}" \
      "${managed_process_group_starts[${component}]:-}"
  done
  signal_lifecycle_group "${signal_name}"
  signal_launch_group "${signal_name}"
}

owned_groups_are_running() {
  local component process_group
  collect_managed_process_groups
  for component in rviz teleop; do
    process_group="${managed_process_groups[${component}]:-}"
    if [[ -n "${process_group}" ]] \
        && owned_group_is_running \
          "${process_group}" \
          "${managed_process_group_starts[${component}]:-}"; then
      return 0
    fi
  done
  if [[ -n "${lifecycle_pid}" ]] \
      && owned_group_is_running \
        "${lifecycle_pid}" "${lifecycle_start_ticks}"; then
    return 0
  fi
  launch_group_is_running
}

wait_for_owned_groups_exit() {
  local attempts="$1"
  local index
  for ((index = 0; index < attempts; index++)); do
    owned_groups_are_running || return 0
    shutdown_deadline_has_time || return 1
    sleep 0.1
  done
  ! owned_groups_are_running
}

cleanup_managed_metadata() {
  local component
  for component in rviz teleop; do
    remove_runtime_session_metadata \
      "${component}" "${ISAAC_NAV_SESSION_ID}" || true
  done
}

shutdown_owned_process_groups() {
  collect_managed_process_groups
  signal_all_owned_groups INT
  if wait_for_owned_groups_exit "${shutdown_int_checks}"; then
    cleanup_managed_metadata
    return 0
  fi
  log_warn "managed runtime groups ignored SIGINT; escalating to SIGTERM"
  signal_all_owned_groups TERM
  if wait_for_owned_groups_exit "${shutdown_term_checks}"; then
    cleanup_managed_metadata
    return 0
  fi
  log_warn "managed runtime groups ignored SIGTERM; escalating to SIGKILL"
  signal_all_owned_groups KILL
  wait_for_owned_groups_exit "${shutdown_kill_checks}" \
    || log_warn "a managed runtime group is still visible at the shutdown deadline"
  cleanup_managed_metadata
}

hard_stop() {
  trap '' INT TERM HUP
  collect_managed_process_groups
  log_warn "third stop request received; sending SIGKILL to authenticated runtime groups"
  signal_all_owned_groups KILL
}

force_stop() {
  local signal_name="$1"
  [[ -n "${shutdown_signal}" ]] || shutdown_signal="${signal_name}"
  ensure_shutdown_deadline
  collect_managed_process_groups
  if [[ "${force_requested}" != true ]]; then
    force_requested=true
    trap 'hard_stop' INT TERM HUP
    log_warn "second stop request received; sending SIGTERM to authenticated runtime groups"
    signal_all_owned_groups TERM
  else
    hard_stop
  fi
}

ordered_stop() {
  local signal_name="$1"
  local lifecycle_status lifecycle_timeout
  if [[ -n "${shutdown_signal}" ]]; then
    force_stop "${signal_name}"
    return 0
  fi
  shutdown_signal="${signal_name}"
  ensure_shutdown_deadline

  # Launch, integrated RViz, and Mapping Teleop each have separate process
  # groups.  Keep them alive while the sole lifecycle owner performs the
  # ordered transition, then stop only groups authenticated to this session.
  trap 'force_stop INT' INT
  trap 'force_stop TERM' TERM
  trap 'force_stop HUP' HUP
  lifecycle_timeout="$(
    remaining_shutdown_timeout "${lifecycle_timeout_seconds}" || true
  )"
  if [[ -n "${lifecycle_timeout}" ]]; then
    log_info "requesting ordered ${operation} lifecycle shutdown"
    set +e
    env \
      --default-signal=INT \
      --default-signal=TERM \
      --default-signal=HUP \
      setsid -- timeout --signal=TERM --kill-after=1.0 \
        "${lifecycle_timeout}" \
        python3 -m robot_bringup.ordered_shutdown \
        "${operation}" --timeout "${lifecycle_timeout}" &
    lifecycle_pid=$!
    lifecycle_start_ticks="$(
      runtime_process_start_ticks "${lifecycle_pid}" || true
    )"
    wait "${lifecycle_pid}"
    lifecycle_status=$?
    lifecycle_pid=""
    lifecycle_start_ticks=""
    set -e
    if [[ "${lifecycle_status}" -ne 0 ]]; then
      log_warn "ordered lifecycle shutdown completed with warnings"
    fi
  else
    log_warn "ordered lifecycle shutdown skipped because the global deadline expired"
  fi
  shutdown_owned_process_groups
}

cleanup_supervisor() {
  trap - INT TERM HUP
  set +e
  ensure_shutdown_deadline
  collect_managed_process_groups
  if owned_groups_are_running; then
    signal_all_owned_groups TERM
    wait_for_owned_groups_exit "${shutdown_term_checks}" || true
    if owned_groups_are_running; then
      signal_all_owned_groups KILL
      wait_for_owned_groups_exit "${shutdown_kill_checks}" || true
    fi
  fi
  cleanup_managed_metadata
  release_instance_lock ros || true
  return 0
}

trap 'ordered_stop INT' INT
trap 'ordered_stop TERM' TERM
trap 'ordered_stop HUP' HUP
trap cleanup_supervisor EXIT

setsid -- ros2 launch robot_bringup \
  "${operation}_bringup.launch.py" "${launch_args[@]}" &
launch_pid=$!
launch_start_ticks="$(runtime_process_start_ticks "${launch_pid}" || true)"

launch_status=0
set +e
wait "${launch_pid}"
launch_status=$?
set -e

collect_managed_process_groups
if owned_groups_are_running; then
  if [[ -z "${shutdown_signal}" ]]; then
    shutdown_signal="unexpected_launch_exit"
    ensure_shutdown_deadline
    log_warn "ROS launch exited while authenticated runtime groups are still active"
  fi
  shutdown_owned_process_groups
fi

# Reap the launch leader if the first wait was interrupted by a signal trap.
set +e
wait "${launch_pid}" 2>/dev/null
reaped_status=$?
set -e
if [[ "${reaped_status}" -ne 127 ]]; then
  launch_status="${reaped_status}"
fi

cleanup_managed_metadata
release_instance_lock ros
trap - EXIT INT TERM HUP
exit "${launch_status}"
