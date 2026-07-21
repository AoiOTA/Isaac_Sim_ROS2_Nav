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
acquire_instance_lock ros "ROS stack"
export ISAAC_NAV_SPAWN_POSES="${ISAAC_NAV_SPAWN_POSES:-${PROJECT_ROOT}/isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.spawn.yaml}"

launch_args=("$@")
if [[ "${operation}" == "localization" || "${operation}" == "navigation" ]]; then
  # This custom-scene branch promotes the complete Kujiale map bundle while
  # the original warehouse branch keeps warehouse_v2 as its own default.
  default_map_version="warehouse_new"
  posegraph_file=""
  map_file=""
  odometry_mode="ideal"
  posegraph_calibration="false"
  for argument in "${launch_args[@]}"; do
    case "${argument}" in
      posegraph_file:=*) posegraph_file="${argument#posegraph_file:=}" ;;
      map_file:=*) map_file="${argument#map_file:=}" ;;
      odometry_mode:=*) odometry_mode="${argument#odometry_mode:=}" ;;
      posegraph_calibration:=*) posegraph_calibration="${argument#posegraph_calibration:=}" ;;
    esac
  done
  if [[ -z "${posegraph_file}" ]]; then
    if [[ -n "${map_file}" ]]; then
      map_prefix="${map_file%.yaml}"
      posegraph_file="${PROJECT_ROOT}/data/maps/posegraphs/$(basename "${map_prefix}")"
    else
      posegraph_file="${PROJECT_ROOT}/data/maps/posegraphs/${default_map_version}"
    fi
    require_file "${posegraph_file}.posegraph"
    require_file "${posegraph_file}.data"
    launch_args+=("posegraph_file:=${posegraph_file}")
  fi
  if [[ -z "${map_file}" ]]; then
    posegraph_prefix="${posegraph_file%.posegraph}"
    posegraph_prefix="${posegraph_prefix%.data}"
    map_file="${PROJECT_ROOT}/data/maps/occupancy/$(basename "${posegraph_prefix}").yaml"
    require_file "${map_file}"
    launch_args+=("map_file:=${map_file}")
  fi
  posegraph_version="$(basename "${posegraph_file%.posegraph}")"
  posegraph_version="${posegraph_version%.data}"
  if [[ "${posegraph_version}" == "warehouse_new" ]] \
      && [[ "${odometry_mode}" != "ideal" \
            || "${posegraph_calibration}" == "true" ]]; then
    die "warehouse_new is calibrated for normal Ideal localization/navigation only; rebuild it with scan matching before Realistic or Pose Graph localization"
  fi
fi

launch_pid=""
shutdown_signal=""
force_requested=false
shutdown_int_checks="${ISAAC_NAV_SHUTDOWN_INT_CHECKS:-100}"
shutdown_term_checks="${ISAAC_NAV_SHUTDOWN_TERM_CHECKS:-50}"
[[ "${shutdown_int_checks}" =~ ^[1-9][0-9]*$ ]] \
  || die "ISAAC_NAV_SHUTDOWN_INT_CHECKS must be a positive integer"
[[ "${shutdown_term_checks}" =~ ^[1-9][0-9]*$ ]] \
  || die "ISAAC_NAV_SHUTDOWN_TERM_CHECKS must be a positive integer"

launch_group_is_running() {
  [[ -n "${launch_pid}" ]] || return 1
  ps -eo pgid=,stat= | awk -v group="${launch_pid}" '
    $1 == group && $2 !~ /^Z/ { found = 1 }
    END { exit !found }
  '
}

signal_launch_group() {
  local signal_name="$1"
  launch_group_is_running || return 0
  kill "-${signal_name}" -- "-${launch_pid}" 2>/dev/null || true
}

wait_for_launch_group_exit() {
  local attempts="$1"
  local index
  for ((index=0; index<attempts; index++)); do
    launch_group_is_running || return 0
    sleep 0.1
  done
  ! launch_group_is_running
}

hard_stop() {
  trap '' INT TERM HUP
  log_warn "ROS launch process group ${launch_pid} received a third stop request; sending SIGKILL"
  signal_launch_group KILL
}

force_stop() {
  local signal_name="$1"
  [[ -n "${launch_pid}" ]] || return 0
  [[ -n "${shutdown_signal}" ]] || shutdown_signal="${signal_name}"
  if [[ "${force_requested}" != true ]]; then
    force_requested=true
    trap 'hard_stop' INT TERM HUP
    log_warn "forcing ROS launch process group ${launch_pid} to stop with SIGTERM"
    signal_launch_group TERM
  else
    hard_stop
  fi
}

ordered_stop() {
  local signal_name="$1"
  [[ -n "${launch_pid}" ]] || return 0
  if [[ -n "${shutdown_signal}" ]]; then
    force_stop "${signal_name}"
    return 0
  fi
  shutdown_signal="${signal_name}"

  # A terminal sends Ctrl+C to every process in its foreground process group.
  # The launch child has its own session so lifecycle services remain alive
  # while this supervisor performs their ordered shutdown.
  trap 'force_stop INT' INT
  trap 'force_stop TERM' TERM
  trap 'force_stop HUP' HUP
  log_info "requesting ordered ${operation} lifecycle shutdown"
  if ! (
      trap - INT TERM HUP
      exec python3 -m robot_bringup.ordered_shutdown \
        "${operation}" --timeout 20.0
    ); then
    log_warn "ordered lifecycle shutdown completed with warnings"
  fi
  if launch_group_is_running; then
    log_info "stopping ROS launch process group ${launch_pid} with SIGINT"
    signal_launch_group INT
  fi
}

cleanup_launch_process() {
  [[ -n "${launch_pid}" ]] || return 0
  if launch_group_is_running; then
    signal_launch_group TERM
  fi
}

trap 'ordered_stop INT' INT
trap 'ordered_stop TERM' TERM
trap 'ordered_stop HUP' HUP
trap cleanup_launch_process EXIT

setsid -- ros2 launch robot_bringup \
  "${operation}_bringup.launch.py" "${launch_args[@]}" &
launch_pid=$!

launch_status=0
set +e
wait "${launch_pid}"
launch_status=$?
set -e

if launch_group_is_running; then
  if [[ -z "${shutdown_signal}" ]]; then
    shutdown_signal="unexpected_launch_leader_exit"
    log_warn "ROS launch leader exited while its process group is still active"
    signal_launch_group INT
  fi
  if ! wait_for_launch_group_exit "${shutdown_int_checks}"; then
    log_warn "ROS launch process group ${launch_pid} ignored SIGINT; sending SIGTERM"
    signal_launch_group TERM
    if ! wait_for_launch_group_exit "${shutdown_term_checks}"; then
      log_warn "ROS launch process group ${launch_pid} ignored SIGTERM; sending SIGKILL"
      signal_launch_group KILL
      wait_for_launch_group_exit 20 \
        || log_warn "ROS launch process group ${launch_pid} is still visible after SIGKILL"
    fi
  fi
fi

# Reap the launch leader if the first wait was interrupted by a signal trap.
set +e
wait "${launch_pid}" 2>/dev/null
reaped_status=$?
set -e
if [[ "${reaped_status}" -ne 127 ]]; then
  launch_status="${reaped_status}"
fi

trap - EXIT INT TERM HUP
exit "${launch_status}"
