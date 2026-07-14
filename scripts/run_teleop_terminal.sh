#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

(($# > 0)) || die "usage: run_teleop_terminal.sh <terminal command...>"
terminal_command=("$@")
terminal_pid=""
terminal_start_ticks=""
teleop_int_checks="${ISAAC_NAV_TELEOP_INT_CHECKS:-20}"
teleop_term_checks="${ISAAC_NAV_TELEOP_TERM_CHECKS:-20}"
teleop_kill_checks="${ISAAC_NAV_TELEOP_KILL_CHECKS:-20}"
terminal_term_checks="${ISAAC_NAV_TERMINAL_TERM_CHECKS:-20}"
terminal_kill_checks="${ISAAC_NAV_TERMINAL_KILL_CHECKS:-20}"
for check_setting in \
  teleop_int_checks teleop_term_checks teleop_kill_checks \
  terminal_term_checks terminal_kill_checks; do
  check_value="${!check_setting}"
  [[ "${check_value}" =~ ^[1-9][0-9]*$ ]] \
    || die "${check_setting} must be a positive integer"
done

registered_teleop_group() {
  [[ -n "${ISAAC_NAV_SESSION_ID:-}" ]] || return 1
  runtime_registered_process_group teleop "${ISAAC_NAV_SESSION_ID}"
}

registered_teleop_group_is_current() {
  local expected_group="$1"
  local current_group
  current_group="$(registered_teleop_group || true)"
  [[ -n "${current_group}" && "${current_group}" == "${expected_group}" ]]
}

stop_registered_teleop() {
  local process_group
  process_group="$(registered_teleop_group || true)"
  [[ -n "${process_group}" ]] || return 0

  log_info "stopping managed Mapping teleop process group ${process_group} with SIGINT"
  registered_teleop_group_is_current "${process_group}" || return 1
  kill -INT -- "-${process_group}" 2>/dev/null || true
  if wait_for_runtime_group_exit "${process_group}" "${teleop_int_checks}"; then
    remove_runtime_session_metadata teleop "${ISAAC_NAV_SESSION_ID}"
    return 0
  fi

  log_warn "Mapping teleop group ${process_group} did not exit after SIGINT; sending SIGTERM"
  registered_teleop_group_is_current "${process_group}" || return 1
  kill -TERM -- "-${process_group}" 2>/dev/null || true
  if wait_for_runtime_group_exit "${process_group}" "${teleop_term_checks}"; then
    remove_runtime_session_metadata teleop "${ISAAC_NAV_SESSION_ID}"
    return 0
  fi

  log_warn "Mapping teleop group ${process_group} did not exit after SIGTERM; sending SIGKILL"
  registered_teleop_group_is_current "${process_group}" || return 1
  kill -KILL -- "-${process_group}" 2>/dev/null || true
  wait_for_runtime_group_exit "${process_group}" "${teleop_kill_checks}" \
    || return 1
  remove_runtime_session_metadata teleop "${ISAAC_NAV_SESSION_ID}"
}

wait_for_runtime_group_exit() {
  local process_group="$1"
  local attempts="$2"
  local attempt
  for ((attempt = 0; attempt < attempts; attempt++)); do
    runtime_process_group_is_running "${process_group}" || return 0
    sleep 0.1
  done
  ! runtime_process_group_is_running "${process_group}"
}

wait_for_terminal_exit() {
  local attempts="$1"
  local attempt
  for ((attempt = 0; attempt < attempts; attempt++)); do
    terminal_process_is_running || return 0
    sleep 0.1
  done
  ! terminal_process_is_running
}

terminal_process_is_running() {
  local actual_start
  [[ -n "${terminal_pid}" && -n "${terminal_start_ticks}" ]] || return 1
  runtime_process_is_running "${terminal_pid}" || return 1
  actual_start="$(runtime_process_start_ticks "${terminal_pid}" || true)"
  [[ "${actual_start}" == "${terminal_start_ticks}" ]]
}

stop_terminal_process() {
  terminal_process_is_running || return 0
  log_info "stopping managed Teleop terminal pid ${terminal_pid} with SIGTERM"
  kill -TERM "${terminal_pid}" 2>/dev/null || true
  wait_for_terminal_exit "${terminal_term_checks}" && return 0
  log_warn "Teleop terminal pid ${terminal_pid} ignored SIGTERM; sending SIGKILL"
  terminal_process_is_running || return 0
  kill -KILL "${terminal_pid}" 2>/dev/null || true
  wait_for_terminal_exit "${terminal_kill_checks}"
}

cleanup() {
  local failed=false
  trap - INT TERM HUP
  stop_registered_teleop || failed=true
  stop_terminal_process || failed=true
  [[ "${failed}" == false ]]
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

"${terminal_command[@]}" &
terminal_pid="$!"
terminal_start_ticks="$(runtime_process_start_ticks "${terminal_pid}" || true)"
terminal_status=0
set +e
wait "${terminal_pid}"
terminal_status=$?
set -e

trap - INT TERM HUP
if ! cleanup; then
  terminal_status=1
fi
trap - EXIT
exit "${terminal_status}"
