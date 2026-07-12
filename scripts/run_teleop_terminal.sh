#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

(($# > 0)) || die "usage: run_teleop_terminal.sh <terminal command...>"
terminal_command=("$@")
terminal_pid=""

registered_teleop_pid() {
  local pid_file pid command_line
  pid_file="$(runtime_pid_file teleop)"
  [[ -r "${pid_file}" ]] || return 1
  pid="$(sed -n 's/^pid=//p' "${pid_file}" | head -n 1)"
  [[ "${pid}" =~ ^[0-9]+$ && -r "/proc/${pid}/cmdline" ]] || return 1
  command_line="$(tr '\0' ' ' <"/proc/${pid}/cmdline")"
  [[ "${command_line}" == *"robot_teleop"*"keyboard_teleop"* ]] || return 1
  printf '%s\n' "${pid}"
}

stop_registered_teleop() {
  local pid attempt
  pid="$(registered_teleop_pid || true)"
  [[ -n "${pid}" ]] || return 0
  log_info "stopping managed Mapping teleop pid ${pid}"
  kill -INT "${pid}" 2>/dev/null || return 0
  for ((attempt = 0; attempt < 20; attempt++)); do
    kill -0 "${pid}" 2>/dev/null || return 0
    sleep 0.1
  done
  log_warn "Mapping teleop pid ${pid} did not exit after SIGINT; sending SIGTERM"
  kill -TERM "${pid}" 2>/dev/null || true
}

cleanup() {
  stop_registered_teleop
  if [[ -n "${terminal_pid}" ]]; then
    kill -TERM "${terminal_pid}" 2>/dev/null || true
  fi
}

trap cleanup EXIT
trap 'exit 0' INT TERM HUP

"${terminal_command[@]}" &
terminal_pid="$!"
wait "${terminal_pid}"
