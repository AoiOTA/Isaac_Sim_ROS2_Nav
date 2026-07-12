#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

usage() {
  cat <<'EOF'
usage: run_teleop.sh [params.yaml]

Start the deadman-protected W/A/S/D controller for Mapping or Incremental
Mapping. This command requires an interactive terminal and must never be run
alongside Localization or Navigation.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
(($# <= 1)) || {
  usage >&2
  die "run_teleop.sh accepts at most one parameter file"
}
[[ -t 0 ]] || die "mapping teleop requires an interactive TTY"

source_ros --require-workspace
require_command ros2
require_command realpath
require_command timeout

active_nodes="$(
  timeout 3 ros2 node list --no-daemon --spin-time 0.5 2>/dev/null || true
)"
if printf '%s\n' "${active_nodes}" | grep -Eq '^/(map_server|controller_server|collision_monitor)$'; then
  die "Localization or Navigation nodes are active; Mapping teleop must not publish /cmd_vel"
fi
acquire_instance_lock teleop "Mapping teleop"

if (($# == 1)); then
  params_file="$1"
  if [[ "${params_file}" != /* ]]; then
    params_file="$(realpath -m "${params_file}")"
  fi
else
  teleop_prefix="$(ros2 pkg prefix robot_teleop)"
  params_file="${teleop_prefix}/share/robot_teleop/config/teleop.yaml"
fi

require_file "${params_file}"
teleop_executable="${teleop_prefix:-$(ros2 pkg prefix robot_teleop)}/lib/robot_teleop/keyboard_teleop"
require_executable "${teleop_executable}"
log_info "starting deadman-protected mapping teleop: ${params_file}"
exec "${teleop_executable}" \
  --ros-args \
  --params-file "${params_file}"
