#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

original_args=("$@")

usage() {
  cat <<'EOF'
usage: run_teleop.sh [params.yaml] [speed_parameter:=value ...]

Start the deadman-protected W/A/S/D controller for Mapping or Incremental
Mapping. This command requires an interactive terminal and must never be run
alongside Localization or Navigation.

Allowed runtime overrides:
  linear_speed angular_speed linear_speed_step angular_speed_step
  min_linear_speed min_angular_speed max_linear_speed max_angular_speed
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
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
ensure_dedicated_process_group "${original_args[@]}"
acquire_instance_lock teleop "Mapping teleop"

params_file=""
if (($# > 0)) && [[ "$1" != *':='* ]]; then
  params_file="$1"
  shift
  if [[ "${params_file}" != /* ]]; then
    params_file="$(realpath -m "${params_file}")"
  fi
fi
if [[ -z "${params_file}" ]]; then
  teleop_prefix="$(ros2 pkg prefix robot_teleop)"
  params_file="${teleop_prefix}/share/robot_teleop/config/teleop.yaml"
fi

parameter_arguments=()
for override in "$@"; do
  case "${override}" in
    linear_speed:=*|angular_speed:=*|\
    linear_speed_step:=*|angular_speed_step:=*|\
    min_linear_speed:=*|min_angular_speed:=*|\
    max_linear_speed:=*|max_angular_speed:=*)
      [[ -n "${override#*:=}" ]] \
        || die "teleop speed override has an empty value: ${override}"
      parameter_arguments+=(--param "${override}")
      ;;
    *)
      usage >&2
      die "unsupported teleop speed override: ${override}"
      ;;
  esac
done

require_file "${params_file}"
teleop_executable="${teleop_prefix:-$(ros2 pkg prefix robot_teleop)}/lib/robot_teleop/keyboard_teleop"
require_executable "${teleop_executable}"
log_info "starting deadman-protected mapping teleop: ${params_file}; overrides=${*:-none}"
exec "${teleop_executable}" \
  --ros-args \
  --params-file "${params_file}" \
  "${parameter_arguments[@]}"
