#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

original_args=("$@")

usage() {
  cat <<'EOF'
usage: run_teleop.sh [--localization-collection] [params.yaml] [speed_parameter:=value ...]

Start the deadman-protected W/A/S/D controller for Mapping or Incremental
Mapping. The explicit --localization-collection mode additionally permits
simulation-only manual data collection alongside Localization. It must never
be run alongside Navigation; Localization collection does not start Nav2's
controller_server or collision_monitor.

Allowed runtime overrides:
  linear_speed angular_speed linear_speed_step angular_speed_step
  min_linear_speed min_angular_speed max_linear_speed max_angular_speed
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

localization_collection=false
if [[ "${1:-}" == "--localization-collection" ]]; then
  localization_collection=true
  shift
fi

if [[ "${localization_collection}" == true ]]; then
  teleop_label="simulation-only localization collection teleop"
else
  teleop_label="mapping teleop"
fi
[[ -t 0 ]] || die "${teleop_label} requires an interactive TTY"

source_ros --require-workspace
require_command ros2
require_command realpath
require_command timeout

active_nodes="$(
  timeout 3 ros2 node list --no-daemon --spin-time 0.5 2>/dev/null || true
)"
if [[ "${localization_collection}" == true ]]; then
  if printf '%s\n' "${active_nodes}" | grep -Eq '^/(controller_server|planner_server|bt_navigator|behavior_server|waypoint_follower|velocity_smoother|collision_monitor)$'; then
    die "Navigation nodes are active; localization collection teleop must not publish /cmd_vel"
  fi
elif printf '%s\n' "${active_nodes}" | grep -Eq '^/(map_server|controller_server|collision_monitor)$'; then
  die "Localization or Navigation nodes are active; Mapping teleop must not publish /cmd_vel"
fi
ensure_dedicated_process_group "${original_args[@]}"
acquire_instance_lock teleop "${teleop_label}"

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
log_info "starting deadman-protected ${teleop_label}: ${params_file}; overrides=${*:-none}"
exec "${teleop_executable}" \
  --ros-args \
  --params-file "${params_file}" \
  "${parameter_arguments[@]}"
