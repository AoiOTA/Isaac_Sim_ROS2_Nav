#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

original_args=("$@")

usage() {
  cat <<'EOF'
usage: run_motion_baseline.sh --environment ID --odometry-mode MODE
                              [--config FILE] [--output FILE]

Run the non-interactive, three-tier forward/backward/rotation chassis
diagnostic plus the 5 s left/right arc A/B commands. Isaac Sim must already
be running and exposing Reset, /clock, /odom, and /joint_states. Navigation,
Collision Monitor, and Teleop must be stopped because this diagnostic
exclusively owns /cmd_vel.

MODE must be ideal or realistic. ID should identify the actual stage, for
example SimplePlane or Warehouse, so A/B reports cannot be confused.
EOF
}

environment_id=""
odometry_mode=""
config_file=""
output_file=""
while (($#)); do
  case "$1" in
    --environment|--odometry-mode|--config|--output)
      (($# >= 2)) || die "$1 requires a value"
      case "$1" in
        --environment) environment_id="$2" ;;
        --odometry-mode) odometry_mode="$2" ;;
        --config) config_file="$2" ;;
        --output) output_file="$2" ;;
      esac
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      die "unknown motion-baseline argument: $1"
      ;;
  esac
done

[[ -n "${environment_id}" ]] || die "--environment is required"
[[ "${environment_id}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] \
  || die "--environment must be a path-safe identifier"
case "${odometry_mode}" in
  ideal|realistic) ;;
  *) die "--odometry-mode must be ideal or realistic" ;;
esac

source_ros --require-workspace
require_command ros2
require_command timeout
require_command realpath

active_nodes="$(
  timeout 3 ros2 node list --no-daemon --spin-time 0.5 2>/dev/null || true
)"
if printf '%s\n' "${active_nodes}" | grep -Eq \
    '^/(controller_server|velocity_smoother|collision_monitor|keyboard_teleop)$'; then
  die "Navigation, Collision Monitor, or Teleop is active; /cmd_vel is not exclusive"
fi

ensure_dedicated_process_group "${original_args[@]}"
acquire_instance_lock motion_baseline "motion baseline diagnostic"

if [[ -z "${config_file}" ]]; then
  package_prefix="$(ros2 pkg prefix robot_experiments)"
  config_file="${package_prefix}/share/robot_experiments/config/motion_baseline.yaml"
elif [[ "${config_file}" != /* ]]; then
  config_file="$(realpath -m "${config_file}")"
fi
require_file "${config_file}"

if [[ -z "${output_file}" ]]; then
  timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
  report_name="${environment_id}_${odometry_mode}_${timestamp}.json"
  output_file="${PROJECT_ROOT}/data/reports/motion/${report_name}"
elif [[ "${output_file}" != /* ]]; then
  output_file="${PROJECT_ROOT}/${output_file}"
fi
mkdir -p "$(dirname "${output_file}")"

log_info "running motion baseline: environment=${environment_id}; odometry=${odometry_mode}"
log_info "report=${output_file}"
exec ros2 run robot_experiments motion_baseline_runner --ros-args \
  -p "config_file:=${config_file}" \
  -p "environment_id:=${environment_id}" \
  -p "odometry_mode:=${odometry_mode}" \
  -p "output_file:=${output_file}"
