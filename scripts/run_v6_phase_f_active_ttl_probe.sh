#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  echo "usage: $0 plan|run --output-root PATH [--selected-arm {M2,M3}] [arguments...]" >&2
}

command_name="${1:-}"
if [[ "${command_name}" != "plan" && "${command_name}" != "run" ]]; then
  usage
  exit 2
fi
shift

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
default_config="${script_dir}/../ros2_ws/src/robot_experiments/config/v6_kujiale_low_obstacle_causal.yaml"

exec ros2 run robot_experiments v6_phase_f_active_ttl_probe \
  "${command_name}" --config "${BIO_NAV_V6_CAUSAL_CONFIG:-${default_config}}" "$@"
