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
phase_f_domain="${BIO_NAV_PHASE_F_DOMAIN_ID:-150}"
export BIO_NAV_PHASE_F_DOMAIN_ID="${phase_f_domain}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-${phase_f_domain}}"
export ISAAC_NAV_EXPECTED_DOMAIN_ID="${ISAAC_NAV_EXPECTED_DOMAIN_ID:-${phase_f_domain}}"

exec ros2 run robot_experiments v6_phase_f_active_ttl_probe \
  "${command_name}" --config "${BIO_NAV_V6_CAUSAL_CONFIG:-${default_config}}" "$@"
