#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  echo "usage: $0 plan|run M1|M3|M2-fallback --output-root PATH [arguments...]" >&2
}

command_name="${1:-}"
arm="${2:-}"
if [[ "${command_name}" != "plan" && "${command_name}" != "run" ]]; then
  usage
  exit 2
fi
if [[ "${arm}" != "M1" && "${arm}" != "M3" && "${arm}" != "M2-fallback" ]]; then
  usage
  exit 2
fi
shift 2

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
default_config="${script_dir}/../ros2_ws/src/robot_experiments/config/v6_single_dynamic_low_obstacle.yaml"

if [[ "${command_name}" == "run" ]]; then
  # shellcheck source=lib/v6_dynamic_startup.sh
  source "${script_dir}/lib/v6_dynamic_startup.sh"
  configure_v6_dynamic_integration_overlay
  # shellcheck source=lib/common.sh
  source "${script_dir}/lib/common.sh"
  source_ros --require-integration-underlay
  validate_v6_dynamic_integration_overlay
  prepare_v6_dynamic_assets
fi

exec ros2 run robot_experiments v6_single_dynamic_low_obstacle \
  "${command_name}" \
  --config "${BIO_NAV_V6_DYNAMIC_LOW_CONFIG:-${default_config}}" \
  --arm "${arm}" \
  "$@"
