#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  echo "usage: $0 manifest|plan|evaluate|run [arguments...]" >&2
}

command_name="${1:-}"
if [[ "${command_name}" != "manifest" && "${command_name}" != "plan" \
  && "${command_name}" != "evaluate" && "${command_name}" != "run" ]]; then
  usage
  exit 2
fi
shift

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
default_config="${script_dir}/../ros2_ws/src/robot_experiments/config/v6_localization_causal.yaml"

exec ros2 run robot_experiments v6_localization_causal \
  "${command_name}" --config "${BIO_NAV_V6_LOCALIZATION_CAUSAL_CONFIG:-$default_config}" "$@"
