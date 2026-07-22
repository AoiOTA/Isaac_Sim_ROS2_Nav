#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

original_args=("$@")

require_executable "${ISAAC_PYTHON}"
require_file "${PROJECT_ROOT}/isaac_sim/apps/navigation_sim.py"
PROJECT_CONFIG="${ISAAC_NAV_PROJECT_CONFIG:-${PROJECT_ROOT}/isaac_sim/configs/project.yaml}"
require_file "${PROJECT_CONFIG}"
source_ros
ensure_dedicated_process_group "${original_args[@]}"
acquire_instance_lock isaac "Isaac Sim"
# Keep the lock in this supervisor only.  Isaac Kit may leave auxiliary
# Omniverse processes running after its main process exits; they must not keep
# the next Isaac launch locked through an inherited descriptor.
isaac_lock_fd="${ISAAC_NAV_LOCK_FDS[-1]}"

export ISAAC_ASSET_ROOT
export ISAAC_NAV__ASSET_ROOT="${ISAAC_ASSET_ROOT}"
export ISAAC_NAV__ROS2__DOMAIN_ID="${ROS_DOMAIN_ID}"
export ISAAC_NAV__ROS2__RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}"

isaac_pid=""

stop_isaac() {
  local signal_name="$1"
  [[ -n "${isaac_pid}" ]] || return 0
  kill "-${signal_name}" "${isaac_pid}" 2>/dev/null || true
}

trap 'stop_isaac INT' INT
trap 'stop_isaac TERM' TERM
trap 'stop_isaac HUP' HUP

"${ISAAC_PYTHON}" "${PROJECT_ROOT}/isaac_sim/apps/navigation_sim.py" \
  --config "${PROJECT_CONFIG}" "$@" {isaac_lock_fd}>&- &
isaac_pid=$!

set +e
wait "${isaac_pid}"
isaac_status=$?
set -e

trap - INT TERM HUP
exit "${isaac_status}"
