#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

require_executable "${ISAAC_PYTHON}"
require_file "${PROJECT_ROOT}/isaac_sim/apps/navigation_sim.py"
PROJECT_CONFIG="${ISAAC_NAV_PROJECT_CONFIG:-${PROJECT_ROOT}/isaac_sim/configs/project.yaml}"
require_file "${PROJECT_CONFIG}"
source_ros

export ISAAC_ASSET_ROOT
export ISAAC_NAV__ASSET_ROOT="${ISAAC_ASSET_ROOT}"
export ISAAC_NAV__ROS2__DOMAIN_ID="${ROS_DOMAIN_ID}"
export ISAAC_NAV__ROS2__RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}"
exec "${ISAAC_PYTHON}" "${PROJECT_ROOT}/isaac_sim/apps/navigation_sim.py" \
  --config "${PROJECT_CONFIG}" "$@"
