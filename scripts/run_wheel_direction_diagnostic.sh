#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

original_args=("$@")
require_executable "${ISAAC_PYTHON}"
require_file "${PROJECT_ROOT}/isaac_sim/apps/wheel_direction_diagnostic.py"

PROJECT_CONFIG="${ISAAC_NAV_PROJECT_CONFIG:-${PROJECT_ROOT}/isaac_sim/configs/project.yaml}"
DIAGNOSTIC_CONFIG="${ISAAC_NAV_WHEEL_DIAGNOSTIC_CONFIG:-${PROJECT_ROOT}/isaac_sim/configs/diagnostics/wheel_direction.yaml}"
require_file "${PROJECT_CONFIG}"
require_file "${DIAGNOSTIC_CONFIG}"

ensure_dedicated_process_group "${original_args[@]}"
acquire_instance_lock isaac "Isaac Sim wheel direction diagnostic"

export ISAAC_ASSET_ROOT
export ISAAC_NAV__ASSET_ROOT="${ISAAC_ASSET_ROOT}"
exec "${ISAAC_PYTHON}" \
  "${PROJECT_ROOT}/isaac_sim/apps/wheel_direction_diagnostic.py" \
  --config "${PROJECT_CONFIG}" \
  --diagnostic-config "${DIAGNOSTIC_CONFIG}" \
  "$@"
