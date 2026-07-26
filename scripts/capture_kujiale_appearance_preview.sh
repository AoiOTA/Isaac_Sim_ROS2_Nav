#!/usr/bin/env bash
# Export high-resolution, non-front-camera appearance images without ROS/Nav2.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

require_executable "${ISAAC_PYTHON}"
require_file "${PROJECT_ROOT}/isaac_sim/apps/appearance_preview.py"
require_directory "${KUJIALE_ENVIRONMENT_ROOT:-/home/lyb/kujiale_usd_rooms_20260717}"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  exec "${ISAAC_PYTHON}" "${PROJECT_ROOT}/isaac_sim/apps/appearance_preview.py" "$@"
fi

ensure_dedicated_process_group "$@"
acquire_instance_lock isaac "Isaac Sim appearance preview"
preview_lock_fd="${ISAAC_NAV_LOCK_FDS[-1]}"

export ISAAC_NAV__ASSET_ROOT="${ISAAC_ASSET_ROOT}"
export ISAAC_NAV_RUNTIME_DIR="${ISAAC_NAV_RUNTIME_DIR}/appearance_preview"

exec "${ISAAC_PYTHON}" "${PROJECT_ROOT}/isaac_sim/apps/appearance_preview.py" \
  --environment-root "${KUJIALE_ENVIRONMENT_ROOT:-/home/lyb/kujiale_usd_rooms_20260717}" \
  --/crashreporter/skipOldDumpUpload=1 \
  --/app/skipOldDumpUpload=true \
  "$@" {preview_lock_fd}>&-
