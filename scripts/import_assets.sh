#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

require_executable "${ISAAC_PYTHON}"
require_file "${PROJECT_ROOT}/isaac_sim/tools/import_assets.py"

exec "${ISAAC_PYTHON}" "${PROJECT_ROOT}/isaac_sim/tools/import_assets.py" \
  --asset-root "${ISAAC_ASSET_ROOT}" "$@"
