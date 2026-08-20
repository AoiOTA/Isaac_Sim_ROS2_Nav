#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
source_ros --require-integration-underlay

cd "${PROJECT_ROOT}/ros2_ws"
exec colcon build --symlink-install "$@"
