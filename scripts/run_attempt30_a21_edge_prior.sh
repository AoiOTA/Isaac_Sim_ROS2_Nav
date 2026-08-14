#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

source_ros --require-workspace
integration_root="${BIO_NAV_ATTEMPT30_V310_INTEGRATION_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/integration/attempt30-a21-v310-srdr-rviz}"
integration_setup="${BIO_NAV_ATTEMPT30_V310_INTEGRATION_SETUP:-${integration_root}/install/setup.bash}"
if [[ ! -f "${integration_setup}" \
    && -f "${integration_root}/ros2_ws/install/setup.bash" ]]; then
  integration_setup="${integration_root}/ros2_ws/install/setup.bash"
fi
require_file "${integration_setup}"
require_file "${integration_root}/ros2_ws/src/bio_nav_ros_bridge/bio_nav_common/v310.py"
set +u
source "${integration_setup}"
set -u
export PYTHONPATH="${integration_root}/ros2_ws/src/bio_nav_ros_bridge${PYTHONPATH:+:${PYTHONPATH}}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-151}"
ensure_dedicated_process_group "$@"
acquire_instance_lock edge_prior "A21 edge-prior bridge"
exec ros2 launch bio_nav_ros_bridge attempt30_a21_v310.launch.py "$@"
