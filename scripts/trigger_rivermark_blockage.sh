#!/usr/bin/env bash
set -Eeuo pipefail

module3_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
integration_root="${BIO_NAV_INTEGRATION_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/integration/attempt31-outdoor-nav}"
state="${1:-blocked}"
shift || true

if [[ "${state}" != "blocked" && "${state}" != "clear" ]]; then
  echo "usage: $0 [blocked|clear] [--edge-id ID]" >&2
  exit 2
fi

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH \
  PYTHONPATH ROS_PACKAGE_PATH
set +u
source /opt/ros/jazzy/setup.bash
source "${integration_root}/ros2_ws/install/local_setup.bash"
source "${module3_root}/ros2_ws/install/local_setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-231}"
exec ros2 run robot_experiments runtime_blockage_demo --state "${state}" "$@"
