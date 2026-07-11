#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ISAAC_PYTHON="${ISAAC_PYTHON:-/home/lyb/miniconda3/envs/isaacsim/bin/python}"
ISAAC_ASSET_ROOT="${ISAAC_ASSET_ROOT:-/home/lyb/isaacsim_assets/Assets/Isaac/6.0}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

die() {
  echo "error: $*" >&2
  exit 1
}

require_file() {
  [[ -f "$1" ]] || die "required file not found: $1"
}

require_executable() {
  [[ -x "$1" ]] || die "required executable not found: $1"
}

source_ros() {
  require_file "${ROS_SETUP}"
  # ROS-generated setup scripts reference optional variables before defining
  # them, so nounset must be suspended only while they are sourced.
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"

  if [[ -f "${PROJECT_ROOT}/ros2_ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${PROJECT_ROOT}/ros2_ws/install/setup.bash"
  fi
  set -u
}
