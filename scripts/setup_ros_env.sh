#!/usr/bin/env bash

# This file intentionally has no `set -euo pipefail`: it is sourced into the
# user's interactive shell and must not mutate that shell's option state.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  printf '[isaac-nav] error: setup_ros_env.sh must be sourced, not executed\n' >&2
  printf 'usage: source ./scripts/setup_ros_env.sh [--restart-daemon]\n' >&2
  exit 1
fi

_isaac_nav_setup_ros_env() {
  local restart_daemon=false
  local script_dir project_root ros_setup workspace_setup
  local nounset_was_enabled=false
  local ros2_cli

  case "${1:-}" in
    '') ;;
    --restart-daemon) restart_daemon=true ;;
    *)
      printf '[isaac-nav] error: unknown setup_ros_env option: %s\n' "$1" >&2
      printf 'usage: source ./scripts/setup_ros_env.sh [--restart-daemon]\n' >&2
      return 1
      ;;
  esac
  if (($# > 1)); then
    printf '[isaac-nav] error: setup_ros_env accepts at most one option\n' >&2
    return 1
  fi

  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)" || return 1
  project_root="$(cd "${script_dir}/.." && pwd)" || return 1
  if [[ ! -f "${project_root}/scripts/lib/common.sh" \
        || ! -d "${project_root}/ros2_ws/src" ]]; then
    printf '[isaac-nav] error: invalid project root resolved from script: %s\n' \
      "${project_root}" >&2
    return 1
  fi
  if [[ -n "${PROJECT_ROOT:-}" && "${PROJECT_ROOT}" != "${project_root}" ]]; then
    printf '[isaac-nav] error: PROJECT_ROOT points to another checkout: %s\n' \
      "${PROJECT_ROOT}" >&2
    return 1
  fi

  ros_setup="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
  workspace_setup="${ISAAC_NAV_WORKSPACE_SETUP:-${project_root}/ros2_ws/install/setup.bash}"
  [[ -f "${ros_setup}" ]] || {
    printf '[isaac-nav] error: ROS setup not found: %s\n' "${ros_setup}" >&2
    return 1
  }
  [[ -f "${workspace_setup}" ]] || {
    printf '[isaac-nav] error: ROS workspace is not built: %s\n' \
      "${workspace_setup}" >&2
    printf '[isaac-nav] error: run ./scripts/build_ros2.sh first\n' >&2
    return 1
  }
  if [[ -n "${ROS_DOMAIN_ID:-}" && "${ROS_DOMAIN_ID}" != 42 ]]; then
    printf '[isaac-nav] error: ROS_DOMAIN_ID must be 42; got %s\n' \
      "${ROS_DOMAIN_ID}" >&2
    return 1
  fi
  if [[ -n "${RMW_IMPLEMENTATION:-}" \
        && "${RMW_IMPLEMENTATION}" != rmw_fastrtps_cpp ]]; then
    printf '[isaac-nav] error: RMW_IMPLEMENTATION must be rmw_fastrtps_cpp; got %s\n' \
      "${RMW_IMPLEMENTATION}" >&2
    return 1
  fi

  [[ $- == *u* ]] && nounset_was_enabled=true
  set +u
  # shellcheck disable=SC1090
  source "${ros_setup}" || {
    [[ "${nounset_was_enabled}" == true ]] && set -u
    printf '[isaac-nav] error: failed to source ROS setup: %s\n' \
      "${ros_setup}" >&2
    return 1
  }
  # shellcheck disable=SC1090
  source "${workspace_setup}" || {
    [[ "${nounset_was_enabled}" == true ]] && set -u
    printf '[isaac-nav] error: failed to source workspace: %s\n' \
      "${workspace_setup}" >&2
    return 1
  }
  [[ "${nounset_was_enabled}" == true ]] && set -u

  export PROJECT_ROOT="${project_root}"
  export ROS_SETUP="${ros_setup}"
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

  if [[ "${ROS_DISTRO:-}" != jazzy ]]; then
    printf '[isaac-nav] error: ROS_DISTRO must be jazzy; got %s\n' \
      "${ROS_DISTRO:-unset}" >&2
    return 1
  fi

  if [[ "${restart_daemon}" == true ]]; then
    ros2_cli="${ISAAC_NAV_ROS2_CLI:-ros2}"
    command -v "${ros2_cli}" >/dev/null 2>&1 || {
      printf '[isaac-nav] error: ROS 2 CLI not found: %s\n' "${ros2_cli}" >&2
      return 1
    }
    "${ros2_cli}" daemon stop || return 1
    "${ros2_cli}" daemon start || return 1
  fi

  printf '[isaac-nav] ROS terminal environment ready\n'
  printf 'PROJECT_ROOT=%s\n' "${PROJECT_ROOT}"
  printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO}"
  printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID}"
  printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION}"
  printf 'AMENT_PREFIX_PATH=%s\n' "${AMENT_PREFIX_PATH:-}"
  printf 'ROS_SETUP=%s\n' "${ROS_SETUP}"
  printf 'WORKSPACE_SETUP=%s\n' "${workspace_setup}"
}

_isaac_nav_setup_ros_env "$@"
_isaac_nav_setup_status=$?
unset -f _isaac_nav_setup_ros_env
return "${_isaac_nav_setup_status}"
