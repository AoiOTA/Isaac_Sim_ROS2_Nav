#!/usr/bin/env bash

set -Eeuo pipefail

export PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ISAAC_PYTHON="${ISAAC_PYTHON:-/home/lyb/miniconda3/envs/isaacsim/bin/python}"
export ISAAC_ASSET_ROOT="${ISAAC_ASSET_ROOT:-/home/lyb/isaacsim_assets/Assets/Isaac/6.0}"
export ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
export ISAAC_NAV_RUNTIME_DIR="${ISAAC_NAV_RUNTIME_DIR:-/tmp/isaac_sim_ros2_nav_${UID}}"
export ISAAC_NAV_FASTDDS_PROFILE="${ISAAC_NAV_FASTDDS_PROFILE:-${PROJECT_ROOT}/isaac_sim/configs/ros2_bridge/fastdds_udp_only.xml}"
export BIO_NAV_INTEGRATION_ROOT="${BIO_NAV_INTEGRATION_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_intergration}"
export BIO_NAV_INTEGRATION_INSTALL="${BIO_NAV_INTEGRATION_INSTALL:-${BIO_NAV_INTEGRATION_ROOT}/ros2_ws/install}"
export BIO_NAV_INTEGRATION_SETUP="${BIO_NAV_INTEGRATION_SETUP:-${BIO_NAV_INTEGRATION_INSTALL}/setup.bash}"

readonly ISAAC_NAV_EXPECTED_ROS_DISTRO="jazzy"
# Domain 42 remains the normal default.  Engineering runs may select another
# domain to avoid an already-running ROS graph without stopping that graph.
readonly ISAAC_NAV_EXPECTED_DOMAIN_ID="${ISAAC_NAV_EXPECTED_DOMAIN_ID:-42}"
readonly ISAAC_NAV_EXPECTED_RMW="rmw_fastrtps_cpp"

log_info() {
  printf '[isaac-nav] %s\n' "$*"
}

log_warn() {
  printf '[isaac-nav] warning: %s\n' "$*" >&2
}

die() {
  printf '[isaac-nav] error: %s\n' "$*" >&2
  exit 1
}

require_file() {
  [[ -f "$1" ]] || die "required file not found: $1"
}

require_directory() {
  [[ -d "$1" ]] || die "required directory not found: $1"
}

require_executable() {
  [[ -x "$1" ]] || die "required executable not found: $1"
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "required command not found: $1"
}

validate_runtime_environment() {
  if [[ -n "${ROS_DOMAIN_ID:-}" && "${ROS_DOMAIN_ID}" != "${ISAAC_NAV_EXPECTED_DOMAIN_ID}" ]]; then
    die "ROS_DOMAIN_ID must be ${ISAAC_NAV_EXPECTED_DOMAIN_ID}; got ${ROS_DOMAIN_ID}"
  fi
  if [[ -n "${RMW_IMPLEMENTATION:-}" && "${RMW_IMPLEMENTATION}" != "${ISAAC_NAV_EXPECTED_RMW}" ]]; then
    die "RMW_IMPLEMENTATION must be ${ISAAC_NAV_EXPECTED_RMW}; got ${RMW_IMPLEMENTATION}"
  fi
  export ROS_DOMAIN_ID="${ISAAC_NAV_EXPECTED_DOMAIN_ID}"
  export RMW_IMPLEMENTATION="${ISAAC_NAV_EXPECTED_RMW}"
  require_file "${ISAAC_NAV_FASTDDS_PROFILE}"
  # Both names are exported because Isaac's bundled Fast DDS and Jazzy's RMW
  # may consult different compatibility aliases.
  export FASTRTPS_DEFAULT_PROFILES_FILE="${ISAAC_NAV_FASTDDS_PROFILE}"
  export FASTDDS_DEFAULT_PROFILES_FILE="${ISAAC_NAV_FASTDDS_PROFILE}"
}

reset_ros_overlay_environment() {
  unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
  unset LD_LIBRARY_PATH PYTHONPATH CPATH CPLUS_INCLUDE_PATH
}

validate_v6_integration_underlay() {
  local setup_path setup_real root_real install_real bridge_prefix interfaces_prefix
  setup_path="${BIO_NAV_INTEGRATION_SETUP}"
  require_directory "${BIO_NAV_INTEGRATION_ROOT}"
  require_directory "${BIO_NAV_INTEGRATION_INSTALL}"
  require_file "${setup_path}"
  setup_real="$(readlink -f "${setup_path}")"
  root_real="$(readlink -f "${BIO_NAV_INTEGRATION_ROOT}")"
  install_real="$(readlink -f "${BIO_NAV_INTEGRATION_INSTALL}")"
  [[ "${install_real}" == "${root_real}"/* ]] || die \
    "V6 Integration install must resolve inside ${root_real}; got ${install_real}"
  [[ "${setup_real}" == "${install_real}"/* \
      && ("${setup_real}" == */setup.bash \
      || "${setup_real}" == */local_setup.bash) ]] || die \
    "V6 Integration underlay setup must resolve inside ${install_real}; got ${setup_real}"

  bridge_prefix="$(ros2 pkg prefix bio_nav_ros_bridge 2>/dev/null || true)"
  interfaces_prefix="$(ros2 pkg prefix bio_nav_interfaces 2>/dev/null || true)"
  [[ -n "${bridge_prefix}" && "$(readlink -f "${bridge_prefix}")" == "${install_real}"/* ]] || die \
    "bio_nav_ros_bridge did not resolve inside the selected Integration install ${install_real}; rebuild that snapshot/install before retrying"
  [[ -n "${interfaces_prefix}" && "$(readlink -f "${interfaces_prefix}")" == "${install_real}"/* ]] || die \
    "bio_nav_interfaces did not resolve inside the selected Integration install ${install_real}; rebuild that snapshot/install before retrying"
  require_file "${bridge_prefix}/share/bio_nav_ros_bridge/config/engineering_defaults.yaml"
  local obstacle_header prior_header
  obstacle_header="${interfaces_prefix}/include/bio_nav_interfaces/bio_nav_interfaces/msg/detail/cognitive_obstacle_array__struct.hpp"
  prior_header="${interfaces_prefix}/include/bio_nav_interfaces/bio_nav_interfaces/msg/detail/planning_prior__struct.hpp"
  require_file "${interfaces_prefix}/include/bio_nav_interfaces/bio_nav_interfaces/msg/local_risk_grid.hpp"
  require_file "${obstacle_header}"
  require_file "${prior_header}"
  grep -q 'observation_valid' "${obstacle_header}" || die \
    "bio_nav_interfaces underlay is stale: CognitiveObstacleArray.observation_valid is missing in ${install_real}; rebuild the selected snapshot/install"
  grep -q 'local_direction_schema_version' "${prior_header}" || die \
    "bio_nav_interfaces underlay is stale: PlanningPrior local-direction schema is missing in ${install_real}; rebuild the selected snapshot/install"
}

source_v6_integration_underlay() {
  require_file "${BIO_NAV_INTEGRATION_SETUP}"
  set +u
  # shellcheck disable=SC1090
  source "${BIO_NAV_INTEGRATION_SETUP}"
  set -u
  validate_v6_integration_underlay
  log_info "using allowed V6 Integration underlay: ${BIO_NAV_INTEGRATION_SETUP}"
}

source_ros() {
  local require_workspace=false
  local require_integration=false
  if [[ "${1:-}" == "--require-workspace" ]]; then
    require_workspace=true
  elif [[ "${1:-}" == "--require-integration-underlay" ]]; then
    require_integration=true
  elif [[ -n "${1:-}" ]]; then
    die "source_ros accepts only --require-workspace or --require-integration-underlay"
  fi
  if [[ "${ISAAC_NAV_REQUIRE_V6_INTEGRATION:-0}" == 1 ]]; then
    require_integration=true
  fi

  require_file "${ROS_SETUP}"
  if [[ "${require_integration}" == true ]]; then
    reset_ros_overlay_environment
  fi
  # ROS-generated setup scripts reference optional variables before defining
  # them, so nounset must be suspended only while they are sourced.
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"

  if [[ "${require_integration}" == true ]]; then
    source_v6_integration_underlay
  fi

  local workspace_setup="${ISAAC_NAV_WORKSPACE_SETUP:-}"
  local explicit_workspace_setup=false
  [[ -n "${workspace_setup}" ]] && explicit_workspace_setup=true
  if [[ -z "${workspace_setup}" && "${require_integration}" == true ]]; then
    # setup.bash replays the underlays captured at the previous build.  V6
    # already sourced its pinned Integration underlay explicitly, so source
    # only this worktree's overlay and do not reintroduce a stale underlay.
    workspace_setup="${PROJECT_ROOT}/ros2_ws/install/local_setup.bash"
  elif [[ -z "${workspace_setup}" ]]; then
    workspace_setup="${PROJECT_ROOT}/ros2_ws/install/setup.bash"
  fi
  if [[ -f "${workspace_setup}" && ("${require_workspace}" == true \
      || "${require_integration}" == false \
      || "${explicit_workspace_setup}" == true) ]]; then
    set +u
    # shellcheck disable=SC1091
    source "${workspace_setup}"
  elif [[ "${require_workspace}" == true ]]; then
    set -u
    die "ROS workspace is not built: ${workspace_setup}; run scripts/build_ros2.sh"
  fi
  set -u

  [[ "${ROS_DISTRO:-}" == "${ISAAC_NAV_EXPECTED_ROS_DISTRO}" ]] \
    || die "ROS_DISTRO must be ${ISAAC_NAV_EXPECTED_ROS_DISTRO}; got ${ROS_DISTRO:-unset}"
  if [[ "${require_integration}" == true ]]; then
    validate_v6_integration_underlay
  fi
  validate_runtime_environment
}

source_bio_nav_interfaces_underlay() {
  [[ -d "${PROJECT_ROOT}/ros2_ws/src/bio_nav_fusion" ]] || return 0
  source_v6_integration_underlay
}

prepare_runtime_directory() {
  require_command flock
  if [[ -L "${ISAAC_NAV_RUNTIME_DIR}" ]]; then
    die "runtime directory must not be a symlink: ${ISAAC_NAV_RUNTIME_DIR}"
  fi
  if [[ -e "${ISAAC_NAV_RUNTIME_DIR}" && ! -d "${ISAAC_NAV_RUNTIME_DIR}" ]]; then
    die "runtime path is not a directory: ${ISAAC_NAV_RUNTIME_DIR}"
  fi
  mkdir -p -m 700 "${ISAAC_NAV_RUNTIME_DIR}"
  local owner
  owner="$(stat -c '%u' "${ISAAC_NAV_RUNTIME_DIR}")"
  [[ "${owner}" == "${UID}" ]] \
    || die "runtime directory is not owned by uid ${UID}: ${ISAAC_NAV_RUNTIME_DIR}"
  chmod 700 "${ISAAC_NAV_RUNTIME_DIR}"
}

runtime_pid_file() {
  local component="$1"
  [[ "${component}" =~ ^[a-z0-9_]+$ ]] \
    || die "unsafe runtime component name: ${component}"
  printf '%s/%s.pid\n' "${ISAAC_NAV_RUNTIME_DIR}" "${component}"
}

runtime_lock_file() {
  local component="$1"
  [[ "${component}" =~ ^[a-z0-9_]+$ ]] \
    || die "unsafe runtime component name: ${component}"
  printf '%s/%s.lock\n' "${ISAAC_NAV_RUNTIME_DIR}" "${component}"
}

current_process_group() {
  local pid="${1:-$$}"
  ps -o pgid= -p "${pid}" | tr -d '[:space:]'
}

ensure_dedicated_process_group() {
  local current_group
  require_command ps
  require_command setsid
  current_group="$(current_process_group "$$")"
  [[ "${current_group}" =~ ^[0-9]+$ ]] \
    || die "cannot determine process group for pid $$"

  if [[ "${current_group}" == "$$" ]]; then
    export ISAAC_NAV_DEDICATED_PROCESS_GROUP=1
    return 0
  fi
  if [[ "${ISAAC_NAV_DEDICATED_PROCESS_GROUP:-}" == 1 ]]; then
    die "failed to create a dedicated process group for pid $$ (pgid ${current_group})"
  fi

  # A dedicated group lets clean_runtime stop every launch child after the
  # ros2/Isaac/RViz leader exits. Re-exec preserves the PID and lock identity.
  export ISAAC_NAV_DEDICATED_PROCESS_GROUP=1
  exec setsid -- "$0" "$@"
}

acquire_instance_lock() {
  local component="$1"
  local label="${2:-${component}}"
  local lock_file pid_file lock_fd process_group start_ticks boot_id
  prepare_runtime_directory
  lock_file="$(runtime_lock_file "${component}")"
  pid_file="$(runtime_pid_file "${component}")"

  exec {lock_fd}>"${lock_file}"
  if ! flock -n "${lock_fd}"; then
    local recorded_pid="unknown"
    if [[ -r "${pid_file}" ]]; then
      recorded_pid="$(sed -n 's/^pid=//p' "${pid_file}" | head -n 1)"
    fi
    die "${label} is already running (recorded pid: ${recorded_pid})"
  fi

  process_group="$(current_process_group "$$")"
  [[ "${process_group}" =~ ^[0-9]+$ ]] \
    || die "cannot determine process group for pid $$"
  start_ticks="$(awk '{print $22}' "/proc/$$/stat")"
  boot_id="$(< /proc/sys/kernel/random/boot_id)"
  {
    printf 'pid=%s\n' "$$"
    printf 'process_group=%s\n' "${process_group}"
    printf 'leader_start_ticks=%s\n' "${start_ticks}"
    printf 'boot_id=%s\n' "${boot_id}"
    printf 'component=%s\n' "${component}"
    printf 'project_root=%s\n' "${PROJECT_ROOT}"
    printf 'started_at=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  } >"${pid_file}"
  chmod 600 "${pid_file}" "${lock_file}"

  # Keep dynamically allocated descriptors reachable for shells that acquire
  # more than one role (for example ROS + integrated RViz).
  ISAAC_NAV_LOCK_FDS+=("${lock_fd}")
  log_info "acquired ${label} single-instance lock"
}

runtime_lock_is_held() {
  local component="$1"
  local lock_file
  prepare_runtime_directory
  lock_file="$(runtime_lock_file "${component}")"
  (
    exec 9>"${lock_file}"
    ! flock -n 9
  )
}

declare -ag ISAAC_NAV_LOCK_FDS=()
validate_runtime_environment
