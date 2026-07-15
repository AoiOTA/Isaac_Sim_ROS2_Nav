#!/usr/bin/env bash

set -Eeuo pipefail

export PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ISAAC_PYTHON="${ISAAC_PYTHON:-/home/lyb/miniconda3/envs/isaacsim/bin/python}"
export ISAAC_ASSET_ROOT="${ISAAC_ASSET_ROOT:-/home/lyb/isaacsim_assets/Assets/Isaac/6.0}"
export ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
export ISAAC_NAV_RUNTIME_DIR="${ISAAC_NAV_RUNTIME_DIR:-/tmp/isaac_sim_ros2_nav_${UID}}"

readonly ISAAC_NAV_EXPECTED_ROS_DISTRO="jazzy"
readonly ISAAC_NAV_EXPECTED_DOMAIN_ID="42"
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
}

source_ros() {
  local require_workspace=false
  if [[ "${1:-}" == "--require-workspace" ]]; then
    require_workspace=true
  elif [[ -n "${1:-}" ]]; then
    die "source_ros accepts only --require-workspace"
  fi

  require_file "${ROS_SETUP}"
  # ROS-generated setup scripts reference optional variables before defining
  # them, so nounset must be suspended only while they are sourced.
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"

  local workspace_setup="${PROJECT_ROOT}/ros2_ws/install/setup.bash"
  if [[ -f "${workspace_setup}" ]]; then
    # shellcheck disable=SC1091
    source "${workspace_setup}"
  elif [[ "${require_workspace}" == true ]]; then
    set -u
    die "ROS workspace is not built: ${workspace_setup}; run scripts/build_ros2.sh"
  fi
  set -u

  [[ "${ROS_DISTRO:-}" == "${ISAAC_NAV_EXPECTED_ROS_DISTRO}" ]] \
    || die "ROS_DISTRO must be ${ISAAC_NAV_EXPECTED_ROS_DISTRO}; got ${ROS_DISTRO:-unset}"
  validate_runtime_environment
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

runtime_metadata_value() {
  local file="$1"
  local key="$2"
  sed -n "s/^${key}=//p" "${file}" | head -n 1
}

runtime_process_start_ticks() {
  local pid="$1"
  [[ -r "/proc/${pid}/stat" ]] || return 1
  awk '{print $22}' "/proc/${pid}/stat"
}

runtime_process_is_running() {
  local pid="$1"
  local state
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
  kill -0 "${pid}" 2>/dev/null || return 1
  [[ -r "/proc/${pid}/stat" ]] || return 1
  state="$(awk '{print $3}' "/proc/${pid}/stat")"
  [[ "${state}" != "Z" ]]
}

runtime_process_group_members() {
  local process_group="$1"
  local stat_file pid stat_tail state parent_pid member_group inspector_pid
  [[ "${process_group}" =~ ^[0-9]+$ ]] || return 1
  inspector_pid="${BASHPID}"
  for stat_file in /proc/[0-9]*/stat; do
    [[ -r "${stat_file}" ]] || continue
    pid="${stat_file#/proc/}"
    pid="${pid%/stat}"
    # Command/process substitution runs this function in a short-lived Bash
    # process inside the caller's group.  Exclude that inspector so checking a
    # wrapper's own group does not manufacture a live descendant.
    if [[ "${inspector_pid}" != "$$" && "${pid}" == "${inspector_pid}" ]]; then
      continue
    fi
    IFS= read -r stat_tail <"${stat_file}" || continue
    stat_tail="${stat_tail##*) }"
    read -r state parent_pid member_group _ <<<"${stat_tail}"
    if [[ "${member_group}" == "${process_group}" && "${state}" != "Z" ]]; then
      printf '%s\n' "${pid}"
    fi
  done
}

runtime_process_group_is_running() {
  local process_group="$1"
  [[ -n "$(runtime_process_group_members "${process_group}" || true)" ]]
}

runtime_process_group_has_members_except() {
  local process_group="$1"
  local excluded_pid="$2"
  local member
  while IFS= read -r member; do
    [[ -n "${member}" && "${member}" != "${excluded_pid}" ]] || continue
    return 0
  done < <(runtime_process_group_members "${process_group}" || true)
  return 1
}

runtime_process_environment_value() {
  local pid="$1"
  local key="$2"
  [[ -r "/proc/${pid}/environ" ]] || return 1
  tr '\0' '\n' <"/proc/${pid}/environ" \
    | sed -n "s/^${key}=//p" \
    | head -n 1
}

runtime_process_command() {
  local pid="$1"
  [[ -r "/proc/${pid}/cmdline" ]] || return 1
  tr '\0' ' ' <"/proc/${pid}/cmdline"
}

runtime_component_command_matches() {
  local component="$1"
  local command_line="$2"
  case "${component}" in
    ros)
      [[ "${command_line}" == *"ros2"*"launch"*"robot_bringup"* \
        || "${command_line}" == *"${PROJECT_ROOT}/scripts/run_ros.sh"* \
        || "${command_line}" == *" scripts/run_ros.sh "* ]]
      ;;
    rviz)
      [[ "${command_line}" == *"rviz2"* \
        || "${command_line}" == *"${PROJECT_ROOT}/scripts/run_rviz.sh"* \
        || "${command_line}" == *" scripts/run_rviz.sh "* ]]
      ;;
    teleop)
      [[ "${command_line}" == *"robot_teleop"*"keyboard_teleop"* \
        || "${command_line}" == *"${PROJECT_ROOT}/scripts/run_teleop.sh"* \
        || "${command_line}" == *" scripts/run_teleop.sh "* ]]
      ;;
    motion_baseline)
      [[ "${command_line}" == *"/lib/robot_experiments/motion_baseline_runner"* \
        || "${command_line}" == *"/bin/ros2 run robot_experiments motion_baseline_runner"* ]]
      ;;
    isaac)
      [[ "${command_line}" == *"${PROJECT_ROOT}/isaac_sim/apps/navigation_sim.py"* ]]
      ;;
    *)
      return 1
      ;;
  esac
}

start_runtime_session() {
  local start_ticks boot_id
  start_ticks="$(runtime_process_start_ticks "$$")" \
    || die "cannot determine process start identity for pid $$"
  boot_id="$(< /proc/sys/kernel/random/boot_id)"
  export ISAAC_NAV_SESSION_ID="${boot_id}:$$:${start_ticks}"
}

runtime_process_identity_matches_session() {
  local member="$1"
  local expected_start="$2"
  local expected_root="$3"
  local expected_session="$4"
  local member_uid member_start_before member_start_after
  local member_root member_session
  member_uid="$(stat -c '%u' "/proc/${member}" 2>/dev/null || true)"
  member_start_before="$(runtime_process_start_ticks "${member}" || true)"
  member_root="$(
    runtime_process_environment_value "${member}" PROJECT_ROOT || true
  )"
  member_session="$(
    runtime_process_environment_value "${member}" ISAAC_NAV_SESSION_ID || true
  )"
  member_start_after="$(runtime_process_start_ticks "${member}" || true)"
  [[ "${member_uid}" == "${UID}" \
        && "${member_start_before}" == "${expected_start}" \
        && "${member_start_after}" == "${expected_start}" \
        && "${member_root}" == "${expected_root}" \
        && "${member_session}" == "${expected_session}" ]]
}

runtime_process_group_is_owned_by_session() {
  local process_group="$1"
  local expected_root="$2"
  local expected_session="$3"
  local member member_start identity_attempt authenticated disappeared
  local found=false
  [[ -n "${expected_session}" ]] || return 1
  while IFS= read -r member; do
    [[ -n "${member}" ]] || continue
    runtime_process_is_running "${member}" || continue
    member_start="$(runtime_process_start_ticks "${member}" || true)"
    if [[ ! "${member_start}" =~ ^[0-9]+$ ]]; then
      runtime_process_is_running "${member}" || continue
      log_warn "refusing process group ${process_group}: member ${member} start identity unavailable"
      return 1
    fi
    authenticated=false
    disappeared=false
    # A short-lived child can be observed while exec is replacing its address
    # space, when /proc/<pid>/environ may momentarily read as empty.  Require a
    # stable mismatch across three reads before rejecting the whole group.
    for ((identity_attempt = 1; identity_attempt <= 3; identity_attempt++)); do
      if runtime_process_identity_matches_session \
          "${member}" "${member_start}" \
          "${expected_root}" "${expected_session}"; then
        authenticated=true
        break
      fi
      if ! runtime_process_is_running "${member}"; then
        disappeared=true
        break
      fi
      if ((identity_attempt < 3)); then
        sleep 0.01
      fi
    done
    [[ "${disappeared}" == false ]] || continue
    if [[ "${authenticated}" != true ]]; then
      log_warn "refusing process group ${process_group}: member ${member} session identity mismatch"
      return 1
    fi
    found=true
  done < <(runtime_process_group_members "${process_group}" || true)
  [[ "${found}" == true ]]
}

runtime_registered_process_group() {
  local component="$1"
  local expected_session="$2"
  local pid_file pid process_group recorded_component recorded_root
  local recorded_session recorded_boot current_boot leader_start actual_start
  local actual_group command_line owner
  pid_file="$(runtime_pid_file "${component}")"
  [[ -r "${pid_file}" && -f "${pid_file}" && ! -L "${pid_file}" ]] \
    || return 1
  owner="$(stat -c '%u' "${pid_file}" 2>/dev/null || true)"
  [[ "${owner}" == "${UID}" ]] || {
    log_warn "refusing ${component} metadata not owned by uid ${UID}: ${pid_file}"
    return 1
  }
  pid="$(runtime_metadata_value "${pid_file}" pid)"
  process_group="$(runtime_metadata_value "${pid_file}" process_group)"
  leader_start="$(runtime_metadata_value "${pid_file}" leader_start_ticks)"
  recorded_boot="$(runtime_metadata_value "${pid_file}" boot_id)"
  recorded_component="$(runtime_metadata_value "${pid_file}" component)"
  recorded_root="$(runtime_metadata_value "${pid_file}" project_root)"
  recorded_session="$(runtime_metadata_value "${pid_file}" session_id)"
  current_boot="$(< /proc/sys/kernel/random/boot_id)"
  if [[ "${recorded_component}" != "${component}" \
        || "${recorded_root}" != "${PROJECT_ROOT}" \
        || "${recorded_session}" != "${expected_session}" \
        || "${recorded_boot}" != "${current_boot}" \
        || ! "${pid}" =~ ^[0-9]+$ \
        || ! "${process_group}" =~ ^[0-9]+$ \
        || "${process_group}" != "${pid}" ]]; then
    log_warn "refusing ${component} metadata outside the current runtime session"
    return 1
  fi

  if runtime_process_is_running "${pid}"; then
    actual_start="$(runtime_process_start_ticks "${pid}" || true)"
    actual_group="$(current_process_group "${pid}" || true)"
    command_line="$(runtime_process_command "${pid}" || true)"
    if [[ "${actual_start}" != "${leader_start}" \
          || "${actual_group}" != "${process_group}" ]] \
        || ! runtime_component_command_matches "${component}" "${command_line}"; then
      if runtime_process_is_running "${pid}"; then
        log_warn "refusing ${component} process group ${process_group}: leader identity mismatch"
        return 1
      fi
    fi
  fi

  runtime_process_group_is_running "${process_group}" || return 1
  runtime_process_group_is_owned_by_session \
    "${process_group}" "${PROJECT_ROOT}" "${expected_session}" || return 1
  printf '%s\n' "${process_group}"
}

remove_runtime_session_metadata() {
  local component="$1"
  local expected_session="$2"
  local pid_file process_group recorded_component recorded_root recorded_session
  local recorded_boot current_boot owner
  pid_file="$(runtime_pid_file "${component}")"
  [[ -e "${pid_file}" ]] || return 0
  if [[ ! -f "${pid_file}" || -L "${pid_file}" ]]; then
    log_warn "refusing unsafe ${component} metadata path: ${pid_file}"
    return 1
  fi
  recorded_component="$(runtime_metadata_value "${pid_file}" component)"
  recorded_root="$(runtime_metadata_value "${pid_file}" project_root)"
  recorded_session="$(runtime_metadata_value "${pid_file}" session_id)"
  recorded_boot="$(runtime_metadata_value "${pid_file}" boot_id)"
  process_group="$(runtime_metadata_value "${pid_file}" process_group)"
  current_boot="$(< /proc/sys/kernel/random/boot_id)"
  owner="$(stat -c '%u' "${pid_file}" 2>/dev/null || true)"
  if [[ "${recorded_component}" != "${component}" \
        || "${recorded_root}" != "${PROJECT_ROOT}" \
        || "${recorded_session}" != "${expected_session}" \
        || "${recorded_boot}" != "${current_boot}" \
        || "${owner}" != "${UID}" ]]; then
    return 1
  fi
  if [[ "${process_group}" =~ ^[0-9]+$ ]] \
      && runtime_process_group_is_running "${process_group}"; then
    log_warn "not removing live ${component} metadata for process group ${process_group}"
    return 1
  fi
  rm -f -- "${pid_file}"
}

release_instance_lock() {
  local component="$1"
  local pid_file pid process_group recorded_start actual_start recorded_boot current_boot
  local recorded_component recorded_root recorded_session
  pid_file="$(runtime_pid_file "${component}")"
  [[ -e "${pid_file}" ]] || return 0
  if [[ ! -f "${pid_file}" || -L "${pid_file}" ]]; then
    log_warn "refusing unsafe ${component} metadata path: ${pid_file}"
    return 1
  fi
  pid="$(runtime_metadata_value "${pid_file}" pid)"
  process_group="$(runtime_metadata_value "${pid_file}" process_group)"
  recorded_start="$(runtime_metadata_value "${pid_file}" leader_start_ticks)"
  recorded_boot="$(runtime_metadata_value "${pid_file}" boot_id)"
  recorded_component="$(runtime_metadata_value "${pid_file}" component)"
  recorded_root="$(runtime_metadata_value "${pid_file}" project_root)"
  recorded_session="$(runtime_metadata_value "${pid_file}" session_id)"
  actual_start="$(runtime_process_start_ticks "$$" || true)"
  current_boot="$(< /proc/sys/kernel/random/boot_id)"
  if [[ "${pid}" != "$$" \
        || "${recorded_start}" != "${actual_start}" \
        || "${recorded_boot}" != "${current_boot}" \
        || "${recorded_component}" != "${component}" \
        || "${recorded_root}" != "${PROJECT_ROOT}" \
        || "${recorded_session}" != "${ISAAC_NAV_SESSION_ID:-}" ]]; then
    log_warn "refusing to remove ${component} metadata with a changed identity"
    return 1
  fi
  if [[ "${process_group}" =~ ^[0-9]+$ ]] \
      && runtime_process_group_has_members_except "${process_group}" "$$"; then
    log_warn "refusing to remove ${component} metadata while process group ${process_group} has live descendants"
    return 1
  fi
  rm -f -- "${pid_file}"
}

close_instance_lock_fds_for_child() {
  local lock_fd
  for lock_fd in "${ISAAC_NAV_LOCK_FDS[@]}"; do
    [[ "${lock_fd}" =~ ^[0-9]+$ ]] \
      || die "invalid runtime lock descriptor: ${lock_fd}"
    exec {lock_fd}>&-
  done
  ISAAC_NAV_LOCK_FDS=()
}

ensure_dedicated_process_group() {
  local current_group
  require_command env
  require_command ps
  require_command setsid
  current_group="$(current_process_group "$$")"
  [[ "${current_group}" =~ ^[0-9]+$ ]] \
    || die "cannot determine process group for pid $$"

  if [[ "${ISAAC_NAV_DEDICATED_PROCESS_GROUP:-}" == 1 ]]; then
    [[ "${current_group}" == "$$" ]] && return 0
    die "failed to create a dedicated process group for pid $$ (pgid ${current_group})"
  fi

  # Re-exec once even when this shell is already a group leader. Variables
  # exported after Bash started are otherwise absent from /proc/<pid>/environ,
  # which prevents an independent cleanup process from authenticating it.
  export ISAAC_NAV_DEDICATED_PROCESS_GROUP=1
  if [[ "${current_group}" == "$$" ]]; then
    exec env \
      --default-signal=INT \
      --default-signal=TERM \
      --default-signal=HUP \
      -- "$0" "$@"
  fi

  # A dedicated group lets clean_runtime stop every launch child after the
  # ros2/Isaac/RViz leader exits. Re-exec preserves the PID and lock identity.
  exec env \
    --default-signal=INT \
    --default-signal=TERM \
    --default-signal=HUP \
    setsid -- "$0" "$@"
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
  if [[ -z "${ISAAC_NAV_SESSION_ID:-}" ]]; then
    export ISAAC_NAV_SESSION_ID="${boot_id}:$$:${start_ticks}"
  fi
  {
    printf 'pid=%s\n' "$$"
    printf 'process_group=%s\n' "${process_group}"
    printf 'leader_start_ticks=%s\n' "${start_ticks}"
    printf 'boot_id=%s\n' "${boot_id}"
    printf 'component=%s\n' "${component}"
    printf 'project_root=%s\n' "${PROJECT_ROOT}"
    printf 'session_id=%s\n' "${ISAAC_NAV_SESSION_ID}"
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
