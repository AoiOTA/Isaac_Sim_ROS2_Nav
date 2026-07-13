#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
source_ros

usage() {
  cat <<'EOF'
usage: clean_runtime.sh [--dry-run] [--dds-shm]

Safely stop only registered Isaac Nav processes. --dds-shm additionally
removes Fast DDS shared-memory artifacts owned by the current user, but only
after proving that no process still has Fast DDS loaded.
EOF
}

dry_run=false
clean_dds_shm=false
while (($#)); do
  case "$1" in
    --dry-run) dry_run=true ;;
    --dds-shm) clean_dds_shm=true ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; die "unknown argument: $1" ;;
  esac
  shift
done

prepare_runtime_directory
shm_root="${ISAAC_NAV_SHM_ROOT:-/dev/shm}"
dds_proc_root="${ISAAC_NAV_DDS_PROC_ROOT:-/proc}"
require_directory "${shm_root}"
require_directory "${dds_proc_root}"
if [[ "${dds_proc_root}" != "/proc" && "${shm_root}" == "/dev/shm" ]]; then
  die "a test DDS proc root may only be used with a non-system SHM root"
fi

metadata_value() {
  local file="$1"
  local key="$2"
  sed -n "s/^${key}=//p" "${file}" | head -n 1
}

process_command() {
  local pid="$1"
  [[ -r "/proc/${pid}/cmdline" ]] || return 1
  tr '\0' ' ' <"/proc/${pid}/cmdline"
}

process_start_ticks() {
  local pid="$1"
  [[ -r "/proc/${pid}/stat" ]] || return 1
  awk '{print $22}' "/proc/${pid}/stat"
}

process_environment_value() {
  local pid="$1"
  local key="$2"
  [[ -r "/proc/${pid}/environ" ]] || return 1
  tr '\0' '\n' <"/proc/${pid}/environ" \
    | sed -n "s/^${key}=//p" \
    | head -n 1
}

process_group_members() {
  local process_group="$1"
  ps -eo pid=,pgid=,stat= | awk -v wanted="${process_group}" '
    $2 == wanted && $3 !~ /^Z/ { print $1 }
  '
}

process_group_is_running() {
  local process_group="$1"
  [[ -n "$(process_group_members "${process_group}")" ]]
}

registered_group_is_safe() {
  local process_group="$1"
  local recorded_root="$2"
  local member member_root member_uid found=false
  while IFS= read -r member; do
    [[ -n "${member}" ]] || continue
    found=true
    member_uid="$(stat -c '%u' "/proc/${member}" 2>/dev/null || true)"
    member_root="$(process_environment_value "${member}" PROJECT_ROOT || true)"
    if [[ "${member_uid}" != "${UID}" || "${member_root}" != "${recorded_root}" ]]; then
      log_warn "refusing process group ${process_group}: member ${member} identity mismatch"
      return 1
    fi
  done < <(process_group_members "${process_group}")
  [[ "${found}" == true ]]
}

matches_registered_component() {
  local component="$1"
  local command_line="$2"
  case "${component}" in
    isaac)
      [[ "${command_line}" == *"${PROJECT_ROOT}/isaac_sim/apps/navigation_sim.py"* ]]
      ;;
    ros)
      [[ "${command_line}" == *"ros2"*"launch"*"robot_bringup"* ]]
      ;;
    rviz)
      [[ "${command_line}" == *"rviz2"* ]]
      ;;
    teleop)
      [[ "${command_line}" == *"robot_teleop"*"keyboard_teleop"* ]]
      ;;
    *)
      return 1
      ;;
  esac
}

wait_for_exit() {
  local pid="$1"
  local attempts="$2"
  local index
  for ((index = 0; index < attempts; index++)); do
    process_is_running "${pid}" || return 0
    sleep 0.1
  done
  return 1
}

wait_for_group_exit() {
  local process_group="$1"
  local attempts="$2"
  local index
  for ((index = 0; index < attempts; index++)); do
    process_group_is_running "${process_group}" || return 0
    sleep 0.1
  done
  return 1
}

process_is_running() {
  local pid="$1"
  local state
  kill -0 "${pid}" 2>/dev/null || return 1
  [[ -r "/proc/${pid}/stat" ]] || return 1
  state="$(awk '{print $3}' "/proc/${pid}/stat")"
  [[ "${state}" != "Z" ]]
}

stop_registered_component() {
  local component="$1"
  local pid_file pid recorded_root command_line process_group
  local leader_start_ticks recorded_boot_id current_boot_id actual_start_ticks
  local leader_running=false group_mode=false
  pid_file="$(runtime_pid_file "${component}")"
  [[ -r "${pid_file}" ]] || return 0
  pid="$(metadata_value "${pid_file}" pid)"
  recorded_root="$(metadata_value "${pid_file}" project_root)"
  process_group="$(metadata_value "${pid_file}" process_group)"
  leader_start_ticks="$(metadata_value "${pid_file}" leader_start_ticks)"
  recorded_boot_id="$(metadata_value "${pid_file}" boot_id)"

  if [[ "${recorded_root}" != "${PROJECT_ROOT}" ]]; then
    log_warn "refusing ${component} pid ${pid}: metadata belongs to ${recorded_root:-unknown}"
    return 1
  fi
  if [[ "${pid}" =~ ^[0-9]+$ ]] && process_is_running "${pid}"; then
    leader_running=true
    command_line="$(process_command "${pid}" || true)"
    if ! matches_registered_component "${component}" "${command_line}"; then
      log_warn "refusing ${component} pid ${pid}: command identity mismatch: ${command_line:-unreadable}"
      return 1
    fi
    if [[ -n "${leader_start_ticks}" ]]; then
      actual_start_ticks="$(process_start_ticks "${pid}" || true)"
      if [[ "${actual_start_ticks}" != "${leader_start_ticks}" ]]; then
        log_warn "refusing ${component} pid ${pid}: process start identity mismatch"
        return 1
      fi
    fi
  fi

  if [[ "${process_group}" =~ ^[0-9]+$ \
        && "${process_group}" == "${pid}" \
        && "${recorded_boot_id}" != "" ]]; then
    current_boot_id="$(< /proc/sys/kernel/random/boot_id)"
    if [[ "${recorded_boot_id}" != "${current_boot_id}" ]]; then
      log_warn "refusing ${component} group ${process_group}: metadata is from another boot"
      return 1
    fi
    if process_group_is_running "${process_group}"; then
      registered_group_is_safe "${process_group}" "${recorded_root}" || return 1
      group_mode=true
    fi
  fi

  if [[ "${leader_running}" == false && "${group_mode}" == false ]]; then
    log_warn "removing stale ${component} metadata: ${pid_file}"
    [[ "${dry_run}" == true ]] || rm -f -- "${pid_file}"
    return 0
  fi

  if [[ "${dry_run}" == true ]]; then
    if [[ "${group_mode}" == true ]]; then
      log_info "would stop ${component} process group ${process_group} (leader pid ${pid})"
    else
      log_info "would stop ${component} pid ${pid}: ${command_line}"
    fi
    return 0
  fi

  if [[ "${group_mode}" == true ]]; then
    log_info "stopping ${component} process group ${process_group} with SIGINT"
    kill -INT -- "-${process_group}"
    if ! wait_for_group_exit "${process_group}" 100; then
      log_warn "${component} group ${process_group} did not finish after SIGINT; sending SIGTERM"
      kill -TERM -- "-${process_group}" 2>/dev/null || true
      wait_for_group_exit "${process_group}" 50 \
        || die "${component} group ${process_group} did not stop; refusing SIGKILL"
    fi
  else
    log_warn "${component} uses legacy PID-only metadata; descendants cannot be verified"
    log_info "stopping ${component} pid ${pid} with SIGINT"
    kill -INT "${pid}"
    if ! wait_for_exit "${pid}" 100; then
      log_warn "${component} pid ${pid} ignored SIGINT; sending SIGTERM"
      kill -TERM "${pid}" 2>/dev/null || true
      wait_for_exit "${pid}" 50 \
        || die "${component} pid ${pid} did not stop; refusing SIGKILL"
    fi
  fi
  rm -f -- "${pid_file}"
}

list_project_processes() {
  log_info "project-related processes after registered cleanup:"
  local matches
  matches="$(ps -eo pid=,user=,args= | awk -v root="${PROJECT_ROOT}" -v self="$$" '
    $1 != self && $0 !~ /clean_runtime[.]sh/ && $0 !~ /awk -v root=/ &&
    (index($0, root "/isaac_sim/apps/navigation_sim.py") ||
    $0 ~ /ros2 launch robot_bringup/ ||
    $0 ~ /robot_teleop.*keyboard_teleop/ || $0 ~ /(^|[[:space:]])rviz2([[:space:]]|$)/) {
      print
    }
  ')"
  if [[ -n "${matches}" ]]; then
    printf '%s\n' "${matches}"
  else
    printf '  none\n'
  fi
}

active_fastdds_processes() {
  local maps pid command_line
  for maps in "${dds_proc_root}"/[0-9]*/maps; do
    [[ -r "${maps}" ]] || continue
    pid="${maps#/proc/}"
    pid="${pid%/maps}"
    [[ "${pid}" != "$$" ]] || continue
    if grep -Eq 'lib(rmw_)?fastrtps|libfastdds' "${maps}" 2>/dev/null; then
      command_line="$(process_command "${pid}" || true)"
      printf '%s\t%s\n' "${pid}" "${command_line:-unreadable}"
    fi
  done
}

dds_shm_candidates() {
  local candidate
  shopt -s nullglob
  for candidate in \
    "${shm_root}"/fastrtps_* \
    "${shm_root}"/fastdds_* \
    "${shm_root}"/sem.fastrtps_* \
    "${shm_root}"/sem.fastdds_*; do
    printf '%s\n' "${candidate}"
  done
  shopt -u nullglob
}

clean_fastdds_shm() {
  local active candidates candidate owner open_pids
  candidates="$(dds_shm_candidates)"
  if [[ -z "${candidates}" ]]; then
    log_info "Fast DDS SHM: no matching artifacts under ${shm_root}"
    return 0
  fi

  if [[ "${dry_run}" == false ]]; then
    log_info "stopping the ROS 2 CLI daemon for domain ${ROS_DOMAIN_ID} before SHM inspection"
    timeout 5 ros2 daemon stop >/dev/null 2>&1 || true
  fi

  active="$(active_fastdds_processes)"
  if [[ -n "${active}" ]]; then
    log_warn "Fast DDS is still loaded by active processes; refusing SHM deletion:"
    printf '%s\n' "${active}" >&2
    return 1
  fi

  while IFS= read -r candidate; do
    [[ -n "${candidate}" ]] || continue
    if [[ -L "${candidate}" || ! -f "${candidate}" ]]; then
      log_warn "refusing non-regular SHM candidate: ${candidate}"
      continue
    fi
    if command -v fuser >/dev/null 2>&1; then
      open_pids="$(fuser "${candidate}" 2>/dev/null || true)"
      if [[ -n "${open_pids}" ]]; then
        log_warn "refusing open SHM artifact ${candidate}; pids:${open_pids}"
        continue
      fi
    fi
    owner="$(stat -c '%u' "${candidate}")"
    if [[ "${owner}" != "${UID}" ]]; then
      log_warn "not removing SHM artifact owned by uid ${owner}: ${candidate}"
      printf '  manual review: sudo rm -- %q\n' "${candidate}" >&2
      continue
    fi
    if [[ "${dry_run}" == true ]]; then
      log_info "would remove Fast DDS SHM artifact: ${candidate}"
    else
      rm -f -- "${candidate}"
      log_info "removed Fast DDS SHM artifact: ${candidate}"
    fi
  done <<<"${candidates}"
}

log_info "runtime cleanup start (dry_run=${dry_run}, dds_shm=${clean_dds_shm})"
cleanup_failed=false
for component in teleop rviz ros isaac; do
  stop_registered_component "${component}" || cleanup_failed=true
done
list_project_processes
if [[ "${clean_dds_shm}" == true ]]; then
  clean_fastdds_shm || cleanup_failed=true
fi
log_info "runtime cleanup complete"
[[ "${cleanup_failed}" == false ]] || exit 1
