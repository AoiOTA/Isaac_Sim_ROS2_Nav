#!/usr/bin/env bash
# One long-running stack adapter for a single Phase-F M0--M3 episode.
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  echo "usage: $0 M0|M1|M2|M3 --domain ID --run-dir PATH --socket PATH [--module2-root PATH]" >&2
  echo "       BIO_NAV_MODULE2_V310_ROOT or --module2-root must name the canonical Module2 root" >&2
  echo "       $0 stop-producer --run-dir PATH --socket PATH" >&2
}

group_is_running() {
  local pgid="$1"
  ps -eo pgid=,stat= | awk -v group="${pgid}" '
    $1 == group && $2 !~ /^Z/ { found = 1 }
    END { exit !found }
  '
}

signal_group() {
  local signal_name="$1" pgid="$2"
  group_is_running "${pgid}" || return 0
  kill "-${signal_name}" -- "-${pgid}" 2>/dev/null || true
}

wait_group_exit() {
  local pgid="$1" attempts="$2" index
  for ((index=0; index<attempts; index++)); do
    group_is_running "${pgid}" || return 0
    sleep 0.05
  done
  ! group_is_running "${pgid}"
}

socket_has_listener() {
  local path="$1"
  awk -v target="${path}" '
    NR > 1 && $8 == target && $4 == "00010000" && $5 == "0001" { found = 1 }
    END { exit !found }
  ' /proc/net/unix
}

socket_connects() {
  python3 - "$1" <<'PY'
import socket
import sys

probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
probe.settimeout(0.05)
try:
    probe.connect(sys.argv[1])
except OSError:
    raise SystemExit(1)
finally:
    probe.close()
PY
}

module2_recorded_process_running() {
  local directory="$1" name pid pgid
  for name in module2_server integration_bridge; do
    if [[ -f "${directory}/${name}.identity" ]]; then
      read -r pid pgid <"${directory}/${name}.identity" || return 0
    elif [[ -f "${directory}/${name}.pid" && -f "${directory}/${name}.pgid" ]]; then
      read -r pid <"${directory}/${name}.pid" || return 0
      read -r pgid <"${directory}/${name}.pgid" || return 0
    else
      continue
    fi
    [[ "${pgid}" =~ ^[1-9][0-9]*$ ]] && group_is_running "${pgid}" && return 0
    if [[ "${pid}" =~ ^[1-9][0-9]*$ && -r "/proc/${pid}/stat" ]]; then
      [[ "$(awk '{print $3}' "/proc/${pid}/stat" 2>/dev/null)" == Z ]] || return 0
    fi
  done
  return 1
}

cleanup_exact_socket() {
  local directory="$1" path="$2" check
  local quiet_checks="${BIO_NAV_PHASE_F_CLEANUP_QUIET_CHECKS:-5}"
  if module2_recorded_process_running "${directory}"; then
    echo "refusing socket cleanup while recorded Module2/Bridge process is active: ${path}" >&2
    return 1
  fi
  if socket_has_listener "${path}" || socket_connects "${path}"; then
    echo "refusing to unlink active Module2 socket: ${path}" >&2
    return 1
  fi
  rm -f -- "${path}"
  for ((check=0; check<quiet_checks; check++)); do
    if [[ -e "${path}" || -S "${path}" ]] \
      || socket_has_listener "${path}" || socket_connects "${path}"; then
      echo "Module2 socket reappeared during cleanup quiet window: ${path}" >&2
      return 1
    fi
    sleep 0.05
  done
}

stop_registered_group() {
  local name="$1" directory="$2" identity_file pid_file pgid_file pid pgid
  local int_checks="${BIO_NAV_PHASE_F_CLEANUP_INT_CHECKS:-100}"
  local term_checks="${BIO_NAV_PHASE_F_CLEANUP_TERM_CHECKS:-100}"
  identity_file="${directory}/${name}.identity"
  pid_file="${directory}/${name}.pid"
  pgid_file="${directory}/${name}.pgid"
  if [[ -f "${identity_file}" ]]; then
    read -r pid pgid <"${identity_file}"
  elif [[ -f "${pid_file}" && -f "${pgid_file}" ]]; then
    read -r pid <"${pid_file}"
    read -r pgid <"${pgid_file}"
  else
    echo "missing producer process identity: ${name}" >&2
    return 1
  fi
  [[ "${pid}" =~ ^[1-9][0-9]*$ && "${pgid}" =~ ^[1-9][0-9]*$ ]] || {
    echo "invalid producer process identity: ${name}" >&2
    return 1
  }
  signal_group INT "${pgid}"
  if ! wait_group_exit "${pgid}" "${int_checks}"; then
    signal_group TERM "${pgid}"
    if ! wait_group_exit "${pgid}" "${term_checks}"; then
      signal_group KILL "${pgid}"
      wait_group_exit "${pgid}" 100 || {
        echo "producer process group did not stop: ${name} pgid=${pgid}" >&2
        return 1
      }
    fi
  fi
  wait "${pid}" 2>/dev/null || true
  rm -f "${identity_file}" "${pid_file}" "${pgid_file}"
}

if [[ "${1:-}" == "stop-producer" ]]; then
  shift
  producer_run_dir=""
  producer_socket=""
  while (($#)); do
    case "$1" in
      --run-dir) producer_run_dir="${2:?--run-dir requires a path}"; shift 2 ;;
      --socket) producer_socket="${2:?--socket requires a path}"; shift 2 ;;
      *) usage; echo "unknown argument: $1" >&2; exit 2 ;;
    esac
  done
  [[ "${producer_run_dir}" == /* && "${producer_socket}" == /* ]] || { usage; exit 2; }
  for name in integration_bridge module2_server; do
    stop_registered_group "${name}" "${producer_run_dir}" || exit 1
  done
  cleanup_exact_socket "${producer_run_dir}" "${producer_socket}" || exit 1
  exit 0
fi

arm="${1:-}"
[[ "${arm}" =~ ^M[0-3]$ ]] || { usage; exit 2; }
shift
run_dir=""
socket_path=""
domain_id="${BIO_NAV_PHASE_F_DOMAIN_ID:-150}"
module2_root="${BIO_NAV_MODULE2_V310_ROOT:-}"
integration_root="${BIO_NAV_INTEGRATION_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_integration}"
while (($#)); do
  case "$1" in
    --run-dir) run_dir="${2:?--run-dir requires a path}"; shift 2 ;;
    --socket) socket_path="${2:?--socket requires a path}"; shift 2 ;;
    --domain) domain_id="${2:?--domain requires an ID}"; shift 2 ;;
    --module2-root) module2_root="${2:?--module2-root requires a path}"; shift 2 ;;
    *) usage; echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ -n "${run_dir}" && -n "${socket_path}" ]] || { usage; exit 2; }
[[ "${run_dir}" == /* && "${socket_path}" == /* ]] || {
  echo "run-dir and socket must be absolute" >&2
  exit 2
}
[[ "${domain_id}" =~ ^[0-9]+$ && "${domain_id}" -le 232 ]] || {
  echo "domain must be an integer in [0,232]" >&2
  exit 2
}
export ISAAC_NAV_EXPECTED_DOMAIN_ID="${domain_id}"
export ROS_DOMAIN_ID="${domain_id}"
# shellcheck source=lib/common.sh
source "${script_dir}/lib/common.sh"

require_directory "${integration_root}"
[[ -n "${module2_root}" ]] || {
  echo "BIO_NAV_MODULE2_V310_ROOT or --module2-root is required for the Phase-F map context" >&2
  exit 2
}
require_directory "${module2_root}"
module2_root="$(cd "${module2_root}" && pwd -P)"
canonical_constraints_file="${module2_root}/configs/kujiale_0026_module1_visual_shadow_v310.yaml"
require_file "${canonical_constraints_file}"
export BIO_NAV_MODULE2_V310_ROOT="${module2_root}"
if [[ "${arm}" != "M0" ]]; then
  require_file "${integration_root}/scripts/run_module2_v310_server.sh"
  require_file "${integration_root}/scripts/run_v6_module2_causal_obstacle_server.sh"
fi
mkdir -p "${run_dir}" "$(dirname "${socket_path}")"
cleanup_exact_socket "${run_dir}" "${socket_path}"

declare -a child_names=()
declare -a child_pids=()
declare -a child_pgids=()
terminating=false
termination_status=0

request_shutdown() {
  terminating=true
  termination_status="$1"
}

exit_if_terminating() {
  [[ "${terminating}" == false ]] || exit "${termination_status}"
}

register_child() {
  local name="$1" pid="$2" pgid="" index prefix
  for ((index=0; index<100; index++)); do
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
    [[ "${pgid}" =~ ^[1-9][0-9]*$ ]] && break
    sleep 0.01
  done
  [[ "${pgid}" =~ ^[1-9][0-9]*$ ]] || {
    echo "could not identify process group for ${name} pid=${pid}" >&2
    return 1
  }
  child_names+=("${name}")
  child_pids+=("${pid}")
  child_pgids+=("${pgid}")
  prefix="${run_dir}/.${name}.$$"
  printf '%s %s\n' "${pid}" "${pgid}" >"${prefix}.identity.tmp"
  mv -f "${prefix}.identity.tmp" "${run_dir}/${name}.identity"
  printf '%s\n' "${pid}" >"${prefix}.pid.tmp"
  mv -f "${prefix}.pid.tmp" "${run_dir}/${name}.pid"
  printf '%s\n' "${pgid}" >"${prefix}.pgid.tmp"
  mv -f "${prefix}.pgid.tmp" "${run_dir}/${name}.pgid"
}

descendant_groups() {
  local root_pid="$1" current child pgid
  local -a queue=("${root_pid}")
  local -A seen=(["${root_pid}"]=1)
  while ((${#queue[@]})); do
    current="${queue[0]}"
    queue=("${queue[@]:1}")
    while read -r child; do
      [[ "${child}" =~ ^[1-9][0-9]*$ && -z "${seen[${child}]:-}" ]] || continue
      seen["${child}"]=1
      queue+=("${child}")
      pgid="$(ps -o pgid= -p "${child}" 2>/dev/null | tr -d '[:space:]')"
      [[ "${pgid}" =~ ^[1-9][0-9]*$ ]] && printf '%s\n' "${pgid}"
    done < <(ps -o pid= --ppid "${current}" 2>/dev/null || true)
  done
}

shutdown() {
  local original_status="$1" index pid pgid failed=false own_pgid phase check
  local running quiet_checks=0
  local int_checks="${BIO_NAV_PHASE_F_CLEANUP_INT_CHECKS:-100}"
  local term_checks="${BIO_NAV_PHASE_F_CLEANUP_TERM_CHECKS:-100}"
  local quiet_target="${BIO_NAV_PHASE_F_CLEANUP_QUIET_CHECKS:-5}"
  local -a tracked_groups=()
  local -A unique_groups=()
  terminating=true
  trap - EXIT INT TERM HUP
  own_pgid="$(ps -o pgid= -p "$$" | tr -d '[:space:]')"

  # Re-read only children registered by this stack.  A child which creates a
  # new session while handling shutdown is picked up on the next descendant
  # scan, and every discovered group remains tracked for the rest of cleanup.
  for phase in INT TERM KILL; do
    quiet_checks=0
    if [[ "${phase}" == INT ]]; then
      check_limit="${int_checks}"
    elif [[ "${phase}" == TERM ]]; then
      check_limit="${term_checks}"
    else
      check_limit=100
    fi
    for ((check=0; check<check_limit; check++)); do
      for index in "${!child_names[@]}"; do
        [[ -f "${run_dir}/${child_names[index]}.identity" \
          || -f "${run_dir}/${child_names[index]}.pid" ]] || continue
        pid="${child_pids[index]}"
        unique_groups["${child_pgids[index]}"]=1
        while read -r pgid; do
          [[ "${pgid}" =~ ^[1-9][0-9]*$ ]] && unique_groups["${pgid}"]=1
        done < <(descendant_groups "${pid}")
      done
      running=false
      for pgid in "${!unique_groups[@]}"; do
        [[ "${pgid}" != "${own_pgid}" ]] || continue
        if group_is_running "${pgid}"; then
          running=true
          signal_group "${phase}" "${pgid}"
        fi
      done
      if [[ "${running}" == false ]]; then
        ((quiet_checks+=1))
        ((quiet_checks >= quiet_target)) && break
      else
        quiet_checks=0
      fi
      sleep 0.05
    done
    ((quiet_checks >= quiet_target)) && break
  done
  for pgid in "${!unique_groups[@]}"; do
    [[ "${pgid}" != "${own_pgid}" ]] || continue
    wait_group_exit "${pgid}" 100 || failed=true
  done
  for pid in "${child_pids[@]}"; do wait "${pid}" 2>/dev/null || true; done
  cleanup_exact_socket "${run_dir}" "${socket_path}" || failed=true
  for index in "${!child_names[@]}"; do
    rm -f "${run_dir}/${child_names[index]}.identity" \
      "${run_dir}/${child_names[index]}.pid" "${run_dir}/${child_names[index]}.pgid"
  done
  if [[ "${failed}" == true ]]; then
    echo "Phase-F stack cleanup left a tracked process group alive" >&2
    exit 1
  fi
  exit "${original_status}"
}
trap 'request_shutdown 130' INT
trap 'request_shutdown 143' TERM
trap 'request_shutdown 129' HUP
trap 'shutdown $?' EXIT

exit_if_terminating
setsid --wait -- "${script_dir}/run_v6_kujiale_low_obstacles.sh" ros "${arm}" \
  >"${run_dir}/module3_ros.log" 2>&1 &
module3_pid="$!"
register_child module3_ros "${module3_pid}"
exit_if_terminating

if [[ "${arm}" != "M0" ]]; then
  if [[ "${arm}" == "M1" ]]; then
    exit_if_terminating
    setsid --wait -- "${integration_root}/scripts/run_module2_v310_server.sh" \
      --module2-root "${module2_root}" \
      --shadow-config configs/kujiale_0026_module1_visual_shadow_v310.yaml \
      --socket "${socket_path}" \
      >"${run_dir}/module2_server.log" 2>&1 &
  else
    exit_if_terminating
    setsid --wait -- "${integration_root}/scripts/run_v6_module2_causal_obstacle_server.sh" \
      --startup-profile module2_causal_obstacle_active \
      --active-effect-scope obstacle_only \
      --socket "${socket_path}" \
      --module2-root "${module2_root}" \
      --shadow-config configs/kujiale_0026_module1_visual_shadow_v310.yaml \
      >"${run_dir}/module2_server.log" 2>&1 &
  fi
  module2_server_pid="$!"
  register_child module2_server "${module2_server_pid}"
  exit_if_terminating

  source_ros --require-integration-underlay
  startup_profile="estimated_shadow"
  [[ "${arm}" =~ ^M[23]$ ]] && startup_profile="module2_causal_obstacle_active"
  exit_if_terminating
  setsid --wait -- ros2 launch bio_nav_ros_bridge v6_cognitive_navigation.launch.py \
    startup_profile:="${startup_profile}" \
    socket_path:="${socket_path}" \
    use_sim_time:=true \
    >"${run_dir}/integration_bridge.log" 2>&1 &
  integration_bridge_pid="$!"
  register_child integration_bridge "${integration_bridge_pid}"
  exit_if_terminating
fi

set +e
wait "${module3_pid}"
status=$?
set -e
if [[ "${terminating}" == true ]]; then
  exit "${termination_status}"
fi
exit "${status}"
