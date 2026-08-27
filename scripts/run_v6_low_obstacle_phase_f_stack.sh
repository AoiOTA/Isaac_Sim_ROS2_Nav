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

any_group_is_running() {
  ps -eo pgid=,stat= | awk '
    BEGIN {
      for (index = 1; index < ARGC; index++) {
        wanted[ARGV[index]] = 1
        delete ARGV[index]
      }
    }
    $1 in wanted && $2 !~ /^Z/ { found = 1; exit }
    END { exit !found }
  ' "$@"
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

process_group_of_pid() {
  local pid="$1"
  ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]'
}

process_is_running() {
  local pid="$1" state
  [[ "${pid}" =~ ^[1-9][0-9]*$ && -r "/proc/${pid}/stat" ]] || return 1
  state="$(awk '{print $3}' "/proc/${pid}/stat" 2>/dev/null)"
  [[ -n "${state}" && "${state}" != Z ]]
}

read_recorded_identity() {
  local name="$1" directory="$2" pid_variable="$3" pgid_variable="$4"
  local identity_file pid_file pgid_file recorded_pid recorded_pgid extra=""
  identity_file="${directory}/${name}.identity"
  pid_file="${directory}/${name}.pid"
  pgid_file="${directory}/${name}.pgid"
  if [[ -f "${identity_file}" ]]; then
    read -r recorded_pid recorded_pgid extra <"${identity_file}" || return 1
    [[ -z "${extra}" ]] || return 1
  elif [[ -f "${pid_file}" && -f "${pgid_file}" ]]; then
    read -r recorded_pid <"${pid_file}" || return 1
    read -r recorded_pgid <"${pgid_file}" || return 1
  else
    return 1
  fi
  [[ "${recorded_pid}" =~ ^[1-9][0-9]*$ \
      && "${recorded_pgid}" =~ ^[1-9][0-9]*$ ]] || return 1
  printf -v "${pid_variable}" '%s' "${recorded_pid}"
  printf -v "${pgid_variable}" '%s' "${recorded_pgid}"
}

recorded_identity_is_running() {
  local pid="$1" pgid="$2" actual_pgid
  process_is_running "${pid}" || return 1
  actual_pgid="$(process_group_of_pid "${pid}")"
  [[ "${actual_pgid}" == "${pgid}" ]] || return 1
  group_is_running "${pgid}"
}

write_process_identity() {
  local name="$1" directory="$2" pid="$3" pgid="$4" prefix
  prefix="${directory}/.${name}.$$"
  printf '%s %s\n' "${pid}" "${pgid}" >"${prefix}.identity.tmp"
  mv -f "${prefix}.identity.tmp" "${directory}/${name}.identity"
  printf '%s\n' "${pid}" >"${prefix}.pid.tmp"
  mv -f "${prefix}.pid.tmp" "${directory}/${name}.pid"
  printf '%s\n' "${pgid}" >"${prefix}.pgid.tmp"
  mv -f "${prefix}.pgid.tmp" "${directory}/${name}.pgid"
}

module2_recorded_process_running() {
  local directory="$1" name pid pgid
  for name in module2_server integration_bridge; do
    if [[ ! -f "${directory}/${name}.identity" \
        && ! -f "${directory}/${name}.pid" \
        && ! -f "${directory}/${name}.pgid" ]]; then
      continue
    fi
    # A malformed partial identity is not proof that cleanup may unlink a
    # possibly-live producer socket.
    read_recorded_identity "${name}" "${directory}" pid pgid || return 0
    recorded_identity_is_running "${pid}" "${pgid}" && return 0
  done
  return 1
}

cleanup_exact_socket() {
  local directory="$1" path="$2" check
  local quiet_checks="${3:-${BIO_NAV_PHASE_F_CLEANUP_QUIET_CHECKS:-5}}"
  local quiet_sleep_sec="${4:-0.05}"
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
    sleep "${quiet_sleep_sec}"
  done
}

fast_stop_registered_groups() {
  local directory="$1" name pid pgid check
  local term_checks=10
  local kill_checks=10
  local -a names=(integration_bridge module2_server)
  local -a pgids=()

  # Fault injection is deliberately abrupt.  Resolve both isolated producer
  # groups first, then signal them back-to-back so their shared TERM grace
  # window cannot consume the obstacle TTL serially.
  for name in "${names[@]}"; do
    if ! read_recorded_identity "${name}" "${directory}" pid pgid; then
      echo "missing producer process identity: ${name}" >&2
      return 1
    fi
    if ! recorded_identity_is_running "${pid}" "${pgid}"; then
      echo "producer process identity is not running: ${name} pid=${pid} pgid=${pgid}" >&2
      return 1
    fi
    pgids+=("${pgid}")
  done
  for pgid in "${pgids[@]}"; do
    kill -TERM -- "-${pgid}" 2>/dev/null || true
  done

  for ((check=0; check<term_checks; check++)); do
    any_group_is_running "${pgids[@]}" || break
    sleep 0.02
  done
  for pgid in "${pgids[@]}"; do
    if group_is_running "${pgid}"; then
      kill -KILL -- "-${pgid}" 2>/dev/null || true
    fi
  done
  for ((check=0; check<kill_checks; check++)); do
    any_group_is_running "${pgids[@]}" || break
    sleep 0.01
  done
  any_group_is_running "${pgids[@]}" && {
    echo "producer process groups did not stop: ${pgids[*]}" >&2
    return 1
  }
  for name in "${names[@]}"; do
    rm -f "${directory}/${name}.identity" \
      "${directory}/${name}.pid" "${directory}/${name}.pgid"
  done
}

validate_producer_stop_isolation() {
  local directory="$1" stack_pid stack_pgid module3_pid module3_pgid
  local module2_pid module2_pgid bridge_pid bridge_pgid
  read_recorded_identity stack "${directory}" stack_pid stack_pgid || {
    echo "missing stack process identity" >&2
    return 1
  }
  read_recorded_identity module3_ros "${directory}" module3_pid module3_pgid || {
    echo "missing Module3 ROS consumer identity" >&2
    return 1
  }
  read_recorded_identity module2_server "${directory}" module2_pid module2_pgid || {
    echo "missing producer process identity: module2_server" >&2
    return 1
  }
  read_recorded_identity integration_bridge "${directory}" bridge_pid bridge_pgid || {
    echo "missing producer process identity: integration_bridge" >&2
    return 1
  }
  recorded_identity_is_running "${stack_pid}" "${stack_pgid}" || {
    echo "Phase-F stack identity is not running" >&2
    return 1
  }
  recorded_identity_is_running "${module3_pid}" "${module3_pgid}" || {
    echo "Module3 ROS consumer identity is not running" >&2
    return 1
  }
  recorded_identity_is_running "${module2_pid}" "${module2_pgid}" || {
    echo "Module2 producer identity is not running" >&2
    return 1
  }
  recorded_identity_is_running "${bridge_pid}" "${bridge_pgid}" || {
    echo "Integration bridge identity is not running" >&2
    return 1
  }
  if [[ "${stack_pgid}" == "${module3_pgid}" \
      || "${module2_pgid}" == "${stack_pgid}" \
      || "${module2_pgid}" == "${module3_pgid}" \
      || "${bridge_pgid}" == "${stack_pgid}" \
      || "${bridge_pgid}" == "${module3_pgid}" \
      || "${bridge_pgid}" == "${module2_pgid}" ]]; then
    echo "producer-stop process groups are not isolated from the Phase-F stack/consumer" >&2
    return 1
  fi
}

ros_lock_is_owned() {
  local runtime_dir lock_file lock_fd
  runtime_dir="${ISAAC_NAV_RUNTIME_DIR:-/tmp/isaac_sim_ros2_nav_${UID}}"
  lock_file="${runtime_dir}/ros.lock"
  [[ -f "${lock_file}" ]] || return 1
  exec {lock_fd}<>"${lock_file}" || return 1
  if flock -n "${lock_fd}"; then
    flock -u "${lock_fd}" || true
    exec {lock_fd}>&-
    return 1
  fi
  exec {lock_fd}>&-
}

verify_consumer_after_producer_stop() {
  local directory="$1" stack_pid stack_pgid module3_pid module3_pgid
  read_recorded_identity stack "${directory}" stack_pid stack_pgid \
    && read_recorded_identity module3_ros "${directory}" module3_pid module3_pgid \
    && recorded_identity_is_running "${stack_pid}" "${stack_pgid}" \
    && recorded_identity_is_running "${module3_pid}" "${module3_pgid}"
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
  validate_producer_stop_isolation "${producer_run_dir}" || exit 1
  fast_stop_registered_groups "${producer_run_dir}" || exit 1
  cleanup_exact_socket "${producer_run_dir}" "${producer_socket}" 2 0.02 || exit 1
  verify_consumer_after_producer_stop "${producer_run_dir}" || {
    echo "producer-stop terminated the Phase-F stack or Module3 ROS consumer" >&2
    exit 1
  }
  ros_lock_is_owned || {
    echo "producer-stop released the ROS/Nav2 runtime lock" >&2
    exit 1
  }
  if [[ -e "${producer_socket}" || -S "${producer_socket}" ]] \
      || socket_has_listener "${producer_socket}" || socket_connects "${producer_socket}"; then
    echo "Module2 socket remains after producer-stop: ${producer_socket}" >&2
    exit 1
  fi
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
# shellcheck source=lib/v6_dynamic_startup.sh
source "${script_dir}/lib/v6_dynamic_startup.sh"
configure_v6_dynamic_integration_overlay
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
  source_ros --require-integration-underlay
  validate_v6_dynamic_integration_overlay
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
  local name="$1" pid="$2" own_pgid anchor_pid="" pgid="" index candidate
  local previous_anchor="" previous_pgid="" stable_checks=0
  own_pgid="$(process_group_of_pid "$$")"
  for ((index=0; index<200; index++)); do
    candidate="$(independent_group_candidate "${pid}" "${own_pgid}" || true)"
    if read -r anchor_pid pgid <<<"${candidate}" \
        && [[ "${anchor_pid}" =~ ^[1-9][0-9]*$ \
        && "${pgid}" =~ ^[1-9][0-9]*$ ]]; then
      if [[ "${anchor_pid}" == "${previous_anchor}" \
          && "${pgid}" == "${previous_pgid}" ]]; then
        ((stable_checks+=1))
      else
        previous_anchor="${anchor_pid}"
        previous_pgid="${pgid}"
        stable_checks=1
      fi
      ((stable_checks >= 2)) && break
    else
      previous_anchor=""
      previous_pgid=""
      stable_checks=0
    fi
    sleep 0.01
  done
  if ((stable_checks < 2)) || [[ "${pgid}" == "${own_pgid}" ]]; then
    echo "could not identify a stable independent process group for ${name} pid=${pid}" >&2
    return 1
  fi
  child_names+=("${name}")
  child_pids+=("${pid}")
  child_pgids+=("${pgid}")
  write_process_identity "${name}" "${run_dir}" "${anchor_pid}" "${pgid}"
}

descendant_pids() {
  local root_pid="$1" current child
  local -a queue=("${root_pid}")
  local -A seen=(["${root_pid}"]=1)
  while ((${#queue[@]})); do
    current="${queue[0]}"
    queue=("${queue[@]:1}")
    while read -r child; do
      [[ "${child}" =~ ^[1-9][0-9]*$ && -z "${seen[${child}]:-}" ]] || continue
      seen["${child}"]=1
      queue+=("${child}")
      printf '%s\n' "${child}"
    done < <(ps -o pid= --ppid "${current}" 2>/dev/null || true)
  done
}

independent_group_candidate() {
  local root_pid="$1" own_pgid="$2" candidate pgid
  if process_is_running "${root_pid}"; then
    pgid="$(process_group_of_pid "${root_pid}")"
    if [[ "${pgid}" =~ ^[1-9][0-9]*$ && "${pgid}" != "${own_pgid}" ]]; then
      printf '%s %s\n' "${root_pid}" "${pgid}"
      return 0
    fi
  fi
  while read -r candidate; do
    process_is_running "${candidate}" || continue
    pgid="$(process_group_of_pid "${candidate}")"
    [[ "${pgid}" =~ ^[1-9][0-9]*$ && "${pgid}" != "${own_pgid}" ]] || continue
    printf '%s %s\n' "${candidate}" "${pgid}"
    return 0
  done < <(descendant_pids "${root_pid}")
  return 1
}

descendant_groups() {
  local root_pid="$1" child pgid
  while read -r child; do
    pgid="$(process_group_of_pid "${child}")"
    [[ "${pgid}" =~ ^[1-9][0-9]*$ ]] && printf '%s\n' "${pgid}"
  done < <(descendant_pids "${root_pid}")
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
  rm -f "${run_dir}/stack.identity" "${run_dir}/stack.pid" "${run_dir}/stack.pgid"
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

stack_pgid="$(process_group_of_pid "$$")"
[[ "${stack_pgid}" =~ ^[1-9][0-9]*$ ]] || {
  echo "could not identify Phase-F stack process group" >&2
  exit 1
}
write_process_identity stack "${run_dir}" "$$" "${stack_pgid}"

exit_if_terminating
setsid --wait -- "${script_dir}/run_v6_kujiale_low_obstacles.sh" ros "${arm}" \
  route_prior_enabled:=false \
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
