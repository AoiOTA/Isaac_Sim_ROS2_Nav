#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

original_args=("$@")

usage() {
  cat <<'EOF'
usage: run_rviz.sh mapping|incremental_mapping|localization|navigation [config]

Open the project RViz workflow for one operation. The optional config may be
an absolute or relative .rviz file; otherwise the mode-specific installed
configuration is selected automatically.
EOF
}

operation="${1:-}"
[[ -n "${operation}" ]] || {
  usage >&2
  exit 1
}
shift

case "${operation}" in
  mapping|incremental_mapping)
    config_name="mapping.rviz"
    ;;
  localization)
    config_name="localization.rviz"
    ;;
  navigation)
    config_name="navigation.rviz"
    ;;
  -h|--help)
    usage
    exit 0
    ;;
  *)
    usage >&2
    die "unsupported RViz operation: ${operation}"
    ;;
esac

(($# <= 1)) || die "run_rviz.sh accepts at most one custom config path"

source_ros --require-workspace
require_command ros2
require_command realpath
require_command rviz2
[[ -n "${ISAAC_NAV_SESSION_ID:-}" ]] || start_runtime_session
ensure_dedicated_process_group "${original_args[@]}"
acquire_instance_lock rviz "RViz"
trap 'release_instance_lock rviz || true' EXIT

if (($# == 1)); then
  rviz_config="$1"
  if [[ "${rviz_config}" != /* ]]; then
    rviz_config="$(realpath -m "${rviz_config}")"
  fi
else
  description_prefix="$(ros2 pkg prefix robot_description)"
  rviz_config="${description_prefix}/share/robot_description/rviz/${config_name}"
fi

require_file "${rviz_config}"
log_info "starting RViz workflow: operation=${operation}, config=${rviz_config}"
rviz_signal=""
rviz_group="$(current_process_group "$$")"
rviz_int_checks="${ISAAC_NAV_RVIZ_INT_CHECKS:-20}"
rviz_term_checks="${ISAAC_NAV_RVIZ_TERM_CHECKS:-20}"
rviz_kill_checks="${ISAAC_NAV_RVIZ_KILL_CHECKS:-20}"
for check_setting in rviz_int_checks rviz_term_checks rviz_kill_checks; do
  check_value="${!check_setting}"
  [[ "${check_value}" =~ ^[1-9][0-9]*$ ]] \
    || die "${check_setting} must be a positive integer"
done
[[ "${rviz_group}" == "$$" ]] \
  || die "RViz wrapper must lead its process group; pid=$$, pgid=${rviz_group}"

record_rviz_signal() {
  local signal_name="$1"
  [[ -n "${rviz_signal}" ]] || rviz_signal="${signal_name}"
}

cleanup_rviz_metadata() {
  if runtime_process_group_has_members_except "${rviz_group}" "$$"; then
    log_warn "retaining RViz metadata because process group ${rviz_group} still has live descendants"
    return 0
  fi
  release_instance_lock rviz || true
}

rviz_descendants_are_running() {
  runtime_process_group_has_members_except "${rviz_group}" "$$"
}

wait_for_rviz_descendants() {
  local attempts="$1"
  local index
  for ((index = 0; index < attempts; index++)); do
    rviz_descendants_are_running || return 0
    sleep 0.1
  done
  ! rviz_descendants_are_running
}

signal_rviz_descendants() {
  local signal_name="$1"
  local member
  rviz_descendants_are_running || return 0
  runtime_process_group_is_owned_by_session \
    "${rviz_group}" "${PROJECT_ROOT}" "${ISAAC_NAV_SESSION_ID}" || return 1
  while IFS= read -r member; do
    [[ -n "${member}" && "${member}" != "$$" ]] || continue
    runtime_process_is_running "${member}" || continue
    kill "-${signal_name}" "${member}" 2>/dev/null || true
  done < <(runtime_process_group_members "${rviz_group}" || true)
}

stop_rviz_descendants() {
  rviz_descendants_are_running || return 0
  log_warn "RViz primary process exited with live process-group descendants; sending SIGINT"
  signal_rviz_descendants INT || return 1
  wait_for_rviz_descendants "${rviz_int_checks}" && return 0

  log_warn "RViz descendants ignored SIGINT; sending SIGTERM"
  signal_rviz_descendants TERM || return 1
  wait_for_rviz_descendants "${rviz_term_checks}" && return 0

  log_warn "RViz descendants ignored SIGTERM; sending SIGKILL"
  signal_rviz_descendants KILL || return 1
  wait_for_rviz_descendants "${rviz_kill_checks}"
}

trap cleanup_rviz_metadata EXIT
trap 'record_rviz_signal INT' INT
trap 'record_rviz_signal TERM' TERM
trap 'record_rviz_signal HUP' HUP

set +e
(
  trap - INT TERM HUP
  close_instance_lock_fds_for_child
  exec rviz2 \
    -d "${rviz_config}" \
    --ros-args \
    -p use_sim_time:=true
)
rviz_status=$?
set -e

trap - INT TERM HUP
if stop_rviz_descendants; then
  release_instance_lock rviz
else
  log_warn "RViz process group ${rviz_group} did not stop; retaining authenticated metadata"
  rviz_status=1
fi
trap - EXIT

if [[ -n "${rviz_signal}" && "${rviz_status}" -eq 0 ]]; then
  case "${rviz_signal}" in
    HUP) rviz_status=129 ;;
    INT) rviz_status=130 ;;
    TERM) rviz_status=143 ;;
  esac
fi
exit "${rviz_status}"
