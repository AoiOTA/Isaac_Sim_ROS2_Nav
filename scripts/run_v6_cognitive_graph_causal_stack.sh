#!/usr/bin/env bash
# Phase-G graph-causal stack: one Module3 ROS consumer plus the matching
# Module2 server and Integration bridge. Isaac, recorder, and episode dispatch
# remain owned by the campaign runner.
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat >&2 <<'EOF'
usage: run_v6_cognitive_graph_causal_stack.sh --arm G0|G1|G2|G3 \
  --domain ID --run-dir PATH --socket PATH [--obstacle-arm M3|M2] \
  [--module2-root PATH] [--dry-run]

G0: GVG + route prior off       G1: graph shadow + route prior off
G2: graph hybrid + prior on     G3: graph primary + prior on
The local-obstacle arm defaults to M3; M2 is the whole-group rollback.
EOF
}

arm=""
domain_id="${BIO_NAV_PHASE_G_DOMAIN_ID:-150}"
run_dir=""
socket_path=""
obstacle_arm="M3"
module2_root="${BIO_NAV_MODULE2_V310_ROOT:-}"
integration_root="${BIO_NAV_INTEGRATION_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_integration}"
dry_run=false

while (($#)); do
  case "$1" in
    --arm) arm="${2:?--arm requires G0, G1, G2, or G3}"; shift 2 ;;
    --domain) domain_id="${2:?--domain requires an ID}"; shift 2 ;;
    --run-dir|--run-root) run_dir="${2:?$1 requires a path}"; shift 2 ;;
    --socket) socket_path="${2:?--socket requires a path}"; shift 2 ;;
    --obstacle-arm) obstacle_arm="${2:?--obstacle-arm requires M3 or M2}"; shift 2 ;;
    --module2-root) module2_root="${2:?--module2-root requires a path}"; shift 2 ;;
    --dry-run) dry_run=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) usage; echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

[[ "${arm}" =~ ^G[0-3]$ ]] || { usage; exit 2; }
[[ "${obstacle_arm}" =~ ^M[23]$ ]] || {
  echo "obstacle-arm must be M3 or M2" >&2
  exit 2
}
[[ "${domain_id}" =~ ^[0-9]+$ && "${domain_id}" -le 232 ]] || {
  echo "domain must be an integer in [0,232]" >&2
  exit 2
}
[[ -n "${run_dir}" && -n "${socket_path}" ]] || { usage; exit 2; }
[[ "${run_dir}" == /* && "${socket_path}" == /* ]] || {
  echo "run-dir and socket must be absolute" >&2
  exit 2
}

case "${arm}" in
  G0)
    graph_mode="gvg"
    route_prior_enabled="false"
    startup_profile="module2_causal_obstacle_active"
    active_effect_scope="obstacle_only"
    ;;
  G1)
    graph_mode="shadow"
    route_prior_enabled="false"
    startup_profile="cognitive_graph_causal_shadow"
    active_effect_scope="obstacle_and_graph"
    ;;
  G2)
    graph_mode="hybrid"
    route_prior_enabled="true"
    startup_profile="cognitive_graph_causal_hybrid"
    active_effect_scope="all"
    ;;
  G3)
    graph_mode="primary"
    route_prior_enabled="true"
    startup_profile="cognitive_graph_causal_primary"
    active_effect_scope="all"
    ;;
esac

module3_command=(
  "${script_dir}/run_v6_kujiale_low_obstacles.sh"
)
if [[ "${graph_mode}" == "gvg" ]]; then
  module3_command+=(ros "${obstacle_arm}")
else
  module3_command+=(ros-d "${graph_mode}" "${obstacle_arm}")
fi
module3_command+=("route_prior_enabled:=${route_prior_enabled}")

module2_command=(
  "${integration_root}/scripts/run_v6_module2_graph_causal_server.sh"
  --startup-profile "${startup_profile}"
  --active-effect-scope "${active_effect_scope}"
  --cognitive-graph-mode "${graph_mode}"
)
module2_command+=(
  --socket "${socket_path}"
  --module2-root "${module2_root}"
)
if [[ "${arm}" =~ ^G[01]$ ]]; then
  module2_command+=(
    --shadow-config configs/kujiale_0026_module1_visual_shadow_v310.yaml
  )
fi

bridge_command=(
  ros2 launch bio_nav_ros_bridge v6_cognitive_navigation.launch.py
  "startup_profile:=${startup_profile}"
  "cognitive_graph_mode:=${graph_mode}"
  "route_prior_enabled:=${route_prior_enabled}"
  "socket_path:=${socket_path}"
  use_sim_time:=true
)

if [[ "${dry_run}" == true ]]; then
  printf 'arm=%s\n' "${arm}"
  printf 'graph_mode=%s\n' "${graph_mode}"
  printf 'route_prior_enabled=%s\n' "${route_prior_enabled}"
  printf 'obstacle_arm=%s\n' "${obstacle_arm}"
  printf 'startup_profile=%s\n' "${startup_profile}"
  printf 'active_effect_scope=%s\n' "${active_effect_scope}"
  printf 'module3:'; printf ' %q' "${module3_command[@]}"; printf '\n'
  printf 'module2:'; printf ' %q' "${module2_command[@]}"; printf '\n'
  printf 'bridge:'; printf ' %q' "${bridge_command[@]}"; printf '\n'
  exit 0
fi

# shellcheck source=lib/common.sh
source "${script_dir}/lib/common.sh"
require_directory "${integration_root}"
require_file "${integration_root}/scripts/run_v6_module2_graph_causal_server.sh"
[[ -n "${module2_root}" ]] || {
  echo "BIO_NAV_MODULE2_V310_ROOT or --module2-root is required" >&2
  exit 2
}
require_directory "${module2_root}"
module2_root="$(cd "${module2_root}" && pwd -P)"
require_file "${module2_root}/configs/kujiale_0026_module1_visual_shadow_v310.yaml"
export BIO_NAV_MODULE2_V310_ROOT="${module2_root}"
export ISAAC_NAV_EXPECTED_DOMAIN_ID="${domain_id}"
export ROS_DOMAIN_ID="${domain_id}"

mkdir -p "${run_dir}" "$(dirname "${socket_path}")"
if [[ -S "${socket_path}" ]] && grep -Fq "${socket_path}" /proc/net/unix; then
  echo "refusing to replace active Module2 socket: ${socket_path}" >&2
  exit 1
fi
rm -f -- "${socket_path}"

declare -a child_names=()
declare -a child_pids=()
declare -a child_pgids=()
terminating=false
termination_status=0

process_group_of_pid() {
  ps -o pgid= -p "$1" 2>/dev/null | tr -d '[:space:]'
}

group_is_running() {
  local pgid="$1"
  ps -eo pgid=,stat= | awk -v group="${pgid}" '
    $1 == group && $2 !~ /^Z/ { found = 1 }
    END { exit !found }
  '
}

register_child() {
  local name="$1" pid="$2" pgid="" own_pgid attempt
  for ((attempt=0; attempt<100; attempt++)); do
    pgid="$(process_group_of_pid "${pid}")"
    [[ "${pgid}" =~ ^[1-9][0-9]*$ ]] && break
    sleep 0.01
  done
  [[ "${pgid}" =~ ^[1-9][0-9]*$ ]] || {
    echo "could not identify process group for ${name}" >&2
    return 1
  }
  own_pgid="$(process_group_of_pid "$$")"
  [[ "${pgid}" != "${own_pgid}" ]] || {
    echo "${name} did not enter an independent process group" >&2
    return 1
  }
  child_names+=("${name}")
  child_pids+=("${pid}")
  child_pgids+=("${pgid}")
  printf '%s %s\n' "${pid}" "${pgid}" >"${run_dir}/${name}.identity"
}

start_child() {
  local name="$1" log_file="$2"
  shift 2
  setsid --wait -- "$@" >"${log_file}" 2>&1 &
  register_child "${name}" "$!"
}

request_shutdown() {
  terminating=true
  termination_status="$1"
}

shutdown() {
  local original_status="$1" signal_name index pgid attempt
  trap - EXIT INT TERM HUP
  for signal_name in INT TERM KILL; do
    for index in "${!child_pgids[@]}"; do
      pgid="${child_pgids[index]}"
      group_is_running "${pgid}" || continue
      kill "-${signal_name}" -- "-${pgid}" 2>/dev/null || true
    done
    for ((attempt=0; attempt<100; attempt++)); do
      local any_running=false
      for pgid in "${child_pgids[@]}"; do
        group_is_running "${pgid}" && any_running=true
      done
      [[ "${any_running}" == false ]] && break
      sleep 0.05
    done
    [[ "${any_running:-false}" == false ]] && break
  done
  for index in "${!child_pids[@]}"; do
    wait "${child_pids[index]}" 2>/dev/null || true
    rm -f "${run_dir}/${child_names[index]}.identity"
  done
  if [[ -S "${socket_path}" ]] && ! grep -Fq "${socket_path}" /proc/net/unix; then
    rm -f -- "${socket_path}"
  fi
  exit "${original_status}"
}

trap 'request_shutdown 130' INT
trap 'request_shutdown 143' TERM
trap 'request_shutdown 129' HUP
trap 'shutdown $?' EXIT

start_child module3_ros "${run_dir}/module3_ros.log" "${module3_command[@]}"
start_child module2_server "${run_dir}/module2_server.log" "${module2_command[@]}"
source_ros --require-integration-underlay
start_child integration_bridge "${run_dir}/integration_bridge.log" \
  "${bridge_command[@]}"

set +e
wait "${child_pids[0]}"
status=$?
set -e
if [[ "${terminating}" == true ]]; then
  exit "${termination_status}"
fi
exit "${status}"
