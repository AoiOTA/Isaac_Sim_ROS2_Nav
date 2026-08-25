#!/usr/bin/env bash
# One long-running stack adapter for a single Phase-F M0--M3 episode.
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  echo "usage: $0 M0|M1|M2|M3 --domain ID --run-dir PATH --socket PATH [--module2-root PATH]" >&2
}

arm="${1:-}"
[[ "${arm}" =~ ^M[0-3]$ ]] || { usage; exit 2; }
shift
run_dir=""
socket_path=""
domain_id="${BIO_NAV_PHASE_F_DOMAIN_ID:-150}"
module2_root="${BIO_NAV_MODULE2_V310_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_module2}"
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
if [[ "${arm}" != "M0" ]]; then
  require_directory "${module2_root}"
  require_file "${module2_root}/configs/kujiale_0026_module1_visual_shadow_v310.yaml"
  require_file "${integration_root}/scripts/run_module2_v310_server.sh"
  require_file "${integration_root}/scripts/run_v6_module2_causal_obstacle_server.sh"
fi
mkdir -p "${run_dir}" "$(dirname "${socket_path}")"
rm -f "${socket_path}"

declare -a child_pids=()
shutdown() {
  local index pid
  trap - EXIT INT TERM
  for ((index=${#child_pids[@]}-1; index>=0; --index)); do
    pid="${child_pids[index]}"
    kill -INT "${pid}" 2>/dev/null || true
  done
  for pid in "${child_pids[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
  rm -f "${socket_path}"
}
trap shutdown EXIT INT TERM

"${script_dir}/run_v6_kujiale_low_obstacles.sh" ros "${arm}" \
  >"${run_dir}/module3_ros.log" 2>&1 &
child_pids+=("$!")

if [[ "${arm}" != "M0" ]]; then
  if [[ "${arm}" == "M1" ]]; then
    "${integration_root}/scripts/run_module2_v310_server.sh" \
      --module2-root "${module2_root}" \
      --shadow-config configs/kujiale_0026_module1_visual_shadow_v310.yaml \
      --socket "${socket_path}" \
      >"${run_dir}/module2_server.log" 2>&1 &
  else
    "${integration_root}/scripts/run_v6_module2_causal_obstacle_server.sh" \
      --startup-profile module2_causal_obstacle_active \
      --active-effect-scope obstacle_only \
      --socket "${socket_path}" \
      --module2-root "${module2_root}" \
      --shadow-config configs/kujiale_0026_module1_visual_shadow_v310.yaml \
      >"${run_dir}/module2_server.log" 2>&1 &
  fi
  child_pids+=("$!")

  source_ros --require-integration-underlay
  startup_profile="estimated_shadow"
  [[ "${arm}" =~ ^M[23]$ ]] && startup_profile="module2_causal_obstacle_active"
  ros2 launch bio_nav_ros_bridge v6_cognitive_navigation.launch.py \
    startup_profile:="${startup_profile}" \
    socket_path:="${socket_path}" \
    use_sim_time:=true \
    >"${run_dir}/integration_bridge.log" 2>&1 &
  child_pids+=("$!")
fi

set +e
wait -n "${child_pids[@]}"
status=$?
set -e
exit "${status}"
