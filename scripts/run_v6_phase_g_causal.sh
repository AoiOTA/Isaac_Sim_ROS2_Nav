#!/usr/bin/env bash
# Phase-G G0--G3 component wrapper: one reset, two warmups, one scoring loop.
set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
default_config="${project_root}/ros2_ws/src/robot_experiments/config/v6_phase_g_causal.yaml"
stack_entry="${script_dir}/run_v6_cognitive_graph_causal_stack.sh"
isaac_entry="${script_dir}/run_v6_kujiale_low_obstacles.sh"
qos_config="${project_root}/ros2_ws/src/robot_experiments/config/v6_low_obstacle_phase_f_rosbag_qos.yaml"

usage() {
  cat >&2 <<'USAGE'
usage: run_v6_phase_g_causal.sh [--run-root PATH] [--domain ID]
       [--arm G0|G1|G2|G3] [--obstacle-arm M3|M2] [--socket PATH] [--dry-run]
       config|plan|isaac|stack|record|runner|evaluate [arguments...]

Per-arm live terminals: isaac, stack, record, runner.
The stack owns Module3 ROS, Module2 and Integration.  The runner performs one
reset, two full-house warmup loops, then one full-house scoring loop without a
reset between loops.  M3 is the default obstacle arm; M2 is a whole-group
fallback and must never be mixed within one G0--G3 comparison.
USAGE
}

run_root="${BIO_NAV_PHASE_G_RUN_ROOT:-/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_phase_g_causal_current}"
domain_id="${BIO_NAV_PHASE_G_DOMAIN_ID:-151}"
arm="${BIO_NAV_PHASE_G_ARM:-}"
obstacle_arm="${BIO_NAV_PHASE_G_OBSTACLE_ARM:-M3}"
socket_path="${BIO_NAV_PHASE_G_SOCKET_PATH:-}"
dry_run=false
while (($# > 0)); do
  case "$1" in
    --run-root)
      (($# >= 2)) || { usage; exit 2; }
      run_root="$2"
      shift 2
      ;;
    --domain)
      (($# >= 2)) || { usage; exit 2; }
      domain_id="$2"
      shift 2
      ;;
    --arm)
      (($# >= 2)) || { usage; exit 2; }
      arm="$2"
      shift 2
      ;;
    --obstacle-arm)
      (($# >= 2)) || { usage; exit 2; }
      obstacle_arm="$2"
      shift 2
      ;;
    --socket)
      (($# >= 2)) || { usage; exit 2; }
      socket_path="$2"
      shift 2
      ;;
    --dry-run)
      dry_run=true
      shift
      ;;
    *) break ;;
  esac
done

component="${1:-}"
[[ -n "${component}" ]] || { usage; exit 2; }
shift
[[ "${domain_id}" =~ ^[0-9]+$ && "${domain_id}" -le 232 ]] || {
  echo "domain must be an integer in [0,232]" >&2
  exit 2
}
[[ "${obstacle_arm}" =~ ^(M3|M2)$ ]] || {
  echo "--obstacle-arm must be M3 or M2" >&2
  exit 2
}
if [[ "${component}" =~ ^(plan|isaac|stack|record|runner)$ ]]; then
  [[ "${arm}" =~ ^(G0|G1|G2|G3)$ ]] || {
    echo "--arm G0|G1|G2|G3 is required for ${component}" >&2
    exit 2
  }
fi

if [[ -z "${socket_path}" ]]; then
  run_id="$(basename "${run_root%/}")"
  run_id="${run_id//[^A-Za-z0-9_.-]/_}"
  socket_path="${XDG_RUNTIME_DIR:-/tmp}/bio_nav_phase_g_${UID}/domain_${domain_id}/${run_id}_${arm:-group}.sock"
fi

export ROS_DOMAIN_ID="${domain_id}"
export ISAAC_NAV_EXPECTED_DOMAIN_ID="${domain_id}"

print_command() {
  printf '%q ' "$@"
  printf '\n'
}

run_command() {
  if [[ "${dry_run}" == true ]]; then
    print_command "$@"
  else
    exec "$@"
  fi
}

source_phase_g_ros() {
  # shellcheck source=scripts/lib/common.sh
  source "${script_dir}/lib/common.sh"
  source_ros --require-integration-underlay
}

case "${component}" in
  config)
    source_phase_g_ros
    run_command ros2 run robot_experiments v6_phase_g_causal \
      config --config "${BIO_NAV_PHASE_G_CONFIG:-$default_config}" "$@"
    ;;
  plan)
    source_phase_g_ros
    run_command ros2 run robot_experiments v6_phase_g_causal \
      plan --config "${BIO_NAV_PHASE_G_CONFIG:-$default_config}" \
      --arm "${arm}" "$@"
    ;;
  isaac)
    run_command env BIO_NAV_PHASE_B_DOMAIN_ID="${domain_id}" \
      "${isaac_entry}" isaac "$@"
    ;;
  stack)
    [[ -x "${stack_entry}" || "${dry_run}" == true ]] || {
      echo "Phase-G stack entry is missing: ${stack_entry}" >&2
      exit 2
    }
    mkdir -p "${run_root}/stack/${arm,,}"
    run_command "${stack_entry}" --arm "${arm}" --domain "${domain_id}" \
      --run-root "${run_root}/stack/${arm,,}" --socket "${socket_path}" \
      --obstacle-arm "${obstacle_arm}" "$@"
    ;;
  record)
    source_phase_g_ros
    mkdir -p "${run_root}/rosbag"
    bag_path="${run_root}/rosbag/phase_g_${arm,,}"
    [[ ! -e "${bag_path}" ]] || {
      echo "refusing to overwrite ${bag_path}" >&2
      exit 2
    }
    mapfile -t topics < <(
      python3 -m robot_experiments.phase_b_observability --print-recorder-topics
    )
    extras=(
      /bio_nav/module2/cognitive_place_graph
      /bio_nav/module3/cognitive_graph_validation_ack
      /bio_nav/structural_graph_status
      /bio_nav/module2/srdr_edge_diagnostics
      /bio_nav/route_edge_costs
      /bio_nav/runtime_edge_states
      /bio_nav/cognitive_obstacle_layer/status
      /bio_nav/local_risk_layer/status
      /bio_nav/cognitive_risk_critic/status
      /plan
      /optimal_trajectory
      /experiment/obstacles/state
    )
    run_command ros2 bag record --use-sim-time --storage mcap \
      --storage-preset-profile zstd_fast \
      --qos-profile-overrides-path "${qos_config}" \
      --output "${bag_path}" "${topics[@]}" "${extras[@]}" "$@"
    ;;
  runner)
    source_phase_g_ros
    mkdir -p "${run_root}/episodes"
    output="${run_root}/episodes/phase_g_${arm,,}.jsonl"
    [[ ! -e "${output}" ]] || {
      echo "refusing to overwrite ${output}" >&2
      exit 2
    }
    run_command ros2 run robot_experiments v6_phase_g_causal \
      run --config "${BIO_NAV_PHASE_G_CONFIG:-$default_config}" \
      --arm "${arm}" --obstacle-arm "${obstacle_arm}" \
      --output-jsonl "${output}" "$@"
    ;;
  evaluate)
    source_phase_g_ros
    run_command ros2 run robot_experiments v6_phase_g_causal \
      evaluate --config "${BIO_NAV_PHASE_G_CONFIG:-$default_config}" "$@"
    ;;
  *) usage; exit 2 ;;
esac
