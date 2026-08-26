#!/usr/bin/env bash
# Minimal Phase D/E entry; reuse Phase B components and change only localization.
set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
phase_b_entry="${script_dir}/run_v6_r5_phase_b_kujiale.sh"
default_config="${script_dir}/../ros2_ws/src/robot_experiments/config/v6_localization_causal.yaml"

usage() {
  cat >&2 <<'USAGE'
usage: run_v6_localization_causal.sh [--run-root PATH] [--domain ID] [--arm S0|S1|R0|R1]
       config|plan|isaac|ros|module1|bridge|record|runner [arguments...]

Start one live component per terminal in this order:
  ros, isaac, module1, bridge, record, runner

S0: broad frozen runner initialpose + supervisor shadow
S1: no runner seed + supervisor startup
R0: F2 + global_localization
R1: F2 + supervisor active + one explicit manual rescue

Module2 navigation effect, CPG, low obstacles, and dynamic actors stay off.
Ground Truth is recorded only for the independent passive evaluator.
USAGE
}

run_root="${BIO_NAV_PHASE_DE_RUN_ROOT:-/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_phase_de_localization_current}"
domain_id="${BIO_NAV_PHASE_DE_DOMAIN_ID:-151}"
arm="${BIO_NAV_PHASE_DE_ARM:-}"
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
if [[ "${component}" != "config" && "${component}" != "plan" ]]; then
  [[ "${arm}" =~ ^(S0|S1|R0|R1)$ ]] || {
    echo "--arm S0|S1|R0|R1 is required for ${component}" >&2
    exit 2
  }
fi

export ROS_DOMAIN_ID="${domain_id}"
export ISAAC_NAV_EXPECTED_DOMAIN_ID="${domain_id}"
export BIO_NAV_PHASE_B_RUN_ROOT="${run_root}"
export BIO_NAV_PHASE_B_DOMAIN_ID="${domain_id}"

supervisor_mode="shadow"
case "${arm}" in
  S1) supervisor_mode="startup" ;;
  R1) supervisor_mode="active" ;;
esac

case "${component}" in
  config|plan)
    exec ros2 run robot_experiments v6_localization_causal \
      "${component}" --config "${BIO_NAV_V6_LOCALIZATION_CAUSAL_CONFIG:-$default_config}" "$@"
    ;;
  isaac)
    exec "${phase_b_entry}" --run-root "${run_root}" --domain "${domain_id}" isaac "$@"
    ;;
  ros)
    # Disable the Phase B automatic initial-pose publisher. The selected arm
    # must be the only startup seed authority.
    exec "${phase_b_entry}" --run-root "${run_root}" --domain "${domain_id}" \
      ros initial_pose_source:=rviz "$@"
    ;;
  module1)
    exec "${phase_b_entry}" --run-root "${run_root}" --domain "${domain_id}" \
      module1-shadow "$@"
    ;;
  bridge)
    # The Integration launch owns the mode contract.  estimated_shadow keeps
    # Module2/CPG navigation effects off while selecting only B5 behavior.
    exec "${phase_b_entry}" --run-root "${run_root}" --domain "${domain_id}" \
      bridge localization_supervisor_mode:="${supervisor_mode}" "$@"
    ;;
  record)
    source "${script_dir}/lib/common.sh"
    source_ros --require-integration-underlay
    mkdir -p "${run_root}/rosbag"
    bag_path="${run_root}/rosbag/phase_de_${arm,,}"
    [[ ! -e "${bag_path}" ]] || {
      echo "refusing to overwrite ${bag_path}" >&2
      exit 2
    }
    mapfile -t topics < <(
      python3 -m robot_experiments.phase_b_observability --print-recorder-topics
    )
    exec ros2 bag record --use-sim-time --storage mcap \
      --storage-preset-profile zstd_fast --output "${bag_path}" \
      "${topics[@]}" /initialpose /diagnostics \
      /bio_nav/localization/request_manual_rescue "$@"
    ;;
  runner)
    source "${script_dir}/lib/common.sh"
    source_ros --require-integration-underlay
    mkdir -p "${run_root}/episodes"
    output="${run_root}/episodes/phase_de_${arm,,}.jsonl"
    exec ros2 run robot_experiments v6_localization_causal run \
      --config "${BIO_NAV_V6_LOCALIZATION_CAUSAL_CONFIG:-$default_config}" \
      --arm "${arm}" --output-jsonl "${output}" "$@"
    ;;
  *) usage; exit 2 ;;
esac
