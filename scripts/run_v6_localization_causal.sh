#!/usr/bin/env bash
# Minimal Phase D/E entry; reuse Phase B components and change only localization.
set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
phase_b_entry="${script_dir}/run_v6_r5_phase_b_kujiale.sh"
default_config="${script_dir}/../ros2_ws/src/robot_experiments/config/v6_localization_causal.yaml"

usage() {
  cat >&2 <<'USAGE'
usage: run_v6_localization_causal.sh [--run-root PATH] [--domain ID] [--arm S0|S1|R0|R1]
       [--dry-run]
       config|plan|isaac|ros|module1|bridge|record|runner [arguments...]

Start one live component per terminal in this order:
  ros, isaac, module1, bridge, record, runner

S0: broad frozen runner initialpose + supervisor shadow
S1: no runner seed + supervisor startup
R0: F2 + global_localization
R1: F2 + supervisor active + one explicit manual rescue

S0/S1 use the same Run4 startup-only candidate. R0/R1 keep the Phase B
Module1 path and never receive the Run4 candidate manifest.

Module2 navigation effect, CPG, low obstacles, and dynamic actors stay off.
Ground Truth is recorded only for the independent passive evaluator.
USAGE
}

run_root="${BIO_NAV_PHASE_DE_RUN_ROOT:-/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_phase_de_localization_current}"
domain_id="${BIO_NAV_PHASE_DE_DOMAIN_ID:-151}"
arm="${BIO_NAV_PHASE_DE_ARM:-}"
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

integration_root="${BIO_NAV_INTEGRATION_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_integration}"
module2_root="${BIO_NAV_MODULE2_V310_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_module2}"
candidate_manifest="${BIO_NAV_PHASE_DE_RUN4_CANDIDATE_MANIFEST:-${integration_root}/ros2_ws/src/bio_nav_ros_bridge/config/kujiale_0026_run4_read_only_shadow_candidate.json}"
server_entry="${integration_root}/scripts/run_module2_v310_server.sh"
if [[ -n "${BIO_NAV_PHASE_DE_SOCKET_PATH:-}" ]]; then
  socket_path="${BIO_NAV_PHASE_DE_SOCKET_PATH}"
else
  run_id="$(basename "${run_root%/}")"
  run_id="${run_id//[^A-Za-z0-9_.-]/_}"
  socket_path="${XDG_RUNTIME_DIR:-/tmp}/bio_nav_phase_de_${UID}/domain_${domain_id}/${run_id}/module2.sock"
fi
export BIO_NAV_PHASE_B_SOCKET_PATH="${socket_path}"

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

require_run4_candidate_manifest() {
  local mode="$1"
  python3 - "${candidate_manifest}" "${mode}" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
mode = sys.argv[2]
try:
    payload = json.loads(path.read_text(encoding="utf-8"))
except (OSError, json.JSONDecodeError) as exc:
    raise SystemExit(f"Run4 candidate manifest is unreadable: {exc}")
if payload.get("status") != "READ_ONLY_CAUSAL_CANDIDATE_STARTUP_ONLY":
    raise SystemExit("Run4 candidate manifest is not startup-allowed")
if payload.get("recovery_qualification") != "NOT_ACTIVE_RECOVERY_QUALIFIED":
    raise SystemExit("Run4 candidate manifest must remain non-recovery")
if payload.get("default_enabled") is not False:
    raise SystemExit("Run4 candidate manifest must remain default-disabled")
allowed = payload.get("allowed_supervisor_modes")
if allowed != ["shadow", "startup"] or mode not in allowed:
    raise SystemExit(f"Run4 candidate does not allow supervisor mode {mode}")
PY
}

supervisor_mode="shadow"
case "${arm}" in
  S1) supervisor_mode="startup" ;;
  R1) supervisor_mode="active" ;;
esac

case "${component}" in
  config|plan)
    run_command ros2 run robot_experiments v6_localization_causal \
      "${component}" --config "${BIO_NAV_V6_LOCALIZATION_CAUSAL_CONFIG:-$default_config}" "$@"
    ;;
  isaac)
    run_command "${phase_b_entry}" --run-root "${run_root}" --domain "${domain_id}" isaac "$@"
    ;;
  ros)
    # Disable the Phase B automatic initial-pose publisher. The selected arm
    # must be the only startup seed authority.  Phase D starts ROS before its
    # runner/supervisor seed exists, so keep the activation gate alive until
    # that later seed arrives instead of failing at the startup timeout.
    if [[ "${arm}" =~ ^S[01]$ ]]; then
      run_command "${phase_b_entry}" --run-root "${run_root}" --domain "${domain_id}" \
        ros initial_pose_source:=rviz \
        activation_startup_policy:=wait_for_seed "$@"
    else
      run_command "${phase_b_entry}" --run-root "${run_root}" --domain "${domain_id}" \
        ros initial_pose_source:=rviz "$@"
    fi
    ;;
  module1)
    if [[ "${arm}" =~ ^S[01]$ ]]; then
      require_run4_candidate_manifest "${supervisor_mode}"
      if [[ "${dry_run}" == false ]]; then
        [[ -x "${server_entry}" ]] || {
          echo "Run4 candidate server is missing: ${server_entry}" >&2
          exit 2
        }
        mkdir -p -m 700 "$(dirname -- "${socket_path}")"
      fi
      run_command "${server_entry}" \
        --module2-root "${module2_root}" \
        --candidate-manifest "${candidate_manifest}" \
        --socket "${socket_path}" \
        --device "${BIO_NAV_PHASE_DE_DEVICE:-cuda}" "$@"
    else
      run_command "${phase_b_entry}" --run-root "${run_root}" --domain "${domain_id}" \
        module1-shadow "$@"
    fi
    ;;
  bridge)
    # The Integration launch owns the mode contract.  estimated_shadow keeps
    # Module2/CPG navigation effects off while selecting only B5 behavior.
    if [[ "${arm}" =~ ^S[01]$ ]]; then
      require_run4_candidate_manifest "${supervisor_mode}"
      run_command "${phase_b_entry}" --run-root "${run_root}" --domain "${domain_id}" \
        bridge localization_supervisor_mode:="${supervisor_mode}" \
        "localization_candidate_manifest:=${candidate_manifest}" "$@"
    else
      run_command "${phase_b_entry}" --run-root "${run_root}" --domain "${domain_id}" \
        bridge localization_supervisor_mode:="${supervisor_mode}" "$@"
    fi
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
    run_command ros2 bag record --use-sim-time --storage mcap \
      --storage-preset-profile zstd_fast --output "${bag_path}" \
      "${topics[@]}" /initialpose /diagnostics \
      /bio_nav/localization/request_manual_rescue "$@"
    ;;
  runner)
    source "${script_dir}/lib/common.sh"
    source_ros --require-integration-underlay
    mkdir -p "${run_root}/episodes"
    output="${run_root}/episodes/phase_de_${arm,,}.jsonl"
    run_command ros2 run robot_experiments v6_localization_causal run \
      --config "${BIO_NAV_V6_LOCALIZATION_CAUSAL_CONFIG:-$default_config}" \
      --arm "${arm}" --output-jsonl "${output}" "$@"
    ;;
  *) usage; exit 2 ;;
esac
