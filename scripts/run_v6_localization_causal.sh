#!/usr/bin/env bash
# Minimal Phase D/E entry; reuse Phase B components and change only localization.
set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
phase_b_entry="${script_dir}/run_v6_r5_phase_b_kujiale.sh"
default_config="${script_dir}/../ros2_ws/src/robot_experiments/config/v6_localization_causal.yaml"
whole_house_variant="whole_house_onebox_recovery"

usage() {
  cat >&2 <<'USAGE'
usage: run_v6_localization_causal.sh [--run-root PATH] [--domain ID]
       [--arm S0|S1|R0|R1|W0|W1] [--variant whole_house_onebox_recovery]
       [--dry-run]
       config|plan|isaac|ros|module1|bridge|record|runner [arguments...]

Start one live component per terminal in this order:
  ros, isaac, module1, bridge, record, runner

S0: broad frozen runner initialpose + supervisor shadow
S1: no runner seed + supervisor startup
R0: completed G2 + AMCL particle spread + supervisor shadow
R1: identical fault + supervisor active + one explicit manual rescue topic action
W0/W1: whole-house one-box aliases for internal R0/R1 respectively

All arms use the same Run4 candidate server/manifest. R0 uses its top-level
shadow permission; R1 uses the nested explicit-manual-purpose permission.

S0/S1/R0/R1 keep Module2 navigation effect, CPG, obstacles, and actors off.
W0/W1 enable only the frozen one-box M3 obstacle effect; CPG/actors stay off.
Ground Truth is recorded only for the independent passive evaluator.
USAGE
}

run_root="${BIO_NAV_PHASE_DE_RUN_ROOT:-/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_phase_de_localization_current}"
domain_id="${BIO_NAV_PHASE_DE_DOMAIN_ID:-151}"
arm="${BIO_NAV_PHASE_DE_ARM:-}"
variant="${BIO_NAV_PHASE_DE_VARIANT:-}"
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
    --variant)
      (($# >= 2)) || { usage; exit 2; }
      variant="$2"
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
  [[ "${arm}" =~ ^(S0|S1|R0|R1|W0|W1)$ ]] || {
    echo "--arm S0|S1|R0|R1|W0|W1 is required for ${component}" >&2
    exit 2
  }
fi
user_arm="${arm}"
case "${arm}" in
  W0) arm="R0"; variant="${whole_house_variant}" ;;
  W1) arm="R1"; variant="${whole_house_variant}" ;;
esac
[[ -z "${variant}" || "${variant}" == "${whole_house_variant}" ]] || {
  echo "--variant must be ${whole_house_variant}" >&2
  exit 2
}
whole_house=false
[[ "${variant}" == "${whole_house_variant}" ]] && whole_house=true
if [[ "${whole_house}" == true && "${component}" != "config" && "${component}" != "plan" \
    && ! "${arm}" =~ ^R[01]$ ]]; then
  echo "${whole_house_variant} supports only R0/R1 or W0/W1" >&2
  exit 2
fi

export ROS_DOMAIN_ID="${domain_id}"
export ISAAC_NAV_EXPECTED_DOMAIN_ID="${domain_id}"
export BIO_NAV_PHASE_B_RUN_ROOT="${run_root}"
export BIO_NAV_PHASE_B_DOMAIN_ID="${domain_id}"

integration_root="${BIO_NAV_INTEGRATION_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_integration}"
module2_root="${BIO_NAV_MODULE2_V310_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_module2}"
candidate_manifest="${BIO_NAV_PHASE_DE_RUN4_CANDIDATE_MANIFEST:-${integration_root}/ros2_ws/src/bio_nav_ros_bridge/config/kujiale_0026_run4_read_only_shadow_candidate.json}"
server_entry="${integration_root}/scripts/run_module2_v310_server.sh"
obstacle_server_entry="${integration_root}/scripts/run_v6_module2_causal_obstacle_server.sh"
low_obstacle_entry="${script_dir}/run_v6_kujiale_low_obstacles.sh"
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
if allowed != ["shadow", "startup"]:
    raise SystemExit("Run4 candidate top-level modes changed")
manual = payload.get("manual_recovery_experiment")
expected_manual = {
    "status": "ENGINEERING_EXPLICIT_MANUAL_RECOVERY_ONLY",
    "allowed_supervisor_modes": ["active"],
    "requires_explicit_request": True,
    "auto_rescue_enabled": False,
}
if manual != expected_manual:
    raise SystemExit("Run4 candidate manual recovery experiment block changed")
if mode not in allowed and mode not in manual["allowed_supervisor_modes"]:
    raise SystemExit(f"Run4 candidate does not allow supervisor mode {mode}")
PY
}

supervisor_mode="shadow"
case "${arm}" in
  S1) supervisor_mode="startup" ;;
  R1) supervisor_mode="active" ;;
esac
variant_args=()
if [[ "${whole_house}" == true ]]; then
  variant_args=(--variant "${whole_house_variant}")
fi

case "${component}" in
  config|plan)
    run_command ros2 run robot_experiments v6_localization_causal \
      "${component}" --config "${BIO_NAV_V6_LOCALIZATION_CAUSAL_CONFIG:-$default_config}" \
      "${variant_args[@]}" "$@"
    ;;
  isaac)
    if [[ "${whole_house}" == true ]]; then
      run_command "${low_obstacle_entry}" isaac "$@"
    else
      run_command "${phase_b_entry}" --run-root "${run_root}" --domain "${domain_id}" isaac "$@"
    fi
    ;;
  ros)
    # Disable the Phase B automatic initial-pose publisher. The selected arm
    # must be the only startup seed authority. All four arms start ROS before
    # their runner/supervisor seed exists, so keep the activation gate alive
    # until that later seed arrives instead of failing at the startup timeout.
    if [[ "${whole_house}" == true ]]; then
      run_command "${low_obstacle_entry}" ros M3 \
        route_prior_enabled:=false initial_pose_source:=rviz \
        activation_startup_policy:=wait_for_seed "$@"
    else
      run_command "${phase_b_entry}" --run-root "${run_root}" --domain "${domain_id}" \
        ros initial_pose_source:=rviz \
        activation_startup_policy:=wait_for_seed "$@"
    fi
    ;;
  module1)
    require_run4_candidate_manifest "${supervisor_mode}"
    if [[ "${dry_run}" == false ]]; then
      selected_server="${server_entry}"
      [[ "${whole_house}" == false ]] || selected_server="${obstacle_server_entry}"
      [[ -x "${selected_server}" ]] || {
        echo "Run4 candidate server is missing: ${selected_server}" >&2
        exit 2
      }
      mkdir -p -m 700 "$(dirname -- "${socket_path}")"
    fi
    if [[ "${whole_house}" == true ]]; then
      run_command "${obstacle_server_entry}" \
        --startup-profile module2_causal_obstacle_active \
        --active-effect-scope obstacle_only \
        --module2-root "${module2_root}" \
        --candidate-manifest "${candidate_manifest}" \
        --socket "${socket_path}" \
        --device "${BIO_NAV_PHASE_DE_DEVICE:-cuda}" "$@"
    else
      run_command "${server_entry}" \
        --module2-root "${module2_root}" \
        --candidate-manifest "${candidate_manifest}" \
        --socket "${socket_path}" \
        --device "${BIO_NAV_PHASE_DE_DEVICE:-cuda}" "$@"
    fi
    ;;
  bridge)
    # The Integration launch owns the mode contract.  estimated_shadow keeps
    # Module2/CPG navigation effects off while selecting only B5 behavior.
    require_run4_candidate_manifest "${supervisor_mode}"
    if [[ "${whole_house}" == true ]]; then
      run_command "${phase_b_entry}" --run-root "${run_root}" --domain "${domain_id}" \
        bridge startup_profile:=module2_causal_obstacle_active \
        localization_supervisor_mode:="${supervisor_mode}" \
        "localization_candidate_manifest:=${candidate_manifest}" "$@"
    else
      run_command "${phase_b_entry}" --run-root "${run_root}" --domain "${domain_id}" \
        bridge localization_supervisor_mode:="${supervisor_mode}" \
        "localization_candidate_manifest:=${candidate_manifest}" "$@"
    fi
    ;;
  record)
    source "${script_dir}/lib/common.sh"
    source_ros --require-integration-underlay
    mkdir -p "${run_root}/rosbag"
    bag_path="${run_root}/rosbag/phase_de_${user_arm,,}"
    [[ ! -e "${bag_path}" ]] || {
      echo "refusing to overwrite ${bag_path}" >&2
      exit 2
    }
    mapfile -t topics < <(
      python3 -m robot_experiments.phase_b_observability --print-recorder-topics
    )
    run_command ros2 bag record --use-sim-time --storage mcap \
      --storage-preset-profile zstd_fast --output "${bag_path}" \
      "${topics[@]}" /initialpose /particle_cloud /diagnostics \
      /bio_nav/localization/request_manual_rescue "$@"
    ;;
  runner)
    source "${script_dir}/lib/common.sh"
    source_ros --require-integration-underlay
    mkdir -p "${run_root}/episodes"
    output="${run_root}/episodes/phase_de_${user_arm,,}.jsonl"
    run_command ros2 run robot_experiments v6_localization_causal run \
      --config "${BIO_NAV_V6_LOCALIZATION_CAUSAL_CONFIG:-$default_config}" \
      "${variant_args[@]}" --arm "${arm}" --output-jsonl "${output}" \
      "$@"
    ;;
  *) usage; exit 2 ;;
esac
