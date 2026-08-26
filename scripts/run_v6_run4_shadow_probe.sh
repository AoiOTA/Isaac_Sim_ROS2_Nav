#!/usr/bin/env bash
# Bounded Run4 read-only localization probe.  This wrapper intentionally has
# no ROS/Isaac stack or navigation-runner entrypoint.
set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
default_probe_config="${project_root}/ros2_ws/src/robot_experiments/config/v6_run4_shadow_probe.yaml"

usage() {
  cat <<'USAGE'
usage: run_v6_run4_shadow_probe.sh [options] plan|server|bridge|record [arguments...]

Options:
  --run-root PATH          probe output root
  --domain ID              ROS domain in [0,232]
  --duration SEC           bounded recorder duration (default: 20)
  --integration-root PATH  Integration checkout containing the Run4 manifest
  --module2-root PATH      Module2 checkout used by the candidate server
  --dry-run                print the exact command without executing it

The probe is stationary/read-only: it exposes only the Run4 server, the
estimated_shadow Bridge with localization supervisor=shadow, and a bounded
recorder.  It cannot start Isaac, the Module3 ROS/Nav2 stack, or a goal runner.
Expected /initialpose writes: zero.  T2 status remains FAIL/NOT_QUALIFIED.
USAGE
}

run_root="${BIO_NAV_RUN4_SHADOW_RUN_ROOT:-/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6r5_run4_shadow_probe_current}"
domain_id="${BIO_NAV_RUN4_SHADOW_DOMAIN_ID:-152}"
duration_s="${BIO_NAV_RUN4_SHADOW_DURATION_S:-20}"
integration_root="${BIO_NAV_INTEGRATION_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_integration}"
module2_root="${BIO_NAV_MODULE2_V310_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_module2}"
dry_run=false

while (($# > 0)); do
  case "$1" in
    --run-root)
      (($# >= 2)) || { usage >&2; exit 2; }
      run_root="$2"
      shift 2
      ;;
    --domain)
      (($# >= 2)) || { usage >&2; exit 2; }
      domain_id="$2"
      shift 2
      ;;
    --duration)
      (($# >= 2)) || { usage >&2; exit 2; }
      duration_s="$2"
      shift 2
      ;;
    --integration-root)
      (($# >= 2)) || { usage >&2; exit 2; }
      integration_root="$2"
      shift 2
      ;;
    --module2-root)
      (($# >= 2)) || { usage >&2; exit 2; }
      module2_root="$2"
      shift 2
      ;;
    --dry-run)
      dry_run=true
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *) break ;;
  esac
done

component="${1:-}"
[[ -n "${component}" ]] || { usage >&2; exit 2; }
shift

[[ "${domain_id}" =~ ^[0-9]+$ && "${domain_id}" -le 232 ]] || {
  echo "domain must be an integer in [0,232]" >&2
  exit 2
}
[[ "${duration_s}" =~ ^[1-9][0-9]*$ ]] || {
  echo "duration must be a positive integer number of seconds" >&2
  exit 2
}

candidate_manifest="${BIO_NAV_RUN4_CANDIDATE_MANIFEST:-${integration_root}/ros2_ws/src/bio_nav_ros_bridge/config/kujiale_0026_run4_read_only_shadow_candidate.json}"
probe_config="${BIO_NAV_RUN4_SHADOW_PROBE_CONFIG:-${default_probe_config}}"
server_entry="${integration_root}/scripts/run_module2_v310_server.sh"
socket_path="${BIO_NAV_RUN4_SHADOW_SOCKET_PATH:-/tmp/bio_nav_run4_shadow_${UID}/domain_${domain_id}/module2.sock}"
bag_path="${run_root}/rosbag/run4_shadow_stationary"

readonly expected_manifest_status="READ_ONLY_CAUSAL_CANDIDATE_STARTUP_ONLY"
readonly expected_model_id="kujiale_0026_visual_heads_run4_v310"
readonly expected_checkpoint="/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6r5_kujiale_run4_20260826T112503Z/checkpoints/kujiale_0026_visual_heads_run4_v310.pt"
readonly expected_checkpoint_sha256="80f0b104c68899f1865a4369f091a16631ed8f178b895c84b8d072cbe10a7821"
readonly expected_pregate_config="/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6r5_module1_v2b_validation_20260826T133516Z/derived_v310_read_only/posterior_pregate_v2/posterior_region_pregate_config_v1.json"
readonly expected_pregate_sha256="72d5ba175b97ae12d55881d0f7ad73e025b5971d23bae9857c6da76fc532d1da"

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

require_live_file() {
  [[ "${dry_run}" == true || -f "$1" ]] || {
    echo "required file is missing: $1" >&2
    exit 2
  }
}

server_command=(
  "${server_entry}"
  --candidate-manifest "${candidate_manifest}"
  --socket "${socket_path}"
  --device "${BIO_NAV_RUN4_SHADOW_DEVICE:-cuda}"
)
bridge_command=(
  ros2 launch bio_nav_ros_bridge v6_cognitive_navigation.launch.py
  startup_profile:=estimated_shadow
  localization_supervisor_mode:=shadow
  "localization_candidate_manifest:=${candidate_manifest}"
  "socket_path:=${socket_path}"
  use_sim_time:=true
)
record_topics=(
  /bio_nav/module2/planning_prior
  /bio_nav/localization/candidates
  /diagnostics
  /initialpose
  /bio_nav/module1/odom
  /cmd_vel
  /cmd_vel_nav
  /cmd_vel_sim
  /tf
  /tf_static
)
record_command=(
  timeout --signal=INT --kill-after=5s "${duration_s}s"
  ros2 bag record --use-sim-time --storage mcap
  --storage-preset-profile zstd_fast --output "${bag_path}"
  "${record_topics[@]}"
)

case "${component}" in
  plan)
    cat <<EOF
status=T2_FAIL_KEEP_SHADOW_ONLY
qualification=NOT_QUALIFIED
probe_config=${probe_config}
candidate_manifest=${candidate_manifest}
expected_manifest_status=${expected_manifest_status}
expected_model_id=${expected_model_id}
expected_checkpoint=${expected_checkpoint}
expected_checkpoint_sha256=${expected_checkpoint_sha256}
expected_pregate_config=${expected_pregate_config}
expected_pregate_sha256=${expected_pregate_sha256}
startup_profile=estimated_shadow
localization_supervisor_mode=shadow
active_effect_scope=none
navigation_dispatch=false
expected_initialpose_writes=0
server:
EOF
    print_command "${server_command[@]}" "$@"
    printf 'bridge:\n'
    print_command "${bridge_command[@]}" "$@"
    printf 'record:\n'
    print_command "${record_command[@]}" "$@"
    ;;
  server)
    require_live_file "${candidate_manifest}"
    require_live_file "${server_entry}"
    if [[ "${dry_run}" == false ]]; then
      mkdir -p -m 700 "$(dirname -- "${socket_path}")"
    fi
    export BIO_NAV_MODULE2_V310_ROOT="${module2_root}"
    export ROS_DOMAIN_ID="${domain_id}"
    run_command "${server_command[@]}" "$@"
    ;;
  bridge)
    require_live_file "${candidate_manifest}"
    export ROS_DOMAIN_ID="${domain_id}"
    if [[ "${dry_run}" == false ]]; then
      # shellcheck source=lib/common.sh
      source "${script_dir}/lib/common.sh"
      source_ros --require-integration-underlay
    fi
    run_command "${bridge_command[@]}" "$@"
    ;;
  record)
    export ROS_DOMAIN_ID="${domain_id}"
    if [[ "${dry_run}" == false ]]; then
      # shellcheck source=lib/common.sh
      source "${script_dir}/lib/common.sh"
      source_ros --require-integration-underlay
      mkdir -p "${run_root}/rosbag"
      [[ ! -e "${bag_path}" ]] || {
        echo "refusing to overwrite ${bag_path}" >&2
        exit 2
      }
    fi
    run_command "${record_command[@]}" "$@"
    ;;
  *)
    usage >&2
    echo "unknown component: ${component}" >&2
    exit 2
    ;;
esac
