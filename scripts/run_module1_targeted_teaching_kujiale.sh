#!/usr/bin/env bash
# Automatic raw teaching capture on the accepted original Kujiale scene.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<'USAGE'
usage: run_module1_targeted_teaching_kujiale.sh [--run-root PATH] [--domain ID]
       ros|isaac|collect [arguments...]
       manifest|record|runner|episode en|sw [arguments...]

Start `ros` and `isaac` in separate terminals, then run `collect` once.  The
collect command records and dispatches EN followed by SW, with one MCAP bag and
one exactly-once reset per episode.  `episode en|sw` runs only one route.

This is raw Module1 teaching capture: original USD, accepted static map/spawn/
GVG, mixed odometry, M0 navigation, Module2/CPG/dynamic effects off.  GT is
recorded for the later offline evaluator/label audit and is never a goal input.
USAGE
}

run_root="${BIO_NAV_TARGETED_TEACHING_RUN_ROOT:-/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6r5_module1_targeted_teaching_current}"
domain_id="${BIO_NAV_TARGETED_TEACHING_DOMAIN_ID:-150}"
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
    -h|--help)
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

export ISAAC_NAV_EXPECTED_DOMAIN_ID="${domain_id}"
export ROS_DOMAIN_ID="${domain_id}"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

readonly PHASE_B_WRAPPER="${SCRIPT_DIR}/run_v6_r5_phase_b_kujiale.sh"
readonly RECORDER="${SCRIPT_DIR}/record_module1_kujiale_scene.sh"
readonly CONFIG_DIR="${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config"
readonly RECORDER_SHUTDOWN_GRACE_SECONDS=10

normalize_route() {
  case "${1,,}" in
    en) printf 'en\n' ;;
    sw) printf 'sw\n' ;;
    *) die "route must be en or sw" ;;
  esac
}

manifest_for_route() {
  local route
  route="$(normalize_route "$1")"
  printf '%s/module1_targeted_teaching_kujiale_%s.yaml\n' \
    "${CONFIG_DIR}" "${route}"
}

episode_name() {
  local route
  route="$(normalize_route "$1")"
  printf '%s_A_base\n' "${route}"
}

run_runner() {
  local route manifest output
  route="$(normalize_route "$1")"
  shift
  manifest="$(manifest_for_route "${route}")"
  require_file "${manifest}"
  mkdir -p "${run_root}/episodes"
  output="${run_root}/episodes/${route}.jsonl"
  [[ ! -e "${output}" ]] || die "refusing to overwrite ${output}"
  ros2 run robot_experiments module1_targeted_teaching \
    --manifest "${manifest}" \
    --dispatch \
    --output-jsonl "${output}" \
    "$@"
}

run_episode() {
  local route name bag recorder_log recorder_pid deadline runner_status
  route="$(normalize_route "$1")"
  shift
  name="$(episode_name "${route}")"
  bag="${run_root}/raw_mcap/${name}"
  recorder_log="${run_root}/logs/record_${route}.log"
  [[ ! -e "${bag}" ]] || die "refusing to overwrite ${bag}"
  mkdir -p "${run_root}/raw_mcap" "${run_root}/logs"

  "${RECORDER}" \
    --root "${run_root}/raw_mcap" \
    --episode "${name}" \
    >"${recorder_log}" 2>&1 &
  recorder_pid=$!
  cleanup_recorder() {
    if kill -0 "${recorder_pid}" 2>/dev/null; then
      # Background jobs inherit SIGINT as ignored from a non-interactive shell.
      # rosbag2 handles SIGTERM gracefully and writes its metadata before exit.
      kill -TERM "${recorder_pid}" 2>/dev/null || true
      deadline=$((SECONDS + RECORDER_SHUTDOWN_GRACE_SECONDS))
      while kill -0 "${recorder_pid}" 2>/dev/null \
          && ((SECONDS < deadline)); do
        sleep 0.1
      done
      if kill -0 "${recorder_pid}" 2>/dev/null; then
        kill -KILL "${recorder_pid}" 2>/dev/null || true
      fi
    fi
    wait "${recorder_pid}" 2>/dev/null || true
  }
  trap cleanup_recorder EXIT INT TERM

  deadline=$((SECONDS + 20))
  while [[ ! -d "${bag}" ]]; do
    kill -0 "${recorder_pid}" 2>/dev/null \
      || die "recorder exited before ${route} bag startup; see ${recorder_log}"
    ((SECONDS < deadline)) \
      || die "recorder did not create ${bag}; see ${recorder_log}"
    sleep 1
  done

  set +e
  run_runner "${route}" "$@"
  runner_status=$?
  set -e
  cleanup_recorder
  trap - EXIT INT TERM
  ((runner_status == 0)) \
    || die "${route} targeted teaching STOP (runner status ${runner_status})"
}

case "${component}" in
  ros|isaac)
    exec "${PHASE_B_WRAPPER}" \
      --run-root "${run_root}" \
      --domain "${domain_id}" \
      "${component}" "$@"
    ;;
  manifest)
    route="$(normalize_route "${1:-}")"
    shift
    manifest="$(manifest_for_route "${route}")"
    require_file "${manifest}"
    source_ros --require-integration-underlay
    exec ros2 run robot_experiments module1_targeted_teaching \
      --manifest "${manifest}" --validate-only "$@"
    ;;
  record)
    route="$(normalize_route "${1:-}")"
    shift
    exec "${RECORDER}" \
      --root "${run_root}/raw_mcap" \
      --episode "$(episode_name "${route}")" \
      "$@"
    ;;
  runner)
    route="$(normalize_route "${1:-}")"
    shift
    source_ros --require-integration-underlay
    run_runner "${route}" "$@"
    ;;
  episode)
    route="$(normalize_route "${1:-}")"
    shift
    source_ros --require-integration-underlay
    run_episode "${route}" "$@"
    ;;
  collect)
    source_ros --require-integration-underlay
    run_episode en "$@"
    run_episode sw "$@"
    ;;
  *)
    usage >&2
    die "unknown component: ${component}"
    ;;
esac
