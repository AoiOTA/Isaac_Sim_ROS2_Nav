#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

usage() {
  cat <<'EOF'
usage: profile_runtime.sh [--duration SEC] [--warmup SEC] [--output FILE]
                          [--label NAME]

Measure actual /clock RTF, Topic Hz/age/bandwidth, TF lag, Nav2 warnings,
registered-process CPU/RSS, host load, and NVIDIA GPU state into one JSON file.
EOF
}

duration=60
warmup=2
label=runtime
output=""
while (($#)); do
  case "$1" in
    --duration|--warmup|--output|--label)
      (($# >= 2)) || die "$1 requires a value"
      case "$1" in
        --duration) duration="$2" ;;
        --warmup) warmup="$2" ;;
        --output) output="$2" ;;
        --label) label="$2" ;;
      esac
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      die "unknown profiler argument: $1"
      ;;
  esac
done

[[ "${duration}" =~ ^[0-9]+([.][0-9]+)?$ && "${duration}" != 0 ]] \
  || die "--duration must be positive"
[[ "${warmup}" =~ ^[0-9]+([.][0-9]+)?$ ]] \
  || die "--warmup must be non-negative"
[[ -n "${label}" ]] || die "--label must be non-empty"

source_ros --require-workspace
require_command ros2
if [[ -z "${output}" ]]; then
  timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
  output="${PROJECT_ROOT}/data/reports/runtime/runtime_${timestamp}.json"
elif [[ "${output}" != /* ]]; then
  output="${PROJECT_ROOT}/${output}"
fi

mkdir -p "$(dirname "${output}")"
log_info "profiling runtime for ${duration}s after ${warmup}s warmup"
exec ros2 run robot_experiments runtime_profiler \
  --duration "${duration}" \
  --warmup "${warmup}" \
  --output "${output}" \
  --label "${label}"
