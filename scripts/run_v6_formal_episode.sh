#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  echo "usage: $0 --pilot [--dispatch-pilot] MANIFEST [runner arguments...]" >&2
  echo "       $0 --freeze-pilot PILOT_MANIFEST PILOT_AGGREGATE OUTPUT_MANIFEST FORMAL_OUTPUT_ROOT" >&2
  echo "       $0 --formal MANIFEST" >&2
  echo "       $0 --formal --execute-formal --condition-stack-id ID" >&2
  echo "          --condition-stack-contract PATH MANIFEST" >&2
}

mode="${1:-}"
if [[ "$mode" != "--pilot" && "$mode" != "--formal" && "$mode" != "--freeze-pilot" ]]; then
  usage
  echo "STOP: mode is required; formal dispatch is disabled by default" >&2
  exit 2
fi
shift

if [[ "$mode" == "--freeze-pilot" ]]; then
  [[ -n "${1:-}" && -n "${2:-}" && -n "${3:-}" && -n "${4:-}" && $# -eq 4 ]] || {
    usage
    exit 2
  }
  exec ros2 run robot_experiments v6_formal_episode \
    --pilot-manifest "$1" \
    --pilot-aggregate "$2" \
    --output-manifest "$3" \
    --formal-output-root "$4"
fi

pilot_dispatch=()
formal_execute=()
if [[ "$mode" == "--pilot" && "${1:-}" == "--dispatch-pilot" ]]; then
  pilot_dispatch=(--dispatch-pilot)
  shift
elif [[ "$mode" == "--formal" && "${1:-}" == "--execute-formal" ]]; then
  [[ "${2:-}" == "--condition-stack-id" && -n "${3:-}" \
      && "${4:-}" == "--condition-stack-contract" && -n "${5:-}" ]] || {
    usage
    echo "STOP: formal execution requires stack ID and contract path" >&2
    exit 2
  }
  formal_execute=(
    --execute-formal
    --condition-stack-id "$3"
    --condition-stack-contract "$5"
  )
  shift 5
fi

manifest="${1:-}"
if [[ -z "$manifest" ]]; then
  usage
  exit 2
fi
shift

if [[ "$mode" == "--pilot" ]]; then
  exec ros2 run robot_experiments v6_formal_episode \
    --manifest "$manifest" --pilot "${pilot_dispatch[@]}" "$@"
fi

exec ros2 run robot_experiments v6_formal_episode \
  --formal-manifest "$manifest" "${formal_execute[@]}" "$@"
