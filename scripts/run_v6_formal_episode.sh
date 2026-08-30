#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  echo "usage: $0 --pilot [--dispatch-pilot] MANIFEST [runner arguments...]" >&2
  echo "       $0 --formal MANIFEST" >&2
  echo "       $0 --formal --execute-formal --condition-stack-id ID MANIFEST" >&2
}

mode="${1:-}"
if [[ "$mode" != "--pilot" && "$mode" != "--formal" ]]; then
  usage
  echo "STOP: mode is required; formal dispatch is disabled by default" >&2
  exit 2
fi
shift

pilot_dispatch=()
formal_execute=()
if [[ "$mode" == "--pilot" && "${1:-}" == "--dispatch-pilot" ]]; then
  pilot_dispatch=(--dispatch-pilot)
  shift
elif [[ "$mode" == "--formal" && "${1:-}" == "--execute-formal" ]]; then
  [[ "${2:-}" == "--condition-stack-id" && -n "${3:-}" ]] || {
    usage
    echo "STOP: formal execution requires --condition-stack-id ID" >&2
    exit 2
  }
  formal_execute=(--execute-formal --condition-stack-id "$3")
  shift 3
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
