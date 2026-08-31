#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  echo "usage: $0 --pilot [--dispatch-pilot] MANIFEST [runner arguments...]" >&2
  echo "       $0 --freeze-pilot PILOT_MANIFEST PILOT_AGGREGATE OUTPUT_MANIFEST FORMAL_OUTPUT_ROOT" >&2
  echo "       $0 --aggregate-pilot PILOT_ROOT OUT_MANIFEST OUT_AGGREGATE" >&2
  echo "       $0 --aggregate-indoor-pilot PILOT_ROOT OUT_MANIFEST OUT_AGGREGATE" >&2
  echo "       $0 --aggregate-outdoor-pilot PILOT_ROOT OUT_MANIFEST OUT_AGGREGATE" >&2
  echo "       $0 --freeze-indoor-pilot PILOT_MANIFEST PILOT_AGGREGATE OUTPUT_MANIFEST INDOOR_OUTPUT_ROOT" >&2
  echo "       $0 --freeze-outdoor-pilot PILOT_MANIFEST PILOT_AGGREGATE OUTPUT_MANIFEST OUTDOOR_OUTPUT_ROOT" >&2
  echo "       $0 --combine INDOOR_MANIFEST OUTDOOR_MANIFEST" >&2
  echo "       $0 --continue-indoor PARENT_MANIFEST SUCCESSOR_MANIFEST SUCCESSOR_OUTPUT_ROOT" >&2
  echo "       $0 --indoor MANIFEST" >&2
  echo "       $0 --indoor --execute-indoor --condition-stack-id ID" >&2
  echo "          --condition-stack-contract PATH MANIFEST" >&2
  echo "       $0 --outdoor [--execute-outdoor --condition-stack-id ID" >&2
  echo "          --condition-stack-contract PATH] MANIFEST" >&2
  echo "       $0 --formal MANIFEST" >&2
  echo "       $0 --formal --execute-formal --condition-stack-id ID" >&2
  echo "          --condition-stack-contract PATH MANIFEST" >&2
}

mode="${1:-}"
if [[ "$mode" != "--pilot" && "$mode" != "--formal" \
    && "$mode" != "--indoor" && "$mode" != "--freeze-pilot" \
    && "$mode" != "--aggregate-pilot" \
    && "$mode" != "--aggregate-outdoor-pilot" \
    && "$mode" != "--freeze-indoor-pilot" \
    && "$mode" != "--freeze-outdoor-pilot" \
    && "$mode" != "--outdoor" && "$mode" != "--combine" \
    && "$mode" != "--continue-indoor" \
    && "$mode" != "--aggregate-indoor-pilot" ]]; then
  usage
  echo "STOP: mode is required; formal dispatch is disabled by default" >&2
  exit 2
fi
shift

if [[ "$mode" == "--combine" ]]; then
  [[ -n "${1:-}" && -n "${2:-}" && $# -eq 2 ]] || { usage; exit 2; }
  exec ros2 run robot_experiments v6_formal_episode \
    --combine-qualified-halves "$1" "$2"
fi

if [[ "$mode" == "--continue-indoor" ]]; then
  [[ -n "${1:-}" && -n "${2:-}" && -n "${3:-}" && $# -eq 3 ]] || {
    usage
    exit 2
  }
  exec ros2 run robot_experiments v6_formal_episode \
    --continue-indoor-parent "$1" \
    --continuation-output-manifest "$2" \
    --continuation-output-root "$3"
fi

if [[ "$mode" == "--aggregate-indoor-pilot" ]]; then
  [[ -n "${1:-}" && -n "${2:-}" && -n "${3:-}" && $# -eq 3 ]] || {
    usage
    exit 2
  }
  exec ros2 run robot_experiments v6_formal_episode \
    --aggregate-indoor-pilot-root "$1" \
    --output-pilot-manifest "$2" \
    --output-pilot-aggregate "$3"
fi

if [[ "$mode" == "--aggregate-outdoor-pilot" ]]; then
  [[ -n "${1:-}" && -n "${2:-}" && -n "${3:-}" && $# -eq 3 ]] || {
    usage
    exit 2
  }
  exec ros2 run robot_experiments v6_formal_episode \
    --aggregate-outdoor-pilot-root "$1" \
    --output-pilot-manifest "$2" \
    --output-pilot-aggregate "$3"
fi

if [[ "$mode" == "--aggregate-pilot" ]]; then
  [[ -n "${1:-}" && -n "${2:-}" && -n "${3:-}" && $# -eq 3 ]] || {
    usage
    exit 2
  }
  exec ros2 run robot_experiments v6_formal_episode \
    --aggregate-pilot-root "$1" \
    --output-pilot-manifest "$2" \
    --output-pilot-aggregate "$3"
fi

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

if [[ "$mode" == "--freeze-indoor-pilot" ]]; then
  [[ -n "${1:-}" && -n "${2:-}" && -n "${3:-}" && -n "${4:-}" && $# -eq 4 ]] || {
    usage
    exit 2
  }
  exec ros2 run robot_experiments v6_formal_episode \
    --indoor-pilot-manifest "$1" \
    --indoor-pilot-aggregate "$2" \
    --output-manifest "$3" \
    --indoor-output-root "$4"
fi

if [[ "$mode" == "--freeze-outdoor-pilot" ]]; then
  [[ -n "${1:-}" && -n "${2:-}" && -n "${3:-}" && -n "${4:-}" && $# -eq 4 ]] || {
    usage
    exit 2
  }
  exec ros2 run robot_experiments v6_formal_episode \
    --outdoor-pilot-manifest "$1" \
    --outdoor-pilot-aggregate "$2" \
    --output-manifest "$3" \
    --outdoor-output-root "$4"
fi

pilot_dispatch=()
formal_execute=()
indoor_execute=()
outdoor_execute=()
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
elif [[ "$mode" == "--indoor" && "${1:-}" == "--execute-indoor" ]]; then
  [[ "${2:-}" == "--condition-stack-id" && -n "${3:-}" \
      && "${4:-}" == "--condition-stack-contract" && -n "${5:-}" ]] || {
    usage
    echo "STOP: indoor execution requires stack ID and contract path" >&2
    exit 2
  }
  indoor_execute=(
    --execute-indoor
    --condition-stack-id "$3"
    --condition-stack-contract "$5"
  )
  shift 5
elif [[ "$mode" == "--outdoor" && "${1:-}" == "--execute-outdoor" ]]; then
  [[ "${2:-}" == "--condition-stack-id" && -n "${3:-}" \
      && "${4:-}" == "--condition-stack-contract" && -n "${5:-}" ]] || {
    usage
    echo "STOP: outdoor execution requires stack ID and contract path" >&2
    exit 2
  }
  outdoor_execute=(
    --execute-outdoor
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

if [[ "$mode" == "--indoor" ]]; then
  exec ros2 run robot_experiments v6_formal_episode \
    --indoor-manifest "$manifest" "${indoor_execute[@]}" "$@"
fi

if [[ "$mode" == "--outdoor" ]]; then
  exec ros2 run robot_experiments v6_formal_episode \
    --outdoor-manifest "$manifest" "${outdoor_execute[@]}" "$@"
fi

exec ros2 run robot_experiments v6_formal_episode \
  --formal-manifest "$manifest" "${formal_execute[@]}" "$@"
