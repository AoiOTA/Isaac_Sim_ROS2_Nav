#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: $0 --pilot [--dispatch-pilot] MANIFEST [runner arguments...]" >&2
  echo "       $0 --formal MANIFEST [runner arguments...]" >&2
}

mode="${1:-}"
if [[ "$mode" != "--pilot" && "$mode" != "--formal" ]]; then
  usage
  echo "STOP: mode is required; formal dispatch is disabled by default" >&2
  exit 2
fi
shift

pilot_dispatch=()
if [[ "$mode" == "--pilot" && "${1:-}" == "--dispatch-pilot" ]]; then
  pilot_dispatch=(--dispatch-pilot)
  shift
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

if [[ "${BIO_NAV_V6_ALLOW_FORMAL_DISPATCH:-}" != "YES" ]]; then
  echo "STOP: set BIO_NAV_V6_ALLOW_FORMAL_DISPATCH=YES for an explicit formal dispatch" >&2
  exit 2
fi
exec ros2 run robot_experiments v6_formal_episode \
  --manifest "$manifest" --allow-formal-dispatch "$@"
