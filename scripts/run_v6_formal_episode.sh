#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: $0 --pilot|--formal MANIFEST [runner arguments...]" >&2
}

mode="${1:-}"
if [[ "$mode" != "--pilot" && "$mode" != "--formal" ]]; then
  usage
  echo "STOP: mode is required; formal dispatch is disabled by default" >&2
  exit 2
fi
shift

manifest="${1:-}"
if [[ -z "$manifest" ]]; then
  usage
  exit 2
fi
shift

if [[ "$mode" == "--pilot" ]]; then
  exec ros2 run robot_experiments v6_formal_episode \
    --manifest "$manifest" --mode pilot "$@"
fi

if [[ "${BIO_NAV_V6_ALLOW_FORMAL_DISPATCH:-}" != "YES" ]]; then
  echo "STOP: set BIO_NAV_V6_ALLOW_FORMAL_DISPATCH=YES for an explicit formal dispatch" >&2
  exit 2
fi
exec ros2 run robot_experiments v6_formal_episode \
  --manifest "$manifest" --mode formal --allow-formal-dispatch "$@"
