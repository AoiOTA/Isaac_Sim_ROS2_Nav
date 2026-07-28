#!/usr/bin/env bash
# Authorization-only cold starts must not create RGB-D sensors or depend on
# camera frames: no navigation goal or Module2 image inference is permitted.
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[[ $# -ge 1 ]] || { echo "usage: $0 static|dynamic [run_isaac.sh options]" >&2; exit 2; }

# The delegated launcher declares rgbd_navigation for full navigation.  Put
# the explicit off profile last so argparse resolves the authorization-only
# contract unambiguously without changing any production 4x20 behavior.
exec "${script_dir}/run_kujiale_4x20_isaac.sh" "$1" --camera-profile off "${@:2}"
