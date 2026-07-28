#!/usr/bin/env bash
# Authorization-only cold starts need the same Bridge-owned raw RGB and
# CameraInfo ingress contract as Shadow, but never create a depth stream,
# navigation goal, or control command.
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[[ $# -ge 1 ]] || { echo "usage: $0 static|dynamic [run_isaac.sh options]" >&2; exit 2; }

# The delegated launcher declares rgbd_navigation for full navigation.  Put
# the lower-rate RGB-only monitoring profile last so argparse resolves the
# authorization ingress contract unambiguously without changing production
# 4x20 RGB-D behavior.
exec "${script_dir}/run_kujiale_4x20_isaac.sh" "$1" --camera-profile monitoring "${@:2}"
