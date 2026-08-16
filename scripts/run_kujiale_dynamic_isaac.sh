#!/usr/bin/env bash
# Canonical Isaac entrypoint for the Kujiale dynamic-avoidance benchmark.
# Keep every run-critical contract explicit: USD, G1 spawn, odometry mode
# (ideal|realistic, default ideal via the optional first positional argument),
# schema-v4 sequential actors, and the independent ground-truth evidence recorder.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

environment_root="${KUJIALE_ENVIRONMENT_ROOT:-/home/lyb/kujiale_usd_rooms_20260717}"
[[ -d "${environment_root}" ]] || die "Kujiale environment root not found: ${environment_root}"

export ISAAC_NAV__GROUND_TRUTH__ENABLED=true
odometry_mode="ideal"
if [[ $# -ge 1 && "$1" != -* ]]; then
  odometry_mode="$1"; shift
fi
case "${odometry_mode}" in
  ideal|realistic) ;;
  *) die "usage: $0 [ideal|realistic] [--spawn-pose NAME] [run_isaac.sh options]; odometry mode must be ideal or realistic, got: ${odometry_mode}" ;;
esac
spawn_pose="long_route_start_g1"
forward_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --spawn-pose)
      [[ $# -ge 2 ]] || die "--spawn-pose requires a configured pose name"
      spawn_pose="$2"
      shift 2
      ;;
    *)
      forward_args+=("$1")
      shift
      ;;
  esac
done
exec "${SCRIPT_DIR}/run_isaac.sh" \
  --environment-root "${environment_root}" \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --spawn-poses-file "${PROJECT_ROOT}/isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.spawn.yaml" \
  --spawn-pose "${spawn_pose}" \
  --navigation-mode localization \
  --mode "${odometry_mode}" \
  --camera-profile rgbd_navigation \
  --dynamic-obstacles \
  --dynamic-obstacle-config "${PROJECT_ROOT}/isaac_sim/configs/experiments/kujiale_long_range_dynamic.yaml" \
  "${forward_args[@]}"
