#!/usr/bin/env bash
# Canonical Isaac entrypoint for the Kujiale dynamic-avoidance benchmark.
# Keep every run-critical contract explicit: USD, G1 spawn, Ideal odometry,
# schema-v3 actors, and the independent ground-truth evidence recorder.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

environment_root="${KUJIALE_ENVIRONMENT_ROOT:-/home/lyb/kujiale_usd_rooms_20260717}"
[[ -d "${environment_root}" ]] || die "Kujiale environment root not found: ${environment_root}"

export ISAAC_NAV__GROUND_TRUTH__ENABLED=true
exec "${SCRIPT_DIR}/run_isaac.sh" \
  --environment-root "${environment_root}" \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --spawn-poses-file "${PROJECT_ROOT}/isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.spawn.yaml" \
  --spawn-pose long_route_start_g1 \
  --navigation-mode localization \
  --mode ideal \
  --camera-profile rgbd_navigation \
  --dynamic-obstacles \
  --dynamic-obstacle-config "${PROJECT_ROOT}/isaac_sim/configs/experiments/kujiale_long_range_dynamic.yaml" \
  "$@"
