#!/usr/bin/env bash
# Start the only Isaac process needed for one half of the 4x20 campaign.
# The runner switches appearance_profile_id between resets through Isaac's
# anonymous Session Layer; do not restart Isaac between paired rounds.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

[[ $# -ge 1 ]] \
  || die "usage: $0 static|dynamic|v6-low-obstacles [run_isaac.sh options]"
mode="$1"; shift
environment_root="${KUJIALE_ENVIRONMENT_ROOT:-/home/lyb/kujiale_usd_rooms_20260717}"
require_directory "${environment_root}"
appearance="${PROJECT_ROOT}/isaac_sim/configs/experiments/kujiale_appearance_profiles.yaml"
spawn_pose="${ISAAC_NAV_ATTEMPT30_SPAWN_POSE:-long_route_start_g1}"
odometry_mode="ideal"
require_file "${appearance}"

case "${mode}" in
  static)
    obstacle_config="${ISAAC_NAV_STATIC_OBSTACLE_CONFIG:-${PROJECT_ROOT}/isaac_sim/configs/experiments/kujiale_long_range_static.yaml}"
    ;;
  dynamic)
    obstacle_config="${ISAAC_NAV_DYNAMIC_OBSTACLE_CONFIG:-${PROJECT_ROOT}/isaac_sim/configs/experiments/kujiale_long_range_dynamic.yaml}"
    ;;
  v6-low-obstacles)
    obstacle_config="${PROJECT_ROOT}/isaac_sim/configs/experiments/v6_kujiale_low_obstacles_frozen.yaml"
    odometry_mode="realistic"
    ;;
  *) die "mode must be static, dynamic, or v6-low-obstacles, got: ${mode}" ;;
esac

export ISAAC_NAV__GROUND_TRUTH__ENABLED=true
# Explicit caller options are last and therefore override profile defaults.
exec "${SCRIPT_DIR}/run_isaac.sh" \
  --environment-root "${environment_root}" \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --spawn-poses-file "${PROJECT_ROOT}/isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.spawn.yaml" \
  --spawn-pose "${spawn_pose}" \
  --navigation-mode localization \
  --mode "${odometry_mode}" \
  --camera-profile rgbd_navigation \
  --dynamic-obstacle-config "${obstacle_config}" \
  --dynamic-obstacles \
  --appearance-config "${appearance}" \
  --appearance-profile baseline \
  "$@"
