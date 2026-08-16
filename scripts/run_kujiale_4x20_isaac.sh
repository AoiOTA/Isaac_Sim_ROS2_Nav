#!/usr/bin/env bash
# Start the only Isaac process needed for one half of the 4x20 campaign.
# The runner switches appearance_profile_id between resets through Isaac's
# anonymous Session Layer; do not restart Isaac between paired rounds.
# The optional second positional argument selects the odometry mode
# (ideal|realistic, default ideal) forwarded to run_isaac.sh --mode.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

[[ $# -ge 1 ]] || die "usage: $0 static|dynamic [ideal|realistic] [run_isaac.sh options]"
mode="$1"; shift
odometry_mode="ideal"
if [[ $# -ge 1 && "$1" != -* ]]; then
  odometry_mode="$1"; shift
fi
case "${odometry_mode}" in
  ideal|realistic) ;;
  *) die "usage: $0 static|dynamic [ideal|realistic] [run_isaac.sh options]; odometry mode must be ideal or realistic, got: ${odometry_mode}" ;;
esac
environment_root="${KUJIALE_ENVIRONMENT_ROOT:-/home/lyb/kujiale_usd_rooms_20260717}"
require_directory "${environment_root}"
appearance="${PROJECT_ROOT}/isaac_sim/configs/experiments/kujiale_appearance_profiles.yaml"
spawn_pose="${ISAAC_NAV_ATTEMPT30_SPAWN_POSE:-long_route_start_g1}"
require_file "${appearance}"

case "${mode}" in
  static)
    obstacle_config="${ISAAC_NAV_STATIC_OBSTACLE_CONFIG:-${PROJECT_ROOT}/isaac_sim/configs/experiments/kujiale_long_range_static.yaml}"
    ;;
  dynamic)
    obstacle_config="${ISAAC_NAV_DYNAMIC_OBSTACLE_CONFIG:-${PROJECT_ROOT}/isaac_sim/configs/experiments/kujiale_long_range_dynamic.yaml}"
    ;;
  *) die "mode must be static or dynamic, got: ${mode}" ;;
esac

export ISAAC_NAV__GROUND_TRUTH__ENABLED=true
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
