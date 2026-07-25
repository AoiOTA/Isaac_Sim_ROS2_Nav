#!/usr/bin/env bash
# Start the only Isaac process needed for one half of the 4x20 campaign.
# The runner switches appearance_profile_id between resets through Isaac's
# anonymous Session Layer; do not restart Isaac between paired rounds.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

[[ $# -ge 1 ]] || die "usage: $0 static|dynamic [run_isaac.sh options]"
mode="$1"; shift
environment_root="${KUJIALE_ENVIRONMENT_ROOT:-/home/lyb/kujiale_usd_rooms_20260717}"
require_directory "${environment_root}"
appearance="${PROJECT_ROOT}/isaac_sim/configs/experiments/kujiale_appearance_profiles.yaml"
require_file "${appearance}"

case "${mode}" in
  static)
    obstacle_config="${PROJECT_ROOT}/isaac_sim/configs/experiments/kujiale_long_range_static.yaml"
    ;;
  dynamic)
    obstacle_config="${PROJECT_ROOT}/isaac_sim/configs/experiments/kujiale_long_range_dynamic.yaml"
    ;;
  *) die "mode must be static or dynamic, got: ${mode}" ;;
esac

export ISAAC_NAV__GROUND_TRUTH__ENABLED=true
exec "${SCRIPT_DIR}/run_isaac.sh" \
  --environment-root "${environment_root}" \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --spawn-poses-file "${PROJECT_ROOT}/isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.spawn.yaml" \
  --spawn-pose long_route_start_g1 \
  --navigation-mode localization \
  --mode ideal \
  --camera-profile rgbd_navigation \
  --dynamic-obstacle-config "${obstacle_config}" \
  --dynamic-obstacles \
  --appearance-config "${appearance}" \
  --appearance-profile baseline \
  "$@"
