#!/usr/bin/env bash
# Focused, user-operated visual entrypoints for the three-stage dynamic relay.
# Isaac and the normal navigation/RViz stack must already be running.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"

[[ $# -ge 1 ]] || die "usage: $0 {g1-g2|g2-g3|g5-g1|full} [run_kujiale_dynamic_visual.sh options]"
segment="$1"
shift
case "${segment}" in
  g1-g2) case_id="local_bypass" ;;
  g2-g3) case_id="g2_g3_exit" ;;
  g5-g1) case_id="g5_g1_crossing" ;;
  full) case_id="full_route_three_stage" ;;
  *) die "unknown segment ${segment}; expected g1-g2, g2-g3, g5-g1, or full" ;;
esac

exec "${SCRIPT_DIR}/run_kujiale_dynamic_visual.sh" --case "${case_id}" "$@"
