#!/usr/bin/env bash
set -Eeuo pipefail

module3_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Reuse the validated visual launcher and change only goal ownership: RViz is
# the sole goal publisher until the operator clicks 2D Goal Pose.
export RIVERMARK_AUTO_GOAL=0
export RIVERMARK_VISUAL_ROUTE=0

exec "${module3_root}/scripts/run_rivermark_visual.sh" "$@"
