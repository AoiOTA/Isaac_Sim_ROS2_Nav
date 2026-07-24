#!/usr/bin/env bash
# User-operated formal acceptance.  Isaac and Nav2 must already be active with
# --dynamic-obstacles; every stage refuses to overwrite a campaign directory.
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; source "${SCRIPT_DIR}/lib/common.sh"
[[ $# -eq 2 ]] || die "usage: $0 {pilot|controlled-20|full-route-5|report|all} CAMPAIGN_ID"
stage="$1"; id="$2"; [[ "$id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || die "invalid CAMPAIGN_ID"
source_ros --require-workspace
base="${PROJECT_ROOT}/data/experiment_runs/kujiale_dynamic_${id}"; report="${PROJECT_ROOT}/data/reports/kujiale_dynamic_${id}"
run_stage() { local name="$1" file="$2" root="${base}/${name}"; [[ ! -e "$root" ]] || die "refusing to overwrite $root"; "${SCRIPT_DIR}/run_experiment.sh" "$file" "$root"; }
case "$stage" in
 pilot) run_stage pilot "${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/kujiale_dynamic_visual.yaml";;
 controlled-20) run_stage controlled-20 "${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/kujiale_dynamic_controlled_20.yaml";;
 full-route-5) run_stage full-route-5 "${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/kujiale_dynamic_full_route_5.yaml";;
 report) [[ -d "$base/controlled-20" && -d "$base/full-route-5" ]] || die "controlled-20 and full-route-5 results are required"; [[ ! -e "$report" ]] || die "refusing to overwrite $report"; ros2 run robot_experiments dynamic_avoidance_campaign --controlled-directory "$base/controlled-20" --full-route-directory "$base/full-route-5" --output-directory "$report";;
 all)
   # Always render the report once both evidence directories exist, including
   # a failed gate result; preserve the first non-zero status for automation.
   set +e; "$0" controlled-20 "$id"; controlled_status=$?; "$0" full-route-5 "$id"; full_status=$?; "$0" report "$id"; report_status=$?; set -e
   [[ $controlled_status -eq 0 && $full_status -eq 0 ]] || exit 1
   exit "$report_status";;
 *) die "unknown stage: $stage";; esac
