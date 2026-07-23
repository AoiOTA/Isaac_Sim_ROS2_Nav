#!/usr/bin/env bash

# Run the current Kujiale static 20-seed candidate batch and render its
# self-contained report.  Isaac and Nav2 are deliberately started in separate
# terminals so their live logs remain visible and Ctrl+C ownership is clear.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

if [[ $# -gt 1 ]]; then
  die "usage: $0 [CAMPAIGN_ID]"
fi

campaign_id="${1:-$(date +%Y%m%d-%H%M%S)}"
if [[ ! "${campaign_id}" =~ ^[0-9]{8}-[0-9]{6}$ ]]; then
  die "CAMPAIGN_ID must use YYYYMMDD-HHMMSS, got: ${campaign_id}"
fi

source_ros --require-workspace
scenario_file="${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/kujiale_static_long_range.yaml"
run_root="${PROJECT_ROOT}/data/experiment_runs/kujiale_long_route_static_${campaign_id}"
report_root="${PROJECT_ROOT}/data/reports/kujiale_long_route_static_${campaign_id}"
require_file "${scenario_file}"

if [[ -e "${run_root}" || -e "${report_root}" ]]; then
  die "refusing to overwrite an existing campaign: ${run_root} or ${report_root}"
fi

log_info "Kujiale static candidate campaign=${campaign_id}"
log_info "scenario=${scenario_file}"
log_info "run output=${run_root}"
log_info "report output=${report_root}"
log_info "requires an already-active static Isaac + Nav2 session; see docs/user_manual.md §7.2"

"${SCRIPT_DIR}/run_experiment.sh" "${scenario_file}" "${run_root}"

# A failed acceptance still produces a report and intentionally returns 2.
set +e
ros2 run robot_experiments kujiale_campaign \
  --static-only \
  --run-directory "${run_root}" \
  --output-directory "${report_root}"
report_status=$?
set -e

if [[ ${report_status} -eq 0 ]]; then
  log_info "static candidate passed; open: ${report_root}/index.html"
elif [[ ${report_status} -eq 2 ]]; then
  log_warn "static candidate did not meet acceptance gates; report was still generated: ${report_root}/index.html"
else
  die "report generation failed (exit ${report_status}); inspect ${run_root}"
fi

exit "${report_status}"
