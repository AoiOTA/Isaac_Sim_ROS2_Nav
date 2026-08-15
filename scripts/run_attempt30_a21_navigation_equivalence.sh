#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
INTEGRATION_ROOT="/home/lyb/Workspace/Bio_Nav/worktrees/integration/attempt30-a21-v310-srdr-rviz"
EVIDENCE_ROOT="${INTEGRATION_ROOT}/docs/evidence/attempt30_a21_v310"
TRIAL_RUNNER="${SCRIPT_DIR}/run_attempt30_a21_four_arm_trial.sh"

campaign_id="${1:-}"
first_domain="${2:-}"
[[ "${campaign_id}" =~ ^campaign_[a-zA-Z0-9_]+$ && "${first_domain}" =~ ^[0-9]+$ ]] || {
  echo "usage: $0 campaign_ID FIRST_ROS_DOMAIN" >&2
  exit 2
}
(( first_domain >= 0 && first_domain + 5 <= 232 )) || {
  echo "six consecutive ROS domains are required in 0..232" >&2
  exit 2
}
campaign_root="${EVIDENCE_ROOT}/contract_navigation/5_3_13/${campaign_id}"
[[ ! -e "${campaign_root}" ]] || {
  echo "refusing to reuse campaign path: ${campaign_root}" >&2
  exit 2
}
mkdir -p "${campaign_root}/summary"

current_method="predispatch"
current_variant="none"
current_domain="${first_domain}"
complete=false
write_marker() {
  python3 - "$1" "$2" "${campaign_id}" "${current_method}" "${current_variant}" "${current_domain}" <<'PY'
import json, sys
from datetime import datetime, timezone
from pathlib import Path
Path(sys.argv[1]).write_text(json.dumps({
    "schema": "attempt30_a21_contract_navigation_campaign_status_v1",
    "classification": "engineering_evidence_not_qualification",
    "status": sys.argv[2], "campaign_id": sys.argv[3],
    "method_at_exit": sys.argv[4], "variant_at_exit": sys.argv[5],
    "ros_domain_at_exit": int(sys.argv[6]),
    "timestamp_utc": datetime.now(timezone.utc).isoformat(),
    "reuse_allowed": False,
}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY
}
on_exit() {
  status=$?
  trap - EXIT INT TERM
  [[ "${complete}" == true ]] || write_marker "${campaign_root}/CAMPAIGN_STOP.json" STOP
  exit "${status}"
}
trap on_exit EXIT
trap 'exit 130' INT TERM

domain="${first_domain}"
for current_variant in v1 v3 v5; do
  for current_method in cognitive classic_iterative; do
    current_domain="${domain}"
    echo "EQUIVALENCE_ROW_START method=${current_method} variant=${current_variant} domain=${current_domain}"
    ATTEMPT30_A21_ADAPTATION_METHOD="${current_method}" \
      ATTEMPT30_A21_TRIAL_DOMAIN_ID="${current_domain}" \
      ATTEMPT30_A21_TRIAL_TIMEOUT_S=110 \
      "${TRIAL_RUNNER}" Q36_51 srdr "${current_variant}" "${campaign_root}/${current_method}"
    echo "EQUIVALENCE_ROW_PASS method=${current_method} variant=${current_variant} domain=${current_domain}"
    domain=$((domain + 1))
  done
done

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
set +u
source /opt/ros/jazzy/setup.bash
source "${PROJECT_ROOT}/install/setup.bash"
set -u
python3 "${SCRIPT_DIR}/summarize_attempt30_a21_navigation_equivalence.py" \
  --campaign-root "${campaign_root}" \
  --map "${EVIDENCE_ROOT}/multiroute_benchmark_v4/attempt30_a21_multiroute_v4.yaml" \
  --defaults "${INTEGRATION_ROOT}/ros2_ws/src/bio_nav_ros_bridge/config/engineering_defaults.yaml" \
  --output-prefix "${campaign_root}/summary/navigation_equivalence"
write_marker "${campaign_root}/CAMPAIGN_COMPLETE.json" COMPLETE
complete=true
trap - EXIT
echo "NAVIGATION_EQUIVALENCE_CAMPAIGN_COMPLETE campaign=${campaign_id} pairs=3"
