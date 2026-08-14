#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
INTEGRATION_ROOT="${BIO_NAV_ATTEMPT30_V310_INTEGRATION_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/integration/attempt30-a21-v310-srdr-rviz}"
EVIDENCE_ROOT="${ATTEMPT30_A21_V4_EVIDENCE_ROOT:-${INTEGRATION_ROOT}/docs/evidence/attempt30_a21_v310/multiroute_benchmark_v4}"
CAMPAIGN_OUTPUT_ROOT="${ATTEMPT30_A21_CAMPAIGN_OUTPUT_ROOT:-${EVIDENCE_ROOT}/four_arm_engineering}"
TRIAL_RUNNER="${SCRIPT_DIR}/run_attempt30_a21_four_arm_trial.sh"
SUMMARIZER="${SCRIPT_DIR}/summarize_attempt30_a21_multiroute_modes.py"
MAP_FILE="${EVIDENCE_ROOT}/attempt30_a21_multiroute_v4.yaml"
DEFAULTS="${INTEGRATION_ROOT}/ros2_ws/src/bio_nav_ros_bridge/config/engineering_defaults.yaml"
INTEGRATION_SETUP="${BIO_NAV_ATTEMPT30_V310_INTEGRATION_SETUP:-${INTEGRATION_ROOT}/install/local_setup.bash}"
if [[ ! -f "${INTEGRATION_SETUP}" \
    && -f "${INTEGRATION_ROOT}/ros2_ws/install/local_setup.bash" ]]; then
  INTEGRATION_SETUP="${INTEGRATION_ROOT}/ros2_ws/install/local_setup.bash"
fi

campaign_id="${1:-}"
query_id="${2:-}"
first_domain="${3:-}"
[[ -n "${campaign_id}" && -n "${query_id}" && -n "${first_domain}" ]] || {
  echo "usage: $0 CAMPAIGN_ID Q02_58|Q01_50|Q36_04|Q14_45|Q36_51 FIRST_ROS_DOMAIN" >&2
  exit 2
}
[[ "${campaign_id}" =~ ^campaign_[a-zA-Z0-9_]+$ ]] || {
  echo "campaign ID must match campaign_[a-zA-Z0-9_]+" >&2
  exit 2
}
case "${query_id}" in
  Q02_58|Q01_50|Q36_04|Q14_45|Q36_51) ;;
  *) echo "unsupported query: ${query_id}" >&2; exit 2 ;;
esac
[[ "${first_domain}" =~ ^[0-9]+$ ]] || {
  echo "FIRST_ROS_DOMAIN must be an integer" >&2
  exit 2
}
(( first_domain >= 0 && first_domain + 19 <= 232 )) || {
  echo "campaign needs 20 consecutive ROS domains in the range 0..232" >&2
  exit 2
}
for required in "${TRIAL_RUNNER}" "${SUMMARIZER}" "${MAP_FILE}" "${DEFAULTS}" "${INTEGRATION_SETUP}"; do
  [[ -f "${required}" ]] || { echo "required file missing: ${required}" >&2; exit 2; }
done

campaign_root="${CAMPAIGN_OUTPUT_ROOT}/${campaign_id}"
[[ ! -e "${campaign_root}" ]] || {
  echo "refusing to reuse campaign path: ${campaign_root}" >&2
  exit 2
}
mkdir -p "${campaign_root}/summary"

current_arm="predispatch"
current_variant="none"
current_domain="${first_domain}"
campaign_complete=false

write_json_marker() {
  local output="$1" status="$2" exit_code="$3"
  python3 - "${output}" "${status}" "${exit_code}" "${campaign_id}" \
    "${query_id}" "${current_arm}" "${current_variant}" "${current_domain}" <<'PY'
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

path = Path(sys.argv[1])
payload = {
    "schema": "attempt30_a21_four_arm_campaign_status_v1",
    "classification": "engineering_evidence_not_qualification",
    "status": sys.argv[2],
    "exit_code": int(sys.argv[3]),
    "campaign_id": sys.argv[4],
    "query_id": sys.argv[5],
    "arm_at_exit": sys.argv[6],
    "variant_at_exit": sys.argv[7],
    "ros_domain_at_exit": int(sys.argv[8]),
    "timestamp_utc": datetime.now(timezone.utc).isoformat(),
    "reuse_allowed": False,
}
path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY
}

on_exit() {
  local status=$?
  trap - EXIT INT TERM
  if [[ "${campaign_complete}" != true ]]; then
    write_json_marker "${campaign_root}/CAMPAIGN_STOP.json" "STOP" "${status}"
  fi
  exit "${status}"
}
on_signal() {
  exit 130
}
trap on_exit EXIT
trap on_signal INT TERM

declare -a variants=(v1 v2 v3 v4 v5)
declare -a arms=(baseline sr_only dr_only srdr)
domain="${first_domain}"
predispatch_retries="${ATTEMPT30_A21_PREDISPATCH_RETRIES:-2}"
[[ "${predispatch_retries}" =~ ^[0-9]+$ ]] || {
  echo "ATTEMPT30_A21_PREDISPATCH_RETRIES must be a nonnegative integer" >&2
  exit 2
}
for current_variant in "${variants[@]}"; do
  for current_arm in "${arms[@]}"; do
    current_domain="${domain}"
    attempt=0
    while true; do
      echo "CAMPAIGN_ROW_START campaign=${campaign_id} query=${query_id} variant=${current_variant} arm=${current_arm} domain=${current_domain} attempt=${attempt}"
      set +e
      ATTEMPT30_A21_TRIAL_DOMAIN_ID="${current_domain}" \
        ATTEMPT30_A21_TRIAL_TIMEOUT_S="${ATTEMPT30_A21_TRIAL_TIMEOUT_S:-110}" \
        "${TRIAL_RUNNER}" "${query_id}" "${current_arm}" "${current_variant}" "${campaign_root}"
      trial_status=$?
      set -e
      (( trial_status == 0 )) && break

      trial_dir="${campaign_root}/${query_id}/${current_arm}/${current_variant}"
      if [[ -f "${trial_dir}/TRIAL_DISPATCHED.json" ]] \
          || (( attempt >= predispatch_retries )); then
        echo "CAMPAIGN_ROW_STOP status=${trial_status} dispatched=$([[ -f "${trial_dir}/TRIAL_DISPATCHED.json" ]] && echo true || echo false) attempt=${attempt}" >&2
        exit "${trial_status}"
      fi

      attempt_root="${campaign_root}/predispatch_attempts/${query_id}/${current_arm}/${current_variant}"
      mkdir -p "${attempt_root}"
      attempt_path="${attempt_root}/attempt_${attempt}_domain_${current_domain}"
      [[ ! -e "${attempt_path}" ]] || {
        echo "refusing to overwrite predispatch attempt: ${attempt_path}" >&2
        exit 2
      }
      mv "${trial_dir}" "${attempt_path}"
      echo "CAMPAIGN_PREDISPATCH_RETRY isolated=${attempt_path} next_attempt=$((attempt + 1))" >&2
      attempt=$((attempt + 1))
    done
    echo "CAMPAIGN_ROW_PASS campaign=${campaign_id} query=${query_id} variant=${current_variant} arm=${current_arm} domain=${current_domain}"
    domain=$((domain + 1))
  done
done

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
set +u
source /opt/ros/jazzy/setup.bash
source "${INTEGRATION_SETUP}"
source "${PROJECT_ROOT}/ros2_ws/install/local_setup.bash"
set -u
python3 "${SUMMARIZER}" \
  --runtime-root "${campaign_root}" \
  --map "${MAP_FILE}" \
  --defaults "${DEFAULTS}" \
  --output-prefix "${campaign_root}/summary/four_arm_comparison"

python3 - "${campaign_root}/summary/four_arm_comparison.json" "${query_id}" <<'PY'
import json
import sys

value = json.load(open(sys.argv[1], encoding="utf-8"))
rows = value["rows"]
if len(rows) != 20:
    raise SystemExit(f"expected 20 rows, found {len(rows)}")
if {row["query_id"] for row in rows} != {sys.argv[2]}:
    raise SystemExit("summary contains another query")
required = (
    "all_four_arms_present_per_pair",
    "all_completed",
    "all_collision_free",
    "all_factor_gates_pass",
    "all_distance_deviations_within_20pct",
)
failed = [name for name in required if not value["campaign_gate"].get(name, False)]
if failed:
    raise SystemExit(f"required campaign gates failed: {failed}")
print("CAMPAIGN_SUMMARY_PASS", sys.argv[2], len(rows))
PY

write_json_marker "${campaign_root}/CAMPAIGN_COMPLETE.json" "COMPLETE" 0
campaign_complete=true
trap - EXIT
echo "CAMPAIGN_COMPLETE campaign=${campaign_id} query=${query_id} rows=20"
