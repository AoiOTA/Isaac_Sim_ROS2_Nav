#!/usr/bin/env bash
# Controller for the exact A21 3x20 campaign. It never starts or stops stacks.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

usage() {
  cat <<'USAGE'
usage:
  run_attempt30_a21_qualification.sh preflight static|dynamic|appearance
  run_attempt30_a21_qualification.sh pilot static|dynamic|appearance CAMPAIGN_ID [--resume]
  run_attempt30_a21_qualification.sh run static|dynamic|appearance CAMPAIGN_ID [--resume]
  run_attempt30_a21_qualification.sh status CAMPAIGN_ID
  run_attempt30_a21_qualification.sh report CAMPAIGN_ID
USAGE
}

[[ $# -ge 1 ]] || { usage >&2; exit 2; }
command_name="$1"; shift

require_group() {
  [[ "$1" == "static" || "$1" == "dynamic" || "$1" == "appearance" ]] \
    || die "group must be static, dynamic or appearance"
}

require_campaign() {
  [[ "$1" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || die "invalid campaign id: $1"
}

preflight() {
  local group="$1" available_gib topics services
  require_group "${group}"
  source_ros --require-workspace
  available_gib="$(df -Pk "${PROJECT_ROOT}" | awk 'NR == 2 { print int($4 / 1024 / 1024) }')"
  [[ "${available_gib}" =~ ^[0-9]+$ && "${available_gib}" -ge 10 ]] \
    || die "qualification without bags requires at least 10 GiB free; available=${available_gib:-unknown}"
  require_file "${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/attempt30_a21_qualification_${group}.yaml"
  require_file "${PROJECT_ROOT}/data/maps/occupancy/warehouse_new.yaml"
  require_file "${PROJECT_ROOT}/data/maps/posegraphs/warehouse_new.posegraph"
  topics="$(ros2 topic list 2>/dev/null || true)"
  for topic in /clock /ground_truth/odom /odom /scan_safety /simulation/collision \
    /bio_nav/navigation_graph /bio_nav/canonical_route /bio_nav/route_progress \
    /bio_nav/route_goal_complete /bio_nav/cognitive_map/constraints \
    /bio_nav/module2/planning_prior /bio_nav/module2/edge_priors \
    /bio_nav/module2/srdr_edge_diagnostics /bio_nav/route_edge_costs \
    /bio_nav/v310/rviz; do
    grep -qx "${topic}" <<<"${topics}" || die "required live topic is absent: ${topic}"
  done
  services="$(ros2 service list 2>/dev/null || true)"
  grep -qx "/bio_nav/get_goal_planning_prior" <<<"${services}" \
    || die "V3.10 same-session goal-prior service is absent"
  if [[ "${group}" == "appearance" ]]; then
    grep -qx "/experiment/appearance/state" <<<"${topics}" \
      || die "appearance state topic is absent"
  fi
  timeout 8s bash -c 'ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -qm 1 "Translation"' \
    || die "map -> base_link TF is unavailable"
  timeout 8s ros2 lifecycle get /route_server 2>/dev/null | grep -qi active \
    || die "Nav2 Route Server is not active"
  log_info "A21 ${group} preflight passed; free=${available_gib} GiB; bags=disabled"
}

run_group() {
  local group="$1" campaign="$2" pilot="$3" resume="$4" scenario output profile indices=""
  require_group "${group}"; require_campaign "${campaign}"; preflight "${group}"
  scenario="${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/attempt30_a21_qualification_${group}.yaml"
  profile="stable"
  [[ "${group}" == "dynamic" ]] && profile="dynamic_avoidance"
  if [[ "${pilot}" == "true" ]]; then
    output="${PROJECT_ROOT}/data/experiment_runs/attempt30_a21_pilot_${campaign}/${group}"
    indices="1"
  else
    output="${PROJECT_ROOT}/data/experiment_runs/attempt30_a21_${campaign}"
  fi
  arguments=(
    "${SCRIPT_DIR}/run_experiment.sh" "${scenario}" "${output}"
    "navigation_execution_backend:=route_guided"
    "record_bag:=false"
    "record_evidence:=true"
    "nav2_profile:=${profile}"
    "resume:=${resume}"
  )
  [[ -z "${indices}" ]] || arguments+=("run_indices:=${indices}")
  "${arguments[@]}"
  if [[ "${pilot}" == "true" ]]; then
    PYTHONPATH="${PROJECT_ROOT}/ros2_ws/src/robot_experiments${PYTHONPATH:+:${PYTHONPATH}}" \
      python3 - "${output}/attempt30_a21_qualification_${group}" <<'PY'
import json, pathlib, sys
roots = list(pathlib.Path(sys.argv[1]).glob("run-*/run_summary.json"))
if len(roots) != 1:
    raise SystemExit(f"pilot expected one run summary, found {len(roots)}")
value = json.loads(roots[0].read_text())
if not value.get("strict_success") or not value.get("data_complete"):
    raise SystemExit(f"pilot failed: {roots[0]}")
PY
  fi
}

case "${command_name}" in
  preflight)
    [[ $# -eq 1 ]] || { usage >&2; exit 2; }
    preflight "$1"
    ;;
  pilot|run)
    [[ $# -ge 2 && $# -le 3 ]] || { usage >&2; exit 2; }
    group="$1"; campaign="$2"; resume=false
    [[ "${3:-}" == "--resume" ]] && resume=true
    [[ -z "${3:-}" || "${3:-}" == "--resume" ]] || die "expected optional --resume"
    [[ "${command_name}" == "pilot" ]] && pilot=true || pilot=false
    run_group "${group}" "${campaign}" "${pilot}" "${resume}"
    ;;
  status)
    [[ $# -eq 1 ]] || { usage >&2; exit 2; }
    require_campaign "$1"
    find "${PROJECT_ROOT}/data/experiment_runs/attempt30_a21_$1" \
      -name run_summary.json -not -path '*.incomplete-*' -print 2>/dev/null | sort
    ;;
  report)
    [[ $# -eq 1 ]] || { usage >&2; exit 2; }
    require_campaign "$1"; source_ros --require-workspace
    PYTHONPATH="${PROJECT_ROOT}/ros2_ws/src/robot_experiments${PYTHONPATH:+:${PYTHONPATH}}" \
      python3 -m robot_experiments.attempt30_a21_qualification \
      --input-root "${PROJECT_ROOT}/data/experiment_runs/attempt30_a21_$1" \
      --output-dir "${PROJECT_ROOT}/data/reports/attempt30_a21_$1"
    ;;
  *) usage >&2; exit 2 ;;
esac
