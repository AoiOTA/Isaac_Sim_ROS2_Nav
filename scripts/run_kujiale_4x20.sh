#!/usr/bin/env bash
# User-operated 4x20 campaign controller.  Isaac and Nav2 remain visible in
# their own terminals; this script owns only the formal runner/report process.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

usage() {
  cat <<'USAGE'
usage:
  run_kujiale_4x20.sh preflight static|dynamic
  run_kujiale_4x20.sh pilot static|dynamic CAMPAIGN_ID [--resume]
  run_kujiale_4x20.sh static-pair CAMPAIGN_ID [--resume]
  run_kujiale_4x20.sh dynamic-pair CAMPAIGN_ID [--resume]
  run_kujiale_4x20.sh status CAMPAIGN_ID
  run_kujiale_4x20.sh static-status|dynamic-status CAMPAIGN_ID
  run_kujiale_4x20.sh report CAMPAIGN_ID [--replace]
  run_kujiale_4x20.sh static-report|dynamic-report CAMPAIGN_ID [--replace]
USAGE
}

[[ $# -ge 1 ]] || { usage >&2; exit 2; }
command_name="$1"; shift

require_campaign_id() {
  local value="$1"
  [[ "${value}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || die "invalid CAMPAIGN_ID: ${value}"
}

parse_resume() {
  local value="${1:-}"
  [[ -z "${value}" || "${value}" == "--resume" ]] || die "expected optional --resume"
  [[ "${value}" == "--resume" ]] && printf 'true' || printf 'false'
}

parse_replace() {
  local value="${1:-}"
  [[ -z "${value}" || "${value}" == "--replace" ]] || die "expected optional --replace"
  [[ "${value}" == "--replace" ]] && printf 'true' || printf 'false'
}

preflight() {
  local mode="$1"
  local nav2_profile nav2_batch nav2_frequency environment_root scene_file
  [[ "${mode}" == "static" || "${mode}" == "dynamic" ]] || die "preflight mode must be static or dynamic"
  source_ros --require-workspace
  local available_gib
  available_gib="$(df -Pk "${PROJECT_ROOT}" | awk 'NR == 2 { print int($4 / 1024 / 1024) }')"
  [[ "${available_gib}" =~ ^[0-9]+$ && "${available_gib}" -ge 120 ]] \
    || die "4x20 campaign requires at least 120 GiB free; available=${available_gib:-unknown} GiB"
  require_file "${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/kujiale_4x20_${mode}_pair.yaml"
  require_file "${PROJECT_ROOT}/isaac_sim/configs/experiments/kujiale_appearance_profiles.yaml"
  require_file "${PROJECT_ROOT}/data/maps/occupancy/warehouse_new.yaml"
  require_file "${PROJECT_ROOT}/data/maps/occupancy/warehouse_new.pgm"
  require_file "${PROJECT_ROOT}/data/maps/posegraphs/warehouse_new.posegraph"
  require_file "${PROJECT_ROOT}/data/maps/posegraphs/warehouse_new.data"
  require_file "${PROJECT_ROOT}/isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.spawn.yaml"
  environment_root="${KUJIALE_ENVIRONMENT_ROOT:-/home/lyb/kujiale_usd_rooms_20260717}"
  require_directory "${environment_root}"
  # `source_ros` can activate an Isaac/Conda environment whose PATH does not
  # include ripgrep.  Use the POSIX base-system `find` here: preflight must not
  # depend on an optional developer CLI before it can validate the live stack.
  mapfile -t scene_matches < <(
    find "${environment_root}" -type f -name 'kujiale_0026_A_to_B_door_open.usd' -print
  )
  [[ "${#scene_matches[@]}" -eq 1 ]] || die "expected exactly one Kujiale scene USD below ${environment_root}; found ${#scene_matches[@]}"
  scene_file="${scene_matches[0]}"
  nav2_profile="stable"; nav2_batch="700"; nav2_frequency="10.0"
  if [[ "${mode}" == "dynamic" ]]; then
    nav2_profile="dynamic_avoidance"; nav2_batch="500"; nav2_frequency="15.0"
  fi
  require_file "${PROJECT_ROOT}/ros2_ws/src/robot_navigation/config/nav2_${nav2_profile}.yaml"
  if [[ "${mode}" == "dynamic" ]] && ! ros2 pkg prefix spatio_temporal_voxel_layer >/dev/null 2>&1; then
    die "dynamic stage requires STVL: sudo apt install ros-jazzy-spatio-temporal-voxel-layer"
  fi
  local topics
  topics="$(ros2 topic list 2>/dev/null || true)"
  for topic in /clock /ground_truth/odom /odom /camera/front/image_raw /camera/front/depth/points /experiment/appearance/state; do
    grep -qx "${topic}" <<<"${topics}" || die "required live topic is absent: ${topic}; start Isaac/Nav2 and wait for readiness"
  done
  ros2 param get /isaac_navigation_sim appearance_config_sha256 >/dev/null \
    || die "Isaac appearance contract is unavailable; start run_kujiale_4x20_isaac.sh and wait for ready log"
  ros2 param get /controller_server FollowPath.batch_size 2>/dev/null | grep -Eq "${nav2_batch}" \
    || die "Nav2 profile mismatch: expected ${nav2_profile} (FollowPath.batch_size=${nav2_batch})"
  ros2 param get /controller_server controller_frequency 2>/dev/null | grep -Eq "${nav2_frequency}" \
    || die "Nav2 profile mismatch: expected ${nav2_profile} (controller_frequency=${nav2_frequency})"
  timeout 8s bash -c 'ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -qm 1 "Translation"' \
    || die "map -> base_link TF is unavailable; wait for localization before starting the campaign"
  local map_hash scene_hash
  map_hash="$(sha256sum "${PROJECT_ROOT}/data/maps/occupancy/warehouse_new.yaml" | awk '{print $1}')"
  scene_hash="$(sha256sum "${scene_file}" | awk '{print $1}')"
  log_info "preflight passed for ${mode}; nav2=${nav2_profile}; free=${available_gib} GiB; map_sha256=${map_hash:0:12}; scene_sha256=${scene_hash:0:12}"
}

verify_pilot_evidence() {
  local output="$1" indices="$2"
  python3 - "${output}" "${indices}" <<'PY'
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
expected = {int(value) for value in sys.argv[2].split(",")}
observed: set[int] = set()
problems: list[str] = []
for manifest_path in root.rglob("run_manifest.json"):
    # A failed pilot is intentionally retained under a sibling
    # ``.incomplete-<UTC>`` directory before retry.  It is historical
    # evidence, never a second current pilot result.
    if ".incomplete-" in manifest_path.parent.name:
        continue
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        summary = json.loads(
            (manifest_path.parent / "run_summary.json").read_text(encoding="utf-8")
        )
    except (OSError, json.JSONDecodeError) as exc:
        problems.append(f"unreadable evidence {manifest_path}: {exc}")
        continue
    index = manifest.get("run_index")
    if index not in expected:
        continue
    observed.add(index)
    if manifest.get("result") != "success":
        problems.append(f"pilot run {index} result={manifest.get('result')!r}")
    if summary.get("data_complete") is not True or summary.get("checksums_verified") is not True:
        problems.append(f"pilot run {index} has incomplete evidence")
missing = sorted(expected - observed)
if missing:
    problems.append("missing pilot evidence for run indices: " + ",".join(map(str, missing)))
if problems:
    raise SystemExit("[isaac-nav] error: pilot validation failed: " + "; ".join(problems))
PY
}

run_stage() {
  local mode="$1" output="$2" indices="$3" resume="$4"
  local scenario="${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/kujiale_4x20_${mode}_pair.yaml"
  local nav2_profile="stable"
  [[ "${mode}" == "dynamic" ]] && nav2_profile="dynamic_avoidance"
  [[ ! -e "${output}" || "${resume}" == true ]] || die "refusing to overwrite ${output}; use --resume only after an interrupted run"
  mkdir -p "${output}"
  local arguments=("${SCRIPT_DIR}/run_experiment.sh" "${scenario}" "${output}" "resume:=${resume}" "nav2_profile:=${nav2_profile}")
  if [[ -n "${indices}" ]]; then
    arguments+=("run_indices:=${indices}" "require_successful_resume:=true")
  fi
  "${arguments[@]}"
  # ros2 launch can return successfully even if a launched node exits during
  # startup.  A failed pilot must never allow the formal 40-round stage to run.
  [[ -z "${indices}" ]] || verify_pilot_evidence "${output}" "${indices}"
}

case "${command_name}" in
  preflight)
    [[ $# -eq 1 ]] || { usage >&2; exit 2; }
    preflight "$1"
    ;;
  pilot)
    [[ $# -ge 2 && $# -le 3 ]] || { usage >&2; exit 2; }
    mode="$1"; campaign_id="$2"; resume="$(parse_resume "${3:-}")"; require_campaign_id "${campaign_id}"
    [[ "${mode}" == "static" || "${mode}" == "dynamic" ]] || die "pilot mode must be static or dynamic"
    preflight "${mode}"
    # Matrix row 2 is the first non-baseline profile (dim_warm) for both modes.
    run_stage "${mode}" "${PROJECT_ROOT}/data/experiment_runs/kujiale_4x20_${campaign_id}/pilot-${mode}" "2" "${resume}"
    ;;
  static-pair|dynamic-pair)
    [[ $# -ge 1 && $# -le 2 ]] || { usage >&2; exit 2; }
    campaign_id="$1"; resume="$(parse_resume "${2:-}")"; require_campaign_id "${campaign_id}"
    mode="${command_name%-pair}"
    preflight "${mode}"
    run_stage "${mode}" "${PROJECT_ROOT}/data/experiment_runs/kujiale_4x20_${campaign_id}/${mode}" "" "${resume}"
    ;;
  status|static-status|dynamic-status)
    [[ $# -eq 1 ]] || { usage >&2; exit 2; }
    campaign_id="$1"; require_campaign_id "${campaign_id}"; source_ros --require-workspace
    arguments=(python3 -m robot_experiments.kujiale_4x20_campaign --run-root "${PROJECT_ROOT}/data/experiment_runs/kujiale_4x20_${campaign_id}" --status)
    [[ "${command_name}" == "static-status" ]] && arguments+=(--scope static)
    [[ "${command_name}" == "dynamic-status" ]] && arguments+=(--scope dynamic)
    PYTHONPATH="${PROJECT_ROOT}/ros2_ws/src/robot_experiments${PYTHONPATH:+:${PYTHONPATH}}" "${arguments[@]}"
    ;;
  report|static-report|dynamic-report)
    [[ $# -ge 1 && $# -le 2 ]] || { usage >&2; exit 2; }
    campaign_id="$1"; replace_output="$(parse_replace "${2:-}")"; require_campaign_id "${campaign_id}"; source_ros --require-workspace
    run_root="${PROJECT_ROOT}/data/experiment_runs/kujiale_4x20_${campaign_id}"
    report_root="${PROJECT_ROOT}/data/reports/kujiale_4x20_${campaign_id}"
    scope="full"
    output_directory="${report_root}"
    if [[ "${command_name}" == "static-report" ]]; then
      scope="static"; output_directory="${report_root}/static_2x20"
    elif [[ "${command_name}" == "dynamic-report" ]]; then
      scope="dynamic"; output_directory="${report_root}/dynamic_2x20"
    fi
    if [[ -e "${output_directory}" && "${replace_output}" == false ]]; then
      if [[ "${scope}" != "full" ]]; then
        die "refusing to overwrite existing report: ${output_directory}"
      fi
      mapfile -t existing_entries < <(find "${output_directory}" -mindepth 1 -maxdepth 1 -printf '%f\n')
      for entry in "${existing_entries[@]}"; do
        [[ "${entry}" == "static_2x20" || "${entry}" == "dynamic_2x20" ]] \
          || die "refusing to overwrite existing report: ${output_directory}"
      done
    fi
    arguments=(python3 -m robot_experiments.kujiale_4x20_campaign --run-root "${run_root}" --scope "${scope}" --output-directory "${output_directory}")
    [[ "${replace_output}" == true ]] && arguments+=(--replace-output)
    PYTHONPATH="${PROJECT_ROOT}/ros2_ws/src/robot_experiments${PYTHONPATH:+:${PYTHONPATH}}" "${arguments[@]}"
    ;;
  *) usage >&2; exit 2 ;;
esac
