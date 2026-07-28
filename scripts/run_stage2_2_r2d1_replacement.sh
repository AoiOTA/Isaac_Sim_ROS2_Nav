#!/usr/bin/env bash
# Capture a new 20-record development dataset for the Stage 2.2-R2D1
# twist-versus-pose-delta audit. This is explicitly not a formal Gate.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

usage() {
  cat <<'USAGE'
usage: run_stage2_2_r2d1_replacement.sh [CAMPAIGN_ID] [--resume] [--skip-build] [--startup-timeout-sec SECONDS]

Records exactly ten static and ten dynamic development runs under
data/experiment_runs/stage2_2_r2d1_replacement_<CAMPAIGN_ID>. The scenarios
reuse the consumed G1 matrix but carry new R2D1 replacement scenario IDs.
USAGE
}

campaign_id="${CAMPAIGN_ID:-$(date +%Y%m%d-%H%M%S)}"
resume=false
build_workspace=true
startup_timeout_sec=900

if [[ $# -gt 0 && "${1}" != --* ]]; then
  campaign_id="$1"
  shift
fi
while (($#)); do
  case "$1" in
    --resume) resume=true ;;
    --skip-build) build_workspace=false ;;
    --startup-timeout-sec)
      shift
      [[ $# -gt 0 ]] || die "--startup-timeout-sec requires a positive integer"
      startup_timeout_sec="$1"
      ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; die "unknown argument: $1" ;;
  esac
  shift
done

[[ "${campaign_id}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] \
  || die "invalid CAMPAIGN_ID: ${campaign_id}"
[[ "${startup_timeout_sec}" =~ ^[1-9][0-9]*$ ]] \
  || die "startup timeout must be a positive integer"

run_root="${PROJECT_ROOT}/data/experiment_runs/stage2_2_r2d1_replacement_${campaign_id}"
control_root="${run_root}/orchestrator"
module3_head="$(git -C "${PROJECT_ROOT}" rev-parse HEAD)"
isaac_pid=""
ros_pid=""
active_mode=""

[[ -z "$(git -C "${PROJECT_ROOT}" status --porcelain --untracked-files=no)" ]] \
  || die "tracked Module3 worktree must be clean before replacement capture"
[[ ! -e "${run_root}" || "${resume}" == true ]] \
  || die "capture directory already exists: ${run_root}; use --resume or a new CAMPAIGN_ID"
mkdir -p "${control_root}"

pid_is_running() {
  local pid="$1"
  [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null
}

dedicated_process_group_for() {
  local pid="$1" process_group
  process_group="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
  [[ "${process_group}" == "${pid}" ]] && printf '%s' "${process_group}"
}

ros_launch_process_group_for() {
  local supervisor_pid="$1"
  ps -eo pid=,ppid=,pgid=,stat= | awk -v parent="${supervisor_pid}" '
    $2 == parent && $1 == $3 && $4 !~ /^Z/ { print $3; exit }
  '
}

stop_stage() {
  local process_group status=0
  [[ -n "${active_mode}" ]] || return 0
  log_info "stopping ${active_mode} Nav2 supervisor"
  if pid_is_running "${ros_pid}"; then
    process_group="$(ros_launch_process_group_for "${ros_pid}" || true)"
    if [[ "${process_group}" =~ ^[1-9][0-9]*$ ]]; then
      kill -INT -- "-${process_group}" 2>/dev/null || true
    else
      kill -INT "${ros_pid}" 2>/dev/null || true
    fi
    wait "${ros_pid}" || status=$?
  fi
  log_info "stopping ${active_mode} Isaac supervisor"
  if pid_is_running "${isaac_pid}"; then
    process_group="$(dedicated_process_group_for "${isaac_pid}" || true)"
    if [[ "${process_group}" =~ ^[1-9][0-9]*$ ]]; then
      kill -INT -- "-${process_group}" 2>/dev/null || true
    else
      kill -INT "${isaac_pid}" 2>/dev/null || true
    fi
    wait "${isaac_pid}" || status=$?
  fi
  isaac_pid=""
  ros_pid=""
  active_mode=""
  return 0
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM HUP
  stop_stage || true
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

wait_for_preflight() {
  local mode="$1" deadline log_file
  deadline=$((SECONDS + startup_timeout_sec))
  log_file="${control_root}/${mode}-preflight.log"
  while (( SECONDS < deadline )); do
    pid_is_running "${isaac_pid}" \
      || die "${mode} Isaac supervisor exited; inspect ${control_root}/${mode}-isaac.log"
    pid_is_running "${ros_pid}" \
      || die "${mode} Nav2 supervisor exited; inspect ${control_root}/${mode}-nav2.log"
    if "${SCRIPT_DIR}/run_kujiale_4x20.sh" preflight "${mode}" >"${log_file}" 2>&1; then
      cat "${log_file}"
      return 0
    fi
    sleep 5
  done
  tail -n 80 "${log_file}" >&2 || true
  die "${mode} stage did not satisfy preflight within ${startup_timeout_sec}s"
}

start_stage() {
  local mode="$1" nav2_profile
  [[ -z "${active_mode}" ]] || die "${active_mode} stage is still active"
  nav2_profile="stable"
  [[ "${mode}" == "dynamic" ]] && nav2_profile="dynamic_avoidance"
  log_info "starting ${mode} Isaac; log=${control_root}/${mode}-isaac.log"
  "${SCRIPT_DIR}/run_kujiale_4x20_isaac.sh" "${mode}" --headless \
    >"${control_root}/${mode}-isaac.log" 2>&1 &
  isaac_pid=$!
  log_info "starting ${mode} Nav2; log=${control_root}/${mode}-nav2.log"
  "${SCRIPT_DIR}/run_ros.sh" navigation \
    odometry_mode:=ideal spawn_pose_name:=long_route_start_g1 \
    nav2_profile:="${nav2_profile}" interactive:=false use_rviz:=false \
    >"${control_root}/${mode}-nav2.log" 2>&1 &
  ros_pid=$!
  active_mode="${mode}"
  wait_for_preflight "${mode}"
}

run_stage() {
  local mode="$1" nav2_profile scenario output
  scenario="${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/kujiale_stage2_2_r2d1_replacement_${mode}.yaml"
  output="${run_root}/${mode}"
  nav2_profile="stable"
  [[ "${mode}" == "dynamic" ]] && nav2_profile="dynamic_avoidance"
  require_file "${scenario}"
  [[ ! -e "${output}" || "${resume}" == true ]] \
    || die "refusing to overwrite ${output}"
  mkdir -p "${output}"
  "${SCRIPT_DIR}/run_experiment.sh" "${scenario}" "${output}" \
    "resume:=${resume}" "nav2_profile:=${nav2_profile}"
}

validate_stage() {
  local mode="$1"
  source_ros --require-workspace
  PYTHONPATH="${PROJECT_ROOT}/ros2_ws/src/robot_experiments${PYTHONPATH:+:${PYTHONPATH}}" \
    python3 - "${PROJECT_ROOT}" "${run_root}/${mode}" "${mode}" "${module3_head}" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

from robot_experiments.scenario import load_scenario


project = Path(sys.argv[1]).resolve()
evidence_root = Path(sys.argv[2]).resolve()
kind = sys.argv[3]
expected_head = sys.argv[4]
scenario = load_scenario(
    project
    / "ros2_ws/src/robot_experiments/config"
    / f"kujiale_stage2_2_r2d1_replacement_{kind}.yaml"
)
expected = {
    (
        kind,
        selection.seed,
        selection.condition_id,
        selection.appearance_profile_id,
    )
    for selection in scenario.run_matrix
}
required = (
    "run_manifest.json",
    "run_summary.json",
    "ground_truth.csv.gz",
    "odom.csv.gz",
    "checksums.sha256",
    "telemetry/metadata.yaml",
    "telemetry/telemetry_0.mcap",
)


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


found = {}
errors = []
for path in evidence_root.rglob("run_manifest.json"):
    if ".incomplete-" in path.parent.name:
        continue
    try:
        run = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        errors.append(f"{path}: unreadable manifest: {exc}")
        continue
    appearance = run.get("appearance", {})
    identity = (
        str(run.get("scenario_type", "")),
        int(run.get("random_seed", -1)),
        str(run.get("condition_id", "")),
        str(appearance.get("profile_id", "")) if isinstance(appearance, dict) else "",
    )
    if identity not in expected:
        continue
    if identity in found:
        errors.append(f"duplicate identity: {identity}")
        continue
    found[identity] = path.parent
    for relative in required:
        if not (path.parent / relative).is_file():
            errors.append(f"{identity}: missing {relative}")
    if run.get("scenario_id") != scenario.scenario_id:
        errors.append(f"{identity}: scenario_id mismatch")
    if run.get("map_version") != "warehouse_new":
        errors.append(f"{identity}: map_version mismatch")
    if run.get("posegraph_version") != "warehouse_new":
        errors.append(f"{identity}: posegraph_version mismatch")
    profile = "stable" if kind == "static" else "dynamic_avoidance"
    if run.get("nav2_profile") != profile or run.get("nav2_status") != 4:
        errors.append(f"{identity}: Nav2 contract mismatch")
    metrics = run.get("metrics", {})
    if not isinstance(metrics, dict) or float(
        metrics.get("ground_truth_path_length_m", 0.0)
    ) < 20.0:
        errors.append(f"{identity}: ground-truth path shorter than 20 m")
    provenance = run.get("provenance", {})
    if (
        not isinstance(provenance, dict)
        or provenance.get("git_head") != expected_head
        or provenance.get("git_dirty") is not False
    ):
        errors.append(f"{identity}: Module3 provenance mismatch")
    if kind == "static" and str(run.get("failure_reason", "")):
        errors.append(f"{identity}: static failure={run.get('failure_reason')!r}")
    if kind == "dynamic":
        contract = run.get("dynamic_runtime_contract", {})
        interaction = run.get("dynamic_interaction", {})
        if not isinstance(contract, dict) or contract.get("verified") is not True:
            errors.append(f"{identity}: dynamic runtime contract not verified")
        if not isinstance(interaction, dict) or interaction.get("complete") is not True:
            errors.append(f"{identity}: dynamic interaction incomplete")
    summary_path = path.parent / "run_summary.json"
    if summary_path.is_file():
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        if (
            summary.get("data_complete") is not True
            or summary.get("checksums_verified") is not True
        ):
            errors.append(f"{identity}: evidence summary incomplete")
    checksum_path = path.parent / "checksums.sha256"
    if checksum_path.is_file():
        for line in checksum_path.read_text(encoding="utf-8").splitlines():
            expected_digest, separator, relative = line.partition("  ")
            target = path.parent / relative
            if (
                not separator
                or not target.is_file()
                or digest(target) != expected_digest
            ):
                errors.append(f"{identity}: checksum mismatch for {relative!r}")

missing = sorted(expected.difference(found))
if missing:
    errors.append(f"missing identities: {missing}")
if len(found) != 10:
    errors.append(f"expected 10 identities, found {len(found)}")
if errors:
    raise SystemExit(
        "[isaac-nav] error: R2D1 replacement stage invalid: " + "; ".join(errors)
    )
print(
    f"[isaac-nav] R2D1 replacement {kind} stage valid: "
    f"records={len(found)} scenario={scenario.scenario_id}"
)
PY
}

if [[ "${build_workspace}" == true ]]; then
  log_info "building ROS workspace before R2D1 replacement capture"
  "${SCRIPT_DIR}/build_ros2.sh"
fi

log_info "starting R2D1 replacement development capture=${campaign_id}"
start_stage static
run_stage static
validate_stage static
stop_stage

start_stage dynamic
run_stage dynamic
validate_stage dynamic
stop_stage

log_info "R2D1 replacement capture complete: ${run_root}"
log_info "This dataset is development-audit-only and is not a formal Gate."
