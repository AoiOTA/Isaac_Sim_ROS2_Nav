#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

original_args=("$@")

usage() {
  cat <<'EOF'
usage: run_contact_ab_matrix.sh [--environment Warehouse|SimplePlane|all]
                                [--repeats N] --output-dir DIR

Run the committed skid-steer motion A/B protocol in strict serial order.
The default environment is all (SimplePlane, then Warehouse), and the
default repeat count is 3: 2 environments x 6 contact profiles x 3 = 36
independent Isaac processes. DIR must be empty and is never overwritten.
EOF
}

environment_selection="all"
repeats=3
output_dir=""
while (($#)); do
  case "$1" in
    --environment|--repeats|--output-dir)
      (($# >= 2)) || die "$1 requires a value"
      case "$1" in
        --environment) environment_selection="$2" ;;
        --repeats) repeats="$2" ;;
        --output-dir) output_dir="$2" ;;
      esac
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      die "unknown contact A/B argument: $1"
      ;;
  esac
done

case "${environment_selection}" in
  Warehouse) environments=(Warehouse) ;;
  SimplePlane) environments=(SimplePlane) ;;
  all) environments=(SimplePlane Warehouse) ;;
  *) die "--environment must be Warehouse, SimplePlane, or all" ;;
esac
[[ "${repeats}" =~ ^[1-9][0-9]*$ ]] \
  || die "--repeats must be a positive integer"
if ((${#repeats} > 3)); then
  die "--repeats must be an integer in [1, 100]"
fi
if ((10#${repeats} > 100)); then
  die "--repeats must be an integer in [1, 100]"
fi
repeats=$((10#${repeats}))
[[ -n "${output_dir}" ]] || die "--output-dir is required"
[[ "${output_dir}" != *$'\t'* && "${output_dir}" != *$'\n'* ]] \
  || die "--output-dir must not contain tabs or newlines"

profile_ids=(
  legacy_baseline
  threshold_corr_0p00025_offset_0p0004
  threshold_corr_0p025_offset_0p0004
  threshold_corr_0p00025_offset_0p04
  threshold_corr_0p025_offset_0p04
  explicit_material
)
profile_modes=(
  legacy_baseline
  threshold_only
  threshold_only
  threshold_only
  threshold_only
  explicit_material
)

motion_config="${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/motion_skid_steer_ab.yaml"
warehouse_config="${PROJECT_ROOT}/isaac_sim/configs/project.yaml"
simple_plane_config="${PROJECT_ROOT}/isaac_sim/configs/simple_plane.project.yaml"
physics_dir="${PROJECT_ROOT}/isaac_sim/configs/physics"
manifest=""
batch_git_commit=""
batch_git_branch=""
batch_motion_sha256=""
batch_warehouse_project_sha256=""
batch_simple_plane_project_sha256=""
batch_profile_hashes_json=""

robot_config=""
robot_asset=""
robot_config_sha256=""
robot_asset_sha256=""
warehouse_project_stage=""
warehouse_project_stage_sha256=""
warehouse_source_asset=""
warehouse_source_asset_sha256=""
simple_plane_project_stage=""
simple_plane_project_stage_sha256=""
simple_plane_source_asset=""
simple_plane_source_asset_sha256=""

declare -Ag locked_input_hashes=()
declare -ag locked_input_paths=()

require_clean_git() {
  local status
  status="$(
    git -C "${PROJECT_ROOT}" status --porcelain --untracked-files=normal
  )" || die "cannot inspect Git worktree"
  [[ -z "${status}" ]] || die "contact A/B requires a clean Git worktree"
}

require_tracked_input() {
  local path="$1"
  local relative="${path#"${PROJECT_ROOT}/"}"
  git -C "${PROJECT_ROOT}" ls-files --error-unmatch -- "${relative}" \
    >/dev/null 2>&1 \
    || die "contact A/B input is not committed: ${relative}"
}

sha256_file() {
  sha256sum "$1" | awk '{print $1}'
}

git_commit() {
  git -C "${PROJECT_ROOT}" rev-parse --verify 'HEAD^{commit}'
}

git_branch() {
  git -C "${PROJECT_ROOT}" symbolic-ref --quiet --short HEAD
}

project_runtime_contract() {
  local project_config="$1"
  python3 - "${PROJECT_ROOT}" "${ISAAC_ASSET_ROOT}" \
    "${project_config}" <<'PY'
from pathlib import Path
import sys

repository_root, asset_root, project_path = sys.argv[1:]
sys.path.insert(0, repository_root)
from isaac_sim.src.config import load_project_config

# Do not pass os.environ here: inherited ISAAC_NAV__* values are untrusted
# nested YAML overrides.  Only interpolation inputs are needed to resolve the
# committed project contract.
config = load_project_config(
    project_path,
    {"PROJECT_ROOT": repository_root, "ISAAC_ASSET_ROOT": asset_root},
)
values = (
    config.environment.identifier,
    str(config.files.robot),
    str(config.robot.asset_path),
    str(config.environment.project_stage),
    str(config.environment.source_asset),
)
if any("\t" in value or "\n" in value for value in values):
    raise SystemExit("runtime contract paths must not contain tabs or newlines")
print("\t".join(values))
PY
}

load_runtime_contracts() {
  local warehouse_contract simple_plane_contract
  local warehouse_id simple_plane_id simple_robot_config simple_robot_asset
  local runtime_input
  warehouse_contract="$(project_runtime_contract "${warehouse_config}")" \
    || die "cannot resolve the committed Warehouse runtime contract"
  simple_plane_contract="$(
    project_runtime_contract "${simple_plane_config}"
  )" || die "cannot resolve the committed SimplePlane runtime contract"
  IFS=$'\t' read -r \
    warehouse_id robot_config robot_asset \
    warehouse_project_stage warehouse_source_asset \
    <<<"${warehouse_contract}"
  IFS=$'\t' read -r \
    simple_plane_id simple_robot_config simple_robot_asset \
    simple_plane_project_stage simple_plane_source_asset \
    <<<"${simple_plane_contract}"
  [[ "${warehouse_id}" == Warehouse && "${simple_plane_id}" == SimplePlane ]] \
    || die "project runtime environment identifiers are not canonical"
  [[ "${simple_robot_config}" == "${robot_config}" \
        && "${simple_robot_asset}" == "${robot_asset}" ]] \
    || die "Warehouse and SimplePlane must use the same robot inputs"
  for runtime_input in \
      "${robot_config}" "${robot_asset}" \
      "${warehouse_project_stage}" "${warehouse_source_asset}" \
      "${simple_plane_project_stage}" "${simple_plane_source_asset}"; do
    require_file "${runtime_input}"
  done
  robot_config_sha256="$(sha256_file "${robot_config}")" \
    || die "cannot hash robot config"
  robot_asset_sha256="$(sha256_file "${robot_asset}")" \
    || die "cannot hash robot asset"
  warehouse_project_stage_sha256="$(
    sha256_file "${warehouse_project_stage}"
  )" || die "cannot hash Warehouse project stage"
  warehouse_source_asset_sha256="$(
    sha256_file "${warehouse_source_asset}"
  )" || die "cannot hash Warehouse source asset"
  simple_plane_project_stage_sha256="$(
    sha256_file "${simple_plane_project_stage}"
  )" || die "cannot hash SimplePlane project stage"
  simple_plane_source_asset_sha256="$(
    sha256_file "${simple_plane_source_asset}"
  )" || die "cannot hash SimplePlane source asset"
}

lock_batch_identity() {
  local profile_id profile_path
  batch_git_commit="$(git_commit)" \
    || die "contact A/B requires an attached Git HEAD"
  batch_git_branch="$(git_branch)" \
    || die "contact A/B requires an attached Git branch"
  locked_input_paths=(
    "${motion_config}"
    "${warehouse_config}"
    "${simple_plane_config}"
  )
  for profile_id in "${profile_ids[@]}"; do
    locked_input_paths+=("${physics_dir}/${profile_id}.yaml")
  done
  for profile_path in "${locked_input_paths[@]}"; do
    locked_input_hashes["${profile_path}"]="$(sha256_file "${profile_path}")" \
      || die "cannot hash contact A/B input: ${profile_path}"
  done
  batch_motion_sha256="${locked_input_hashes[${motion_config}]}"
  batch_warehouse_project_sha256="${locked_input_hashes[${warehouse_config}]}"
  batch_simple_plane_project_sha256="${locked_input_hashes[${simple_plane_config}]}"
  batch_profile_hashes_json="$(python3 - "${physics_dir}" \
    "${profile_ids[@]}" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

directory = Path(sys.argv[1])
profiles = {
    profile_id: hashlib.sha256(
        (directory / f"{profile_id}.yaml").read_bytes()
    ).hexdigest()
    for profile_id in sys.argv[2:]
}
print(json.dumps(profiles, sort_keys=True, separators=(",", ":")))
PY
  )" || die "cannot serialize locked contact profile hashes"
}

verify_batch_identity() {
  local phase="$1"
  local actual path status
  status="$(
    git -C "${PROJECT_ROOT}" status --porcelain --untracked-files=normal
  )" || {
    log_warn "cannot inspect Git worktree during ${phase}"
    return 1
  }
  [[ -z "${status}" ]] || {
    log_warn "Git worktree changed during contact A/B ${phase}"
    return 1
  }
  actual="$(git_commit)" || return 1
  [[ "${actual}" == "${batch_git_commit}" ]] || {
    log_warn "Git HEAD changed during contact A/B ${phase}"
    return 1
  }
  actual="$(git_branch)" || return 1
  [[ "${actual}" == "${batch_git_branch}" ]] || {
    log_warn "Git branch changed during contact A/B ${phase}"
    return 1
  }
  for path in "${locked_input_paths[@]}"; do
    actual="$(sha256_file "${path}")" || return 1
    [[ "${actual}" == "${locked_input_hashes[${path}]}" ]] || {
      log_warn "locked contact A/B input changed during ${phase}: ${path}"
      return 1
    }
  done
}

project_config_for_environment() {
  case "$1" in
    Warehouse) printf '%s\n' "${warehouse_config}" ;;
    SimplePlane) printf '%s\n' "${simple_plane_config}" ;;
    *) return 1 ;;
  esac
}

environment_slug() {
  case "$1" in
    Warehouse) printf 'warehouse\n' ;;
    SimplePlane) printf 'simple_plane\n' ;;
    *) return 1 ;;
  esac
}

tsv_safe() {
  local value="$1"
  value="${value//$'\t'/ }"
  value="${value//$'\n'/ }"
  printf '%s' "${value}"
}

bounded_positive_integer() {
  local value="$1"
  local maximum="$2"
  [[ "${value}" =~ ^[1-9][0-9]*$ ]] || return 1
  ((${#value} <= ${#maximum})) || return 1
  ((10#${value} <= maximum))
}

current_active=false
current_recorded=false
current_run_id=""
current_environment=""
current_profile_id=""
current_profile_mode=""
current_repeat=""
current_report=""
current_isaac_log=""
current_runner_log=""
current_project_config=""
current_project_sha256=""
current_profile_path=""
current_profile_sha256=""
current_project_stage=""
current_project_stage_sha256=""
current_source_asset=""
current_source_asset_sha256=""
current_started=""
current_failure_reason="batch_exit"

final_evidence_sha256() {
  local path="$1"
  local required="$2"
  local label="$3"
  if [[ ! -e "${path}" ]]; then
    [[ "${required}" == false ]] && return 0
    log_warn "required ${label} evidence is missing: ${path}"
    return 1
  fi
  if [[ ! -f "${path}" || -L "${path}" ]]; then
    log_warn "refusing unsafe ${label} evidence path: ${path}"
    return 1
  fi
  sha256_file "${path}"
}

append_manifest_line_atomically() {
  local row="$1"
  local temporary
  temporary="$(mktemp "${manifest}.tmp.XXXXXX")" || return 1
  if ! cp -- "${manifest}" "${temporary}" \
      || ! printf '%s\n' "${row}" >>"${temporary}" \
      || ! mv -f -- "${temporary}" "${manifest}"; then
    rm -f -- "${temporary}"
    return 1
  fi
}

append_current_manifest() {
  local status="$1"
  local detail="$2"
  local completed report_sha256 isaac_log_sha256 runner_log_sha256 row
  local evidence_required=false
  [[ "${status}" == success ]] && evidence_required=true
  completed="$(date -u +%Y-%m-%dT%H:%M:%SZ)" || return 1
  report_sha256="$(
    final_evidence_sha256 \
      "${current_report}" "${evidence_required}" "motion report"
  )" || return 1
  # A log digest is final only after its writer has stopped.  A live,
  # unauthenticated child is intentionally left unsignalled and therefore
  # cannot be represented by a misleading "final" digest.
  if [[ -n "${owned_pids[isaac]:-}" ]] \
      && runtime_process_is_running "${owned_pids[isaac]}"; then
    isaac_log_sha256=""
  else
    isaac_log_sha256="$(
      final_evidence_sha256 \
        "${current_isaac_log}" "${evidence_required}" "Isaac log"
    )" || return 1
  fi
  if [[ -n "${owned_pids[motion_baseline]:-}" ]] \
      && runtime_process_is_running "${owned_pids[motion_baseline]}"; then
    runner_log_sha256=""
  else
    runner_log_sha256="$(
      final_evidence_sha256 \
        "${current_runner_log}" "${evidence_required}" "runner log"
    )" || return 1
  fi
  if ! printf -v row \
    '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s' \
    "$(tsv_safe "${current_run_id}")" \
    "$(tsv_safe "${current_environment}")" \
    "$(tsv_safe "${current_profile_id}")" \
    "$(tsv_safe "${current_profile_mode}")" \
    "${current_repeat}" \
    "${status}" \
    "$(tsv_safe "${detail}")" \
    "$(tsv_safe "${current_report}")" \
    "${report_sha256}" \
    "$(tsv_safe "${current_isaac_log}")" \
    "${isaac_log_sha256}" \
    "$(tsv_safe "${current_runner_log}")" \
    "${runner_log_sha256}" \
    "${batch_git_commit}" \
    "$(tsv_safe "${batch_git_branch}")" \
    "$(tsv_safe "${motion_config}")" \
    "${batch_motion_sha256}" \
    "$(tsv_safe "${warehouse_config}")" \
    "${batch_warehouse_project_sha256}" \
    "$(tsv_safe "${simple_plane_config}")" \
    "${batch_simple_plane_project_sha256}" \
    "$(tsv_safe "${current_project_config}")" \
    "${current_project_sha256}" \
    "$(tsv_safe "${current_profile_path}")" \
    "${current_profile_sha256}" \
    "$(tsv_safe "${batch_profile_hashes_json}")" \
    "$(tsv_safe "${current_project_stage}")" \
    "${current_project_stage_sha256}" \
    "$(tsv_safe "${current_source_asset}")" \
    "${current_source_asset_sha256}" \
    "${current_started}/${completed}"; then
    return 1
  fi
  append_manifest_line_atomically "${row}" || return 1
  current_recorded=true
}

declare -Ag owned_pids=()
declare -Ag owned_groups=()
declare -Ag owned_start_ticks=()

component_identity_is_current() {
  local component="$1"
  local expected_pid="$2"
  local expected_group="$3"
  local expected_start="$4"
  local registered pid_file recorded_pid recorded_group recorded_start
  registered="$(
    runtime_registered_process_group \
      "${component}" "${ISAAC_NAV_SESSION_ID}" || true
  )"
  [[ "${registered}" == "${expected_group}" ]] || return 1
  pid_file="$(runtime_pid_file "${component}")" || return 1
  recorded_pid="$(runtime_metadata_value "${pid_file}" pid)" || return 1
  recorded_group="$(
    runtime_metadata_value "${pid_file}" process_group
  )" || return 1
  recorded_start="$(
    runtime_metadata_value "${pid_file}" leader_start_ticks
  )" || return 1
  [[ "${recorded_pid}" == "${expected_pid}" \
        && "${recorded_group}" == "${expected_group}" \
        && "${recorded_start}" == "${expected_start}" ]] || return 1
  runtime_process_group_is_owned_by_session \
    "${expected_group}" "${PROJECT_ROOT}" "${ISAAC_NAV_SESSION_ID}"
}

wait_for_component_registration() {
  local component="$1"
  local pid="$2"
  local timeout_seconds="${ISAAC_NAV_CONTACT_AB_REGISTER_TIMEOUT_SECONDS:-20}"
  local deadline group pid_file start_ticks
  bounded_positive_integer "${timeout_seconds}" 120 || {
    log_warn "registration timeout must be an integer in [1, 120]"
    return 1
  }
  deadline=$((SECONDS + timeout_seconds))
  while ((SECONDS < deadline)); do
    group="$(
      runtime_registered_process_group \
        "${component}" "${ISAAC_NAV_SESSION_ID}" || true
    )"
    if [[ "${group}" == "${pid}" ]]; then
      pid_file="$(runtime_pid_file "${component}")"
      start_ticks="$(
        runtime_metadata_value "${pid_file}" leader_start_ticks || true
      )"
      if [[ "${start_ticks}" =~ ^[0-9]+$ ]] \
          && component_identity_is_current \
            "${component}" "${pid}" "${group}" "${start_ticks}"; then
        owned_groups["${component}"]="${group}"
        owned_start_ticks["${component}"]="${start_ticks}"
        return 0
      fi
    fi
    runtime_process_is_running "${pid}" || return 1
    sleep 0.1
  done
  return 1
}

wait_for_group_exit() {
  local process_group="$1"
  local checks="$2"
  local index
  for ((index = 0; index < checks; index++)); do
    runtime_process_group_is_running "${process_group}" || return 0
    sleep 0.1
  done
  ! runtime_process_group_is_running "${process_group}"
}

stop_owned_component() {
  local component="$1"
  local label="$2"
  local pid="${owned_pids[${component}]:-}"
  local group="${owned_groups[${component}]:-}"
  local start_ticks="${owned_start_ticks[${component}]:-}"
  local registered pid_file
  local int_checks="${ISAAC_NAV_CONTACT_AB_INT_CHECKS:-150}"
  local term_checks="${ISAAC_NAV_CONTACT_AB_TERM_CHECKS:-50}"
  local kill_checks="${ISAAC_NAV_CONTACT_AB_KILL_CHECKS:-50}"
  [[ -n "${pid}" ]] || return 0
  if ! bounded_positive_integer "${int_checks}" 600 \
      || ! bounded_positive_integer "${term_checks}" 600 \
      || ! bounded_positive_integer "${kill_checks}" 600; then
    log_warn "refusing to stop ${label}: signal wait checks must be integers in [1, 600]"
    return 1
  fi

  if [[ -z "${group}" ]]; then
    registered="$(
      runtime_registered_process_group \
        "${component}" "${ISAAC_NAV_SESSION_ID}" || true
    )"
    if [[ "${registered}" == "${pid}" ]]; then
      group="${registered}"
      pid_file="$(runtime_pid_file "${component}")"
      start_ticks="$(
        runtime_metadata_value "${pid_file}" leader_start_ticks || true
      )"
    fi
  fi

  if [[ -z "${group}" ]]; then
    if runtime_process_is_running "${pid}" \
        || runtime_process_group_is_running "${pid}"; then
      log_warn "refusing to stop ${label}: live child never registered an authenticated process group"
      return 1
    fi
    wait "${pid}" 2>/dev/null || true
    remove_runtime_session_metadata \
      "${component}" "${ISAAC_NAV_SESSION_ID}" || true
    unset 'owned_pids['"${component}"']' \
      'owned_groups['"${component}"']' \
      'owned_start_ticks['"${component}"']'
    return 0
  fi
  if ! runtime_process_group_is_running "${group}"; then
    if runtime_process_is_running "${pid}"; then
      log_warn "refusing to stop ${label}: registered process group disappeared while its leader is live"
      return 1
    fi
    wait "${pid}" 2>/dev/null || true
    remove_runtime_session_metadata \
      "${component}" "${ISAAC_NAV_SESSION_ID}" || true
    unset 'owned_pids['"${component}"']' \
      'owned_groups['"${component}"']' \
      'owned_start_ticks['"${component}"']'
    return 0
  fi
  if [[ ! "${start_ticks}" =~ ^[0-9]+$ ]] \
      || ! component_identity_is_current \
        "${component}" "${pid}" "${group}" "${start_ticks}"; then
    log_warn "refusing to stop ${label}: authenticated process identity changed"
    return 1
  fi

  log_info "stopping ${label} process group ${group} with SIGINT"
  kill -INT -- "-${group}" 2>/dev/null || true
  if ! wait_for_group_exit "${group}" "${int_checks}"; then
    component_identity_is_current \
      "${component}" "${pid}" "${group}" "${start_ticks}" || {
        log_warn "refusing SIGTERM for ${label}: process identity changed"
        return 1
      }
    log_warn "${label} ignored SIGINT; sending SIGTERM"
    kill -TERM -- "-${group}" 2>/dev/null || true
    if ! wait_for_group_exit "${group}" "${term_checks}"; then
      component_identity_is_current \
        "${component}" "${pid}" "${group}" "${start_ticks}" || {
          log_warn "refusing SIGKILL for ${label}: process identity changed"
          return 1
        }
      log_warn "${label} ignored SIGTERM; sending SIGKILL"
      kill -KILL -- "-${group}" 2>/dev/null || true
      wait_for_group_exit "${group}" "${kill_checks}" || {
        log_warn "${label} process group ${group} survived SIGKILL"
        return 1
      }
    fi
  fi
  wait "${pid}" 2>/dev/null || true
  remove_runtime_session_metadata \
    "${component}" "${ISAAC_NAV_SESSION_ID}" || return 1
  unset 'owned_pids['"${component}"']' \
    'owned_groups['"${component}"']' \
    'owned_start_ticks['"${component}"']'
}

cleanup_started=false
cleanup_batch() {
  local exit_status=$?
  local cleanup_status=0
  [[ "${cleanup_started}" == false ]] || return
  cleanup_started=true
  trap - EXIT INT TERM HUP
  set +e
  stop_owned_component motion_baseline "motion baseline" || cleanup_status=1
  stop_owned_component isaac "Isaac Sim" || cleanup_status=1
  if [[ "${current_active}" == true \
        && "${current_recorded}" == false \
        && -n "${manifest}" ]]; then
    append_current_manifest failure "${current_failure_reason}" \
      || cleanup_status=1
  fi
  release_instance_lock contact_ab_matrix || cleanup_status=1
  if ((exit_status == 0 && cleanup_status != 0)); then
    exit_status=1
  fi
  exit "${exit_status}"
}

trap cleanup_batch EXIT
trap 'current_failure_reason="signal_INT"; exit 130' INT
trap 'current_failure_reason="signal_TERM"; exit 143' TERM
trap 'current_failure_reason="signal_HUP"; exit 129' HUP

ros_parameter() {
  local name="$1"
  timeout 5 ros2 param get \
    --no-daemon --spin-time 0.2 --hide-type --timeout 1 \
    /isaac_navigation_sim "${name}" 2>/dev/null
}

ros_parameter_boolean_matches() {
  local actual="$1"
  local expected="$2"
  case "${expected}" in
    true) [[ "${actual}" == True ]] ;;
    false) [[ "${actual}" == False ]] ;;
    *) return 1 ;;
  esac
}

clear_inherited_config_overrides() {
  local variable
  while IFS= read -r variable; do
    [[ "${variable}" == ISAAC_NAV__* ]] || continue
    unset "${variable}"
  done < <(compgen -e)
}

contact_readiness_matches() {
  local payload="$1"
  local payload_sha256="$2"
  local expected_profile_id="$3"
  local expected_profile_mode="$4"
  local expected_profile_path="$5"
  local expected_profile_sha256="$6"
  python3 - \
    "${payload}" "${payload_sha256}" \
    "${expected_profile_id}" "${expected_profile_mode}" \
    "${expected_profile_path}" "${expected_profile_sha256}" <<'PY'
import hashlib
import json
import sys

payload, payload_sha256, profile_id, profile_mode, profile_path, profile_sha256 = sys.argv[1:]
try:
    contact = json.loads(payload)
    canonical = json.dumps(
        contact, sort_keys=True, separators=(",", ":"), allow_nan=False
    )
except (TypeError, ValueError):
    raise SystemExit(1)
if canonical != payload:
    raise SystemExit(1)
if hashlib.sha256(payload.encode("utf-8")).hexdigest() != payload_sha256:
    raise SystemExit(1)
expected = {
    "profile_id": profile_id,
    "profile_mode": profile_mode,
    "profile_path": profile_path,
    "profile_sha256": profile_sha256,
    "stage_usd_readback_verified": True,
}
if any(contact.get(key) != value for key, value in expected.items()):
    raise SystemExit(1)
PY
}

wait_for_isaac_ready() {
  local expected_environment="$1"
  local profile_id="$2"
  local profile_mode="$3"
  local profile_path="$4"
  local profile_sha256="$5"
  local timeout_seconds="${ISAAC_NAV_CONTACT_AB_READY_TIMEOUT_SECONDS:-180}"
  local deadline schema environment_id contact_json contact_sha256
  local actual_robot_config actual_robot_config_sha256
  local actual_robot_asset actual_robot_asset_sha256
  local solver_position solver_velocity solver_readback physics_hz
  local navigation_mode odometry_mode
  local project_stage project_stage_sha256 source_asset source_asset_sha256
  local provenance_commit provenance_branch provenance_dirty
  bounded_positive_integer "${timeout_seconds}" 900 || {
    log_warn "readiness timeout must be an integer in [1, 900]"
    return 1
  }
  deadline=$((SECONDS + timeout_seconds))
  while ((SECONDS < deadline)); do
    component_identity_is_current \
      isaac "${owned_pids[isaac]}" "${owned_groups[isaac]}" \
      "${owned_start_ticks[isaac]}" || return 1
    schema="$(ros_parameter runtime_provenance.schema_version || true)"
    if [[ "${schema}" != 3 ]]; then
      sleep 0.2
      continue
    fi
    environment_id="$(
      ros_parameter runtime_provenance.environment.id || true
    )"
    contact_json="$(ros_parameter runtime_provenance.contact.json || true)"
    contact_sha256="$(
      ros_parameter runtime_provenance.contact.sha256 || true
    )"
    actual_robot_config="$(
      ros_parameter runtime_provenance.robot.config.path || true
    )"
    actual_robot_config_sha256="$(
      ros_parameter runtime_provenance.robot.config.sha256 || true
    )"
    actual_robot_asset="$(
      ros_parameter runtime_provenance.robot.asset.path || true
    )"
    actual_robot_asset_sha256="$(
      ros_parameter runtime_provenance.robot.asset.sha256 || true
    )"
    solver_position="$(
      ros_parameter runtime_provenance.robot.solver.position_iterations \
        || true
    )"
    solver_velocity="$(
      ros_parameter runtime_provenance.robot.solver.velocity_iterations \
        || true
    )"
    solver_readback="$(
      ros_parameter \
        runtime_provenance.robot.solver.stage_articulation_usd_readback_verified \
        || true
    )"
    project_stage="$(
      ros_parameter runtime_provenance.environment.project_stage.path || true
    )"
    project_stage_sha256="$(
      ros_parameter runtime_provenance.environment.project_stage.sha256 || true
    )"
    source_asset="$(
      ros_parameter runtime_provenance.environment.source_asset.path || true
    )"
    source_asset_sha256="$(
      ros_parameter runtime_provenance.environment.source_asset.sha256 || true
    )"
    navigation_mode="$(
      ros_parameter runtime_provenance.simulation.navigation_mode || true
    )"
    odometry_mode="$(
      ros_parameter runtime_provenance.simulation.odometry_mode || true
    )"
    physics_hz="$(
      ros_parameter runtime_provenance.simulation.physics_hz || true
    )"
    provenance_commit="$(
      ros_parameter runtime_provenance.git.commit || true
    )"
    provenance_branch="$(
      ros_parameter runtime_provenance.git.branch || true
    )"
    provenance_dirty="$(
      ros_parameter runtime_provenance.git.dirty || true
    )"
    if [[ "${environment_id}" == "${expected_environment}" \
          && "${actual_robot_config}" == "${robot_config}" \
          && "${actual_robot_config_sha256}" == "${robot_config_sha256}" \
          && "${actual_robot_asset}" == "${robot_asset}" \
          && "${actual_robot_asset_sha256}" == "${robot_asset_sha256}" \
          && "${solver_position}" == 32 \
          && "${solver_velocity}" == 4 \
          && "${project_stage}" == "${current_project_stage}" \
          && "${project_stage_sha256}" == "${current_project_stage_sha256}" \
          && "${source_asset}" == "${current_source_asset}" \
          && "${source_asset_sha256}" == "${current_source_asset_sha256}" \
          && "${navigation_mode}" == mapping \
          && "${odometry_mode}" == ideal \
          && ( "${physics_hz}" == 60 || "${physics_hz}" == 60.0 ) \
          && "${provenance_commit}" == "${batch_git_commit}" \
          && "${provenance_branch}" == "${batch_git_branch}" ]] \
        && ros_parameter_boolean_matches "${solver_readback}" true \
        && ros_parameter_boolean_matches "${provenance_dirty}" false \
        && contact_readiness_matches \
          "${contact_json}" "${contact_sha256}" \
          "${profile_id}" "${profile_mode}" \
          "${profile_path}" "${profile_sha256}"; then
      return 0
    fi
    sleep 0.2
  done
  return 1
}

launch_isaac() {
  local project_config="$1"
  local profile_path="$2"
  local log_path="$3"
  (
    close_instance_lock_fds_for_child
    unset ISAAC_NAV_DEDICATED_PROCESS_GROUP
    clear_inherited_config_overrides
    export ISAAC_NAV_PROJECT_CONFIG="${project_config}"
    export ISAAC_NAV__FILES__CONTACT_PROFILE="${profile_path}"
    # Schema-v3 provenance verifies mapping/ideal/60 Hz below.  It does not
    # currently expose headless, pacing, or camera selection, so those remain
    # a pinned CLI launch contract and are not misreported as provenance locks.
    exec "${SCRIPT_DIR}/run_isaac.sh" \
      --headless --pacing-mode unbounded \
      --navigation-mode mapping --mode ideal --camera-profile off
  ) >"${log_path}" 2>&1 &
  owned_pids[isaac]=$!
  wait_for_component_registration isaac "${owned_pids[isaac]}"
}

launch_motion_runner() {
  local environment_id="$1"
  local report_path="$2"
  local log_path="$3"
  (
    close_instance_lock_fds_for_child
    unset ISAAC_NAV_DEDICATED_PROCESS_GROUP
    clear_inherited_config_overrides
    exec "${SCRIPT_DIR}/run_motion_baseline.sh" \
      --environment "${environment_id}" \
      --odometry-mode ideal \
      --config "${motion_config}" \
      --output "${report_path}"
  ) >"${log_path}" 2>&1 &
  owned_pids[motion_baseline]=$!
  wait_for_component_registration \
    motion_baseline "${owned_pids[motion_baseline]}"
}

verify_motion_report() {
  local report_path="$1"
  local environment_id="$2"
  local profile_id="$3"
  local profile_mode="$4"
  local profile_path="$5"
  local profile_sha256="$6"
  local motion_sha256="$7"
  python3 - \
    "${PROJECT_ROOT}" "${report_path}" "${environment_id}" \
    "${profile_id}" "${profile_mode}" \
    "${profile_path}" "${profile_sha256}" "${motion_sha256}" \
    "${batch_git_commit}" "${batch_git_branch}" \
    "${robot_config}" "${robot_config_sha256}" \
    "${robot_asset}" "${robot_asset_sha256}" \
    "${current_project_stage}" "${current_project_stage_sha256}" \
    "${current_source_asset}" "${current_source_asset_sha256}" <<'PY'
import json
from pathlib import Path
import sys

repository_root = Path(sys.argv[1])
path = Path(sys.argv[2])
environment_id, profile_id, profile_mode = sys.argv[3:6]
profile_path, profile_sha256, motion_sha256 = sys.argv[6:9]
git_commit, git_branch = sys.argv[9:11]
robot_config, robot_config_sha256 = sys.argv[11:13]
robot_asset, robot_asset_sha256 = sys.argv[13:15]
project_stage, project_stage_sha256 = sys.argv[15:17]
source_asset, source_asset_sha256 = sys.argv[17:19]
if not path.is_file():
    raise SystemExit("motion report is missing")
try:
    report = json.loads(path.read_text(encoding="utf-8"))
except (OSError, ValueError) as exc:
    raise SystemExit(f"motion report is not valid JSON: {exc}")

# Load the current workspace source deliberately.  The installed ROS package
# may predate this batch runner during development, while the committed source
# is one of the inputs protected by the clean Git/HEAD lock.
source_root = repository_root / "ros2_ws/src/robot_experiments"
sys.path.insert(0, str(source_root))
try:
    from robot_experiments.contact_ab_analysis import analyse_contact_ab
except ImportError as exc:
    raise SystemExit(f"strict contact A/B validator is unavailable: {exc}")
try:
    analysis = analyse_contact_ab(
        [path],
        0.098,
        min_repeats=1,
        expected_environments=(environment_id,),
        expected_profiles=(profile_id,),
    )
except Exception as exc:
    raise SystemExit(f"strict contact A/B report validation failed: {exc}")
if analysis.get("analysis_valid") is not True:
    raise SystemExit("strict contact A/B report validation excluded the report")
if report.get("result") != "success":
    raise SystemExit("motion report result != \"success\"")
if report.get("environment_id") != environment_id:
    raise SystemExit("motion report environment mismatch")
if report.get("odometry_mode") != "ideal":
    raise SystemExit("motion report odometry mode mismatch")
if report.get("config_sha256") != motion_sha256:
    raise SystemExit("motion config SHA256 mismatch")
provenance = report.get("runtime_provenance", {})
contact = provenance.get("contact", {})
environment = provenance.get("environment", {})
robot = provenance.get("robot", {})
simulation = provenance.get("simulation", {})
git = provenance.get("git", {})
if environment.get("id") != environment_id:
    raise SystemExit("runtime provenance environment mismatch")
if git.get("dirty") is not False:
    raise SystemExit("runtime_provenance.git.dirty must be false")
if git.get("commit") != git_commit or git.get("branch") != git_branch:
    raise SystemExit("runtime provenance Git identity mismatch")
expected_robot_inputs = {
    "config": {"path": robot_config, "sha256": robot_config_sha256},
    "asset": {"path": robot_asset, "sha256": robot_asset_sha256},
}
if any(robot.get(name) != value for name, value in expected_robot_inputs.items()):
    raise SystemExit("runtime provenance robot input mismatch")
solver = robot.get("solver", {})
if (
    solver.get("position_iterations") != 32
    or solver.get("velocity_iterations") != 4
    or solver.get("stage_articulation_usd_readback_verified") is not True
):
    raise SystemExit("runtime provenance solver mismatch")
expected_environment_inputs = {
    "project_stage": {"path": project_stage, "sha256": project_stage_sha256},
    "source_asset": {"path": source_asset, "sha256": source_asset_sha256},
}
if any(
    environment.get(name) != value
    for name, value in expected_environment_inputs.items()
):
    raise SystemExit("runtime provenance environment input mismatch")
if (
    simulation.get("navigation_mode") != "mapping"
    or simulation.get("odometry_mode") != "ideal"
    or simulation.get("physics_hz") != 60
):
    raise SystemExit("runtime provenance simulation contract mismatch")
expected_contact = {
    "profile_id": profile_id,
    "profile_mode": profile_mode,
    "profile_path": profile_path,
    "profile_sha256": profile_sha256,
    "stage_usd_readback_verified": True,
}
if any(contact.get(key) != value for key, value in expected_contact.items()):
    raise SystemExit("runtime provenance contact profile mismatch")
PY
}

run_one_condition() {
  local sequence="$1"
  local environment_id="$2"
  local profile_id="$3"
  local profile_mode="$4"
  local repeat="$5"
  local slug project_config profile_path runner_status

  slug="$(environment_slug "${environment_id}")" || {
    current_failure_reason="environment_slug_failed"
    return 1
  }
  project_config="$(
    project_config_for_environment "${environment_id}"
  )" || {
    current_failure_reason="project_config_resolution_failed"
    return 1
  }
  profile_path="${physics_dir}/${profile_id}.yaml"
  if ! printf -v current_run_id '%03d_%s_%s_r%02d' \
      "${sequence}" "${slug}" "${profile_id}" "${repeat}"; then
    current_failure_reason="run_id_format_failed"
    return 1
  fi
  current_environment="${environment_id}"
  current_profile_id="${profile_id}"
  current_profile_mode="${profile_mode}"
  current_repeat="${repeat}"
  current_report="${output_dir}/reports/${current_run_id}.json"
  current_isaac_log="${output_dir}/logs/${current_run_id}.isaac.log"
  current_runner_log="${output_dir}/logs/${current_run_id}.runner.log"
  current_project_config="${project_config}"
  current_project_sha256="${locked_input_hashes[${project_config}]}"
  current_profile_path="${profile_path}"
  current_profile_sha256="${locked_input_hashes[${profile_path}]}"
  case "${environment_id}" in
    Warehouse)
      current_project_stage="${warehouse_project_stage}"
      current_project_stage_sha256="${warehouse_project_stage_sha256}"
      current_source_asset="${warehouse_source_asset}"
      current_source_asset_sha256="${warehouse_source_asset_sha256}"
      ;;
    SimplePlane)
      current_project_stage="${simple_plane_project_stage}"
      current_project_stage_sha256="${simple_plane_project_stage_sha256}"
      current_source_asset="${simple_plane_source_asset}"
      current_source_asset_sha256="${simple_plane_source_asset_sha256}"
      ;;
    *)
      current_failure_reason="unknown_environment_contract"
      return 1
      ;;
  esac
  current_started="$(date -u +%Y-%m-%dT%H:%M:%SZ)" || {
    current_failure_reason="start_timestamp_failed"
    return 1
  }
  current_failure_reason="condition_failed"
  current_recorded=false
  current_active=true

  if ! verify_batch_identity "before ${current_run_id}"; then
    current_failure_reason="batch_identity_changed_before_run"
    return 1
  fi
  log_info "contact A/B ${current_run_id}: starting Isaac"
  if ! launch_isaac \
      "${project_config}" "${profile_path}" "${current_isaac_log}"; then
    current_failure_reason="isaac_registration_failed"
    return 1
  fi
  if ! wait_for_isaac_ready \
      "${environment_id}" "${profile_id}" "${profile_mode}" \
      "${profile_path}" "${current_profile_sha256}"; then
    current_failure_reason="isaac_readiness_failed"
    return 1
  fi

  log_info "contact A/B ${current_run_id}: running committed motion profile"
  if ! launch_motion_runner \
      "${environment_id}" "${current_report}" "${current_runner_log}"; then
    current_failure_reason="motion_runner_registration_failed"
    return 1
  fi
  if wait "${owned_pids[motion_baseline]}"; then
    runner_status=0
  else
    runner_status=$?
  fi
  if ! stop_owned_component motion_baseline "motion baseline"; then
    current_failure_reason="motion_runner_cleanup_failed"
    return 1
  fi
  if ((runner_status != 0)); then
    current_failure_reason="motion_runner_exit_${runner_status}"
    return 1
  fi

  if ! stop_owned_component isaac "Isaac Sim"; then
    current_failure_reason="isaac_cleanup_failed"
    return 1
  fi
  if ! verify_batch_identity "after ${current_run_id}"; then
    current_failure_reason="batch_identity_changed_after_run"
    return 1
  fi
  if ! verify_motion_report \
      "${current_report}" "${environment_id}" \
      "${profile_id}" "${profile_mode}" \
      "${profile_path}" "${current_profile_sha256}" \
      "${batch_motion_sha256}"; then
    current_failure_reason="motion_report_verification_failed"
    return 1
  fi
  if ! verify_batch_identity "after report verification ${current_run_id}"; then
    current_failure_reason="batch_identity_changed_during_report_verification"
    return 1
  fi

  if ! append_current_manifest success complete; then
    current_failure_reason="manifest_append_failed"
    return 1
  fi
  current_active=false
  log_info "contact A/B ${current_run_id}: complete"
}

reject_symlink_path_components() {
  local raw_path="$1"
  python3 - "${raw_path}" <<'PY'
import os
from pathlib import Path
import sys

path = Path(sys.argv[1])
if not path.is_absolute():
    raise SystemExit("output path inspection requires an absolute raw path")
cursor = Path(path.anchor)
for component in path.parts[1:]:
    if component in {"", "."}:
        continue
    if component == "..":
        cursor = cursor.parent
        continue
    cursor /= component
    if os.path.lexists(cursor) and cursor.is_symlink():
        raise SystemExit(f"--output-dir path contains a symlink: {cursor}")
PY
}

prepare_output_directory() {
  local raw_output_dir="${output_dir}"
  local existing_entry relative_output verified_output_dir
  if [[ "${raw_output_dir}" != /* ]]; then
    raw_output_dir="${PROJECT_ROOT}/${raw_output_dir}"
  fi
  reject_symlink_path_components "${raw_output_dir}" \
    || die "--output-dir and its existing ancestors must not be symlinks"
  output_dir="$(realpath -m "${raw_output_dir}")" \
    || die "cannot normalize --output-dir"
  [[ "${output_dir}" != "${PROJECT_ROOT}" ]] \
    || die "--output-dir must not be the repository root"
  if [[ "${output_dir}" == "${PROJECT_ROOT}/"* ]]; then
    relative_output="${output_dir#"${PROJECT_ROOT}/"}"
    git -C "${PROJECT_ROOT}" check-ignore -q -- "${relative_output}" \
      || die "an in-repository --output-dir must be ignored by Git"
  fi
  if [[ -e "${output_dir}" ]]; then
    [[ -d "${output_dir}" ]] \
      || die "--output-dir exists and is not a directory"
    existing_entry="$(
      find "${output_dir}" -mindepth 1 -maxdepth 1 -print -quit
    )" || die "cannot inspect existing --output-dir"
    [[ -z "${existing_entry}" ]] \
      || die "--output-dir must be empty; existing evidence is never overwritten"
  else
    mkdir -p -- "${output_dir}" || die "cannot create --output-dir"
  fi
  reject_symlink_path_components "${raw_output_dir}" \
    || die "--output-dir acquired a symlink while it was prepared"
  verified_output_dir="$(realpath -m "${raw_output_dir}")" \
    || die "cannot recheck --output-dir"
  [[ "${verified_output_dir}" == "${output_dir}" ]] \
    || die "--output-dir changed while it was prepared"
  mkdir -p -- "${output_dir}/logs" "${output_dir}/reports" \
    || die "cannot create contact A/B evidence directories"
}

initialize_manifest() {
  local header temporary
  manifest="${output_dir}/manifest.tsv"
  header=$'run_id\tenvironment\tprofile_id\tprofile_mode\trepeat\tstatus\tdetail\treport\treport_sha256\tisaac_log\tisaac_log_sha256\trunner_log\trunner_log_sha256\tgit_commit\tgit_branch\tmotion_config\tmotion_config_sha256\twarehouse_project_config\twarehouse_project_config_sha256\tsimple_plane_project_config\tsimple_plane_project_config_sha256\tselected_project_config\tselected_project_config_sha256\tprofile_path\tprofile_sha256\tall_profile_hashes_json\tenvironment_project_stage\tenvironment_project_stage_sha256\tenvironment_source_asset\tenvironment_source_asset_sha256\tstarted_at_utc/completed_at_utc'
  temporary="$(mktemp "${manifest}.tmp.XXXXXX")" \
    || die "cannot create manifest temporary file"
  if ! printf '%s\n' "${header}" >"${temporary}" \
      || ! mv -f -- "${temporary}" "${manifest}"; then
    rm -f -- "${temporary}"
    die "cannot initialize contact A/B manifest"
  fi
}

frozen_manifest_sha256=""
analysis_path=""
analysis_sha256=""
batch_summary_path=""

freeze_manifest() {
  [[ -f "${manifest}" && ! -L "${manifest}" ]] || {
    log_warn "refusing to freeze an unsafe manifest path: ${manifest}"
    return 1
  }
  chmod 0444 -- "${manifest}" || return 1
  frozen_manifest_sha256="$(sha256_file "${manifest}")" || return 1
}

finalize_contact_analysis() {
  analysis_path="${output_dir}/analysis.json"
  [[ ! -e "${analysis_path}" && ! -L "${analysis_path}" ]] || {
    log_warn "refusing to overwrite aggregate analysis: ${analysis_path}"
    return 1
  }
  python3 - \
    "${PROJECT_ROOT}" "${manifest}" "${analysis_path}" \
    "${environment_selection}" "${repeats}" \
    "${expected_conditions}" "${expected_groups}" <<'PY'
import csv
import hashlib
from pathlib import Path
import sys

(
    repository_root,
    manifest_name,
    output_name,
    environment_selection,
    repeats_text,
    expected_runs_text,
    expected_groups_text,
) = sys.argv[1:]
repeats = int(repeats_text)
expected_runs = int(expected_runs_text)
expected_groups = int(expected_groups_text)
manifest_path = Path(manifest_name)
output_path = Path(output_name)

# Use the committed workspace source explicitly; the install tree may be stale.
source_root = Path(repository_root) / "ros2_ws/src/robot_experiments"
sys.path.insert(0, str(source_root))
from robot_experiments.contact_ab_analysis import (
    COMPLETE_MATRIX_PROFILES,
    analyse_contact_ab,
    write_contact_ab_report,
)

with manifest_path.open("r", encoding="utf-8", newline="") as stream:
    reader = csv.DictReader(stream, delimiter="\t", strict=True)
    rows = list(reader)
if len(rows) != expected_runs:
    raise SystemExit(
        f"manifest run count mismatch: {len(rows)} != {expected_runs}"
    )
report_paths = []
for row_index, row in enumerate(rows, start=1):
    if row.get("status") != "success":
        raise SystemExit(f"manifest row {row_index} is not successful")
    report_path = Path(row.get("report", ""))
    if not report_path.is_file() or report_path.is_symlink():
        raise SystemExit(f"manifest row {row_index} report path is unsafe")
    actual_sha256 = hashlib.sha256(report_path.read_bytes()).hexdigest()
    if row.get("report_sha256") != actual_sha256:
        raise SystemExit(f"manifest row {row_index} report SHA256 mismatch")
    report_paths.append(report_path)
if len(set(report_paths)) != expected_runs:
    raise SystemExit("manifest report paths must be unique")

arguments = {"min_repeats": repeats}
if environment_selection == "all":
    arguments["require_complete_matrix"] = True
elif environment_selection in {"Warehouse", "SimplePlane"}:
    arguments["expected_environments"] = (environment_selection,)
    arguments["expected_profiles"] = COMPLETE_MATRIX_PROFILES
else:
    raise SystemExit("unknown batch environment selection")
analysis = analyse_contact_ab(report_paths, 0.098, **arguments)
counts = analysis.get("counts", {})
selection = analysis.get("selection", {})
if analysis.get("analysis_valid") is not True:
    raise SystemExit("aggregate contact A/B analysis is not valid")
if counts.get("excluded_reports") != 0 or selection.get("excluded") != []:
    raise SystemExit("aggregate contact A/B analysis excluded reports")
if counts.get("included_reports") != expected_runs:
    raise SystemExit("aggregate included-report count mismatch")
if counts.get("groups") != expected_groups:
    raise SystemExit("aggregate group count mismatch")
write_contact_ab_report(analysis, output_path)
PY
}

write_batch_summary() {
  local successful_rows="$1"
  local manifest_rows="$2"
  batch_summary_path="${output_dir}/batch_summary.json"
  [[ ! -e "${batch_summary_path}" && ! -L "${batch_summary_path}" ]] || {
    log_warn "refusing to overwrite batch summary: ${batch_summary_path}"
    return 1
  }
  python3 - \
    "${batch_summary_path}" \
    "${environment_selection}" "${repeats}" \
    "${expected_conditions}" "${expected_groups}" \
    "${successful_rows}" "${manifest_rows}" \
    "${batch_git_commit}" "${batch_git_branch}" \
    "${motion_config}" "${batch_motion_sha256}" \
    "${warehouse_config}" "${batch_warehouse_project_sha256}" \
    "${simple_plane_config}" "${batch_simple_plane_project_sha256}" \
    "${physics_dir}" "${batch_profile_hashes_json}" \
    "${manifest}" "${frozen_manifest_sha256}" \
    "${analysis_path}" "${analysis_sha256}" <<'PY'
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile

(
    output_name,
    environment_selection,
    repeats_text,
    expected_runs_text,
    expected_groups_text,
    successful_rows_text,
    manifest_rows_text,
    git_commit,
    git_branch,
    motion_path,
    motion_sha256,
    warehouse_path,
    warehouse_sha256,
    simple_plane_path,
    simple_plane_sha256,
    physics_directory,
    profile_hashes_json,
    manifest_name,
    manifest_sha256,
    analysis_name,
    analysis_sha256,
) = sys.argv[1:]
output_path = Path(output_name)
manifest_path = Path(manifest_name)
analysis_path = Path(analysis_name)

def file_sha256(path):
    if not path.is_file() or path.is_symlink():
        raise SystemExit(f"summary evidence path is unsafe: {path}")
    return hashlib.sha256(path.read_bytes()).hexdigest()

if file_sha256(manifest_path) != manifest_sha256:
    raise SystemExit("frozen manifest SHA256 changed before summary")
if file_sha256(analysis_path) != analysis_sha256:
    raise SystemExit("aggregate analysis SHA256 changed before summary")
analysis = json.loads(analysis_path.read_text(encoding="utf-8"))
analysis_counts = analysis.get("counts", {})
expected_runs = int(expected_runs_text)
expected_groups = int(expected_groups_text)
if (
    int(successful_rows_text) != expected_runs
    or int(manifest_rows_text) != expected_runs
    or analysis_counts.get("included_reports") != expected_runs
    or analysis_counts.get("excluded_reports") != 0
    or analysis_counts.get("groups") != expected_groups
):
    raise SystemExit("batch summary counts are not a complete successful matrix")
profile_hashes = json.loads(profile_hashes_json)
if not isinstance(profile_hashes, dict) or len(profile_hashes) != 6:
    raise SystemExit("locked profile hash map must contain six profiles")
profiles = {
    profile_id: {
        "path": str(Path(physics_directory) / f"{profile_id}.yaml"),
        "sha256": digest,
    }
    for profile_id, digest in sorted(profile_hashes.items())
}
selected_environments = (
    ["SimplePlane", "Warehouse"]
    if environment_selection == "all"
    else [environment_selection]
)
summary = {
    "schema_version": 1,
    "report_type": "contact_ab_batch_summary",
    "result": "success",
    "environment_selection": environment_selection,
    "environments": selected_environments,
    "repeats": int(repeats_text),
    "expected_counts": {
        "runs": expected_runs,
        "groups": expected_groups,
        "environments": len(selected_environments),
        "profiles": 6,
    },
    "actual_counts": {
        "manifest_rows": int(manifest_rows_text),
        "successful_runs": int(successful_rows_text),
        "analysis_included_reports": analysis_counts.get("included_reports"),
        "analysis_excluded_reports": analysis_counts.get("excluded_reports"),
        "analysis_groups": analysis_counts.get("groups"),
    },
    "git": {"commit": git_commit, "branch": git_branch},
    "locked_protocol_inputs": {
        "motion_config": {"path": motion_path, "sha256": motion_sha256},
        "project_configs": {
            "Warehouse": {
                "path": warehouse_path,
                "sha256": warehouse_sha256,
            },
            "SimplePlane": {
                "path": simple_plane_path,
                "sha256": simple_plane_sha256,
            },
        },
        "contact_profiles": profiles,
    },
    "evidence": {
        "manifest": {"path": str(manifest_path), "sha256": manifest_sha256},
        "analysis": {"path": str(analysis_path), "sha256": analysis_sha256},
    },
}
temporary_descriptor, temporary_name = tempfile.mkstemp(
    dir=output_path.parent,
    prefix=f".{output_path.name}.",
    suffix=".tmp",
    text=True,
)
temporary_path = Path(temporary_name)
try:
    with os.fdopen(
        temporary_descriptor, "w", encoding="utf-8", newline=""
    ) as stream:
        json.dump(summary, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary_path, output_path)
except BaseException:
    temporary_path.unlink(missing_ok=True)
    raise
PY
}

require_command git
require_command realpath
require_command sha256sum
require_command python3
require_command timeout
require_command mktemp
require_executable "${SCRIPT_DIR}/run_isaac.sh"
require_executable "${SCRIPT_DIR}/run_motion_baseline.sh"
require_file "${motion_config}"
require_file "${warehouse_config}"
require_file "${simple_plane_config}"
for profile_id in "${profile_ids[@]}"; do
  require_file "${physics_dir}/${profile_id}.yaml"
done
require_clean_git
require_tracked_input "${motion_config}"
require_tracked_input "${warehouse_config}"
require_tracked_input "${simple_plane_config}"
for profile_id in "${profile_ids[@]}"; do
  require_tracked_input "${physics_dir}/${profile_id}.yaml"
done

source_ros --require-workspace
require_command ros2
ensure_dedicated_process_group "${original_args[@]}"
acquire_instance_lock contact_ab_matrix "contact A/B matrix"
runtime_lock_is_held isaac \
  && die "Isaac Sim is already running; stop it before contact A/B"
runtime_lock_is_held motion_baseline \
  && die "motion baseline is already running; stop it before contact A/B"
load_runtime_contracts
lock_batch_identity
verify_batch_identity "batch initialization" \
  || die "contact A/B batch identity changed during initialization"
prepare_output_directory
initialize_manifest

sequence=0
for environment_id in "${environments[@]}"; do
  for profile_index in "${!profile_ids[@]}"; do
    for ((repeat = 1; repeat <= repeats; repeat++)); do
      sequence=$((sequence + 1))
      if ! run_one_condition \
          "${sequence}" "${environment_id}" \
          "${profile_ids[profile_index]}" \
          "${profile_modes[profile_index]}" "${repeat}"; then
        die "contact A/B failed closed at ${current_run_id}: ${current_failure_reason}"
      fi
    done
  done
done

expected_conditions=$((${#environments[@]} * ${#profile_ids[@]} * repeats))
expected_groups=$((${#environments[@]} * ${#profile_ids[@]}))
successful_rows="$(
  awk -F $'\t' 'NR > 1 && $6 == "success" {count++} END {print count + 0}' \
    "${manifest}"
)" || die "cannot count successful manifest rows"
manifest_rows="$(awk 'END {print NR - 1}' "${manifest}")" \
  || die "cannot count manifest rows"
[[ "${sequence}" == "${expected_conditions}" \
      && "${successful_rows}" == "${expected_conditions}" \
      && "${manifest_rows}" == "${expected_conditions}" ]] \
  || die "contact A/B completion count mismatch: sequence=${sequence}, success=${successful_rows}, rows=${manifest_rows}, expected=${expected_conditions}"

freeze_manifest \
  || die "cannot freeze and hash the completed contact A/B manifest"
finalize_contact_analysis \
  || die "contact A/B aggregate analysis failed closed"
analysis_sha256="$(
  final_evidence_sha256 "${analysis_path}" true "aggregate analysis"
)" || die "cannot hash aggregate contact A/B analysis"
write_batch_summary "${successful_rows}" "${manifest_rows}" \
  || die "cannot write atomic contact A/B batch summary"

log_info "contact A/B matrix complete: ${batch_summary_path}"
