#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

original_args=("$@")

usage() {
  cat <<'EOF'
usage: run_contact_ab_matrix.sh [--environment Warehouse|SimplePlane|all]
                                [--ground-topology baseline|all|ID]
                                [--contact-profile all|ID]
                                [--reset-strategy project|pose_restore_v1|
                                  separate_recontact_0p20m_1step_v1|all]
                                [--repeats N] [--robot-config FILE]
                                --output-dir DIR

Run the committed skid-steer motion A/B protocol in strict serial order.
The default environment is all (SimplePlane, then Warehouse), the default
ground topology is baseline (one committed default per environment), all six
contact profiles are selected, the project reset strategy is resolved from
both committed project configs, and the default repeat count is 3: 2
environment/topology pairs x 6 contact profiles x 1 reset strategy x 3 = 36
independent Isaac processes.  --ground-topology all selects every
legal pair (3 pairs for --environment all, producing 54 default runs).  A
specific topology ID requires its one matching --environment.  DIR must be
empty and is never overwritten.
By default the committed robot selected by the project configuration is used.
FILE selects a committed robot contract by canonical absolute path.
EOF
}

environment_selection="all"
ground_topology_selection="baseline"
contact_profile_selection="all"
reset_strategy_selection="project"
repeats=3
output_dir=""
robot_config_argument=""
robot_config_option_seen=false
required_motion_report_schema_version=3
required_runtime_provenance_schema_version=6
required_reset_strategy_schema_version=1
readonly manifest_header_contract=$'run_id\tenvironment\tprofile_id\tprofile_mode\trepeat\tstatus\tdetail\treport\treport_sha256\treport_schema_version\truntime_provenance_schema_version\treset_strategy_schema_version\treset_strategy_id\tisaac_log\tisaac_log_sha256\trunner_log\trunner_log_sha256\tgit_commit\tgit_branch\tmotion_config\tmotion_config_sha256\twarehouse_project_config\twarehouse_project_config_sha256\tsimple_plane_project_config\tsimple_plane_project_config_sha256\trobot_config_selection\trobot_config\trobot_config_sha256\trobot_kinematics_profile_id\trobot_kinematics_lifecycle\trobot_wheel_radius_m\trobot_wheel_width_m\trobot_geometric_track_width_m\trobot_effective_track_width_m\tselected_project_config\tselected_project_config_sha256\tprofile_path\tprofile_sha256\tground_topology_id\tground_topology_profile_path\tground_topology_profile_sha256\tall_profile_hashes_json\tenvironment_project_stage\tenvironment_project_stage_sha256\tenvironment_source_asset\tenvironment_source_asset_sha256\tstarted_at_utc/completed_at_utc'
while (($#)); do
  case "$1" in
    --environment|--ground-topology|--contact-profile|--reset-strategy|\
      --repeats|--output-dir|--robot-config)
      (($# >= 2)) || die "$1 requires a value"
      case "$1" in
        --environment) environment_selection="$2" ;;
        --ground-topology) ground_topology_selection="$2" ;;
        --contact-profile) contact_profile_selection="$2" ;;
        --reset-strategy) reset_strategy_selection="$2" ;;
        --repeats) repeats="$2" ;;
        --output-dir) output_dir="$2" ;;
        --robot-config)
          robot_config_argument="$2"
          robot_config_option_seen=true
          ;;
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

declare -ag matrix_environment_ids=()
declare -ag matrix_ground_topology_ids=()

select_ground_topology_pairs() {
  local environment_id
  matrix_environment_ids=()
  matrix_ground_topology_ids=()
  case "${ground_topology_selection}" in
    baseline)
      for environment_id in "${environments[@]}"; do
        matrix_environment_ids+=("${environment_id}")
        case "${environment_id}" in
          SimplePlane)
            matrix_ground_topology_ids+=(simple_plane_only1_v1)
            ;;
          Warehouse)
            matrix_ground_topology_ids+=(warehouse_combined32_v1)
            ;;
          *) return 1 ;;
        esac
      done
      ;;
    all)
      for environment_id in "${environments[@]}"; do
        case "${environment_id}" in
          SimplePlane)
            matrix_environment_ids+=(SimplePlane)
            matrix_ground_topology_ids+=(simple_plane_only1_v1)
            ;;
          Warehouse)
            matrix_environment_ids+=(Warehouse Warehouse)
            matrix_ground_topology_ids+=(
              warehouse_combined32_v1
              warehouse_plane_only1_v1
            )
            ;;
          *) return 1 ;;
        esac
      done
      ;;
    simple_plane_only1_v1)
      [[ "${environment_selection}" == SimplePlane ]] || return 2
      matrix_environment_ids+=(SimplePlane)
      matrix_ground_topology_ids+=(simple_plane_only1_v1)
      ;;
    warehouse_combined32_v1|warehouse_plane_only1_v1)
      [[ "${environment_selection}" == Warehouse ]] || return 2
      matrix_environment_ids+=(Warehouse)
      matrix_ground_topology_ids+=("${ground_topology_selection}")
      ;;
    *) return 3 ;;
  esac
}

selection_status=0
select_ground_topology_pairs || selection_status=$?
if ((selection_status != 0)); then
  if ((selection_status == 2)); then
    die "--ground-topology ID must match the selected --environment"
  fi
  die "--ground-topology must be baseline, all, simple_plane_only1_v1, warehouse_combined32_v1, or warehouse_plane_only1_v1"
fi
(( ${#matrix_environment_ids[@]} == ${#matrix_ground_topology_ids[@]} )) \
  || die "internal environment/ground-topology pair selection mismatch"
(( ${#matrix_environment_ids[@]} > 0 )) \
  || die "ground-topology selection produced no legal pairs"
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
[[ "${output_dir}" != *$'\t'* && "${output_dir}" != *$'\r'* \
      && "${output_dir}" != *$'\n'* ]] \
  || die \
    "--output-dir must not contain tabs, carriage returns, or newlines"
if [[ "${robot_config_option_seen}" == true \
      && -z "${robot_config_argument}" ]]; then
  die "--robot-config requires a non-empty value"
fi
[[ "${robot_config_argument}" != *$'\t'* \
      && "${robot_config_argument}" != *$'\r'* \
      && "${robot_config_argument}" != *$'\n'* ]] \
  || die \
    "--robot-config must not contain tabs, carriage returns, or newlines"

shipped_profile_ids=(
  legacy_baseline
  threshold_corr_0p00025_offset_0p0004
  threshold_corr_0p025_offset_0p0004
  threshold_corr_0p00025_offset_0p04
  threshold_corr_0p025_offset_0p04
  explicit_material
)
shipped_profile_modes=(
  legacy_baseline
  threshold_only
  threshold_only
  threshold_only
  threshold_only
  explicit_material
)
profile_ids=()
profile_modes=()

select_contact_profiles() {
  local index
  profile_ids=()
  profile_modes=()
  if [[ "${contact_profile_selection}" == all ]]; then
    profile_ids=("${shipped_profile_ids[@]}")
    profile_modes=("${shipped_profile_modes[@]}")
    return 0
  fi
  for index in "${!shipped_profile_ids[@]}"; do
    if [[ "${shipped_profile_ids[index]}" \
          == "${contact_profile_selection}" ]]; then
      profile_ids+=("${shipped_profile_ids[index]}")
      profile_modes+=("${shipped_profile_modes[index]}")
      return 0
    fi
  done
  return 1
}

select_contact_profiles \
  || die "--contact-profile must be all or a shipped contact profile ID"

case "${reset_strategy_selection}" in
  project|pose_restore_v1|separate_recontact_0p20m_1step_v1|all) ;;
  *)
    die "--reset-strategy must be project, pose_restore_v1, separate_recontact_0p20m_1step_v1, or all"
    ;;
esac

reset_strategy_ids=()
select_reset_strategies() {
  local project_default_id="$1"
  case "${reset_strategy_selection}" in
    project)
      reset_strategy_ids=("${project_default_id}")
      ;;
    pose_restore_v1|separate_recontact_0p20m_1step_v1)
      reset_strategy_ids=("${reset_strategy_selection}")
      ;;
    all)
      reset_strategy_ids=(
        pose_restore_v1
        separate_recontact_0p20m_1step_v1
      )
      ;;
    *)
      return 1
      ;;
  esac
}

motion_config="${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/motion_skid_steer_ab.yaml"
warehouse_config="${PROJECT_ROOT}/isaac_sim/configs/project.yaml"
simple_plane_config="${PROJECT_ROOT}/isaac_sim/configs/simple_plane.project.yaml"
physics_dir="${PROJECT_ROOT}/isaac_sim/configs/physics"
ground_topology_dir="${PROJECT_ROOT}/isaac_sim/configs/ground_topologies"
manifest=""
batch_git_commit=""
batch_git_branch=""
batch_motion_sha256=""
batch_motion_configuration_json=""
batch_warehouse_project_sha256=""
batch_simple_plane_project_sha256=""
batch_robot_config_sha256=""
batch_profile_hashes_json=""
batch_ground_topology_hashes_json=""
batch_environment_topology_pairs_json=""
batch_reset_strategy_ids_json=""

robot_config=""
robot_config_selection="project_default"
robot_asset=""
robot_config_sha256=""
robot_asset_sha256=""
robot_kinematics_profile_id=""
robot_kinematics_lifecycle=""
robot_wheel_radius=""
robot_wheel_width=""
robot_geometric_track_width=""
robot_effective_track_width=""
project_reset_strategy_id=""
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
  local canonical relative head_entry head_mode head_type head_blob
  local verified_head_blob worktree_blob
  [[ "${path}" == "${PROJECT_ROOT}/"* ]] \
    || die "contact A/B input is outside the repository: ${path}"
  [[ "${path}" != *$'\t'* && "${path}" != *$'\r'* \
        && "${path}" != *$'\n'* && -f "${path}" && ! -L "${path}" ]] \
    || die "contact A/B input is not a committed regular file: ${path}"
  canonical="$(realpath -e -- "${path}" 2>/dev/null)" \
    || die "cannot canonicalize contact A/B input: ${path}"
  [[ "${canonical}" == "${path}" ]] \
    || die "contact A/B input path is not canonical: ${path}"
  relative="${path#"${PROJECT_ROOT}/"}"
  git -C "${PROJECT_ROOT}" ls-files --error-unmatch -- "${relative}" \
    >/dev/null 2>&1 \
    || die "contact A/B input is not committed: ${relative}"
  head_entry="$(
    git -C "${PROJECT_ROOT}" ls-tree HEAD -- "${relative}"
  )" || die "cannot inspect committed contact A/B input: ${relative}"
  IFS=$' \t' read -r \
    head_mode head_type head_blob _ <<<"${head_entry}"
  [[ ( "${head_mode}" == 100644 || "${head_mode}" == 100755 ) \
        && "${head_type}" == blob && -n "${head_blob}" ]] \
    || die "contact A/B input is not a committed regular file: ${relative}"
  verified_head_blob="$(
    git -C "${PROJECT_ROOT}" rev-parse --verify "HEAD:${relative}"
  )" || die "cannot resolve committed contact A/B blob: ${relative}"
  [[ "${verified_head_blob}" == "${head_blob}" ]] \
    || die "committed contact A/B blob identity is inconsistent: ${relative}"
  worktree_blob="$(git hash-object --no-filters -- "${path}")" \
    || die "cannot hash contact A/B working-tree input: ${relative}"
  [[ "${worktree_blob}" == "${head_blob}" ]] \
    || die \
      "contact A/B input does not match the committed HEAD blob: ${relative}"
}

validate_robot_config_path() {
  local path="$1"
  local canonical
  if [[ "${path}" != /* || ! -f "${path}" || -L "${path}" ]]; then
    log_warn \
      "--robot-config must be a canonical absolute regular file: ${path}"
    return 1
  fi
  canonical="$(realpath -e -- "${path}" 2>/dev/null)" || {
    log_warn \
      "--robot-config must be a canonical absolute regular file: ${path}"
    return 1
  }
  if [[ "${canonical}" != "${path}" ]]; then
    log_warn \
      "--robot-config must be a canonical absolute regular file: ${path}"
    return 1
  fi
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
  local robot_override="${2:-}"
  python3 - "${PROJECT_ROOT}" "${ISAAC_ASSET_ROOT}" \
    "${project_config}" "${robot_override}" <<'PY'
from pathlib import Path
import sys

repository_root, asset_root, project_path, robot_override = sys.argv[1:]
sys.path.insert(0, repository_root)
from isaac_sim.src.config import load_project_config
from isaac_sim.src.robot.kinematics_config import load_robot_config_contract

# Do not pass os.environ here: inherited ISAAC_NAV__* values are untrusted
# nested YAML overrides.  Only interpolation inputs are needed to resolve the
# committed project contract, plus the one explicitly selected robot input.
effective_environment = {
    "PROJECT_ROOT": repository_root,
    "ISAAC_ASSET_ROOT": asset_root,
}
if robot_override:
    effective_environment["ISAAC_NAV__FILES__ROBOT"] = robot_override
config = load_project_config(
    project_path,
    effective_environment,
)
kinematics = load_robot_config_contract(config.files.robot).kinematics
values = (
    str(config.schema_version),
    config.environment.identifier,
    str(config.files.robot),
    str(config.robot.asset_path),
    str(config.environment.project_stage),
    str(config.environment.source_asset),
    kinematics.kinematics_profile_id,
    kinematics.lifecycle,
    str(kinematics.wheel_radius),
    str(kinematics.wheel_width),
    str(kinematics.geometric_track_width),
    str(kinematics.effective_track_width),
    str(config.simulation.reset_strategy.schema_version),
    config.simulation.reset_strategy.identifier,
)
if any("\t" in value or "\n" in value for value in values):
    raise SystemExit("runtime contract paths must not contain tabs or newlines")
print("\t".join(values))
PY
}

load_runtime_contracts() {
  local warehouse_contract simple_plane_contract
  local warehouse_project_schema simple_plane_project_schema
  local warehouse_id simple_plane_id simple_robot_config simple_robot_asset
  local simple_kinematics_profile_id simple_kinematics_lifecycle
  local simple_wheel_radius simple_wheel_width
  local simple_geometric_track_width simple_effective_track_width
  local warehouse_reset_schema warehouse_reset_id
  local simple_reset_schema simple_reset_id
  local runtime_input
  warehouse_contract="$(
    project_runtime_contract \
      "${warehouse_config}" "${robot_config_argument}"
  )" \
    || die "cannot resolve the committed Warehouse runtime contract"
  simple_plane_contract="$(
    project_runtime_contract \
      "${simple_plane_config}" "${robot_config_argument}"
  )" || die "cannot resolve the committed SimplePlane runtime contract"
  IFS=$'\t' read -r \
    warehouse_project_schema warehouse_id robot_config robot_asset \
    warehouse_project_stage warehouse_source_asset \
    robot_kinematics_profile_id robot_kinematics_lifecycle \
    robot_wheel_radius robot_wheel_width \
    robot_geometric_track_width robot_effective_track_width \
    warehouse_reset_schema warehouse_reset_id \
    <<<"${warehouse_contract}"
  IFS=$'\t' read -r \
    simple_plane_project_schema simple_plane_id \
    simple_robot_config simple_robot_asset \
    simple_plane_project_stage simple_plane_source_asset \
    simple_kinematics_profile_id simple_kinematics_lifecycle \
    simple_wheel_radius simple_wheel_width \
    simple_geometric_track_width simple_effective_track_width \
    simple_reset_schema simple_reset_id \
    <<<"${simple_plane_contract}"
  [[ "${warehouse_project_schema}" == 2 \
        && "${simple_plane_project_schema}" == 2 ]] \
    || die "contact A/B requires project schema version 2"
  [[ "${warehouse_id}" == Warehouse && "${simple_plane_id}" == SimplePlane ]] \
    || die "project runtime environment identifiers are not canonical"
  [[ "${simple_robot_config}" == "${robot_config}" \
        && "${simple_robot_asset}" == "${robot_asset}" ]] \
    || die "Warehouse and SimplePlane must use the same robot inputs"
  [[ "${simple_kinematics_profile_id}" == "${robot_kinematics_profile_id}" \
        && "${simple_kinematics_lifecycle}" == "${robot_kinematics_lifecycle}" \
        && "${simple_wheel_radius}" == "${robot_wheel_radius}" \
        && "${simple_wheel_width}" == "${robot_wheel_width}" \
        && "${simple_geometric_track_width}" == "${robot_geometric_track_width}" \
        && "${simple_effective_track_width}" == "${robot_effective_track_width}" ]] \
    || die "Warehouse and SimplePlane robot kinematics contracts differ"
  [[ "${warehouse_reset_schema}" == 1 \
        && "${simple_reset_schema}" == 1 \
        && "${warehouse_reset_id}" == "${simple_reset_id}" ]] \
    || die "Warehouse and SimplePlane project reset strategy contracts differ"
  case "${warehouse_reset_id}" in
    pose_restore_v1|separate_recontact_0p20m_1step_v1) ;;
    *) die "project reset strategy ID is unsupported" ;;
  esac
  project_reset_strategy_id="${warehouse_reset_id}"
  select_reset_strategies "${project_reset_strategy_id}" \
    || die "cannot resolve selected reset strategies"
  batch_reset_strategy_ids_json="$(
    python3 - "${reset_strategy_ids[@]}" <<'PY'
import json
import sys

strategy_ids = sys.argv[1:]
if (
    not strategy_ids
    or len(strategy_ids) != len(set(strategy_ids))
    or any(
        strategy_id not in {
            "pose_restore_v1",
            "separate_recontact_0p20m_1step_v1",
        }
        for strategy_id in strategy_ids
    )
):
    raise SystemExit("selected reset strategy IDs are invalid")
print(json.dumps(strategy_ids, separators=(",", ":")))
PY
  )" || die "cannot serialize selected reset strategies"
  if [[ "${robot_config_option_seen}" == true ]]; then
    robot_config_selection="explicit_cli"
  fi
  validate_robot_config_path "${robot_config}" \
    || die "selected robot config path is not trusted"
  require_tracked_input "${robot_config}"
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
  local profile_id profile_path topology_id topology_path
  local -a profile_hash_arguments=() topology_hash_arguments=()
  batch_git_commit="$(git_commit)" \
    || die "contact A/B requires an attached Git HEAD"
  batch_git_branch="$(git_branch)" \
    || die "contact A/B requires an attached Git branch"
  locked_input_paths=(
    "${motion_config}"
    "${warehouse_config}"
    "${simple_plane_config}"
    "${robot_config}"
  )
  for profile_id in "${profile_ids[@]}"; do
    locked_input_paths+=("${physics_dir}/${profile_id}.yaml")
  done
  for topology_id in "${matrix_ground_topology_ids[@]}"; do
    topology_path="$(ground_topology_path "${topology_id}")" \
      || die "cannot resolve selected ground topology: ${topology_id}"
    locked_input_paths+=("${topology_path}")
  done
  for profile_path in "${locked_input_paths[@]}"; do
    require_tracked_input "${profile_path}"
    locked_input_hashes["${profile_path}"]="$(sha256_file "${profile_path}")" \
      || die "cannot hash contact A/B input: ${profile_path}"
  done
  batch_motion_sha256="${locked_input_hashes[${motion_config}]}"
  batch_warehouse_project_sha256="${locked_input_hashes[${warehouse_config}]}"
  batch_simple_plane_project_sha256="${locked_input_hashes[${simple_plane_config}]}"
  batch_robot_config_sha256="${locked_input_hashes[${robot_config}]}"
  [[ "${batch_robot_config_sha256}" == "${robot_config_sha256}" ]] \
    || die "selected robot config changed while batch identity was locked"
  robot_config_sha256="${batch_robot_config_sha256}"
  for profile_id in "${profile_ids[@]}"; do
    profile_path="${physics_dir}/${profile_id}.yaml"
    profile_hash_arguments+=(
      "${profile_id}" "${locked_input_hashes[${profile_path}]}"
    )
  done
  batch_profile_hashes_json="$(python3 - \
    "${profile_hash_arguments[@]}" <<'PY'
import json
import sys

arguments = sys.argv[1:]
if len(arguments) % 2:
    raise SystemExit("contact profile hash argument count mismatch")
profiles = dict(zip(arguments[::2], arguments[1::2], strict=True))
if len(profiles) * 2 != len(arguments):
    raise SystemExit("contact profile identifiers must be unique")
print(json.dumps(profiles, sort_keys=True, separators=(",", ":")))
PY
  )" || die "cannot serialize locked contact profile hashes"
  for topology_id in "${matrix_ground_topology_ids[@]}"; do
    topology_path="$(ground_topology_path "${topology_id}")" \
      || die "cannot resolve selected ground topology: ${topology_id}"
    topology_hash_arguments+=(
      "${topology_id}" "${locked_input_hashes[${topology_path}]}"
    )
  done
  batch_ground_topology_hashes_json="$(
    python3 - "${topology_hash_arguments[@]}" <<'PY'
import json
import sys

arguments = sys.argv[1:]
if len(arguments) % 2:
    raise SystemExit("ground-topology hash argument count mismatch")
profiles = dict(zip(arguments[::2], arguments[1::2], strict=True))
if len(profiles) * 2 != len(arguments):
    raise SystemExit("ground-topology identifiers must be unique")
print(json.dumps(profiles, sort_keys=True, separators=(",", ":")))
PY
  )" || die "cannot serialize locked ground-topology profile hashes"
  batch_environment_topology_pairs_json="$(
    python3 - "${#matrix_environment_ids[@]}" \
      "${matrix_environment_ids[@]}" \
      "${matrix_ground_topology_ids[@]}" <<'PY'
import json
import sys

count = int(sys.argv[1])
environments = sys.argv[2:2 + count]
topologies = sys.argv[2 + count:]
if len(environments) != count or len(topologies) != count:
    raise SystemExit("environment/topology pair argument count mismatch")
pairs = [
    {"environment_id": environment, "ground_topology_id": topology}
    for environment, topology in zip(environments, topologies, strict=True)
]
print(json.dumps(pairs, sort_keys=True, separators=(",", ":")))
PY
  )" || die "cannot serialize selected environment/topology pairs"
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

freeze_motion_configuration_contract() {
  batch_motion_configuration_json="$(python3 - \
    "${motion_config}" \
    "${PROJECT_ROOT}/ros2_ws/src/robot_experiments" <<'PY'
import json
from pathlib import Path
import sys

config_path = Path(sys.argv[1])
package_root = Path(sys.argv[2])
sys.path.insert(0, str(package_root))
from robot_experiments.motion_baseline import load_motion_baseline_config

configuration = load_motion_baseline_config(config_path).as_dict()
print(json.dumps(configuration, sort_keys=True, separators=(",", ":")))
PY
  )" || die "cannot freeze normalized motion configuration"
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

ground_topology_path() {
  case "$1" in
    simple_plane_only1_v1|warehouse_combined32_v1|warehouse_plane_only1_v1)
      printf '%s/%s.yaml\n' "${ground_topology_dir}" "$1"
      ;;
    *) return 1 ;;
  esac
}

tsv_safe() {
  local value="$1"
  value="${value//$'\t'/ }"
  value="${value//$'\r'/ }"
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
current_reset_strategy_id=""
current_reset_strategy_token=""
current_repeat=""
current_report=""
current_isaac_log=""
current_runner_log=""
current_project_config=""
current_project_sha256=""
current_profile_path=""
current_profile_sha256=""
current_ground_topology_id=""
current_ground_topology_path=""
current_ground_topology_sha256=""
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

motion_report_schema_version() {
  local path="$1"
  python3 - "${path}" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
try:
    report = json.loads(path.read_text(encoding="utf-8"))
except (OSError, UnicodeError, ValueError) as exc:
    raise SystemExit(f"cannot read motion report schema version: {exc}")
schema_version = report.get("schema_version")
if isinstance(schema_version, bool) or not isinstance(schema_version, int):
    raise SystemExit("motion report schema_version must be an integer")
print(schema_version)
PY
}

validate_success_manifest_evidence() {
  local phase="$1"
  python3 - \
    "${manifest}" "${manifest_header_contract}" \
    "${required_motion_report_schema_version}" \
    "${required_runtime_provenance_schema_version}" \
    "${required_reset_strategy_schema_version}" "${phase}" \
    "${output_dir}" "${repeats}" "${expected_conditions}" \
    "${batch_environment_topology_pairs_json}" \
    "${batch_profile_hashes_json}" \
    "${batch_ground_topology_hashes_json}" \
    "${batch_reset_strategy_ids_json}" \
    "${batch_git_commit}" "${batch_git_branch}" \
    "${motion_config}" "${batch_motion_sha256}" \
    "${batch_motion_configuration_json}" \
    "${warehouse_config}" "${batch_warehouse_project_sha256}" \
    "${simple_plane_config}" "${batch_simple_plane_project_sha256}" \
    "${robot_config_selection}" "${robot_config}" \
    "${batch_robot_config_sha256}" \
    "${robot_asset}" "${robot_asset_sha256}" \
    "${robot_kinematics_profile_id}" \
    "${robot_kinematics_lifecycle}" \
    "${robot_wheel_radius}" "${robot_wheel_width}" \
    "${robot_geometric_track_width}" \
    "${robot_effective_track_width}" \
    "${physics_dir}" "${ground_topology_dir}" \
    "${warehouse_project_stage}" \
    "${warehouse_project_stage_sha256}" \
    "${warehouse_source_asset}" "${warehouse_source_asset_sha256}" \
    "${simple_plane_project_stage}" \
    "${simple_plane_project_stage_sha256}" \
    "${simple_plane_source_asset}" \
    "${simple_plane_source_asset_sha256}" <<'PY'
import csv
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import sys

(
    manifest_name,
    manifest_header,
    required_report_schema_text,
    required_provenance_schema_text,
    required_reset_schema_text,
    phase,
    output_directory,
    repeats_text,
    expected_rows_text,
    selected_pairs_json,
    profile_hashes_json,
    topology_hashes_json,
    reset_strategy_ids_json,
    git_commit,
    git_branch,
    motion_config,
    motion_sha256,
    motion_configuration_json,
    warehouse_config,
    warehouse_config_sha256,
    simple_plane_config,
    simple_plane_config_sha256,
    robot_config_selection,
    robot_config,
    robot_config_sha256,
    robot_asset,
    robot_asset_sha256,
    kinematics_profile_id,
    kinematics_lifecycle,
    wheel_radius,
    wheel_width,
    geometric_track_width,
    effective_track_width,
    physics_directory,
    ground_topology_directory,
    warehouse_project_stage,
    warehouse_project_stage_sha256,
    warehouse_source_asset,
    warehouse_source_asset_sha256,
    simple_plane_project_stage,
    simple_plane_project_stage_sha256,
    simple_plane_source_asset,
    simple_plane_source_asset_sha256,
) = sys.argv[1:]
manifest_path = Path(manifest_name)
expected_fieldnames = manifest_header.split("\t")
required_report_schema = int(required_report_schema_text)
required_provenance_schema = int(required_provenance_schema_text)
required_reset_schema = int(required_reset_schema_text)
repeats = int(repeats_text)
expected_rows = int(expected_rows_text)
output_path = Path(output_directory)

if len(expected_fieldnames) != 47 or len(set(expected_fieldnames)) != 47:
    raise SystemExit("internal manifest header contract must contain 47 unique columns")
if manifest_path.is_symlink() or not manifest_path.is_file():
    raise SystemExit(f"{phase}: manifest path is unsafe: {manifest_path}")

with manifest_path.open("r", encoding="utf-8", newline="") as stream:
    reader = csv.DictReader(stream, delimiter="\t", strict=True)
    if reader.fieldnames != expected_fieldnames:
        raise SystemExit(f"{phase}: manifest header does not match the 47-column contract")
    rows = list(reader)
if len(rows) != expected_rows:
    raise SystemExit(
        f"{phase}: manifest row count mismatch: {len(rows)} != {expected_rows}"
    )

try:
    selected_pairs = json.loads(selected_pairs_json)
    profile_hashes = json.loads(profile_hashes_json)
    topology_hashes = json.loads(topology_hashes_json)
    reset_strategy_ids = json.loads(reset_strategy_ids_json)
    motion_configuration = json.loads(motion_configuration_json)
except ValueError as exc:
    raise SystemExit(f"{phase}: locked matrix JSON is invalid: {exc}") from exc
if (
    not isinstance(selected_pairs, list)
    or not selected_pairs
    or not isinstance(profile_hashes, dict)
    or not isinstance(topology_hashes, dict)
    or not isinstance(reset_strategy_ids, list)
    or not reset_strategy_ids
    or not isinstance(motion_configuration, dict)
):
    raise SystemExit(f"{phase}: locked matrix inputs are incomplete")
if json.dumps(
    profile_hashes, sort_keys=True, separators=(",", ":")
) != profile_hashes_json:
    raise SystemExit(f"{phase}: profile hash JSON must be canonical")
if json.dumps(
    motion_configuration, sort_keys=True, separators=(",", ":")
) != motion_configuration_json:
    raise SystemExit(f"{phase}: motion configuration JSON must be canonical")

complete_profile_contract = (
    ("legacy_baseline", "legacy_baseline"),
    ("threshold_corr_0p00025_offset_0p0004", "threshold_only"),
    ("threshold_corr_0p025_offset_0p0004", "threshold_only"),
    ("threshold_corr_0p00025_offset_0p04", "threshold_only"),
    ("threshold_corr_0p025_offset_0p04", "threshold_only"),
    ("explicit_material", "explicit_material"),
)
if not set(profile_hashes).issubset(
    {profile_id for profile_id, _ in complete_profile_contract}
):
    raise SystemExit(f"{phase}: profile hash map contains an unsupported profile")
profile_contract = tuple(
    item for item in complete_profile_contract if item[0] in profile_hashes
)
if not profile_contract or len(profile_contract) != len(profile_hashes):
    raise SystemExit(f"{phase}: profile hash map does not match the selected profiles")
if (
    reset_strategy_ids
    not in [
        ["pose_restore_v1"],
        ["separate_recontact_0p20m_1step_v1"],
        ["pose_restore_v1", "separate_recontact_0p20m_1step_v1"],
    ]
):
    raise SystemExit(f"{phase}: reset strategy selection is invalid")
pair_identities = []
for pair_index, pair in enumerate(selected_pairs, start=1):
    if not isinstance(pair, dict) or set(pair) != {
        "environment_id",
        "ground_topology_id",
    }:
        raise SystemExit(f"{phase}: selected pair {pair_index} is invalid")
    identity = (pair["environment_id"], pair["ground_topology_id"])
    if identity not in {
        ("SimplePlane", "simple_plane_only1_v1"),
        ("Warehouse", "warehouse_combined32_v1"),
        ("Warehouse", "warehouse_plane_only1_v1"),
    }:
        raise SystemExit(f"{phase}: selected pair {pair_index} is unsupported")
    pair_identities.append(identity)
if len(pair_identities) != len(set(pair_identities)):
    raise SystemExit(f"{phase}: selected environment/topology pairs must be unique")
if set(topology_hashes) != {topology for _, topology in pair_identities}:
    raise SystemExit(f"{phase}: topology hash map does not match selected pairs")
if expected_rows != (
    len(pair_identities)
    * len(profile_contract)
    * len(reset_strategy_ids)
    * repeats
):
    raise SystemExit(f"{phase}: expected row count contradicts the matrix contract")

environment_contracts = {
    "Warehouse": {
        "project_config": warehouse_config,
        "project_config_sha256": warehouse_config_sha256,
        "project_stage": warehouse_project_stage,
        "project_stage_sha256": warehouse_project_stage_sha256,
        "source_asset": warehouse_source_asset,
        "source_asset_sha256": warehouse_source_asset_sha256,
        "slug": "warehouse",
    },
    "SimplePlane": {
        "project_config": simple_plane_config,
        "project_config_sha256": simple_plane_config_sha256,
        "project_stage": simple_plane_project_stage,
        "project_stage_sha256": simple_plane_project_stage_sha256,
        "source_asset": simple_plane_source_asset,
        "source_asset_sha256": simple_plane_source_asset_sha256,
        "slug": "simple_plane",
    },
}
expected_matrix_rows = []
sequence = 0
for environment_id, topology_id in pair_identities:
    for profile_id, profile_mode in profile_contract:
        for repeat in range(1, repeats + 1):
            for reset_strategy_id in reset_strategy_ids:
                sequence += 1
                reset_token = f"reset-v1-{reset_strategy_id}"
                run_id = (
                    f"{sequence:03d}_{environment_contracts[environment_id]['slug']}_"
                    f"{topology_id}_{reset_token}_{profile_id}_r{repeat:02d}"
                )
                expected_matrix_rows.append(
                    {
                        "run_id": run_id,
                        "environment": environment_id,
                        "ground_topology_id": topology_id,
                        "profile_id": profile_id,
                        "profile_mode": profile_mode,
                        "reset_strategy_id": reset_strategy_id,
                        "repeat": str(repeat),
                    }
                )


def canonical_regular_file(raw_path, row_index, field):
    if not isinstance(raw_path, str) or not raw_path:
        raise SystemExit(f"{phase}: manifest row {row_index} {field} path is empty")
    path = Path(raw_path)
    if not path.is_absolute() or path.is_symlink() or not path.is_file():
        raise SystemExit(
            f"{phase}: manifest row {row_index} {field} path is unsafe"
        )
    try:
        canonical = path.resolve(strict=True)
    except OSError as exc:
        raise SystemExit(
            f"{phase}: manifest row {row_index} {field} path cannot be resolved: {exc}"
        ) from exc
    if path != canonical:
        raise SystemExit(
            f"{phase}: manifest row {row_index} {field} path is not canonical"
        )
    return path


def digest_file(path):
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            while chunk := stream.read(1024 * 1024):
                digest.update(chunk)
    except OSError as exc:
        raise SystemExit(f"{phase}: cannot hash evidence file {path}: {exc}") from exc
    return digest.hexdigest()


def reset_strategy_contract_matches(strategy, expected_id, topology):
    expected_steps = {
        "pose_restore_v1": (0.0, 0, 1),
        "separate_recontact_0p20m_1step_v1": (0.2, 1, 1),
    }
    if (
        not isinstance(strategy, dict)
        or set(strategy)
        != {
            "schema_version",
            "id",
            "lift_distance_m",
            "separation_step_count",
            "recontact_step_count",
            "contact_probe",
        }
        or strategy.get("schema_version") != required_reset_schema
        or strategy.get("id") != expected_id
        or expected_id not in expected_steps
    ):
        return False
    lift, separation_steps, recontact_steps = expected_steps[expected_id]
    actual_lift = strategy.get("lift_distance_m")
    if (
        isinstance(actual_lift, bool)
        or not isinstance(actual_lift, (int, float))
        or not math.isfinite(float(actual_lift))
        or float(actual_lift) != lift
        or isinstance(strategy.get("separation_step_count"), bool)
        or strategy.get("separation_step_count") != separation_steps
        or isinstance(strategy.get("recontact_step_count"), bool)
        or strategy.get("recontact_step_count") != recontact_steps
    ):
        return False
    probe = strategy.get("contact_probe")
    if not isinstance(probe, dict) or set(probe) != {
        "schema_version",
        "enabled",
        "wheel_bindings",
        "wheel_count",
        "ground_filter_paths",
        "ground_filter_count",
        "max_contact_count",
        "report_threshold_n",
        "stage_usd_readback_verified",
    }:
        return False
    wheel_bindings = probe.get("wheel_bindings")
    ground_paths = probe.get("ground_filter_paths")
    target_colliders = topology.get("target_colliders")
    prim_path = re.compile(
        r"^/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*$"
    )
    threshold = probe.get("report_threshold_n")
    return (
        probe.get("schema_version") == 1
        and probe.get("enabled") is True
        and isinstance(wheel_bindings, list)
        and len(wheel_bindings) == 4
        and probe.get("wheel_count") == 4
        and all(
            isinstance(binding, dict)
            and set(binding) == {"joint_name", "wheel_link_path"}
            and isinstance(binding["joint_name"], str)
            and binding["joint_name"]
            and isinstance(binding["wheel_link_path"], str)
            and prim_path.fullmatch(binding["wheel_link_path"])
            for binding in wheel_bindings
        )
        and len({binding["joint_name"] for binding in wheel_bindings}) == 4
        and len({binding["wheel_link_path"] for binding in wheel_bindings}) == 4
        and isinstance(ground_paths, list)
        and ground_paths == target_colliders
        and ground_paths == sorted(ground_paths)
        and len(set(ground_paths)) == len(ground_paths)
        and all(
            isinstance(path, str) and prim_path.fullmatch(path)
            for path in ground_paths
        )
        and not isinstance(probe.get("ground_filter_count"), bool)
        and probe.get("ground_filter_count") == len(ground_paths)
        and not isinstance(probe.get("max_contact_count"), bool)
        and probe.get("max_contact_count") == 128
        and not isinstance(threshold, bool)
        and isinstance(threshold, (int, float))
        and math.isfinite(float(threshold))
        and float(threshold) == 0.0
        and probe.get("stage_usd_readback_verified") is True
    )


all_evidence_paths = set()
typed_evidence_paths = {
    "report": set(),
    "isaac_log": set(),
    "runner_log": set(),
}
for row_index, (row, expected_row) in enumerate(
    zip(rows, expected_matrix_rows, strict=True), start=1
):
    if None in row or any(value is None for value in row.values()):
        raise SystemExit(
            f"{phase}: manifest row {row_index} has missing or extra fields"
        )
    if row.get("status") != "success":
        raise SystemExit(f"{phase}: manifest row {row_index} is not successful")
    if row.get("detail") != "complete":
        raise SystemExit(f"{phase}: manifest row {row_index} detail must be complete")
    for field, expected in expected_row.items():
        if row.get(field) != expected:
            raise SystemExit(
                f"{phase}: manifest row {row_index} {field} contradicts matrix order"
            )
    environment = expected_row["environment"]
    topology_id = expected_row["ground_topology_id"]
    profile_id = expected_row["profile_id"]
    environment_contract = environment_contracts[environment]
    expected_scalars = {
        "runtime_provenance_schema_version": str(required_provenance_schema),
        "reset_strategy_schema_version": str(required_reset_schema),
        "git_commit": git_commit,
        "git_branch": git_branch,
        "motion_config": motion_config,
        "motion_config_sha256": motion_sha256,
        "warehouse_project_config": warehouse_config,
        "warehouse_project_config_sha256": warehouse_config_sha256,
        "simple_plane_project_config": simple_plane_config,
        "simple_plane_project_config_sha256": simple_plane_config_sha256,
        "robot_config_selection": robot_config_selection,
        "robot_config": robot_config,
        "robot_config_sha256": robot_config_sha256,
        "robot_kinematics_profile_id": kinematics_profile_id,
        "robot_kinematics_lifecycle": kinematics_lifecycle,
        "robot_wheel_radius_m": wheel_radius,
        "robot_wheel_width_m": wheel_width,
        "robot_geometric_track_width_m": geometric_track_width,
        "robot_effective_track_width_m": effective_track_width,
        "selected_project_config": environment_contract["project_config"],
        "selected_project_config_sha256": environment_contract[
            "project_config_sha256"
        ],
        "profile_path": str(Path(physics_directory) / f"{profile_id}.yaml"),
        "profile_sha256": profile_hashes[profile_id],
        "ground_topology_profile_path": str(
            Path(ground_topology_directory) / f"{topology_id}.yaml"
        ),
        "ground_topology_profile_sha256": topology_hashes[topology_id],
        "all_profile_hashes_json": profile_hashes_json,
        "environment_project_stage": environment_contract["project_stage"],
        "environment_project_stage_sha256": environment_contract[
            "project_stage_sha256"
        ],
        "environment_source_asset": environment_contract["source_asset"],
        "environment_source_asset_sha256": environment_contract[
            "source_asset_sha256"
        ],
    }
    for field, expected in expected_scalars.items():
        if row.get(field) != expected:
            raise SystemExit(
                f"{phase}: manifest row {row_index} {field} identity mismatch"
            )
    run_id = expected_row["run_id"]
    expected_evidence_paths = {
        "report": output_path / "reports" / f"{run_id}.json",
        "isaac_log": output_path / "logs" / f"{run_id}.isaac.log",
        "runner_log": output_path / "logs" / f"{run_id}.runner.log",
    }
    locked_paths = {}
    for field, sha_field in (
        ("report", "report_sha256"),
        ("isaac_log", "isaac_log_sha256"),
        ("runner_log", "runner_log_sha256"),
    ):
        path = canonical_regular_file(row.get(field), row_index, field)
        if path != expected_evidence_paths[field]:
            raise SystemExit(
                f"{phase}: manifest row {row_index} {field} path mismatch"
            )
        if path in all_evidence_paths or path in typed_evidence_paths[field]:
            raise SystemExit(
                f"{phase}: manifest evidence paths must be unique and disjoint"
            )
        all_evidence_paths.add(path)
        typed_evidence_paths[field].add(path)
        actual_sha256 = digest_file(path)
        if row.get(sha_field) != actual_sha256:
            raise SystemExit(
                f"{phase}: manifest row {row_index} {field} SHA256 mismatch"
            )
        locked_paths[field] = path
    try:
        report = json.loads(locked_paths["report"].read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError) as exc:
        raise SystemExit(
            f"{phase}: manifest row {row_index} report is not valid JSON: {exc}"
        ) from exc
    if report.get("schema_version") != required_report_schema:
        raise SystemExit(
            f"{phase}: manifest row {row_index} report JSON schema version mismatch"
        )
    if report.get("output_file") != str(locked_paths["report"]):
        raise SystemExit(
            f"{phase}: manifest row {row_index} report output path mismatch"
        )
    if row.get("report_schema_version") != str(required_report_schema):
        raise SystemExit(
            f"{phase}: manifest row {row_index} report schema version mismatch"
        )
    try:
        report_configuration_json = json.dumps(
            report.get("configuration"),
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
    except (TypeError, ValueError) as exc:
        raise SystemExit(
            f"{phase}: manifest row {row_index} report configuration is invalid: "
            f"{exc}"
        ) from exc
    provenance = report.get("runtime_provenance")
    if not isinstance(provenance, dict):
        raise SystemExit(f"{phase}: manifest row {row_index} report provenance is invalid")
    contact = provenance.get("contact")
    ground_topology = provenance.get("ground_topology")
    report_git = provenance.get("git")
    report_robot = provenance.get("robot")
    report_environment = provenance.get("environment")
    report_simulation = provenance.get("simulation")
    report_reset_strategy = (
        report_simulation.get("reset_strategy")
        if isinstance(report_simulation, dict)
        else None
    )
    robot_config_lock = (
        report_robot.get("config") if isinstance(report_robot, dict) else None
    )
    robot_asset_lock = (
        report_robot.get("asset") if isinstance(report_robot, dict) else None
    )
    solver = (
        report_robot.get("solver") if isinstance(report_robot, dict) else None
    )
    kinematics = (
        report_robot.get("kinematics")
        if isinstance(report_robot, dict)
        else None
    )

    def numeric_lock_matches(field, expected_text):
        value = kinematics.get(field) if isinstance(kinematics, dict) else None
        try:
            return (
                not isinstance(value, bool)
                and isinstance(value, (int, float))
                and math.isfinite(float(value))
                and float(value) == float(expected_text)
            )
        except (TypeError, ValueError):
            return False

    if (
        provenance.get("schema_version") != required_provenance_schema
        or report.get("environment_id") != environment
        or report.get("odometry_mode") != "ideal"
        or report.get("config_file") != motion_config
        or report.get("config_sha256") != motion_sha256
        or report_configuration_json != motion_configuration_json
        or not isinstance(contact, dict)
        or contact.get("profile_id") != profile_id
        or contact.get("profile_mode") != expected_row["profile_mode"]
        or contact.get("profile_path") != expected_scalars["profile_path"]
        or contact.get("profile_sha256") != expected_scalars["profile_sha256"]
        or not isinstance(ground_topology, dict)
        or ground_topology.get("profile_id") != topology_id
        or ground_topology.get("profile_path")
        != expected_scalars["ground_topology_profile_path"]
        or ground_topology.get("profile_sha256")
        != expected_scalars["ground_topology_profile_sha256"]
        or not isinstance(report_git, dict)
        or report_git.get("commit") != git_commit
        or report_git.get("branch") != git_branch
        or report_git.get("dirty") is not False
        or not isinstance(robot_config_lock, dict)
        or robot_config_lock.get("path") != robot_config
        or robot_config_lock.get("sha256") != robot_config_sha256
        or not isinstance(robot_asset_lock, dict)
        or robot_asset_lock.get("path") != robot_asset
        or robot_asset_lock.get("sha256") != robot_asset_sha256
        or solver
        != {
            "position_iterations": 32,
            "velocity_iterations": 4,
            "stage_articulation_usd_readback_verified": True,
        }
        or not isinstance(kinematics, dict)
        or kinematics.get("profile_id") != kinematics_profile_id
        or kinematics.get("lifecycle") != kinematics_lifecycle
        or kinematics.get("controller_contract_verified") is not True
        or not numeric_lock_matches("wheel_radius_m", wheel_radius)
        or not numeric_lock_matches("wheel_width_m", wheel_width)
        or not numeric_lock_matches(
            "geometric_track_width_m", geometric_track_width
        )
        or not numeric_lock_matches(
            "effective_track_width_m", effective_track_width
        )
        or not isinstance(report_environment, dict)
        or report_environment.get("id") != environment
        or report_environment.get("project_stage")
        != {
            "path": expected_scalars["environment_project_stage"],
            "sha256": expected_scalars["environment_project_stage_sha256"],
        }
        or report_environment.get("source_asset")
        != {
            "path": expected_scalars["environment_source_asset"],
            "sha256": expected_scalars["environment_source_asset_sha256"],
        }
        or not isinstance(report_simulation, dict)
        or report_simulation.get("navigation_mode") != "mapping"
        or report_simulation.get("odometry_mode") != "ideal"
        or isinstance(report_simulation.get("physics_hz"), bool)
        or not isinstance(report_simulation.get("physics_hz"), (int, float))
        or float(report_simulation.get("physics_hz")) != 60.0
        or not reset_strategy_contract_matches(
            report_reset_strategy,
            expected_row["reset_strategy_id"],
            ground_topology,
        )
    ):
        raise SystemExit(
            f"{phase}: manifest row {row_index} report identity mismatch"
        )
    interval = row.get("started_at_utc/completed_at_utc", "")
    parts = interval.split("/")
    if len(parts) != 2:
        raise SystemExit(f"{phase}: manifest row {row_index} time interval is invalid")
    try:
        started, completed = (
            datetime.strptime(value, "%Y-%m-%dT%H:%M:%SZ").replace(
                tzinfo=timezone.utc
            )
            for value in parts
        )
    except ValueError as exc:
        raise SystemExit(
            f"{phase}: manifest row {row_index} time interval is invalid: {exc}"
        ) from exc
    if completed < started:
        raise SystemExit(
            f"{phase}: manifest row {row_index} completes before it starts"
        )
    try:
        report_started, report_completed = (
            datetime.fromisoformat(report[field])
            for field in ("started_at_utc", "completed_at_utc")
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise SystemExit(
            f"{phase}: manifest row {row_index} report time interval is invalid: {exc}"
        ) from exc
    if any(
        value.tzinfo is None or value.utcoffset() != timezone.utc.utcoffset(value)
        for value in (report_started, report_completed)
    ):
        raise SystemExit(
            f"{phase}: manifest row {row_index} report timestamps must be UTC"
        )
    # The manifest is intentionally written at whole-second precision while
    # motion reports retain microseconds.  Treat the completed second as a
    # closed one-second bucket so a report completed later in that same second
    # is not rejected as being outside its manifest interval.
    if not (
        started <= report_started <= report_completed
        and report_completed < completed + timedelta(seconds=1)
    ):
        raise SystemExit(
            f"{phase}: manifest row {row_index} does not enclose report timestamps"
        )
PY
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
  local completed report_sha256 report_schema_version=""
  local isaac_log_sha256 runner_log_sha256 row
  local -a manifest_fields=()
  local evidence_required=false
  [[ "${status}" == success ]] && evidence_required=true
  completed="$(date -u +%Y-%m-%dT%H:%M:%SZ)" || return 1
  report_sha256="$(
    final_evidence_sha256 \
      "${current_report}" "${evidence_required}" "motion report"
  )" || return 1
  if [[ -f "${current_report}" && ! -L "${current_report}" ]]; then
    if ! report_schema_version="$(
      motion_report_schema_version "${current_report}"
    )"; then
      [[ "${evidence_required}" == false ]] || return 1
      report_schema_version=""
    fi
  fi
  if [[ "${evidence_required}" == true \
        && "${report_schema_version}" \
        != "${required_motion_report_schema_version}" ]]; then
    log_warn \
      "successful motion report schema mismatch: ${report_schema_version:-missing} != ${required_motion_report_schema_version}"
    return 1
  fi
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
  manifest_fields=(
    "$(tsv_safe "${current_run_id}")" \
    "$(tsv_safe "${current_environment}")" \
    "$(tsv_safe "${current_profile_id}")" \
    "$(tsv_safe "${current_profile_mode}")" \
    "${current_repeat}" \
    "${status}" \
    "$(tsv_safe "${detail}")" \
    "$(tsv_safe "${current_report}")" \
    "${report_sha256}" \
    "${report_schema_version}" \
    "${required_runtime_provenance_schema_version}" \
    "${required_reset_strategy_schema_version}" \
    "$(tsv_safe "${current_reset_strategy_id}")" \
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
    "$(tsv_safe "${robot_config_selection}")" \
    "$(tsv_safe "${robot_config}")" \
    "${batch_robot_config_sha256}" \
    "$(tsv_safe "${robot_kinematics_profile_id}")" \
    "$(tsv_safe "${robot_kinematics_lifecycle}")" \
    "${robot_wheel_radius}" \
    "${robot_wheel_width}" \
    "${robot_geometric_track_width}" \
    "${robot_effective_track_width}" \
    "$(tsv_safe "${current_project_config}")" \
    "${current_project_sha256}" \
    "$(tsv_safe "${current_profile_path}")" \
    "${current_profile_sha256}" \
    "$(tsv_safe "${current_ground_topology_id}")" \
    "$(tsv_safe "${current_ground_topology_path}")" \
    "${current_ground_topology_sha256}" \
    "$(tsv_safe "${batch_profile_hashes_json}")" \
    "$(tsv_safe "${current_project_stage}")" \
    "${current_project_stage_sha256}" \
    "$(tsv_safe "${current_source_asset}")" \
    "${current_source_asset_sha256}" \
    "${current_started}/${completed}"
  )
  ((${#manifest_fields[@]} == 47)) || return 1
  if ! row="$(IFS=$'\t'; printf '%s' "${manifest_fields[*]}")"; then
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

reset_strategy_readiness_matches() {
  local payload="$1"
  local payload_sha256="$2"
  local expected_strategy_id="$3"
  local ground_topology_payload="$4"
  python3 - \
    "${payload}" "${payload_sha256}" \
    "${expected_strategy_id}" "${ground_topology_payload}" <<'PY'
import hashlib
import json
import math
import re
import sys

payload, payload_sha256, strategy_id, topology_payload = sys.argv[1:]
try:
    strategy = json.loads(payload)
    topology = json.loads(topology_payload)
    canonical = json.dumps(
        strategy, sort_keys=True, separators=(",", ":"), allow_nan=False
    )
except (TypeError, ValueError):
    raise SystemExit(1)
if canonical != payload:
    raise SystemExit(1)
if hashlib.sha256(payload.encode("utf-8")).hexdigest() != payload_sha256:
    raise SystemExit(1)
if set(strategy) != {
    "schema_version",
    "id",
    "lift_distance_m",
    "separation_step_count",
    "recontact_step_count",
    "contact_probe",
}:
    raise SystemExit(1)
expected_strategies = {
    "pose_restore_v1": {
        "lift_distance_m": 0.0,
        "separation_step_count": 0,
        "recontact_step_count": 1,
    },
    "separate_recontact_0p20m_1step_v1": {
        "lift_distance_m": 0.2,
        "separation_step_count": 1,
        "recontact_step_count": 1,
    },
}
expected = expected_strategies.get(strategy_id)
if expected is None or strategy.get("schema_version") != 1:
    raise SystemExit(1)
if strategy.get("id") != strategy_id:
    raise SystemExit(1)
for field, value in expected.items():
    actual = strategy.get(field)
    if isinstance(value, float):
        if (
            isinstance(actual, bool)
            or not isinstance(actual, (int, float))
            or not math.isfinite(float(actual))
            or float(actual) != value
        ):
            raise SystemExit(1)
    elif isinstance(actual, bool) or actual != value:
        raise SystemExit(1)

probe = strategy.get("contact_probe")
if not isinstance(probe, dict) or set(probe) != {
    "schema_version",
    "enabled",
    "wheel_bindings",
    "wheel_count",
    "ground_filter_paths",
    "ground_filter_count",
    "max_contact_count",
    "report_threshold_n",
    "stage_usd_readback_verified",
}:
    raise SystemExit(1)
wheel_bindings = probe.get("wheel_bindings")
ground_paths = probe.get("ground_filter_paths")
target_colliders = topology.get("target_colliders")
prim_path = re.compile(
    r"^/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*$"
)
if (
    probe.get("schema_version") != 1
    or probe.get("enabled") is not True
    or not isinstance(wheel_bindings, list)
    or len(wheel_bindings) != 4
    or probe.get("wheel_count") != 4
    or not all(
        isinstance(binding, dict)
        and set(binding) == {"joint_name", "wheel_link_path"}
        and isinstance(binding["joint_name"], str)
        and binding["joint_name"]
        and isinstance(binding["wheel_link_path"], str)
        and prim_path.fullmatch(binding["wheel_link_path"])
        for binding in wheel_bindings
    )
    or len({binding["joint_name"] for binding in wheel_bindings}) != 4
    or len({binding["wheel_link_path"] for binding in wheel_bindings}) != 4
    or not isinstance(ground_paths, list)
    or ground_paths != target_colliders
    or ground_paths != sorted(ground_paths)
    or len(set(ground_paths)) != len(ground_paths)
    or not all(isinstance(path, str) and prim_path.fullmatch(path) for path in ground_paths)
    or isinstance(probe.get("ground_filter_count"), bool)
    or probe.get("ground_filter_count") != len(ground_paths)
    or isinstance(probe.get("max_contact_count"), bool)
    or probe.get("max_contact_count") != 128
    or isinstance(probe.get("report_threshold_n"), bool)
    or not isinstance(probe.get("report_threshold_n"), (int, float))
    or not math.isfinite(float(probe.get("report_threshold_n")))
    or float(probe.get("report_threshold_n")) != 0.0
    or probe.get("stage_usd_readback_verified") is not True
):
    raise SystemExit(1)
PY
}

ground_topology_readiness_matches() {
  local payload="$1"
  local payload_sha256="$2"
  local contact_payload="$3"
  local expected_profile_id="$4"
  local expected_profile_path="$5"
  local expected_profile_sha256="$6"
  local expected_environment="$7"
  local expected_source_asset="$8"
  local expected_source_asset_sha256="$9"
  python3 - \
    "${PROJECT_ROOT}" "${payload}" "${payload_sha256}" \
    "${contact_payload}" "${expected_profile_id}" \
    "${expected_profile_path}" "${expected_profile_sha256}" \
    "${expected_environment}" "${expected_source_asset}" \
    "${expected_source_asset_sha256}" <<'PY'
import hashlib
import json
from pathlib import Path
import re
import sys

(
    repository_root,
    payload,
    payload_sha256,
    contact_payload,
    profile_id,
    profile_path,
    profile_sha256,
    environment_id,
    source_asset_path,
    source_asset_sha256,
) = sys.argv[1:]
sys.path.insert(0, repository_root)
from isaac_sim.src.stage.ground_topology import load_ground_topology_profile

try:
    topology = json.loads(payload)
    contact = json.loads(contact_payload)
    canonical = json.dumps(
        topology, sort_keys=True, separators=(",", ":"), allow_nan=False
    )
except (TypeError, ValueError):
    raise SystemExit(1)
if canonical != payload:
    raise SystemExit(1)
if hashlib.sha256(payload.encode("utf-8")).hexdigest() != payload_sha256:
    raise SystemExit(1)
expected_keys = {
    "profile_path",
    "profile_sha256",
    "profile_id",
    "environment_id",
    "operation",
    "source_asset_path",
    "source_asset_sha256",
    "overlay_identifier",
    "overlay_sha256",
    "source_colliders",
    "source_collider_count",
    "source_collider_paths_sha256",
    "target_colliders",
    "target_collider_count",
    "target_collider_paths_sha256",
    "disabled_colliders",
    "disabled_collider_count",
    "disabled_collider_paths_sha256",
    "stage_usd_readback_verified",
}
if set(topology) != expected_keys:
    raise SystemExit(1)
profile = load_ground_topology_profile(profile_path)
expected_identity = {
    "profile_path": str(Path(profile_path).resolve()),
    "profile_sha256": profile_sha256,
    "profile_id": profile_id,
    "environment_id": environment_id,
    "operation": profile.operation,
    "source_asset_path": str(Path(source_asset_path).resolve()),
    "source_asset_sha256": source_asset_sha256,
    "stage_usd_readback_verified": True,
}
if any(topology.get(key) != value for key, value in expected_identity.items()):
    raise SystemExit(1)
if (
    profile.identifier != profile_id
    or profile.environment_id != environment_id
    or profile.sha256 != profile_sha256
    or profile.source_asset_sha256 != source_asset_sha256
):
    raise SystemExit(1)
sha_pattern = re.compile(r"^[0-9a-f]{64}$")
prim_path_pattern = re.compile(
    r"^/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*$"
)
if (
    not isinstance(topology["overlay_identifier"], str)
    or not topology["overlay_identifier"].startswith("anon:")
    or not isinstance(topology["overlay_sha256"], str)
    or not sha_pattern.fullmatch(topology["overlay_sha256"])
):
    raise SystemExit(1)

collider_sets = {}
for name, specification in (
    ("source", profile.source),
    ("target", profile.target),
    ("disabled", profile.disabled),
):
    paths = topology[f"{name}_colliders"]
    count = topology[f"{name}_collider_count"]
    digest = topology[f"{name}_collider_paths_sha256"]
    if (
        not isinstance(paths, list)
        or not all(
            isinstance(path, str) and prim_path_pattern.fullmatch(path)
            for path in paths
        )
        or paths != sorted(paths)
        or len(set(paths)) != len(paths)
        or isinstance(count, bool)
        or not isinstance(count, int)
        or count != len(paths)
        or count != specification.collider_count
        or digest != specification.collider_paths_sha256
        or digest
        != hashlib.sha256(
            json.dumps(paths, separators=(",", ":")).encode("utf-8")
        ).hexdigest()
    ):
        raise SystemExit(1)
    required = set(getattr(specification, "required_prim_paths", ()))
    if not required.issubset(paths):
        raise SystemExit(1)
    collider_sets[name] = set(paths)

source = collider_sets["source"]
target = collider_sets["target"]
disabled = collider_sets["disabled"]
if target & disabled or target | disabled != source:
    raise SystemExit(1)
if profile.operation == "preserve_source_colliders":
    if target != source or disabled:
        raise SystemExit(1)
elif not target < source or disabled != source - target:
    raise SystemExit(1)
if contact.get("ground_colliders") != topology["target_colliders"]:
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
  local ground_topology_json ground_topology_sha256
  local reset_strategy_json reset_strategy_sha256
  local actual_robot_config actual_robot_config_sha256
  local actual_robot_asset actual_robot_asset_sha256
  local actual_kinematics_profile actual_kinematics_lifecycle
  local actual_wheel_radius actual_wheel_width
  local actual_geometric_track_width actual_effective_track_width
  local controller_contract_verified
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
    if [[ "${schema}" != 6 ]]; then
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
    ground_topology_json="$(
      ros_parameter runtime_provenance.ground_topology.json || true
    )"
    ground_topology_sha256="$(
      ros_parameter runtime_provenance.ground_topology.sha256 || true
    )"
    reset_strategy_json="$(
      ros_parameter runtime_provenance.simulation.reset_strategy.json || true
    )"
    reset_strategy_sha256="$(
      ros_parameter runtime_provenance.simulation.reset_strategy.sha256 || true
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
    actual_kinematics_profile="$(
      ros_parameter runtime_provenance.robot.kinematics.profile_id || true
    )"
    actual_kinematics_lifecycle="$(
      ros_parameter runtime_provenance.robot.kinematics.lifecycle || true
    )"
    actual_wheel_radius="$(
      ros_parameter runtime_provenance.robot.kinematics.wheel_radius_m || true
    )"
    actual_wheel_width="$(
      ros_parameter runtime_provenance.robot.kinematics.wheel_width_m || true
    )"
    actual_geometric_track_width="$(
      ros_parameter \
        runtime_provenance.robot.kinematics.geometric_track_width_m || true
    )"
    actual_effective_track_width="$(
      ros_parameter \
        runtime_provenance.robot.kinematics.effective_track_width_m || true
    )"
    controller_contract_verified="$(
      ros_parameter \
        runtime_provenance.robot.kinematics.controller_contract_verified \
        || true
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
          && "${actual_kinematics_profile}" == "${robot_kinematics_profile_id}" \
          && "${actual_kinematics_lifecycle}" == "${robot_kinematics_lifecycle}" \
          && "${actual_wheel_radius}" == "${robot_wheel_radius}" \
          && "${actual_wheel_width}" == "${robot_wheel_width}" \
          && "${actual_geometric_track_width}" == "${robot_geometric_track_width}" \
          && "${actual_effective_track_width}" == "${robot_effective_track_width}" \
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
        && ros_parameter_boolean_matches \
          "${controller_contract_verified}" true \
        && ros_parameter_boolean_matches "${provenance_dirty}" false \
        && contact_readiness_matches \
          "${contact_json}" "${contact_sha256}" \
          "${profile_id}" "${profile_mode}" \
          "${profile_path}" "${profile_sha256}" \
        && ground_topology_readiness_matches \
          "${ground_topology_json}" "${ground_topology_sha256}" \
          "${contact_json}" "${current_ground_topology_id}" \
          "${current_ground_topology_path}" \
          "${current_ground_topology_sha256}" \
          "${expected_environment}" "${current_source_asset}" \
          "${current_source_asset_sha256}" \
        && reset_strategy_readiness_matches \
          "${reset_strategy_json}" "${reset_strategy_sha256}" \
          "${current_reset_strategy_id}" \
          "${ground_topology_json}"; then
      return 0
    fi
    sleep 0.2
  done
  return 1
}

launch_isaac() {
  local project_config="$1"
  local profile_path="$2"
  local ground_topology_path="$3"
  local reset_strategy_id="$4"
  local log_path="$5"
  (
    close_instance_lock_fds_for_child
    unset ISAAC_NAV_DEDICATED_PROCESS_GROUP
    clear_inherited_config_overrides
    export ISAAC_NAV_PROJECT_CONFIG="${project_config}"
    export ISAAC_NAV__FILES__CONTACT_PROFILE="${profile_path}"
    export ISAAC_NAV__FILES__GROUND_TOPOLOGY_PROFILE="${ground_topology_path}"
    export ISAAC_NAV__FILES__ROBOT="${robot_config}"
    [[ "${reset_strategy_id}" == "${current_reset_strategy_id}" ]] || return 1
    export ISAAC_NAV__SIMULATION__RESET_STRATEGY__ID="${reset_strategy_id}"
    # Schema-v6 provenance verifies mapping/ideal/60 Hz, kinematics, and the
    # selected Reset strategy below.
    # It does not
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
  local ground_topology_id="$7"
  local ground_topology_path="$8"
  local ground_topology_sha256="$9"
  local motion_sha256="${10}"
  local reset_strategy_id="${11}"
  python3 - \
    "${PROJECT_ROOT}" "${report_path}" "${environment_id}" \
    "${profile_id}" "${profile_mode}" \
    "${profile_path}" "${profile_sha256}" \
    "${ground_topology_id}" "${ground_topology_path}" \
    "${ground_topology_sha256}" "${motion_sha256}" \
    "${batch_git_commit}" "${batch_git_branch}" \
    "${robot_config}" "${robot_config_sha256}" \
    "${robot_asset}" "${robot_asset_sha256}" \
    "${current_project_stage}" "${current_project_stage_sha256}" \
    "${current_source_asset}" "${current_source_asset_sha256}" \
    "${robot_kinematics_profile_id}" "${robot_kinematics_lifecycle}" \
    "${robot_wheel_radius}" "${robot_wheel_width}" \
    "${robot_geometric_track_width}" "${robot_effective_track_width}" \
    "${reset_strategy_id}" <<'PY'
import json
from pathlib import Path
import sys

repository_root = Path(sys.argv[1])
path = Path(sys.argv[2])
sys.path.insert(0, str(repository_root))
environment_id, profile_id, profile_mode = sys.argv[3:6]
profile_path, profile_sha256 = sys.argv[6:8]
ground_topology_id, ground_topology_path, ground_topology_sha256 = sys.argv[8:11]
motion_sha256 = sys.argv[11]
git_commit, git_branch = sys.argv[12:14]
robot_config, robot_config_sha256 = sys.argv[14:16]
robot_asset, robot_asset_sha256 = sys.argv[16:18]
project_stage, project_stage_sha256 = sys.argv[18:20]
source_asset, source_asset_sha256 = sys.argv[20:22]
kinematics_profile_id, kinematics_lifecycle = sys.argv[22:24]
wheel_radius, wheel_width = map(float, sys.argv[24:26])
geometric_track_width, effective_track_width = map(float, sys.argv[26:28])
reset_strategy_id = sys.argv[28]
reset_strategy_token = f"reset-v1-{reset_strategy_id}"
if not path.is_file():
    raise SystemExit("motion report is missing")
try:
    report = json.loads(path.read_text(encoding="utf-8"))
except (OSError, ValueError) as exc:
    raise SystemExit(f"motion report is not valid JSON: {exc}")
if report.get("schema_version") != 3:
    raise SystemExit("motion report schema must be integer 3")

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
        wheel_radius,
        min_repeats=1,
        expected_environments=(environment_id,),
        expected_topologies=(ground_topology_id,),
        expected_reset_strategies=(reset_strategy_token,),
        expected_profiles=(profile_id,),
    )
except Exception as exc:
    raise SystemExit(f"strict contact A/B report validation failed: {exc}")
if analysis.get("analysis_valid") is not True:
    raise SystemExit("strict contact A/B report validation excluded the report")
physical_acceptance = analysis.get("physical_acceptance", {})
if analysis.get("schema_version") != 5:
    raise SystemExit("strict contact A/B analysis schema must be integer 5")
if (
    not isinstance(physical_acceptance, dict)
    or physical_acceptance.get("schema_version") != 3
    or physical_acceptance.get("policy_id") != "skid_steer_plan_8_7_v3"
):
    raise SystemExit("strict contact A/B physical acceptance contract mismatch")
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
ground_topology = provenance.get("ground_topology", {})
environment = provenance.get("environment", {})
robot = provenance.get("robot", {})
simulation = provenance.get("simulation", {})
git = provenance.get("git", {})
if provenance.get("schema_version") != 6:
    raise SystemExit("runtime provenance schema must be integer 6")
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
expected_kinematics = {
    "profile_id": kinematics_profile_id,
    "lifecycle": kinematics_lifecycle,
    "wheel_radius_m": wheel_radius,
    "wheel_width_m": wheel_width,
    "geometric_track_width_m": geometric_track_width,
    "effective_track_width_m": effective_track_width,
    "controller_contract_verified": True,
}
if robot.get("kinematics") != expected_kinematics:
    raise SystemExit("runtime provenance robot kinematics mismatch")
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
reset_strategy = simulation.get("reset_strategy")
if (
    not isinstance(reset_strategy, dict)
    or reset_strategy.get("schema_version") != 1
    or reset_strategy.get("id") != reset_strategy_id
):
    raise SystemExit("runtime provenance reset strategy mismatch")
expected_contact = {
    "profile_id": profile_id,
    "profile_mode": profile_mode,
    "profile_path": profile_path,
    "profile_sha256": profile_sha256,
    "stage_usd_readback_verified": True,
}
if any(contact.get(key) != value for key, value in expected_contact.items()):
    raise SystemExit("runtime provenance contact profile mismatch")

from isaac_sim.src.stage.ground_topology import load_ground_topology_profile

topology_profile = load_ground_topology_profile(ground_topology_path)
expected_topology_identity = {
    "profile_path": ground_topology_path,
    "profile_sha256": ground_topology_sha256,
    "profile_id": ground_topology_id,
    "environment_id": environment_id,
    "operation": topology_profile.operation,
    "source_asset_path": source_asset,
    "source_asset_sha256": source_asset_sha256,
    "stage_usd_readback_verified": True,
}
if any(
    ground_topology.get(key) != value
    for key, value in expected_topology_identity.items()
):
    raise SystemExit("runtime provenance ground topology identity mismatch")
if (
    topology_profile.identifier != ground_topology_id
    or topology_profile.environment_id != environment_id
    or topology_profile.sha256 != ground_topology_sha256
    or topology_profile.source_asset_sha256 != source_asset_sha256
):
    raise SystemExit("ground topology profile contract mismatch")
for name, specification in (
    ("source", topology_profile.source),
    ("target", topology_profile.target),
    ("disabled", topology_profile.disabled),
):
    paths = ground_topology.get(f"{name}_colliders")
    if (
        ground_topology.get(f"{name}_collider_count")
        != specification.collider_count
        or ground_topology.get(f"{name}_collider_paths_sha256")
        != specification.collider_paths_sha256
        or not isinstance(paths, list)
        or not set(getattr(specification, "required_prim_paths", ())).issubset(
            paths
        )
    ):
        raise SystemExit(
            f"runtime provenance ground topology {name} contract mismatch"
        )
if ground_topology.get("target_colliders") != contact.get("ground_colliders"):
    raise SystemExit("ground topology target/contact ground mismatch")
PY
}

run_one_condition() {
  local sequence="$1"
  local environment_id="$2"
  local ground_topology_id="$3"
  local profile_id="$4"
  local profile_mode="$5"
  local reset_strategy_id="$6"
  local repeat="$7"
  local slug project_config profile_path topology_path runner_status

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
  topology_path="$(ground_topology_path "${ground_topology_id}")" || {
    current_failure_reason="ground_topology_path_resolution_failed"
    return 1
  }
  case "${environment_id}:${ground_topology_id}" in
    SimplePlane:simple_plane_only1_v1|\
      Warehouse:warehouse_combined32_v1|\
      Warehouse:warehouse_plane_only1_v1) ;;
    *)
      current_failure_reason="illegal_environment_ground_topology_pair"
      return 1
      ;;
  esac
  current_reset_strategy_id="${reset_strategy_id}"
  current_reset_strategy_token="reset-v1-${reset_strategy_id}"
  if ! printf -v current_run_id '%03d_%s_%s_%s_%s_r%02d' \
      "${sequence}" "${slug}" "${ground_topology_id}" \
      "${current_reset_strategy_token}" "${profile_id}" "${repeat}"; then
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
  current_ground_topology_id="${ground_topology_id}"
  current_ground_topology_path="${topology_path}"
  current_ground_topology_sha256="${locked_input_hashes[${topology_path}]}"
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
      "${project_config}" "${profile_path}" "${topology_path}" \
      "${reset_strategy_id}" \
      "${current_isaac_log}"; then
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
      "${ground_topology_id}" "${topology_path}" \
      "${current_ground_topology_sha256}" \
      "${batch_motion_sha256}" "${reset_strategy_id}"; then
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
  header="${manifest_header_contract}"
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
    "${PROJECT_ROOT}" "${manifest}" "${manifest_header_contract}" \
    "${analysis_path}" \
    "${environment_selection}" "${ground_topology_selection}" \
    "${batch_environment_topology_pairs_json}" "${repeats}" \
    "${expected_conditions}" "${expected_groups}" \
    "${robot_wheel_radius}" "${batch_profile_hashes_json}" \
    "${batch_reset_strategy_ids_json}" <<'PY'
import csv
import hashlib
import json
from pathlib import Path
import sys

(
    repository_root,
    manifest_name,
    manifest_header,
    output_name,
    environment_selection,
    ground_topology_selection,
    selected_pairs_json,
    repeats_text,
    expected_runs_text,
    expected_groups_text,
    wheel_radius_text,
    profile_hashes_json,
    reset_strategy_ids_json,
) = sys.argv[1:]
repeats = int(repeats_text)
expected_runs = int(expected_runs_text)
expected_groups = int(expected_groups_text)
wheel_radius = float(wheel_radius_text)
manifest_path = Path(manifest_name)
output_path = Path(output_name)
expected_manifest_fieldnames = manifest_header.split("\t")
selected_pairs_data = json.loads(selected_pairs_json)
profile_hashes = json.loads(profile_hashes_json)
reset_strategy_ids = json.loads(reset_strategy_ids_json)
selected_profiles = tuple(profile_hashes)
selected_reset_tokens = tuple(
    f"reset-v1-{strategy_id}" for strategy_id in reset_strategy_ids
)
selected_pairs = {
    (pair["environment_id"], pair["ground_topology_id"])
    for pair in selected_pairs_data
}
if (
    not selected_pairs
    or len(selected_pairs) != len(selected_pairs_data)
    or not selected_profiles
    or not selected_reset_tokens
):
    raise SystemExit("selected environment/topology pairs must be unique")

# Use the committed workspace source explicitly; the install tree may be stale.
source_root = Path(repository_root) / "ros2_ws/src/robot_experiments"
sys.path.insert(0, str(source_root))
from robot_experiments.contact_ab_analysis import (
    analyse_contact_ab,
    validate_physical_acceptance_accounting,
    write_contact_ab_report,
)

with manifest_path.open("r", encoding="utf-8", newline="") as stream:
    reader = csv.DictReader(stream, delimiter="\t", strict=True)
    if reader.fieldnames != expected_manifest_fieldnames:
        raise SystemExit("manifest header does not match the 47-column contract")
    rows = list(reader)
if len(rows) != expected_runs:
    raise SystemExit(
        f"manifest run count mismatch: {len(rows)} != {expected_runs}"
    )
report_paths = []
manifest_report_locks = {}
for row_index, row in enumerate(rows, start=1):
    if None in row or any(value is None for value in row.values()):
        raise SystemExit(
            f"manifest row {row_index} has missing or extra fields"
        )
    if row.get("status") != "success":
        raise SystemExit(f"manifest row {row_index} is not successful")
    report_path = Path(row.get("report", ""))
    if not report_path.is_file() or report_path.is_symlink():
        raise SystemExit(f"manifest row {row_index} report path is unsafe")
    actual_sha256 = hashlib.sha256(report_path.read_bytes()).hexdigest()
    if row.get("report_sha256") != actual_sha256:
        raise SystemExit(f"manifest row {row_index} report SHA256 mismatch")
    if row.get("report_schema_version") != "3":
        raise SystemExit(
            f"manifest row {row_index} report schema version must be 3"
        )
    if row.get("runtime_provenance_schema_version") != "6":
        raise SystemExit(
            f"manifest row {row_index} runtime provenance schema version must be 6"
        )
    if row.get("reset_strategy_schema_version") != "1":
        raise SystemExit(
            f"manifest row {row_index} reset strategy schema version must be 1"
        )
    try:
        report_document = json.loads(report_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError) as exc:
        raise SystemExit(
            f"manifest row {row_index} report is not valid JSON: {exc}"
        ) from exc
    if report_document.get("schema_version") != 3:
        raise SystemExit(
            f"manifest row {row_index} report JSON schema version mismatch"
        )
    provenance = report_document.get("runtime_provenance")
    simulation = provenance.get("simulation") if isinstance(provenance, dict) else None
    reset_strategy = (
        simulation.get("reset_strategy") if isinstance(simulation, dict) else None
    )
    reset_strategy_id = row.get("reset_strategy_id")
    reset_strategy_token = f"reset-v1-{reset_strategy_id}"
    if (
        not isinstance(provenance, dict)
        or provenance.get("schema_version") != 6
        or not isinstance(reset_strategy, dict)
        or reset_strategy.get("schema_version") != 1
        or reset_strategy.get("id") != reset_strategy_id
        or reset_strategy_token not in selected_reset_tokens
    ):
        raise SystemExit(
            f"manifest row {row_index} report reset strategy identity mismatch"
        )
    row_pair = (row.get("environment"), row.get("ground_topology_id"))
    if row_pair not in selected_pairs:
        raise SystemExit(
            f"manifest row {row_index} has an unselected environment/topology pair"
        )
    canonical_report_path = str(report_path.resolve())
    if canonical_report_path in manifest_report_locks:
        raise SystemExit("manifest report paths must be unique")
    manifest_report_locks[canonical_report_path] = {
        "sha256": actual_sha256,
        "report_schema_version": 3,
        "runtime_provenance_schema_version": 6,
        "reset_strategy_schema_version": 1,
        "reset_strategy_id": reset_strategy_id,
        "reset_strategy_token": reset_strategy_token,
        "environment_id": row.get("environment"),
        "ground_topology_id": row.get("ground_topology_id"),
        "contact_profile_id": row.get("profile_id"),
    }
    report_paths.append(Path(canonical_report_path))
if len(set(report_paths)) != expected_runs:
    raise SystemExit("manifest report paths must be unique")

if environment_selection not in {"all", "Warehouse", "SimplePlane"}:
    raise SystemExit("unknown batch environment selection")
if ground_topology_selection not in {
    "baseline",
    "all",
    "simple_plane_only1_v1",
    "warehouse_combined32_v1",
    "warehouse_plane_only1_v1",
}:
    raise SystemExit("unknown batch ground-topology selection")
selected_environments = tuple(
    dict.fromkeys(pair["environment_id"] for pair in selected_pairs_data)
)
selected_topologies = tuple(
    dict.fromkeys(pair["ground_topology_id"] for pair in selected_pairs_data)
)
arguments = {
    "min_repeats": repeats,
    "expected_environments": selected_environments,
    "expected_topologies": selected_topologies,
    "expected_reset_strategies": selected_reset_tokens,
    "expected_profiles": selected_profiles,
}
analysis = analyse_contact_ab(report_paths, wheel_radius, **arguments)
counts = analysis.get("counts", {})
selection = analysis.get("selection", {})
physical_acceptance = analysis.get("physical_acceptance", {})
expected_physical_thresholds = {
    "forward_abs_lateral_drift_max_m": 0.05,
    "backward_abs_lateral_drift_max_m": 0.08,
    "rotation_center_drift_max_m": 0.10,
    "rotation_center_drift_asymmetry_ratio_max": 0.20,
    "rotation_mean_yaw_rate_absolute_error_fraction_max": 0.10,
    "stop_stable_duration_min_sec": 0.5,
    "stop_linear_velocity_threshold_max_mps": 0.02,
    "stop_angular_velocity_threshold_max_radps": 0.05,
    "stop_wheel_velocity_threshold_max_radps": 0.20,
}
if analysis.get("analysis_valid") is not True:
    raise SystemExit("aggregate contact A/B analysis is not valid")
if analysis.get("schema_version") != 5:
    raise SystemExit("aggregate contact A/B analysis schema must be 5")
if counts.get("excluded_reports") != 0 or selection.get("excluded") != []:
    raise SystemExit("aggregate contact A/B analysis excluded reports")
if counts.get("included_reports") != expected_runs:
    raise SystemExit("aggregate included-report count mismatch")
if counts.get("groups") != expected_groups:
    raise SystemExit("aggregate group count mismatch")
selection_included = selection.get("included")
if (
    not isinstance(selection_included, list)
    or len(selection_included) != expected_runs
):
    raise SystemExit("aggregate selection included-report count mismatch")
analysis_report_locks = {}
for selection_index, included in enumerate(selection_included, start=1):
    if not isinstance(included, dict):
        raise SystemExit(
            f"aggregate selection item {selection_index} must be an object"
        )
    included_path = included.get("path")
    if not isinstance(included_path, str):
        raise SystemExit(
            f"aggregate selection item {selection_index} path is invalid"
        )
    if included_path in analysis_report_locks:
        raise SystemExit("aggregate selection report paths must be unique")
    analysis_report_locks[included_path] = {
        "sha256": included.get("sha256"),
        "report_schema_version": included.get("report_schema_version"),
        "runtime_provenance_schema_version": included.get(
            "runtime_provenance_schema_version"
        ),
        "reset_strategy_schema_version": included.get(
            "reset_strategy_schema_version"
        ),
        "reset_strategy_id": included.get("reset_strategy_id"),
        "reset_strategy_token": included.get("reset_strategy_token"),
        "environment_id": included.get("environment_id"),
        "ground_topology_id": included.get("ground_topology_id"),
        "contact_profile_id": included.get("contact_profile_id"),
    }
if analysis_report_locks != manifest_report_locks:
    raise SystemExit(
        "aggregate selection does not match frozen manifest/report identities"
    )
if (
    not isinstance(physical_acceptance, dict)
    or physical_acceptance.get("schema_version") != 3
    or physical_acceptance.get("policy_id") != "skid_steer_plan_8_7_v3"
    or physical_acceptance.get("evaluation_basis") != "every_repeat"
    or physical_acceptance.get("ranking_policy") != "none; pass/fail only"
    or physical_acceptance.get("steady_state_measurement_basis")
    != "actual_velocity.steady_state_window.angular_z_radps.mean over the final_half_of_command_interval window"
    or physical_acceptance.get("wheel_direction_measurement_basis")
    != "wheels.steady_state_window.per_wheel[*].direction_matches over the final_half_of_command_interval window"
    or physical_acceptance.get("thresholds") != expected_physical_thresholds
    or physical_acceptance.get("applicability") != {
        "required_motion_report_schema": 3,
        "required_runtime_provenance_schema": 6,
        "required_environment_id": "SimplePlane",
        "required_ground_topology_id": "simple_plane_only1_v1",
        "required_odometry_mode": "ideal",
        "minimum_unique_repeats_per_group": 3,
    }
):
    raise SystemExit("aggregate physical acceptance contract is invalid")
acceptance_groups = physical_acceptance.get("groups")
passing_groups = physical_acceptance.get("passing_groups")
failed_groups = physical_acceptance.get("failed_groups")
applicable_groups = physical_acceptance.get("applicable_groups")
not_applicable_groups = physical_acceptance.get("not_applicable_groups")
analysis_groups = analysis.get("groups")
if (
    not isinstance(acceptance_groups, dict)
    or not isinstance(analysis_groups, dict)
    or set(acceptance_groups) != set(analysis_groups)
    or not isinstance(passing_groups, list)
    or not isinstance(failed_groups, list)
    or not isinstance(applicable_groups, list)
    or not isinstance(not_applicable_groups, list)
    or len(acceptance_groups) != expected_groups
    or any(
        not isinstance(group, str)
        for group in (
            passing_groups
            + failed_groups
            + applicable_groups
            + not_applicable_groups
        )
    )
    or len(set(passing_groups)) != len(passing_groups)
    or len(set(failed_groups)) != len(failed_groups)
    or len(set(applicable_groups)) != len(applicable_groups)
    or len(set(not_applicable_groups)) != len(not_applicable_groups)
    or set(passing_groups) & set(failed_groups)
    or set(applicable_groups) & set(not_applicable_groups)
    or set(passing_groups) | set(failed_groups) != set(applicable_groups)
    or set(applicable_groups) | set(not_applicable_groups)
    != set(acceptance_groups)
    or any(
        not isinstance(group_result, dict)
        or group_result.get("applicable") is not (group_id in applicable_groups)
        or (
            group_id in applicable_groups
            and group_result.get("passed") is not (group_id in passing_groups)
        )
        or (
            group_id in not_applicable_groups
            and (
                group_result.get("passed") is not None
                or not isinstance(
                    group_result.get("not_applicable_reasons"), list
                )
                or not group_result.get("not_applicable_reasons")
            )
        )
        for group_id, group_result in acceptance_groups.items()
    )
    or physical_acceptance.get("all_applicable_groups_passed")
    is not (None if not applicable_groups else not failed_groups)
):
    raise SystemExit("aggregate physical acceptance group accounting is invalid")
try:
    validate_physical_acceptance_accounting(analysis, repeats)
except Exception as exc:
    raise SystemExit(
        f"aggregate every-repeat physical acceptance evidence is invalid: {exc}"
    ) from exc
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
  validate_success_manifest_evidence "summary" || return 1
  python3 - \
    "${batch_summary_path}" \
    "${PROJECT_ROOT}" "${manifest_header_contract}" \
    "${environment_selection}" "${ground_topology_selection}" \
    "${contact_profile_selection}" "${reset_strategy_selection}" \
    "${batch_reset_strategy_ids_json}" \
    "${batch_environment_topology_pairs_json}" "${repeats}" \
    "${expected_conditions}" "${expected_groups}" \
    "${successful_rows}" "${manifest_rows}" \
    "${batch_git_commit}" "${batch_git_branch}" \
    "${motion_config}" "${batch_motion_sha256}" \
    "${warehouse_config}" "${batch_warehouse_project_sha256}" \
    "${simple_plane_config}" "${batch_simple_plane_project_sha256}" \
    "${robot_config_selection}" \
    "${robot_config}" "${batch_robot_config_sha256}" \
    "${robot_asset}" "${robot_asset_sha256}" \
    "${robot_kinematics_profile_id}" \
    "${robot_kinematics_lifecycle}" \
    "${robot_wheel_radius}" "${robot_wheel_width}" \
    "${robot_geometric_track_width}" \
    "${robot_effective_track_width}" \
    "${physics_dir}" "${batch_profile_hashes_json}" \
    "${ground_topology_dir}" "${batch_ground_topology_hashes_json}" \
    "${manifest}" "${frozen_manifest_sha256}" \
    "${analysis_path}" "${analysis_sha256}" <<'PY'
import csv
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile

(
    output_name,
    repository_root_text,
    manifest_header,
    environment_selection,
    ground_topology_selection,
    contact_profile_selection,
    reset_strategy_selection,
    reset_strategy_ids_json,
    environment_topology_pairs_json,
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
    robot_config_selection,
    robot_config_path,
    robot_config_sha256,
    robot_asset_path,
    robot_asset_sha256,
    robot_kinematics_profile_id,
    robot_kinematics_lifecycle,
    robot_wheel_radius_text,
    robot_wheel_width_text,
    robot_geometric_track_width_text,
    robot_effective_track_width_text,
    physics_directory,
    profile_hashes_json,
    ground_topology_directory,
    ground_topology_hashes_json,
    manifest_name,
    manifest_sha256,
    analysis_name,
    analysis_sha256,
) = sys.argv[1:]
output_path = Path(output_name)
manifest_path = Path(manifest_name)
analysis_path = Path(analysis_name)
expected_manifest_fieldnames = manifest_header.split("\t")
reset_strategy_ids = json.loads(reset_strategy_ids_json)
reset_strategy_tokens = [
    f"reset-v1-{strategy_id}" for strategy_id in reset_strategy_ids
]
source_root = Path(repository_root_text) / "ros2_ws/src/robot_experiments"
sys.path.insert(0, str(source_root))
from robot_experiments.contact_ab_analysis import (
    validate_physical_acceptance_accounting,
)

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
physical_acceptance = analysis.get("physical_acceptance", {})
expected_physical_thresholds = {
    "forward_abs_lateral_drift_max_m": 0.05,
    "backward_abs_lateral_drift_max_m": 0.08,
    "rotation_center_drift_max_m": 0.10,
    "rotation_center_drift_asymmetry_ratio_max": 0.20,
    "rotation_mean_yaw_rate_absolute_error_fraction_max": 0.10,
    "stop_stable_duration_min_sec": 0.5,
    "stop_linear_velocity_threshold_max_mps": 0.02,
    "stop_angular_velocity_threshold_max_radps": 0.05,
    "stop_wheel_velocity_threshold_max_radps": 0.20,
}
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
acceptance_groups = physical_acceptance.get("groups")
passing_groups = physical_acceptance.get("passing_groups")
failed_groups = physical_acceptance.get("failed_groups")
applicable_groups = physical_acceptance.get("applicable_groups")
not_applicable_groups = physical_acceptance.get("not_applicable_groups")
if (
    analysis.get("schema_version") != 5
    or physical_acceptance.get("schema_version") != 3
    or physical_acceptance.get("policy_id") != "skid_steer_plan_8_7_v3"
    or physical_acceptance.get("evaluation_basis") != "every_repeat"
    or physical_acceptance.get("ranking_policy") != "none; pass/fail only"
    or physical_acceptance.get("steady_state_measurement_basis")
    != "actual_velocity.steady_state_window.angular_z_radps.mean over the final_half_of_command_interval window"
    or physical_acceptance.get("wheel_direction_measurement_basis")
    != "wheels.steady_state_window.per_wheel[*].direction_matches over the final_half_of_command_interval window"
    or physical_acceptance.get("thresholds") != expected_physical_thresholds
    or physical_acceptance.get("applicability") != {
        "required_motion_report_schema": 3,
        "required_runtime_provenance_schema": 6,
        "required_environment_id": "SimplePlane",
        "required_ground_topology_id": "simple_plane_only1_v1",
        "required_odometry_mode": "ideal",
        "minimum_unique_repeats_per_group": 3,
    }
    or not isinstance(acceptance_groups, dict)
    or len(acceptance_groups) != expected_groups
    or not isinstance(passing_groups, list)
    or not isinstance(failed_groups, list)
    or not isinstance(applicable_groups, list)
    or not isinstance(not_applicable_groups, list)
    or any(
        not isinstance(group, str)
        for group in (
            passing_groups
            + failed_groups
            + applicable_groups
            + not_applicable_groups
        )
    )
    or len(set(passing_groups)) != len(passing_groups)
    or len(set(failed_groups)) != len(failed_groups)
    or len(set(applicable_groups)) != len(applicable_groups)
    or len(set(not_applicable_groups)) != len(not_applicable_groups)
    or set(passing_groups) & set(failed_groups)
    or set(applicable_groups) & set(not_applicable_groups)
    or set(passing_groups) | set(failed_groups) != set(applicable_groups)
    or set(applicable_groups) | set(not_applicable_groups)
    != set(acceptance_groups)
    or any(
        not isinstance(group_result, dict)
        or group_result.get("applicable") is not (group_id in applicable_groups)
        or (
            group_id in applicable_groups
            and group_result.get("passed") is not (group_id in passing_groups)
        )
        or (
            group_id in not_applicable_groups
            and (
                group_result.get("passed") is not None
                or not isinstance(
                    group_result.get("not_applicable_reasons"), list
                )
                or not group_result.get("not_applicable_reasons")
            )
        )
        for group_id, group_result in acceptance_groups.items()
    )
    or physical_acceptance.get("all_applicable_groups_passed")
    is not (None if not applicable_groups else not failed_groups)
):
    raise SystemExit("batch physical acceptance accounting is invalid")
try:
    validate_physical_acceptance_accounting(analysis, int(repeats_text))
except Exception as exc:
    raise SystemExit(
        f"batch every-repeat physical acceptance evidence is invalid: {exc}"
    ) from exc
profile_hashes = json.loads(profile_hashes_json)
if not isinstance(profile_hashes, dict) or not profile_hashes:
    raise SystemExit("locked profile hash map must contain selected profiles")
complete_profile_ids = {
    "legacy_baseline",
    "threshold_corr_0p00025_offset_0p0004",
    "threshold_corr_0p025_offset_0p0004",
    "threshold_corr_0p00025_offset_0p04",
    "threshold_corr_0p025_offset_0p04",
    "explicit_material",
}
if (
    not set(profile_hashes).issubset(complete_profile_ids)
    or (
        contact_profile_selection == "all"
        and set(profile_hashes) != complete_profile_ids
    )
    or (
        contact_profile_selection != "all"
        and set(profile_hashes) != {contact_profile_selection}
    )
):
    raise SystemExit("locked profile hash map contradicts contact selection")
supported_reset_strategies = {
    "pose_restore_v1",
    "separate_recontact_0p20m_1step_v1",
}
expected_reset_selections = {
    "pose_restore_v1": ["pose_restore_v1"],
    "separate_recontact_0p20m_1step_v1": [
        "separate_recontact_0p20m_1step_v1"
    ],
    "all": [
        "pose_restore_v1",
        "separate_recontact_0p20m_1step_v1",
    ],
}
if (
    (
        reset_strategy_selection == "project"
        and (
            len(reset_strategy_ids) != 1
            or reset_strategy_ids[0] not in supported_reset_strategies
        )
    )
    or (
        reset_strategy_selection != "project"
        and (
            reset_strategy_selection not in expected_reset_selections
            or reset_strategy_ids
            != expected_reset_selections[reset_strategy_selection]
        )
    )
):
    raise SystemExit("locked reset strategies contradict reset selection")
profiles = {
    profile_id: {
        "path": str(Path(physics_directory) / f"{profile_id}.yaml"),
        "sha256": digest,
    }
    for profile_id, digest in sorted(profile_hashes.items())
}
environment_topology_pairs = json.loads(environment_topology_pairs_json)
ground_topology_hashes = json.loads(ground_topology_hashes_json)
if (
    not isinstance(environment_topology_pairs, list)
    or not environment_topology_pairs
    or not isinstance(ground_topology_hashes, dict)
    or not ground_topology_hashes
):
    raise SystemExit("locked ground-topology selection is empty or invalid")
selected_environments = list(
    dict.fromkeys(
        pair["environment_id"] for pair in environment_topology_pairs
    )
)
selected_topologies = list(
    dict.fromkeys(
        pair["ground_topology_id"] for pair in environment_topology_pairs
    )
)
if set(selected_topologies) != set(ground_topology_hashes):
    raise SystemExit("locked ground-topology hashes do not match selected pairs")
if expected_groups != (
    len(environment_topology_pairs)
    * len(profile_hashes)
    * len(reset_strategy_ids)
):
    raise SystemExit("expected group count does not match selected topology pairs")
if expected_runs != expected_groups * int(repeats_text):
    raise SystemExit("expected run count does not match matrix cardinality")
topology_environments = {
    pair["ground_topology_id"]: pair["environment_id"]
    for pair in environment_topology_pairs
}
ground_topology_profiles = {
    topology_id: {
        "environment_id": topology_environments[topology_id],
        "path": str(
            Path(ground_topology_directory) / f"{topology_id}.yaml"
        ),
        "sha256": digest,
    }
    for topology_id, digest in sorted(ground_topology_hashes.items())
}
selected_pairs = {
    (pair["environment_id"], pair["ground_topology_id"])
    for pair in environment_topology_pairs
}
with manifest_path.open("r", encoding="utf-8", newline="") as stream:
    manifest_reader = csv.DictReader(stream, delimiter="\t", strict=True)
    if manifest_reader.fieldnames != expected_manifest_fieldnames:
        raise SystemExit(
            "frozen manifest header does not match the 47-column contract"
        )
    manifest_documents = list(manifest_reader)
if len(manifest_documents) != expected_runs:
    raise SystemExit("frozen manifest row count does not match expected runs")
manifest_report_locks = {}
for row_index, row in enumerate(manifest_documents, start=1):
    if None in row or any(value is None for value in row.values()):
        raise SystemExit(
            f"frozen manifest row {row_index} has missing or extra fields"
        )
    topology_id = row.get("ground_topology_id")
    expected_topology_path = str(
        Path(ground_topology_directory) / f"{topology_id}.yaml"
    )
    if (
        (row.get("environment"), topology_id) not in selected_pairs
        or topology_id not in ground_topology_hashes
        or row.get("ground_topology_profile_path") != expected_topology_path
        or row.get("ground_topology_profile_sha256")
        != ground_topology_hashes[topology_id]
    ):
        raise SystemExit(
            f"manifest row {row_index} ground-topology identity mismatch"
        )
    report_path = Path(row.get("report", ""))
    if not report_path.is_file() or report_path.is_symlink():
        raise SystemExit(f"manifest row {row_index} report path is unsafe")
    report_sha256 = file_sha256(report_path)
    if row.get("report_sha256") != report_sha256:
        raise SystemExit(f"manifest row {row_index} report SHA256 mismatch")
    if row.get("report_schema_version") != "3":
        raise SystemExit(
            f"manifest row {row_index} report schema version must be 3"
        )
    if row.get("runtime_provenance_schema_version") != "6":
        raise SystemExit(
            f"manifest row {row_index} runtime provenance schema version must be 6"
        )
    if row.get("reset_strategy_schema_version") != "1":
        raise SystemExit(
            f"manifest row {row_index} reset strategy schema version must be 1"
        )
    try:
        report_document = json.loads(report_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError) as exc:
        raise SystemExit(
            f"manifest row {row_index} report is not valid JSON: {exc}"
        ) from exc
    if report_document.get("schema_version") != 3:
        raise SystemExit(
            f"manifest row {row_index} report JSON schema version mismatch"
        )
    provenance = report_document.get("runtime_provenance")
    simulation = provenance.get("simulation") if isinstance(provenance, dict) else None
    reset_strategy = (
        simulation.get("reset_strategy") if isinstance(simulation, dict) else None
    )
    reset_strategy_id = row.get("reset_strategy_id")
    reset_strategy_token = f"reset-v1-{reset_strategy_id}"
    if (
        not isinstance(provenance, dict)
        or provenance.get("schema_version") != 6
        or not isinstance(reset_strategy, dict)
        or reset_strategy.get("schema_version") != 1
        or reset_strategy.get("id") != reset_strategy_id
        or reset_strategy_token not in reset_strategy_tokens
    ):
        raise SystemExit(
            f"manifest row {row_index} report reset strategy identity mismatch"
        )
    canonical_report_path = str(report_path.resolve())
    if canonical_report_path in manifest_report_locks:
        raise SystemExit("manifest report paths must be unique")
    manifest_report_locks[canonical_report_path] = {
        "sha256": report_sha256,
        "report_schema_version": 3,
        "runtime_provenance_schema_version": 6,
        "reset_strategy_schema_version": 1,
        "reset_strategy_id": reset_strategy_id,
        "reset_strategy_token": reset_strategy_token,
        "environment_id": row.get("environment"),
        "ground_topology_id": topology_id,
        "contact_profile_id": row.get("profile_id"),
    }
selection = analysis.get("selection", {})
selection_included = selection.get("included") if isinstance(selection, dict) else None
if (
    not isinstance(selection_included, list)
    or len(selection_included) != expected_runs
):
    raise SystemExit("aggregate selection included-report count mismatch")
analysis_report_locks = {}
for selection_index, included in enumerate(selection_included, start=1):
    if not isinstance(included, dict):
        raise SystemExit(
            f"aggregate selection item {selection_index} must be an object"
        )
    included_path = included.get("path")
    if not isinstance(included_path, str):
        raise SystemExit(
            f"aggregate selection item {selection_index} path is invalid"
        )
    if included_path in analysis_report_locks:
        raise SystemExit("aggregate selection report paths must be unique")
    analysis_report_locks[included_path] = {
        "sha256": included.get("sha256"),
        "report_schema_version": included.get("report_schema_version"),
        "runtime_provenance_schema_version": included.get(
            "runtime_provenance_schema_version"
        ),
        "reset_strategy_schema_version": included.get(
            "reset_strategy_schema_version"
        ),
        "reset_strategy_id": included.get("reset_strategy_id"),
        "reset_strategy_token": included.get("reset_strategy_token"),
        "environment_id": included.get("environment_id"),
        "ground_topology_id": included.get("ground_topology_id"),
        "contact_profile_id": included.get("contact_profile_id"),
    }
if analysis_report_locks != manifest_report_locks:
    raise SystemExit(
        "aggregate selection does not match frozen manifest/report identities"
    )
summary = {
    "schema_version": 6,
    "report_type": "contact_ab_batch_summary",
    "result": "success",
    "schema_contract": {
        "project_config": 2,
        "runtime_provenance": 6,
        "motion_report": 3,
        "aggregate_analysis": 5,
        "physical_acceptance": 3,
        "manifest": 2,
    },
    "manifest_contract": {
        "version": 2,
        "columns": expected_manifest_fieldnames,
    },
    "environment_selection": environment_selection,
    "ground_topology_selection": ground_topology_selection,
    "contact_profile_selection": contact_profile_selection,
    "reset_strategy_selection": reset_strategy_selection,
    "environments": selected_environments,
    "ground_topologies": selected_topologies,
    "environment_topology_pairs": environment_topology_pairs,
    "repeats": int(repeats_text),
    "expected_counts": {
        "runs": expected_runs,
        "groups": expected_groups,
        "environments": len(selected_environments),
        "environment_topology_pairs": len(environment_topology_pairs),
        "ground_topologies": len(selected_topologies),
        "profiles": len(profile_hashes),
        "reset_strategies": len(reset_strategy_ids),
    },
    "actual_counts": {
        "manifest_rows": int(manifest_rows_text),
        "successful_runs": int(successful_rows_text),
        "analysis_included_reports": analysis_counts.get("included_reports"),
        "analysis_excluded_reports": analysis_counts.get("excluded_reports"),
        "analysis_groups": analysis_counts.get("groups"),
        "acceptance_applicable_groups": len(applicable_groups),
        "acceptance_not_applicable_groups": len(not_applicable_groups),
        "acceptance_passing_groups": len(passing_groups),
        "acceptance_failed_groups": len(failed_groups),
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
        "robot_config": {
            "selection": robot_config_selection,
            "path": robot_config_path,
            "sha256": robot_config_sha256,
            "asset": {
                "path": robot_asset_path,
                "sha256": robot_asset_sha256,
            },
            "kinematics": {
                "profile_id": robot_kinematics_profile_id,
                "lifecycle": robot_kinematics_lifecycle,
                "wheel_radius_m": float(robot_wheel_radius_text),
                "wheel_width_m": float(robot_wheel_width_text),
                "geometric_track_width_m": float(
                    robot_geometric_track_width_text
                ),
                "effective_track_width_m": float(
                    robot_effective_track_width_text
                ),
            },
            "solver": {
                "position_iterations": 32,
                "velocity_iterations": 4,
                "stage_articulation_usd_readback_verified": True,
            },
        },
        "simulation": {
            "navigation_mode": "mapping",
            "odometry_mode": "ideal",
            "physics_hz": 60.0,
        },
        "reset_strategies": {
            "selection": reset_strategy_selection,
            "schema_version": 1,
            "ids": reset_strategy_ids,
            "tokens": reset_strategy_tokens,
        },
        "contact_profiles": profiles,
        "ground_topology_profiles": ground_topology_profiles,
    },
    "evidence": {
        "manifest": {"path": str(manifest_path), "sha256": manifest_sha256},
        "analysis": {"path": str(analysis_path), "sha256": analysis_sha256},
    },
    "physical_acceptance": {
        "schema_version": physical_acceptance["schema_version"],
        "policy_id": physical_acceptance["policy_id"],
        "evaluation_basis": physical_acceptance["evaluation_basis"],
        "ranking_policy": physical_acceptance["ranking_policy"],
        "steady_state_measurement_basis": physical_acceptance[
            "steady_state_measurement_basis"
        ],
        "wheel_direction_measurement_basis": physical_acceptance[
            "wheel_direction_measurement_basis"
        ],
        "thresholds": physical_acceptance["thresholds"],
        "applicability": physical_acceptance["applicability"],
        "all_applicable_groups_passed": physical_acceptance[
            "all_applicable_groups_passed"
        ],
        "applicable_groups": applicable_groups,
        "not_applicable_groups": not_applicable_groups,
        "passing_groups": passing_groups,
        "failed_groups": failed_groups,
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
for topology_id in "${matrix_ground_topology_ids[@]}"; do
  topology_path="$(ground_topology_path "${topology_id}")" \
    || die "cannot resolve selected ground topology: ${topology_id}"
  require_file "${topology_path}"
done
require_clean_git
if [[ "${robot_config_option_seen}" == true ]]; then
  validate_robot_config_path "${robot_config_argument}" \
    || die "explicit --robot-config path is not trusted"
  require_tracked_input "${robot_config_argument}"
fi
require_tracked_input "${motion_config}"
require_tracked_input "${warehouse_config}"
require_tracked_input "${simple_plane_config}"
for profile_id in "${profile_ids[@]}"; do
  require_tracked_input "${physics_dir}/${profile_id}.yaml"
done
for topology_id in "${matrix_ground_topology_ids[@]}"; do
  topology_path="$(ground_topology_path "${topology_id}")" \
    || die "cannot resolve selected ground topology: ${topology_id}"
  require_tracked_input "${topology_path}"
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
freeze_motion_configuration_contract
verify_batch_identity "batch initialization" \
  || die "contact A/B batch identity changed during initialization"
prepare_output_directory
initialize_manifest

sequence=0
for pair_index in "${!matrix_environment_ids[@]}"; do
  environment_id="${matrix_environment_ids[pair_index]}"
  topology_id="${matrix_ground_topology_ids[pair_index]}"
  for profile_index in "${!profile_ids[@]}"; do
    for ((repeat = 1; repeat <= repeats; repeat++)); do
      for reset_strategy_id in "${reset_strategy_ids[@]}"; do
        sequence=$((sequence + 1))
        if ! run_one_condition \
            "${sequence}" "${environment_id}" "${topology_id}" \
            "${profile_ids[profile_index]}" \
            "${profile_modes[profile_index]}" \
            "${reset_strategy_id}" "${repeat}"; then
          die "contact A/B failed closed at ${current_run_id}: ${current_failure_reason}"
        fi
      done
    done
  done
done

expected_conditions=$((${#matrix_environment_ids[@]} * ${#profile_ids[@]} * ${#reset_strategy_ids[@]} * repeats))
expected_groups=$((${#matrix_environment_ids[@]} * ${#profile_ids[@]} * ${#reset_strategy_ids[@]}))
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

validate_success_manifest_evidence "aggregate pre-freeze" \
  || die "completed contact A/B evidence failed pre-freeze validation"
freeze_manifest \
  || die "cannot freeze and hash the completed contact A/B manifest"
finalize_contact_analysis \
  || die "contact A/B aggregate analysis failed closed"
analysis_sha256="$(
  final_evidence_sha256 "${analysis_path}" true "aggregate analysis"
)" || die "cannot hash aggregate contact A/B analysis"
write_batch_summary "${successful_rows}" "${manifest_rows}" \
  || die "cannot write atomic contact A/B batch summary"

physical_status_line="$(
  python3 - "${batch_summary_path}" <<'PY'
import json
from pathlib import Path
import sys

summary = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
physical = summary.get("physical_acceptance", {})
verdict = physical.get("all_applicable_groups_passed")
applicable = physical.get("applicable_groups")
passing = physical.get("passing_groups")
failed = physical.get("failed_groups")
if (
    verdict is not None and not isinstance(verdict, bool)
    or not isinstance(applicable, list)
    or not isinstance(passing, list)
    or not isinstance(failed, list)
):
    raise SystemExit("batch summary physical status is invalid")
status = "not_applicable" if verdict is None else "pass" if verdict else "fail"
print(status, len(applicable), len(passing), len(failed), sep="\t")
PY
)" || die "cannot read completed batch physical acceptance status"
IFS=$'\t' read -r physical_status physical_applicable physical_passing physical_failed \
  <<<"${physical_status_line}"
case "${physical_status}" in
  pass)
    log_info "contact A/B evidence complete; plan 8.7 physical acceptance=PASS (${physical_passing}/${physical_applicable} applicable groups)"
    ;;
  fail)
    log_warn "contact A/B evidence complete; plan 8.7 physical acceptance=FAIL (${physical_failed}/${physical_applicable} applicable groups failed)"
    ;;
  not_applicable)
    log_warn "contact A/B evidence complete; plan 8.7 physical acceptance=NOT_APPLICABLE (requires SimplePlane + Ideal + at least 3 repeats)"
    ;;
  *)
    die "unknown completed batch physical acceptance status: ${physical_status}"
    ;;
esac
log_info "contact A/B evidence matrix complete: ${batch_summary_path}"
