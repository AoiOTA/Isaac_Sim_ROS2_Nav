#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
source_ros --require-workspace
acquire_instance_lock map_save "map save"

version="${1:-}"
[[ -n "${version}" ]] || die "usage: $0 MAP_VERSION"
[[ "${version}" =~ ^[A-Za-z0-9._-]+$ ]] || die "map version contains unsafe characters"
[[ "${version}" =~ [A-Za-z0-9_-] ]] || die "map version cannot contain only dots"

occupancy="${PROJECT_ROOT}/data/maps/occupancy/${version}"
posegraph="${PROJECT_ROOT}/data/maps/posegraphs/${version}"
manifest="${PROJECT_ROOT}/data/maps/manifests/${version}.yaml"
targets=(
  "${occupancy}.yaml"
  "${occupancy}.pgm"
  "${posegraph}.posegraph"
  "${posegraph}.data"
  "${manifest}"
)
map_directories=(
  "${PROJECT_ROOT}/data/maps"
  "$(dirname "${occupancy}")"
  "$(dirname "${posegraph}")"
  "$(dirname "${manifest}")"
  "${PROJECT_ROOT}/data/maps/.staging"
)
require_safe_map_directory() {
  local directory="$1"
  local relative current component
  local -a components=()
  [[ "${directory}" == "${PROJECT_ROOT}/data/maps" \
      || "${directory}" == "${PROJECT_ROOT}/data/maps/"* ]] \
    || die "map storage directory escapes project root: ${directory}"
  relative="${directory#"${PROJECT_ROOT}/"}"
  current="${PROJECT_ROOT}"
  IFS='/' read -r -a components <<<"${relative}"
  for component in "${components[@]}"; do
    [[ -n "${component}" && "${component}" != . && "${component}" != .. ]] \
      || die "map storage directory contains an unsafe component: ${directory}"
    current="${current}/${component}"
    [[ ! -L "${current}" ]] \
      || die "map storage directory traverses a symbolic link: ${current}"
  done
}
for directory in "${map_directories[@]}"; do
  require_safe_map_directory "${directory}"
done
for target in "${targets[@]}"; do
  [[ ! -e "${target}" ]] || die "refusing to overwrite map artifact: ${target}"
done
mkdir -p "${map_directories[@]}"
for directory in "${map_directories[@]}"; do
  require_safe_map_directory "${directory}"
  [[ -d "${directory}" ]] || die "map storage path is not a directory: ${directory}"
done
staging="$(mktemp -d "${PROJECT_ROOT}/data/maps/.staging/${version}.XXXXXX")"
staged_occupancy="${staging}/${version}"
staged_posegraph="${staging}/${version}"
staged_manifest="${staging}/${version}.manifest.yaml"
save_committed=false
published_sources=()
published_targets=()
publish_no_clobber() {
  local source_path="$1"
  local target_path="$2"

  # Staging lives below data/maps, so a hard link gives us an atomic,
  # no-clobber publish on the same filesystem. Keep the staging link until the
  # whole transaction commits so rollback can verify ownership with -ef.
  # Register before ln: Bash may run a pending signal trap immediately after
  # the external command returns, before the next shell statement executes.
  published_sources+=("${source_path}")
  published_targets+=("${target_path}")
  if ! ln -- "${source_path}" "${target_path}"; then
    die "refusing to overwrite concurrently-created map artifact: ${target_path}"
  fi
}
cleanup_partial_artifacts() {
  if [[ "${save_committed}" != true ]]; then
    local index
    for ((index=${#published_targets[@]} - 1; index >= 0; index--)); do
      # Never remove a file merely because it has the requested name. Only
      # roll back the exact inode published by this transaction.
      if [[ -e "${published_targets[index]}" \
        && "${published_targets[index]}" -ef "${published_sources[index]}" ]]; then
        rm -f -- "${published_targets[index]}"
      fi
    done
  fi
  rm -rf -- "${staging}"
}
cleanup_interrupted_save() {
  cleanup_partial_artifacts
  trap - EXIT INT TERM
  exit 130
}
trap cleanup_partial_artifacts EXIT
trap cleanup_interrupted_save INT TERM

ros2 run nav2_map_server map_saver_cli -f "${staged_occupancy}"
serialize_output="$(
  ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph "{filename: '${staged_posegraph}'}"
)"
printf '%s\n' "${serialize_output}"
if [[ "${serialize_output}" != *"result=0"* && "${serialize_output}" != *"result: 0"* ]]; then
  die "SLAM Toolbox reported pose-graph serialization failure"
fi
staged_targets=(
  "${staged_occupancy}.yaml"
  "${staged_occupancy}.pgm"
  "${staged_posegraph}.posegraph"
  "${staged_posegraph}.data"
)
for target in "${staged_targets[@]}"; do
  [[ -s "${target}" ]] || die "map artifact was not created or is empty: ${target}"
done

ros2 run robot_bringup map_manifest create \
  --project-root "${PROJECT_ROOT}" \
  --map-version "${version}" \
  --occupancy-yaml "${staged_occupancy}.yaml" \
  --occupancy-image "${staged_occupancy}.pgm" \
  --posegraph "${staged_posegraph}.posegraph" \
  --posegraph-data "${staged_posegraph}.data" \
  --output "${staged_manifest}"

# The manifest is the commit marker. Publish all four artifacts without
# clobbering anything created after the initial check, verify them against the
# staged manifest, and atomically publish the manifest last.
publish_no_clobber "${staged_occupancy}.yaml" "${occupancy}.yaml"
publish_no_clobber "${staged_occupancy}.pgm" "${occupancy}.pgm"
publish_no_clobber "${staged_posegraph}.posegraph" "${posegraph}.posegraph"
publish_no_clobber "${staged_posegraph}.data" "${posegraph}.data"
ros2 run robot_bringup map_manifest verify \
  --project-root "${PROJECT_ROOT}" \
  --manifest "${staged_manifest}" \
  --allow-staged-manifest
publish_no_clobber "${staged_manifest}" "${manifest}"
save_committed=true
rm -rf -- "${staging}"
trap - EXIT INT TERM

echo "saved occupancy map: ${occupancy}.yaml"
echo "saved pose graph: ${posegraph}"
echo "saved uncalibrated map manifest last: ${manifest}"
