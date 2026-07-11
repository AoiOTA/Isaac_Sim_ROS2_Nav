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

occupancy="${PROJECT_ROOT}/data/maps/occupancy/${version}"
posegraph="${PROJECT_ROOT}/data/maps/posegraphs/${version}"
targets=(
  "${occupancy}.yaml"
  "${occupancy}.pgm"
  "${posegraph}.posegraph"
  "${posegraph}.data"
)
for target in "${targets[@]}"; do
  [[ ! -e "${target}" ]] || die "refusing to overwrite map artifact: ${target}"
done
mkdir -p "$(dirname "${occupancy}")" "$(dirname "${posegraph}")"
cleanup_partial_artifacts() {
  rm -f -- "${targets[@]}"
}
trap cleanup_partial_artifacts ERR

ros2 run nav2_map_server map_saver_cli -f "${occupancy}"
serialize_output="$(
  ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph "{filename: '${posegraph}'}"
)"
printf '%s\n' "${serialize_output}"
if [[ "${serialize_output}" != *"result=0"* && "${serialize_output}" != *"result: 0"* ]]; then
  die "SLAM Toolbox reported pose-graph serialization failure"
fi
for target in "${targets[@]}"; do
  [[ -s "${target}" ]] || die "map artifact was not created or is empty: ${target}"
done
trap - ERR

echo "saved occupancy map: ${occupancy}.yaml"
echo "saved pose graph: ${posegraph}"
