#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

readonly DEFAULT_ROOT="/mnt/nas_home/Bio_Nav_Data/experiments/pilots/module1_kujiale_scene_registration_20260825/raw_mcap"
readonly QOS_OVERRIDES="${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/module1_targeted_teaching_rosbag_qos.yaml"

usage() {
  cat <<EOF
usage: record_module1_kujiale_scene.sh [--root PATH] [--episode ID]

Record one simulation-only Kujiale Module1 localization-collection episode as
MCAP. The default root is:
  ${DEFAULT_ROOT}

If --episode is omitted, a unique timestamp-and-process ID is generated.
Existing output paths are never overwritten. Depth and Module2/CPG topics are
intentionally excluded.
EOF
}

root="${DEFAULT_ROOT}"
episode_id=""
while (($# > 0)); do
  case "$1" in
    --root)
      (($# >= 2)) || die "--root requires a path"
      root="$2"
      shift 2
      ;;
    --episode)
      (($# >= 2)) || die "--episode requires an ID"
      episode_id="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      die "unsupported argument: $1"
      ;;
  esac
done

if [[ -z "${episode_id}" ]]; then
  episode_id="kujiale-$(date +%Y%m%d-%H%M%S)-$$"
fi
[[ "${episode_id}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ \
    && "${episode_id}" != "." \
    && "${episode_id}" != ".." ]] \
  || die "episode ID must use only letters, digits, dot, underscore, or dash"

mkdir -p "${root}"
output_path="${root}/${episode_id}"
[[ ! -e "${output_path}" ]] \
  || die "refusing to overwrite existing episode: ${output_path}"

source_ros --require-workspace
require_command ros2
require_file "${QOS_OVERRIDES}"

topics=(
  /clock
  /camera/front/image_raw
  /camera/front/camera_info
  /joint_states
  /wheel/odom
  /imu/data_raw
  /imu/data
  /bio_nav/module1/odom
  /odom
  /amcl_pose
  /ground_truth/odom
  /map
  /scan
  /tf
  /tf_static
  /cmd_vel
  /cmd_vel_sim
  /initialpose
  /simulation/reset_event
  /simulation/localization_seeded
  /simulation/reset_stop_gate/status
  /simulation/collision
)

log_info "recording exactly one MCAP bag for episode ${episode_id}"
log_info "output: ${output_path}"
log_info "in the Teleop window press Space to stop, then press Ctrl+C here"
log_info "a reset requires ending this bag and starting a new episode"

exec ros2 bag record \
  --use-sim-time \
  --storage mcap \
  --storage-preset-profile zstd_fast \
  --qos-profile-overrides-path "${QOS_OVERRIDES}" \
  --output "${output_path}" \
  "${topics[@]}"
