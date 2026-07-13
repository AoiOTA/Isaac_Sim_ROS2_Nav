#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

usage() {
  cat <<'EOF'
usage: run_camera_view.sh [camera_view.rviz]

Open the dedicated RViz front-Camera view. The optional configuration path
overrides the installed robot_description/rviz/camera_view.rviz file.
EOF
}

if [[ "${1:-}" == -h || "${1:-}" == --help ]]; then
  usage
  exit 0
fi
(($# <= 1)) || {
  usage >&2
  die "run_camera_view.sh accepts at most one RViz config"
}
original_args=("$@")

source_ros --require-workspace
require_command realpath
require_command ros2
require_command rviz2
ensure_dedicated_process_group "${original_args[@]}"
acquire_instance_lock rviz "RViz"

if (($# == 1)); then
  rviz_config="$1"
  if [[ "${rviz_config}" != /* ]]; then
    rviz_config="$(realpath -m "${rviz_config}")"
  fi
else
  description_prefix="$(ros2 pkg prefix robot_description)"
  rviz_config="${description_prefix}/share/robot_description/rviz/camera_view.rviz"
fi

require_file "${rviz_config}"
log_info "starting dedicated front-Camera RViz view: ${rviz_config}"
exec rviz2 -d "${rviz_config}" --ros-args -p use_sim_time:=true
