#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

usage() {
  cat <<'EOF'
usage: run_rviz.sh mapping|incremental_mapping|localization|navigation [config]

Open the project RViz workflow for one operation. The optional config may be
an absolute or relative .rviz file; otherwise the mode-specific installed
configuration is selected automatically.
EOF
}

operation="${1:-}"
[[ -n "${operation}" ]] || {
  usage >&2
  exit 1
}
shift

case "${operation}" in
  mapping|incremental_mapping)
    config_name="mapping.rviz"
    ;;
  localization)
    config_name="localization.rviz"
    ;;
  navigation)
    config_name="navigation.rviz"
    ;;
  -h|--help)
    usage
    exit 0
    ;;
  *)
    usage >&2
    die "unsupported RViz operation: ${operation}"
    ;;
esac

(($# <= 1)) || die "run_rviz.sh accepts at most one custom config path"

source_ros --require-workspace
require_command ros2
require_command realpath
require_command rviz2
acquire_instance_lock rviz "RViz"

if (($# == 1)); then
  rviz_config="$1"
  if [[ "${rviz_config}" != /* ]]; then
    rviz_config="$(realpath -m "${rviz_config}")"
  fi
else
  description_prefix="$(ros2 pkg prefix robot_description)"
  rviz_config="${description_prefix}/share/robot_description/rviz/${config_name}"
fi

require_file "${rviz_config}"
log_info "starting RViz workflow: operation=${operation}, config=${rviz_config}"
exec rviz2 \
  -d "${rviz_config}" \
  --ros-args \
  -p use_sim_time:=true
