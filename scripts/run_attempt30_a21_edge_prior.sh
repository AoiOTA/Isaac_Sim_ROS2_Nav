#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

source_ros --require-workspace
module2_a21_root="${BIO_NAV_MODULE2_A21_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/module2/attempt30-a21-edge-prior}"
require_file "${module2_a21_root}/src/module2_srdr_pdf_v30/edge_prior.py"
export PYTHONPATH="${module2_a21_root}/src${PYTHONPATH:+:${PYTHONPATH}}"
ensure_dedicated_process_group "$@"
acquire_instance_lock edge_prior "A21 edge-prior bridge"
exec ros2 launch bio_nav_ros_bridge attempt30_a21_integration.launch.py "$@"
