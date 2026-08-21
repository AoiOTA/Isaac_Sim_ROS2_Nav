#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKTREE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
CONFIG="${WORKTREE_ROOT}/ros2_ws/src/robot_experiments/config/v6_estimated_calibration.yaml"
export PYTHONPATH="${WORKTREE_ROOT}/ros2_ws/src/robot_experiments${PYTHONPATH:+:${PYTHONPATH}}"

exec python3 -m robot_experiments.v6_estimated_calibration --config "${CONFIG}" "$@"
