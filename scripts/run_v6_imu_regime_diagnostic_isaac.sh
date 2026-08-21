#!/usr/bin/env bash
set -Eeuo pipefail

if [[ "$#" -ne 1 ]]; then
  echo "usage: $0 PHASE_TRACE_JSONL" >&2
  exit 64
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
TRACE_PATH="$1"
GRID_USD="/home/lyb/isaacsim_assets/Assets/Isaac/6.0/Isaac/Environments/Grid/default_environment.usd"
SPAWN_FILE="${PROJECT_ROOT}/isaac_sim/configs/environments/v6_calibration_flat_20m.spawn.yaml"

if [[ ! -f "${GRID_USD}" || ! -f "${SPAWN_FILE}" ]]; then
  echo "locked flat20 asset or spawn profile is missing" >&2
  exit 66
fi

export ISAAC_NAV__GROUND_TRUTH__ENABLED=true

exec "${SCRIPT_DIR}/run_isaac.sh" \
  --mode realistic \
  --navigation-mode mapping \
  --environment-usd "${GRID_USD}" \
  --spawn-poses-file "${SPAWN_FILE}" \
  --spawn-pose flat20_start \
  --no-dynamic-obstacles \
  --camera-profile off \
  --no-third-person-camera \
  --appearance-profile baseline \
  --imu-regime-phase-trace "${TRACE_PATH}"
