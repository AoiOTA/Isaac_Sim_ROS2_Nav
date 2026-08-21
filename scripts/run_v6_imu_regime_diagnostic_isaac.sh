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
PACKAGE_PREFIX="$(ros2 pkg prefix robot_experiments 2>/dev/null)" || {
  echo "installed robot_experiments package is unavailable" >&2
  exit 69
}
PACKAGE_SHARE="${PACKAGE_PREFIX}/share/robot_experiments"
SPAWN_FILE="${PACKAGE_SHARE}/environments/v6_calibration_flat_20m.spawn.yaml"
DIAGNOSTIC_CONFIG="${PACKAGE_SHARE}/config/v6_imu_regime_diagnostic.yaml"
RESOURCE_MANIFEST="${PACKAGE_SHARE}/config/v6_imu_regime_resources.json"
FEATURE_CONFIG="${PACKAGE_SHARE}/config/v6_calibration_grid_features.yaml"

if [[ ! -f "${GRID_USD}" || ! -f "${SPAWN_FILE}" || ! -f "${DIAGNOSTIC_CONFIG}" || ! -f "${RESOURCE_MANIFEST}" || ! -f "${FEATURE_CONFIG}" ]]; then
  echo "locked installed flat20 asset/config/resource manifest is missing" >&2
  exit 66
fi

export ISAAC_NAV__GROUND_TRUTH__ENABLED=true

exec "${SCRIPT_DIR}/run_isaac.sh" \
  --mode realistic \
  --navigation-mode mapping \
  --environment-usd "${GRID_USD}" \
  --spawn-poses-file "${SPAWN_FILE}" \
  --spawn-pose flat20_start \
  --dynamic-obstacle-config "${FEATURE_CONFIG}" \
  --dynamic-obstacles \
  --camera-profile off \
  --no-third-person-camera \
  --appearance-profile baseline \
  --imu-regime-diagnostic-config "${DIAGNOSTIC_CONFIG}" \
  --imu-regime-phase-trace "${TRACE_PATH}"
