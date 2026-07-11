#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
source_ros

with_isaac=false
if [[ "${1:-}" == "--with-isaac" ]]; then
  with_isaac=true
  shift
fi

cd "${PROJECT_ROOT}"
python3 -m pytest -m "not isaac and not ros" "$@"

cd "${PROJECT_ROOT}/ros2_ws"
colcon test --event-handlers console_direct+
colcon test-result --verbose

if [[ "${with_isaac}" == true ]]; then
  require_executable "${ISAAC_PYTHON}"
  cd "${PROJECT_ROOT}"
  if "${ISAAC_PYTHON}" -c 'import pytest' >/dev/null 2>&1; then
    "${ISAAC_PYTHON}" -m pytest isaac_sim/tests -m isaac
  else
    # The Isaac Sim Conda environment intentionally has no pytest in a stock
    # installation. USD-only tests can reuse its Python 3.12 bindings from the
    # system pytest process without installing packages into that environment.
    ISAAC_SITE_PACKAGES="$(
      "${ISAAC_PYTHON}" -c 'import site; print(site.getsitepackages()[0])'
    )"
    PYTHONPATH="${ISAAC_SITE_PACKAGES}:${PYTHONPATH:-}" \
      python3 -m pytest isaac_sim/tests -m isaac
  fi
fi
