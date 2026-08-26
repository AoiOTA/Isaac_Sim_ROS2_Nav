#!/usr/bin/env bash

configure_v6_dynamic_integration_overlay() {
  export BIO_NAV_INTEGRATION_ROOT="${BIO_NAV_INTEGRATION_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_integration}"
  export BIO_NAV_INTEGRATION_SETUP="${BIO_NAV_INTEGRATION_SETUP:-${BIO_NAV_INTEGRATION_ROOT}/ros2_ws/install_run4_candidate/setup.bash}"
}

validate_v6_dynamic_integration_overlay() {
  local expected_root package prefix prefix_real
  expected_root="$(readlink -f "${BIO_NAV_INTEGRATION_ROOT}")"
  for package in bio_nav_interfaces bio_nav_ros_bridge; do
    prefix="$(ros2 pkg prefix "${package}" 2>/dev/null || true)"
    [[ -n "${prefix}" ]] || {
      echo "${package} is unavailable after sourcing ${BIO_NAV_INTEGRATION_SETUP}" >&2
      return 2
    }
    prefix_real="$(readlink -f "${prefix}")"
    [[ "${prefix_real}" == "${expected_root}"/* ]] || {
      echo "${package} resolved outside ${expected_root}: ${prefix_real}" >&2
      return 2
    }
  done
  if ! python3 -c \
      'from bio_nav_interfaces.msg import CognitivePoseModeCandidate' 2>/dev/null; then
    echo "bio_nav_interfaces from ${BIO_NAV_INTEGRATION_SETUP} does not provide CognitivePoseModeCandidate" >&2
    return 2
  fi
}

prepare_v6_dynamic_assets() {
  local importer="${PROJECT_ROOT}/isaac_sim/tools/import_assets.py"
  [[ -f "${importer}" ]] || {
    echo "asset importer is missing: ${importer}" >&2
    return 2
  }
  "${ISAAC_PYTHON}" "${importer}" --asset-root "${ISAAC_ASSET_ROOT}"
  "${ISAAC_PYTHON}" "${importer}" --asset-root "${ISAAC_ASSET_ROOT}" --check
}
