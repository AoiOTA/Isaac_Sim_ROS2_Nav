#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

require_executable "${ISAAC_PYTHON}"
require_file "${ISAAC_ASSET_ROOT}/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
require_file "${ISAAC_ASSET_ROOT}/Isaac/Robots/Clearpath/Jackal/jackal.usd"
require_file "${ISAAC_ASSET_ROOT}/Isaac/Robots/Clearpath/Jackal/configuration/jackal_robot_schema.usd"

"${ISAAC_PYTHON}" -c 'from importlib.metadata import version; assert version("isaacsim") == "6.0.1.0"; print("Isaac Sim", version("isaacsim"))'

source_ros
for package in nav2_bringup nav2_mppi_controller nav2_smac_planner \
  nav2_velocity_smoother nav2_collision_monitor slam_toolbox \
  pointcloud_to_laserscan robot_localization xacro; do
  ros2 pkg prefix "${package}" >/dev/null
done

MAP_MANIFEST="${PROJECT_ROOT}/data/maps/manifests/warehouse_v1.yaml"
require_file "${MAP_MANIFEST}"
python3 - "${PROJECT_ROOT}" "${MAP_MANIFEST}" <<'PY'
import hashlib
from pathlib import Path
import sys

import yaml

project_root = Path(sys.argv[1])
manifest = yaml.safe_load(Path(sys.argv[2]).read_text(encoding="utf-8"))
entries = [
    *manifest["occupancy_grid"]["files"],
    *manifest["pose_graph"]["files"],
]
for entry in entries:
    path = project_root / entry["path"]
    if not path.is_file():
        raise SystemExit(f"missing curated map artifact: {path}")
    if path.read_bytes().startswith(b"version https://git-lfs.github.com/spec/v1"):
        raise SystemExit(f"Git LFS artifact is not hydrated: {path}; run git lfs pull")
    if path.stat().st_size != entry["bytes"]:
        raise SystemExit(f"map artifact size mismatch: {path}")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != entry["sha256"]:
        raise SystemExit(f"map artifact SHA256 mismatch: {path}")
print(f"map baseline: {manifest['map_version']} (integrity verified)")
PY

command -v colcon >/dev/null || die "colcon is not available"
command -v nvidia-smi >/dev/null || die "nvidia-smi is not available"
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader

echo "ROS 2: ${ROS_DISTRO:-unknown}; RMW: ${RMW_IMPLEMENTATION}; domain: ${ROS_DOMAIN_ID}"
echo "asset root: ${ISAAC_ASSET_ROOT}"
echo "preflight: PASS"
