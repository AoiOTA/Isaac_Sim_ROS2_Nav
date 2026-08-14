#!/usr/bin/env bash
set -Eeuo pipefail

module3_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
integration_root="${BIO_NAV_INTEGRATION_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/integration/attempt31-outdoor-nav}"
demo_dir="${RIVERMARK_DEMO_DIR:-${module3_root}/data/rivermark_demo}"
asset="${RIVERMARK_USD:-/home/lyb/Rivermark/rivermark.usd}"
mode="${1:-off}"
scenario="${2:-static}"
domain_id="${ROS_DOMAIN_ID:-231}"
runtime_dir="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/bionav-rivermark-${domain_id}"
auto_goal="${RIVERMARK_AUTO_GOAL:-1}"
ground_truth="${RIVERMARK_GROUND_TRUTH:-0}"
guidance_profile="${RIVERMARK_GUIDANCE_PROFILE:-}"
controller_max_linear_velocity_mps="${RIVERMARK_MAX_LINEAR_SPEED_MPS:-0.75}"
controller_linear_velocity_std_mps="${RIVERMARK_LINEAR_SPEED_STD_MPS:-0.35}"
rendering_hz="${RIVERMARK_RENDERING_HZ:-30}"

if [[ "${mode}" != "off" && "${mode}" != "module2" ]]; then
  echo "usage: $0 [off|module2] [static|dynamic]" >&2
  exit 2
fi
if [[ "${scenario}" != "static" && "${scenario}" != "dynamic" ]]; then
  echo "usage: $0 [off|module2] [static|dynamic]" >&2
  exit 2
fi
if [[ "${auto_goal}" != "0" && "${auto_goal}" != "1" ]]; then
  echo "RIVERMARK_AUTO_GOAL must be 0 or 1" >&2
  exit 2
fi
if [[ "${ground_truth}" != "0" && "${ground_truth}" != "1" ]]; then
  echo "RIVERMARK_GROUND_TRUTH must be 0 or 1" >&2
  exit 2
fi
python3 - "${controller_max_linear_velocity_mps}" \
  "${controller_linear_velocity_std_mps}" "${rendering_hz}" <<'PY'
import math
import sys

names = (
    "RIVERMARK_MAX_LINEAR_SPEED_MPS",
    "RIVERMARK_LINEAR_SPEED_STD_MPS",
    "RIVERMARK_RENDERING_HZ",
)
for name, raw in zip(names, sys.argv[1:]):
    try:
        value = float(raw)
    except ValueError as error:
        raise SystemExit(f"{name} must be numeric, got {raw!r}") from error
    if not math.isfinite(value) or value <= 0.0:
        raise SystemExit(f"{name} must be finite and positive, got {raw!r}")
PY
for path in \
  "${demo_dir}/rivermark_selected.yaml" \
  "${demo_dir}/rivermark_selected.geojson" \
  "${demo_dir}/rivermark_regions.yaml" \
  "${demo_dir}/rivermark.spawn.yaml" \
  "${demo_dir}/rivermark_demo_goals.yaml" \
  "${demo_dir}/rivermark_dynamic.yaml" \
  "${demo_dir}/rivermark_appearance_profiles.yaml"; do
  [[ -f "${path}" ]] || { echo "missing ${path}; run prepare_rivermark_demo.sh first" >&2; exit 2; }
done

python3 - "${demo_dir}/rivermark_selected.yaml" <<'PY'
import sys
from pathlib import Path

import cv2
import yaml

yaml_path = Path(sys.argv[1])
metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
resolution = float(metadata["resolution"])
image_path = yaml_path.parent / metadata["image"]
image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
if resolution != 0.05:
    raise SystemExit(f"Rivermark demo requires 0.05 m/cell, got {resolution}")
if image is None or image.shape[:2] != (1600, 1600):
    shape = None if image is None else image.shape[:2]
    raise SystemExit(f"Rivermark demo requires the current 1600x1600 map, got {shape}")
PY

readarray -t values < <(python3 - "${demo_dir}" <<'PY'
import sys,yaml
from pathlib import Path
d=Path(sys.argv[1])
g=yaml.safe_load((d/'rivermark_demo_goals.yaml').read_text())
print(*g['start'])
print(*g['goal'])
PY
)
read -r start_x start_y start_yaw <<<"${values[0]}"
read -r goal_x goal_y goal_yaw <<<"${values[1]}"

export ROS_DOMAIN_ID="${domain_id}"
export ISAAC_NAV_RUNTIME_DIR="${runtime_dir}"
mkdir -p "${runtime_dir}"

cleanup() {
  jobs -pr | xargs -r kill 2>/dev/null || true
}
trap cleanup EXIT INT TERM

dynamic_args=(--no-dynamic-obstacles)
if [[ "${scenario}" == "dynamic" ]]; then
  dynamic_args=(--dynamic-obstacles)
fi
isaac_runtime_args=()
if [[ "${RIVERMARK_HEADLESS:-0}" == "1" ]]; then
  isaac_runtime_args+=(--headless)
fi
if [[ -n "${RIVERMARK_MAX_STEPS:-}" ]]; then
  isaac_runtime_args+=(--max-steps "${RIVERMARK_MAX_STEPS}")
fi

env \
  -u AMENT_PREFIX_PATH \
  -u CMAKE_PREFIX_PATH \
  -u COLCON_PREFIX_PATH \
  -u LD_LIBRARY_PATH \
  -u PYTHONPATH \
  -u ROS_PACKAGE_PATH \
  ISAAC_NAV_EXPECTED_DOMAIN_ID="${domain_id}" \
  ISAAC_NAV__GROUND_TRUTH__ENABLED="$([[ "${ground_truth}" == "1" ]] && echo true || echo false)" \
  ISAAC_NAV__SIMULATION__RENDERING_HZ="${rendering_hz}" \
  BIO_NAV_INTERFACES_SETUP="${integration_root}/ros2_ws/install/local_setup.bash" \
  "${module3_root}/scripts/run_isaac.sh" \
  --environment-usd "${asset}" \
  --spawn-poses-file "${demo_dir}/rivermark.spawn.yaml" \
  --spawn-pose rivermark_start \
  --camera-profile rgbd_navigation \
  --navigation-mode localization \
  --dynamic-obstacle-config "${demo_dir}/rivermark_dynamic.yaml" \
  --appearance-config "${demo_dir}/rivermark_appearance_profiles.yaml" \
  "${isaac_runtime_args[@]}" \
  "${dynamic_args[@]}" &
isaac_pid=$!

sleep 8
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH \
  PYTHONPATH ROS_PACKAGE_PATH
set +u
source /opt/ros/jazzy/setup.bash
source "${integration_root}/ros2_ws/install/local_setup.bash"
source "${module3_root}/ros2_ws/install/local_setup.bash"
set -u
ros2 launch robot_bringup rivermark_navigation.launch.py \
  map_file:="${demo_dir}/rivermark_selected.yaml" \
  route_graph_file:="${demo_dir}/rivermark_selected.geojson" \
  region_config_file:="${demo_dir}/rivermark_regions.yaml" \
  start_x:="${start_x}" start_y:="${start_y}" start_yaw_deg:="${start_yaw}" \
  controller_max_linear_velocity_mps:="${controller_max_linear_velocity_mps}" \
  controller_linear_velocity_std_mps:="${controller_linear_velocity_std_mps}" \
  module2_enabled:="$([[ "${mode}" == "module2" ]] && echo true || echo false)" &

if [[ "${mode}" == "module2" ]]; then
  socket="${runtime_dir}/module2-v310.sock"
  conda run --no-capture-output -n bionav-module2 python \
    "${integration_root}/module2_runtime/bio_nav_module2_server/server.py" \
    --runtime-version v310 \
    --module2-root /home/lyb/Workspace/Bio_Nav/repos/MODULE2_SRDR_V310_MODULE3_HANDOFF_20260812 \
    --config configs/module2_pdf_v310_module3.yaml \
    --checkpoint weights/module2_srdr_v310_seed20260822.pt \
    --bridge-source "${integration_root}/ros2_ws/src/bio_nav_ros_bridge" \
    --socket "${socket}" &
  sleep 4
  ros2 launch bio_nav_ros_bridge attempt31_rivermark.launch.py \
    socket_path:="${socket}" use_sim_time:=true \
    guidance_profile:="${guidance_profile}" &
fi

if [[ "${auto_goal}" == "0" ]]; then
  echo "Rivermark runtime ready for an external experiment runner; automatic goal disabled"
  wait "${isaac_pid}"
  exit $?
fi

goal_deadline=$((SECONDS + ${RIVERMARK_STARTUP_TIMEOUT_S:-120}))
while (( SECONDS < goal_deadline )); do
  subscription_count="$(
    ros2 topic info /bio_nav/route_goal 2>/dev/null \
      | awk '/Subscription count:/ {print $3}'
  )" || true
  nav_state="$(ros2 lifecycle get /bt_navigator 2>/dev/null || true)"
  if [[ "${subscription_count:-0}" =~ ^[1-9][0-9]*$ ]] \
      && [[ "${nav_state}" == active* ]]; then
    break
  fi
  sleep 1
done
if (( SECONDS >= goal_deadline )); then
  echo "Rivermark ROS startup timed out before route goal dispatch" >&2
  exit 4
fi
read -r goal_qz goal_qw < <(python3 - "${goal_yaw}" <<'PY'
import math,sys
yaw=math.radians(float(sys.argv[1]))
print(math.sin(yaw/2.0), math.cos(yaw/2.0))
PY
)
ros2 topic pub --once /bio_nav/route_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: ${goal_x}, y: ${goal_y}, z: 0.0}, orientation: {z: ${goal_qz}, w: ${goal_qw}}}}"

wait "${isaac_pid}"
