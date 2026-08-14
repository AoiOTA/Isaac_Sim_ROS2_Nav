#!/usr/bin/env bash
set -Eeuo pipefail

module3_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
integration_root="${BIO_NAV_INTEGRATION_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/integration/attempt31-outdoor-nav}"
demo_dir="${RIVERMARK_DEMO_DIR:-${module3_root}/data/rivermark_demo}"
asset="${RIVERMARK_USD:-/home/lyb/Rivermark/rivermark.usd}"
mode="${1:-off}"
scenario="${2:-static}"
appearance_profile="${3:-${RIVERMARK_APPEARANCE_PROFILE:-}}"
domain_id="${ROS_DOMAIN_ID:-231}"
runtime_dir="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/bionav-rivermark-${domain_id}"
auto_goal="${RIVERMARK_AUTO_GOAL:-1}"
ground_truth="${RIVERMARK_GROUND_TRUTH:-0}"
visual_route="${RIVERMARK_VISUAL_ROUTE:-0}"
guidance_profile="${RIVERMARK_GUIDANCE_PROFILE:-}"
controller_max_linear_velocity_mps="${RIVERMARK_MAX_LINEAR_SPEED_MPS:-0.75}"
controller_linear_velocity_std_mps="${RIVERMARK_LINEAR_SPEED_STD_MPS:-0.35}"
rendering_hz="${RIVERMARK_RENDERING_HZ:-30}"

if [[ "${mode}" != "off" && "${mode}" != "module2" ]]; then
  echo "usage: $0 [off|module2] [static|dynamic|appearance] [appearance-profile]" >&2
  exit 2
fi
if [[ "${scenario}" != "static" && "${scenario}" != "dynamic" && "${scenario}" != "appearance" ]]; then
  echo "usage: $0 [off|module2] [static|dynamic|appearance] [appearance-profile]" >&2
  exit 2
fi
if [[ -z "${appearance_profile}" ]]; then
  appearance_profile="baseline"
  [[ "${scenario}" == "appearance" ]] && appearance_profile="bright_warm"
fi
if [[ "${auto_goal}" != "0" && "${auto_goal}" != "1" ]]; then
  echo "RIVERMARK_AUTO_GOAL must be 0 or 1" >&2
  exit 2
fi
if [[ "${ground_truth}" != "0" && "${ground_truth}" != "1" ]]; then
  echo "RIVERMARK_GROUND_TRUTH must be 0 or 1" >&2
  exit 2
fi
if [[ "${visual_route}" != "0" && "${visual_route}" != "1" ]]; then
  echo "RIVERMARK_VISUAL_ROUTE must be 0 or 1" >&2
  exit 2
fi
if [[ "${scenario}" == "appearance" && "${appearance_profile}" == "baseline" ]]; then
  echo "appearance navigation requires a non-baseline appearance profile" >&2
  exit 2
fi
python3 - "${demo_dir}/rivermark_appearance_profiles.yaml" "${appearance_profile}" <<'PY'
import sys
import yaml

payload = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
profiles = payload.get("profiles", {}) if isinstance(payload, dict) else {}
if sys.argv[2] not in profiles:
    raise SystemExit(f"unknown Rivermark appearance profile: {sys.argv[2]}")
PY
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

isaac_pid=""
ros_pid=""
module2_pid=""
bridge_launch_pid=""
socket=""
isaac_console_log="${RIVERMARK_ISAAC_CONSOLE_LOG:-${runtime_dir}/isaac-console.log}"
declare -a managed_process_groups=()

terminate_tree() {
  local parent_pid="$1"
  local signal="$2"
  local child_pid
  while read -r child_pid; do
    [[ -n "${child_pid}" ]] && terminate_tree "${child_pid}" "${signal}"
  done < <(ps -o pid= --ppid "${parent_pid}" 2>/dev/null)
  kill "-${signal}" "${parent_pid}" 2>/dev/null || true
}

group_has_live_members() {
  local process_group="$1"
  ps -eo pgid=,stat= | awk -v group="${process_group}" '
    $1 == group && $2 !~ /^Z/ { found=1 }
    END { exit(found ? 0 : 1) }
  '
}

remember_process_group() {
  local pid="$1" label="$2" process_group=""
  local deadline=$((SECONDS + 5))
  while (( SECONDS < deadline )); do
    kill -0 "${pid}" 2>/dev/null \
      || { echo "${label} exited before establishing its process group" >&2; return 1; }
    process_group="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
    if [[ "${process_group}" == "${pid}" ]]; then
      managed_process_groups+=("${process_group}")
      return 0
    fi
    sleep 0.1
  done
  echo "${label} did not establish a dedicated process group (pid=${pid}, pgid=${process_group:-unknown})" >&2
  return 1
}

stop_managed_process_groups() {
  local signal_name="$1" process_group
  for process_group in "${managed_process_groups[@]}"; do
    [[ "${process_group}" =~ ^[1-9][0-9]*$ ]] || continue
    group_has_live_members "${process_group}" || continue
    kill "-${signal_name}" -- "-${process_group}" 2>/dev/null || true
  done
}

stale_module2_pids() {
  local socket_path="$1"
  ps -eo pid=,uid=,comm=,args= | awk \
    -v owner="$(id -u)" -v socket_path="${socket_path}" '
      $2 == owner && ($3 == "python" || $3 == "python3") &&
      index($0, "bio_nav_module2_server/server.py") &&
      index($0, socket_path) { print $1 }
    '
}

socket_accepts_connections() {
  local socket_path="$1"
  python3 - "${socket_path}" <<'PY'
import socket
import sys

client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
client.settimeout(0.5)
try:
    client.connect(sys.argv[1])
except OSError:
    raise SystemExit(1)
finally:
    client.close()
PY
}

remove_stale_module2_runtime() {
  local socket_path="$1" stale_pid
  local -a stale_pids=()
  mapfile -t stale_pids < <(stale_module2_pids "${socket_path}")
  if [[ -S "${socket_path}" ]] && socket_accepts_connections "${socket_path}"; then
    echo "Rivermark Module2 is already active on ${socket_path}; stop that run before starting another" >&2
    return 1
  fi
  if ((${#stale_pids[@]})); then
    echo "Removing unreachable Rivermark Module2 runtime: pids=${stale_pids[*]} socket=${socket_path}"
    for stale_pid in "${stale_pids[@]}"; do terminate_tree "${stale_pid}" TERM; done
    sleep 1
    for stale_pid in "${stale_pids[@]}"; do
      kill -0 "${stale_pid}" 2>/dev/null && terminate_tree "${stale_pid}" KILL
    done
  fi
  # The server performs its own owner/type/connectivity checks before replacing
  # a stale filesystem entry.  Never unlink an arbitrary path here.
  # A successful TERM normally makes the final kill -0 return 1.  Do not leak
  # that expected probe status from this function into the launcher's `set -e`.
  return 0
}

cleanup() {
  local exit_status="${1:-0}"
  local job_pid wait_round
  local -a job_pids=()
  trap - EXIT INT TERM
  # Every long-lived component has its own process group.  Signal the group,
  # not only the shell job leader: run_isaac.sh may be killed while Kit is
  # still shutting down and Kit would otherwise be reparented and left alive.
  stop_managed_process_groups INT
  for wait_round in 1 2 3 4 5; do
    local any_live=0 process_group
    for process_group in "${managed_process_groups[@]}"; do
      group_has_live_members "${process_group}" && any_live=1
    done
    (( any_live == 0 )) && break
    sleep 1
  done
  stop_managed_process_groups TERM
  sleep 1
  stop_managed_process_groups KILL
  mapfile -t job_pids < <(jobs -pr)
  for job_pid in "${job_pids[@]}"; do terminate_tree "${job_pid}" TERM; done
  wait 2>/dev/null || true
  if [[ -n "${socket}" && -S "${socket}" ]]; then
    local socket_owner
    socket_owner="$(stat -c '%u' "${socket}")"
    [[ "${socket_owner}" == "$(id -u)" ]] && rm -f -- "${socket}"
  fi
  exit "${exit_status}"
}
trap 'cleanup $?' EXIT
trap 'exit 130' INT TERM

dynamic_args=(--no-dynamic-obstacles)
if [[ "${scenario}" == "dynamic" ]]; then
  dynamic_case="${RIVERMARK_DYNAMIC_CASE:-full_route_four_stage}"
  dynamic_variant="${RIVERMARK_DYNAMIC_VARIANT:-v3}"
  dynamic_args=(
    --dynamic-obstacles
    --dynamic-case-id "${dynamic_case}"
    --dynamic-variant-id "${dynamic_variant}"
  )
fi
isaac_runtime_args=()
if [[ "${RIVERMARK_HEADLESS:-0}" == "1" ]]; then
  isaac_runtime_args+=(--headless)
fi
rviz_enabled="${RIVERMARK_RVIZ:-}"
if [[ -z "${rviz_enabled}" ]]; then
  rviz_enabled="1"
  if [[ "${RIVERMARK_HEADLESS:-0}" == "1" ]]; then
    rviz_enabled="0"
  fi
fi
if [[ "${rviz_enabled}" != "0" && "${rviz_enabled}" != "1" ]]; then
  echo "RIVERMARK_RVIZ must be 0 or 1" >&2
  exit 2
fi
if [[ -n "${RIVERMARK_MAX_STEPS:-}" ]]; then
  isaac_runtime_args+=(--max-steps "${RIVERMARK_MAX_STEPS}")
fi

# Load Module2 before the 12 GB USD competes for CPU/GPU resources.  A prior
# interrupted visual run can leave a server listening on an unlinked socket;
# remove only that exact, unreachable same-user runtime before starting again.
if [[ "${mode}" == "module2" ]]; then
  socket="${runtime_dir}/module2-v310.sock"
  remove_stale_module2_runtime "${socket}"
  echo "Starting Rivermark Module2 before Isaac scene loading"
  setsid -- conda run --no-capture-output -n bionav-module2 python \
    "${integration_root}/module2_runtime/bio_nav_module2_server/server.py" \
    --runtime-version v310 \
    --module2-root /home/lyb/Workspace/Bio_Nav/repos/MODULE2_SRDR_V310_MODULE3_HANDOFF_20260812 \
    --config configs/module2_pdf_v310_module3.yaml \
    --checkpoint weights/module2_srdr_v310_seed20260822.pt \
    --bridge-source "${integration_root}/ros2_ws/src/bio_nav_ros_bridge" \
    --socket "${socket}" &
  module2_pid=$!
  remember_process_group "${module2_pid}" "Rivermark Module2"
  module2_deadline=$((SECONDS + ${RIVERMARK_MODULE2_STARTUP_TIMEOUT_S:-120}))
  while (( SECONDS < module2_deadline )) && [[ ! -S "${socket}" ]]; do
    if ! kill -0 "${module2_pid}" 2>/dev/null; then
      wait "${module2_pid}" || true
      echo "Rivermark Module2 server exited before its socket became ready" >&2
      exit 5
    fi
    sleep 1
  done
  if [[ ! -S "${socket}" ]]; then
    echo "Rivermark Module2 socket did not become ready within ${RIVERMARK_MODULE2_STARTUP_TIMEOUT_S:-120}s: ${socket}" >&2
    exit 5
  fi
  echo "Rivermark Module2 socket ready: ${socket}"
fi

# Rivermark's source USD contains hundreds of thousands of repeated Hydra
# warnings for incomplete curve/foliage authoring.  Rendering those lines in
# an interactive terminal can freeze the terminal itself.  Kit already keeps
# its own detailed log; retain a per-run console log here and keep this
# one-terminal launcher limited to actionable lifecycle status.
: >"${isaac_console_log}"
echo "Isaac console log: ${isaac_console_log}"
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
  --appearance-profile "${appearance_profile}" \
  "${isaac_runtime_args[@]}" \
  "${dynamic_args[@]}" >"${isaac_console_log}" 2>&1 &
isaac_pid=$!
remember_process_group "${isaac_pid}" "Rivermark Isaac"

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH \
  PYTHONPATH ROS_PACKAGE_PATH
set +u
source /opt/ros/jazzy/setup.bash
source "${integration_root}/ros2_ws/install/local_setup.bash"
source "${module3_root}/ros2_ws/install/local_setup.bash"
set -u

# Do not open RViz against a half-loaded scene.  It otherwise reports missing
# TF/costmap/marker data for roughly a minute and looks frozen while Isaac is
# still building the stage.  One ROS process waits for actual fresh messages.
isaac_readiness_status=0
python3 - "${isaac_pid}" "${RIVERMARK_ISAAC_READY_TIMEOUT_S:-180}" <<'PY' \
  || isaac_readiness_status=$?
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2

isaac_pid = int(sys.argv[1])
timeout_s = float(sys.argv[2])
deadline = time.monotonic() + timeout_s
last_report = 0.0
received = set()
rclpy.init()
node = Node("rivermark_isaac_readiness")
qos = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
node.create_subscription(Clock, "/clock", lambda _: received.add("clock"), qos)
# /scan is created later by the pointcloud_to_laserscan node inside the Nav2
# launch.  Waiting for it here would deadlock startup.  Isaac directly owns
# /lidar/points_raw, so it is the correct pre-Nav2 sensor readiness barrier.
node.create_subscription(
    PointCloud2,
    "/lidar/points_raw",
    lambda _: received.add("lidar_points_raw"),
    qos,
)
node.create_subscription(Odometry, "/odom", lambda _: received.add("odom"), qos)
required = {"clock", "lidar_points_raw", "odom"}
try:
    while time.monotonic() < deadline and received != required:
        try:
            os.kill(isaac_pid, 0)
        except ProcessLookupError:
            raise SystemExit("Rivermark Isaac exited before sensor readiness")
        rclpy.spin_once(node, timeout_sec=0.5)
        now = time.monotonic()
        if now - last_report >= 10.0:
            missing = ",".join(sorted(required - received)) or "none"
            print(f"Waiting for Rivermark Isaac sensor readiness; missing={missing}", flush=True)
            last_report = now
    if received != required:
        missing = ",".join(sorted(required - received))
        raise SystemExit(f"Rivermark Isaac readiness timed out; missing={missing}")
    print("Rivermark Isaac sensors ready; starting Nav2 and RViz", flush=True)
except KeyboardInterrupt:
    # Ctrl+C during the cold-start barrier is a normal operator stop.  rclpy's
    # signal handler may already have shut the context down, so avoid printing
    # a misleading double-shutdown traceback.
    raise SystemExit(130)
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY
if (( isaac_readiness_status != 0 )); then
  echo "Rivermark Isaac failed during startup; recent actionable log lines:" >&2
  grep -E 'Traceback|  File |Error|Exception|KeyError|RuntimeError|fatal|Fatal' \
    "${isaac_console_log}" | tail -40 >&2 || true
  echo "Full Isaac console log: ${isaac_console_log}" >&2
  exit "${isaac_readiness_status}"
fi

setsid -- ros2 launch robot_bringup rivermark_navigation.launch.py \
  map_file:="${demo_dir}/rivermark_selected.yaml" \
  route_graph_file:="${demo_dir}/rivermark_selected.geojson" \
  region_config_file:="${demo_dir}/rivermark_regions.yaml" \
  waypoint_config_file:="${demo_dir}/rivermark_demo_goals.yaml" \
  start_x:="${start_x}" start_y:="${start_y}" start_yaw_deg:="${start_yaw}" \
  controller_max_linear_velocity_mps:="${controller_max_linear_velocity_mps}" \
  controller_linear_velocity_std_mps:="${controller_linear_velocity_std_mps}" \
  use_rviz:="${rviz_enabled}" \
  module2_enabled:="$([[ "${mode}" == "module2" ]] && echo true || echo false)" &
ros_pid=$!
remember_process_group "${ros_pid}" "Rivermark Nav2/RViz"

if [[ "${mode}" == "module2" ]]; then
  bridge_launch_args=(
    socket_path:="${socket}"
    use_sim_time:=true
  )
  if [[ -n "${guidance_profile}" ]]; then
    bridge_launch_args+=(guidance_profile:="${guidance_profile}")
  fi
  setsid -- ros2 launch bio_nav_ros_bridge attempt31_rivermark.launch.py \
    "${bridge_launch_args[@]}" &
  bridge_launch_pid=$!
  remember_process_group "${bridge_launch_pid}" "Rivermark Module2 Bridge"
  bridge_deadline=$((SECONDS + ${RIVERMARK_BRIDGE_STARTUP_TIMEOUT_S:-60}))
  bridge_ready=0
  while (( SECONDS < bridge_deadline )); do
    if ! kill -0 "${module2_pid}" 2>/dev/null; then
      echo "Rivermark Module2 server exited during Bridge startup" >&2
      exit 5
    fi
    if ! kill -0 "${bridge_launch_pid}" 2>/dev/null; then
      wait "${bridge_launch_pid}" || true
      echo "Rivermark Module2 ROS Bridge exited during startup" >&2
      exit 5
    fi
    bridge_nodes="$(ros2 node list 2>/dev/null || true)"
    if grep -Fxq /bio_nav_ros_bridge <<<"${bridge_nodes}" \
        && grep -Fxq /bio_nav_edge_prior_bridge <<<"${bridge_nodes}"; then
      bridge_ready=1
      break
    fi
    sleep 1
  done
  if [[ "${bridge_ready}" != "1" ]]; then
    echo "Rivermark Module2 ROS Bridge did not become ready within ${RIVERMARK_BRIDGE_STARTUP_TIMEOUT_S:-60}s" >&2
    exit 5
  fi
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
if [[ "${auto_goal}" == "0" ]]; then
  echo "Rivermark manual navigation ready; use RViz 2D Goal Pose on the map"
  echo "RViz goal topic: /bio_nav/route_goal"
  wait "${isaac_pid}"
  exit $?
fi
read -r goal_qz goal_qw < <(python3 - "${goal_yaw}" <<'PY'
import math,sys
yaw=math.radians(float(sys.argv[1]))
print(math.sin(yaw/2.0), math.cos(yaw/2.0))
PY
)
if [[ "${visual_route}" == "1" ]]; then
  visual_route_args=(
    --config "${demo_dir}/rivermark_demo_goals.yaml"
    --leg-timeout-s "${RIVERMARK_VISUAL_LEG_TIMEOUT_S:-240}"
  )
  if [[ "${scenario}" == "dynamic" ]]; then
    visual_route_args+=(--dynamic)
  fi
  ros2 run robot_experiments rivermark_visual_route -- \
    "${visual_route_args[@]}"
  echo "Rivermark five-waypoint visual navigation completed"
else
  ros2 topic pub --once /bio_nav/route_goal geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: ${goal_x}, y: ${goal_y}, z: 0.0}, orientation: {z: ${goal_qz}, w: ${goal_qw}}}}"
fi

wait "${isaac_pid}"
