#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
source_ros

query_timeout="${ISAAC_NAV_DIAGNOSE_TIMEOUT:-3}"
[[ "${query_timeout}" =~ ^[0-9]+([.][0-9]+)?$ ]] \
  || die "ISAAC_NAV_DIAGNOSE_TIMEOUT must be numeric"
require_command timeout

section() {
  printf '\n== %s ==\n' "$1"
}

run_query() {
  local label="$1"
  shift
  printf '\n-- %s --\n' "${label}"
  timeout "${query_timeout}" "$@" 2>&1 || {
    local status=$?
    if [[ ${status} -eq 124 ]]; then
      printf '[timeout after %ss]\n' "${query_timeout}"
    else
      printf '[unavailable, exit=%s]\n' "${status}"
    fi
  }
}

section "Environment"
printf 'PROJECT_ROOT=%s\n' "${PROJECT_ROOT}"
printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-unset}"
printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID}"
printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION}"
printf 'ROS_SETUP=%s\n' "${ROS_SETUP}"
printf 'WORKSPACE_SETUP=%s\n' "${PROJECT_ROOT}/ros2_ws/install/setup.bash"

section "Registered runtime"
prepare_runtime_directory
for component in isaac ros rviz teleop; do
  pid_file="$(runtime_pid_file "${component}")"
  if [[ -r "${pid_file}" ]]; then
    printf '%s:\n' "${component}"
    sed 's/^/  /' "${pid_file}"
    pid="$(sed -n 's/^pid=//p' "${pid_file}" | head -n 1)"
    if [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
      printf '  state=alive\n'
      if [[ -r "/proc/${pid}/cmdline" ]]; then
        printf '  command='
        tr '\0' ' ' <"/proc/${pid}/cmdline"
        printf '\n'
      fi
    else
      printf '  state=stale\n'
    fi
  else
    printf '%s: not registered\n' "${component}"
  fi
done

section "Project-related processes"
ps -eo pid=,ppid=,user=,stat=,args= | awk -v root="${PROJECT_ROOT}" -v self="$$" '
  $1 != self && $0 !~ /diagnose[.]sh/ && $0 !~ /awk -v root=/ {
    if (index($0, root) || $0 ~ /ros2 launch robot_bringup/ ||
        $0 ~ /robot_teleop.*keyboard_teleop/ ||
        $0 ~ /(^|[[:space:]])rviz2([[:space:]]|$)/) print
  }
' || true

section "ROS graph"
nodes="$(timeout "${query_timeout}" ros2 node list --no-daemon --spin-time 0.5 2>/dev/null || true)"
topics="$(timeout "${query_timeout}" ros2 topic list --no-daemon --spin-time 0.5 2>/dev/null || true)"
services="$(timeout "${query_timeout}" ros2 service list --no-daemon --spin-time 0.5 2>/dev/null || true)"
actions="$(timeout "${query_timeout}" ros2 action list 2>/dev/null || true)"
printf '%s\n' "${nodes:-[no nodes]}"
printf '\nDuplicate node names:\n'
duplicates="$(printf '%s\n' "${nodes}" | sed '/^$/d' | sort | uniq -d)"
printf '%s\n' "${duplicates:-none}"
printf '\nKey topics:\n'
printf '%s\n' "${topics}" | grep -E '^/(clock|map|slam_toolbox/map|scan|lidar/points_raw|odom|tf|tf_static|cmd_vel|cmd_vel_nav|cmd_vel_smoothed|initialpose|goal_pose|global_costmap|local_costmap|collision_monitor)' || true
printf '\nKey services:\n'
printf '%s\n' "${services}" | grep -E '(lifecycle|clear|simulation/reset)' || true
printf '\nKey actions:\n'
printf '%s\n' "${actions}" | grep -E '(navigate_to_pose|navigate_through_poses)' || true

section "Lifecycle"
for node in \
  map_server slam_toolbox controller_server planner_server behavior_server \
  bt_navigator velocity_smoother collision_monitor; do
  if printf '%s\n' "${nodes}" | grep -qx "/${node}"; then
    run_query "/${node}" ros2 lifecycle get "/${node}"
  else
    printf '/%s: not present\n' "${node}"
  fi
done

section "QoS"
for topic in /map /scan /lidar/points_raw /odom /tf /tf_static \
  /global_costmap/costmap /local_costmap/costmap; do
  if printf '%s\n' "${topics}" | grep -qx "${topic}"; then
    run_query "${topic}" ros2 topic info "${topic}" --verbose
  else
    printf '%s: not present\n' "${topic}"
  fi
done

section "Interaction interfaces"
for topic in /initialpose /goal_pose; do
  if printf '%s\n' "${topics}" | grep -qx "${topic}"; then
    run_query "${topic}" ros2 topic info "${topic}" --verbose
  else
    printf '%s: not present\n' "${topic}"
  fi
done
if printf '%s\n' "${actions}" | grep -qx '/navigate_to_pose'; then
  run_query "/navigate_to_pose" ros2 action info /navigate_to_pose
else
  printf '/navigate_to_pose: not present\n'
fi

section "TF"
for pair in 'map odom' 'odom base_link' 'map base_link'; do
  # Jazzy tf2_echo has no --once option, so every query is bounded whether a
  # sample is printed or the transform is missing.
  read -r target source <<<"${pair}"
  run_query "${target} -> ${source}" \
    ros2 run tf2_ros tf2_echo "${target}" "${source}"
done

section "Simulation time"
run_query "/clock sample" ros2 topic echo /clock --once
run_query "/scan header" ros2 topic echo /scan header --once
run_query "/odom header" ros2 topic echo /odom header --once

section "Fast DDS shared memory"
shm_root="${ISAAC_NAV_SHM_ROOT:-/dev/shm}"
if [[ -d "${shm_root}" ]]; then
  shopt -s nullglob
  shm_files=(
    "${shm_root}"/fastrtps_*
    "${shm_root}"/fastdds_*
    "${shm_root}"/sem.fastrtps_*
    "${shm_root}"/sem.fastdds_*
  )
  shopt -u nullglob
  if ((${#shm_files[@]})); then
    for path in "${shm_files[@]}"; do
      stat -c '%A uid=%u gid=%g size=%s %n' "${path}" 2>/dev/null || true
    done
  else
    printf 'none\n'
  fi
else
  printf 'SHM root unavailable: %s\n' "${shm_root}"
fi

section "CPU governor"
governors=(/sys/devices/system/cpu/cpu*/cpufreq/scaling_governor)
if [[ -e "${governors[0]}" ]]; then
  for governor in "${governors[@]}"; do
    printf '%s=%s\n' "${governor}" "$(<"${governor}")"
  done | sort -u
else
  printf 'unavailable\n'
fi

section "Recent ROS warnings and errors"
latest_log="${PROJECT_ROOT}/ros2_ws/log/latest"
if [[ -e "${latest_log}" ]]; then
  rg -n -i --glob '*.log' --glob '*.txt' '(error|warn|failed|missed.*rate)' \
    "${latest_log}" 2>/dev/null | tail -n 100 || printf 'none\n'
else
  printf 'no ROS log directory\n'
fi
