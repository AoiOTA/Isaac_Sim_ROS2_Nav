#!/usr/bin/env bash
# Observe one selected interaction (or the three-stage full route).  This is
# deliberately not a formal acceptance result.
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/lib/common.sh"
case_id="crossing"; variant_id="v1"; seed=7401; record=false; rviz=true
while [[ $# -gt 0 ]]; do case "$1" in
  --case) case_id="$2"; shift 2;; --variant) variant_id="v$2"; [[ "$2" == v* ]] && variant_id="$2"; shift 2;;
  --seed) seed="$2"; shift 2;; --record) record=true; shift;; --no-rviz) rviz=false; shift;;
  *) die "usage: $0 [--case crossing|oncoming|same_direction_slow|local_bypass|temporary_block|g2_g3_exit|g5_g1_crossing|full_route_three_stage] [--variant 1..5] [--seed N] [--record] [--no-rviz]";; esac; done
[[ "$case_id" =~ ^(crossing|oncoming|same_direction_slow|local_bypass|temporary_block|g2_g3_exit|g5_g1_crossing|full_route_three_stage)$ ]] || die "invalid case"
[[ "$variant_id" =~ ^v[1-5]$ && "$seed" =~ ^[0-9]+$ ]] || die "invalid variant or seed"
source_ros --require-workspace
case "$case_id" in
  g2_g3_exit)
    scenario="${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/kujiale_dynamic_visual_g2_g3.yaml";;
  g5_g1_crossing)
    scenario="${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/kujiale_dynamic_visual_g5_g1.yaml";;
  full_route_three_stage)
    # The full relay deliberately retains its calibrated G1->...->G5 ingress.
    scenario="${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/kujiale_dynamic_full_route_5.yaml";;
  *)
    scenario="${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/kujiale_dynamic_visual.yaml";;
esac
out="${PROJECT_ROOT}/data/dynamic_visual/${case_id}-${variant_id}-seed-${seed}-$(date +%Y%m%d-%H%M%S)"
# Bypass ros2cli's long-lived daemon and keep one participant alive long
# enough to receive Fast DDS periodic discovery announcements.  Repeated
# one-second processes reset discovery every time and can falsely miss a
# healthy publisher for the full retry window.
ground_truth_info="$(
  ros2 topic info --no-daemon --spin-time 15.0 /ground_truth/odom 2>/dev/null \
    || true
)"
if ! grep -Eq 'Publisher count: [1-9][0-9]*' <<<"${ground_truth_info}"; then
  die "/ground_truth/odom has no publisher after a continuous 15 s discovery window; start ./scripts/run_kujiale_dynamic_isaac.sh and wait for ground_truth=True before running this script"
fi

# The runner enforces this same contract, but checking it here avoids starting
# a launch only to print a Python traceback when Isaac was started before a
# dynamic-obstacle YAML edit.  The actor configuration is read by Isaac only
# at startup, whereas the experiment runner hashes the current workspace file.
expected_dynamic_hash="$(sha256sum "${PROJECT_ROOT}/isaac_sim/configs/experiments/kujiale_long_range_dynamic.yaml" | awk '{print $1}')"
runtime_dynamic_contract="$(
  ros2 param get --no-daemon --spin-time 3.0 \
    /isaac_navigation_sim dynamic_obstacles_config_sha256 2>/dev/null || true
)"
runtime_dynamic_hash="$(sed -n 's/^String value is: //p' <<<"${runtime_dynamic_contract}" | tail -n 1)"
if [[ -n "${runtime_dynamic_hash}" && "${runtime_dynamic_hash}" != "${expected_dynamic_hash}" ]]; then
  die "Isaac is using an older dynamic-obstacle YAML (runtime ${runtime_dynamic_hash}, workspace ${expected_dynamic_hash}); restart Isaac with the matching --spawn-pose before this run"
fi
args=("${SCRIPT_DIR}/run_experiment.sh" "$scenario" "$out" "record_evidence:=$record" "dynamic_case_id:=$case_id" "dynamic_variant_id:=$variant_id" "dynamic_seed:=$seed")
if $rviz; then log_info "Use the navigation.rviz started by ./scripts/run_ros.sh navigation to observe this run."; fi
exec "${args[@]}"
