#!/usr/bin/env bash
set -Eeuo pipefail

module3_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
integration_root="${BIO_NAV_INTEGRATION_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/integration/attempt31-outdoor-nav}"
asset="${RIVERMARK_USD:-/home/lyb/Rivermark/rivermark.usd}"
run_id="${1:-$(date +%Y%m%d_%H%M%S)}"
candidate_dir="${module3_root}/runs/rivermark_roi_selection/${run_id}"
selected_dir="${module3_root}/data/rivermark_demo"
defaults="${integration_root}/ros2_ws/src/bio_nav_ros_bridge/config/engineering_defaults.yaml"
pythonpath="${module3_root}/ros2_ws/src/robot_route_planner"

if pgrep -af 'navigation_sim.py' | grep -qv "${module3_root}"; then
  echo "another Isaac navigation_sim.py is active; retry after it exits" >&2
  exit 3
fi

mkdir -p "${candidate_dir}" "${selected_dir}"
"${ISAACSIM_PYTHON:-/home/lyb/miniconda3/envs/isaacsim/bin/python}" \
  "${module3_root}/isaac_sim/tools/rivermark_prepare.py" \
  --usd "${asset}" \
  --output-dir "${candidate_dir}" \
  --candidate all

for candidate in A B; do
  python3 "${module3_root}/scripts/compare_rivermark_map.py" \
    --rgb "${candidate_dir}/candidate_${candidate}_topdown_rgb.png" \
    --occupancy "${candidate_dir}/candidate_${candidate}_occupancy.png" \
    --output "${candidate_dir}/candidate_${candidate}_rgb_occupancy_overlay.png" \
    --edge-overlay-output "${candidate_dir}/candidate_${candidate}_edge_overlay.png"
  if ! PYTHONPATH="${pythonpath}" python3 -m robot_route_planner.cli \
    --map "${candidate_dir}/candidate_${candidate}_occupancy.yaml" \
    --defaults "${defaults}" \
    --geojson "${candidate_dir}/candidate_${candidate}_gvg.geojson" \
    --mapping "${candidate_dir}/candidate_${candidate}_gvg_mapping.json" \
    --summary "${candidate_dir}/candidate_${candidate}_graph_summary.json"; then
    echo "candidate ${candidate}: graph extraction failed" >&2
    rm -f "${candidate_dir}/candidate_${candidate}_gvg.geojson" \
      "${candidate_dir}/candidate_${candidate}_gvg_mapping.json" \
      "${candidate_dir}/candidate_${candidate}_graph_summary.json"
    continue
  fi
  if ! PYTHONPATH="${pythonpath}" python3 -m robot_route_planner.visualize \
    --map "${candidate_dir}/candidate_${candidate}_occupancy.yaml" \
    --defaults "${defaults}" \
    --geojson "${candidate_dir}/candidate_${candidate}_gvg.geojson" \
    --mapping "${candidate_dir}/candidate_${candidate}_gvg_mapping.json" \
    --output-dir "${candidate_dir}/candidate_${candidate}_gvg_preview"; then
    echo "candidate ${candidate}: preview has no usable alternative route" >&2
  fi
done

python3 "${module3_root}/scripts/finalize_rivermark_roi.py" \
  --candidate-dir "${candidate_dir}" \
  --output-dir "${selected_dir}"

echo "Rivermark candidate outputs: ${candidate_dir}"
echo "Selected demo assets: ${selected_dir}"
