# Nav2 Smac source binding

`TieBreakSmacPlanner2D` derives its search and path-conversion flow from the
Apache-2.0 Nav2 `SmacPlanner2D` release `1.3.12`:

- upstream: `https://github.com/ros-navigation/navigation2`
- tag: `1.3.12`
- commit: `6be3614013ec586051b86c97b919b293281490fe`
- package: `nav2_smac_planner`
- local binary contract: ROS 2 Jazzy package version `1.3.12`

The only search-order extension is the lexicographic open-set key
`(Smac f-cost, -SR tie score, deterministic serial)`. Traversal cost,
Node2D heuristic, collision checker, tolerance handling, world-coordinate
conversion, and Smac smoother remain aligned with the pinned implementation.

The upstream copyright and Apache-2.0 SPDX notice are retained in the derived
source file.
