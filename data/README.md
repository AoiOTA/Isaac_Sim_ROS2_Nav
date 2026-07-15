# Experiment data

This tree is the local output boundary for maps, recordings, trajectories,
metrics, reports, and repeatable experiment runs. Directory placeholders are
versioned, while generated contents are ignored by default.

| Directory | Purpose |
| --- | --- |
| `maps/occupancy/` | Saved occupancy-grid images and metadata |
| `maps/posegraphs/` | SLAM Toolbox pose graphs and serialized state |
| `maps/manifests/` | Small, reviewable integrity/provenance records for map versions |
| `bags/` | ROS 2 bag recordings |
| `trajectories/` | Estimated and ground-truth trajectories |
| `metrics/` | Per-run and aggregate metric output |
| `reports/` | Generated CSV, JSON, plots, and human-readable reports |
| `experiment_runs/` | Run manifests and scenario-specific output bundles |

## Generated data policy

Do not commit routine simulator output or experiment batches. Keep them locally
or publish them to the designated artifact store. Each published run should
include enough provenance to reproduce it: scenario and random seed, map and
pose-graph versions, robot and Nav2 configuration hashes, named spawn pose, USD
and map start poses, goal, obstacle trajectories, physics timestep, real-time
factor, result, and failure reason.

Small deterministic data used by automated tests may be versioned below a
`fixtures/` directory. Prefer minimal synthetic samples, document how they were
created, and keep each fixture small enough for ordinary Git review. A fixture
must not contain credentials, machine-specific paths, or an externally licensed
warehouse asset.

## Maps and pose graphs

Generated maps are ignored by default. Large curated Pose Graph files must use
Git LFS or an external artifact store rather than ordinary Git history. The
validated `warehouse_v1` bundle is the calibrated repository baseline;
`warehouse_v2` is a recovered, uncalibrated candidate whose provenance and
runtime alignment are not yet verified. Each large `.posegraph` is tracked by
Git LFS, while its matching OccupancyGrid and `.data` are regular Git artifacts.
Hydrate LFS after cloning, then follow the normal asset/build/preflight order:

```bash
git lfs install
git lfs pull
./scripts/import_assets.sh
./scripts/build_ros2.sh
./scripts/preflight.sh
```

Install ROS/Python dependencies first as described in `docs/user_manual.md`;
running preflight before the local asset import and ROS workspace build fails by
design.

For externally stored artifacts, commit only a small manifest containing the
artifact URI, checksum, byte size, format/version, creation scenario, and map to
USD alignment metadata. Verify the checksum after retrieval before running an
experiment.

The current repository bundles are described by
`data/maps/manifests/warehouse_v1.yaml` and `warehouse_v2.yaml`. Each records the
byte size and SHA256 of the OccupancyGrid (`.yaml`/`.pgm`) and serialized Pose
Graph (`.posegraph`/`.data`), plus map dimensions, origin, source environment,
and calibration state. Those four generated files are one indivisible version:
do not mix artifacts from different mapping runs. `preflight.sh` automatically
checks the calibrated v1 release baseline; v2 can be checked explicitly with
`map_manifest verify`, but must not be treated as calibrated or accepted for
formal navigation statistics. New generated versions remain local until
deliberately curated and added to the manifest/LFS policy.

Localization and Navigation consume both halves of that version for different
purposes: `nav2_map_server` serves the saved `.yaml`/`.pgm` pair as the immutable
`/map`, while SLAM Toolbox loads `.posegraph`/`.data` to estimate and publish
`map -> odom`. `scripts/run_ros.sh` derives
`maps/occupancy/<posegraph-basename>.yaml` when `map_file` is omitted, so matching
basenames are the normal convention. Pass `map_file:=...` explicitly for a
deliberate nonmatching name; never substitute SLAM Toolbox's diagnostic
`/slam_toolbox/map` for the saved OccupancyGrid in a navigation run.
