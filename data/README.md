# Experiment data

This tree is the local output boundary for maps, recordings, trajectories,
metrics, reports, and repeatable experiment runs. Directory placeholders are
versioned, while generated contents are ignored by default.

> 当前分支说明：正式酷家乐报告目录（例如
> `data/reports/kujiale_long_route_<campaign_id>/`）是本地交付工件，包含 HTML、PDF、PNG、CSV、JSON 和运行证据，因体积和原始数据原因不推送到 Git；报告生成器、schema、校验和测试规格受版本控制。

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

Generated maps are ignored by default. Large curated maps and pose graphs must
use Git LFS or an external artifact store rather than ordinary Git history. The
validated `warehouse_new` bundle is this custom-scene branch's baseline: its large
`.posegraph` is tracked by Git LFS, while the matching OccupancyGrid and `.data`
file are regular Git artifacts. Hydrate it after cloning:

```bash
git lfs install
git lfs pull
./scripts/preflight.sh
```

For externally stored artifacts, commit only a small manifest containing the
artifact URI, checksum, byte size, format/version, creation scenario, and map to
USD alignment metadata. Verify the checksum after retrieval before running an
experiment.

The current repository baseline is described by
`data/maps/manifests/warehouse_new.yaml`. It records the byte size and SHA256 of
the OccupancyGrid (`.yaml`/`.pgm`) and serialized Pose Graph
(`.posegraph`/`.data`), plus map dimensions, origin, source environment, and the
Map/USD calibration pair. Those four generated files are one indivisible
version: do not mix artifacts from different mapping runs. `preflight.sh`
rejects missing, unhydrated, size-mismatched, or checksum-mismatched baseline
files. New generated versions remain local until deliberately curated and
added to the manifest/LFS policy.

Localization and Navigation validate both halves of that version:
`nav2_map_server` serves the saved `.yaml`/`.pgm` pair as immutable `/map`.
Normal Ideal operation uses the calibrated identity `map -> odom`.
`warehouse_new` was generated with scan matching and loop closing disabled, so
its serialized Pose Graph remains mapping provenance and is not approved for
Realistic localization or explicit Pose Graph calibration. `scripts/run_ros.sh`
defaults to `warehouse_new` and derives
`maps/occupancy/<posegraph-basename>.yaml` when `map_file` is omitted, so matching
basenames are the normal convention. Pass `map_file:=...` explicitly for a
deliberate nonmatching name; never substitute SLAM Toolbox's diagnostic
`/slam_toolbox/map` for the saved OccupancyGrid in a navigation run.
