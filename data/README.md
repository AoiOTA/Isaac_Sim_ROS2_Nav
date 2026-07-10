# Experiment data

This tree is the local output boundary for maps, recordings, trajectories,
metrics, reports, and repeatable experiment runs. Directory placeholders are
versioned, while generated contents are ignored by default.

| Directory | Purpose |
| --- | --- |
| `maps/occupancy/` | Saved occupancy-grid images and metadata |
| `maps/posegraphs/` | SLAM Toolbox pose graphs and serialized state |
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
use Git LFS or an external artifact store rather than ordinary Git history. To
adopt Git LFS for selected formats, first agree on the formats and repository
policy, narrow the corresponding root ignore rule, and create and commit the
resulting `.gitattributes`, for example:

```bash
git lfs install
git lfs track "data/maps/**/*.pgm"
git lfs track "data/maps/**/*.posegraph"
```

For externally stored artifacts, commit only a small manifest containing the
artifact URI, checksum, byte size, format/version, creation scenario, and map to
USD alignment metadata. Verify the checksum after retrieval before running an
experiment.
