# Development guide

第一次使用项目请先阅读 [`user_manual.md`](user_manual.md)；不确定一个文件
属于哪一层或会影响哪些模块时，先查 [`repository_index.md`](repository_index.md)，
再修改源码或配置。

## Supported environment

The baseline environment is:

- Python 3.12;
- Isaac Sim 6.0.1;
- ROS 2 Jazzy;
- Fast DDS via `rmw_fastrtps_cpp`.

Keep the Isaac Sim and ROS 2 Python environments distinct unless an entrypoint
explicitly integrates them. Isaac runtime/integration checks must use Isaac
Sim's launcher; the current USD-only marker tests may use the documented
same-version site-packages fallback. ROS package tests should run through colcon
so package metadata and dependencies are honored.

## Python tooling

Create a repository-local development environment for pure Python checks:

```bash
python3 --version  # must report Python 3.12.x
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -e ".[dev]"
```

The root pytest configuration discovers tests in `isaac_sim/tests` and
recursively under `ros2_ws/src`, including conventional ROS Python package
`test/` directories. Inspect collection without running tests with:

```bash
python3 -m pytest --collect-only
```

Run pure tests independently of simulator and ROS graph integration tests:

```bash
python3 -m pytest -m "not isaac and not ros"
```

Apply the `isaac`, `ros`, `integration`, or `unit` marker to tests according to
their runtime requirements. Do not make an Isaac import at module collection
time in a pure unit test.

## ROS 2 workspace

In a Jazzy shell, install declared package dependencies and build the workspace:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install
source install/setup.bash
```

Run package tests and render all failures:

```bash
cd ros2_ws
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

The generated `build/`, `install/`, and `log/` directories are local artifacts
and must not be committed.

## Isaac Sim checks

Set `ISAAC_PYTHON` to the Python executable from the local Isaac Sim 6.0.1
environment. The exact path is machine-specific and must not be embedded in
project configuration. For the pip/Conda installation used by this workspace:

```bash
export ISAAC_PYTHON=/home/lyb/miniconda3/envs/isaacsim/bin/python
./scripts/test.sh --with-isaac
```

An extracted Isaac Sim installation can instead point `ISAAC_PYTHON` at its
`python.sh` launcher. The unified test script first tries pytest inside the
Isaac environment. A stock Isaac Sim Conda environment may not contain pytest;
in that case the script runs system pytest with Isaac's Python 3.12
site-packages on `PYTHONPATH`, which is sufficient for the current USD-only
Isaac marker tests. Do not install packages into the Isaac environment merely
to make this fallback unnecessary.

To run only the Isaac-marked tests manually, use one of these equivalent
paths:

```bash
# If pytest is already installed in the Isaac environment
"${ISAAC_PYTHON}" -m pytest isaac_sim/tests -m isaac

# Stock environment fallback used by scripts/test.sh
ISAAC_SITE_PACKAGES="$(
  "${ISAAC_PYTHON}" -c 'import site; print(site.getsitepackages()[0])'
)"
PYTHONPATH="${ISAAC_SITE_PACKAGES}:${PYTHONPATH:-}" \
  python3 -m pytest isaac_sim/tests -m isaac
```

Tests requiring rendering, RTX sensors, PhysX stepping, or a running ROS bridge
are integration tests. Record the simulator version, command, and concise result
in the commit or handoff notes, while keeping generated logs out of Git.

The complete non-interactive validation entrypoints are:

```bash
./scripts/preflight.sh
./scripts/build_ros2.sh
./scripts/test.sh
./scripts/test.sh --with-isaac
```

`test.sh --with-isaac` already includes the pure-Python and colcon suites; it is
not necessary to run both `test.sh` forms for one final verification pass.

## Runtime inspection

Once the simulator and ROS graph are running, these checks provide acceptance
evidence for timing, sensor publication, and TF ownership:

```bash
ros2 topic hz /clock
ros2 topic hz /lidar/points_raw
ros2 topic hz /scan
ros2 topic info --verbose /odom
ros2 topic info --verbose /map
ros2 topic info --verbose /slam_toolbox/map
ros2 topic info --verbose /tf
ros2 lifecycle get /map_server
ros2 run tf2_tools view_frames
```

Treat generated TF diagrams as temporary evidence unless a deliberately curated
small fixture is needed by a test.

Use the mode matrix and runtime contracts in `docs/interfaces.md` while
interpreting these probes. In particular, `/odom` must have exactly one
publisher and `map -> odom` must have exactly one owner. In Localization or
Navigation, `/map` must have exactly one `map_server` publisher, while
`/slam_toolbox/map` is the diagnostic SLAM output; in Mapping, SLAM Toolbox owns
`/map` and no `map_server` should run. A positive `/simulation/reset` response
is not by itself a readiness check; wait for `/simulation/localization_seeded`,
fresh odometry, and a strictly newer stable TF afterward.

## Calibration and acceptance records

`isaac_sim/configs/spawn_poses.yaml` is the single source of truth for the USD
and Map poses. Never set `map.calibrated: true` from an assumed transform.
Follow `docs/calibration.md`, keep Ground Truth disabled during calibration,
and commit the measured pose together with its map/Pose Graph version and
verification evidence.

Keep smoke evidence separate from statistical acceptance. The current status is
tracked in `docs/verification.md`; an RTX topic-rate check, one successful SLAM
run, a small deterministic navigation batch, or passing unit tests does not
establish broad Localization/Nav2 success rates. When the statistical matrix is
eventually run, preserve raw reports under the ignored `data/` boundary and
commit only deliberately curated summaries.

## Commit and data discipline

Follow `CONTRIBUTING.md`: keep code, its configuration, tests, and necessary
documentation in one coherent commit. Do not commit the externally licensed
warehouse source, rosbags, generated experiment batches, or simulator logs. The
deliberately curated `warehouse_v2` map bundle is the exception: its manifest
pins every digest and its large Pose Graph is stored with Git LFS. See
`data/README.md` before versioning another map or pose graph.
