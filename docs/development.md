# Development guide

## Supported environment

The baseline environment is:

- Python 3.12;
- Isaac Sim 6.0.1;
- ROS 2 Jazzy;
- Fast DDS via `rmw_fastrtps_cpp`.

Keep the Isaac Sim and ROS 2 Python environments distinct unless an entrypoint
explicitly integrates them. Isaac-dependent tests must run with Isaac Sim's
Python launcher. ROS package tests should run through colcon so package metadata
and dependencies are honored.

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
"${ISAAC_PYTHON}" -m pytest isaac_sim/tests -m isaac
```

An extracted Isaac Sim installation can instead point `ISAAC_PYTHON` at its
`python.sh` launcher.

Tests requiring rendering, RTX sensors, PhysX stepping, or a running ROS bridge
are integration tests. Record the simulator version, command, and concise result
in the commit or handoff notes, while keeping generated logs out of Git.

## Runtime inspection

Once the simulator and ROS graph are running, these checks provide acceptance
evidence for timing, sensor publication, and TF ownership:

```bash
ros2 topic hz /clock
ros2 topic hz /lidar/points_raw
ros2 topic hz /scan
ros2 topic info --verbose /odom
ros2 topic info --verbose /tf
ros2 run tf2_tools view_frames
```

Treat generated TF diagrams as temporary evidence unless a deliberately curated
small fixture is needed by a test.

## Commit and data discipline

Follow `CONTRIBUTING.md`: keep code, its configuration, tests, and necessary
documentation in one coherent commit. Do not commit the external warehouse,
rosbags, generated experiment batches, or simulator logs. See `data/README.md`
before versioning fixtures, maps, or pose graphs.
