# Upstream provenance and local modifications

The solver implementation in this package is derived from Slam Toolbox 2.8.5:

- Repository: `https://github.com/SteveMacenski/slam_toolbox`
- Tag: `2.8.5`
- Commit: `ec8f7635dea317b531c419f798f87d90a336f32e`
- Upstream license: GNU Lesser General Public License 2.1

Vendored upstream files and SHA-256 digests before modification:

- `solvers/ceres_solver.cpp`: `e55f6442834e9dc450b049570f34399d0654f86df956597af6eab700fcb7c118`
- `solvers/ceres_solver.hpp`: `b99c5d7f012827c396d6ac5377eddf4c3c56b60dc91c624d012682aa6c0adc6a`
- `solvers/ceres_utils.h`: `ed56f5d2a9843b3029ed85fa46db44de024e3c2a16036774a3c4dc73d0fde08a`

Local changes are intentionally narrow:

1. Rename the namespace/class and library to avoid colliding with the system plugin.
2. Move headers under the `robot_slam_solver` include namespace.
3. Replace upstream's hard-coded `options_.num_threads = 50` with the declared
   `ceres_num_threads` parameter.
4. Reject values outside `1..std::thread::hardware_concurrency()` and log the
   effective value during configuration.
5. Package the code as a separate workspace plugin; `/opt/ros` is never modified.

The files in this package remain under LGPL-2.1-only. This notice is engineering
provenance information and is not legal advice.
