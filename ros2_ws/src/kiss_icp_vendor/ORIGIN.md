# Vendored source origin

Retrieved on 2026-08-25 from the official upstream repositories:

| Source | Official URL | Tag | Full commit |
|---|---|---|---|
| KISS-ICP | https://github.com/PRBonn/kiss-icp.git | `v1.3.0` | `b16835283aee62f7d5e2bdf6c1c3bb2930de74ff` |
| Sophus | https://github.com/strasdat/Sophus.git | `1.24.6` | `d0b7315a0d90fc6143defa54596a3a95d9fa10ec` |
| robin-map | https://github.com/Tessil/robin-map.git | `v1.4.0` | `4ec1bf19c6a96125ea22062f38c2cf5b958e448e` |

Sophus and robin-map are the exact releases declared by KISS-ICP v1.3.0 in
`cpp/kiss_icp/3rdparty`. The complete KISS-ICP C++ core and ROS 2 wrapper used
by this workspace are retained under `cpp/` and `ros/`; the two header-only
dependencies are retained under `third_party/`. Each upstream license is kept
with its source, and no vendored directory contains Git metadata.

## Minimal offline patch

- The upstream ROS CMake file forces Sophus and robin-map to the vendored copies.
- The two upstream dependency CMake declarations replace their network archive
  URLs with local `SOURCE_DIR` paths.
- The upstream ROS CMake file already selects the adjacent local KISS-ICP core,
  so its network fallback is unchanged but unreachable in this layout.
- No algorithm, ROS node implementation, parameter default, or message
  interface is changed.

The package is built with `FETCHCONTENT_FULLY_DISCONNECTED=ON`.
