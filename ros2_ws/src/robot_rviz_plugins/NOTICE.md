# Third-party notice

`include/robot_rviz_plugins/nav2_panel.hpp` and
`src/nav2_panel.cpp` are derived from Navigation2's `nav2_rviz_plugins`
version 1.3.12 (`Nav2Panel`), distributed under the Apache License 2.0.

- Upstream project: <https://github.com/ros-navigation/navigation2>
- Upstream package: `nav2_rviz_plugins`
- Debian/ROS package inspected for this derivative:
  `ros-jazzy-nav2-rviz-plugins 1.3.12-1noble.20260615.171114`
- Original source SHA-256:
  - `src/nav2_panel.cpp`:
    `766f5fa37c1887739d8289f8534c7c27472a0662e10882662172a158108a11ba`
  - `include/nav2_rviz_plugins/nav2_panel.hpp`:
    `b58b17299a97cfeb41ae2ce3cf7b299153e38c765a445ec7dc6619df8d601bee`
- Local changes: the plugin class and namespace are isolated under
  `robot_rviz_plugins`; the startup thread supports interruption and catches
  ROS shutdown exceptions; every QtConcurrent task is owned and awaited; and
  timer callbacks stop when the ROS context is shutting down.

The complete Apache License 2.0 is installed as `LICENSE` in this package.
