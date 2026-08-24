# Vendored origin and pin

- Upstream: `https://github.com/Ericsii/FAST_LIO_ROS2.git`
- Upstream commit: `2fffc570a25d0df172720bac034fbdb6a13d2162`
- Upstream ikd-Tree submodule commit: `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`
- Upstream license: GNU General Public License, version 2 (`LICENSE` is copied verbatim)

The algorithm, IKFoM, and ikd-Tree sources were vendored from those exact
audited revisions. Large upstream papers, images, logs, example PCD data,
RViz files, and unused message definitions are not vendored.

Project-local Jazzy changes are intentionally narrow:

- build only the standard `sensor_msgs/msg/PointCloud2` input path;
- remove the unused `pcl_ros`, Python, visualization, and Livox SDK/message dependencies;
- parameterize shadow topics and frames and default all non-odometry outputs off;
- make TF publication opt-in and avoid creating a broadcaster while disabled;
- populate pose/covariance before odometry publication and mark unavailable twist with large covariance;
- add the Ouster PointCloud2 shadow configuration, launch, and direct tests.

No FAST-LIO2 output is fused or connected to navigation/control by this package.
