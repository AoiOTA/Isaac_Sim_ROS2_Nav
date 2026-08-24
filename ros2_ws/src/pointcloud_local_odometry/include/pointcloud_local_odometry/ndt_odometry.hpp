#ifndef POINTCLOUD_LOCAL_ODOMETRY__NDT_ODOMETRY_HPP_
#define POINTCLOUD_LOCAL_ODOMETRY__NDT_ODOMETRY_HPP_

#include <cstddef>
#include <limits>
#include <string>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pointcloud_local_odometry
{

struct NdtConfig
{
  double voxel_leaf_size{0.15};
  std::size_t min_points{100U};
  double resolution{0.5};
  double step_size{0.1};
  int max_iterations{40};
  double transformation_epsilon{1.0e-3};
  double max_fitness_score{0.25};
};

struct OdometryResult
{
  bool accepted{false};
  bool initializing{false};
  bool converged{false};
  std::size_t raw_points{0U};
  std::size_t filtered_points{0U};
  double fitness{std::numeric_limits<double>::quiet_NaN()};
  std::string reason;
  Eigen::Isometry3d relative_lidar{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d relative_base{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d odom_base{Eigen::Isometry3d::Identity()};
};

class ScanToScanOdometry
{
public:
  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<Point>;

  explicit ScanToScanOdometry(NdtConfig config);

  OdometryResult process(
    const Cloud::ConstPtr & raw_cloud,
    const Eigen::Isometry3d & base_to_lidar);

  static Eigen::Isometry3d conjugateToBase(
    const Eigen::Isometry3d & base_to_lidar,
    const Eigen::Isometry3d & previous_lidar_to_current_lidar);

private:
  Cloud::Ptr filterCloud(
    const Cloud::ConstPtr & raw_cloud,
    OdometryResult & result) const;

  NdtConfig config_;
  Cloud::Ptr previous_successful_scan_;
  Eigen::Isometry3d odom_base_{Eigen::Isometry3d::Identity()};
};

}  // namespace pointcloud_local_odometry

#endif  // POINTCLOUD_LOCAL_ODOMETRY__NDT_ODOMETRY_HPP_
