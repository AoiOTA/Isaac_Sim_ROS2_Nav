#include "pointcloud_local_odometry/gicp_odometry.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>

namespace pointcloud_local_odometry
{

ScanToScanOdometry::ScanToScanOdometry(GicpConfig config)
: config_(std::move(config))
{
  if (!std::isfinite(config_.voxel_leaf_size) || config_.voxel_leaf_size <= 0.0) {
    throw std::invalid_argument("voxel_leaf_size must be finite and positive");
  }
  if (config_.min_points < 3U) {
    throw std::invalid_argument("min_points must be at least three");
  }
  if (!std::isfinite(config_.max_correspondence_distance) ||
    config_.max_correspondence_distance <= 0.0)
  {
    throw std::invalid_argument("max_correspondence_distance must be finite and positive");
  }
  if (config_.max_iterations <= 0) {
    throw std::invalid_argument("max_iterations must be positive");
  }
  if (!std::isfinite(config_.transformation_epsilon) ||
    config_.transformation_epsilon <= 0.0 ||
    !std::isfinite(config_.euclidean_fitness_epsilon) ||
    config_.euclidean_fitness_epsilon <= 0.0 ||
    !std::isfinite(config_.max_fitness_score) || config_.max_fitness_score < 0.0)
  {
    throw std::invalid_argument("GICP epsilon and fitness limits are invalid");
  }
}

ScanToScanOdometry::Cloud::Ptr ScanToScanOdometry::filterCloud(
  const Cloud::ConstPtr & raw_cloud,
  OdometryResult & result) const
{
  result.raw_points = raw_cloud ? raw_cloud->size() : 0U;
  if (!raw_cloud || raw_cloud->empty()) {
    result.reason = "insufficient_points";
    return nullptr;
  }

  auto finite_cloud = std::make_shared<Cloud>();
  finite_cloud->reserve(raw_cloud->size());
  for (const auto & point : raw_cloud->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      result.reason = "nonfinite_points";
      return nullptr;
    }
    finite_cloud->push_back(point);
  }
  finite_cloud->width = static_cast<std::uint32_t>(finite_cloud->size());
  finite_cloud->height = 1U;
  finite_cloud->is_dense = true;

  auto filtered = std::make_shared<Cloud>();
  pcl::VoxelGrid<Point> voxel_grid;
  const auto leaf = static_cast<float>(config_.voxel_leaf_size);
  voxel_grid.setLeafSize(leaf, leaf, leaf);
  voxel_grid.setInputCloud(finite_cloud);
  voxel_grid.filter(*filtered);
  result.filtered_points = filtered->size();
  if (filtered->size() < config_.min_points) {
    result.reason = "insufficient_points";
    return nullptr;
  }
  return filtered;
}

Eigen::Isometry3d ScanToScanOdometry::conjugateToBase(
  const Eigen::Isometry3d & base_to_lidar,
  const Eigen::Isometry3d & previous_lidar_to_current_lidar)
{
  return base_to_lidar * previous_lidar_to_current_lidar * base_to_lidar.inverse();
}

OdometryResult ScanToScanOdometry::process(
  const Cloud::ConstPtr & raw_cloud,
  const Eigen::Isometry3d & base_to_lidar)
{
  OdometryResult result;
  result.odom_base = odom_base_;
  if (!base_to_lidar.matrix().allFinite()) {
    result.reason = "nonfinite_transform";
    return result;
  }

  const auto current_scan = filterCloud(raw_cloud, result);
  if (!current_scan) {
    return result;
  }

  if (!previous_successful_scan_) {
    previous_successful_scan_ = current_scan;
    result.accepted = true;
    result.initializing = true;
    result.reason = "first_valid_scan";
    result.odom_base = odom_base_;
    return result;
  }

  pcl::GeneralizedIterativeClosestPoint<Point, Point> gicp;
  gicp.setMaxCorrespondenceDistance(config_.max_correspondence_distance);
  gicp.setMaximumIterations(config_.max_iterations);
  gicp.setTransformationEpsilon(config_.transformation_epsilon);
  gicp.setEuclideanFitnessEpsilon(config_.euclidean_fitness_epsilon);
  gicp.setInputSource(current_scan);
  gicp.setInputTarget(previous_successful_scan_);

  Cloud aligned;
  try {
    gicp.align(aligned);
  } catch (const std::exception &) {
    result.reason = "gicp_exception";
    return result;
  }

  result.converged = gicp.hasConverged();
  if (!result.converged) {
    result.reason = "gicp_not_converged";
    return result;
  }

  result.fitness = gicp.getFitnessScore();
  const Eigen::Matrix4d transform = gicp.getFinalTransformation().cast<double>();
  if (!std::isfinite(result.fitness) || !transform.allFinite()) {
    result.reason = "nonfinite_result";
    return result;
  }
  if (result.fitness > config_.max_fitness_score) {
    result.reason = "fitness_rejected";
    return result;
  }

  result.relative_lidar.matrix() = transform;
  result.relative_base = conjugateToBase(base_to_lidar, result.relative_lidar);
  const Eigen::Isometry3d candidate = odom_base_ * result.relative_base;
  if (!candidate.matrix().allFinite()) {
    result.reason = "nonfinite_result";
    return result;
  }

  odom_base_ = candidate;
  previous_successful_scan_ = current_scan;
  result.accepted = true;
  result.reason = "tracking";
  result.odom_base = odom_base_;
  return result;
}

}  // namespace pointcloud_local_odometry
