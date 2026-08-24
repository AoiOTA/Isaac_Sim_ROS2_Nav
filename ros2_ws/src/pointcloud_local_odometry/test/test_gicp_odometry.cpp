#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>

#include "pointcloud_local_odometry/gicp_odometry.hpp"

namespace
{

using pointcloud_local_odometry::GicpConfig;
using pointcloud_local_odometry::ScanToScanOdometry;

ScanToScanOdometry::Cloud::Ptr asymmetricCloud()
{
  auto cloud = std::make_shared<ScanToScanOdometry::Cloud>();
  for (int ix = 0; ix < 8; ++ix) {
    for (int iy = 0; iy < 7; ++iy) {
      for (int iz = 0; iz < 5; ++iz) {
        ScanToScanOdometry::Point point;
        point.x = static_cast<float>(0.17 * ix + 0.013 * std::sin(iy + 2.0 * iz));
        point.y = static_cast<float>(0.19 * iy + 0.021 * std::cos(2.0 * ix + iz));
        point.z = static_cast<float>(0.23 * iz + 0.017 * std::sin(ix + 3.0 * iy));
        cloud->push_back(point);
      }
    }
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1U;
  cloud->is_dense = true;
  return cloud;
}

Eigen::Isometry3d motion(
  double x, double y, double z, double roll, double pitch, double yaw)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = (
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  transform.translation() = Eigen::Vector3d(x, y, z);
  return transform;
}

ScanToScanOdometry::Cloud::Ptr scanAtPose(
  const ScanToScanOdometry::Cloud::ConstPtr & reference,
  const Eigen::Isometry3d & reference_to_scan)
{
  auto scan = std::make_shared<ScanToScanOdometry::Cloud>();
  pcl::transformPointCloud(*reference, *scan, reference_to_scan.inverse().matrix());
  return scan;
}

GicpConfig testConfig()
{
  GicpConfig config;
  config.voxel_leaf_size = 0.01;
  config.min_points = 100U;
  config.max_correspondence_distance = 0.8;
  config.max_iterations = 80;
  config.transformation_epsilon = 1.0e-8;
  config.euclidean_fitness_epsilon = 1.0e-8;
  config.max_fitness_score = 0.01;
  return config;
}

void expectTransformNear(
  const Eigen::Isometry3d & observed,
  const Eigen::Isometry3d & expected,
  double translation_tolerance,
  double rotation_tolerance)
{
  EXPECT_LT(
    (observed.translation() - expected.translation()).norm(),
    translation_tolerance);
  const Eigen::Matrix3d delta = observed.rotation().transpose() * expected.rotation();
  EXPECT_LT(Eigen::AngleAxisd(delta).angle(), rotation_tolerance);
}

TEST(GicpOdometry, IdentityScanInitializesThenTracksIdentity)
{
  ScanToScanOdometry odometry(testConfig());
  const auto cloud = asymmetricCloud();
  const auto first = odometry.process(cloud, Eigen::Isometry3d::Identity());
  ASSERT_TRUE(first.accepted);
  EXPECT_TRUE(first.initializing);
  EXPECT_EQ(first.reason, "first_valid_scan");

  const auto second = odometry.process(cloud, Eigen::Isometry3d::Identity());
  ASSERT_TRUE(second.accepted) << second.reason;
  EXPECT_TRUE(second.converged);
  EXPECT_EQ(second.reason, "tracking");
  expectTransformNear(
    second.odom_base, Eigen::Isometry3d::Identity(), 1.0e-4, 1.0e-4);
}

TEST(GicpOdometry, RecoversKnownXyzAndRpyWithCorrectSourceTargetDirection)
{
  ScanToScanOdometry odometry(testConfig());
  const auto reference = asymmetricCloud();
  const Eigen::Isometry3d expected = motion(0.14, -0.07, 0.05, 0.035, -0.045, 0.11);
  ASSERT_TRUE(odometry.process(reference, Eigen::Isometry3d::Identity()).accepted);

  const auto result = odometry.process(
    scanAtPose(reference, expected), Eigen::Isometry3d::Identity());
  ASSERT_TRUE(result.accepted) << result.reason << " fitness=" << result.fitness;
  expectTransformNear(result.relative_lidar, expected, 0.015, 0.015);
  expectTransformNear(result.odom_base, expected, 0.015, 0.015);
}

TEST(GicpOdometry, AccumulatesTwoSuccessfulRelativeTransforms)
{
  ScanToScanOdometry odometry(testConfig());
  const auto reference = asymmetricCloud();
  const Eigen::Isometry3d first_pose = motion(0.10, -0.04, 0.03, 0.02, -0.03, 0.08);
  const Eigen::Isometry3d second_relative = motion(0.08, 0.03, -0.01, -0.01, 0.025, -0.06);
  const Eigen::Isometry3d second_pose = first_pose * second_relative;

  ASSERT_TRUE(odometry.process(reference, Eigen::Isometry3d::Identity()).accepted);
  const auto first = odometry.process(
    scanAtPose(reference, first_pose), Eigen::Isometry3d::Identity());
  ASSERT_TRUE(first.accepted) << first.reason;
  const auto second = odometry.process(
    scanAtPose(reference, second_pose), Eigen::Isometry3d::Identity());
  ASSERT_TRUE(second.accepted) << second.reason;
  expectTransformNear(second.odom_base, second_pose, 0.025, 0.02);
}

TEST(GicpOdometry, ConjugatesLidarMotionIntoBaseFrame)
{
  const Eigen::Isometry3d base_to_lidar = motion(
    0.12, -0.03, 0.33, 0.04, -0.02, 0.09);
  const Eigen::Isometry3d lidar_motion = motion(
    0.18, 0.06, -0.02, -0.03, 0.05, 0.14);
  const Eigen::Isometry3d expected =
    base_to_lidar * lidar_motion * base_to_lidar.inverse();
  const auto observed = ScanToScanOdometry::conjugateToBase(
    base_to_lidar, lidar_motion);
  expectTransformNear(observed, expected, 1.0e-12, 1.0e-12);
}

TEST(GicpOdometry, RejectsInsufficientAndNonfiniteClouds)
{
  ScanToScanOdometry odometry(testConfig());
  auto insufficient = std::make_shared<ScanToScanOdometry::Cloud>();
  insufficient->resize(3U);
  const auto too_small = odometry.process(insufficient, Eigen::Isometry3d::Identity());
  EXPECT_FALSE(too_small.accepted);
  EXPECT_EQ(too_small.reason, "insufficient_points");

  auto nonfinite = asymmetricCloud();
  nonfinite->points[5].z = std::numeric_limits<float>::quiet_NaN();
  const auto invalid = odometry.process(nonfinite, Eigen::Isometry3d::Identity());
  EXPECT_FALSE(invalid.accepted);
  EXPECT_EQ(invalid.reason, "nonfinite_points");
}

TEST(GicpOdometry, RejectsConvergedRegistrationAboveFitnessLimit)
{
  GicpConfig config = testConfig();
  config.max_fitness_score = 1.0e-12;
  ScanToScanOdometry odometry(config);
  const auto reference = asymmetricCloud();
  ASSERT_TRUE(odometry.process(reference, Eigen::Isometry3d::Identity()).accepted);

  const auto expected = motion(0.08, -0.02, 0.01, 0.01, -0.015, 0.05);
  auto deformed = scanAtPose(reference, expected);
  for (std::size_t index = 0; index < deformed->size(); index += 3U) {
    deformed->points[index].x += 0.003F;
    deformed->points[index].z -= 0.002F;
  }
  const auto result = odometry.process(deformed, Eigen::Isometry3d::Identity());
  ASSERT_TRUE(result.converged) << result.reason;
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "fitness_rejected");
  EXPECT_GT(result.fitness, config.max_fitness_score);
}

TEST(GicpOdometry, RejectedScanDoesNotReplacePreviousSuccessfulScan)
{
  ScanToScanOdometry odometry(testConfig());
  const auto reference = asymmetricCloud();
  ASSERT_TRUE(odometry.process(reference, Eigen::Isometry3d::Identity()).accepted);

  auto rejected = asymmetricCloud();
  rejected->points[10].x = std::numeric_limits<float>::infinity();
  const auto bad_result = odometry.process(rejected, Eigen::Isometry3d::Identity());
  ASSERT_FALSE(bad_result.accepted);
  ASSERT_EQ(bad_result.reason, "nonfinite_points");

  const Eigen::Isometry3d expected = motion(0.09, 0.03, -0.02, 0.01, 0.02, -0.07);
  const auto recovered = odometry.process(
    scanAtPose(reference, expected), Eigen::Isometry3d::Identity());
  ASSERT_TRUE(recovered.accepted) << recovered.reason;
  expectTransformNear(recovered.odom_base, expected, 0.015, 0.015);
}

}  // namespace
