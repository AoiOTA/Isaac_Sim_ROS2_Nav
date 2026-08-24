#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>

#include "pointcloud_local_odometry/ndt_odometry.hpp"

namespace
{

using pointcloud_local_odometry::NdtConfig;
using pointcloud_local_odometry::ScanToScanOdometry;

ScanToScanOdometry::Cloud::Ptr asymmetricCloud()
{
  auto cloud = std::make_shared<ScanToScanOdometry::Cloud>();
  for (int ix = 0; ix < 5; ++ix) {
    for (int iy = 0; iy < 4; ++iy) {
      for (int iz = 0; iz < 3; ++iz) {
        for (int sx = -1; sx <= 1; ++sx) {
          for (int sy : {-1, 1}) {
            for (int sz : {-1, 1}) {
              ScanToScanOdometry::Point point;
              const double phase = 3.0 * ix + 5.0 * iy + 7.0 * iz + sx - sy + sz;
              point.x = static_cast<float>(
                0.21 + 0.61 * ix + 0.042 * sx + 0.004 * std::sin(phase));
              point.y = static_cast<float>(
                0.18 + 0.73 * iy + 0.047 * sy + 0.005 * std::cos(phase));
              point.z = static_cast<float>(
                0.23 + 0.83 * iz + 0.053 * sz + 0.003 * std::sin(2.0 * phase));
              point.intensity = static_cast<float>(1 + 11 * ix + 7 * iy + 3 * iz);
              cloud->push_back(point);
            }
          }
        }
      }
    }
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1U;
  cloud->is_dense = true;
  return cloud;
}

ScanToScanOdometry::Cloud::Ptr sparseCloud()
{
  auto cloud = std::make_shared<ScanToScanOdometry::Cloud>();
  for (int index = 0; index < 120; ++index) {
    ScanToScanOdometry::Point point;
    point.x = static_cast<float>(0.7 * index);
    point.y = static_cast<float>(0.9 * (index % 11));
    point.z = static_cast<float>(1.1 * (index % 7));
    point.intensity = static_cast<float>(index);
    cloud->push_back(point);
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

NdtConfig testConfig()
{
  NdtConfig config;
  config.voxel_leaf_size = 0.02;
  config.min_points = 100U;
  config.resolution = 0.4;
  config.step_size = 0.15;
  config.max_iterations = 80;
  config.transformation_epsilon = 1.0e-6;
  config.max_fitness_score = 0.02;
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

TEST(NdtOdometry, IdentityScanInitializesThenTracksIdentity)
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
    second.odom_base, Eigen::Isometry3d::Identity(), 0.003, 0.001);
}

TEST(NdtOdometry, RecoversKnownXyzAndRpyWithCorrectSourceTargetDirection)
{
  ScanToScanOdometry odometry(testConfig());
  const auto reference = asymmetricCloud();
  const Eigen::Isometry3d expected = motion(0.10, -0.06, 0.04, 0.025, -0.03, 0.07);
  ASSERT_TRUE(odometry.process(reference, Eigen::Isometry3d::Identity()).accepted);

  const auto result = odometry.process(
    scanAtPose(reference, expected), Eigen::Isometry3d::Identity());
  ASSERT_TRUE(result.accepted) << result.reason << " fitness=" << result.fitness;
  expectTransformNear(result.relative_lidar, expected, 0.025, 0.025);
  expectTransformNear(result.odom_base, expected, 0.025, 0.025);
}

TEST(NdtOdometry, AccumulatesTwoSuccessfulRelativeTransforms)
{
  ScanToScanOdometry odometry(testConfig());
  const auto reference = asymmetricCloud();
  const Eigen::Isometry3d first_pose = motion(0.10, -0.06, 0.04, 0.025, -0.03, 0.07);
  const Eigen::Isometry3d second_relative = motion(0.10, -0.06, 0.04, 0.025, -0.03, 0.07);
  const Eigen::Isometry3d second_pose = first_pose * second_relative;

  ASSERT_TRUE(odometry.process(reference, Eigen::Isometry3d::Identity()).accepted);
  const auto first = odometry.process(
    scanAtPose(reference, first_pose), Eigen::Isometry3d::Identity());
  ASSERT_TRUE(first.accepted) << first.reason;
  const auto second = odometry.process(
    scanAtPose(reference, second_pose), Eigen::Isometry3d::Identity());
  ASSERT_TRUE(second.accepted) << second.reason;
  expectTransformNear(second.odom_base, second_pose, 0.05, 0.04);
}

TEST(NdtOdometry, ConjugatesLidarMotionIntoBaseFrame)
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

TEST(NdtOdometry, RejectsInsufficientAndNonfiniteClouds)
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

TEST(NdtOdometry, RejectsConvergedRegistrationAboveFitnessLimit)
{
  NdtConfig config = testConfig();
  config.max_fitness_score = 1.0e-12;
  ScanToScanOdometry odometry(config);
  const auto reference = asymmetricCloud();
  ASSERT_TRUE(odometry.process(reference, Eigen::Isometry3d::Identity()).accepted);

  auto deformed = scanAtPose(reference, motion(0.06, -0.02, 0.01, 0.01, -0.01, 0.04));
  for (std::size_t index = 0; index < deformed->size(); index += 3U) {
    deformed->points[index].x += 0.004F;
    deformed->points[index].z -= 0.003F;
  }
  const auto result = odometry.process(deformed, Eigen::Isometry3d::Identity());
  ASSERT_TRUE(result.converged) << result.reason;
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "fitness_rejected");
  EXPECT_GT(result.fitness, config.max_fitness_score);
}

TEST(NdtOdometry, RejectsNonconvergedRegistration)
{
  NdtConfig config = testConfig();
  ScanToScanOdometry odometry(config);
  const auto reference = sparseCloud();
  ASSERT_TRUE(odometry.process(reference, Eigen::Isometry3d::Identity()).accepted);

  const auto result = odometry.process(reference, Eigen::Isometry3d::Identity());
  EXPECT_FALSE(result.accepted);
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.reason, "ndt_not_converged");
}

TEST(NdtOdometry, RejectedScanDoesNotReplacePreviousSuccessfulScan)
{
  ScanToScanOdometry odometry(testConfig());
  const auto reference = asymmetricCloud();
  ASSERT_TRUE(odometry.process(reference, Eigen::Isometry3d::Identity()).accepted);

  auto rejected = asymmetricCloud();
  rejected->points[10].x = std::numeric_limits<float>::infinity();
  const auto bad_result = odometry.process(rejected, Eigen::Isometry3d::Identity());
  ASSERT_FALSE(bad_result.accepted);
  ASSERT_EQ(bad_result.reason, "nonfinite_points");

  const Eigen::Isometry3d expected = motion(0.10, -0.06, 0.04, 0.025, -0.03, 0.07);
  const auto recovered = odometry.process(
    scanAtPose(reference, expected), Eigen::Isometry3d::Identity());
  ASSERT_TRUE(recovered.accepted) << recovered.reason;
  expectTransformNear(recovered.odom_base, expected, 0.025, 0.025);
}

}  // namespace
