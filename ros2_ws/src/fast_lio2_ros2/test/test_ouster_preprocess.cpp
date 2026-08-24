#include <cstdint>
#include <memory>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "preprocess.h"

TEST(OusterPreprocess, AcceptsStandardPointCloud2Schema)
{
  auto message = std::make_unique<sensor_msgs::msg::PointCloud2>();
  sensor_msgs::PointCloud2Modifier modifier(*message);
  modifier.setPointCloud2Fields(
    6,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
    "ring", 1, sensor_msgs::msg::PointField::UINT8,
    "t", 1, sensor_msgs::msg::PointField::UINT32);
  modifier.resize(3);

  sensor_msgs::PointCloud2Iterator<float> x(*message, "x");
  sensor_msgs::PointCloud2Iterator<float> y(*message, "y");
  sensor_msgs::PointCloud2Iterator<float> z(*message, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity(*message, "intensity");
  sensor_msgs::PointCloud2Iterator<std::uint8_t> ring(*message, "ring");
  sensor_msgs::PointCloud2Iterator<std::uint32_t> time(*message, "t");

  const float xs[] = {0.1F, 1.0F, 2.0F};
  const std::uint32_t times[] = {0U, 1000000U, 2000000U};
  for (std::size_t index = 0; index < 3; ++index, ++x, ++y, ++z, ++intensity, ++ring, ++time) {
    *x = xs[index];
    *y = 0.0F;
    *z = 0.0F;
    *intensity = static_cast<float>(10 + index);
    *ring = static_cast<std::uint8_t>(index);
    *time = times[index];
  }

  Preprocess preprocess;
  preprocess.feature_enabled = false;
  preprocess.lidar_type = OUST64;
  preprocess.point_filter_num = 1;
  preprocess.N_SCANS = 64;
  preprocess.SCAN_RATE = 10;
  preprocess.time_unit = NS;
  preprocess.blind = 0.3;

  auto output = std::make_shared<PointCloudXYZI>();
  preprocess.process(std::move(message), output);

  ASSERT_EQ(output->size(), 2U);
  EXPECT_FLOAT_EQ(output->points[0].x, 1.0F);
  EXPECT_FLOAT_EQ(output->points[0].intensity, 11.0F);
  EXPECT_FLOAT_EQ(output->points[0].curvature, 1.0F);
  EXPECT_FLOAT_EQ(output->points[1].curvature, 2.0F);
}
