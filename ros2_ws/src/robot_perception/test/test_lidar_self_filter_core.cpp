// Copyright 2026 AoiOTA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"
#include "robot_perception/lidar_self_filter_core.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace
{

sensor_msgs::msg::PointCloud2 make_cloud(
  const std::vector<std::array<float, 4>> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "base_link";
  cloud.header.stamp.sec = 42;
  cloud.header.stamp.nanosec = 123456789U;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity(cloud, "intensity");
  for (const auto & point : points) {
    *x = point[0];
    *y = point[1];
    *z = point[2];
    *intensity = point[3];
    ++x;
    ++y;
    ++z;
    ++intensity;
  }
  return cloud;
}

std::vector<std::array<float, 4>> read_cloud(
  const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::vector<std::array<float, 4>> result;
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2ConstIterator<float> intensity(cloud, "intensity");
  for (std::size_t index = 0; index < cloud.width * cloud.height; ++index) {
    result.push_back({*x, *y, *z, *intensity});
    ++x;
    ++y;
    ++z;
    ++intensity;
  }
  return result;
}

const robot_perception::AxisAlignedBounds kBounds{
  {-0.235, -0.215, -0.05},
  {0.260, 0.215, 0.55},
};

TEST(LidarSelfFilterCore, RemovesInteriorAndInclusiveBoundaryPoints)
{
  const auto cloud = make_cloud({
      {0.0F, 0.0F, 0.20F, 1.0F},
      {-0.235F, -0.215F, -0.05F, 2.0F},
      {0.260F, 0.215F, 0.55F, 3.0F},
      {0.261F, 0.0F, 0.20F, 4.0F},
      {0.0F, 0.216F, 0.20F, 5.0F},
  });

  const auto filtered =
    robot_perception::filter_axis_aligned_self_points(cloud, kBounds);
  const auto points = read_cloud(filtered);

  ASSERT_EQ(points.size(), 2U);
  EXPECT_FLOAT_EQ(points[0][0], 0.261F);
  EXPECT_FLOAT_EQ(points[0][3], 4.0F);
  EXPECT_FLOAT_EQ(points[1][1], 0.216F);
  EXPECT_FLOAT_EQ(points[1][3], 5.0F);
  EXPECT_EQ(filtered.header.frame_id, cloud.header.frame_id);
  EXPECT_EQ(filtered.header.stamp, cloud.header.stamp);
  EXPECT_EQ(filtered.fields, cloud.fields);
  EXPECT_EQ(filtered.point_step, cloud.point_step);
  EXPECT_EQ(filtered.height, 1U);
  EXPECT_EQ(filtered.row_step, filtered.width * filtered.point_step);
}

TEST(LidarSelfFilterCore, PreservesNonFinitePointsForDownstreamHandling)
{
  const auto cloud = make_cloud({
      {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.2F, 7.0F},
      {0.4F, 0.0F, 0.2F, 8.0F},
  });

  const auto filtered =
    robot_perception::filter_axis_aligned_self_points(cloud, kBounds);
  const auto points = read_cloud(filtered);

  ASSERT_EQ(points.size(), 2U);
  EXPECT_TRUE(std::isnan(points[0][0]));
  EXPECT_FLOAT_EQ(points[0][3], 7.0F);
  EXPECT_FLOAT_EQ(points[1][0], 0.4F);
}

TEST(LidarSelfFilterCore, RejectsInvalidBounds)
{
  const robot_perception::AxisAlignedBounds invalid{
    {0.0, -0.2, -0.1},
    {0.0, 0.2, 0.5},
  };
  EXPECT_THROW(robot_perception::validate_bounds(invalid), std::invalid_argument);
}

}  // namespace
