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

#ifndef ROBOT_PERCEPTION__LIDAR_SELF_FILTER_CORE_HPP_
#define ROBOT_PERCEPTION__LIDAR_SELF_FILTER_CORE_HPP_

#include <array>

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace robot_perception
{

struct AxisAlignedBounds
{
  std::array<double, 3> minimum;
  std::array<double, 3> maximum;
};

void validate_bounds(const AxisAlignedBounds & bounds);

sensor_msgs::msg::PointCloud2 filter_axis_aligned_self_points(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const AxisAlignedBounds & bounds);

}  // namespace robot_perception

#endif  // ROBOT_PERCEPTION__LIDAR_SELF_FILTER_CORE_HPP_
