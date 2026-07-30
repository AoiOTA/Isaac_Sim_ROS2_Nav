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

#include "robot_perception/lidar_self_filter_core.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>

#include "sensor_msgs/msg/point_field.hpp"

namespace robot_perception
{
namespace
{

constexpr double kCoordinateTolerance = 1.0e-6;

struct CoordinateField
{
  std::size_t offset;
  std::uint8_t datatype;
};

CoordinateField find_coordinate_field(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & name)
{
  for (const auto & field : cloud.fields) {
    if (field.name != name) {
      continue;
    }
    if (field.count != 1U) {
      throw std::invalid_argument("PointCloud2 field " + name + " must have count=1");
    }
    if (
      field.datatype != sensor_msgs::msg::PointField::FLOAT32 &&
      field.datatype != sensor_msgs::msg::PointField::FLOAT64)
    {
      throw std::invalid_argument("PointCloud2 field " + name + " must be FLOAT32 or FLOAT64");
    }
    const std::size_t width =
      field.datatype == sensor_msgs::msg::PointField::FLOAT32 ? sizeof(float) : sizeof(double);
    if (static_cast<std::size_t>(field.offset) + width > cloud.point_step) {
      throw std::invalid_argument("PointCloud2 field " + name + " exceeds point_step");
    }
    return {field.offset, field.datatype};
  }
  throw std::invalid_argument("PointCloud2 is missing required field " + name);
}

double read_coordinate(const std::uint8_t * point, const CoordinateField & field)
{
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
    float value = std::numeric_limits<float>::quiet_NaN();
    std::memcpy(&value, point + field.offset, sizeof(value));
    return static_cast<double>(value);
  }
  double value = std::numeric_limits<double>::quiet_NaN();
  std::memcpy(&value, point + field.offset, sizeof(value));
  return value;
}

bool inside_bounds(
  const double x,
  const double y,
  const double z,
  const AxisAlignedBounds & bounds)
{
  return
    std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
    x >= bounds.minimum[0] - kCoordinateTolerance &&
    x <= bounds.maximum[0] + kCoordinateTolerance &&
    y >= bounds.minimum[1] - kCoordinateTolerance &&
    y <= bounds.maximum[1] + kCoordinateTolerance &&
    z >= bounds.minimum[2] - kCoordinateTolerance &&
    z <= bounds.maximum[2] + kCoordinateTolerance;
}

}  // namespace

void validate_bounds(const AxisAlignedBounds & bounds)
{
  for (std::size_t axis = 0; axis < 3U; ++axis) {
    if (!std::isfinite(bounds.minimum[axis]) || !std::isfinite(bounds.maximum[axis])) {
      throw std::invalid_argument("self-filter bounds must be finite");
    }
    if (bounds.minimum[axis] >= bounds.maximum[axis]) {
      throw std::invalid_argument("self-filter minimum must be less than maximum");
    }
  }
}

sensor_msgs::msg::PointCloud2 filter_axis_aligned_self_points(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const AxisAlignedBounds & bounds)
{
  validate_bounds(bounds);
  if (cloud.is_bigendian) {
    throw std::invalid_argument("big-endian PointCloud2 is not supported");
  }
  if (cloud.point_step == 0U) {
    throw std::invalid_argument("PointCloud2 point_step must be positive");
  }
  if (cloud.row_step < cloud.width * cloud.point_step) {
    throw std::invalid_argument("PointCloud2 row_step is smaller than its populated row");
  }
  const std::size_t required_size =
    cloud.height == 0U ? 0U :
    (static_cast<std::size_t>(cloud.height) - 1U) * cloud.row_step +
    static_cast<std::size_t>(cloud.width) * cloud.point_step;
  if (cloud.data.size() < required_size) {
    throw std::invalid_argument("PointCloud2 data is truncated");
  }

  const auto x_field = find_coordinate_field(cloud, "x");
  const auto y_field = find_coordinate_field(cloud, "y");
  const auto z_field = find_coordinate_field(cloud, "z");

  sensor_msgs::msg::PointCloud2 filtered = cloud;
  filtered.height = 1U;
  filtered.width = 0U;
  filtered.row_step = 0U;
  filtered.data.clear();
  filtered.data.reserve(
    static_cast<std::size_t>(cloud.width) * cloud.height * cloud.point_step);

  for (std::uint32_t row = 0; row < cloud.height; ++row) {
    for (std::uint32_t column = 0; column < cloud.width; ++column) {
      const std::size_t offset =
        static_cast<std::size_t>(row) * cloud.row_step +
        static_cast<std::size_t>(column) * cloud.point_step;
      const std::uint8_t * point = cloud.data.data() + offset;
      const double x = read_coordinate(point, x_field);
      const double y = read_coordinate(point, y_field);
      const double z = read_coordinate(point, z_field);
      if (inside_bounds(x, y, z, bounds)) {
        continue;
      }
      filtered.data.insert(filtered.data.end(), point, point + cloud.point_step);
      ++filtered.width;
    }
  }
  filtered.row_step = filtered.width * filtered.point_step;
  return filtered;
}

}  // namespace robot_perception
