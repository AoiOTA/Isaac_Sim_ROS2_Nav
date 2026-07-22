#include "robot_rviz_plugins/voxel_grid_display.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace robot_rviz_plugins
{

VoxelGridDisplay::VoxelGridDisplay() = default;

VoxelGridDisplay::~VoxelGridDisplay() = default;

void VoxelGridDisplay::onInitialize()
{
  MFDClass::onInitialize();
  point_cloud_common_ =
    std::make_unique<rviz_default_plugins::PointCloudCommon>(this);
  point_cloud_common_->initialize(context_, scene_node_);
}

void VoxelGridDisplay::reset()
{
  MFDClass::reset();
  if (point_cloud_common_) {
    point_cloud_common_->reset();
  }
}

void VoxelGridDisplay::update(float wall_dt, float ros_dt)
{
  if (point_cloud_common_) {
    point_cloud_common_->update(wall_dt, ros_dt);
  }
}

void VoxelGridDisplay::onDisable()
{
  if (point_cloud_common_) {
    point_cloud_common_->onDisable();
  }
  MFDClass::onDisable();
}

void VoxelGridDisplay::processMessage(
  nav2_msgs::msg::VoxelGrid::ConstSharedPtr message)
{
  const auto size_x = static_cast<std::size_t>(message->size_x);
  const auto size_y = static_cast<std::size_t>(message->size_y);
  const auto size_z = static_cast<std::size_t>(message->size_z);

  if (size_z > 16U ||
    (size_x != 0U && size_y > std::numeric_limits<std::size_t>::max() / size_x) ||
    message->data.size() < size_x * size_y)
  {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Voxel Grid",
      "Invalid voxel-grid dimensions or data length");
    return;
  }

  if (!std::isfinite(message->resolutions.x) || message->resolutions.x <= 0.0 ||
    !std::isfinite(message->resolutions.y) || message->resolutions.y <= 0.0 ||
    !std::isfinite(message->resolutions.z) || message->resolutions.z <= 0.0)
  {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Voxel Grid",
      "Voxel-grid resolutions must be finite and positive");
    return;
  }

  const uint32_t valid_z_mask =
    size_z == 16U ? 0xffffU : ((1U << size_z) - 1U);
  std::size_t marked_count = 0U;
  for (std::size_t index = 0U; index < size_x * size_y; ++index) {
    marked_count += static_cast<std::size_t>(
      __builtin_popcount((message->data[index] >> 16U) & valid_z_mask));
  }

  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud->header = message->header;
  sensor_msgs::PointCloud2Modifier modifier(*cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(marked_count);

  sensor_msgs::PointCloud2Iterator<float> x_iter(*cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y_iter(*cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z_iter(*cloud, "z");

  for (std::size_t y = 0U; y < size_y; ++y) {
    for (std::size_t x = 0U; x < size_x; ++x) {
      const uint32_t marked = message->data[y * size_x + x] >> 16U;
      for (std::size_t z = 0U; z < size_z; ++z) {
        if ((marked & (1U << z)) == 0U) {
          continue;
        }
        *x_iter = static_cast<float>(
          message->origin.x + (static_cast<double>(x) + 0.5) * message->resolutions.x);
        *y_iter = static_cast<float>(
          message->origin.y + (static_cast<double>(y) + 0.5) * message->resolutions.y);
        *z_iter = static_cast<float>(
          message->origin.z + (static_cast<double>(z) + 0.5) * message->resolutions.z);
        ++x_iter;
        ++y_iter;
        ++z_iter;
      }
    }
  }

  point_cloud_common_->addMessage(cloud);
  setStatus(
    rviz_common::properties::StatusProperty::Ok, "Voxel Grid",
    QString("%1 marked voxels").arg(marked_count));
}

}  // namespace robot_rviz_plugins

PLUGINLIB_EXPORT_CLASS(robot_rviz_plugins::VoxelGridDisplay, rviz_common::Display)
