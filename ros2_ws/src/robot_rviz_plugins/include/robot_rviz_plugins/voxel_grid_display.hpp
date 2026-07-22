#ifndef ROBOT_RVIZ_PLUGINS__VOXEL_GRID_DISPLAY_HPP_
#define ROBOT_RVIZ_PLUGINS__VOXEL_GRID_DISPLAY_HPP_

#include <memory>

#include "nav2_msgs/msg/voxel_grid.hpp"
#include "rviz_common/message_filter_display.hpp"

namespace rviz_default_plugins
{
class PointCloudCommon;
}  // namespace rviz_default_plugins

namespace robot_rviz_plugins
{

/// Render the marked cells of a Nav2 voxel grid with RViz's point-cloud renderer.
class VoxelGridDisplay
  : public rviz_common::MessageFilterDisplay<nav2_msgs::msg::VoxelGrid>
{
  Q_OBJECT

public:
  VoxelGridDisplay();
  ~VoxelGridDisplay() override;

  void reset() override;
  void update(float wall_dt, float ros_dt) override;
  void processMessage(nav2_msgs::msg::VoxelGrid::ConstSharedPtr message) override;

protected:
  void onInitialize() override;
  void onDisable() override;

private:
  std::unique_ptr<rviz_default_plugins::PointCloudCommon> point_cloud_common_;
};

}  // namespace robot_rviz_plugins

#endif  // ROBOT_RVIZ_PLUGINS__VOXEL_GRID_DISPLAY_HPP_
