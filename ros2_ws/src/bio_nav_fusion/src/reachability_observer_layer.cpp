#include "bio_nav_fusion/reachability_observer_layer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace bio_nav_fusion
{

ReachabilityObserverLayer::ReachabilityObserverLayer()
{
  enabled_ = true;
  current_ = true;
}

void ReachabilityObserverLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("ReachabilityObserverLayer lifecycle node expired");
  }
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".output_topic", rclcpp::ParameterValue(output_topic_));
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".output_topic", output_topic_);
  if (output_topic_.empty()) {
    throw std::runtime_error("ReachabilityObserverLayer output_topic is required");
  }
  publisher_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    output_topic_, rclcpp::QoS(1).reliable().transient_local());
  current_ = true;
}

void ReachabilityObserverLayer::activate()
{
  if (publisher_) {
    publisher_->on_activate();
  }
}

void ReachabilityObserverLayer::deactivate()
{
  if (publisher_) {
    publisher_->on_deactivate();
  }
}

void ReachabilityObserverLayer::reset()
{
  cache_initialized_ = false;
  cached_grid_.data.clear();
  current_ = true;
}

void ReachabilityObserverLayer::updateBounds(
  double, double, double, double *, double *, double *, double *)
{
  // Observer-only: never expands update bounds and never writes layer data.
}

int8_t ReachabilityObserverLayer::occupancyValue(uint8_t cost)
{
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return -1;
  }
  if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    return 100;
  }
  return static_cast<int8_t>(std::clamp(
      static_cast<int>(std::lround(100.0 * cost / nav2_costmap_2d::MAX_NON_OBSTACLE)),
      0, 100));
}

void ReachabilityObserverLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !publisher_ || !publisher_->is_activated()) {
    return;
  }
  auto node = node_.lock();
  if (!node) {
    return;
  }
  const auto width = master_grid.getSizeInCellsX();
  const auto height = master_grid.getSizeInCellsY();
  const auto resolution = static_cast<float>(master_grid.getResolution());
  const auto geometry_changed =
    cached_grid_.info.width != width || cached_grid_.info.height != height ||
    cached_grid_.info.resolution != resolution ||
    cached_grid_.info.origin.position.x != master_grid.getOriginX() ||
    cached_grid_.info.origin.position.y != master_grid.getOriginY();
  const auto stamp = node->now();
  if (!cache_initialized_ || geometry_changed) {
    cached_grid_ = nav_msgs::msg::OccupancyGrid();
    cached_grid_.info.map_load_time = stamp;
    cached_grid_.info.resolution = resolution;
    cached_grid_.info.width = width;
    cached_grid_.info.height = height;
    cached_grid_.info.origin.position.x = master_grid.getOriginX();
    cached_grid_.info.origin.position.y = master_grid.getOriginY();
    cached_grid_.info.origin.orientation.w = 1.0;
    cached_grid_.data.resize(static_cast<size_t>(width) * height);
    const auto * source = master_grid.getCharMap();
    std::transform(
      source, source + cached_grid_.data.size(), cached_grid_.data.begin(), occupancyValue);
    cache_initialized_ = true;
  } else {
    const auto start_x = std::clamp(min_i, 0, static_cast<int>(width));
    const auto end_x = std::clamp(max_i, start_x, static_cast<int>(width));
    const auto start_y = std::clamp(min_j, 0, static_cast<int>(height));
    const auto end_y = std::clamp(max_j, start_y, static_cast<int>(height));
    const auto * source = master_grid.getCharMap();
    for (auto y = start_y; y < end_y; ++y) {
      for (auto x = start_x; x < end_x; ++x) {
        const auto index = static_cast<size_t>(y) * width + x;
        cached_grid_.data[index] = occupancyValue(source[index]);
      }
    }
  }
  cached_grid_.header.stamp = stamp;
  cached_grid_.header.frame_id = layered_costmap_->getGlobalFrameID();
  publisher_->publish(cached_grid_);
  // Deliberately do not call setCost/updateWith*: master_grid is read-only here.
  current_ = true;
}

}  // namespace bio_nav_fusion

PLUGINLIB_EXPORT_CLASS(
  bio_nav_fusion::ReachabilityObserverLayer, nav2_costmap_2d::Layer)
