#include "bio_nav_fusion/local_risk_grid_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bio_nav_fusion
{

LocalRiskGridLayer::LocalRiskGridLayer()
{
  enabled_ = true;
  current_ = true;
}

void LocalRiskGridLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("LocalRiskGridLayer lifecycle node expired");
  }
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".shadow_only", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".risk_topic", rclcpp::ParameterValue(risk_topic_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".reset_topic", rclcpp::ParameterValue(reset_topic_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_message_age_s", rclcpp::ParameterValue(max_message_age_s_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".transform_tolerance_s", rclcpp::ParameterValue(transform_tolerance_s_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".minimum_reliability", rclcpp::ParameterValue(minimum_reliability_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".maximum_ood_probability", rclcpp::ParameterValue(maximum_ood_probability_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".activation_threshold", rclcpp::ParameterValue(activation_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".clear_threshold", rclcpp::ParameterValue(clear_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".maximum_cost", rclcpp::ParameterValue(maximum_cost_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".expected_map_version", rclcpp::ParameterValue(expected_map_version_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".expected_model_sha256", rclcpp::ParameterValue(std::string("")));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".expected_qualification_sha256", rclcpp::ParameterValue(std::string("")));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".initial_reset_epoch", rclcpp::ParameterValue(0));
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".shadow_only", shadow_only_);
  node->get_parameter(name_ + ".risk_topic", risk_topic_);
  node->get_parameter(name_ + ".reset_topic", reset_topic_);
  node->get_parameter(name_ + ".max_message_age_s", max_message_age_s_);
  node->get_parameter(name_ + ".transform_tolerance_s", transform_tolerance_s_);
  node->get_parameter(name_ + ".minimum_reliability", minimum_reliability_);
  node->get_parameter(name_ + ".maximum_ood_probability", maximum_ood_probability_);
  node->get_parameter(name_ + ".activation_threshold", activation_threshold_);
  node->get_parameter(name_ + ".clear_threshold", clear_threshold_);
  node->get_parameter(name_ + ".maximum_cost", maximum_cost_);
  node->get_parameter(name_ + ".expected_map_version", expected_map_version_);
  node->get_parameter(name_ + ".expected_model_sha256", expected_model_sha256_);
  node->get_parameter(
    name_ + ".expected_qualification_sha256", expected_qualification_sha256_);
  int initial_reset_epoch = 0;
  node->get_parameter(name_ + ".initial_reset_epoch", initial_reset_epoch);
  reset_epoch_ = static_cast<uint32_t>(std::max(0, initial_reset_epoch));
  reset_epoch_initialized_ = initial_reset_epoch > 0;
  maximum_cost_ = std::clamp(maximum_cost_, 1, 80);
  activation_threshold_ = std::clamp(activation_threshold_, 0.01, 0.99);
  clear_threshold_ = std::clamp(clear_threshold_, 0.0, activation_threshold_);

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  grid_subscription_ = node->create_subscription<
    bio_nav_interfaces::msg::LocalRiskGrid>(
    risk_topic_, rclcpp::QoS(1).reliable(),
    std::bind(&LocalRiskGridLayer::gridCallback, this, std::placeholders::_1),
    options);
  reset_subscription_ = node->create_subscription<std_msgs::msg::Empty>(
    reset_topic_, rclcpp::QoS(10).reliable(),
    std::bind(&LocalRiskGridLayer::resetCallback, this, std::placeholders::_1),
    options);
  status_publisher_ = node->create_publisher<bio_nav_interfaces::msg::RiskLayerStatus>(
    "/bio_nav/local_risk_layer/status", rclcpp::QoS(10).reliable());
  matchSize();
  resetMaps();
}

void LocalRiskGridLayer::activate()
{
  if (status_publisher_) {
    status_publisher_->on_activate();
  }
}

void LocalRiskGridLayer::deactivate()
{
  if (status_publisher_) {
    status_publisher_->on_deactivate();
  }
}

void LocalRiskGridLayer::reset()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_.reset();
    active_cells_.fill(false);
  }
  resetMaps();
  current_ = true;
}

void LocalRiskGridLayer::gridCallback(
  const bio_nav_interfaces::msg::LocalRiskGrid::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!reset_epoch_initialized_) {
    reset_epoch_ = message->reset_epoch;
    reset_epoch_initialized_ = true;
  }
  latest_ = message;
}

void LocalRiskGridLayer::resetCallback(const std_msgs::msg::Empty::SharedPtr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (reset_epoch_initialized_) {
    ++reset_epoch_;
  }
  latest_.reset();
  active_cells_.fill(false);
}

void LocalRiskGridLayer::updateBounds(
  double robot_x, double robot_y, double, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  useExtraBounds(min_x, min_y, max_x, max_y);
  // The local grid is 16 m square. A yaw-independent radius safely covers
  // every transformed corner without assuming map/base alignment.
  constexpr double radius = 11.4;
  touch(robot_x - radius, robot_y - radius, min_x, min_y, max_x, max_y);
  touch(robot_x + radius, robot_y + radius, min_x, min_y, max_x, max_y);
  current_ = true;
}

std::string LocalRiskGridLayer::validateGrid(
  const bio_nav_interfaces::msg::LocalRiskGrid * grid,
  double age_s, double maximum_age_s, double minimum_reliability,
  double maximum_ood_probability, uint32_t reset_epoch,
  const std::string & expected_map_version,
  const std::string & expected_model_sha256,
  const std::string & expected_qualification_sha256)
{
  if (grid == nullptr) {
    return "no_grid";
  }
  if (!std::isfinite(age_s) || age_s < 0.0 || age_s > maximum_age_s) {
    return "stale";
  }
  if (
    grid->schema_version != "bio_nav_local_risk_grid_v1" ||
    grid->header.frame_id != "base_link" || grid->width != 32U ||
    grid->height != 32U || std::abs(grid->resolution - 0.5F) > 1.0e-6F ||
    std::abs(grid->origin_x + 8.0F) > 1.0e-6F ||
    std::abs(grid->origin_y + 8.0F) > 1.0e-6F ||
    std::abs(grid->horizon_s - 0.8F) > 1.0e-6F)
  {
    return "geometry_mismatch";
  }
  if (
    !grid->healthy || grid->rejection_mask != 0U ||
    !std::isfinite(grid->reliability) ||
    !std::isfinite(grid->ood_probability) ||
    grid->reliability < minimum_reliability ||
    grid->ood_probability > maximum_ood_probability)
  {
    return "risk_unhealthy";
  }
  if (grid->reset_epoch != reset_epoch || grid->map_version != expected_map_version) {
    return "map_reset_mismatch";
  }
  if (
    (!expected_model_sha256.empty() && grid->model_sha256 != expected_model_sha256) ||
    (!expected_qualification_sha256.empty() &&
    grid->qualification_receipt_sha256 != expected_qualification_sha256))
  {
    return "model_hash_mismatch";
  }
  for (const float probability : grid->risk) {
    if (!std::isfinite(probability) || probability < 0.0F || probability > 1.0F) {
      return "nonfinite_or_unbounded";
    }
  }
  return "";
}

uint8_t LocalRiskGridLayer::mapRiskCost(
  float probability, float threshold, int maximum_cost)
{
  if (
    !std::isfinite(probability) || !std::isfinite(threshold) ||
    threshold < 0.0F || threshold >= 1.0F || probability < threshold ||
    maximum_cost <= 0)
  {
    return 0;
  }
  const auto normalized = std::clamp(
    (probability - threshold) / (1.0F - threshold), 0.0F, 1.0F);
  return static_cast<uint8_t>(std::clamp(
      static_cast<int>(std::lround(1.0 + normalized * (maximum_cost - 1))),
      1, std::min(maximum_cost, 80)));
}

void LocalRiskGridLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  resetMap(
    static_cast<unsigned int>(std::max(0, min_i)),
    static_cast<unsigned int>(std::max(0, min_j)),
    static_cast<unsigned int>(std::max(0, max_i)),
    static_cast<unsigned int>(std::max(0, max_j)));
  bio_nav_interfaces::msg::LocalRiskGrid::SharedPtr grid;
  std::array<bool, 1024> previous_active{};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    grid = latest_;
    previous_active = active_cells_;
  }
  const double age_s = grid ?
    (clock_->now() - rclcpp::Time(grid->header.stamp)).seconds() :
    std::numeric_limits<double>::infinity();
  const auto reason = validateGrid(
    grid.get(), age_s, max_message_age_s_, minimum_reliability_,
    maximum_ood_probability_, reset_epoch_, expected_map_version_,
    expected_model_sha256_, expected_qualification_sha256_);
  if (!enabled_ || !reason.empty()) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_cells_.fill(false);
    }
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    publishStatus(false, enabled_ ? reason : "disabled", age_s, 0, 0);
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_->lookupTransform(
      layered_costmap_->getGlobalFrameID(), grid->header.frame_id,
      rclcpp::Time(grid->header.stamp),
      tf2::durationFromSec(transform_tolerance_s_));
  } catch (const tf2::TransformException &) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_cells_.fill(false);
    }
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    publishStatus(false, "tf_invalid", age_s, 0, 0);
    return;
  }

  std::array<bool, 1024> next_active{};
  uint32_t active_count = 0;
  uint8_t maximum_written = 0;
  for (std::size_t index = 0; index < grid->risk.size(); ++index) {
    const auto threshold = previous_active[index] ? clear_threshold_ : activation_threshold_;
    const float probability = grid->risk[index];
    if (grid->visibility[index] == 0U || probability < threshold) {
      continue;
    }
    next_active[index] = true;
    const auto row = static_cast<unsigned int>(index / 32U);
    const auto column = static_cast<unsigned int>(index % 32U);
    geometry_msgs::msg::PointStamped local;
    local.header = grid->header;
    local.point.x = grid->origin_x + (static_cast<double>(column) + 0.5) * grid->resolution;
    local.point.y = grid->origin_y + (static_cast<double>(row) + 0.5) * grid->resolution;
    geometry_msgs::msg::PointStamped global;
    tf2::doTransform(local, global, transform);
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!worldToMap(global.point.x, global.point.y, mx, my)) {
      continue;
    }
    if (
      static_cast<int>(mx) < min_i || static_cast<int>(mx) >= max_i ||
      static_cast<int>(my) < min_j || static_cast<int>(my) >= max_j)
    {
      continue;
    }
    const auto cost = mapRiskCost(probability, static_cast<float>(threshold), maximum_cost_);
    setCost(mx, my, std::max(getCost(mx, my), cost));
    ++active_count;
    maximum_written = std::max(maximum_written, cost);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_cells_ = next_active;
  }
  if (!shadow_only_) {
    // Max combination cannot clear or lower LiDAR/depth/static obstacles and
    // maximum_cost_ is capped at 80, far below lethal cost.
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  }
  publishStatus(
    !shadow_only_, shadow_only_ ? "shadow_only" : "", age_s,
    active_count, maximum_written);
}

void LocalRiskGridLayer::publishStatus(
  bool applied, const std::string & reason, double age_s,
  uint32_t active_cells, uint8_t maximum_cost)
{
  if (!status_publisher_ || !status_publisher_->is_activated()) {
    return;
  }
  bio_nav_interfaces::msg::RiskLayerStatus status;
  status.stamp = clock_->now();
  status.applied = applied;
  status.fallback_reason = reason;
  status.message_age_ms = std::isfinite(age_s) ?
    static_cast<float>(age_s * 1000.0) : std::numeric_limits<float>::infinity();
  status.active_cell_count = active_cells;
  status.maximum_cost = maximum_cost;
  status.reset_epoch = reset_epoch_;
  status.map_version = expected_map_version_;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_) {
      status.risk_model_sha256 = latest_->model_sha256;
      status.qualification_receipt_sha256 = latest_->qualification_receipt_sha256;
    }
  }
  status_publisher_->publish(status);
}

}  // namespace bio_nav_fusion

PLUGINLIB_EXPORT_CLASS(
  bio_nav_fusion::LocalRiskGridLayer, nav2_costmap_2d::Layer)
