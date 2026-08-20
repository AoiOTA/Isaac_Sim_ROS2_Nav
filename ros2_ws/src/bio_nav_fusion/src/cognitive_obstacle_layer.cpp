#include "bio_nav_fusion/cognitive_obstacle_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bio_nav_fusion
{
namespace
{

int64_t stampNs(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

double durationSeconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1.0e-9;
}

bool unit(double value)
{
  return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

}  // namespace

CognitiveObstacleLayer::CognitiveObstacleLayer()
{
  enabled_ = true;
  current_ = true;
}

void CognitiveObstacleLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("CognitiveObstacleLayer lifecycle node expired");
  }
  const auto declare = [this, &node](const std::string & key, const auto & value) {
      nav2_util::declare_parameter_if_not_declared(
        node, name_ + "." + key, rclcpp::ParameterValue(value));
    };
  declare("enabled", true);
  declare("mode", mode_);
  declare("obstacle_topic", obstacle_topic_);
  declare("maximum_age_s", maximum_age_s_);
  declare("maximum_soft_cost", maximum_soft_cost_);
  declare("collision_min_height_m", collision_min_height_m_);
  declare("collision_max_height_m", collision_max_height_m_);
  declare("expected_reset_epoch", 0);
  declare("expected_recurrent_session_id", std::string(""));
  declare("expected_map_version", std::string(""));
  declare("expected_cognitive_tile_id", std::string(""));
  declare("expected_tile_revision", 0);
  declare("expected_graph_revision", 0);
  declare("expected_model_id", std::string(""));
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".mode", mode_);
  node->get_parameter(name_ + ".obstacle_topic", obstacle_topic_);
  node->get_parameter(name_ + ".maximum_age_s", maximum_age_s_);
  node->get_parameter(name_ + ".maximum_soft_cost", maximum_soft_cost_);
  node->get_parameter(name_ + ".collision_min_height_m", collision_min_height_m_);
  node->get_parameter(name_ + ".collision_max_height_m", collision_max_height_m_);
  int reset_epoch = 0;
  int tile_revision = 0;
  int graph_revision = 0;
  node->get_parameter(name_ + ".expected_reset_epoch", reset_epoch);
  node->get_parameter(
    name_ + ".expected_recurrent_session_id", expected_.recurrent_session_id);
  node->get_parameter(name_ + ".expected_map_version", expected_.map_version);
  node->get_parameter(
    name_ + ".expected_cognitive_tile_id", expected_.cognitive_tile_id);
  node->get_parameter(name_ + ".expected_tile_revision", tile_revision);
  node->get_parameter(name_ + ".expected_graph_revision", graph_revision);
  node->get_parameter(name_ + ".expected_model_id", expected_.model_id);
  expected_.reset_epoch = static_cast<uint32_t>(std::max(0, reset_epoch));
  expected_.tile_revision = static_cast<uint64_t>(std::max(0, tile_revision));
  expected_.graph_revision = static_cast<uint64_t>(std::max(0, graph_revision));
  maximum_soft_cost_ = std::clamp(maximum_soft_cost_, 1, 80);
  if (mode_ != "off" && mode_ != "shadow" && mode_ != "active") {
    throw std::runtime_error("CognitiveObstacleLayer mode must be off, shadow, or active");
  }
  status_publisher_ = node->create_publisher<bio_nav_interfaces::msg::RiskLayerStatus>(
    "/bio_nav/cognitive_obstacle_layer/status", rclcpp::QoS(10).reliable());
  if (enabled_ && mode_ != "off") {
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    subscription_ = node->create_subscription<
      bio_nav_interfaces::msg::CognitiveObstacleArray>(
      obstacle_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&CognitiveObstacleLayer::obstacleCallback, this, std::placeholders::_1),
      options);
  }
  default_value_ = nav2_costmap_2d::FREE_SPACE;
  matchSize();
  resetMaps();
}

void CognitiveObstacleLayer::activate()
{
  if (status_publisher_) {
    status_publisher_->on_activate();
  }
}

void CognitiveObstacleLayer::deactivate()
{
  if (status_publisher_) {
    status_publisher_->on_deactivate();
  }
}

void CognitiveObstacleLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_.reset();
  have_sequence_ = false;
  resetMaps();
  current_ = true;
}

std::string CognitiveObstacleLayer::validateMessage(
  const bio_nav_interfaces::msg::CognitiveObstacleArray & message,
  int64_t now_ns, const Identity & expected, uint64_t last_sequence,
  double maximum_age_s)
{
  const int64_t source_ns = stampNs(message.header.stamp);
  const double ttl_s = durationSeconds(message.ttl);
  const double age_s = static_cast<double>(now_ns - source_ns) * 1.0e-9;
  if (message.schema_version != "bio_nav_cognitive_obstacles_v1") {return "schema";}
  if (message.header.frame_id != "base_link") {return "frame";}
  if (source_ns <= 0 || !std::isfinite(ttl_s) || ttl_s <= 0.0 || ttl_s > 0.5 ||
    !std::isfinite(age_s) || age_s < 0.0 || age_s > std::min(ttl_s, maximum_age_s))
  {
    return "stale";
  }
  if (last_sequence != std::numeric_limits<uint64_t>::max() &&
    message.sequence <= last_sequence)
  {
    return "sequence";
  }
  if (message.reset_epoch != expected.reset_epoch ||
    message.recurrent_session_id != expected.recurrent_session_id ||
    message.map_version != expected.map_version ||
    message.cognitive_tile_id != expected.cognitive_tile_id ||
    message.tile_revision != expected.tile_revision ||
    message.graph_revision != expected.graph_revision ||
    message.model_id != expected.model_id)
  {
    return "identity";
  }
  if (!message.input_healthy || !message.module2_healthy ||
    !message.trusted_write || message.rejection_mask != 0U)
  {
    return "untrusted";
  }
  if (!unit(message.reliability) || !unit(message.ood_probability)) {return "nonfinite";}
  for (const auto & obstacle : message.obstacles) {
    if (obstacle.id.empty() || obstacle.class_id != "unknown_low_obstacle" ||
      !std::isfinite(obstacle.pose_xy_m[0]) || !std::isfinite(obstacle.pose_xy_m[1]) ||
      !std::isfinite(obstacle.radius_m) || obstacle.radius_m <= 0.0 ||
      !std::isfinite(obstacle.height_m) || obstacle.height_m < 0.0 ||
      !unit(obstacle.confidence) || !unit(obstacle.reliability) ||
      !unit(obstacle.ood_probability) ||
      !std::isfinite(obstacle.position_stddev_m[0]) ||
      !std::isfinite(obstacle.position_stddev_m[1]) ||
      obstacle.position_stddev_m[0] < 0.0 || obstacle.position_stddev_m[1] < 0.0 ||
      obstacle.count == 0U)
    {
      return "obstacle";
    }
    const int64_t seen_ns = stampNs(obstacle.last_seen);
    if (seen_ns <= 0 || seen_ns > now_ns ||
      static_cast<double>(now_ns - seen_ns) * 1.0e-9 > ttl_s)
    {
      return "obstacle_stale";
    }
  }
  return "";
}

uint8_t CognitiveObstacleLayer::obstacleCost(
  const bio_nav_interfaces::msg::CognitiveObstacle & obstacle,
  int maximum_soft_cost, double collision_min_height_m,
  double collision_max_height_m)
{
  const bool hard = obstacle.confidence >= 0.85 && obstacle.reliability >= 0.8 &&
    obstacle.ood_probability <= 0.2 && obstacle.count >= 3U &&
    std::max(obstacle.position_stddev_m[0], obstacle.position_stddev_m[1]) <= 0.10 &&
    obstacle.height_m >= collision_min_height_m &&
    obstacle.height_m <= collision_max_height_m;
  if (hard) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  const double evidence = std::clamp(
    obstacle.confidence * obstacle.reliability * (1.0 - obstacle.ood_probability),
    0.0, 1.0);
  return static_cast<uint8_t>(std::clamp(
      static_cast<int>(std::lround(1.0 + evidence * (maximum_soft_cost - 1))),
      1, std::clamp(maximum_soft_cost, 1, 80)));
}

void CognitiveObstacleLayer::obstacleCallback(
  const bio_nav_interfaces::msg::CognitiveObstacleArray::SharedPtr message)
{
  const auto now = clock_->now();
  Identity expected;
  uint64_t last = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (expected_.recurrent_session_id.empty()) {
      expected_ = Identity{
        message->reset_epoch, message->recurrent_session_id, message->map_version,
        message->cognitive_tile_id, message->tile_revision, message->graph_revision,
        message->model_id};
    }
    expected = expected_;
    last = have_sequence_ ? last_sequence_ : std::numeric_limits<uint64_t>::max();
  }
  const auto reason = validateMessage(*message, now.nanoseconds(), expected, last, maximum_age_s_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (reason.empty()) {
      latest_ = message;
      last_sequence_ = message->sequence;
      have_sequence_ = true;
    } else {
      latest_.reset();
    }
  }
  addExtraBounds(-1000.0, -1000.0, 1000.0, 1000.0);
  const double age_s = static_cast<double>(now.nanoseconds() - stampNs(message->header.stamp)) * 1.0e-9;
  publishStatus(*message, reason.empty() && mode_ == "active", reason, age_s);
}

void CognitiveObstacleLayer::updateBounds(
  double, double, double, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  useExtraBounds(min_x, min_y, max_x, max_y);
  std::lock_guard<std::mutex> lock(mutex_);
  if (latest_) {
    touch(-1000.0, -1000.0, min_x, min_y, max_x, max_y);
    touch(1000.0, 1000.0, min_x, min_y, max_x, max_y);
  }
  current_ = true;
}

void CognitiveObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  resetMap(
    static_cast<unsigned int>(std::max(0, min_i)),
    static_cast<unsigned int>(std::max(0, min_j)),
    static_cast<unsigned int>(std::max(0, max_i)),
    static_cast<unsigned int>(std::max(0, max_j)));
  bio_nav_interfaces::msg::CognitiveObstacleArray::SharedPtr message;
  Identity expected;
  uint64_t prior_sequence = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    message = latest_;
    expected = expected_;
    prior_sequence = message && message->sequence > 0 ?
      message->sequence - 1 : std::numeric_limits<uint64_t>::max();
  }
  if (!modeWritesCostmap(mode_) || !message ||
    !validateMessage(*message, clock_->now().nanoseconds(), expected,
      prior_sequence, maximum_age_s_).empty())
  {
    if (message && mode_ == "active") {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_.reset();
    }
    if (modeWritesCostmap(mode_)) {
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    }
    return;
  }
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_->lookupTransform(
      layered_costmap_->getGlobalFrameID(), message->header.frame_id,
      rclcpp::Time(message->header.stamp), tf2::durationFromSec(0.0));
  } catch (const std::exception &) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_.reset();
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    return;
  }
  for (const auto & obstacle : message->obstacles) {
    geometry_msgs::msg::PointStamped source;
    geometry_msgs::msg::PointStamped target;
    source.header = message->header;
    source.point.x = obstacle.pose_xy_m[0];
    source.point.y = obstacle.pose_xy_m[1];
    tf2::doTransform(source, target, transform);
    unsigned int center_x = 0;
    unsigned int center_y = 0;
    if (!worldToMap(target.point.x, target.point.y, center_x, center_y)) {continue;}
    const int radius_cells = static_cast<int>(std::ceil(obstacle.radius_m / getResolution()));
    const uint8_t cost = obstacleCost(
      obstacle, maximum_soft_cost_, collision_min_height_m_, collision_max_height_m_);
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        if (dx * dx + dy * dy > radius_cells * radius_cells) {continue;}
        const int mx = static_cast<int>(center_x) + dx;
        const int my = static_cast<int>(center_y) + dy;
        if (mx >= min_i && mx < max_i && my >= min_j && my < max_j &&
          mx >= 0 && my >= 0 && mx < static_cast<int>(getSizeInCellsX()) &&
          my < static_cast<int>(getSizeInCellsY()))
        {
          setCost(mx, my, std::max(getCost(mx, my), cost));
        }
      }
    }
  }
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

void CognitiveObstacleLayer::publishStatus(
  const bio_nav_interfaces::msg::CognitiveObstacleArray & message,
  bool applied, const std::string & reason, double age_s)
{
  if (!status_publisher_ || !status_publisher_->is_activated()) {return;}
  bio_nav_interfaces::msg::RiskLayerStatus status;
  status.stamp = clock_->now();
  status.consumer = name_;
  status.mode = mode_;
  status.offered = true;
  status.applied = applied;
  status.rejected = !reason.empty();
  status.source_sequence = message.sequence;
  status.recurrent_session_id = message.recurrent_session_id;
  status.rejection_mask = message.rejection_mask;
  status.fallback_reason = reason;
  status.message_age_ms = std::isfinite(age_s) ? age_s * 1000.0 :
    std::numeric_limits<float>::infinity();
  status.reset_epoch = message.reset_epoch;
  status.map_version = message.map_version;
  status_publisher_->publish(status);
}

}  // namespace bio_nav_fusion

PLUGINLIB_EXPORT_CLASS(
  bio_nav_fusion::CognitiveObstacleLayer, nav2_costmap_2d::Layer)
