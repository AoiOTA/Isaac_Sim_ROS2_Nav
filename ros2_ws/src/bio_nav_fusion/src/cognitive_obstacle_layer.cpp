#include "bio_nav_fusion/cognitive_obstacle_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
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

int64_t durationNs(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<int64_t>(duration.sec) * 1000000000LL + duration.nanosec;
}

bool validStamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec >= 0 && stamp.nanosec < 1000000000U && stampNs(stamp) > 0;
}

bool zeroStamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec == 0 && stamp.nanosec == 0U;
}

bool validNonnegativeDuration(const builtin_interfaces::msg::Duration & duration)
{
  return duration.sec >= 0 && duration.nanosec < 1000000000U;
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

bool identityFieldsPresent(
  const bio_nav_interfaces::msg::CognitiveObstacleArray & message)
{
  return !message.recurrent_session_id.empty() && !message.map_version.empty() &&
         !message.cognitive_tile_id.empty() && !message.model_id.empty();
}

bool sameIdentity(
  const bio_nav_interfaces::msg::CognitiveObstacleArray & message,
  const CognitiveObstacleLayer::Identity & expected)
{
  return message.reset_epoch == expected.reset_epoch &&
         message.recurrent_session_id == expected.recurrent_session_id &&
         message.map_version == expected.map_version &&
         message.cognitive_tile_id == expected.cognitive_tile_id &&
         message.tile_revision == expected.tile_revision &&
         message.graph_revision == expected.graph_revision &&
         message.model_id == expected.model_id;
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
  declare("consumer_id", std::string(""));
  declare("obstacle_topic", obstacle_topic_);
  declare("maximum_age_s", maximum_age_s_);
  declare("maximum_ood_probability", maximum_ood_probability_);
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
  std::string consumer_id_override;
  node->get_parameter(name_ + ".consumer_id", consumer_id_override);
  consumer_id_ = resolveConsumerId(
    node->get_fully_qualified_name(), name_, consumer_id_override);
  node->get_parameter(name_ + ".obstacle_topic", obstacle_topic_);
  node->get_parameter(name_ + ".maximum_age_s", maximum_age_s_);
  node->get_parameter(
    name_ + ".maximum_ood_probability", maximum_ood_probability_);
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
  const bool any_identity_parameter =
    !expected_.recurrent_session_id.empty() || !expected_.map_version.empty() ||
    !expected_.cognitive_tile_id.empty() || !expected_.model_id.empty();
  const bool complete_identity_parameter =
    !expected_.recurrent_session_id.empty() && !expected_.map_version.empty() &&
    !expected_.cognitive_tile_id.empty() && !expected_.model_id.empty();
  if (any_identity_parameter && !complete_identity_parameter) {
    throw std::runtime_error(
            "CognitiveObstacleLayer expected identity must be complete or omitted");
  }
  identity_parameters_configured_ = complete_identity_parameter;
  identity_bound_ = identity_parameters_configured_;
  maximum_soft_cost_ = std::clamp(maximum_soft_cost_, 1, 80);
  if (!std::isfinite(maximum_age_s_) || maximum_age_s_ <= 0.0 ||
    !unit(maximum_ood_probability_))
  {
    throw std::runtime_error("CognitiveObstacleLayer age/OOD gates are invalid");
  }
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
  accepted_.reset();
  if (!identity_parameters_configured_) {
    expected_ = Identity{};
    identity_bound_ = false;
  }
  resetMaps();
  current_ = true;
}

std::string CognitiveObstacleLayer::validateMessage(
  const bio_nav_interfaces::msg::CognitiveObstacleArray & message,
  int64_t now_ns, const Identity & expected, const AcceptanceCursor & accepted,
  double maximum_age_s, double maximum_ood_probability, bool enforce_identity)
{
  const int64_t source_ns = stampNs(message.header.stamp);
  const int64_t validation_ns = stampNs(message.validation_stamp);
  const int64_t source_age_ns = durationNs(message.source_age);
  const int64_t source_odom_ns = stampNs(message.source_odom_stamp);
  const int64_t validation_odom_ns = stampNs(message.validation_odom_stamp);
  const double ttl_s = durationSeconds(message.ttl);
  const double validation_ttl_s = durationSeconds(message.validation_ttl);
  const double validation_age_s = static_cast<double>(now_ns - validation_ns) * 1.0e-9;
  constexpr int64_t kMaximumSourceAgeNs = 2000000000LL;
  constexpr int64_t kFutureToleranceNs = 50000000LL;
  if (message.schema_version != "bio_nav_cognitive_obstacles_v1") {return "schema";}
  if (message.header.frame_id != "base_link") {return "frame";}
  if (!identityFieldsPresent(message)) {return "identity";}
  if (!validStamp(message.header.stamp) ||
    !validStamp(message.validation_stamp) || validation_ns < source_ns)
  {
    return "validation_time";
  }
  if (!validNonnegativeDuration(message.source_age) || source_age_ns > kMaximumSourceAgeNs ||
    validation_ns - source_ns != source_age_ns)
  {
    return "source_age";
  }
  if (!validNonnegativeDuration(message.ttl) || !std::isfinite(ttl_s) ||
    ttl_s <= 0.0 || ttl_s > 0.5 ||
    !validNonnegativeDuration(message.validation_ttl) ||
    !std::isfinite(validation_ttl_s) || validation_ttl_s <= 0.0 ||
    validation_ttl_s > 0.5)
  {
    return "ttl";
  }
  if (!std::isfinite(validation_age_s) || now_ns + kFutureToleranceNs < validation_ns ||
    validation_age_s > std::min(validation_ttl_s, maximum_age_s))
  {
    return "validation_stale";
  }
  if (message.validation_mode ==
    bio_nav_interfaces::msg::CognitiveObstacleArray::VALIDATION_FRESH)
  {
    if (validation_ns != source_ns || source_age_ns != 0 ||
      message.validation_sensor_mask != 0U)
    {
      return "fresh_mismatch";
    }
    const bool source_odom_zero = zeroStamp(message.source_odom_stamp);
    const bool validation_odom_zero = zeroStamp(message.validation_odom_stamp);
    if (source_odom_zero != validation_odom_zero ||
      (!source_odom_zero &&
      (!validStamp(message.source_odom_stamp) ||
      !validStamp(message.validation_odom_stamp) ||
      validation_odom_ns != source_odom_ns ||
      now_ns + kFutureToleranceNs < validation_odom_ns)))
    {
      return "odom_time";
    }
  } else if (message.validation_mode ==
    bio_nav_interfaces::msg::CognitiveObstacleArray::VALIDATION_STATIC_DEPTH_REVALIDATED)
  {
    if (!validStamp(message.source_odom_stamp) ||
      !validStamp(message.validation_odom_stamp) || validation_odom_ns < source_odom_ns ||
      validation_odom_ns - source_odom_ns != source_age_ns ||
      now_ns + kFutureToleranceNs < validation_odom_ns)
    {
      return "odom_time";
    }
    if ((message.validation_sensor_mask &
      bio_nav_interfaces::msg::CognitiveObstacleArray::VALIDATION_SENSOR_DEPTH) == 0U)
    {
      return "validation_sensor";
    }
  } else {
    return "validation_mode";
  }
  if (message.sequence == 0U) {return "sequence";}
  if (accepted.valid) {
    if (message.sequence < accepted.source_sequence) {return "sequence";}
    if (message.sequence > accepted.source_sequence) {
      if (source_ns <= accepted.source_stamp_ns) {return "source_regression";}
    } else {
      if (message.validation_mode !=
        bio_nav_interfaces::msg::CognitiveObstacleArray::VALIDATION_STATIC_DEPTH_REVALIDATED)
      {
        return "sequence";
      }
      if (source_ns != accepted.source_stamp_ns) {return "source_mismatch";}
      if (validation_ns <= accepted.validation_stamp_ns) {
        return "validation_regression";
      }
    }
  }
  if (!message.input_healthy || !message.module2_healthy ||
    !message.observation_valid || !message.trusted_write ||
    message.rejection_mask != 0U)
  {
    return "untrusted";
  }
  if (!unit(message.reliability) || !unit(message.ood_probability) ||
    message.ood_probability > maximum_ood_probability)
  {
    return "ood";
  }
  for (const auto & obstacle : message.obstacles) {
    if (obstacle.id.empty() || obstacle.class_id != "unknown_low_obstacle" ||
      !std::isfinite(obstacle.pose_xy_m[0]) || !std::isfinite(obstacle.pose_xy_m[1]) ||
      !std::isfinite(obstacle.radius_m) || obstacle.radius_m <= 0.0 ||
      !std::isfinite(obstacle.height_m) || obstacle.height_m < 0.0 ||
      !unit(obstacle.confidence) || !unit(obstacle.reliability) ||
      !unit(obstacle.ood_probability) ||
      obstacle.ood_probability > maximum_ood_probability ||
      !std::isfinite(obstacle.position_stddev_m[0]) ||
      !std::isfinite(obstacle.position_stddev_m[1]) ||
      obstacle.position_stddev_m[0] < 0.0 || obstacle.position_stddev_m[1] < 0.0 ||
      obstacle.count == 0U)
    {
      return "obstacle";
    }
    if (message.validation_mode ==
      bio_nav_interfaces::msg::CognitiveObstacleArray::VALIDATION_STATIC_DEPTH_REVALIDATED &&
      (obstacle.motion_class !=
      bio_nav_interfaces::msg::CognitiveObstacle::MOTION_STATIC ||
      !obstacle.static_confirmed))
    {
      return "static_confirmation";
    }
    const int64_t seen_ns = stampNs(obstacle.last_seen);
    if (!validStamp(obstacle.last_seen) || seen_ns > source_ns ||
      static_cast<double>(source_ns - seen_ns) * 1.0e-9 > ttl_s)
    {
      return "obstacle_stale";
    }
  }
  if (enforce_identity && !sameIdentity(message, expected)) {return "identity";}
  return "";
}

std::string CognitiveObstacleLayer::validateMessage(
  const bio_nav_interfaces::msg::CognitiveObstacleArray & message,
  int64_t now_ns, const Identity & expected, uint64_t last_sequence,
  double maximum_age_s, double maximum_ood_probability, bool enforce_identity)
{
  if (last_sequence != std::numeric_limits<uint64_t>::max() &&
    message.sequence <= last_sequence)
  {
    return "sequence";
  }
  return validateMessage(
    message, now_ns, expected, AcceptanceCursor{}, maximum_age_s,
    maximum_ood_probability, enforce_identity);
}

void CognitiveObstacleLayer::recordAccepted(
  const bio_nav_interfaces::msg::CognitiveObstacleArray & message,
  AcceptanceCursor & accepted)
{
  accepted.valid = true;
  accepted.source_sequence = message.sequence;
  accepted.source_stamp_ns = stampNs(message.header.stamp);
  accepted.validation_stamp_ns = stampNs(message.validation_stamp);
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

std::string CognitiveObstacleLayer::resolveConsumerId(
  const std::string & node_fully_qualified_name,
  const std::string & layer_name,
  const std::string & override_id)
{
  if (!override_id.empty()) {
    return override_id;
  }
  const std::string node_id = node_fully_qualified_name.empty() ?
    std::string("/unknown_costmap") : node_fully_qualified_name;
  const std::string layer_id = layer_name.empty() ?
    std::string("cognitive_obstacle_layer") : layer_name;
  return node_id + ":" + layer_id;
}

void CognitiveObstacleLayer::obstacleCallback(
  const bio_nav_interfaces::msg::CognitiveObstacleArray::SharedPtr message)
{
  const auto now = clock_->now();
  std::string reason;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool enforce_identity = identity_bound_;
    reason = validateMessage(
      *message, now.nanoseconds(), expected_, accepted_, maximum_age_s_,
      maximum_ood_probability_, enforce_identity);
    if (reason.empty()) {
      if (!identity_bound_) {
        expected_ = Identity{
          message->reset_epoch, message->recurrent_session_id, message->map_version,
          message->cognitive_tile_id, message->tile_revision, message->graph_revision,
          message->model_id};
        identity_bound_ = true;
      }
      latest_ = message;
      recordAccepted(*message, accepted_);
    } else {
      latest_.reset();
      resetMaps();
    }
  }
  addExtraBounds(-1000.0, -1000.0, 1000.0, 1000.0);
  const double age_s = static_cast<double>(
    now.nanoseconds() - stampNs(message->validation_stamp)) * 1.0e-9;
  const std::string status_reason = reason.empty() ?
    (mode_ == "shadow" ? "shadow" : "offered") : reason;
  publishStatus(*message, false, status_reason, age_s);
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
  {
    std::lock_guard<std::mutex> lock(mutex_);
    message = latest_;
    expected = expected_;
  }
  const auto now = clock_->now();
  const AcceptanceCursor no_ordering_gate;
  const auto validation_reason = message ? validateMessage(
    *message, now.nanoseconds(), expected, no_ordering_gate, maximum_age_s_,
    maximum_ood_probability_, true) : std::string("missing");
  if (!modeWritesCostmap(mode_) || !message || !validation_reason.empty())
  {
    if (message && mode_ == "active") {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_.reset();
    }
    if (modeWritesCostmap(mode_)) {
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      if (message && !validation_reason.empty()) {
        const double age_s = static_cast<double>(
          now.nanoseconds() - stampNs(message->header.stamp)) * 1.0e-9;
        publishStatus(*message, false, validation_reason, age_s);
      }
    }
    return;
  }
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_->lookupTransform(
      layered_costmap_->getGlobalFrameID(), message->header.frame_id,
      rclcpp::Time(message->validation_stamp), tf2::durationFromSec(0.0));
  } catch (const std::exception &) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_.reset();
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    const double age_s = static_cast<double>(
      now.nanoseconds() - stampNs(message->validation_stamp)) * 1.0e-9;
    publishStatus(*message, false, tfFailureReason(), age_s);
    return;
  }
  uint32_t active_cells = 0;
  uint32_t raised_cells = 0;
  uint32_t masked_cells = 0;
  uint8_t maximum_cost = 0;
  uint8_t maximum_cost_increase = 0;
  for (const auto & obstacle : message->obstacles) {
    geometry_msgs::msg::PointStamped source;
    geometry_msgs::msg::PointStamped target;
    source.header = message->header;
    source.header.stamp = message->validation_stamp;
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
          const auto private_before = getCost(mx, my);
          const auto private_after = mergeCellCost(mode_, private_before, cost);
          if (private_after > private_before) {
            ++active_cells;
            maximum_cost = std::max(maximum_cost, private_after);
            const auto master_before = master_grid.getCost(mx, my);
            if (master_before != nav2_costmap_2d::NO_INFORMATION &&
              private_after <= master_before)
            {
              ++masked_cells;
            } else {
              ++raised_cells;
              maximum_cost_increase = std::max<uint8_t>(
                maximum_cost_increase,
                master_before == nav2_costmap_2d::NO_INFORMATION ?
                private_after :
                static_cast<uint8_t>(private_after - master_before));
            }
            setCost(mx, my, private_after);
          }
        }
      }
    }
  }
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  const bool applied = raised_cells > 0U;
  const std::string reason = applied ? "" :
    (active_cells > 0U ? "masked" : "no_costmap_cells");
  const double age_s = static_cast<double>(
    now.nanoseconds() - stampNs(message->validation_stamp)) * 1.0e-9;
  publishStatus(
    *message, applied, reason, age_s, active_cells, maximum_cost,
    raised_cells, masked_cells, maximum_cost_increase);
}

void CognitiveObstacleLayer::publishStatus(
  const bio_nav_interfaces::msg::CognitiveObstacleArray & message,
  bool applied, const std::string & reason, double age_s,
  uint32_t active_cells, uint8_t maximum_cost, uint32_t raised_cells,
  uint32_t masked_cells, uint8_t maximum_cost_increase)
{
  if (!status_publisher_ || !status_publisher_->is_activated()) {return;}
  bio_nav_interfaces::msg::RiskLayerStatus status;
  status.stamp = clock_->now();
  status.consumer = consumer_id_;
  status.mode = mode_;
  status.offered = true;
  status.applied = applied;
  status.rejected = !reason.empty() && reason != "offered" && reason != "shadow";
  status.source_sequence = message.sequence;
  status.recurrent_session_id = message.recurrent_session_id;
  status.rejection_mask = message.rejection_mask;
  std::ostringstream detail;
  detail << "validation_mode=" << static_cast<unsigned int>(message.validation_mode)
         << ";source_age_ms=" << durationSeconds(message.source_age) * 1000.0
         << ";rejection_reason=" << reason
         << ";confirmed_count=" << std::count_if(
    message.obstacles.begin(), message.obstacles.end(),
    [](const auto & obstacle) {
      return obstacle.motion_class ==
             bio_nav_interfaces::msg::CognitiveObstacle::MOTION_STATIC &&
             obstacle.static_confirmed;
    });
  status.fallback_reason = detail.str();
  status.message_age_ms = std::isfinite(age_s) ? age_s * 1000.0 :
    std::numeric_limits<float>::infinity();
  status.active_cell_count = active_cells;
  status.maximum_cost = maximum_cost;
  status.raised_cell_count = raised_cells;
  status.masked_by_existing_cost_count = masked_cells;
  status.maximum_cost_increase = maximum_cost_increase;
  status.reset_epoch = message.reset_epoch;
  status.map_version = message.map_version;
  status_publisher_->publish(status);
}

}  // namespace bio_nav_fusion

PLUGINLIB_EXPORT_CLASS(
  bio_nav_fusion::CognitiveObstacleLayer, nav2_costmap_2d::Layer)
