#include "bio_nav_fusion/cognitive_obstacle_layer.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <tuple>
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

constexpr int64_t kMaximumStaticSourceAgeNs = 5000000000LL;

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
  declare("track_ttl_s", track_ttl_s_);
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
  node->get_parameter(name_ + ".track_ttl_s", track_ttl_s_);
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
    !std::isfinite(track_ttl_s_) || track_ttl_s_ <= 0.0 ||
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
  latest_admission_reason_.clear();
  accepted_.reset();
  clearStaticTracks();
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
  constexpr int64_t kFutureToleranceNs = 50000000LL;
  constexpr int64_t kOdomSemanticEndpointToleranceNs = 100000000LL;
  if (message.schema_version != "bio_nav_cognitive_obstacles_v1") {return "schema";}
  if (message.header.frame_id != "base_link") {return "frame";}
  if (!identityFieldsPresent(message)) {return "identity";}
  if (!validStamp(message.header.stamp) ||
    !validStamp(message.validation_stamp) || validation_ns < source_ns)
  {
    return "validation_time";
  }
  if (!validNonnegativeDuration(message.source_age) ||
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
    if (source_age_ns > kMaximumStaticSourceAgeNs) {
      return "source_age";
    }
    if (!validStamp(message.source_odom_stamp) ||
      !validStamp(message.validation_odom_stamp) || validation_odom_ns < source_odom_ns ||
      std::abs(source_odom_ns - source_ns) > kOdomSemanticEndpointToleranceNs ||
      std::abs(validation_odom_ns - validation_ns) > kOdomSemanticEndpointToleranceNs ||
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
  return obstacleCost(
    obstacle, obstacle.count, maximum_soft_cost, collision_min_height_m,
    collision_max_height_m);
}

uint8_t CognitiveObstacleLayer::obstacleCost(
  const bio_nav_interfaces::msg::CognitiveObstacle & obstacle,
  uint64_t effective_count, int maximum_soft_cost,
  double collision_min_height_m, double collision_max_height_m)
{
  const bool hard = obstacle.confidence >= 0.85 && obstacle.reliability >= 0.8 &&
    obstacle.ood_probability <= 0.2 && effective_count >= 3U &&
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

bool CognitiveObstacleLayer::StaticTrackKey::operator<(
  const StaticTrackKey & other) const
{
  return std::tie(
    reset_epoch, recurrent_session_id, map_version, cognitive_tile_id,
    tile_revision, graph_revision, model_id, track_id) <
         std::tie(
    other.reset_epoch, other.recurrent_session_id, other.map_version,
    other.cognitive_tile_id, other.tile_revision, other.graph_revision,
    other.model_id, other.track_id);
}

CognitiveObstacleLayer::StaticTrackKey CognitiveObstacleLayer::staticTrackKey(
  const bio_nav_interfaces::msg::CognitiveObstacleArray & message,
  const std::string & track_id)
{
  return StaticTrackKey{
    message.reset_epoch, message.recurrent_session_id, message.map_version,
    message.cognitive_tile_id, message.tile_revision, message.graph_revision,
    message.model_id, track_id};
}

bool CognitiveObstacleLayer::sameStaticIdentity(
  const StaticTrackKey & first, const StaticTrackKey & second)
{
  return first.reset_epoch == second.reset_epoch &&
         first.recurrent_session_id == second.recurrent_session_id &&
         first.map_version == second.map_version &&
         first.cognitive_tile_id == second.cognitive_tile_id &&
         first.tile_revision == second.tile_revision &&
         first.graph_revision == second.graph_revision &&
         first.model_id == second.model_id;
}

uint64_t CognitiveObstacleLayer::observeStaticTrack(
  const bio_nav_interfaces::msg::CognitiveObstacleArray & message,
  const bio_nav_interfaces::msg::CognitiveObstacle & obstacle,
  double map_x, double map_y)
{
  const auto key = staticTrackKey(message, obstacle.id);
  auto & state = static_tracks_[key];
  const int64_t validation_ns = stampNs(message.validation_stamp);
  const bool independent_rehit =
    state.rehit_count == 0U || state.last_source_sequence != message.sequence ||
    state.last_validation_stamp_ns != validation_ns;
  if (independent_rehit) {
    const int64_t refresh_ns = clock_->now().nanoseconds();
    if (state.rehit_count == 0U) {
      state.anchor_map_x = map_x;
      state.anchor_map_y = map_y;
      state.first_refresh_ns = refresh_ns;
    }
    ++state.rehit_count;
    state.last_source_sequence = message.sequence;
    state.last_validation_stamp_ns = validation_ns;
    state.last_refresh_ns = refresh_ns;
    state.map_x = map_x;
    state.map_y = map_y;
    if (!state.promoted) {
      state.obstacle = obstacle;
      state.radius_m = obstacle.radius_m;
      state.height_m = obstacle.height_m;
      const uint64_t effective_count = std::max(obstacle.count, state.rehit_count);
      state.promoted = obstacleCost(
        state.obstacle, effective_count, maximum_soft_cost_,
        collision_min_height_m_, collision_max_height_m_) ==
        nav2_costmap_2d::LETHAL_OBSTACLE;
    }
  }
  return std::max(obstacle.count, state.rehit_count);
}

std::vector<CognitiveObstacleLayer::AppliedObstacle>
CognitiveObstacleLayer::promotedStaticObstacles()
{
  pruneStaticTracks();
  std::vector<AppliedObstacle> obstacles;
  for (const auto & [key, state] : static_tracks_) {
    (void)key;
    if (!state.promoted || !state.reassociated_to_track_id.empty()) {
      continue;
    }
    obstacles.push_back(AppliedObstacle{
      state.obstacle, state.map_x, state.map_y,
      std::max(state.obstacle.count, state.rehit_count)});
  }
  return obstacles;
}

bool CognitiveObstacleLayer::hasPromotedStaticObstacle()
{
  pruneStaticTracks();
  return std::any_of(
    static_tracks_.begin(), static_tracks_.end(),
    [](const auto & entry) {
      return entry.second.promoted && entry.second.reassociated_to_track_id.empty();
    });
}

void CognitiveObstacleLayer::pruneStaticTracks()
{
  const int64_t now_ns = clock_->now().nanoseconds();
  const int64_t cutoff_ns =
    now_ns - static_cast<int64_t>(track_ttl_s_ * 1.0e9);
  for (auto it = static_tracks_.begin(); it != static_tracks_.end(); ) {
    if (it->second.last_refresh_ns < cutoff_ns) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "CognitiveObstacleLayer static track '%s' expired after %.1f s without refresh",
        it->first.track_id.c_str(), track_ttl_s_);
      it = static_tracks_.erase(it);
    } else {
      ++it;
    }
  }
  for (auto it = static_tracks_.begin(); it != static_tracks_.end(); ) {
    if (it->second.reassociated_to_track_id.empty()) {
      ++it;
      continue;
    }
    auto canonical_key = it->first;
    canonical_key.track_id = it->second.reassociated_to_track_id;
    const auto canonical = static_tracks_.find(canonical_key);
    if (canonical == static_tracks_.end() ||
      !canonical->second.reassociated_to_track_id.empty())
    {
      it = static_tracks_.erase(it);
    } else {
      ++it;
    }
  }
}

void CognitiveObstacleLayer::clearStaticTracks()
{
  static_tracks_.clear();
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
      latest_admission_reason_.clear();
      recordAccepted(*message, accepted_);
    } else {
      latest_ = message;
      latest_admission_reason_ = reason;
      if (reason == "identity" && identityFieldsPresent(*message) &&
        !sameIdentity(*message, expected_))
      {
        clearStaticTracks();
        resetMaps();
      }
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
  if (latest_ || hasPromotedStaticObstacle()) {
    touch(-1000.0, -1000.0, min_x, min_y, max_x, max_y);
    touch(1000.0, 1000.0, min_x, min_y, max_x, max_y);
  }
  current_ = true;
}

void CognitiveObstacleLayer::synchronizeCostmapGeometry(
  const nav2_costmap_2d::Costmap2D & master_grid)
{
  if (getSizeInCellsX() != master_grid.getSizeInCellsX() ||
    getSizeInCellsY() != master_grid.getSizeInCellsY() ||
    getResolution() != master_grid.getResolution())
  {
    resizeMap(
      master_grid.getSizeInCellsX(), master_grid.getSizeInCellsY(),
      master_grid.getResolution(), master_grid.getOriginX(),
      master_grid.getOriginY());
    return;
  }
  if (getOriginX() != master_grid.getOriginX() ||
    getOriginY() != master_grid.getOriginY())
  {
    nav2_costmap_2d::Costmap2D::updateOrigin(
      master_grid.getOriginX(), master_grid.getOriginY());
    const double residual_x = master_grid.getOriginX() - getOriginX();
    const double residual_y = master_grid.getOriginY() - getOriginY();
    if (std::abs(residual_x) >= 0.5 * getResolution() ||
      std::abs(residual_y) >= 0.5 * getResolution())
    {
      nav2_costmap_2d::Costmap2D::updateOrigin(
        master_grid.getOriginX() + std::copysign(0.5 * getResolution(), residual_x),
        master_grid.getOriginY() + std::copysign(0.5 * getResolution(), residual_y));
    }
    // Both maps share a resolution and cell alignment. Keep the exact master
    // origin after updateOrigin has shifted and cleared the backing cells.
    origin_x_ = master_grid.getOriginX();
    origin_y_ = master_grid.getOriginY();
  }
}

std::string CognitiveObstacleLayer::applicationReason(
  uint32_t active_cells, uint32_t raised_cells)
{
  if (raised_cells > 0U) {
    return "";
  }
  return active_cells > 0U ? "masked" : "no_costmap_cells";
}

void CognitiveObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  synchronizeCostmapGeometry(master_grid);
  resetMap(
    static_cast<unsigned int>(std::max(0, min_i)),
    static_cast<unsigned int>(std::max(0, min_j)),
    static_cast<unsigned int>(std::max(0, max_i)),
    static_cast<unsigned int>(std::max(0, max_j)));
  bio_nav_interfaces::msg::CognitiveObstacleArray::SharedPtr message;
  Identity expected;
  std::string admission_reason;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    message = latest_;
    expected = expected_;
    admission_reason = latest_admission_reason_;
  }
  const auto now = clock_->now();
  const AcceptanceCursor no_ordering_gate;
  const std::string validation_reason = !message ? std::string("missing") :
    (!admission_reason.empty() ? admission_reason : validateMessage(
      *message, now.nanoseconds(), expected, no_ordering_gate, maximum_age_s_,
      maximum_ood_probability_, true));
  if (!modeWritesCostmap(mode_)) {
    return;
  }

  std::vector<AppliedObstacle> applied_obstacles;
  bool tf_failed = false;
  if (message && validation_reason.empty() && !message->obstacles.empty()) {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_->lookupTransform(
        layered_costmap_->getGlobalFrameID(), message->header.frame_id,
        rclcpp::Time(message->validation_stamp), tf2::durationFromSec(0.0));
    } catch (const std::exception &) {
      tf_failed = true;
    }
    if (!tf_failed) {
      struct TransformedObstacle
      {
        StaticTrackKey key;
        bio_nav_interfaces::msg::CognitiveObstacle obstacle;
        double map_x;
        double map_y;
        bool static_revalidated;
      };
      std::map<StaticTrackKey, TransformedObstacle> transformed_batch;
      for (const auto & obstacle : message->obstacles) {
        geometry_msgs::msg::PointStamped source;
        geometry_msgs::msg::PointStamped target;
        source.header = message->header;
        source.header.stamp = message->validation_stamp;
        source.point.x = obstacle.pose_xy_m[0];
        source.point.y = obstacle.pose_xy_m[1];
        tf2::doTransform(source, target, transform);

        if (!std::isfinite(target.point.x) || !std::isfinite(target.point.y)) {
          continue;
        }

        const bool static_revalidated =
          message->validation_mode ==
          bio_nav_interfaces::msg::CognitiveObstacleArray::
          VALIDATION_STATIC_DEPTH_REVALIDATED &&
          obstacle.motion_class ==
          bio_nav_interfaces::msg::CognitiveObstacle::MOTION_STATIC &&
          obstacle.static_confirmed;
        const auto key = staticTrackKey(*message, obstacle.id);
        TransformedObstacle transformed{
          key, obstacle, target.point.x, target.point.y, static_revalidated};
        const auto [it, inserted] = transformed_batch.emplace(key, transformed);
        if (!inserted) {
          // Duplicate IDs are not expected, but choose a canonical observation
          // so input ordering cannot affect tracking or costmap output.
          const auto canonical = [](const TransformedObstacle & value) {
              return std::tie(
                value.map_x, value.map_y, value.obstacle.pose_xy_m[0],
                value.obstacle.pose_xy_m[1], value.obstacle.radius_m,
                value.obstacle.height_m, value.obstacle.confidence,
                value.obstacle.reliability, value.obstacle.ood_probability,
                value.obstacle.position_stddev_m[0],
                value.obstacle.position_stddev_m[1], value.obstacle.count,
                value.obstacle.last_seen.sec, value.obstacle.last_seen.nanosec,
                value.obstacle.motion_class, value.obstacle.static_confirmed,
                value.obstacle.class_id, value.obstacle.id);
            };
          if (canonical(transformed) < canonical(it->second)) {
            it->second = transformed;
          }
        }
      }

      std::lock_guard<std::mutex> lock(mutex_);
      const AcceptanceCursor no_batch_ordering_gate;
      const std::string locked_validation_reason =
        latest_ == message && latest_admission_reason_.empty() ?
        validateMessage(
          *message, clock_->now().nanoseconds(), expected_,
          no_batch_ordering_gate, maximum_age_s_, maximum_ood_probability_, true) :
        std::string("superseded");
      if (locked_validation_reason.empty()) {
        pruneStaticTracks();

        std::set<StaticTrackKey> observed_static_tracks;
        for (const auto & [key, transformed] : transformed_batch) {
          if (transformed.static_revalidated) {
            observed_static_tracks.insert(key);
          }
        }

        const auto pre_batch_tracks = static_tracks_;
        const int64_t matching_now_ns = clock_->now().nanoseconds();
        constexpr int64_t kMaximumReassociationAgeNs = 2000000000LL;
        constexpr double kMaximumReassociationDistanceM = 0.20;

        std::map<StaticTrackKey, std::vector<StaticTrackKey>>
        observed_aliases_by_canonical;
        for (const auto & observed_key : observed_static_tracks) {
          const auto observed = static_tracks_.find(observed_key);
          if (observed == static_tracks_.end() ||
            observed->second.reassociated_to_track_id.empty())
          {
            continue;
          }
          auto canonical_key = observed_key;
          canonical_key.track_id = observed->second.reassociated_to_track_id;
          observed_aliases_by_canonical[canonical_key].push_back(observed_key);
        }
        for (const auto & [canonical_key, observed_aliases] :
          observed_aliases_by_canonical)
        {
          if (observed_aliases.size() < 2U &&
            observed_static_tracks.count(canonical_key) == 0U)
          {
            continue;
          }
          for (const auto & alias_key : observed_aliases) {
            auto alias = static_tracks_.find(alias_key);
            if (alias != static_tracks_.end() &&
              alias->second.reassociated_to_track_id == canonical_key.track_id)
            {
              alias->second.reassociated_to_track_id.clear();
            }
          }
        }

        for (const auto & [key, transformed] : transformed_batch) {
          if (!transformed.static_revalidated) {
            applied_obstacles.push_back(AppliedObstacle{
                transformed.obstacle, transformed.map_x, transformed.map_y,
                transformed.obstacle.count});
            continue;
          }

          auto state = static_tracks_.find(key);
          if (state != static_tracks_.end() &&
            !state->second.reassociated_to_track_id.empty())
          {
            auto canonical_key = key;
            canonical_key.track_id = state->second.reassociated_to_track_id;
            auto canonical = static_tracks_.find(canonical_key);
            const int64_t canonical_age_ns = canonical == static_tracks_.end() ?
              -1 : matching_now_ns - canonical->second.last_refresh_ns;
            const double distance_limit = canonical == static_tracks_.end() ?
              -1.0 : std::min(
              kMaximumReassociationDistanceM,
              canonical->second.radius_m + transformed.obstacle.radius_m);
            const bool canonical_observed =
              observed_static_tracks.count(canonical_key) != 0U;
            const bool alias_valid =
              !canonical_observed && canonical != static_tracks_.end() &&
              canonical->second.promoted &&
              canonical->second.reassociated_to_track_id.empty() &&
              sameStaticIdentity(key, canonical_key) &&
              canonical_age_ns >= 0 &&
              canonical_age_ns <= kMaximumReassociationAgeNs &&
              std::hypot(
              transformed.map_x - canonical->second.anchor_map_x,
              transformed.map_y - canonical->second.anchor_map_y) <= distance_limit;
            if (alias_valid) {
              const int64_t validation_ns = stampNs(message->validation_stamp);
              const bool independent_refresh =
                state->second.last_source_sequence != message->sequence ||
                state->second.last_validation_stamp_ns != validation_ns;
              if (independent_refresh) {
                const int64_t refresh_ns = clock_->now().nanoseconds();
                const uint64_t canonical_rehit_count = canonical->second.rehit_count;
                const bool canonical_promoted = canonical->second.promoted;
                const double canonical_anchor_x = canonical->second.anchor_map_x;
                const double canonical_anchor_y = canonical->second.anchor_map_y;
                const int64_t canonical_first_refresh_ns =
                  canonical->second.first_refresh_ns;

                state->second.obstacle = transformed.obstacle;
                state->second.map_x = transformed.map_x;
                state->second.map_y = transformed.map_y;
                state->second.radius_m = transformed.obstacle.radius_m;
                state->second.height_m = transformed.obstacle.height_m;
                state->second.last_source_sequence = message->sequence;
                state->second.last_validation_stamp_ns = validation_ns;
                state->second.last_refresh_ns = refresh_ns;

                canonical->second = state->second;
                canonical->second.obstacle.id = canonical_key.track_id;
                canonical->second.rehit_count = canonical_rehit_count;
                canonical->second.promoted = canonical_promoted;
                canonical->second.anchor_map_x = canonical_anchor_x;
                canonical->second.anchor_map_y = canonical_anchor_y;
                canonical->second.first_refresh_ns = canonical_first_refresh_ns;
                canonical->second.reassociated_to_track_id.clear();
              }
              continue;
            }
            state->second.reassociated_to_track_id.clear();
          }
          observeStaticTrack(
            *message, transformed.obstacle, transformed.map_x, transformed.map_y);
        }

        struct MatchCandidate
        {
          StaticTrackKey old_key;
          StaticTrackKey new_key;
        };
        std::vector<MatchCandidate> candidates;
        std::map<StaticTrackKey, size_t> old_degrees;
        std::map<StaticTrackKey, size_t> new_degrees;
        for (const auto & [old_key, old_state] : pre_batch_tracks) {
          const int64_t old_age_ns = matching_now_ns - old_state.last_refresh_ns;
          if (!old_state.promoted || !old_state.reassociated_to_track_id.empty() ||
            observed_static_tracks.count(old_key) != 0U ||
            old_age_ns < 0 || old_age_ns > kMaximumReassociationAgeNs)
          {
            continue;
          }
          for (const auto & new_key : observed_static_tracks) {
            const auto previous_new = pre_batch_tracks.find(new_key);
            if (previous_new != pre_batch_tracks.end() && previous_new->second.promoted) {
              continue;
            }
            const auto current_new = static_tracks_.find(new_key);
            if (current_new == static_tracks_.end() || !current_new->second.promoted ||
              !current_new->second.reassociated_to_track_id.empty() ||
              !sameStaticIdentity(old_key, new_key))
            {
              continue;
            }
            const double dx = current_new->second.map_x - old_state.anchor_map_x;
            const double dy = current_new->second.map_y - old_state.anchor_map_y;
            const double distance_limit = std::min(
              kMaximumReassociationDistanceM,
              old_state.radius_m + current_new->second.radius_m);
            if (std::hypot(dx, dy) > distance_limit) {
              continue;
            }
            candidates.push_back(MatchCandidate{old_key, new_key});
            ++old_degrees[old_key];
            ++new_degrees[new_key];
          }
        }

        std::vector<MatchCandidate> selected;
        for (const auto & candidate : candidates) {
          if (old_degrees[candidate.old_key] == 1U &&
            new_degrees[candidate.new_key] == 1U)
          {
            selected.push_back(candidate);
          }
        }

        for (const auto & match : selected) {
          auto old_it = static_tracks_.find(match.old_key);
          auto new_it = static_tracks_.find(match.new_key);
          if (old_it == static_tracks_.end() || new_it == static_tracks_.end()) {
            continue;
          }
          const uint64_t old_rehit_count = old_it->second.rehit_count;
          const bool old_promoted = old_it->second.promoted;
          const double old_anchor_x = old_it->second.anchor_map_x;
          const double old_anchor_y = old_it->second.anchor_map_y;
          const int64_t old_first_refresh_ns = old_it->second.first_refresh_ns;
          old_it->second = new_it->second;
          old_it->second.obstacle.id = match.old_key.track_id;
          old_it->second.rehit_count = old_rehit_count;
          old_it->second.promoted = old_promoted;
          old_it->second.anchor_map_x = old_anchor_x;
          old_it->second.anchor_map_y = old_anchor_y;
          old_it->second.first_refresh_ns = old_first_refresh_ns;
          old_it->second.reassociated_to_track_id.clear();
          new_it->second.reassociated_to_track_id = match.old_key.track_id;
        }

        for (const auto & [key, state] : static_tracks_) {
          (void)key;
          if (state.promoted && state.reassociated_to_track_id.empty()) {
            applied_obstacles.push_back(AppliedObstacle{
                state.obstacle, state.map_x, state.map_y,
                std::max(state.obstacle.count, state.rehit_count)});
          }
        }
        for (const auto & observed_key : observed_static_tracks) {
          const auto state = static_tracks_.find(observed_key);
          if (state != static_tracks_.end() && !state->second.promoted &&
            state->second.reassociated_to_track_id.empty())
          {
            applied_obstacles.push_back(AppliedObstacle{
                state->second.obstacle, state->second.map_x, state->second.map_y,
                std::max(state->second.obstacle.count, state->second.rehit_count)});
          }
        }
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (applied_obstacles.empty()) {
      auto promoted = promotedStaticObstacles();
      applied_obstacles.insert(
        applied_obstacles.end(), promoted.begin(), promoted.end());
    }
    if ((tf_failed || !validation_reason.empty()) && latest_ == message) {
      latest_.reset();
      latest_admission_reason_.clear();
    }
  }

  uint32_t active_cells = 0;
  uint32_t raised_cells = 0;
  uint32_t masked_cells = 0;
  uint8_t maximum_cost = 0;
  uint8_t maximum_cost_increase = 0;
  for (const auto & applied_obstacle : applied_obstacles) {
    const auto & obstacle = applied_obstacle.obstacle;
    unsigned int center_x = 0;
    unsigned int center_y = 0;
    if (!worldToMap(
        applied_obstacle.map_x, applied_obstacle.map_y, center_x, center_y))
    {
      continue;
    }
    const double costmap_diagonal_cells = std::hypot(
      static_cast<double>(getSizeInCellsX()),
      static_cast<double>(getSizeInCellsY()));
    const double bounded_radius_m = std::min(
      obstacle.radius_m, costmap_diagonal_cells * getResolution());
    const double requested_radius_cells = bounded_radius_m / getResolution();
    if (!std::isfinite(requested_radius_cells)) {
      continue;
    }
    const double bounded_radius_cells = std::ceil(std::min(
      requested_radius_cells,
      std::min(
        costmap_diagonal_cells,
        static_cast<double>(std::numeric_limits<int>::max()))));
    const int radius_cells = static_cast<int>(bounded_radius_cells);
    const uint8_t cost = obstacleCost(
      obstacle, applied_obstacle.effective_count, maximum_soft_cost_,
      collision_min_height_m_, collision_max_height_m_);
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        if (static_cast<int64_t>(dx) * dx + static_cast<int64_t>(dy) * dy >
          static_cast<int64_t>(radius_cells) * radius_cells)
        {
          continue;
        }
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
  const std::string reason = applicationReason(active_cells, raised_cells);
  const bool applied = reason.empty();
  if (message) {
    const double age_s = static_cast<double>(
      now.nanoseconds() - stampNs(message->validation_stamp)) * 1.0e-9;
    const std::string offer_reason = tf_failed ? tfFailureReason() : validation_reason;
    publishStatus(
      *message, applied, offer_reason.empty() ? reason : offer_reason, age_s,
      active_cells, maximum_cost, raised_cells, masked_cells,
      maximum_cost_increase);
  }
}

void CognitiveObstacleLayer::publishStatus(
  const bio_nav_interfaces::msg::CognitiveObstacleArray & message,
  bool applied, const std::string & reason, double age_s,
  uint32_t active_cells, uint8_t maximum_cost, uint32_t raised_cells,
  uint32_t masked_cells, uint8_t maximum_cost_increase)
{
  if (!status_publisher_ || !status_publisher_->is_activated()) {return;}
  (void)age_s;
  bio_nav_interfaces::msg::RiskLayerStatus status;
  const auto now = clock_->now();
  status.stamp = now;
  status.consumer = consumer_id_;
  status.mode = mode_;
  status.offered = true;
  status.applied = applied;
  status.rejected = !reason.empty() && reason != "offered" && reason != "shadow";
  status.source_sequence = message.sequence;
  status.recurrent_session_id = message.recurrent_session_id;
  status.risk_model_sha256 = message.risk_model_sha256;
  status.qualification_receipt_sha256 = message.qualification_receipt_sha256;
  status.rejection_mask = message.rejection_mask;
  std::ostringstream detail;
  detail << "validation_mode=" << static_cast<unsigned int>(message.validation_mode)
         << ";source_age_ms="
         << std::setprecision(std::numeric_limits<double>::max_digits10)
         << durationSeconds(message.source_age) * 1000.0
         << ";rejection_reason=" << reason
         << ";confirmed_count=" << std::count_if(
    message.obstacles.begin(), message.obstacles.end(),
    [](const auto & obstacle) {
      return obstacle.motion_class ==
             bio_nav_interfaces::msg::CognitiveObstacle::MOTION_STATIC &&
             obstacle.static_confirmed;
    });
  status.fallback_reason = detail.str();
  status.message_age_ms = static_cast<double>(std::max<int64_t>(
      0, now.nanoseconds() - stampNs(message.validation_stamp))) * 1.0e-6;
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
