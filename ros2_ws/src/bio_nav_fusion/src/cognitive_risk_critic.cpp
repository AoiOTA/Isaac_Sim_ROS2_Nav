#include "bio_nav_fusion/cognitive_risk_critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/create_publisher.hpp"
#include "tf2/time.h"
#include "tf2/utils.h"
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
  return static_cast<double>(duration.sec) + duration.nanosec * 1.0e-9;
}

bool unit(double value)
{
  return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

bool validStamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec >= 0 && stamp.nanosec < 1000000000U && stampNs(stamp) > 0;
}

double wrap(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

CognitiveObstacleLayer::Identity identityOf(
  const bio_nav_interfaces::msg::CognitiveObstacleArray & message)
{
  return CognitiveObstacleLayer::Identity{
    message.reset_epoch, message.recurrent_session_id, message.map_version,
    message.cognitive_tile_id, message.tile_revision, message.graph_revision,
    message.model_id};
}

bool sameStableIdentity(
  const bio_nav_interfaces::msg::CognitiveObstacleArray & message,
  const CognitiveObstacleLayer::Identity & expected)
{
  return message.map_version == expected.map_version &&
         message.cognitive_tile_id == expected.cognitive_tile_id &&
         message.tile_revision == expected.tile_revision &&
         message.graph_revision == expected.graph_revision &&
         message.model_id == expected.model_id;
}

}  // namespace

void CognitiveRiskCritic::initialize()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    obstacles_.reset();
    prior_.reset();
    expected_ = CognitiveObstacleLayer::Identity{};
    accepted_.reset();
    identity_bound_ = false;
  }
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(mode_, "mode", mode_);
  getParam(obstacle_topic_, "obstacle_topic", obstacle_topic_);
  getParam(prior_topic_, "planning_prior_topic", prior_topic_);
  getParam(maximum_age_s_, "maximum_age_s", maximum_age_s_);
  getParam(maximum_ood_probability_, "maximum_ood_probability", maximum_ood_probability_);
  getParam(obstacle_weight_, "obstacle_weight", obstacle_weight_);
  getParam(direction_weight_, "direction_weight", direction_weight_);
  getParam(novelty_weight_, "novelty_weight", novelty_weight_);
  getParam(uncertainty_weight_, "uncertainty_weight", uncertainty_weight_);
  if (mode_ != "off" && mode_ != "shadow" && mode_ != "active") {
    throw std::runtime_error("CognitiveRiskCritic mode must be off, shadow, or active");
  }
  if (!std::isfinite(maximum_age_s_) || maximum_age_s_ <= 0.0 ||
    !unit(maximum_ood_probability_))
  {
    throw std::runtime_error("CognitiveRiskCritic age/OOD gates are invalid");
  }
  auto node = parent_.lock();
  auto parameters = node->get_node_parameters_interface();
  auto topics = node->get_node_topics_interface();
  status_publisher_ = rclcpp::create_publisher<
    bio_nav_interfaces::msg::RiskLayerStatus>(
    parameters, topics, "/bio_nav/cognitive_risk_critic/status",
    rclcpp::QoS(10).reliable());
  if (enabled_ && mode_ != "off") {
    obstacle_subscription_ = node->create_subscription<
      bio_nav_interfaces::msg::CognitiveObstacleArray>(
      obstacle_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&CognitiveRiskCritic::obstacleCallback, this, std::placeholders::_1));
    prior_subscription_ = node->create_subscription<bio_nav_interfaces::msg::PlanningPrior>(
      prior_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&CognitiveRiskCritic::priorCallback, this, std::placeholders::_1));
  }
}

void CognitiveRiskCritic::obstacleCallback(
  const bio_nav_interfaces::msg::CognitiveObstacleArray::SharedPtr message)
{
  const auto now_ns = parent_.lock()->get_clock()->now().nanoseconds();
  std::lock_guard<std::mutex> lock(mutex_);
  const bool reset_rebind_candidate = identity_bound_ &&
    message->reset_epoch > expected_.reset_epoch &&
    message->recurrent_session_id != expected_.recurrent_session_id &&
    sameStableIdentity(*message, expected_);
  const auto candidate_identity = identityOf(*message);
  std::string reason;
  if (reset_rebind_candidate) {
    reason = CognitiveObstacleLayer::validateMessage(
      *message, now_ns, candidate_identity,
      CognitiveObstacleLayer::AcceptanceCursor{}, maximum_age_s_,
      maximum_ood_probability_, true);
  } else {
    reason = CognitiveObstacleLayer::validateMessage(
      *message, now_ns, expected_, accepted_, maximum_age_s_,
      maximum_ood_probability_, identity_bound_);
  }
  if (!reason.empty()) {
    return;
  }
  if (reset_rebind_candidate) {
    obstacles_.reset();
    accepted_.reset();
    expected_ = candidate_identity;
  } else if (!identity_bound_) {
    expected_ = candidate_identity;
  }
  identity_bound_ = true;
  obstacles_ = message;
  CognitiveObstacleLayer::recordAccepted(*message, accepted_);
}

void CognitiveRiskCritic::priorCallback(
  const bio_nav_interfaces::msg::PlanningPrior::SharedPtr message)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    prior_ = message;
  }
  if (mode_ == "shadow") {
    publishStatus(message->sequence, false, "shadow");
  }
}

std::string CognitiveRiskCritic::validateInputs(
  const bio_nav_interfaces::msg::CognitiveObstacleArray * obstacles,
  const bio_nav_interfaces::msg::PlanningPrior * prior, int64_t now_ns,
  double maximum_age_s, double maximum_ood_probability)
{
  if (!obstacles || !prior) {return "missing";}
  const CognitiveObstacleLayer::Identity obstacle_identity{
    prior->reset_epoch, prior->recurrent_session_id, prior->map_version,
    prior->cognitive_tile_id, prior->tile_revision, prior->graph_revision,
    prior->model_id};
  return validateInputs(
    obstacles, prior, now_ns, obstacle_identity,
    CognitiveObstacleLayer::AcceptanceCursor{}, true, maximum_age_s,
    maximum_ood_probability);
}

std::string CognitiveRiskCritic::validateInputs(
  const bio_nav_interfaces::msg::CognitiveObstacleArray * obstacles,
  const bio_nav_interfaces::msg::PlanningPrior * prior, int64_t now_ns,
  const CognitiveObstacleLayer::Identity & expected,
  const CognitiveObstacleLayer::AcceptanceCursor & accepted,
  bool enforce_identity, double maximum_age_s,
  double maximum_ood_probability)
{
  if (!obstacles || !prior) {return "missing";}
  const auto obstacle_reason = CognitiveObstacleLayer::validateMessage(
    *obstacles, now_ns, expected, accepted, maximum_age_s,
    maximum_ood_probability, enforce_identity);
  if (!obstacle_reason.empty()) {return obstacle_reason;}
  return validatePriorComponents(
    obstacles, prior, now_ns, maximum_age_s, maximum_ood_probability);
}

std::string CognitiveRiskCritic::validatePriorComponents(
  const bio_nav_interfaces::msg::CognitiveObstacleArray * obstacles,
  const bio_nav_interfaces::msg::PlanningPrior * prior, int64_t now_ns,
  double maximum_age_s, double maximum_ood_probability)
{
  if (!obstacles) {return "obstacle_missing";}
  if (!prior) {return "prior_missing";}
  if (!validStamp(prior->stamp)) {return "prior_time";}
  const double prior_age = (now_ns - stampNs(prior->stamp)) * 1.0e-9;
  if (!std::isfinite(prior_age) || prior_age < 0.0 || prior_age > maximum_age_s) {
    return "prior_stale";
  }
  if ((prior->schema_version != "bio_nav_planning_prior_v4" &&
    prior->schema_version != "bio_nav_planning_prior_v310") ||
    !prior->input_healthy || !prior->module2_healthy ||
    !prior->observation_valid || !prior->trusted_write ||
    prior->trust_rejection_mask != 0U)
  {
    return "prior_untrusted";
  }
  if (obstacles->sequence == 0U || prior->sequence == 0U ||
    obstacles->sequence != prior->sequence)
  {
    return "prior_sequence";
  }
  if (obstacles->reset_epoch != prior->reset_epoch ||
    obstacles->recurrent_session_id != prior->recurrent_session_id ||
    obstacles->map_version != prior->map_version ||
    obstacles->cognitive_tile_id != prior->cognitive_tile_id ||
    obstacles->tile_revision != prior->tile_revision ||
    obstacles->graph_revision != prior->graph_revision ||
    obstacles->model_id != prior->model_id)
  {
    return "prior_identity";
  }
  if (!unit(prior->novelty_probability) || !unit(prior->context_uncertainty) ||
    !unit(prior->visual_reliability) || !unit(prior->visual_ood_probability))
  {
    return "prior_nonfinite";
  }
  if (prior->visual_ood_probability > maximum_ood_probability)
  {
    return "prior_ood";
  }
  return "";
}

std::string CognitiveRiskCritic::validateDirectionPrior(
  const bio_nav_interfaces::msg::PlanningPrior & prior, double prior_age_s)
{
  const double direction_ttl = durationSeconds(prior.local_direction_ttl);
  if (direction_ttl <= 0.0 || direction_ttl > 0.5 || prior_age_s < 0.0 ||
    prior_age_s > direction_ttl)
  {
    return "direction_stale";
  }
  if (prior.local_direction_schema_version != "bio_nav_local_direction_prior_v1") {
    return "direction_schema";
  }
  if (prior.local_direction_frame_id != "base_link") {return "direction_frame";}
  if (prior.local_direction_source_sequence == 0U ||
    prior.local_direction_source_sequence != prior.sequence)
  {
    return "direction_sequence";
  }
  if (!prior.local_direction_input_healthy ||
    !prior.local_direction_module2_healthy ||
    !prior.local_direction_trusted_write ||
    prior.local_direction_rejection_mask != 0U)
  {
    return "direction_untrusted";
  }
  double total = 0.0;
  for (double value : prior.local_direction_weights) {
    if (!unit(value)) {return "direction_nonfinite";}
    total += value;
  }
  if (!std::isfinite(total) || std::abs(total - 1.0) > 1.0e-3) {
    return "direction_normalization";
  }
  return "";
}

double CognitiveRiskCritic::trajectoryScore(
  const std::vector<std::array<double, 3>> & trajectory,
  const std::vector<ObstacleSample> & obstacles,
  const std::array<double, 5> & direction_weights,
  double robot_yaw, double novelty, double uncertainty, double obstacle_weight,
  double direction_weight, double novelty_weight, double uncertainty_weight)
{
  if (trajectory.empty()) {return 0.0;}
  double obstacle_cost = 0.0;
  for (const auto & pose : trajectory) {
    for (const auto & obstacle : obstacles) {
      const double clearance = std::hypot(pose[0] - obstacle.x, pose[1] - obstacle.y) -
        obstacle.radius;
      obstacle_cost += obstacle.confidence *
        (clearance <= 0.0 ? 2.0 : std::exp(-clearance / 0.35));
    }
  }
  double travelled_heading = trajectory.back()[2];
  if (trajectory.size() >= 2) {
    travelled_heading = std::atan2(
      trajectory.back()[1] - trajectory.front()[1],
      trajectory.back()[0] - trajectory.front()[0]);
  }
  const double direction_x = direction_weights[1] - direction_weights[3];
  const double direction_y = direction_weights[0] - direction_weights[2];
  const double direction_vector = std::hypot(direction_x, direction_y);
  const double total_weight = std::accumulate(
    direction_weights.begin(), direction_weights.end(), 0.0);
  double direction_cost = 0.0;
  if (direction_vector > 1.0e-9 && total_weight > 1.0e-9) {
    const double preferred_heading = robot_yaw + std::atan2(direction_y, direction_x);
    const double deviation =
      std::abs(wrap(travelled_heading - preferred_heading)) / M_PI;
    direction_cost = deviation * std::min(1.0, direction_vector / total_weight);
  }
  const double horizon = static_cast<double>(trajectory.size());
  return std::max(0.0,
      obstacle_weight * obstacle_cost + direction_weight * direction_cost +
      novelty_weight * novelty * horizon +
      uncertainty_weight * uncertainty * horizon);
}

void CognitiveRiskCritic::score(mppi::CriticData & data)
{
  if (!enabled_ || mode_ != "active") {return;}
  bio_nav_interfaces::msg::CognitiveObstacleArray::SharedPtr obstacles;
  bio_nav_interfaces::msg::PlanningPrior::SharedPtr prior;
  CognitiveObstacleLayer::Identity expected;
  std::string reason;
  const auto now = parent_.lock()->get_clock()->now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    obstacles = obstacles_;
    prior = prior_;
    expected = expected_;
  }
  const CognitiveObstacleLayer::AcceptanceCursor no_ordering_gate;
  reason = obstacles ? CognitiveObstacleLayer::validateMessage(
    *obstacles, now.nanoseconds(), expected, no_ordering_gate,
    maximum_age_s_, maximum_ood_probability_, true) : "obstacle_missing";
  if (!reason.empty()) {
    publishStatus(
      obstacles ? obstacles->sequence : 0U, false,
      "obstacle_rejected=" + reason);
    return;
  }
  std::vector<ObstacleSample> samples;
  samples.reserve(obstacles->obstacles.size());
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = costmap_ros_->getTfBuffer()->lookupTransform(
      costmap_ros_->getGlobalFrameID(), obstacles->header.frame_id,
      rclcpp::Time(obstacles->validation_stamp), tf2::durationFromSec(0.0));
  } catch (const tf2::TransformException &) {
    publishStatus(obstacles->sequence, false, "obstacle_rejected=tf");
    return;
  }
  for (const auto & item : obstacles->obstacles) {
    geometry_msgs::msg::PointStamped local;
    geometry_msgs::msg::PointStamped global;
    local.header = obstacles->header;
    local.header.stamp = obstacles->validation_stamp;
    local.point.x = item.pose_xy_m[0];
    local.point.y = item.pose_xy_m[1];
    tf2::doTransform(local, global, transform);
    samples.push_back({
      global.point.x, global.point.y, item.radius_m,
      item.confidence * item.reliability * (1.0 - item.ood_probability)});
  }
  std::array<double, 5> direction{};
  const auto prior_reason = validatePriorComponents(
    obstacles.get(), prior.get(), now.nanoseconds(), maximum_age_s_,
    maximum_ood_probability_);
  std::string context_reason;
  std::string direction_reason;
  double novelty = 0.0;
  double uncertainty = 0.0;
  if (prior_reason.empty()) {
    context_reason = prior->context_trusted ? "" : "context_untrusted";
    if (context_reason.empty()) {
      novelty = prior->novelty_probability;
      uncertainty = prior->context_uncertainty;
    }
    const double prior_age_s =
      (now.nanoseconds() - stampNs(prior->stamp)) * 1.0e-9;
    direction_reason = validateDirectionPrior(*prior, prior_age_s);
  }
  if (prior_reason.empty() && direction_reason.empty()) {
    std::copy(prior->local_direction_weights.begin(),
      prior->local_direction_weights.end(), direction.begin());
  }
  const double robot_yaw = tf2::getYaw(transform.transform.rotation);
  const auto batch = data.trajectories.x.shape(0);
  const auto steps = data.trajectories.x.shape(1);
  for (std::size_t index = 0; index < batch; ++index) {
    std::vector<std::array<double, 3>> trajectory;
    trajectory.reserve(steps);
    for (std::size_t step = 0; step < steps; ++step) {
      trajectory.push_back({
        data.trajectories.x(index, step), data.trajectories.y(index, step),
        data.trajectories.yaws(index, step)});
    }
    data.costs(index) += static_cast<float>(trajectoryScore(
        trajectory, samples, direction, robot_yaw,
        novelty, uncertainty,
        obstacle_weight_, direction_weight_,
        novelty_weight_, uncertainty_weight_));
  }
  publishStatus(
    obstacles->sequence, true,
    appliedStatus(prior_reason, context_reason, direction_reason));
}

std::string CognitiveRiskCritic::appliedStatus(
  const std::string & prior_reason, const std::string & context_reason,
  const std::string & direction_reason)
{
  std::string status = "obstacle_applied=true";
  if (!prior_reason.empty()) {
    return status + ";prior_suppressed=" + prior_reason +
           ";context_suppressed=" + prior_reason +
           ";novelty_suppressed=" + prior_reason +
           ";uncertainty_suppressed=" + prior_reason +
           ";direction_suppressed=" + prior_reason;
  }
  status += ";prior_accepted=true";
  if (context_reason.empty()) {
    status += ";context_applied=true;novelty_applied=true;uncertainty_applied=true";
  } else {
    status += ";context_suppressed=" + context_reason +
      ";novelty_suppressed=" + context_reason +
      ";uncertainty_suppressed=" + context_reason;
  }
  status += direction_reason.empty() ?
    ";direction_applied=true" : ";direction_suppressed=" + direction_reason;
  return status;
}

void CognitiveRiskCritic::publishStatus(
  uint64_t sequence, bool applied, const std::string & reason)
{
  if (!status_publisher_) {return;}
  bio_nav_interfaces::msg::RiskLayerStatus status;
  status.stamp = parent_.lock()->get_clock()->now();
  status.consumer = name_;
  status.mode = mode_;
  status.offered = sequence > 0;
  status.applied = applied;
  status.rejected = !applied && reason != "" && reason != "shadow" &&
    reason != "offered";
  status.source_sequence = sequence;
  status.fallback_reason = reason;
  status_publisher_->publish(status);
}

}  // namespace bio_nav_fusion

PLUGINLIB_EXPORT_CLASS(
  bio_nav_fusion::CognitiveRiskCritic, mppi::critics::CriticFunction)
