#include "bio_nav_fusion/cognitive_risk_critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/create_publisher.hpp"
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
  return static_cast<double>(duration.sec) + duration.nanosec * 1.0e-9;
}

bool unit(double value)
{
  return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

double wrap(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

void CognitiveRiskCritic::initialize()
{
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
  std::lock_guard<std::mutex> lock(mutex_);
  obstacles_ = message;
}

void CognitiveRiskCritic::priorCallback(
  const bio_nav_interfaces::msg::PlanningPrior::SharedPtr message)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    prior_ = message;
  }
  publishStatus(message->sequence, false, mode_ == "shadow" ? "shadow" : "offered");
}

std::string CognitiveRiskCritic::validateInputs(
  const bio_nav_interfaces::msg::CognitiveObstacleArray * obstacles,
  const bio_nav_interfaces::msg::PlanningPrior * prior, int64_t now_ns,
  double maximum_age_s, double maximum_ood_probability)
{
  if (!obstacles || !prior) {return "missing";}
  const double obstacle_ttl = durationSeconds(obstacles->ttl);
  const double obstacle_age = (now_ns - stampNs(obstacles->header.stamp)) * 1.0e-9;
  const double prior_age = (now_ns - stampNs(prior->stamp)) * 1.0e-9;
  const double direction_ttl = durationSeconds(prior->local_direction_ttl);
  if (obstacle_ttl <= 0.0 || obstacle_ttl > 0.5 || direction_ttl <= 0.0 ||
    direction_ttl > 0.5 || obstacle_age < 0.0 || prior_age < 0.0 ||
    obstacle_age > std::min(obstacle_ttl, maximum_age_s) ||
    prior_age > std::min(direction_ttl, maximum_age_s))
  {
    return "stale";
  }
  if (obstacles->schema_version != "bio_nav_cognitive_obstacles_v1" ||
    obstacles->header.frame_id != "base_link" ||
    !obstacles->input_healthy || !obstacles->module2_healthy ||
    !obstacles->trusted_write || obstacles->rejection_mask != 0U ||
    !prior->input_healthy || !prior->module2_healthy || !prior->trusted_write ||
    !prior->context_trusted || prior->trust_rejection_mask != 0U ||
    !prior->local_direction_input_healthy ||
    !prior->local_direction_module2_healthy ||
    !prior->local_direction_trusted_write ||
    prior->local_direction_rejection_mask != 0U)
  {
    return "unhealthy";
  }
  if (obstacles->reset_epoch != prior->reset_epoch ||
    obstacles->recurrent_session_id != prior->recurrent_session_id ||
    obstacles->map_version != prior->map_version ||
    obstacles->cognitive_tile_id != prior->cognitive_tile_id ||
    obstacles->tile_revision != prior->tile_revision ||
    obstacles->graph_revision != prior->graph_revision ||
    obstacles->model_id != prior->model_id)
  {
    return "generation";
  }
  if (!unit(obstacles->reliability) || !unit(obstacles->ood_probability) ||
    !unit(prior->novelty_probability) || !unit(prior->context_uncertainty) ||
    !unit(prior->visual_reliability) || !unit(prior->visual_ood_probability) ||
    obstacles->ood_probability > maximum_ood_probability ||
    prior->visual_ood_probability > maximum_ood_probability)
  {
    return "ood";
  }
  for (double value : prior->local_direction_weights) {
    if (!unit(value)) {return "nonfinite";}
  }
  return "";
}

double CognitiveRiskCritic::trajectoryScore(
  const std::vector<std::array<double, 3>> & trajectory,
  const std::vector<ObstacleSample> & obstacles,
  const std::array<double, 5> & direction_weights,
  double novelty, double uncertainty, double obstacle_weight,
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
  const auto preferred = static_cast<std::size_t>(std::distance(
      direction_weights.begin(),
      std::max_element(direction_weights.begin(), direction_weights.begin() + 4)));
  constexpr std::array<double, 4> headings{0.5 * M_PI, 0.0, -0.5 * M_PI, M_PI};
  double travelled_heading = trajectory.back()[2];
  if (trajectory.size() >= 2) {
    travelled_heading = std::atan2(
      trajectory.back()[1] - trajectory.front()[1],
      trajectory.back()[0] - trajectory.front()[0]);
  }
  const double deviation = std::abs(wrap(travelled_heading - headings[preferred])) / M_PI;
  const double horizon = static_cast<double>(trajectory.size());
  return std::max(0.0,
      obstacle_weight * obstacle_cost + direction_weight * deviation +
      novelty_weight * novelty * horizon +
      uncertainty_weight * uncertainty * horizon);
}

void CognitiveRiskCritic::score(mppi::CriticData & data)
{
  if (!enabled_ || mode_ != "active") {return;}
  bio_nav_interfaces::msg::CognitiveObstacleArray::SharedPtr obstacles;
  bio_nav_interfaces::msg::PlanningPrior::SharedPtr prior;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    obstacles = obstacles_;
    prior = prior_;
  }
  const auto reason = validateInputs(
    obstacles.get(), prior.get(), parent_.lock()->get_clock()->now().nanoseconds(),
    maximum_age_s_, maximum_ood_probability_);
  if (!reason.empty()) {
    publishStatus(prior ? prior->sequence : 0U, false, reason);
    return;
  }
  std::vector<ObstacleSample> samples;
  samples.reserve(obstacles->obstacles.size());
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = costmap_ros_->getTfBuffer()->lookupTransform(
      costmap_ros_->getGlobalFrameID(), obstacles->header.frame_id,
      rclcpp::Time(obstacles->header.stamp), tf2::durationFromSec(0.0));
  } catch (const tf2::TransformException &) {
    publishStatus(prior->sequence, false, "tf");
    return;
  }
  for (const auto & item : obstacles->obstacles) {
    geometry_msgs::msg::PointStamped local;
    geometry_msgs::msg::PointStamped global;
    local.header = obstacles->header;
    local.point.x = item.pose_xy_m[0];
    local.point.y = item.pose_xy_m[1];
    tf2::doTransform(local, global, transform);
    samples.push_back({
      global.point.x, global.point.y, item.radius_m,
      item.confidence * item.reliability * (1.0 - item.ood_probability)});
  }
  std::array<double, 5> direction{};
  std::copy(prior->local_direction_weights.begin(),
    prior->local_direction_weights.end(), direction.begin());
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
        trajectory, samples, direction, prior->novelty_probability,
        prior->context_uncertainty, obstacle_weight_, direction_weight_,
        novelty_weight_, uncertainty_weight_));
  }
  publishStatus(prior->sequence, true, "");
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
  status.rejected = reason != "" && reason != "shadow" && reason != "offered";
  status.source_sequence = sequence;
  status.fallback_reason = reason;
  status_publisher_->publish(status);
}

}  // namespace bio_nav_fusion

PLUGINLIB_EXPORT_CLASS(
  bio_nav_fusion::CognitiveRiskCritic, mppi::critics::CriticFunction)
