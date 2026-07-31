#include "bio_nav_fusion/cognitive_risk_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace bio_nav_fusion
{

CognitiveRiskLayer::CognitiveRiskLayer()
{
  enabled_ = true;
  current_ = true;
}

void CognitiveRiskLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("CognitiveRiskLayer lifecycle node expired");
  }
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".prior_topic", rclcpp::ParameterValue(prior_topic_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".reset_topic", rclcpp::ParameterValue(reset_topic_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_message_age_s", rclcpp::ParameterValue(max_message_age_s_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".minimum_reliability", rclcpp::ParameterValue(minimum_reliability_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".maximum_cost", rclcpp::ParameterValue(maximum_cost_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".expected_map_version", rclcpp::ParameterValue(expected_map_version_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".expected_risk_model_sha256",
    rclcpp::ParameterValue(std::string("")));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".expected_qualification_sha256",
    rclcpp::ParameterValue(std::string("")));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".initial_reset_epoch", rclcpp::ParameterValue(0));
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".prior_topic", prior_topic_);
  node->get_parameter(name_ + ".reset_topic", reset_topic_);
  node->get_parameter(name_ + ".max_message_age_s", max_message_age_s_);
  node->get_parameter(name_ + ".minimum_reliability", minimum_reliability_);
  node->get_parameter(name_ + ".maximum_cost", maximum_cost_);
  node->get_parameter(name_ + ".expected_map_version", expected_map_version_);
  node->get_parameter(name_ + ".expected_risk_model_sha256", expected_risk_model_sha256_);
  node->get_parameter(
    name_ + ".expected_qualification_sha256", expected_qualification_sha256_);
  int initial_reset_epoch = 0;
  node->get_parameter(name_ + ".initial_reset_epoch", initial_reset_epoch);
  reset_epoch_ = static_cast<uint32_t>(std::max(0, initial_reset_epoch));
  reset_epoch_initialized_ = initial_reset_epoch > 0;
  maximum_cost_ = std::clamp(maximum_cost_, 1, 80);

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  prior_subscription_ = node->create_subscription<
    bio_nav_interfaces::msg::PlanningPrior>(
    prior_topic_, rclcpp::QoS(1).reliable(),
    std::bind(&CognitiveRiskLayer::priorCallback, this, std::placeholders::_1),
    options);
  reset_subscription_ = node->create_subscription<std_msgs::msg::Empty>(
    reset_topic_, rclcpp::QoS(10).reliable(),
    std::bind(&CognitiveRiskLayer::resetCallback, this, std::placeholders::_1),
    options);
  status_publisher_ = node->create_publisher<
    bio_nav_interfaces::msg::RiskLayerStatus>(
    "/bio_nav/risk_layer/status", rclcpp::QoS(10).reliable());
  matchSize();
  resetMaps();
  current_ = true;
}

void CognitiveRiskLayer::activate()
{
  if (status_publisher_) {
    status_publisher_->on_activate();
  }
}

void CognitiveRiskLayer::deactivate()
{
  if (status_publisher_) {
    status_publisher_->on_deactivate();
  }
}

void CognitiveRiskLayer::reset()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_.reset();
  }
  resetMaps();
  addExtraBounds(-8.0, -8.0, 8.0, 8.0);
  current_ = true;
}

void CognitiveRiskLayer::priorCallback(
  const bio_nav_interfaces::msg::PlanningPrior::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // The Integration bridge may start before Nav2 and therefore observe an
  // earlier simulator reset that this plugin could not see. Latch the first
  // fresh prior's absolute epoch, then advance both sides from the shared
  // /simulation/reset_event stream.
  if (!reset_epoch_initialized_) {
    reset_epoch_ = message->reset_epoch;
    reset_epoch_initialized_ = true;
  }
  latest_ = message;
  addExtraBounds(-8.0, -8.0, 8.0, 8.0);
}

void CognitiveRiskLayer::resetCallback(const std_msgs::msg::Empty::SharedPtr)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (reset_epoch_initialized_) {
      ++reset_epoch_;
    }
    latest_.reset();
  }
  addExtraBounds(-8.0, -8.0, 8.0, 8.0);
}

bool CognitiveRiskLayer::validateLocked(
  const rclcpp::Time & now, std::string & reason, double & age_s) const
{
  age_s = std::numeric_limits<double>::infinity();
  if (!enabled_) {
    reason = "disabled";
    return false;
  }
  if (!latest_) {
    reason = "no_prior";
    return false;
  }
  age_s = (now - rclcpp::Time(latest_->stamp)).seconds();
  reason = validatePrior(
    latest_.get(), age_s, max_message_age_s_, minimum_reliability_,
    reset_epoch_, expected_map_version_, expected_risk_model_sha256_,
    expected_qualification_sha256_);
  return reason.empty();
}

std::string CognitiveRiskLayer::validatePrior(
  const bio_nav_interfaces::msg::PlanningPrior * prior,
  double age_s, double maximum_age_s, double minimum_reliability,
  uint32_t reset_epoch, const std::string & expected_map_version,
  const std::string & expected_risk_model_sha256,
  const std::string & expected_qualification_sha256)
{
  if (prior == nullptr) {
    return "no_prior";
  }
  if (!std::isfinite(age_s) || age_s < 0.0 || age_s > maximum_age_s) {
    return "stale";
  }
  if (
    prior->schema_version != "bio_nav_planning_prior_v4" ||
    !prior->risk_healthy ||
    !std::isfinite(prior->risk_reliability) ||
    prior->risk_reliability < minimum_reliability)
  {
    return "risk_unhealthy";
  }
  if (
    prior->map_version != expected_map_version ||
    prior->reset_epoch != reset_epoch)
  {
    return "map_reset_mismatch";
  }
  if (
    (!expected_risk_model_sha256.empty() &&
    prior->risk_model_sha256 != expected_risk_model_sha256) ||
    (!expected_qualification_sha256.empty() &&
    prior->qualification_receipt_sha256 != expected_qualification_sha256))
  {
    return "model_hash_mismatch";
  }
  if (
    !std::isfinite(prior->risk_threshold) ||
    !std::isfinite(prior->risk_ttl_s) ||
    prior->risk_threshold < 0.0F || prior->risk_threshold >= 1.0F ||
    prior->risk_ttl_s <= 0.0F)
  {
    return "invalid_calibration";
  }
  for (const float value : prior->dynamic_cost) {
    if (!std::isfinite(value)) {
      return "nonfinite";
    }
  }
  return "";
}

void CognitiveRiskLayer::updateBounds(
  double, double, double, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  useExtraBounds(min_x, min_y, max_x, max_y);
  std::lock_guard<std::mutex> lock(mutex_);
  if (latest_) {
    touch(-8.0, -8.0, min_x, min_y, max_x, max_y);
    touch(8.0, 8.0, min_x, min_y, max_x, max_y);
  }
  current_ = true;
}

void CognitiveRiskLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  // A CostmapLayer must own the cells it contributes.  Writing cognitive
  // costs directly into master_grid makes a transient prior indistinguishable
  // from costs produced by earlier plugins and can leave it behind after the
  // prior expires.  Rebuild this layer's bounded window every cycle and merge
  // it with Nav2's standard max-combination path instead.
  resetMap(
    static_cast<unsigned int>(std::max(0, min_i)),
    static_cast<unsigned int>(std::max(0, min_j)),
    static_cast<unsigned int>(std::max(0, max_i)),
    static_cast<unsigned int>(std::max(0, max_j)));

  bio_nav_interfaces::msg::PlanningPrior::SharedPtr prior;
  std::string reason;
  double age_s = 0.0;
  bool valid = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    valid = validateLocked(clock_->now(), reason, age_s);
    if (valid) {
      prior = latest_;
    }
  }
  if (!valid) {
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    publishStatus(false, reason, age_s, 0, 0);
    return;
  }
  const double ttl_s = std::min(0.8, static_cast<double>(prior->risk_ttl_s));
  const double decay = std::clamp(1.0 - age_s / ttl_s, 0.0, 1.0);
  uint32_t active_cells = 0;
  uint8_t maximum_written = 0;
  for (int my = min_j; my < max_j; ++my) {
    for (int mx = min_i; mx < max_i; ++mx) {
      double wx = 0.0;
      double wy = 0.0;
      master_grid.mapToWorld(
        static_cast<unsigned int>(mx), static_cast<unsigned int>(my), wx, wy);
      const int column = static_cast<int>(std::floor(wx + 8.0));
      const int row = static_cast<int>(std::floor(wy + 8.0));
      if (column < 0 || column >= 16 || row < 0 || row >= 16) {
        continue;
      }
      const float probability = prior->dynamic_cost[row * 16 + column];
      const auto cost = mapRiskCost(
        probability, prior->risk_threshold, decay, maximum_cost_);
      if (cost == 0) {
        continue;
      }
      setCost(
        static_cast<unsigned int>(mx), static_cast<unsigned int>(my), cost);
      ++active_cells;
      maximum_written = std::max(maximum_written, cost);
    }
  }
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  publishStatus(true, "", age_s, active_cells, maximum_written);
}

uint8_t CognitiveRiskLayer::mapRiskCost(
  float probability, float threshold, double decay, int maximum_cost)
{
  if (
    !std::isfinite(probability) || !std::isfinite(threshold) ||
    !std::isfinite(decay) || threshold < 0.0F || threshold >= 1.0F ||
    probability < threshold || decay <= 0.0 || maximum_cost <= 0)
  {
    return 0;
  }
  const double normalized = std::clamp(
    (static_cast<double>(probability) - threshold) / (1.0 - threshold),
    0.0, 1.0);
  return static_cast<uint8_t>(std::clamp(
      static_cast<int>(std::lround(
        std::clamp(decay, 0.0, 1.0) *
        (1.0 + normalized * (maximum_cost - 1)))),
      1, std::min(80, maximum_cost)));
}

void CognitiveRiskLayer::publishStatus(
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
      status.risk_model_sha256 = latest_->risk_model_sha256;
      status.qualification_receipt_sha256 =
        latest_->qualification_receipt_sha256;
    }
  }
  status_publisher_->publish(status);
}

}  // namespace bio_nav_fusion

PLUGINLIB_EXPORT_CLASS(
  bio_nav_fusion::CognitiveRiskLayer, nav2_costmap_2d::Layer)
