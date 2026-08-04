#include "bio_nav_fusion/predictive_risk_critic.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "rcutils/sha256.h"
#include "tf2/utils.h"

namespace mppi::critics
{

namespace
{
bool isZeroSha(const std::string & value)
{
  return value.size() == 64U &&
         value.find_first_not_of('0') == std::string::npos;
}

std::string trajectoryDigest(
  const models::Trajectories & trajectories,
  const std::vector<float> & standard_cost,
  const std::vector<uint8_t> & standard_valid)
{
  rcutils_sha256_ctx_t context;
  rcutils_sha256_init(&context);
  const auto update = [&context](const auto * data, std::size_t bytes) {
      rcutils_sha256_update(
        &context, reinterpret_cast<const uint8_t *>(data), bytes);
    };
  update(trajectories.x.data(), trajectories.x.size() * sizeof(float));
  update(trajectories.y.data(), trajectories.y.size() * sizeof(float));
  update(trajectories.yaws.data(), trajectories.yaws.size() * sizeof(float));
  update(standard_cost.data(), standard_cost.size() * sizeof(float));
  update(standard_valid.data(), standard_valid.size());
  std::array<uint8_t, 32> digest{};
  rcutils_sha256_final(&context, digest.data());
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : digest) {
    stream << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return stream.str();
}

std::size_t selectedIndex(const std::vector<float> & cost)
{
  return static_cast<std::size_t>(
    std::distance(cost.begin(), std::min_element(cost.begin(), cost.end())));
}
}  // namespace

void PredictiveRiskCritic::initialize()
{
  auto node = parent_.lock();
  if (!node) {
    throw std::runtime_error("PredictiveRiskCritic lifecycle node expired");
  }
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(shadow_only_, "shadow_only", true);
  getParam(active_authorized_, "active_authorized", false);
  getParam(maximum_age_s_, "maximum_age_s", 0.3);
  getParam(minimum_reliability_, "minimum_reliability", 0.6);
  getParam(maximum_ood_probability_, "maximum_ood_probability", 0.4);
  getParam(risk_weight_, "risk_weight", 0.5F);
  getParam(maximum_standard_cost_, "maximum_standard_cost", 100000.0F);
  getParam(risk_topic_, "risk_topic", risk_topic_);
  getParam(audit_topic_, "audit_topic", audit_topic_);
  getParam(expected_frame_, "expected_frame", expected_frame_);
  getParam(expected_map_version_, "expected_map_version", std::string(""));
  getParam(expected_model_sha256_, "expected_model_sha256", std::string(""));
  getParam(
    expected_calibration_sha256_, "expected_calibration_sha256", std::string(""));
  getParam(
    expected_qualification_sha256_, "expected_qualification_sha256", std::string(""));
  int reset_epoch = 0;
  getParam(reset_epoch, "reset_epoch", 0);
  reset_epoch_ = static_cast<uint32_t>(std::max(0, reset_epoch));
  if (!shadow_only_ && !active_authorized_) {
    throw std::runtime_error(
            "PredictiveRiskCritic active mode requires explicit active_authorized=true");
  }
  if (!shadow_only_ && (
      expected_map_version_.empty() || expected_model_sha256_.empty() ||
      expected_calibration_sha256_.empty() || expected_qualification_sha256_.empty() ||
      isZeroSha(expected_model_sha256_) || isZeroSha(expected_calibration_sha256_) ||
      isZeroSha(expected_qualification_sha256_)))
  {
    throw std::runtime_error(
            "PredictiveRiskCritic active mode requires nonzero frozen identities");
  }
  if (risk_weight_ < 0.0F || maximum_age_s_ <= 0.0 ||
    minimum_reliability_ < 0.0 || minimum_reliability_ > 1.0 ||
    maximum_ood_probability_ < 0.0 || maximum_ood_probability_ > 1.0)
  {
    throw std::runtime_error("PredictiveRiskCritic parameters are invalid");
  }
  risk_subscription_ = node->create_subscription<
    bio_nav_interfaces::msg::PredictiveRiskGrid>(
    risk_topic_, rclcpp::QoS(1).reliable(),
    std::bind(&PredictiveRiskCritic::riskCallback, this, std::placeholders::_1));
  audit_publisher_ = node->create_publisher<
    bio_nav_interfaces::msg::LocalCriticAudit>(
    audit_topic_, rclcpp::QoS(10).reliable());
  RCLCPP_INFO(
    logger_,
    "PredictiveRiskCritic initialized: shadow_only=%s weight=%.3f topic=%s",
    shadow_only_ ? "true" : "false", risk_weight_, risk_topic_.c_str());
}

void PredictiveRiskCritic::riskCallback(
  const bio_nav_interfaces::msg::PredictiveRiskGrid::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = message;
}

std::string PredictiveRiskCritic::validateRisk(
  const bio_nav_interfaces::msg::PredictiveRiskGrid * grid,
  double age_s, double maximum_age_s, double minimum_reliability,
  double maximum_ood_probability, uint32_t reset_epoch,
  const std::string & expected_frame,
  const std::string & expected_map_version,
  const std::string & expected_model_sha256,
  const std::string & expected_calibration_sha256,
  const std::string & expected_qualification_sha256)
{
  if (grid == nullptr) {
    return "no_risk_grid";
  }
  if (!std::isfinite(age_s) || age_s < 0.0 || age_s > maximum_age_s) {
    return "stale";
  }
  if (
    grid->header.frame_id != expected_frame || grid->width != 32U ||
    grid->height != 32U || std::abs(grid->resolution_m - 0.5F) > 1.0e-6F ||
    std::abs(grid->origin_x + 8.0F) > 1.0e-6F ||
    std::abs(grid->origin_y + 8.0F) > 1.0e-6F ||
    std::abs(grid->horizons_s[0] - 0.2F) > 1.0e-6F ||
    std::abs(grid->horizons_s[1] - 0.4F) > 1.0e-6F ||
    std::abs(grid->horizons_s[2] - 0.8F) > 1.0e-6F)
  {
    return "geometry_mismatch";
  }
  if (
    !grid->healthy || grid->rejection_mask != 0U ||
    !std::isfinite(grid->reliability) || !std::isfinite(grid->ood_probability) ||
    grid->reliability < minimum_reliability ||
    grid->ood_probability > maximum_ood_probability)
  {
    return "risk_unhealthy";
  }
  if (grid->reset_epoch != reset_epoch || grid->map_version != expected_map_version) {
    return "map_reset_mismatch";
  }
  if (
    expected_model_sha256.empty() || expected_calibration_sha256.empty() ||
    expected_qualification_sha256.empty() || isZeroSha(expected_model_sha256) ||
    isZeroSha(expected_calibration_sha256) || isZeroSha(expected_qualification_sha256))
  {
    return "identity_unconfigured";
  }
  if (
    grid->model_sha256 != expected_model_sha256 ||
    grid->calibration_sha256 != expected_calibration_sha256 ||
    grid->qualification_sha256 != expected_qualification_sha256)
  {
    return "identity_mismatch";
  }
  for (const auto value : grid->risk) {
    if (!std::isfinite(value) || value < 0.0F || value > 1.0F) {
      return "risk_nonfinite_or_unbounded";
    }
  }
  return "";
}

float PredictiveRiskCritic::sampleRisk(
  const bio_nav_interfaces::msg::PredictiveRiskGrid & grid,
  float local_x, float local_y, float time_s)
{
  const int column = static_cast<int>(
    std::floor((local_x - grid.origin_x) / grid.resolution_m));
  const int row = static_cast<int>(
    std::floor((local_y - grid.origin_y) / grid.resolution_m));
  if (column < 0 || column >= static_cast<int>(grid.width) ||
    row < 0 || row >= static_cast<int>(grid.height))
  {
    return 0.0F;
  }
  const std::size_t cell = static_cast<std::size_t>(row) * grid.width +
    static_cast<std::size_t>(column);
  if (grid.visibility[cell] == 0U) {
    return 0.0F;
  }
  const std::size_t horizon = time_s <= 0.3F ? 0U : (time_s <= 0.6F ? 1U : 2U);
  return grid.risk[horizon * 1024U + cell];
}

void PredictiveRiskCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }
  const auto started = std::chrono::steady_clock::now();
  auto node = parent_.lock();
  if (!node) {
    return;
  }
  bio_nav_interfaces::msg::PredictiveRiskGrid::SharedPtr grid;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    grid = latest_;
  }
  const double age_s = grid ?
    (node->now() - rclcpp::Time(grid->header.stamp)).seconds() :
    std::numeric_limits<double>::infinity();
  const auto reason = validateRisk(
    grid.get(), age_s, maximum_age_s_, minimum_reliability_,
    maximum_ood_probability_, reset_epoch_, expected_frame_,
    expected_map_version_, expected_model_sha256_,
    expected_calibration_sha256_, expected_qualification_sha256_);
  const bool risk_healthy = reason.empty();
  const std::size_t candidates = data.trajectories.x.shape()[0];
  const std::size_t steps = data.trajectories.x.shape()[1];
  if (candidates == 0U || steps == 0U) {
    RCLCPP_WARN(logger_, "PredictiveRiskCritic received an empty candidate batch");
    return;
  }
  std::vector<float> standard(candidates);
  std::vector<float> risk_cost(candidates, 0.0F);
  std::vector<float> combined(candidates);
  std::vector<uint8_t> valid_bytes(candidates, 0U);
  std::vector<bool> valid(candidates, false);
  for (std::size_t candidate = 0; candidate < candidates; ++candidate) {
    standard[candidate] = data.costs(candidate);
    valid[candidate] = std::isfinite(standard[candidate]) &&
      standard[candidate] < maximum_standard_cost_;
    valid_bytes[candidate] = valid[candidate] ? 1U : 0U;
  }
  const double robot_x = data.state.pose.pose.position.x;
  const double robot_y = data.state.pose.pose.position.y;
  const double robot_yaw = tf2::getYaw(data.state.pose.pose.orientation);
  const double cosine = std::cos(robot_yaw);
  const double sine = std::sin(robot_yaw);
  if (risk_healthy) {
    for (std::size_t candidate = 0; candidate < candidates; ++candidate) {
      double sum = 0.0;
      for (std::size_t step = 0; step < steps; ++step) {
        const double dx = data.trajectories.x(candidate, step) - robot_x;
        const double dy = data.trajectories.y(candidate, step) - robot_y;
        const float local_x = static_cast<float>(cosine * dx + sine * dy);
        const float local_y = static_cast<float>(-sine * dx + cosine * dy);
        const float risk = sampleRisk(
          *grid, local_x, local_y,
          static_cast<float>((step + 1U) * data.model_dt));
        sum += static_cast<double>(risk) * static_cast<double>(risk);
      }
      risk_cost[candidate] = risk_weight_ * static_cast<float>(sum) * data.model_dt;
    }
  }
  for (std::size_t candidate = 0; candidate < candidates; ++candidate) {
    combined[candidate] = valid[candidate] ?
      standard[candidate] + risk_cost[candidate] :
      std::numeric_limits<float>::max();
  }
  std::vector<float> masked_standard = standard;
  for (std::size_t candidate = 0; candidate < candidates; ++candidate) {
    if (!valid[candidate]) {
      masked_standard[candidate] = std::numeric_limits<float>::max();
    }
  }
  const std::size_t a11 = selectedIndex(masked_standard);
  const std::size_t a12 = risk_healthy ? selectedIndex(combined) : a11;
  if (!shadow_only_ && risk_healthy) {
    for (std::size_t candidate = 0; candidate < candidates; ++candidate) {
      data.costs(candidate) += risk_cost[candidate];
    }
  }

  bio_nav_interfaces::msg::LocalCriticAudit audit;
  audit.header.stamp = node->now();
  audit.header.frame_id = data.state.pose.header.frame_id;
  audit.sequence = ++sequence_;
  audit.reset_epoch = reset_epoch_;
  audit.map_version = grid ? grid->map_version : expected_map_version_;
  audit.model_sha256 = grid ? grid->model_sha256 : expected_model_sha256_;
  audit.calibration_sha256 = grid ? grid->calibration_sha256 : expected_calibration_sha256_;
  audit.qualification_sha256 = grid ?
    grid->qualification_sha256 : expected_qualification_sha256_;
  audit.candidate_batch_sha256 = trajectoryDigest(
    data.trajectories, standard, valid_bytes);
  audit.candidate_count = static_cast<uint32_t>(candidates);
  audit.trajectory_steps = static_cast<uint32_t>(steps);
  audit.model_dt_s = data.model_dt;
  audit.risk_weight = risk_weight_;
  audit.shadow_only = shadow_only_;
  audit.controller_costs_mutated = !shadow_only_ && risk_healthy;
  audit.risk_healthy = risk_healthy;
  audit.risk_identity_matched = risk_healthy;
  audit.risk_rejection_mask = grid ? grid->rejection_mask : 0U;
  audit.input_age_ms = std::isfinite(age_s) ?
    static_cast<float>(age_s * 1000.0) : std::numeric_limits<float>::infinity();
  audit.fallback_reason = reason;
  audit.standard_cost_by_candidate = standard;
  audit.predictive_risk_cost_by_candidate = risk_cost;
  audit.combined_cost_by_candidate = combined;
  audit.standard_valid_by_candidate = valid;
  audit.a11_selected_candidate = static_cast<uint32_t>(a11);
  audit.a12_selected_candidate = static_cast<uint32_t>(a12);
  audit.selected_trajectory_changed = a11 != a12;
  audit.selected_a12_standard_constraints_valid = valid[a12];
  for (std::size_t step = 0; step < steps; ++step) {
    audit.a11_selected_trajectory_x.push_back(data.trajectories.x(a11, step));
    audit.a11_selected_trajectory_y.push_back(data.trajectories.y(a11, step));
    audit.a11_selected_trajectory_yaw.push_back(data.trajectories.yaws(a11, step));
    audit.a12_selected_trajectory_x.push_back(data.trajectories.x(a12, step));
    audit.a12_selected_trajectory_y.push_back(data.trajectories.y(a12, step));
    audit.a12_selected_trajectory_yaw.push_back(data.trajectories.yaws(a12, step));
  }
  audit.compute_latency_ms = static_cast<float>(
    std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count());
  if (audit_publisher_ && audit_publisher_->is_activated()) {
    audit_publisher_->publish(audit);
  }
}

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PredictiveRiskCritic,
  mppi::critics::CriticFunction)
