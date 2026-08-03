#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include "bio_nav_fusion/local_risk_grid_layer.hpp"
#include "bio_nav_interfaces/msg/local_risk_grid.hpp"
#include "bio_nav_interfaces/msg/risk_layer_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace bio_nav_fusion
{

/// Non-writing LocalRiskGrid validation for Shadow qualification.
///
/// This node deliberately runs outside planner_server.  It validates the same
/// timestamp, identity, health and TF contracts as LocalRiskGridLayer, but it
/// has no Costmap2D reference and therefore cannot alter planning behaviour.
class LocalRiskGridShadowAuditor : public rclcpp::Node
{
public:
  LocalRiskGridShadowAuditor()
  : Node("local_risk_grid_shadow_auditor"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    risk_topic_ = declare_parameter<std::string>(
      "risk_topic", "/bio_nav/module2/local_risk_grid");
    reset_topic_ = declare_parameter<std::string>(
      "reset_topic", "/simulation/reset_event");
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    max_message_age_s_ = declare_parameter<double>("max_message_age_s", 0.5);
    transform_tolerance_s_ = declare_parameter<double>("transform_tolerance_s", 0.05);
    minimum_reliability_ = declare_parameter<double>("minimum_reliability", 0.6);
    maximum_ood_probability_ = declare_parameter<double>("maximum_ood_probability", 0.4);
    activation_threshold_ = std::clamp(
      declare_parameter<double>("activation_threshold", 0.5), 0.01, 0.99);
    clear_threshold_ = std::clamp(
      declare_parameter<double>("clear_threshold", 0.4), 0.0, activation_threshold_);
    maximum_cost_ = static_cast<int>(std::clamp<int64_t>(
      declare_parameter<int64_t>("maximum_cost", 80), 1, 80));
    expected_map_version_ = declare_parameter<std::string>("expected_map_version", "");
    expected_model_sha256_ = declare_parameter<std::string>("expected_model_sha256", "");
    expected_qualification_sha256_ = declare_parameter<std::string>(
      "expected_qualification_sha256", "");
    const auto initial_reset_epoch = std::max<int64_t>(
      0, declare_parameter<int64_t>("initial_reset_epoch", 0));
    reset_epoch_ = static_cast<uint32_t>(initial_reset_epoch);
    reset_epoch_initialized_ = initial_reset_epoch > 0;

    status_publisher_ = create_publisher<bio_nav_interfaces::msg::RiskLayerStatus>(
      "/bio_nav/local_risk_layer/status", rclcpp::QoS(10).reliable());
    grid_subscription_ = create_subscription<bio_nav_interfaces::msg::LocalRiskGrid>(
      risk_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&LocalRiskGridShadowAuditor::gridCallback, this, std::placeholders::_1));
    reset_subscription_ = create_subscription<std_msgs::msg::Empty>(
      reset_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&LocalRiskGridShadowAuditor::resetCallback, this, std::placeholders::_1));
  }

private:
  void gridCallback(const bio_nav_interfaces::msg::LocalRiskGrid::SharedPtr grid)
  {
    if (!reset_epoch_initialized_) {
      reset_epoch_ = grid->reset_epoch;
      reset_epoch_initialized_ = true;
    }
    const double age_s = (get_clock()->now() - rclcpp::Time(grid->header.stamp)).seconds();
    auto reason = LocalRiskGridLayer::validateGrid(
      grid.get(), age_s, max_message_age_s_, minimum_reliability_,
      maximum_ood_probability_, reset_epoch_, expected_map_version_,
      expected_model_sha256_, expected_qualification_sha256_);
    if (reason.empty()) {
      try {
        (void)tf_buffer_.lookupTransform(
          global_frame_, grid->header.frame_id, rclcpp::Time(grid->header.stamp),
          tf2::durationFromSec(transform_tolerance_s_));
      } catch (const tf2::TransformException &) {
        reason = "tf_invalid";
      }
    }

    std::array<bool, 1024> next_active{};
    uint32_t active_count = 0;
    uint8_t maximum_cost = 0;
    if (reason.empty()) {
      for (std::size_t index = 0; index < grid->risk.size(); ++index) {
        const auto threshold = active_cells_[index] ? clear_threshold_ : activation_threshold_;
        const auto probability = grid->risk[index];
        if (grid->visibility[index] == 0U || probability < threshold) {
          continue;
        }
        next_active[index] = true;
        ++active_count;
        maximum_cost = std::max(
          maximum_cost,
          LocalRiskGridLayer::mapRiskCost(
            probability, static_cast<float>(threshold), maximum_cost_));
      }
    }
    active_cells_ = next_active;
    publishStatus(
      reason.empty() ? "shadow_only" : reason, age_s, active_count, maximum_cost, *grid);
  }

  void resetCallback(const std_msgs::msg::Empty::SharedPtr)
  {
    if (reset_epoch_initialized_) {
      ++reset_epoch_;
    }
    active_cells_.fill(false);
  }

  void publishStatus(
    const std::string & reason, double age_s, uint32_t active_count,
    uint8_t maximum_cost, const bio_nav_interfaces::msg::LocalRiskGrid & grid)
  {
    bio_nav_interfaces::msg::RiskLayerStatus status;
    status.stamp = get_clock()->now();
    status.applied = false;
    status.fallback_reason = reason;
    status.message_age_ms = std::isfinite(age_s) ?
      static_cast<float>(age_s * 1000.0) : std::numeric_limits<float>::infinity();
    status.active_cell_count = active_count;
    status.maximum_cost = maximum_cost;
    status.reset_epoch = reset_epoch_;
    status.map_version = expected_map_version_;
    status.risk_model_sha256 = grid.model_sha256;
    status.qualification_receipt_sha256 = grid.qualification_receipt_sha256;
    status_publisher_->publish(status);
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::array<bool, 1024> active_cells_{};
  uint32_t reset_epoch_{0};
  bool reset_epoch_initialized_{false};
  double max_message_age_s_{0.5};
  double transform_tolerance_s_{0.05};
  double minimum_reliability_{0.6};
  double maximum_ood_probability_{0.4};
  double activation_threshold_{0.5};
  double clear_threshold_{0.4};
  int maximum_cost_{80};
  std::string risk_topic_;
  std::string reset_topic_;
  std::string global_frame_;
  std::string expected_map_version_;
  std::string expected_model_sha256_;
  std::string expected_qualification_sha256_;
  rclcpp::Subscription<bio_nav_interfaces::msg::LocalRiskGrid>::SharedPtr
    grid_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_subscription_;
  rclcpp::Publisher<bio_nav_interfaces::msg::RiskLayerStatus>::SharedPtr
    status_publisher_;
};

}  // namespace bio_nav_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bio_nav_fusion::LocalRiskGridShadowAuditor>());
  rclcpp::shutdown();
  return 0;
}
