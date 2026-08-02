#ifndef BIO_NAV_FUSION__LOCAL_RISK_GRID_LAYER_HPP_
#define BIO_NAV_FUSION__LOCAL_RISK_GRID_LAYER_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "bio_nav_interfaces/msg/local_risk_grid.hpp"
#include "bio_nav_interfaces/msg/risk_layer_status.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace bio_nav_fusion
{

/// Timestamped base_link LocalRiskGrid projector for the Global Costmap.
class LocalRiskGridLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  LocalRiskGridLayer();

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;
  bool isClearable() override {return true;}
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  static std::string validateGrid(
    const bio_nav_interfaces::msg::LocalRiskGrid * grid,
    double age_s, double maximum_age_s, double minimum_reliability,
    double maximum_ood_probability, uint32_t reset_epoch,
    const std::string & expected_map_version,
    const std::string & expected_model_sha256,
    const std::string & expected_qualification_sha256);
  static uint8_t mapRiskCost(
    float probability, float threshold, int maximum_cost);

private:
  void gridCallback(
    const bio_nav_interfaces::msg::LocalRiskGrid::SharedPtr message);
  void resetCallback(const std_msgs::msg::Empty::SharedPtr message);
  void publishStatus(
    bool applied, const std::string & reason, double age_s,
    uint32_t active_cells, uint8_t maximum_cost);
  void publishVisualization(
    bool applied, const std::string & reason,
    const std::vector<geometry_msgs::msg::Point> & points,
    const std::vector<uint8_t> & costs, uint8_t maximum_cost);

  std::mutex mutex_;
  bio_nav_interfaces::msg::LocalRiskGrid::SharedPtr latest_;
  std::array<bool, 1024> active_cells_{};
  uint32_t reset_epoch_{0};
  bool reset_epoch_initialized_{false};
  bool shadow_only_{true};
  double max_message_age_s_{0.5};
  double transform_tolerance_s_{0.05};
  double minimum_reliability_{0.6};
  double maximum_ood_probability_{0.4};
  double activation_threshold_{0.5};
  double clear_threshold_{0.4};
  double minimum_projection_range_m_{0.0};
  int maximum_cost_{80};
  std::string expected_map_version_;
  std::string expected_model_sha256_;
  std::string expected_qualification_sha256_;
  std::string risk_topic_{"/bio_nav/module2/local_risk_grid"};
  std::string reset_topic_{"/simulation/reset_event"};
  rclcpp::Subscription<bio_nav_interfaces::msg::LocalRiskGrid>::SharedPtr
    grid_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<
    bio_nav_interfaces::msg::RiskLayerStatus>::SharedPtr status_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<
    visualization_msgs::msg::MarkerArray>::SharedPtr visualization_publisher_;
  double robot_x_{0.0};
  double robot_y_{0.0};
};

}  // namespace bio_nav_fusion

#endif  // BIO_NAV_FUSION__LOCAL_RISK_GRID_LAYER_HPP_
