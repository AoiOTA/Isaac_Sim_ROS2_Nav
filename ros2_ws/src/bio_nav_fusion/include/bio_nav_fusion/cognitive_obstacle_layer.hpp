#ifndef BIO_NAV_FUSION__COGNITIVE_OBSTACLE_LAYER_HPP_
#define BIO_NAV_FUSION__COGNITIVE_OBSTACLE_LAYER_HPP_

#include <algorithm>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "bio_nav_interfaces/msg/cognitive_obstacle_array.hpp"
#include "bio_nav_interfaces/msg/risk_layer_status.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace bio_nav_fusion
{

class CognitiveObstacleLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  struct Identity
  {
    uint32_t reset_epoch{0};
    std::string recurrent_session_id;
    std::string map_version;
    std::string cognitive_tile_id;
    uint64_t tile_revision{0};
    uint64_t graph_revision{0};
    std::string model_id;
  };

  CognitiveObstacleLayer();
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

  static std::string validateMessage(
    const bio_nav_interfaces::msg::CognitiveObstacleArray & message,
    int64_t now_ns, const Identity & expected, uint64_t last_sequence,
    double maximum_age_s, double maximum_ood_probability,
    bool enforce_identity = true);
  static uint8_t obstacleCost(
    const bio_nav_interfaces::msg::CognitiveObstacle & obstacle,
    int maximum_soft_cost, double collision_min_height_m,
    double collision_max_height_m);
  static uint8_t mergeCellCost(
    const std::string & mode, uint8_t existing_cost, uint8_t offered_cost)
  {
    return modeWritesCostmap(mode) ?
           std::max(existing_cost, offered_cost) : existing_cost;
  }
  static bool modeWritesCostmap(const std::string & mode)
  {
    return mode == "active";
  }
  static const char * tfFailureReason() {return "tf";}

private:
  void obstacleCallback(
    const bio_nav_interfaces::msg::CognitiveObstacleArray::SharedPtr message);
  void publishStatus(
    const bio_nav_interfaces::msg::CognitiveObstacleArray & message,
    bool applied, const std::string & reason, double age_s,
    uint32_t active_cells = 0, uint8_t maximum_cost = 0,
    uint32_t raised_cells = 0, uint32_t masked_cells = 0,
    uint8_t maximum_cost_increase = 0);

  std::mutex mutex_;
  bio_nav_interfaces::msg::CognitiveObstacleArray::SharedPtr latest_;
  Identity expected_;
  uint64_t last_sequence_{0};
  bool have_sequence_{false};
  bool identity_bound_{false};
  bool identity_parameters_configured_{false};
  std::string mode_{"off"};
  std::string obstacle_topic_{"/bio_nav/module2/cognitive_obstacles"};
  double maximum_age_s_{0.5};
  double maximum_ood_probability_{0.2};
  int maximum_soft_cost_{80};
  double collision_min_height_m_{0.02};
  double collision_max_height_m_{0.45};
  rclcpp::Subscription<
    bio_nav_interfaces::msg::CognitiveObstacleArray>::SharedPtr subscription_;
  rclcpp_lifecycle::LifecyclePublisher<
    bio_nav_interfaces::msg::RiskLayerStatus>::SharedPtr status_publisher_;
};

}  // namespace bio_nav_fusion

#endif  // BIO_NAV_FUSION__COGNITIVE_OBSTACLE_LAYER_HPP_
