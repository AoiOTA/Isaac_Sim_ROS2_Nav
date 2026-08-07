#ifndef BIO_NAV_FUSION__REACHABILITY_OBSERVER_LAYER_HPP_
#define BIO_NAV_FUSION__REACHABILITY_OBSERVER_LAYER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace bio_nav_fusion
{

class ReachabilityObserverLayer : public nav2_costmap_2d::Layer
{
public:
  ReachabilityObserverLayer();

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;
  bool isClearable() override {return false;}
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  static int8_t occupancyValue(uint8_t cost);
  // Attempt-24 (A12-3): low-obstacle accumulation channel.  Each cell keeps
  // an exponentially decayed frequency of lethal observations
  // (``decay * previous + (1 - decay) * occupied``), turning the voxel
  // layer's transient sub-LiDAR markings into a persistent statistic.  The
  // channel is published alongside the observer input and is never written
  // into the costmap itself.
  static float lowObstacleDensityUpdate(float previous, uint8_t cost, double decay);
  static int8_t lowObstacleDensityValue(float density);

private:
  std::string output_topic_{"/global_costmap/reachability_observer_input"};
  std::string low_obstacle_topic_{"/global_costmap/reachability_low_obstacle_density"};
  bool low_obstacle_stats_enabled_{true};
  double low_obstacle_decay_{0.99};
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr low_obstacle_publisher_;
  nav_msgs::msg::OccupancyGrid cached_grid_;
  nav_msgs::msg::OccupancyGrid cached_low_obstacle_grid_;
  std::vector<float> low_obstacle_density_;
  bool cache_initialized_{false};
};

}  // namespace bio_nav_fusion

#endif  // BIO_NAV_FUSION__REACHABILITY_OBSERVER_LAYER_HPP_
