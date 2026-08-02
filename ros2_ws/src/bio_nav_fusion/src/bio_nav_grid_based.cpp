#include "bio_nav_fusion/bio_nav_grid_based.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <queue>
#include <stdexcept>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "bio_nav_fusion/tie_break_smac_planner_2d.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bio_nav_fusion
{

namespace
{

struct OpenEntry
{
  uint64_t index;
  uint64_t f;
  uint32_t tie;
  uint64_t serial;
  uint64_t g;
};

struct OpenWorse
{
  bool operator()(const OpenEntry & left, const OpenEntry & right) const
  {
    if (left.f != right.f) {
      return left.f > right.f;
    }
    if (left.tie != right.tie) {
      return left.tie < right.tie;
    }
    return left.serial > right.serial;
  }
};

uint64_t octileHeuristic(int x0, int y0, int x1, int y1)
{
  const auto dx = static_cast<uint64_t>(std::abs(x1 - x0));
  const auto dy = static_cast<uint64_t>(std::abs(y1 - y0));
  const auto diagonal = std::min(dx, dy);
  return 1414 * diagonal + 1000 * (std::max(dx, dy) - diagonal);
}

bool traversable(nav2_costmap_2d::Costmap2D & costmap, int x, int y, bool allow_unknown)
{
  if (
    x < 0 || y < 0 ||
    x >= static_cast<int>(costmap.getSizeInCellsX()) ||
    y >= static_cast<int>(costmap.getSizeInCellsY()))
  {
    return false;
  }
  const auto cost = costmap.getCost(x, y);
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown;
  }
  return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

uint32_t tieScore(
  nav2_costmap_2d::Costmap2D & costmap, uint64_t index,
  const std::array<float, 256> & score)
{
  unsigned int mx = 0;
  unsigned int my = 0;
  costmap.indexToCells(static_cast<unsigned int>(index), mx, my);
  double wx = 0.0;
  double wy = 0.0;
  costmap.mapToWorld(mx, my, wx, wy);
  const int column = static_cast<int>(std::floor(wx + 8.0));
  const int row = static_cast<int>(std::floor(wy + 8.0));
  if (column < 0 || column >= 16 || row < 0 || row >= 16) {
    return 0;
  }
  const float value = score[row * 16 + column];
  if (!std::isfinite(value)) {
    return 0;
  }
  return static_cast<uint32_t>(
    std::lround(std::clamp(value, 0.0F, 1.0F) * 1000000.0F));
}

}  // namespace

BioNavGridBased::~BioNavGridBased()
{
  cleanup();
}

void BioNavGridBased::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  name_ = std::move(name);
  costmap_ros_ = std::move(costmap_ros);
  global_frame_ = costmap_ros_->getGlobalFrameID();
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("BioNavGridBased lifecycle node expired");
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".prior_service", rclcpp::ParameterValue(service_name_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".planning_prior_topic", rclcpp::ParameterValue(prior_topic_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".service_timeout_ms", rclcpp::ParameterValue(service_timeout_ms_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".maximum_prior_age_s",
    rclcpp::ParameterValue(maximum_prior_age_s_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(allow_unknown_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".planner_profile",
    rclcpp::ParameterValue(planner_profile_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".expected_module3_map_sha256",
    rclcpp::ParameterValue(std::string("")));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".expected_qualification_sha256",
    rclcpp::ParameterValue(std::string("")));
  node->get_parameter(name_ + ".prior_service", service_name_);
  node->get_parameter(name_ + ".planning_prior_topic", prior_topic_);
  node->get_parameter(name_ + ".service_timeout_ms", service_timeout_ms_);
  node->get_parameter(name_ + ".maximum_prior_age_s", maximum_prior_age_s_);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(name_ + ".planner_profile", planner_profile_);
  node->get_parameter(
    name_ + ".expected_module3_map_sha256", expected_module3_map_sha256_);
  node->get_parameter(
    name_ + ".expected_qualification_sha256", expected_qualification_sha256_);
  service_timeout_ms_ = std::clamp(service_timeout_ms_, 1, 500);
  maximum_prior_age_s_ = std::clamp(maximum_prior_age_s_, 0.05, 5.0);
  decision_publisher_ = node->create_publisher<
    bio_nav_interfaces::msg::PlannerDecision>(
    "/bio_nav/planner/decision", rclcpp::QoS(10).reliable());
  visualization_publisher_ = node->create_publisher<
    visualization_msgs::msg::MarkerArray>(
    "/bio_nav/planner/rviz_markers", rclcpp::QoS(1).reliable());

  stock_ = std::make_unique<TieBreakSmacPlanner2D>();
  stock_->configure(parent, name_ + "_fallback", tf, costmap_ros_);

  // The planner server process is launched with the global
  // `__node:=planner_server` remap. A helper node that accepts global
  // arguments would inherit that name and trip the duplicate-node gate.
  auto client_options = rclcpp::NodeOptions().use_global_arguments(false);
  client_node_ = std::make_shared<rclcpp::Node>(
    "bio_nav_goal_prior_client", client_options);
  prior_client_ = client_node_->create_client<
    bio_nav_interfaces::srv::GetGoalPlanningPrior>(service_name_);
  identity_subscription_ = client_node_->create_subscription<
    bio_nav_interfaces::msg::PlanningPrior>(
    prior_topic_, rclcpp::QoS(1).reliable(),
    [this](const bio_nav_interfaces::msg::PlanningPrior::SharedPtr message) {
      std::lock_guard<std::mutex> lock(identity_mutex_);
      identity_seen_ = true;
      identity_stamp_ = rclcpp::Time(message->stamp, RCL_ROS_TIME);
      reset_epoch_ = message->reset_epoch;
      map_version_ = message->map_version;
      qualification_sha256_ = message->qualification_receipt_sha256;
      motion_core_sha256_ = message->motion_core_sha256;
    });
  client_executor_ =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  client_executor_->add_node(client_node_);
  client_thread_ = std::thread([this]() {client_executor_->spin();});
}

void BioNavGridBased::cleanup()
{
  if (client_executor_) {
    client_executor_->cancel();
  }
  if (client_thread_.joinable()) {
    client_thread_.join();
  }
  if (client_executor_ && client_node_) {
    client_executor_->remove_node(client_node_);
  }
  identity_subscription_.reset();
  prior_client_.reset();
  client_node_.reset();
  client_executor_.reset();
  if (stock_) {
    stock_->cleanup();
    stock_.reset();
  }
  decision_publisher_.reset();
  visualization_publisher_.reset();
  costmap_ros_.reset();
  clock_.reset();
}

void BioNavGridBased::activate()
{
  if (stock_) {
    stock_->activate();
  }
  if (decision_publisher_) {
    decision_publisher_->on_activate();
  }
  if (visualization_publisher_) {
    visualization_publisher_->on_activate();
  }
}

void BioNavGridBased::deactivate()
{
  if (decision_publisher_) {
    decision_publisher_->on_deactivate();
  }
  if (visualization_publisher_) {
    visualization_publisher_->on_deactivate();
  }
  if (stock_) {
    stock_->deactivate();
  }
}

nav_msgs::msg::Path BioNavGridBased::stockPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::function<bool()> & cancel_checker,
  const std::string & reason, double begin_s)
{
  auto path = stock_->createPlan(start, goal, cancel_checker);
  const double latency_ms =
    (clock_->now().seconds() - begin_s) * 1000.0;
  uint32_t reset_epoch = 0;
  std::string map_version;
  {
    std::lock_guard<std::mutex> lock(identity_mutex_);
    reset_epoch = reset_epoch_;
    map_version = map_version_;
  }
  publishDecision(
    false, reason, 0.0, 0, latency_ms, reset_epoch, map_version, "", "");
  return path;
}

nav_msgs::msg::Path BioNavGridBased::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  visualization_x_ = start.pose.position.x;
  visualization_y_ = start.pose.position.y;
  const double begin_s = clock_->now().seconds();
  uint32_t reset_epoch = 0;
  std::string map_version;
  std::string identity_error;
  {
    std::lock_guard<std::mutex> lock(identity_mutex_);
    if (!identity_seen_) {
      identity_error = "no_planning_prior";
    } else {
      const double identity_age_s =
        (clock_->now() - identity_stamp_).seconds();
      if (!priorIdentityFresh(identity_age_s, maximum_prior_age_s_)) {
        identity_error = "planning_prior_stale";
      }
      reset_epoch = reset_epoch_;
      map_version = map_version_;
    }
  }
  if (!identity_error.empty()) {
    return stockPlan(
      start, goal, cancel_checker, identity_error, begin_s);
  }
  if (!prior_client_->service_is_ready()) {
    return stockPlan(start, goal, cancel_checker, "goal_prior_service_unavailable", begin_s);
  }
  auto request =
    std::make_shared<bio_nav_interfaces::srv::GetGoalPlanningPrior::Request>();
  request->goal = goal;
  request->reset_epoch = reset_epoch;
  request->map_version = map_version;
  request->goal_hash = "";
  auto future = prior_client_->async_send_request(request);
  if (
    future.wait_for(std::chrono::milliseconds(service_timeout_ms_)) !=
    std::future_status::ready)
  {
    return stockPlan(start, goal, cancel_checker, "goal_prior_timeout", begin_s);
  }
  auto response = future.get();
  if (!response->success || !response->prior.healthy) {
    return stockPlan(
      start, goal, cancel_checker,
      "goal_prior_rejected:" + response->error, begin_s);
  }
  const auto & prior = response->prior;
  if (
    prior.reset_epoch != reset_epoch || prior.map_version != map_version ||
    (!expected_module3_map_sha256_.empty() &&
    prior.module3_map_sha256 != expected_module3_map_sha256_) ||
    (!expected_qualification_sha256_.empty() &&
    prior.qualification_receipt_sha256 != expected_qualification_sha256_))
  {
    return stockPlan(start, goal, cancel_checker, "goal_prior_identity_mismatch", begin_s);
  }
  std::array<float, 256> score{};
  std::copy(prior.tie_break_score.begin(), prior.tie_break_score.end(), score.begin());
  for (const auto value : score) {
    if (!std::isfinite(value)) {
      return stockPlan(start, goal, cancel_checker, "goal_prior_nonfinite", begin_s);
    }
  }
  TieBreakPlanMetrics metrics;
  nav_msgs::msg::Path path;
  try {
    path = stock_->createPlanWithTieBreak(
      start, goal, score, cancel_checker, metrics);
  } catch (const std::exception & error) {
    return stockPlan(
      start, goal, cancel_checker,
      "cognitive_search:" + std::string(error.what()), begin_s);
  }
  const double latency_ms =
    (clock_->now().seconds() - begin_s) * 1000.0;
  publishDecision(
    true, "", metrics.primary_cost, metrics.expanded_nodes, latency_ms,
    reset_epoch, map_version, prior.goal_hash, prior.snapshot_sha256);
  return path;
}

bool BioNavGridBased::priorIdentityFresh(
  double age_s, double maximum_age_s)
{
  return
    std::isfinite(age_s) && std::isfinite(maximum_age_s) &&
    maximum_age_s > 0.0 && age_s >= 0.0 && age_s <= maximum_age_s;
}

GridSearchResult BioNavGridBased::equalCostSearch(
  nav2_costmap_2d::Costmap2D & costmap,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::array<float, 256> & tie_break_score,
  bool allow_unknown,
  const std::string & global_frame,
  const rclcpp::Time & stamp,
  const std::function<bool()> & cancel_checker)
{
  GridSearchResult result;
  unsigned int start_x = 0;
  unsigned int start_y = 0;
  unsigned int goal_x = 0;
  unsigned int goal_y = 0;
  if (
    !costmap.worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
    !costmap.worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
  {
    result.error = "start_or_goal_outside_costmap";
    return result;
  }
  if (
    !traversable(costmap, start_x, start_y, allow_unknown) ||
    !traversable(costmap, goal_x, goal_y, allow_unknown))
  {
    result.error = "start_or_goal_blocked";
    return result;
  }
  const uint64_t size =
    static_cast<uint64_t>(costmap.getSizeInCellsX()) *
    costmap.getSizeInCellsY();
  const uint64_t start_index = costmap.getIndex(start_x, start_y);
  const uint64_t goal_index = costmap.getIndex(goal_x, goal_y);
  const uint64_t infinity = std::numeric_limits<uint64_t>::max();
  std::vector<uint64_t> g(size, infinity);
  std::vector<uint64_t> parent(size, infinity);
  std::vector<bool> closed(size, false);
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenWorse> open;
  uint64_t serial = 0;
  g[start_index] = 0;
  open.push(
    OpenEntry{
      start_index,
      octileHeuristic(start_x, start_y, goal_x, goal_y),
      tieScore(costmap, start_index, tie_break_score),
      serial++,
      0});
  const std::array<std::pair<int, int>, 8> directions{{
    {-1, -1}, {0, -1}, {1, -1}, {-1, 0},
    {1, 0}, {-1, 1}, {0, 1}, {1, 1}}};
  bool found = false;
  while (!open.empty()) {
    const auto entry = open.top();
    open.pop();
    if (entry.g != g[entry.index] || closed[entry.index]) {
      continue;
    }
    closed[entry.index] = true;
    ++result.expanded_nodes;
    if (entry.index == goal_index) {
      found = true;
      break;
    }
    if (result.expanded_nodes % 100 == 0 && cancel_checker()) {
      result.error = "cancelled";
      return result;
    }
    unsigned int current_x = 0;
    unsigned int current_y = 0;
    costmap.indexToCells(entry.index, current_x, current_y);
    for (const auto [dx, dy] : directions) {
      const int next_x = static_cast<int>(current_x) + dx;
      const int next_y = static_cast<int>(current_y) + dy;
      if (!traversable(costmap, next_x, next_y, allow_unknown)) {
        continue;
      }
      if (
        dx != 0 && dy != 0 &&
        (!traversable(costmap, static_cast<int>(current_x) + dx, current_y, allow_unknown) ||
        !traversable(costmap, current_x, static_cast<int>(current_y) + dy, allow_unknown)))
      {
        continue;
      }
      const uint64_t next_index = costmap.getIndex(next_x, next_y);
      if (closed[next_index]) {
        continue;
      }
      const auto raw_cost = costmap.getCost(next_x, next_y);
      const uint64_t cell_cost =
        raw_cost == nav2_costmap_2d::NO_INFORMATION ? 0 : raw_cost;
      const uint64_t step = (dx != 0 && dy != 0 ? 1414 : 1000) + cell_cost * 10;
      const uint64_t candidate = entry.g + step;
      if (candidate >= g[next_index]) {
        continue;
      }
      g[next_index] = candidate;
      parent[next_index] = entry.index;
      const uint64_t heuristic = octileHeuristic(
        next_x, next_y, static_cast<int>(goal_x), static_cast<int>(goal_y));
      open.push(
        OpenEntry{
          next_index, candidate + heuristic,
          tieScore(costmap, next_index, tie_break_score), serial++, candidate});
    }
  }
  if (!found) {
    result.error = "no_path";
    return result;
  }
  std::vector<uint64_t> reverse;
  for (uint64_t index = goal_index;; index = parent[index]) {
    reverse.push_back(index);
    if (index == start_index) {
      break;
    }
    if (parent[index] == infinity) {
      result.error = "broken_parent_chain";
      return result;
    }
  }
  std::reverse(reverse.begin(), reverse.end());
  result.path.header.frame_id = global_frame;
  result.path.header.stamp = stamp;
  result.path.poses.reserve(reverse.size());
  for (std::size_t index = 0; index < reverse.size(); ++index) {
    unsigned int mx = 0;
    unsigned int my = 0;
    costmap.indexToCells(reverse[index], mx, my);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = result.path.header;
    costmap.mapToWorld(mx, my, pose.pose.position.x, pose.pose.position.y);
    pose.pose.position.z = 0.0;
    double yaw = 0.0;
    if (index + 1 < reverse.size()) {
      unsigned int next_x = 0;
      unsigned int next_y = 0;
      costmap.indexToCells(reverse[index + 1], next_x, next_y);
      double next_wx = 0.0;
      double next_wy = 0.0;
      costmap.mapToWorld(next_x, next_y, next_wx, next_wy);
      yaw = std::atan2(
        next_wy - pose.pose.position.y, next_wx - pose.pose.position.x);
    }
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation = tf2::toMsg(quaternion);
    result.path.poses.push_back(pose);
  }
  result.path.poses.back().pose = goal.pose;
  result.primary_cost = static_cast<double>(g[goal_index]) / 1000.0;
  result.success = true;
  return result;
}

void BioNavGridBased::publishDecision(
  bool used, const std::string & fallback_reason, double primary_cost,
  uint64_t expanded_nodes, double latency_ms, uint32_t reset_epoch,
  const std::string & map_version, const std::string & goal_hash,
  const std::string & snapshot_sha256)
{
  if (!decision_publisher_ || !decision_publisher_->is_activated()) {
    return;
  }
  bio_nav_interfaces::msg::PlannerDecision decision;
  decision.stamp = clock_->now();
  decision.sequence = ++sequence_;
  decision.planner_profile = planner_profile_;
  decision.cognitive_tiebreak_used = used;
  decision.fallback_reason = fallback_reason;
  decision.primary_path_cost = primary_cost;
  decision.expanded_nodes = expanded_nodes;
  decision.planning_latency_ms = static_cast<float>(latency_ms);
  decision.reset_epoch = reset_epoch;
  decision.map_version = map_version;
  decision.goal_hash = goal_hash;
  decision.snapshot_sha256 = snapshot_sha256;
  decision.qualification_receipt_sha256 = qualification_sha256_;
  decision.motion_core_sha256 = motion_core_sha256_;
  decision.module3_map_sha256 = expected_module3_map_sha256_;
  decision_publisher_->publish(decision);
  if (visualization_publisher_ && visualization_publisher_->is_activated()) {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = global_frame_;
    clear.header.stamp = decision.stamp;
    clear.ns = "Module2 Planning";
    clear.id = 0;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
    visualization_msgs::msg::Marker status;
    status.header = clear.header;
    status.ns = "Planning Decision";
    status.id = 1;
    status.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    status.action = visualization_msgs::msg::Marker::ADD;
    status.pose.position.x = visualization_x_;
    status.pose.position.y = visualization_y_ + 1.2;
    status.pose.position.z = 2.2;
    status.pose.orientation.w = 1.0;
    status.scale.z = 0.16;
    status.lifetime = rclcpp::Duration::from_seconds(4.0);
    if (used) {
      status.text = "M2 PLANNING ADOPTED | expanded=" +
        std::to_string(expanded_nodes) + " | latency=" +
        std::to_string(static_cast<int>(std::lround(latency_ms))) +
        " ms";
      status.color.r = 0.0F;
      status.color.g = 1.0F;
      status.color.b = 0.85F;
      status.color.a = 1.0F;
    } else {
      status.text = "M2 PLANNING FALLBACK | " + fallback_reason;
      status.color.r = 1.0F;
      status.color.g = 0.65F;
      status.color.b = 0.0F;
      status.color.a = 1.0F;
    }
    array.markers.push_back(status);
    visualization_publisher_->publish(array);
  }
}

}  // namespace bio_nav_fusion

PLUGINLIB_EXPORT_CLASS(
  bio_nav_fusion::BioNavGridBased, nav2_core::GlobalPlanner)
