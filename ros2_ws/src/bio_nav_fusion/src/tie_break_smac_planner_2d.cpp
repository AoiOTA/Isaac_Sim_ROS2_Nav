// The search and plan-conversion flow in this file is derived from Nav2
// SmacPlanner2D 1.3.12, commit 6be3614013ec586051b86c97b919b293281490fe.
// Copyright (c) 2020, Samsung Research America
// Copyright (c) 2020, Applied Electric Vehicles Pty Ltd
// SPDX-License-Identifier: Apache-2.0

#include "bio_nav_fusion/tie_break_smac_planner_2d.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_smac_planner/node_2d.hpp"
#include "nav2_smac_planner/utils.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace bio_nav_fusion
{
namespace
{

using nav2_smac_planner::GridCollisionChecker;
using nav2_smac_planner::MotionModel;
using nav2_smac_planner::Node2D;
using nav2_smac_planner::SearchInfo;

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

struct OpenEntry
{
  float f;
  uint32_t tie;
  uint64_t serial;
  Node2D * node;
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

struct SearchResult
{
  Node2D::CoordinateVector path;
  float primary_cost{0.0F};
  uint64_t expanded_nodes{0};
  bool success{false};
};

class LexicographicSmacSearch2D
{
public:
  LexicographicSmacSearch2D(
    nav2_costmap_2d::Costmap2D & costmap,
    GridCollisionChecker & collision_checker,
    const SearchInfo & search_info,
    bool allow_unknown,
    int max_iterations,
    int max_on_approach_iterations,
    int terminal_checking_interval,
    double max_planning_time,
    const std::array<float, 256> & score)
  : costmap_(costmap),
    collision_checker_(collision_checker),
    allow_unknown_(allow_unknown),
    max_iterations_(max_iterations),
    max_on_approach_iterations_(max_on_approach_iterations),
    terminal_checking_interval_(terminal_checking_interval),
    max_planning_time_(max_planning_time),
    score_(score)
  {
    unsigned int width = costmap_.getSizeInCellsX();
    unsigned int height = costmap_.getSizeInCellsY();
    unsigned int dimensions = 1;
    SearchInfo mutable_search_info = search_info;
    Node2D::initMotionModel(
      MotionModel::TWOD, width, height, dimensions, mutable_search_info);
    graph_.reserve(100000);
  }

  SearchResult createPath(
    float start_x, float start_y, float goal_x, float goal_y,
    float tolerance, const std::function<bool()> & cancel_checker)
  {
    const auto started = std::chrono::steady_clock::now();
    const auto width = costmap_.getSizeInCellsX();
    const auto height = costmap_.getSizeInCellsY();
    const uint64_t max_index = static_cast<uint64_t>(width) * height;
    auto * start = addNode(Node2D::getIndex(
      static_cast<unsigned int>(start_x), static_cast<unsigned int>(start_y), width));
    auto * goal = addNode(Node2D::getIndex(
      static_cast<unsigned int>(goal_x), static_cast<unsigned int>(goal_y), width));
    const Node2D::Coordinates goal_coordinates(goal_x, goal_y);
    if (tolerance < 0.001F &&
      !goal->isNodeValid(allow_unknown_, &collision_checker_))
    {
      throw nav2_core::GoalOccupied("Goal was in lethal cost");
    }

    std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenWorse> open;
    uint64_t serial = 0;
    start->setAccumulatedCost(0.0F);
    open.push(OpenEntry{0.0F, tieScore(costmap_, start->getIndex(), score_), serial++, start});
    std::pair<float, uint64_t> best_heuristic{
      std::numeric_limits<float>::max(), 0};
    int approach_iterations = 0;
    int iterations = 0;
    Node2D::NodeVector neighbors;
    std::function<bool(const uint64_t &, Node2D * &)> getter =
      [this, max_index](const uint64_t & index, Node2D * & output) {
        if (index >= max_index) {
          return false;
        }
        output = addNode(index);
        return true;
      };

    while (iterations < max_iterations_ && !open.empty()) {
      if (iterations % terminal_checking_interval_ == 0) {
        if (cancel_checker()) {
          throw nav2_core::PlannerCancelled("Planner was cancelled");
        }
        const std::chrono::duration<double> elapsed =
          std::chrono::steady_clock::now() - started;
        if (elapsed.count() >= max_planning_time_) {
          return {};
        }
      }
      Node2D * current = open.top().node;
      open.pop();
      if (current->wasVisited()) {
        continue;
      }
      ++iterations;
      current->visited();
      if (current == goal) {
        return backtrace(current, iterations);
      }
      if (best_heuristic.first < tolerance) {
        ++approach_iterations;
        if (approach_iterations >= max_on_approach_iterations_) {
          return backtrace(graph_.at(best_heuristic.second).get(), iterations);
        }
      }

      neighbors.clear();
      current->getNeighbors(
        getter, &collision_checker_, allow_unknown_, neighbors);
      for (Node2D * neighbor : neighbors) {
        const float g_cost = current->getAccumulatedCost() +
          current->getTraversalCost(neighbor);
        if (g_cost >= neighbor->getAccumulatedCost()) {
          continue;
        }
        neighbor->setAccumulatedCost(g_cost);
        neighbor->parent = current;
        const auto coordinates = Node2D::getCoords(
          neighbor->getIndex(), width, 1);
        const float heuristic = Node2D::getHeuristicCost(
          coordinates, goal_coordinates);
        if (heuristic < best_heuristic.first) {
          best_heuristic = {heuristic, neighbor->getIndex()};
        }
        open.push(OpenEntry{
              g_cost + heuristic,
              tieScore(costmap_, neighbor->getIndex(), score_),
              serial++, neighbor});
      }
    }
    if (best_heuristic.first < tolerance) {
      return backtrace(graph_.at(best_heuristic.second).get(), iterations);
    }
    return {};
  }

private:
  Node2D * addNode(uint64_t index)
  {
    auto [iterator, inserted] = graph_.try_emplace(index, nullptr);
    if (inserted) {
      iterator->second = std::make_unique<Node2D>(index);
    }
    return iterator->second.get();
  }

  static SearchResult backtrace(Node2D * node, uint64_t iterations)
  {
    SearchResult result;
    result.primary_cost = node->getAccumulatedCost();
    result.expanded_nodes = iterations;
    result.success = node->backtracePath(result.path);
    return result;
  }

  nav2_costmap_2d::Costmap2D & costmap_;
  GridCollisionChecker & collision_checker_;
  bool allow_unknown_;
  int max_iterations_;
  int max_on_approach_iterations_;
  int terminal_checking_interval_;
  double max_planning_time_;
  const std::array<float, 256> & score_;
  std::unordered_map<uint64_t, std::unique_ptr<Node2D>> graph_;
};

}  // namespace

nav_msgs::msg::Path TieBreakSmacPlanner2D::createPlanWithTieBreak(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::array<float, 256> & tie_break_score,
  std::function<bool()> cancel_checker,
  TieBreakPlanMetrics & metrics)
{
  std::lock_guard<std::mutex> reinit_lock(_mutex);
  const auto started = std::chrono::steady_clock::now();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(
    *(_costmap->getMutex()));
  nav2_costmap_2d::Costmap2D * costmap = _costmap;
  if (_costmap_downsampler) {
    costmap = _costmap_downsampler->downsample(_downsampling_factor);
    _collision_checker.setCostmap(costmap);
  }

  float start_x = 0.0F;
  float start_y = 0.0F;
  float goal_x = 0.0F;
  float goal_y = 0.0F;
  if (!costmap->worldToMapContinuous(
      start.pose.position.x, start.pose.position.y, start_x, start_y))
  {
    throw nav2_core::StartOutsideMapBounds("Start coordinates were outside bounds");
  }
  if (!costmap->worldToMapContinuous(
      goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
  {
    throw nav2_core::GoalOutsideMapBounds("Goal coordinates were outside bounds");
  }

  nav_msgs::msg::Path plan;
  plan.header.stamp = _clock->now();
  plan.header.frame_id = _global_frame;
  geometry_msgs::msg::PoseStamped pose;
  pose.header = plan.header;
  pose.pose.orientation.w = 1.0;
  if (std::floor(start_x) == std::floor(goal_x) &&
    std::floor(start_y) == std::floor(goal_y))
  {
    pose.pose = start.pose;
    if (start.pose.orientation != goal.pose.orientation &&
      !_use_final_approach_orientation)
    {
      pose.pose.orientation = goal.pose.orientation;
    }
    plan.poses.push_back(pose);
    return plan;
  }

  const std::array<float, 256> zero_score{};
  LexicographicSmacSearch2D baseline_search(
    *costmap, _collision_checker, _search_info, _allow_unknown,
    _max_iterations, _max_on_approach_iterations,
    _terminal_checking_interval, _max_planning_time, zero_score);
  const auto baseline = baseline_search.createPath(
    start_x, start_y, goal_x, goal_y,
    _tolerance / static_cast<float>(costmap->getResolution()), cancel_checker);
  if (!baseline.success) {
    throw nav2_core::NoValidPathCouldBeFound(
            "zero-tie Smac reference search found no valid path");
  }
  const std::chrono::duration<double> reference_elapsed =
    std::chrono::steady_clock::now() - started;
  const double tie_time_budget = _max_planning_time - reference_elapsed.count();
  if (tie_time_budget <= 0.0) {
    throw nav2_core::PlannerTimedOut(
            "zero-tie Smac reference exhausted the planning budget");
  }
  LexicographicSmacSearch2D search(
    *costmap, _collision_checker, _search_info, _allow_unknown,
    _max_iterations, _max_on_approach_iterations,
    _terminal_checking_interval, tie_time_budget, tie_break_score);
  const auto result = search.createPath(
    start_x, start_y, goal_x, goal_y,
    _tolerance / static_cast<float>(costmap->getResolution()), cancel_checker);
  if (!result.success) {
    if (result.expanded_nodes == 1) {
      throw nav2_core::StartOccupied("Start occupied");
    }
    if (result.expanded_nodes < static_cast<uint64_t>(_max_iterations)) {
      throw nav2_core::NoValidPathCouldBeFound("no valid path found");
    }
    throw nav2_core::PlannerTimedOut("exceeded maximum iterations");
  }
  if (std::abs(result.primary_cost - baseline.primary_cost) > 1.0e-6F) {
    throw nav2_core::NoValidPathCouldBeFound(
            "tie-break changed the Smac primary path cost");
  }
  metrics.primary_cost = result.primary_cost;
  metrics.expanded_nodes = result.expanded_nodes;
  plan.poses.reserve(result.path.size());
  for (auto iterator = result.path.rbegin(); iterator != result.path.rend(); ++iterator) {
    pose.pose = nav2_smac_planner::getWorldCoords(iterator->x, iterator->y, costmap);
    plan.poses.push_back(pose);
  }
  if (_raw_plan_publisher->get_subscription_count() > 0) {
    _raw_plan_publisher->publish(plan);
  }
  const std::chrono::duration<double> elapsed =
    std::chrono::steady_clock::now() - started;
  _smoother->smooth(plan, costmap, _max_planning_time - elapsed.count());
  const std::size_t size = plan.poses.size();
  if (_use_final_approach_orientation) {
    if (size == 1) {
      plan.poses.back().pose.orientation = start.pose.orientation;
    } else if (size > 1) {
      const auto & last = plan.poses.back().pose.position;
      const auto & previous = plan.poses[size - 2].pose.position;
      plan.poses.back().pose.orientation =
        nav2_util::geometry_utils::orientationAroundZAxis(
        std::atan2(last.y - previous.y, last.x - previous.x));
    }
  } else if (size > 0) {
    plan.poses.back().pose.orientation = goal.pose.orientation;
  }
  return plan;
}

}  // namespace bio_nav_fusion
