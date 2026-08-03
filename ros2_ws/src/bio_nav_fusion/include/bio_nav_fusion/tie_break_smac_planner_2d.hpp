#ifndef BIO_NAV_FUSION__TIE_BREAK_SMAC_PLANNER_2D_HPP_
#define BIO_NAV_FUSION__TIE_BREAK_SMAC_PLANNER_2D_HPP_

#include <array>
#include <cstdint>
#include <functional>

#include "nav_msgs/msg/path.hpp"
#include "nav2_smac_planner/smac_planner_2d.hpp"

namespace bio_nav_fusion
{

struct TieBreakPlanMetrics
{
  double primary_cost{0.0};
  uint64_t expanded_nodes{0};
  bool path_changed{false};
  nav_msgs::msg::Path zero_tie_reference;
  nav_msgs::msg::Path tie_break_result;
};

/// SmacPlanner2D-compatible search with an exact lexicographic queue key:
/// (Smac f-cost, -SR tie score, deterministic serial).
class TieBreakSmacPlanner2D : public nav2_smac_planner::SmacPlanner2D
{
public:
  nav_msgs::msg::Path createPlanWithTieBreak(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::array<float, 256> & tie_break_score,
    std::function<bool()> cancel_checker,
    TieBreakPlanMetrics & metrics);
};

}  // namespace bio_nav_fusion

#endif  // BIO_NAV_FUSION__TIE_BREAK_SMAC_PLANNER_2D_HPP_
