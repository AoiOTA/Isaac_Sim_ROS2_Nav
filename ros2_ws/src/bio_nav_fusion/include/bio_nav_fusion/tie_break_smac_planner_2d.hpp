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
  uint64_t zero_sr_expanded_nodes{0};
  int64_t expanded_node_delta{0};
  bool search_changed{false};
  uint32_t zero_sr_only_expanded_cell_count{0};
  uint32_t sr_only_expanded_cell_count{0};
  bool path_changed{false};
  uint32_t zero_sr_only_cell_count{0};
  uint32_t sr_only_cell_count{0};
  double max_path_delta_m{0.0};
  double path_length_delta_m{0.0};
  nav_msgs::msg::Path zero_tie_reference;
  nav_msgs::msg::Path tie_break_result;
  nav_msgs::msg::Path zero_sr_only_cells;
  nav_msgs::msg::Path sr_only_cells;
  nav_msgs::msg::Path zero_sr_only_expanded_cells;
  nav_msgs::msg::Path sr_only_expanded_cells;
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
