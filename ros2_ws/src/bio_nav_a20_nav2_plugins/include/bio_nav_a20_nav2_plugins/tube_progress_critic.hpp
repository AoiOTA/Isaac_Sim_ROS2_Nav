#ifndef BIO_NAV_A20_NAV2_PLUGINS__TUBE_PROGRESS_CRITIC_HPP_
#define BIO_NAV_A20_NAV2_PLUGINS__TUBE_PROGRESS_CRITIC_HPP_

#include <vector>

#include "nav2_mppi_controller/critic_function.hpp"

namespace bio_nav_a20_nav2_plugins
{

/**
 * @class bio_nav_a20_nav2_plugins::TubeProgressCritic
 * @brief Direction-agnostic path-progress critic for the A20 tube
 * architecture.
 *
 * Scores every sampled trajectory by the fraction of the reference path arc
 * length it covers: cost += cost_weight * (1 - coverage). Coverage is the
 * maximum arc-length coordinate any trajectory point projects to (nearest
 * path point per trajectory point), so cutting across an unexecuted corner
 * cannot claim uncovered progress.
 *
 * Deliberately has no threshold_to_consider dead band (the progress goal
 * stays active to the very end of the path) and no forward-velocity bias:
 * reverse trajectories that cover more of the audited path score better.
 */
class TubeProgressCritic : public mppi::critics::CriticFunction
{
public:
  /**
   * @brief Initialize critic and declare parameters
   */
  void initialize() override;

  /**
   * @brief Add progress cost to every trajectory in the batch
   *
   * @param data Critic data with trajectories, path and the cost tensor
   */
  void score(mppi::CriticData & data) override;

protected:
  float weight_{6.0f};
  // Scratch buffer for the per-cycle path arc length, reused across batches.
  std::vector<float> path_arclength_;
};

}  // namespace bio_nav_a20_nav2_plugins

#endif  // BIO_NAV_A20_NAV2_PLUGINS__TUBE_PROGRESS_CRITIC_HPP_
