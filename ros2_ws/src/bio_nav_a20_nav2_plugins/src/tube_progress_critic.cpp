#include "bio_nav_a20_nav2_plugins/tube_progress_critic.hpp"

#include <algorithm>

#include "bio_nav_a20_nav2_plugins/path_coverage.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "xtensor/xview.hpp"

namespace bio_nav_a20_nav2_plugins
{

void TubeProgressCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(weight_, "cost_weight", 6.0f);
}

void TubeProgressCritic::score(mppi::CriticData & data)
{
  if (!enabled_) {
    return;
  }

  const std::size_t n_path = data.path.x.shape(0);
  if (n_path < 2) {
    // Nothing to make progress along; impose no cost.
    return;
  }

  path_arclength_ = compute_cumulative_arclength(data.path.x, data.path.y);
  const float total_arclength = path_arclength_.back();
  if (total_arclength <= 0.0f) {
    return;
  }

  const std::size_t batch_size = data.trajectories.x.shape(0);
  for (std::size_t i = 0; i < batch_size; ++i) {
    const auto traj_x = xt::view(data.trajectories.x, i, xt::all());
    const auto traj_y = xt::view(data.trajectories.y, i, xt::all());
    const double coverage = compute_covered_arclength(
      traj_x, traj_y, data.path.x, data.path.y, path_arclength_) /
      static_cast<double>(total_arclength);
    data.costs(i) += static_cast<float>(weight_ * (1.0 - coverage));
  }
}

}  // namespace bio_nav_a20_nav2_plugins

PLUGINLIB_EXPORT_CLASS(
  bio_nav_a20_nav2_plugins::TubeProgressCritic, mppi::critics::CriticFunction)
