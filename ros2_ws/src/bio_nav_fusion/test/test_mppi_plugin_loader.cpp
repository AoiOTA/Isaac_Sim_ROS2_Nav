#include <memory>

#include "gtest/gtest.h"
#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_loader.hpp"

TEST(BioNavFusionPlugins, predictive_risk_critic_is_discoverable)
{
  pluginlib::ClassLoader<mppi::critics::CriticFunction> loader(
    "nav2_mppi_controller", "mppi::critics::CriticFunction");
  const auto critic = loader.createUniqueInstance(
    "mppi::critics::PredictiveRiskCritic");
  EXPECT_NE(critic, nullptr);
}
