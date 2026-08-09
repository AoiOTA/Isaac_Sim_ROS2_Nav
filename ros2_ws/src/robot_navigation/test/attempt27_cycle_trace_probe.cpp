#include <dlfcn.h>

#include <array>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#include "nav2_mppi_controller/controller.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2) {
    return 2;
  }
  void * library = dlopen(argv[1], RTLD_NOW | RTLD_GLOBAL);
  if (library == nullptr) {
    throw std::runtime_error(dlerror());
  }
  using Method = geometry_msgs::msg::TwistStamped (*)(
    nav2_mppi_controller::MPPIController *,
    const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *);
  auto method = reinterpret_cast<Method>(dlsym(
      RTLD_DEFAULT,
      "_ZN20nav2_mppi_controller14MPPIController23computeVelocityCommandsERKN13geometry_msgs3msg12PoseStamped_ISaIvEEERKNS2_6Twist_IS4_EEPN9nav2_core11GoalCheckerE"));
  if (method == nullptr) {
    throw std::runtime_error(dlerror());
  }
  alignas(nav2_mppi_controller::MPPIController) std::array<unsigned char, 4096> storage{};
  auto * controller = reinterpret_cast<nav2_mppi_controller::MPPIController *>(
    storage.data());
  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::Twist speed;
  pose.pose.position.x = 1.25;
  pose.pose.position.y = -0.75;
  speed.linear.x = 0.50;
  speed.angular.z = 0.25;
  const auto result = method(controller, pose, speed, nullptr);
  std::cout << std::fixed << std::setprecision(17)
            << result.twist.linear.x << " " << result.twist.angular.z << "\n";
  dlclose(library);
  return 0;
}
