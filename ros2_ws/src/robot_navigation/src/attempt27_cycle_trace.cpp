// Observation-only Attempt27 MPPI compute-cycle tracer.
//
// Load this library with LD_PRELOAD into controller_server.  ELF symbol
// interposition places the timestamps directly around the unchanged
// MPPIController::computeVelocityCommands implementation.  The original
// method is resolved with RTLD_NEXT; its arguments and return value are passed
// through without mutation.  Each JSONL record is one atomic write to avoid a
// DDS publisher or executor callback in the controller loop.

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <dlfcn.h>
#include <fcntl.h>
#include <mutex>
#include <stdexcept>
#include <string>
#include <sys/syscall.h>
#include <sys/types.h>
#include <unistd.h>

#include "nav2_mppi_controller/controller.hpp"

namespace
{

std::atomic<uint64_t> cycle_counter{0};
std::once_flag open_once;
int trace_fd = -1;

uint64_t steady_ns()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

void open_trace()
{
  const char * path = std::getenv("BIO_NAV_ATTEMPT27_CYCLE_TRACE");
  if (path == nullptr || path[0] == '\0') {
    return;
  }
  trace_fd = ::open(path, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0640);
}

void emit(const char * event, uint64_t cycle_id, bool exception_flag)
{
  std::call_once(open_once, open_trace);
  if (trace_fd < 0) {
    return;
  }
  const std::string line =
    "{\"schema\":\"bio_nav_attempt27_controller_cycle_v1\","
    "\"event\":\"" + std::string(event) + "\","
    "\"cycle_id\":\"" + std::to_string(cycle_id) + "\","
    "\"steady_ns\":" + std::to_string(steady_ns()) + ","
    "\"pid\":" + std::to_string(::getpid()) + ","
    "\"tid\":" + std::to_string(::syscall(SYS_gettid)) + ","
    "\"follow_path_active\":true,"
    "\"exception_flag\":" + (exception_flag ? "true" : "false") + "}\n";
  const ssize_t ignored = ::write(trace_fd, line.data(), line.size());
  (void)ignored;
}

}  // namespace

namespace nav2_mppi_controller
{

geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker)
{
  using Original = geometry_msgs::msg::TwistStamped (*)(
    MPPIController *, const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::Twist &, nav2_core::GoalChecker *);
  static Original original = reinterpret_cast<Original>(dlsym(
    RTLD_NEXT,
    "_ZN20nav2_mppi_controller14MPPIController23computeVelocityCommandsERKN13geometry_msgs3msg12PoseStamped_ISaIvEEERKNS2_6Twist_IS4_EEPN9nav2_core11GoalCheckerE"));
  if (original == nullptr) {
    throw std::runtime_error(
      "Attempt27 trace could not resolve the original MPPI compute method");
  }
  const uint64_t cycle_id = cycle_counter.fetch_add(1, std::memory_order_relaxed);
  emit("compute_start", cycle_id, false);
  try {
    auto command = original(this, robot_pose, robot_speed, goal_checker);
    emit("compute_end", cycle_id, false);
    return command;
  } catch (...) {
    emit("compute_end", cycle_id, true);
    throw;
  }
}

}  // namespace nav2_mppi_controller
