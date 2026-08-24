#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "pointcloud_local_odometry/gicp_odometry.hpp"

namespace pointcloud_local_odometry
{

namespace
{

diagnostic_msgs::msg::KeyValue value(const std::string & key, const std::string & text)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = text;
  return item;
}

std::string number(double input)
{
  if (!std::isfinite(input)) {
    return "nan";
  }
  std::ostringstream stream;
  stream.precision(9);
  stream << input;
  return stream.str();
}

Eigen::Isometry3d toEigen(const geometry_msgs::msg::Transform & transform)
{
  const Eigen::Quaterniond rotation(
    transform.rotation.w,
    transform.rotation.x,
    transform.rotation.y,
    transform.rotation.z);
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = rotation.normalized().toRotationMatrix();
  result.translation() = Eigen::Vector3d(
    transform.translation.x,
    transform.translation.y,
    transform.translation.z);
  return result;
}

}  // namespace

class GicpShadowNode : public rclcpp::Node
{
public:
  GicpShadowNode()
  : Node("gicp_shadow_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/lio/points_raw");
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/local_odom/gicp_shadow");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/local_odom/gicp_status");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lio_lidar_link");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "gicp_odom_shadow");

    GicpConfig config;
    config.voxel_leaf_size = declare_parameter<double>("voxel_leaf_size", 0.15);
    config.min_points = static_cast<std::size_t>(
      declare_parameter<int64_t>("min_points", 100));
    config.max_correspondence_distance = declare_parameter<double>(
      "max_correspondence_distance", 1.0);
    config.max_iterations = static_cast<int>(
      declare_parameter<int64_t>("max_iterations", 40));
    config.transformation_epsilon = declare_parameter<double>(
      "transformation_epsilon", 1.0e-4);
    config.euclidean_fitness_epsilon = declare_parameter<double>(
      "euclidean_fitness_epsilon", 1.0e-4);
    config.max_fitness_score = declare_parameter<double>("max_fitness_score", 0.25);
    pose_covariance_ = declare_parameter<std::vector<double>>(
      "pose_covariance_diagonal", {0.04, 0.04, 0.09, 0.09, 0.09, 0.04});
    twist_covariance_ = declare_parameter<std::vector<double>>(
      "twist_covariance_diagonal",
      {1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6});

    if (input_topic_.empty() || odom_topic_.empty() || status_topic_.empty() ||
      lidar_frame_.empty() || base_frame_.empty() || odom_frame_.empty())
    {
      throw std::invalid_argument("topics and frames must not be empty");
    }
    if (pose_covariance_.size() != 6U || twist_covariance_.size() != 6U) {
      throw std::invalid_argument("covariance diagonals must contain six values");
    }
    odometry_ = std::make_unique<ScanToScanOdometry>(config);

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    status_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      status_topic_, 10);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&GicpShadowNode::cloudCallback, this, std::placeholders::_1));
  }

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  void publishStatus(
    const sensor_msgs::msg::PointCloud2 & message,
    const std::string & state,
    const std::string & reason,
    std::size_t raw_points,
    std::size_t filtered_points,
    bool converged,
    double fitness,
    const SteadyTime & started)
  {
    const double processing_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = state == "degraded" ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.name = "pointcloud_local_odometry/gicp_shadow";
    status.hardware_id = "gicp_shadow";
    status.message = state;
    status.values = {
      value("state", state),
      value("raw_points", std::to_string(raw_points)),
      value("filtered_points", std::to_string(filtered_points)),
      value("converged", converged ? "true" : "false"),
      value("fitness", number(fitness)),
      value("processing_ms", number(processing_ms)),
      value("reason", reason),
    };

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header = message.header;
    array.status.push_back(std::move(status));
    status_publisher_->publish(array);
  }

  void publishOdometry(
    const sensor_msgs::msg::PointCloud2 & message,
    const Eigen::Isometry3d & pose)
  {
    nav_msgs::msg::Odometry output;
    output.header.stamp = message.header.stamp;
    output.header.frame_id = odom_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose.position.x = pose.translation().x();
    output.pose.pose.position.y = pose.translation().y();
    output.pose.pose.position.z = pose.translation().z();
    const Eigen::Quaterniond orientation(pose.rotation());
    output.pose.pose.orientation.x = orientation.x();
    output.pose.pose.orientation.y = orientation.y();
    output.pose.pose.orientation.z = orientation.z();
    output.pose.pose.orientation.w = orientation.w();
    output.pose.covariance.fill(0.0);
    output.twist.covariance.fill(0.0);
    for (std::size_t index = 0U; index < 6U; ++index) {
      output.pose.covariance[index * 6U + index] = pose_covariance_[index];
      output.twist.covariance[index * 6U + index] = twist_covariance_[index];
    }
    odom_publisher_->publish(output);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    const auto started = std::chrono::steady_clock::now();
    const std::size_t raw_points =
      static_cast<std::size_t>(message->width) * static_cast<std::size_t>(message->height);
    const int64_t stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
    if (stamp_ns <= 0) {
      publishStatus(*message, "degraded", "invalid_stamp", raw_points, 0U, false,
        std::numeric_limits<double>::quiet_NaN(), started);
      return;
    }
    if (last_input_stamp_ns_ >= 0 && stamp_ns <= last_input_stamp_ns_) {
      publishStatus(*message, "degraded", "stamp_rollback", raw_points, 0U, false,
        std::numeric_limits<double>::quiet_NaN(), started);
      return;
    }
    last_input_stamp_ns_ = stamp_ns;
    if (message->header.frame_id != lidar_frame_) {
      publishStatus(*message, "degraded", "unexpected_frame", raw_points, 0U, false,
        std::numeric_limits<double>::quiet_NaN(), started);
      return;
    }

    Eigen::Isometry3d base_to_lidar;
    try {
      const auto transform = tf_buffer_.lookupTransform(
        base_frame_, lidar_frame_, tf2::TimePointZero);
      base_to_lidar = toEigen(transform.transform);
    } catch (const tf2::TransformException &) {
      publishStatus(*message, "degraded", "tf_missing", raw_points, 0U, false,
        std::numeric_limits<double>::quiet_NaN(), started);
      return;
    }

    auto cloud = std::make_shared<ScanToScanOdometry::Cloud>();
    try {
      pcl::fromROSMsg(*message, *cloud);
    } catch (const std::exception &) {
      publishStatus(*message, "degraded", "invalid_cloud", raw_points, 0U, false,
        std::numeric_limits<double>::quiet_NaN(), started);
      return;
    }

    const OdometryResult result = odometry_->process(cloud, base_to_lidar);
    if (!result.accepted) {
      publishStatus(
        *message, "degraded", result.reason, result.raw_points, result.filtered_points,
        result.converged, result.fitness, started);
      return;
    }

    publishOdometry(*message, result.odom_base);
    publishStatus(
      *message, result.initializing ? "initializing" : "tracking", result.reason,
      result.raw_points, result.filtered_points, result.converged, result.fitness, started);
  }

  std::string input_topic_;
  std::string odom_topic_;
  std::string status_topic_;
  std::string lidar_frame_;
  std::string base_frame_;
  std::string odom_frame_;
  std::vector<double> pose_covariance_;
  std::vector<double> twist_covariance_;
  int64_t last_input_stamp_ns_{-1};
  std::unique_ptr<ScanToScanOdometry> odometry_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace pointcloud_local_odometry

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pointcloud_local_odometry::GicpShadowNode>());
  rclcpp::shutdown();
  return 0;
}
