// Copyright 2026 AoiOTA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_perception/lidar_self_filter_core.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace robot_perception
{

class LidarSelfFilterNode : public rclcpp::Node
{
public:
  LidarSelfFilterNode()
  : Node("lidar_self_filter"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/lidar/points_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/lidar/points_scan");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    const auto minimum = declare_parameter<std::vector<double>>(
      "min_xyz", {-0.235, -0.215, -0.05});
    const auto maximum = declare_parameter<std::vector<double>>(
      "max_xyz", {0.260, 0.215, 0.55});
    transform_timeout_s_ = declare_parameter<double>("transform_timeout", 0.05);

    if (input_topic_.empty() || output_topic_.empty() || target_frame_.empty()) {
      throw std::invalid_argument("self-filter topics and target_frame must not be empty");
    }
    if (minimum.size() != 3U || maximum.size() != 3U) {
      throw std::invalid_argument("min_xyz and max_xyz must contain exactly three values");
    }
    if (!std::isfinite(transform_timeout_s_) || transform_timeout_s_ < 0.0) {
      throw std::invalid_argument("transform_timeout must be finite and non-negative");
    }
    bounds_ = {
      {minimum[0], minimum[1], minimum[2]},
      {maximum[0], maximum[1], maximum[2]},
    };
    validate_bounds(bounds_);

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&LidarSelfFilterNode::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Near-field LiDAR self filter: %s -> %s in %s, "
      "bounds=[%.3f %.3f %.3f]-[%.3f %.3f %.3f]",
      input_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str(),
      bounds_.minimum[0], bounds_.minimum[1], bounds_.minimum[2],
      bounds_.maximum[0], bounds_.maximum[1], bounds_.maximum[2]);
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    sensor_msgs::msg::PointCloud2 transformed;
    try {
      if (message->header.frame_id == target_frame_) {
        transformed = *message;
      } else {
        const auto transform = tf_buffer_.lookupTransform(
          target_frame_,
          message->header.frame_id,
          rclcpp::Time(message->header.stamp),
          rclcpp::Duration::from_seconds(transform_timeout_s_));
        tf2::doTransform(*message, transformed, transform);
      }
      transformed.header.stamp = message->header.stamp;
      transformed.header.frame_id = target_frame_;
      auto filtered = filter_axis_aligned_self_points(transformed, bounds_);
      filtered.header.stamp = message->header.stamp;
      filtered.header.frame_id = target_frame_;
      publisher_->publish(std::move(filtered));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping LiDAR cloud because %s -> %s transform failed: %s",
        message->header.frame_id.c_str(), target_frame_.c_str(), error.what());
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping malformed LiDAR cloud: %s", error.what());
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  double transform_timeout_s_;
  AxisAlignedBounds bounds_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace robot_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<robot_perception::LidarSelfFilterNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("lidar_self_filter"),
      "Failed to start LiDAR self filter: %s", error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
