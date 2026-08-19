/** ****************************************************************************************
*  This node presents a fast and precise method to estimate the planar motion of a lidar
*  from consecutive range scans. It is very useful for the estimation of the robot odometry from
*  2D laser range measurements.
*  This module is developed for mobile robots with innacurate or inexistent built-in odometry.
*  It allows the estimation of a precise odometry with low computational cost.
*  For more information, please refer to:
*
*  Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA'16.
*  Available at: http://mapir.uma.es/papersrepo/2016/2016_Jaimez_ICRA_RF2O.pdf
*
* Maintainer: Javier G. Monroy
* MAPIR group: https://mapir.isa.uma.es
*
* Modifications: Jeremie Deray & (see contributons on github)
******************************************************************************************** */

#include "rf2o_laser_odometry/CLaserOdometry2DNode.hpp"

#include <cmath>
#include <stdexcept>

using namespace rf2o;

CLaserOdometry2DNode::CLaserOdometry2DNode()
: Node("rf2o_laser_odometry_node"),
  rf2o_ref(get_logger()),
  new_scan_available(false),
  last_scan_stamp_ns(-1)
{
  RCLCPP_INFO(get_logger(), "Initializing RF2O node...");

  // Read Parameters
  //----------------
  this->declare_parameter<std::string>("laser_scan_topic", "/scan");
  this->get_parameter("laser_scan_topic", laser_scan_topic);
  this->declare_parameter<std::string>("odom_topic", "/odom_rf2o");
  this->get_parameter("odom_topic", odom_topic);
  this->declare_parameter<std::string>("base_frame_id", "base_link");
  this->get_parameter("base_frame_id", base_frame_id);
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->get_parameter("odom_frame_id", odom_frame_id);
  const bool publish_tf = this->declare_parameter<bool>("publish_tf", false);
  if (publish_tf)
  {
    throw std::invalid_argument(
      "RF2O is topic-only in this stack: publish_tf must remain false");
  }
  this->declare_parameter<std::string>("init_pose_from_topic", "/base_pose_ground_truth");
  this->get_parameter("init_pose_from_topic", init_pose_from_topic);
  this->declare_parameter<double>("freq", 10.0);
  this->get_parameter("freq", freq);
  pose_covariance_diagonal = this->declare_parameter<std::vector<double>>(
    "pose_covariance_diagonal", {0.05, 0.05, 1.0e6, 1.0e6, 1.0e6, 0.10});
  twist_covariance_diagonal = this->declare_parameter<std::vector<double>>(
    "twist_covariance_diagonal", {0.10, 0.10, 1.0e6, 1.0e6, 1.0e6, 0.20});
  validateCovariance(pose_covariance_diagonal, "pose_covariance_diagonal");
  validateCovariance(twist_covariance_diagonal, "twist_covariance_diagonal");

  // Init Publishers and Subscribers
  //---------------------------------
  buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *buffer_, this, false);
  odom_pub  = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 5);
  laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_scan_topic,rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
      std::bind(&CLaserOdometry2DNode::LaserCallBack, this, std::placeholders::_1));

  // Initialize pose
  if (init_pose_from_topic != "")
  {
    initPose_sub = this->create_subscription<nav_msgs::msg::Odometry>(init_pose_from_topic,rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
        std::bind(&CLaserOdometry2DNode::initPoseCallBack, this, std::placeholders::_1));
    GT_pose_initialized  = false;
  }
  else
  {
    // init to 0
    GT_pose_initialized = true;
    initial_robot_pose.pose.pose.position.x = 0;
    initial_robot_pose.pose.pose.position.y = 0;
    initial_robot_pose.pose.pose.position.z = 0;
    initial_robot_pose.pose.pose.orientation.w = 1;
    initial_robot_pose.pose.pose.orientation.x = 0;
    initial_robot_pose.pose.pose.orientation.y = 0;
    initial_robot_pose.pose.pose.orientation.z = 0;
  }

  // Init variables
  rf2o_ref.module_initialized = false;
  rf2o_ref.first_laser_scan   = true;
}


void CLaserOdometry2DNode::validateCovariance(
  const std::vector<double>& diagonal,
  const std::string& parameter_name) const
{
  if (diagonal.size() != 6)
  {
    throw std::invalid_argument(parameter_name + " must contain six values");
  }
  for (const double value : diagonal)
  {
    if (!std::isfinite(value) || value <= 0.0)
    {
      throw std::invalid_argument(
        parameter_name + " values must be finite and positive");
    }
  }
}


/**
 * Keeps the last scan from the 2D lidar to be latter processed
 * On the first laser scan, the node is initialized.
*/
void CLaserOdometry2DNode::LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr new_scan)
{
  if (GT_pose_initialized)
  {
    const int64_t scan_stamp_ns = rclcpp::Time(new_scan->header.stamp).nanoseconds();
    if (!rf2o_ref.first_laser_scan && scan_stamp_ns <= last_scan_stamp_ns)
    {
      RCLCPP_WARN(get_logger(), "Ignoring non-monotonic laser scan timestamp");
      return;
    }

    if (rf2o_ref.first_laser_scan)
    {
      // A valid base_link <- scan transform is required before any algorithm
      // initialization or odometry publication.
      last_scan = *new_scan;
      if (!setLaserPoseFromTf())
        return;
      rf2o_ref.current_scan_time = last_scan.header.stamp;
      rf2o_ref.init(last_scan, initial_robot_pose.pose.pose);
      rf2o_ref.first_laser_scan = false;
      last_scan_stamp_ns = scan_stamp_ns;
      return;
    }

    if (new_scan->ranges.size() != rf2o_ref.width)
    {
      RCLCPP_WARN(
        get_logger(), "Ignoring laser scan whose sample count changed");
      return;
    }

    // Keep in memory only a newer, structurally compatible laser scan.
    last_scan = *new_scan;
    rf2o_ref.current_scan_time = last_scan.header.stamp;
    last_scan_stamp_ns = scan_stamp_ns;
    new_scan_available = true;
  }
}


/**
   * Gets the laser pose with respect the base_link (through TF)
   * This allow estimation of the odometry with respect to the robot base reference system.
   */
bool CLaserOdometry2DNode::setLaserPoseFromTf()
{
  bool retrieved = false;
  geometry_msgs::msg::TransformStamped tf_laser;

  try
  {
    tf_laser = buffer_->lookupTransform(base_frame_id, last_scan.header.frame_id, tf2::TimePointZero);
    retrieved = true;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Waiting for %s <- %s transform: %s",
      base_frame_id.c_str(), last_scan.header.frame_id.c_str(), ex.what());
    return false;
  }

  // Keep this transform as Eigen Matrix3d
  tf2::Transform transform;
  tf2::convert(tf_laser.transform, transform);
  const tf2::Matrix3x3 &basis = transform.getBasis();
  Eigen::Matrix3d R;

  for(int r = 0; r < 3; r++)
    for(int c = 0; c < 3; c++)
      R(r,c) = basis[r][c];

  Pose3d laser_tf(R);

  const tf2::Vector3 &t = transform.getOrigin();
  laser_tf.translation()(0) = t[0];
  laser_tf.translation()(1) = t[1];
  laser_tf.translation()(2) = t[2];

  // Sets this transform in rf2o
  rf2o_ref.setLaserPose(laser_tf);

  return retrieved;
}


bool CLaserOdometry2DNode::scan_available()
{
  return new_scan_available;
}


/**
 * Process the last scans to estimate the current odometry
*/
void CLaserOdometry2DNode::process()
{
  // Do only run when a new scan is ready
  if( rf2o_ref.is_initialized() && scan_available() )
  {
    new_scan_available = false;
    // Process odometry estimation
    if (!rf2o_ref.odometryCalculation(last_scan))
      return;

    // Publish topic-only odometry. The EKF remains the sole odom TF owner.
    publish();
  }
}


/**
 * This function is used to initialize the robot pose before estimating its odometry.
 * By default the odometry will start from pose_0, but when comparing different methods
 * it may be necessary to start from a different pose.
*/
void CLaserOdometry2DNode::initPoseCallBack(const nav_msgs::msg::Odometry::SharedPtr new_initPose)
{
  // Initialize module on first GT pose. Else do Nothing!
  if (!GT_pose_initialized)
  {
    initial_robot_pose = *new_initPose;
    GT_pose_initialized = true;
  }
}


/**
 * Publish current odocmetry estimation over ROS
 * According to the node parameters it will publish over tf and/or especified topic
*/
void CLaserOdometry2DNode::publish()
{
  // 1. publish odom as a topic (no harm!)
  RCLCPP_DEBUG(get_logger(), "Publishing odom over topic:[%s]", odom_topic.c_str());
  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(0.0, 0.0, rf2o::getYaw(rf2o_ref.robot_pose_.rotation()));
  geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(tf_quaternion);

  // compose odom msg
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = rf2o_ref.last_odom_time;    // the time of the last scan used!
  odom.header.frame_id = odom_frame_id;
  //set the position
  odom.pose.pose.position.x = rf2o_ref.robot_pose_.translation()(0);
  odom.pose.pose.position.y = rf2o_ref.robot_pose_.translation()(1);
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = quaternion;
  //set the velocity
  odom.child_frame_id = base_frame_id;
  odom.twist.twist.linear.x = rf2o_ref.lin_speed;    //linear speed
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = rf2o_ref.ang_speed;   //angular speed
  for (size_t index = 0; index < 6; ++index)
  {
    odom.pose.covariance[index * 6 + index] =
      pose_covariance_diagonal[index];
    odom.twist.covariance[index * 6 + index] =
      twist_covariance_diagonal[index];
  }
  if (!std::isfinite(odom.pose.pose.position.x) ||
      !std::isfinite(odom.pose.pose.position.y) ||
      !std::isfinite(odom.pose.pose.orientation.z) ||
      !std::isfinite(odom.pose.pose.orientation.w) ||
      !std::isfinite(odom.twist.twist.linear.x) ||
      !std::isfinite(odom.twist.twist.angular.z))
  {
    RCLCPP_ERROR(get_logger(), "RF2O produced non-finite odometry; dropping sample");
    return;
  }
  //publish the message
  odom_pub->publish(odom);
}

} /* namespace rf2o */


//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto myLaserOdomNode = std::make_shared<rf2o::CLaserOdometry2DNode>();

  // set desired loop rate
  rclcpp::Rate rate(myLaserOdomNode->freq);

  // Loop
  while (rclcpp::ok()){
      rclcpp::spin_some(myLaserOdomNode);
      myLaserOdomNode->process();
      rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
