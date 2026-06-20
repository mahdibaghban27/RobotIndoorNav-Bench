#ifndef G1_COGAR_NAV_BENCHMARK__GEOMETRY_UTILS_HPP_
#define G1_COGAR_NAV_BENCHMARK__GEOMETRY_UTILS_HPP_

#include <array>
#include <cmath>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"

namespace g1_cogar_nav_benchmark
{

inline geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

inline double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

inline geometry_msgs::msg::PoseStamped make_pose_stamped(
  const rclcpp::Node & node,
  const std::array<double, 3> & pose,
  const std::string & frame_id = "map")
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = node.now();
  msg.pose.position.x = pose[0];
  msg.pose.position.y = pose[1];
  msg.pose.position.z = 0.0;
  msg.pose.orientation = quaternion_from_yaw(pose[2]);
  return msg;
}

inline geometry_msgs::msg::PoseWithCovarianceStamped make_initial_pose(
  const rclcpp::Node & node,
  const std::array<double, 3> & pose,
  const std::string & frame_id = "map")
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = node.now();
  msg.pose.pose.position.x = pose[0];
  msg.pose.pose.position.y = pose[1];
  msg.pose.pose.position.z = 0.0;
  msg.pose.pose.orientation = quaternion_from_yaw(pose[2]);
  msg.pose.covariance.fill(0.0);
  msg.pose.covariance[0] = 0.25;
  msg.pose.covariance[7] = 0.25;
  msg.pose.covariance[35] = 0.06853891945200942;
  return msg;
}

}  // namespace g1_cogar_nav_benchmark

#endif  // G1_COGAR_NAV_BENCHMARK__GEOMETRY_UTILS_HPP_
