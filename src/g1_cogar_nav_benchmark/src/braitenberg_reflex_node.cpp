#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

namespace g1_cogar_nav_benchmark
{
namespace
{

double sector_min(
  const std::vector<float> & ranges,
  double angle_min,
  double angle_increment,
  double center_deg,
  double width_deg,
  double max_range)
{
  if (ranges.empty()) {
    return max_range;
  }

  constexpr double kPi = 3.14159265358979323846;
  const double start = (center_deg - width_deg * 0.5) * kPi / 180.0;
  const double stop = (center_deg + width_deg * 0.5) * kPi / 180.0;
  const int idx0 = std::max(0, static_cast<int>((start - angle_min) / angle_increment));
  const int idx1 = std::min(
    static_cast<int>(ranges.size()),
    static_cast<int>((stop - angle_min) / angle_increment) + 1);

  if (idx1 <= idx0) {
    return max_range;
  }

  double best = max_range;
  bool found = false;
  for (int idx = idx0; idx < idx1; ++idx) {
    const double value = ranges[static_cast<std::size_t>(idx)];
    if (std::isfinite(value) && value > 0.01 && value < best) {
      best = value;
      found = true;
    }
  }
  return found ? best : max_range;
}

class BraitenbergReflexNode : public rclcpp::Node
{
public:
  BraitenbergReflexNode()
  : Node("braitenberg_reflex_node"), currently_active_(false)
  {
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("cmd_out_topic", "/reflex_cmd_vel");
    declare_parameter<std::string>("active_topic", "/reflex_active");
    declare_parameter<double>("activation_distance", 0.85);
    declare_parameter<double>("stop_distance", 0.35);
    declare_parameter<double>("clear_distance", 1.10);
    declare_parameter<double>("max_linear", 0.40);
    declare_parameter<double>("max_angular", 1.25);
    declare_parameter<double>("front_sector_deg", 40.0);
    declare_parameter<double>("side_sector_deg", 50.0);
    declare_parameter<bool>("publish_zero_when_inactive", true);

    scan_topic_ = get_parameter("scan_topic").as_string();
    cmd_out_topic_ = get_parameter("cmd_out_topic").as_string();
    active_topic_ = get_parameter("active_topic").as_string();
    activation_distance_ = get_parameter("activation_distance").as_double();
    stop_distance_ = get_parameter("stop_distance").as_double();
    clear_distance_ = get_parameter("clear_distance").as_double();
    max_linear_ = get_parameter("max_linear").as_double();
    max_angular_ = get_parameter("max_angular").as_double();
    front_sector_deg_ = get_parameter("front_sector_deg").as_double();
    side_sector_deg_ = get_parameter("side_sector_deg").as_double();
    publish_zero_when_inactive_ = get_parameter("publish_zero_when_inactive").as_bool();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_out_topic_, 10);
    active_pub_ = create_publisher<std_msgs::msg::Bool>(active_topic_, 10);
    min_range_pub_ = create_publisher<std_msgs::msg::Float32>("/reflex_min_range", 10);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
  scan_topic_,
  rclcpp::SensorDataQoS(),
  std::bind(&BraitenbergReflexNode::on_scan, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(),
      "Braitenberg reflex active: scan=%s cmd_out=%s active_topic=%s",
      scan_topic_.c_str(), cmd_out_topic_.c_str(), active_topic_.c_str());
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const double max_range = msg->range_max > 0.0 ? msg->range_max : 20.0;
    const double left_min = sector_min(
      msg->ranges, msg->angle_min, msg->angle_increment, 55.0, side_sector_deg_, max_range);
    const double right_min = sector_min(
      msg->ranges, msg->angle_min, msg->angle_increment, -55.0, side_sector_deg_, max_range);
    const double front_min = sector_min(
      msg->ranges, msg->angle_min, msg->angle_increment, 0.0, front_sector_deg_, max_range);
    const double overall_min = std::min({left_min, right_min, front_min});

    bool active = currently_active_;
    if (active) {
      active = overall_min < clear_distance_;
    } else {
      active = overall_min < activation_distance_;
    }

    geometry_msgs::msg::Twist cmd;
    if (active) {
      const double left_signal = std::max(0.0, (activation_distance_ - left_min) / activation_distance_);
      const double right_signal = std::max(0.0, (activation_distance_ - right_min) / activation_distance_);
      const double front_signal = std::max(0.0, (activation_distance_ - front_min) / activation_distance_);

      if (front_min < stop_distance_) {
        cmd.linear.x = -0.05;
      } else {
        cmd.linear.x = std::max(0.0, max_linear_ * (1.0 - front_signal));
      }

      double turn = left_signal - right_signal;
      if (std::abs(turn) < 1e-3 && front_signal > 0.0) {
        turn = (left_min > right_min) ? 1.0 : -1.0;
      }
      cmd.angular.z = std::clamp(-max_angular_ * turn, -max_angular_, max_angular_);
    } else if (publish_zero_when_inactive_) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    }

    currently_active_ = active;

    std_msgs::msg::Bool active_msg;
    active_msg.data = active;
    std_msgs::msg::Float32 min_msg;
    min_msg.data = static_cast<float>(overall_min);

    cmd_pub_->publish(cmd);
    active_pub_->publish(active_msg);
    min_range_pub_->publish(min_msg);
  }

  std::string scan_topic_;
  std::string cmd_out_topic_;
  std::string active_topic_;
  double activation_distance_{0.85};
  double stop_distance_{0.35};
  double clear_distance_{1.10};
  double max_linear_{0.40};
  double max_angular_{1.25};
  double front_sector_deg_{40.0};
  double side_sector_deg_{50.0};
  bool publish_zero_when_inactive_{true};
  bool currently_active_{false};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr min_range_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

}  // namespace
}  // namespace g1_cogar_nav_benchmark

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<g1_cogar_nav_benchmark::BraitenbergReflexNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
