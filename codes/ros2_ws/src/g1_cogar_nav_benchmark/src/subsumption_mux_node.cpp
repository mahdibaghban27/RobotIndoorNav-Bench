#include <algorithm>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace g1_cogar_nav_benchmark
{
namespace
{

class SubsumptionMuxNode : public rclcpp::Node
{
public:
  SubsumptionMuxNode()
  : Node("subsumption_mux_node")
  {
    declare_parameter<std::string>("nav_cmd_topic", "/nav_cmd_vel");
    declare_parameter<std::string>("reflex_cmd_topic", "/reflex_cmd_vel");
    declare_parameter<std::string>("reflex_active_topic", "/reflex_active");
    declare_parameter<std::string>("cmd_out_topic", "/cmd_vel");
    declare_parameter<double>("publish_rate_hz", 20.0);
    declare_parameter<double>("timeout_s", 0.5);

    nav_cmd_topic_ = get_parameter("nav_cmd_topic").as_string();
    reflex_cmd_topic_ = get_parameter("reflex_cmd_topic").as_string();
    reflex_active_topic_ = get_parameter("reflex_active_topic").as_string();
    cmd_out_topic_ = get_parameter("cmd_out_topic").as_string();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    timeout_s_ = get_parameter("timeout_s").as_double();

    const double now = now_s();
    last_nav_ = now;
    last_reflex_ = now;

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_out_topic_, 10);
    mode_pub_ = create_publisher<std_msgs::msg::String>("/control_mode", 10);

    nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      nav_cmd_topic_, 10, std::bind(&SubsumptionMuxNode::on_nav, this, std::placeholders::_1));
    reflex_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      reflex_cmd_topic_, 10, std::bind(&SubsumptionMuxNode::on_reflex, this, std::placeholders::_1));
    reflex_active_sub_ = create_subscription<std_msgs::msg::Bool>(
      reflex_active_topic_, 10,
      std::bind(&SubsumptionMuxNode::on_reflex_active, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&SubsumptionMuxNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(), "Subsumption mux: nav=%s reflex=%s out=%s",
      nav_cmd_topic_.c_str(), reflex_cmd_topic_.c_str(), cmd_out_topic_.c_str());
  }

private:
  double now_s()
  {
    return static_cast<double>(this->now().nanoseconds()) / 1e9;
  }

  void on_nav(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    nav_msg_ = *msg;
    last_nav_ = now_s();
  }

  void on_reflex(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    reflex_msg_ = *msg;
    last_reflex_ = now_s();
  }

  void on_reflex_active(const std_msgs::msg::Bool::SharedPtr msg)
  {
    reflex_active_ = msg->data;
  }

  void on_timer()
  {
    const double time_now = now_s();
    const bool nav_valid = (time_now - last_nav_) < timeout_s_;
    const bool reflex_valid = (time_now - last_reflex_) < timeout_s_;

    geometry_msgs::msg::Twist out;
    std_msgs::msg::String mode;
    mode.data = "idle";

    if (reflex_active_ && reflex_valid) {
      out = reflex_msg_;
      mode.data = "reactive_reflex";
    } else if (nav_valid) {
      out = nav_msg_;
      mode.data = "deliberative_nav2";
    }

    cmd_pub_->publish(out);
    mode_pub_->publish(mode);
  }

  std::string nav_cmd_topic_;
  std::string reflex_cmd_topic_;
  std::string reflex_active_topic_;
  std::string cmd_out_topic_;
  double publish_rate_hz_{20.0};
  double timeout_s_{0.5};

  geometry_msgs::msg::Twist nav_msg_;
  geometry_msgs::msg::Twist reflex_msg_;
  bool reflex_active_{false};
  double last_nav_{0.0};
  double last_reflex_{0.0};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr reflex_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reflex_active_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace g1_cogar_nav_benchmark

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<g1_cogar_nav_benchmark::SubsumptionMuxNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
