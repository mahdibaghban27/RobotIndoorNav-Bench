#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "g1_cogar_nav_benchmark/geometry_utils.hpp"

namespace fs = std::filesystem;

namespace g1_cogar_nav_benchmark
{
namespace
{

class BenchmarkLoggerNode : public rclcpp::Node
{
public:
  BenchmarkLoggerNode()
  : Node("benchmark_logger_node")
  {
    declare_parameter<std::string>("odom_topic", "/odom");
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("reflex_active_topic", "/reflex_active");
    declare_parameter<std::string>("output_file", "/tmp/k3_benchmark_log.csv");
    declare_parameter<double>("collision_distance", 0.18);
    declare_parameter<double>("write_rate_hz", 20.0);

    output_file_ = get_parameter("output_file").as_string();
    collision_distance_ = get_parameter("collision_distance").as_double();
    write_rate_hz_ = get_parameter("write_rate_hz").as_double();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), 10,
      std::bind(&BenchmarkLoggerNode::on_odom, this, std::placeholders::_1));
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      get_parameter("cmd_vel_topic").as_string(), 10,
      std::bind(&BenchmarkLoggerNode::on_cmd, this, std::placeholders::_1));
   scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
  get_parameter("scan_topic").as_string(),
  rclcpp::SensorDataQoS(),
  std::bind(&BenchmarkLoggerNode::on_scan, this, std::placeholders::_1));
    reflex_sub_ = create_subscription<std_msgs::msg::Bool>(
      get_parameter("reflex_active_topic").as_string(), 10,
      std::bind(&BenchmarkLoggerNode::on_reflex, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / write_rate_hz_),
      std::bind(&BenchmarkLoggerNode::on_timer, this));
    service_ = create_service<std_srvs::srv::SetBool>(
      "benchmark_logger/enable",
      std::bind(
        &BenchmarkLoggerNode::on_enable, this, std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Benchmark logger ready. Output file: %s", output_file_.c_str());
  }

  ~BenchmarkLoggerNode() override
  {
    close_csv();
  }

private:
  double now_s()
  {
    return static_cast<double>(this->now().nanoseconds()) / 1e9;
  }

  void open_csv()
  {
    close_csv();
    fs::path path(output_file_);
    fs::create_directories(path.parent_path());
    csv_.open(path, std::ios::out | std::ios::trunc);
    if (!csv_.is_open()) {
      throw std::runtime_error("Unable to open benchmark CSV for writing: " + output_file_);
    }
    csv_ << "t,dt,x,y,yaw,linear_x,angular_z,min_scan,reflex_active,collision_distance\n";
    csv_.flush();
  }

  void close_csv()
  {
    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
    }
  }

  void on_enable(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (request->data && !enabled_) {
      enabled_ = true;
      started_at_ = now_s();
      last_t_ = std::numeric_limits<double>::quiet_NaN();
      open_csv();
      response->success = true;
      response->message = "Logging started: " + output_file_;
    } else if (!request->data && enabled_) {
      enabled_ = false;
      close_csv();
      response->success = true;
      response->message = "Logging stopped.";
    } else {
      response->success = true;
      response->message = "No state change.";
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = msg;
  }

  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_ = *msg;
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double best = msg->range_max > 0.0 ? msg->range_max : 999.0;
    bool found = false;
    for (const auto value : msg->ranges) {
      if (std::isfinite(value) && value > 0.01 && value < best) {
        best = value;
        found = true;
      }
    }
    last_scan_min_ = found ? best : best;
  }

  void on_reflex(const std_msgs::msg::Bool::SharedPtr msg)
  {
    last_reflex_active_ = msg->data ? 1.0 : 0.0;
  }

  void on_timer()
  {
    if (!enabled_ || !last_odom_ || !csv_.is_open()) {
      return;
    }

    const double t_now = now_s();
    const double t_rel = t_now - started_at_;
    const double dt = std::isnan(last_t_) ? 0.0 : (t_now - last_t_);
    last_t_ = t_now;

    const auto & pose = last_odom_->pose.pose;
    const double yaw = yaw_from_quaternion(pose.orientation);

    csv_ << std::fixed << std::setprecision(6)
         << t_rel << ',' << dt << ','
         << pose.position.x << ',' << pose.position.y << ',' << yaw << ','
         << last_cmd_.linear.x << ',' << last_cmd_.angular.z << ','
         << last_scan_min_ << ',' << last_reflex_active_ << ',' << collision_distance_ << '\n';
    csv_.flush();
  }

  std::string output_file_;
  double collision_distance_{0.18};
  double write_rate_hz_{20.0};
  bool enabled_{false};
  double started_at_{0.0};
  double last_t_{std::numeric_limits<double>::quiet_NaN()};
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  geometry_msgs::msg::Twist last_cmd_;
  double last_scan_min_{999.0};
  double last_reflex_active_{0.0};
  std::ofstream csv_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reflex_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace g1_cogar_nav_benchmark

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<g1_cogar_nav_benchmark::BenchmarkLoggerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
