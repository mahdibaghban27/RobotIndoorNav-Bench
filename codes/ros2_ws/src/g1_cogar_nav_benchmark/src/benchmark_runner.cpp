#include <algorithm>
#include <chrono>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "g1_cogar_nav_benchmark/geometry_utils.hpp"
#include "g1_cogar_nav_benchmark/scenario_loader.hpp"
#include "g1_cogar_nav_benchmark/topological_graph.hpp"

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace g1_cogar_nav_benchmark
{
namespace
{

enum class NavigationOutcome
{
  Succeeded,
  Failed,
  TimedOut,
};

struct RunnerOptions
{
  std::string scenario_file;
  std::string scenario_id;
  std::string planner_id;
  std::string result_dir;
  double startup_wait{4.0};
  double goal_timeout{240.0};
  bool set_initial_pose{true};
  std::string logger_output_file{"/tmp/k3_latest_log.csv"};
};

std::map<std::string, std::string> parse_cli(int argc, char ** argv)
{
  std::map<std::string, std::string> options;
  for (int i = 1; i < argc; ++i) {
    const std::string token(argv[i]);
    if (token == "--set-initial-pose") {
      options[token] = "true";
      continue;
    }
    if (token.rfind("--", 0) == 0) {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for argument: " + token);
      }
      options[token] = argv[++i];
    }
  }
  return options;
}

bool option_to_bool(const std::string & value)
{
  const auto lower = [&value]() {
      std::string out = value;
      std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {return std::tolower(c);});
      return out;
    }();
  return lower == "1" || lower == "true" || lower == "yes" || lower == "on";
}

RunnerOptions make_runner_options(const std::vector<std::string> & argv)
{
  std::vector<char *> raw;
  raw.reserve(argv.size());
  for (const auto & arg : argv) {
    raw.push_back(const_cast<char *>(arg.c_str()));
  }
  auto options = parse_cli(static_cast<int>(raw.size()), raw.data());

  for (const auto & required : {"--scenario-file", "--scenario-id", "--planner-id", "--result-dir"}) {
    if (options.find(required) == options.end()) {
      throw std::runtime_error(std::string("Required argument missing: ") + required);
    }
  }

  RunnerOptions out;
  out.scenario_file = options.at("--scenario-file");
  out.scenario_id = options.at("--scenario-id");
  out.planner_id = options.at("--planner-id");
  out.result_dir = options.at("--result-dir");
  if (options.count("--startup-wait") != 0U) {
    out.startup_wait = std::stod(options.at("--startup-wait"));
  }
  if (options.count("--goal-timeout") != 0U) {
    out.goal_timeout = std::stod(options.at("--goal-timeout"));
  }
  if (options.count("--set-initial-pose") != 0U) {
    out.set_initial_pose = option_to_bool(options.at("--set-initial-pose"));
  }
  if (options.count("--logger-output-file") != 0U) {
    out.logger_output_file = options.at("--logger-output-file");
  }
  return out;
}

class BenchmarkRunnerNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  BenchmarkRunnerNode()
  : Node("benchmark_runner_helper")
  {
    logger_client_ = create_client<std_srvs::srv::SetBool>("benchmark_logger/enable");
    initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    final_stop_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    nav_stop_pub_ = create_publisher<geometry_msgs::msg::Twist>("/nav_cmd_vel", 10);
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  }

  bool toggle_logger(bool enable, double timeout_s)
  {
    if (!logger_client_->wait_for_service(std::chrono::duration<double>(timeout_s))) {
      RCLCPP_ERROR(get_logger(), "benchmark_logger/enable service not available");
      return false;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enable;
    auto future = logger_client_->async_send_request(request);
    const auto rc = rclcpp::spin_until_future_complete(shared_from_this(), future, std::chrono::duration<double>(timeout_s));
    if (rc != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to toggle benchmark logger service");
      return false;
    }
    const auto response = future.get();
    if (!response->success) {
      RCLCPP_ERROR(get_logger(), "Benchmark logger rejected request: %s", response->message.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    return true;
  }

  bool wait_for_navigation_server(double timeout_s)
  {
    return nav_client_->wait_for_action_server(std::chrono::duration<double>(timeout_s));
  }

  void publish_initial_pose(const std::array<double, 3> & pose)
  {
    const auto msg = make_initial_pose(*this, pose);
    for (int i = 0; i < 5; ++i) {
      initial_pose_pub_->publish(msg);
      rclcpp::sleep_for(200ms);
      rclcpp::spin_some(shared_from_this());
    }
  }

  NavigationOutcome navigate_to_pose(const geometry_msgs::msg::PoseStamped & pose, double timeout_s)
  {
    NavigateToPose::Goal goal;
    goal.pose = pose;
    current_recovery_count_ = 0;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_options;
    send_options.feedback_callback =
      [this](GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        current_recovery_count_ = std::max(
          current_recovery_count_, static_cast<int>(feedback->number_of_recoveries));
      };

    auto goal_future = nav_client_->async_send_goal(goal, send_options);
    const auto goal_rc = rclcpp::spin_until_future_complete(shared_from_this(), goal_future, 10s);
    if (goal_rc != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Timed out while sending navigation goal");
      return NavigationOutcome::Failed;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Navigation goal was rejected by Nav2");
      return NavigationOutcome::Failed;
    }

    auto result_future = nav_client_->async_get_result(goal_handle);
    const auto started = now();
    while (rclcpp::ok()) {
      const auto status = rclcpp::spin_until_future_complete(shared_from_this(), result_future, 250ms);
      if (status == rclcpp::FutureReturnCode::SUCCESS) {
        const auto result = result_future.get();
        total_recovery_count_ += current_recovery_count_;
        publish_stop_burst();
        return result.code == rclcpp_action::ResultCode::SUCCEEDED ?
               NavigationOutcome::Succeeded :
               NavigationOutcome::Failed;
      }

      if ((now() - started).seconds() > timeout_s) {
        RCLCPP_WARN(get_logger(), "Goal timeout reached; canceling current navigation task.");
        auto cancel_future = nav_client_->async_cancel_goal(goal_handle);
        rclcpp::spin_until_future_complete(shared_from_this(), cancel_future, 5s);
        total_recovery_count_ += current_recovery_count_;
        publish_stop_burst();
        return NavigationOutcome::TimedOut;
      }
    }

    total_recovery_count_ += current_recovery_count_;
    publish_stop_burst();
    return NavigationOutcome::Failed;
  }

  int total_recovery_count() const
  {
    return total_recovery_count_;
  }

private:
  void publish_stop_burst()
  {
    const geometry_msgs::msg::Twist stop;
    for (int i = 0; i < 10; ++i) {
      final_stop_pub_->publish(stop);
      nav_stop_pub_->publish(stop);
      rclcpp::sleep_for(50ms);
      rclcpp::spin_some(shared_from_this());
    }
  }

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr logger_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr final_stop_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr nav_stop_pub_;
  int current_recovery_count_{0};
  int total_recovery_count_{0};
};

std::string join_route(const std::vector<std::string> & route)
{
  std::ostringstream oss;
  for (std::size_t i = 0; i < route.size(); ++i) {
    if (i != 0U) {
      oss << " -> ";
    }
    oss << route[i];
  }
  return oss.str();
}

void write_run_metadata(
  const fs::path & path,
  const Scenario & scenario,
  const std::string & planner_id,
  bool success,
  const std::vector<std::string> & route,
  std::size_t num_goals,
  std::size_t goals_reached,
  int recovery_count,
  const std::string & failure_reason)
{
  std::ofstream out(path, std::ios::out | std::ios::trunc);
  if (!out.is_open()) {
    throw std::runtime_error("Unable to write run metadata: " + path.string());
  }

  out << "scenario_id: " << scenario.scenario_id << '\n';
  out << "planner_id: " << planner_id << '\n';
  out << "success: " << (success ? 1 : 0) << '\n';
  out << "failure_reason: " << failure_reason << '\n';
  out << "num_goals: " << num_goals << '\n';
  out << "goals_reached: " << goals_reached << '\n';
  out << "recovery_count: " << recovery_count << '\n';
  out << "recovery_success: " << ((recovery_count == 0 || success) ? 1 : 0) << '\n';
  out << "start_pose: [" << scenario.start_pose[0] << ", " << scenario.start_pose[1] << ", " << scenario.start_pose[2] << "]\n";
  out << "goal_pose: [" << scenario.goal_pose[0] << ", " << scenario.goal_pose[1] << ", " << scenario.goal_pose[2] << "]\n";
  out << "world: " << scenario.world << '\n';
  out << "map_yaml: " << scenario.map_yaml << '\n';
  out << "notes: " << scenario.notes << '\n';
  out << "topological_graph: " << scenario.topological_graph << '\n';
  out << "use_topological_route: " << (scenario.use_topological_route ? 1 : 0) << '\n';
  out << "route: [";
  for (std::size_t i = 0; i < route.size(); ++i) {
    if (i != 0U) {
      out << ", ";
    }
    out << route[i];
  }
  out << "]\n";
}

}  // namespace

}  // namespace g1_cogar_nav_benchmark

int main(int argc, char ** argv)
{
  try {
    const auto filtered_args = rclcpp::remove_ros_arguments(argc, argv);
    const auto options = g1_cogar_nav_benchmark::make_runner_options(filtered_args);
    const auto scenarios = g1_cogar_nav_benchmark::load_scenarios(options.scenario_file);
    const auto scenario_it = scenarios.find(options.scenario_id);
    if (scenario_it == scenarios.end()) {
      throw std::runtime_error("Scenario id not found: " + options.scenario_id);
    }
    const auto & scenario = scenario_it->second;

    fs::create_directories(options.result_dir);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<g1_cogar_nav_benchmark::BenchmarkRunnerNode>();

    RCLCPP_INFO(
      node->get_logger(), "Running scenario=%s planner=%s",
      scenario.scenario_id.c_str(), options.planner_id.c_str());
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(options.startup_wait)));

    if (!node->wait_for_navigation_server(20.0)) {
      throw std::runtime_error("Nav2 action server 'navigate_to_pose' is not available.");
    }

    if (options.set_initial_pose) {
      node->publish_initial_pose(scenario.start_pose);
      rclcpp::sleep_for(1s);
    }

    std::vector<geometry_msgs::msg::PoseStamped> route_waypoints;
    std::vector<std::string> symbolic_route;
    if (scenario.use_topological_route && !scenario.topological_graph.empty()) {
      g1_cogar_nav_benchmark::TopologicalGraph graph(scenario.topological_graph);
      symbolic_route = graph.plan_route(scenario.start_area, scenario.goal_area);
      const auto waypoints = graph.route_to_waypoints(symbolic_route);
      if (waypoints.size() > 1U) {
        for (std::size_t i = 1; i < waypoints.size(); ++i) {
          route_waypoints.push_back(g1_cogar_nav_benchmark::make_pose_stamped(*node, waypoints[i]));
        }
        RCLCPP_INFO(node->get_logger(), "Topological route: %s", g1_cogar_nav_benchmark::join_route(symbolic_route).c_str());
      }
    }

    if (route_waypoints.empty()) {
      route_waypoints.push_back(g1_cogar_nav_benchmark::make_pose_stamped(*node, scenario.goal_pose));
    }

    if (!node->toggle_logger(true, 5.0)) {
      throw std::runtime_error("Failed to enable benchmark logger.");
    }

    bool success = true;
    std::size_t goals_reached = 0;
    std::string failure_reason = "none";
    for (std::size_t i = 0; i < route_waypoints.size(); ++i) {
      RCLCPP_INFO(node->get_logger(), "Dispatching waypoint %zu / %zu", i + 1U, route_waypoints.size());
      const auto outcome = node->navigate_to_pose(route_waypoints[i], options.goal_timeout);
      if (outcome != g1_cogar_nav_benchmark::NavigationOutcome::Succeeded) {
        success = false;
        failure_reason = outcome == g1_cogar_nav_benchmark::NavigationOutcome::TimedOut ?
          "timeout" : "navigation_failed";
        break;
      }
      ++goals_reached;
      rclcpp::sleep_for(250ms);
    }

    if (!node->toggle_logger(false, 5.0)) {
      RCLCPP_WARN(node->get_logger(), "Failed to disable benchmark logger cleanly; continuing.");
    }

    const fs::path logger_src = options.logger_output_file;
    const fs::path logger_dst = fs::path(options.result_dir) / "benchmark_log.csv";
    try {
      if (fs::exists(logger_src)) {
        fs::copy_file(logger_src, logger_dst, fs::copy_options::overwrite_existing);
        RCLCPP_INFO(node->get_logger(), "Copied benchmark log to %s", logger_dst.c_str());
      } else {
        RCLCPP_WARN(node->get_logger(), "Logger output file not found at %s", logger_src.c_str());
      }
    } catch (const std::exception & ex) {
      RCLCPP_WARN(node->get_logger(), "Could not copy benchmark log: %s", ex.what());
    }

    const fs::path metadata_path = fs::path(options.result_dir) / "run_metadata.yaml";
    g1_cogar_nav_benchmark::write_run_metadata(
      metadata_path, scenario, options.planner_id, success, symbolic_route,
      route_waypoints.size(), goals_reached, node->total_recovery_count(), failure_reason);

    RCLCPP_INFO(node->get_logger(), "Run completed. success=%s", success ? "true" : "false");
    rclcpp::shutdown();
    return success ? 0 : 2;
  } catch (const std::exception & ex) {
    std::cerr << "benchmark_runner error: " << ex.what() << std::endl;
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
}
