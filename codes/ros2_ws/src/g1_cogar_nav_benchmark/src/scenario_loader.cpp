#include "g1_cogar_nav_benchmark/scenario_loader.hpp"

#include <filesystem>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

namespace g1_cogar_nav_benchmark
{
namespace
{

std::array<double, 3> read_pose3(const YAML::Node & node)
{
  if (!node || !node.IsSequence() || node.size() != 3U) {
    throw std::runtime_error("Expected a 3-element pose sequence [x, y, yaw].");
  }
  return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
}

std::string resolve_relative_path(const fs::path & base_dir, const std::string & value)
{
  if (value.empty()) {
    return value;
  }
  const fs::path path_value(value);
  if (path_value.is_absolute()) {
    return path_value.string();
  }
  return fs::weakly_canonical(base_dir / path_value).string();
}

}  // namespace

std::map<std::string, Scenario> load_scenarios(const std::string & path)
{
  const fs::path scenario_file = fs::absolute(path);
  const fs::path scenario_dir = scenario_file.parent_path();
  const YAML::Node root = YAML::LoadFile(scenario_file.string());
  const YAML::Node scenarios_node = root["scenarios"];
  if (!scenarios_node || !scenarios_node.IsMap()) {
    throw std::runtime_error("Scenario file does not contain a 'scenarios' map.");
  }

  std::map<std::string, Scenario> scenarios;
  for (const auto & item : scenarios_node) {
    const std::string scenario_id = item.first.as<std::string>();
    const YAML::Node cfg = item.second;

    Scenario scenario;
    scenario.scenario_id = scenario_id;
    scenario.world = resolve_relative_path(scenario_dir, cfg["world"].as<std::string>());
    scenario.map_yaml = resolve_relative_path(scenario_dir, cfg["map_yaml"].as<std::string>());
    scenario.start_pose = read_pose3(cfg["start_pose"]);
    scenario.goal_pose = read_pose3(cfg["goal_pose"]);
    scenario.start_area = cfg["start_area"].as<std::string>();
    scenario.goal_area = cfg["goal_area"].as<std::string>();
    scenario.description = cfg["description"].as<std::string>();
    scenario.dynamic_obstacles = cfg["dynamic_obstacles"] ? cfg["dynamic_obstacles"].as<bool>() : false;
    scenario.use_topological_route = cfg["use_topological_route"] ? cfg["use_topological_route"].as<bool>() : false;
    scenario.notes = cfg["notes"] ? cfg["notes"].as<std::string>() : std::string();
    scenario.topological_graph = cfg["topological_graph"] ?
      resolve_relative_path(scenario_dir, cfg["topological_graph"].as<std::string>()) : std::string();

    scenarios.emplace(scenario_id, scenario);
  }

  return scenarios;
}

}  // namespace g1_cogar_nav_benchmark
