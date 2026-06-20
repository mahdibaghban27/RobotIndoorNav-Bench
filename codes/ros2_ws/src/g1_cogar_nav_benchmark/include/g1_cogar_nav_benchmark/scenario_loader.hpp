#ifndef G1_COGAR_NAV_BENCHMARK__SCENARIO_LOADER_HPP_
#define G1_COGAR_NAV_BENCHMARK__SCENARIO_LOADER_HPP_

#include <array>
#include <map>
#include <string>

namespace g1_cogar_nav_benchmark
{

struct Scenario
{
  std::string scenario_id;
  std::string world;
  std::string map_yaml;
  std::array<double, 3> start_pose{};
  std::array<double, 3> goal_pose{};
  std::string start_area;
  std::string goal_area;
  std::string description;
  bool dynamic_obstacles{false};
  bool use_topological_route{false};
  std::string notes;
  std::string topological_graph;
};

std::map<std::string, Scenario> load_scenarios(const std::string & path);

}  // namespace g1_cogar_nav_benchmark

#endif  // G1_COGAR_NAV_BENCHMARK__SCENARIO_LOADER_HPP_
