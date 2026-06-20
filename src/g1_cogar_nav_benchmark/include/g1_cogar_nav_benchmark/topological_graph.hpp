#ifndef G1_COGAR_NAV_BENCHMARK__TOPOLOGICAL_GRAPH_HPP_
#define G1_COGAR_NAV_BENCHMARK__TOPOLOGICAL_GRAPH_HPP_

#include <array>
#include <map>
#include <string>
#include <vector>

namespace g1_cogar_nav_benchmark
{

struct Area
{
  std::string area_id;
  std::array<double, 3> pose{};
  std::string area_type{"generic"};
};

class TopologicalGraph
{
public:
  explicit TopologicalGraph(const std::string & graph_file);

  double heuristic(const std::string & a, const std::string & b) const;
  std::vector<std::string> plan_route(const std::string & start_area, const std::string & goal_area) const;
  std::vector<std::array<double, 3>> route_to_waypoints(const std::vector<std::string> & route) const;

private:
  std::map<std::string, Area> areas_;
  std::map<std::string, std::vector<std::string>> edges_;
};

}  // namespace g1_cogar_nav_benchmark

#endif  // G1_COGAR_NAV_BENCHMARK__TOPOLOGICAL_GRAPH_HPP_
