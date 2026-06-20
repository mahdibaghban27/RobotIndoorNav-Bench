#include "g1_cogar_nav_benchmark/topological_graph.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

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

}  // namespace

TopologicalGraph::TopologicalGraph(const std::string & graph_file)
{
  const YAML::Node root = YAML::LoadFile(graph_file);
  const YAML::Node areas_node = root["areas"];
  const YAML::Node edges_node = root["edges"];

  if (!areas_node || !areas_node.IsMap() || !edges_node || !edges_node.IsMap()) {
    throw std::runtime_error("Topological graph must contain 'areas' and 'edges' maps.");
  }

  for (const auto & item : areas_node) {
    const std::string area_id = item.first.as<std::string>();
    const YAML::Node cfg = item.second;
    Area area;
    area.area_id = area_id;
    area.pose = read_pose3(cfg["pose"]);
    area.area_type = cfg["type"] ? cfg["type"].as<std::string>() : "generic";
    areas_.emplace(area_id, area);
  }

  for (const auto & item : edges_node) {
    const std::string from = item.first.as<std::string>();
    std::vector<std::string> neighbors;
    for (const auto & nb : item.second) {
      neighbors.push_back(nb.as<std::string>());
    }
    edges_[from] = neighbors;
  }
}

double TopologicalGraph::heuristic(const std::string & a, const std::string & b) const
{
  const auto a_it = areas_.find(a);
  const auto b_it = areas_.find(b);
  if (a_it == areas_.end() || b_it == areas_.end()) {
    throw std::runtime_error("Unknown topological area in heuristic().");
  }
  const auto & pa = a_it->second.pose;
  const auto & pb = b_it->second.pose;
  return std::hypot(pa[0] - pb[0], pa[1] - pb[1]);
}

std::vector<std::string> TopologicalGraph::plan_route(
  const std::string & start_area,
  const std::string & goal_area) const
{
  if (areas_.count(start_area) == 0U || areas_.count(goal_area) == 0U) {
    throw std::runtime_error("Unknown topological start or goal area.");
  }

  std::set<std::string> open_set{start_area};
  std::map<std::string, std::string> came_from;
  std::map<std::string, double> g_score;
  std::map<std::string, double> f_score;

  for (const auto & item : areas_) {
    g_score[item.first] = std::numeric_limits<double>::infinity();
    f_score[item.first] = std::numeric_limits<double>::infinity();
  }

  g_score[start_area] = 0.0;
  f_score[start_area] = heuristic(start_area, goal_area);

  while (!open_set.empty()) {
    auto best_it = open_set.begin();
    for (auto it = open_set.begin(); it != open_set.end(); ++it) {
      if (f_score.at(*it) < f_score.at(*best_it)) {
        best_it = it;
      }
    }

    const std::string current = *best_it;
    if (current == goal_area) {
      std::vector<std::string> route{current};
      auto came_it = came_from.find(current);
      while (came_it != came_from.end()) {
        route.push_back(came_it->second);
        came_it = came_from.find(came_it->second);
      }
      std::reverse(route.begin(), route.end());
      return route;
    }

    open_set.erase(best_it);
    const auto edge_it = edges_.find(current);
    if (edge_it == edges_.end()) {
      continue;
    }

    for (const auto & neighbor : edge_it->second) {
      const double tentative = g_score.at(current) + heuristic(current, neighbor);
      if (tentative < g_score.at(neighbor)) {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative;
        f_score[neighbor] = tentative + heuristic(neighbor, goal_area);
        open_set.insert(neighbor);
      }
    }
  }

  throw std::runtime_error("No topological route found between the requested areas.");
}

std::vector<std::array<double, 3>> TopologicalGraph::route_to_waypoints(
  const std::vector<std::string> & route) const
{
  std::vector<std::array<double, 3>> waypoints;
  waypoints.reserve(route.size());
  for (const auto & area_id : route) {
    const auto it = areas_.find(area_id);
    if (it == areas_.end()) {
      throw std::runtime_error("Unknown area in route_to_waypoints().");
    }
    waypoints.push_back(it->second.pose);
  }
  return waypoints;
}

}  // namespace g1_cogar_nav_benchmark
