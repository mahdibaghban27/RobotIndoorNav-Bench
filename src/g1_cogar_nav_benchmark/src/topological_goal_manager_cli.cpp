#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "g1_cogar_nav_benchmark/topological_graph.hpp"

namespace
{

std::map<std::string, std::string> parse_cli(int argc, char ** argv)
{
  std::map<std::string, std::string> options;
  for (int i = 1; i < argc; ++i) {
    const std::string key(argv[i]);
    if (key.rfind("--", 0) == 0) {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for argument: " + key);
      }
      options[key] = argv[++i];
    }
  }
  return options;
}

void require_option(const std::map<std::string, std::string> & options, const std::string & key)
{
  if (options.find(key) == options.end()) {
    throw std::runtime_error("Required argument missing: " + key);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto options = parse_cli(argc, argv);
    require_option(options, "--graph");
    require_option(options, "--start");
    require_option(options, "--goal");

    const g1_cogar_nav_benchmark::TopologicalGraph graph(options.at("--graph"));
    const auto route = graph.plan_route(options.at("--start"), options.at("--goal"));
    const auto waypoints = graph.route_to_waypoints(route);

    std::cout << "Route:";
    for (std::size_t i = 0; i < route.size(); ++i) {
      std::cout << (i == 0 ? " " : " -> ") << route[i];
    }
    std::cout << "\nWaypoints:\n";
    for (const auto & wp : waypoints) {
      std::cout << "  [" << wp[0] << ", " << wp[1] << ", " << wp[2] << "]\n";
    }
    return 0;
  } catch (const std::exception & ex) {
    std::cerr << "topological_goal_manager_cli error: " << ex.what() << std::endl;
    return 1;
  }
}
