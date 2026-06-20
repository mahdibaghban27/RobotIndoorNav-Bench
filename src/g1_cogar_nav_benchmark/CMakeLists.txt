cmake_minimum_required(VERSION 3.8)
project(g1_cogar_nav_benchmark)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_library(${PROJECT_NAME}_core
  src/scenario_loader.cpp
  src/topological_graph.cpp
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_core
  ament_index_cpp
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  nav2_msgs
  yaml_cpp_vendor)
target_link_libraries(${PROJECT_NAME}_core yaml-cpp)

add_executable(braitenberg_reflex src/braitenberg_reflex_node.cpp)
target_compile_features(braitenberg_reflex PUBLIC cxx_std_17)
ament_target_dependencies(braitenberg_reflex rclcpp geometry_msgs sensor_msgs std_msgs)

add_executable(subsumption_mux src/subsumption_mux_node.cpp)
target_compile_features(subsumption_mux PUBLIC cxx_std_17)
ament_target_dependencies(subsumption_mux rclcpp geometry_msgs std_msgs)

add_executable(benchmark_logger src/benchmark_logger_node.cpp)
target_compile_features(benchmark_logger PUBLIC cxx_std_17)
target_include_directories(benchmark_logger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(benchmark_logger rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs)

add_executable(benchmark_runner src/benchmark_runner.cpp)
target_compile_features(benchmark_runner PUBLIC cxx_std_17)
target_include_directories(benchmark_runner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(benchmark_runner rclcpp rclcpp_action geometry_msgs std_srvs nav2_msgs)
target_link_libraries(benchmark_runner ${PROJECT_NAME}_core)

add_executable(topological_goal_manager_cli src/topological_goal_manager_cli.cpp)
target_compile_features(topological_goal_manager_cli PUBLIC cxx_std_17)
target_include_directories(topological_goal_manager_cli PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(topological_goal_manager_cli ${PROJECT_NAME}_core)

install(
  TARGETS
    ${PROJECT_NAME}_core
    braitenberg_reflex
    subsumption_mux
    benchmark_logger
    benchmark_runner
    topological_goal_manager_cli
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config maps worlds rviz resource scripts tools DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS
    g1_cogar_nav_benchmark/g1_robot_simulation.py
    scripts/run_one.sh
    scripts/run_all_benchmarks.sh
    scripts/dynamic_obstacle_commander.py
    scripts/house_dynamic_obstacle_commander.py
    tools/summarize_results.py
    tools/aggregate_results.py
    tools/plot_results.py
    tools/update_report_data.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  ament_index_cpp
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  nav2_msgs
  yaml_cpp_vendor)

ament_package()
