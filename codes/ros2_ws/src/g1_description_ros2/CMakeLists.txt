cmake_minimum_required(VERSION 3.8)
project(g1_description_ros2)

find_package(ament_cmake REQUIRED)

set(RESOURCE_DIRECTORIES
  urdf
  meshes
  launch
  rviz
  config
)

set(EXISTING_RESOURCE_DIRECTORIES "")

foreach(resource_directory IN LISTS RESOURCE_DIRECTORIES)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${resource_directory}")
    list(APPEND EXISTING_RESOURCE_DIRECTORIES "${resource_directory}")
  else()
    message(STATUS "Skipping missing directory: ${resource_directory}")
  endif()
endforeach()

if(EXISTING_RESOURCE_DIRECTORIES)
  install(
    DIRECTORY ${EXISTING_RESOURCE_DIRECTORIES}
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
