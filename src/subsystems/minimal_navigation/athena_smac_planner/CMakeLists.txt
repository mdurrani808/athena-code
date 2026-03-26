cmake_minimum_required(VERSION 3.8)
project(athena_smac_planner)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_smoother REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ompl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(msgs REQUIRED)

nav2_package()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
else()
  add_compile_options(-O3 -Wextra -Wdeprecated -fPIC)
endif()

set(library_name athena_smac_planner)

add_library(${library_name} SHARED
  src/a_star.cpp
  src/analytic_expansion.cpp
  src/collision_checker.cpp
  src/costmap_downsampler.cpp
  src/node_basic.cpp
  src/node_hybrid.cpp
  src/obstacle_heuristic.cpp
  src/distance_heuristic.cpp
  src/smoother.cpp
  src/smac_planner_hybrid.cpp
)
target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    ${OMPL_INCLUDE_DIRS}
    ${nav2_core_INCLUDE_DIRS}
    ${nav2_smoother_INCLUDE_DIRS}
)
target_link_libraries(${library_name} PUBLIC
  ${geometry_msgs_TARGETS}
  nav2_costmap_2d::layers
  nav2_costmap_2d::nav2_costmap_2d_core
  ${nav_msgs_TARGETS}
  nlohmann_json::nlohmann_json
  ${OMPL_LIBRARIES}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2::tf2
  tf2_ros::tf2_ros
  ${visualization_msgs_TARGETS}
)
target_link_libraries(${library_name} PRIVATE
  angles::angles
)

install(TARGETS ${library_name}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

add_executable(global_planner_node src/global_planner_node.cpp)
target_include_directories(global_planner_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  ${OMPL_INCLUDE_DIRS}
  ${nav2_core_INCLUDE_DIRS}
)
target_link_libraries(global_planner_node
  ${library_name}
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  nav2_costmap_2d::nav2_costmap_2d_core
  ${msgs_TARGETS}
)

install(TARGETS global_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav_msgs
  nlohmann_json
  ompl
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
  visualization_msgs
)
ament_export_targets(${PROJECT_NAME})
ament_package()
