cmake_minimum_required(VERSION 3.8)
project(global_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

add_executable(global_planner_node src/global_planner_node.cpp)
target_compile_features(global_planner_node PUBLIC cxx_std_17)
target_link_libraries(global_planner_node
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  tf2_ros::tf2_ros
  tf2::tf2
)

install(TARGETS global_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
