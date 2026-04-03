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

add_library(global_planner_algo src/global_planner_algo.cpp)
target_include_directories(global_planner_algo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(global_planner_algo PUBLIC cxx_std_17)

add_executable(global_planner_node src/global_planner_node.cpp)
target_include_directories(global_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(global_planner_node PUBLIC cxx_std_17)
target_link_libraries(global_planner_node
  global_planner_algo
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  tf2_ros::tf2_ros
  tf2::tf2
)

install(TARGETS global_planner_algo global_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Add tests here when they exist
endif()

ament_package()
