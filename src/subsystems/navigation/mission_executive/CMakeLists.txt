cmake_minimum_required(VERSION 3.8)
project(mission_executive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(msgs REQUIRED)

add_executable(mission_executive_node src/mission_executive_node.cpp)
target_compile_features(mission_executive_node PUBLIC cxx_std_17)
target_link_libraries(mission_executive_node
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  ${vision_msgs_TARGETS}
  ${msgs_TARGETS}
)

install(TARGETS mission_executive_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
