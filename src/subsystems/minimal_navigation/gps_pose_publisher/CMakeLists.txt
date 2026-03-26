cmake_minimum_required(VERSION 3.8)
project(gps_pose_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(msgs REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

add_executable(gps_pose_publisher_node src/gps_pose_publisher_node.cpp)
target_include_directories(gps_pose_publisher_node PRIVATE ${GeographicLib_INCLUDE_DIRS})
target_link_libraries(gps_pose_publisher_node
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2_ros::tf2_ros
  tf2::tf2
  ${msgs_TARGETS}
  ${GeographicLib_LIBRARIES}
)

install(TARGETS gps_pose_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
