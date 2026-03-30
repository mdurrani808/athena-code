cmake_minimum_required(VERSION 3.8)
project(vector_field_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

add_executable(vector_field_planner_node src/vector_field_planner_node.cpp)
target_compile_features(vector_field_planner_node PUBLIC cxx_std_17)
target_link_libraries(vector_field_planner_node
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  tf2_ros::tf2_ros
  tf2::tf2
)

install(TARGETS vector_field_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_pure_pursuit_sim
    test/test_pure_pursuit_sim.cpp
  )
  target_compile_features(test_pure_pursuit_sim PUBLIC cxx_std_17)
endif()

ament_package()
