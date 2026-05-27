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
find_package(msgs REQUIRED)

add_library(vector_field_planner_algo src/vector_field_planner_algo.cpp)
target_include_directories(vector_field_planner_algo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(vector_field_planner_algo PUBLIC cxx_std_17)

add_executable(vector_field_planner_node src/vector_field_planner_node.cpp)
target_include_directories(vector_field_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(vector_field_planner_node PUBLIC cxx_std_17)
target_link_libraries(vector_field_planner_node
  vector_field_planner_algo
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${msgs_TARGETS}
  tf2_ros::tf2_ros
  tf2::tf2
)

install(TARGETS vector_field_planner_algo vector_field_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_vector_field_planner_algo
    test/test_vector_field_planner_algo.cpp
  )
  target_link_libraries(test_vector_field_planner_algo
    vector_field_planner_algo
  )
endif()

ament_package()
