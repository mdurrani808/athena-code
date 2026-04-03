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

add_library(mission_executive_algo src/mission_executive_algo.cpp)
target_include_directories(mission_executive_algo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(mission_executive_algo PUBLIC cxx_std_17)

add_executable(mission_executive_node src/mission_executive_node.cpp)
target_include_directories(mission_executive_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(mission_executive_node PUBLIC cxx_std_17)
target_link_libraries(mission_executive_node
  mission_executive_algo
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

install(TARGETS mission_executive_algo mission_executive_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_mission_executive
    test/test_mission_executive.cpp
  )
  target_include_directories(test_mission_executive PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_compile_features(test_mission_executive PUBLIC cxx_std_17)
  target_link_libraries(test_mission_executive mission_executive_algo)

  ament_add_gtest(test_mock_pipeline
    test/test_mock_pipeline.cpp
    ../global_planner/src/global_planner_algo.cpp
    ../vector_field_planner/src/vector_field_planner_algo.cpp
  )
  target_include_directories(test_mock_pipeline PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../global_planner/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../vector_field_planner/include>
  )
  target_compile_features(test_mock_pipeline PUBLIC cxx_std_17)
  target_link_libraries(test_mock_pipeline mission_executive_algo)
endif()

ament_package()
