cmake_minimum_required(VERSION 3.8)
project(athena_localizer_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(athena_localizer REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(athena_localizer_test_node src/athena_localizer_test_node.cpp)
target_include_directories(athena_localizer_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(athena_localizer_test_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(athena_localizer_test_node
  athena_localizer
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs)

install(TARGETS athena_localizer_test_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(localizer_evaluation_node src/localizer_evaluation_node.cpp)
target_include_directories(localizer_evaluation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(localizer_evaluation_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(localizer_evaluation_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros)

install(TARGETS localizer_evaluation_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
