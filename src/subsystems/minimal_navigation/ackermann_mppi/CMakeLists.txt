cmake_minimum_required(VERSION 3.8)
project(ackermann_mppi)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mfma" COMPILER_SUPPORTS_FMA)
if(COMPILER_SUPPORTS_FMA)
  add_compile_options(-mfma)
endif()

# --- Core optimizer library (no ROS node, no pluginlib) ---
add_library(ackermann_mppi_core SHARED
  src/noise_generator.cpp
  src/optimizer.cpp
  src/critics/constraint_critic.cpp
  src/critics/goal_critic.cpp
  src/critics/obstacles_critic.cpp
  src/critics/path_align_critic.cpp
  src/critics/path_follow_critic.cpp
)

# Enable C++17 (required for inline constexpr, structured bindings, etc.)
target_compile_features(ackermann_mppi_core PUBLIC cxx_std_17)

# Apply -O3 only for Release and RelWithDebInfo builds so Debug stays debuggable.
# Do not force -O3 unconditionally — it disrespects the developer's chosen build type.
target_compile_options(ackermann_mppi_core PUBLIC
  $<$<CONFIG:Release>:-O3>
  $<$<CONFIG:RelWithDebInfo>:-O3>
)

target_include_directories(ackermann_mppi_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(ackermann_mppi_core PUBLIC
  angles::angles
  ${geometry_msgs_TARGETS}
  nav2_costmap_2d::layers
  nav2_costmap_2d::nav2_costmap_2d_core
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  ${visualization_msgs_TARGETS}
  ${builtin_interfaces_TARGETS}
)

# --- ROS2 node executable ---
add_executable(ackermann_mppi_node
  src/ackermann_mppi_node.cpp
)

target_compile_options(ackermann_mppi_node PRIVATE
  $<$<CONFIG:Release>:-O3>
  $<$<CONFIG:RelWithDebInfo>:-O3>
)

target_include_directories(ackermann_mppi_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

target_link_libraries(ackermann_mppi_node
  ackermann_mppi_core
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  nav2_costmap_2d::nav2_costmap_2d_core
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

# --- Install ---
install(TARGETS ackermann_mppi_core ackermann_mppi_node
  EXPORT ackermann_mppi
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(ackermann_mppi HAS_LIBRARY_TARGET)
ament_export_dependencies(
  angles
  geometry_msgs
  nav2_costmap_2d
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  Eigen3
)
ament_export_include_directories(include/${PROJECT_NAME})

ament_package()
