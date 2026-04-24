#!/usr/bin/env python3
# Copyright (c) 2025 UMD Loop
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""nav.launch.py — Athena navigation stack (GPS-only, Nav2-free).

Node summary
────────────
  athena_gps            : Pixhawk GPS + heading bridge
  gps_pose_publisher    : WGS84→ENU, /robot_pose, map→base_link TF
  dem_costmap_converter : DEM GeoTIFF → nav_msgs/OccupancyGrid /map
  global_planner        : /goal_pose → /global_path (straight-line or A*)
  vector_field_planner  : /global_path → /cmd_vel  (pure-pursuit + obstacle avoidance)
  mission_executive     : state machine, action/service operator interface
  pointcloud_to_laserscan : /zed/.../cloud_registered → /scan (for obstacle avoidance)

Topic graph
───────────
  gps_pose_publisher       → TF map→base_link
  dem_costmap_converter    → /map
  mission_executive        → /nav_enabled, /goal_pose
  global_planner           → /global_path
  pointcloud_to_laserscan  → /scan
  vector_field_planner     → /cmd_vel, ~/debug_markers
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        choices=['true', 'false'],
        description='Use simulation GPS bridge instead of real hardware',
    )

    sim = LaunchConfiguration('sim')

    nav_bringup_dir = get_package_share_directory('nav_bringup')
    
    from launch.substitutions import PythonExpression
    nav_params_file = PythonExpression([
        "'", os.path.join(nav_bringup_dir, 'config', 'nav_params_sim.yaml'), "' if '", sim, "' == 'true' else '",
        os.path.join(nav_bringup_dir, 'config', 'nav_params_real.yaml'), "'"
    ])

    athena_map_dir = get_package_share_directory('athena_map')
    dem_file = os.path.join(athena_map_dir, 'maps', 'north_site800m.tif')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('description'), 'urdf', 'athena_drive.urdf.xacro'
        ]),
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': sim,
        }],
    )

    sensors_share = get_package_share_directory('athena_sensors')
    sensors_launch_file = os.path.join(sensors_share, 'launch', 'sensors.launch.py')

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensors_launch_file),
        launch_arguments={
            'sim': sim,
            'enable_gnss': 'false',
        }.items(),
    )

    gps_pose_publisher_node = Node(
        package='gps_pose_publisher',
        executable='gps_pose_publisher_node',
        name='gps_pose_publisher',
        output='screen',
        parameters=[nav_params_file],
    )

    dem_costmap_converter_node = Node(
        package='athena_map',
        executable='map_node',
        name='dem_costmap_converter',
        output='screen',
        parameters=[
            nav_params_file,
            {'dem_file_path': dem_file},
        ],
    )

    global_planner_node = Node(
        package='global_planner',
        executable='global_planner_node',
        name='global_planner',
        output='screen',
        parameters=[nav_params_file],
    )

    vector_field_planner_node = Node(
        package='vector_field_planner',
        executable='vector_field_planner_node',
        name='vector_field_planner',
        output='screen',
        parameters=[nav_params_file],
        #arguments=['--ros-args', '--log-level', 'debug'],
        remappings=[
            ('/odom', '/odom/ground_truth'),
        ],
    )

    mission_executive_node = Node(
        package='mission_executive',
        executable='mission_executive_node',
        name='mission_executive',
        output='screen',
        parameters=[nav_params_file],
    )


    point_cloud_filterer_node = Node(
        package='point_cloud_filterer',
        executable='point_cloud_filtered',
        name='point_cloud_filterer',
        output='screen',
        parameters=[nav_params_file],
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[nav_params_file],
        remappings=[
            ('cloud_in', '/zed/cloud_base_link'),
            ('scan',     '/scan'),
        ],
    )

    return LaunchDescription([
        sim_arg,
        robot_state_publisher_node,
        sensors_launch,
        gps_pose_publisher_node,
        dem_costmap_converter_node,
        global_planner_node,
        point_cloud_filterer_node,
        pointcloud_to_laserscan_node,
        vector_field_planner_node,
        mission_executive_node,
    ])
