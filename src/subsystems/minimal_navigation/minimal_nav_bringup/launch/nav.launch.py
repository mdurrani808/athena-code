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

"""nav.launch.py — Full Athena navigation stack (GPS-only, Nav2-free).

Launch order / node summary
───────────────────────────
  athena_gps         : Pixhawk GPS + heading bridge  (from athena_gps/gps_launch.py)
  gps_pose_publisher : WGS84→ENU, /robot_pose, map→base_link TF
  map_node           : DEM GeoTIFF → nav_msgs/OccupancyGrid /map
  global_planner     : Hybrid-A* planner, /goal_pose → /global_path
  ackermann_mppi     : MPPI local controller, /global_path → /cmd_vel_nav
  mission_executive  : State machine, action/service operator interface
  cmd_vel_stamper    : TwistStamped bridge → /rear_ackermann_controller/reference
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Arguments ────────────────────────────────────────────────────────────
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        choices=['true', 'false'],
        description='Use simulation GPS bridge instead of real hardware',
    )

    sim = LaunchConfiguration('sim')

    # ── Config ───────────────────────────────────────────────────────────────
    nav_bringup_dir = get_package_share_directory('minimal_nav_bringup')
    nav_params_file = os.path.join(nav_bringup_dir, 'config', 'nav_params.yaml')

    athena_map_dir = get_package_share_directory('dem_costmap')
    dem_file = os.path.join(athena_map_dir, 'maps', 'north_site800m.tif')

    # ── GPS hardware / sim bridge ─────────────────────────────────────────────
    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('athena_gps'),
                'launch',
                'gps_launch.py',
            )
        ),
        launch_arguments={'sim': sim}.items(),
    )

    # ── gps_pose_publisher ────────────────────────────────────────────────────
    gps_pose_publisher_node = Node(
        package='gps_pose_publisher',
        executable='gps_pose_publisher_node',
        name='gps_pose_publisher',
        output='screen',
        parameters=[nav_params_file],
    )

    # ── map_node (DEM → OccupancyGrid) ────────────────────────────────────────
    map_node = Node(
        package='dem_costmap',
        executable='map_node',
        name='map_node',
        output='screen',
        parameters=[
            nav_params_file,
            {'dem_file_path': dem_file},
        ],
    )

    # ── global_planner (Hybrid-A* via athena_smac_planner) ───────────────────
    global_planner_node = Node(
        package='athena_smac_planner',
        executable='global_planner_node',
        name='global_planner',
        output='screen',
        parameters=[nav_params_file],
    )

    # ── ackermann_mppi (MPPI local controller) ────────────────────────────────
    # /odom remapped to /odom/ground_truth (ZED SDK visual-inertial odometry)
    ackermann_mppi_node = Node(
        package='ackermann_mppi',
        executable='ackermann_mppi_node',
        name='mppi_runner',
        output='screen',
        parameters=[nav_params_file],
        remappings=[
            ('/odom', '/odom/ground_truth'),
        ],
    )

    # ── mission_executive (state machine) ────────────────────────────────────
    # /odom remapped to /odom/ground_truth for stop detection
    mission_executive_node = Node(
        package='mission_executive',
        executable='mission_executive_node',
        name='mission_executive',
        output='screen',
        parameters=[nav_params_file],
        remappings=[
            ('/odom', '/odom/ground_truth'),
        ],
    )

    # ── twist_stamper: /cmd_vel_nav → /rear_ackermann_controller/reference ────
    twist_stamper_node = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='cmd_vel_stamper',
        output='screen',
        parameters=[{'use_sim_time': sim}],
        remappings=[
            ('cmd_vel_in',  '/cmd_vel_nav'),
            ('cmd_vel_out', '/rear_ackermann_controller/reference'),
        ],
    )

    return LaunchDescription([
        sim_arg,
        gps_launch,
        gps_pose_publisher_node,
        map_node,
        global_planner_node,
        ackermann_mppi_node,
        mission_executive_node,
        twist_stamper_node,
    ])
