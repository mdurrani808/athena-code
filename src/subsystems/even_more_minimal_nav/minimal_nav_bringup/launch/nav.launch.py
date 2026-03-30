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
  vector_field_planner  : /global_path → /cmd_vel  (pure-pursuit)
  mission_executive     : state machine, action/service operator interface

Topic graph
───────────
  gps_pose_publisher    → TF map→base_link
  dem_costmap_converter → /map
  mission_executive     → /nav_enabled, /goal_pose
  global_planner        → /global_path
  vector_field_planner  → /cmd_vel, ~/debug_markers
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

    dem_costmap_dir = get_package_share_directory('dem_costmap')
    dem_file = os.path.join(dem_costmap_dir, 'maps', 'north_site800m.tif')

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

    # ── dem_costmap_converter (DEM GeoTIFF → OccupancyGrid /map) ─────────────
    # dem_file_path is passed inline so the YAML can leave it blank by default.
    dem_costmap_converter_node = Node(
        package='dem_costmap',
        executable='map_node',
        name='dem_costmap_converter',
        output='screen',
        parameters=[
            nav_params_file,
            {'dem_file_path': dem_file},
        ],
    )

    # ── global_planner (/goal_pose → /global_path) ────────────────────────────
    global_planner_node = Node(
        package='global_planner',
        executable='global_planner_node',
        name='global_planner',
        output='screen',
        parameters=[nav_params_file],
    )

    # ── vector_field_planner (/global_path → /cmd_vel) ────────────────────────
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

    # ── mission_executive (state machine) ────────────────────────────────────
    mission_executive_node = Node(
        package='mission_executive',
        executable='mission_executive_node',
        name='mission_executive',
        output='screen',
        parameters=[nav_params_file],
    )

    return LaunchDescription([
        sim_arg,
        gps_launch,
        gps_pose_publisher_node,
        dem_costmap_converter_node,
        global_planner_node,
        vector_field_planner_node,
        mission_executive_node,
    ])
