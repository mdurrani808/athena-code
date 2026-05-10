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

"""emmn.launch.py — Athena EMMN navigation stack (GPS-only, Nav2-free).

Deployment modes
────────────────
  jetson       : all sensor/compute nodes (run on the rover Jetson)
  base_station : mission_executive only (operator UI)
  standalone   : everything on one machine (default, for local dev/sim)

Node summary
────────────
  Jetson:
    robot_state_publisher     : URDF → TF static transforms
    athena_sensors            : ZED, IMU, etc.
    mag_heading               : magnetometer heading
    gps_pose_publisher        : WGS84→ENU, /robot_pose, map→base_link TF
    dem_costmap_converter     : DEM GeoTIFF → /map
    global_planner            : /goal_pose → /global_path
    point_cloud_filterer      : ZED cloud preprocessing
    pointcloud_to_laserscan   : cloud → /scan for obstacle avoidance
    vector_field_planner      : /global_path → /cmd_vel
    yolo_ros_bt               : YOLO object detection
    aruco_bt                  : ArUco marker detection
    led_indicator             : physical LED status indicator

  Base station:
    mission_executive         : state machine, action/service operator interface
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'mode',
            default_value='standalone',
            choices=['standalone', 'jetson', 'base_station'],
            description=(
                "Deployment mode. "
                "'standalone' (default) starts all nodes on a single machine. "
                "'jetson' starts only the sensor/compute nodes (run on the rover). "
                "'base_station' starts only the mission_executive (operator UI)."
            ),
        ),
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            choices=['true', 'false'],
            description='Use simulation GPS bridge instead of real hardware',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    sim  = LaunchConfiguration('sim')

    nav_bringup_dir = get_package_share_directory('nav_bringup')

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

    # ── Jetson nodes ──────────────────────────────────────────────────────────

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

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('athena_sensors'), 'launch', 'sensors.launch.py')
        ),
        launch_arguments={
            'sim': sim,
            'enable_gnss': 'false',
        }.items(),
    )

    mag_heading_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mag_heading'), 'launch', 'mag_heading.launch.py')
        ),
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

    vector_field_planner_node = Node(
        package='vector_field_planner',
        executable='vector_field_planner_node',
        name='vector_field_planner',
        output='screen',
        parameters=[nav_params_file],
        remappings=[
            ('/odom', '/odom/ground_truth'),
        ],
    )

    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('yolo_ros_bt'), 'launch', 'yolo.ros.launch.py')
        ),
    )

    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('aruco_bt'), 'launch', 'aruco.launch.py')
        ),
    )

    led_indicator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('led_indicator'), 'launch', 'led_indicator.launch.py')
        ),
    )

    # ── Base station nodes ────────────────────────────────────────────────────

    mission_executive_node = Node(
        package='mission_executive',
        executable='mission_executive_node',
        name='mission_executive',
        output='screen',
        parameters=[nav_params_file],
    )

    # ── Mode selection ────────────────────────────────────────────────────────

    jetson_actions = [
        robot_state_publisher_node,
        sensors_launch,
        mag_heading_launch,
        gps_pose_publisher_node,
        dem_costmap_converter_node,
        global_planner_node,
        point_cloud_filterer_node,
        pointcloud_to_laserscan_node,
        vector_field_planner_node,
        yolo_launch,
        aruco_launch,
        led_indicator_launch,
    ]

    base_station_actions = [
        mission_executive_node,
    ]

    if mode == 'jetson':
        return jetson_actions
    elif mode == 'base_station':
        return base_station_actions
    else:
        return jetson_actions + base_station_actions
