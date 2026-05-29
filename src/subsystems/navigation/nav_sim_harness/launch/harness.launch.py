#!/usr/bin/env python3
"""harness.launch.py — full Athena nav stack + sim for the test harness.

Brings up, in one launch:

  * ``simulation/bringup.launch.py`` — Gazebo + robot + bridges, with
    ``publish_sim_heading:=true`` (heading from the simulated magnetometer) and
    ``headless`` forwarded so the harness can run GUI-less.
  * ``nav_bringup/emmn.launch.py`` with ``sim:=true`` — the GPS-only nav stack
    (mission_executive, planners, gps_pose_publisher, aruco/yolo, LED, …).
  * a ``ros_gz_bridge`` service bridge for ``/world/<world_name>/create`` so the
    runner's spawner can drop artifacts in. This service is NOT bridged by
    ``simulation/bridge.launch.py`` — the harness adds it here.

Defaults to ``terrain_world.sdf`` and headless. ``publish_ground_truth_tf`` is
intentionally left off: ``gps_pose_publisher`` always broadcasts map→base_link
from the simulated GPS fix + heading, so the TF chain closes without it.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        'world', default_value='terrain_world.sdf',
        description='Gazebo world file (from description/worlds/)'),
    DeclareLaunchArgument(
        'world_name', default_value='default',
        description='World name inside Gazebo (selects the create service)'),
    DeclareLaunchArgument(
        'headless', default_value='true', choices=['true', 'false'],
        description='Run Gazebo server-only (no GUI). Sensors still render.'),
]


def generate_launch_description():
    pkg_sim = get_package_share_directory('simulation')
    pkg_nav_bringup = get_package_share_directory('nav_bringup')

    world = LaunchConfiguration('world')
    world_name = LaunchConfiguration('world_name')
    headless = LaunchConfiguration('headless')

    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'bringup.launch.py')),
        launch_arguments=[
            ('publish_sim_heading', 'true'),
            ('world', world),
            ('world_name', world_name),
            ('headless', headless),
        ],
    )

    nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav_bringup, 'launch', 'emmn.launch.py')),
        launch_arguments=[
            ('sim', 'true'),
        ],
    )

    # Bridge the Gazebo entity-create service into ROS so the spawner can use it.
    create_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='create_service_bridge',
        output='screen',
        arguments=[
            ['/world/', world_name, '/create@ros_gz_interfaces/srv/SpawnEntity'],
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(sim_bringup)
    ld.add_action(nav_stack)
    ld.add_action(create_bridge)
    return ld
