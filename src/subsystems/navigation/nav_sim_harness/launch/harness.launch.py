#!/usr/bin/env python3
"""harness.launch.py — create-service bridge for the nav test harness.

This launch assumes the Gazebo simulation (``simulation/bringup.launch.py``) and
the nav stack (``nav_bringup/emmn.launch.py`` with ``sim:=true``) are **already
running** — it does NOT bring either of them up. Start them yourself (GUI or
headless, real or sim) before running the harness.

All this launch adds on top of a live stack is the one piece the harness needs:
a ``ros_gz_bridge`` service bridge for ``/world/<world_name>/create`` so the
runner's spawner can drop artifacts into the running world. That service is NOT
bridged by ``simulation/bridge.launch.py``, so the harness adds it here.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        'world_name', default_value='default',
        description='World name inside Gazebo (selects the create service)'),
]


def generate_launch_description():
    world_name = LaunchConfiguration('world_name')

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
    ld.add_action(create_bridge)
    return ld
