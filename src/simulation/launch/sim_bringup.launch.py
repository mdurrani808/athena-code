"""
Simulation Infrastructure Bringup
Launches only simulation-related infrastructure:
- Gazebo simulator
- ROS-Gazebo bridges for sensors

This launch file contains NO subsystem-specific logic.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Gazebo world file to load'
    ),
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use simulation time from Gazebo'
    ),
]


def generate_launch_description():
    pkg_sim = get_package_share_directory('simulation')

    # Include Gazebo simulator
    gazebo_launch = PathJoinSubstitution([pkg_sim, 'launch', 'gz_sim.launch.py'])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('world', LaunchConfiguration('world'))
        ]
    )

    # Include ROS-Gazebo bridges
    bridge_launch = PathJoinSubstitution([pkg_sim, 'launch', 'bridge.launch.py'])
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bridge_launch])
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo)
    ld.add_action(bridge)
    return ld
