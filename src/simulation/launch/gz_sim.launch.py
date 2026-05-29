import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

ARGUMENTS = [
    DeclareLaunchArgument('world', default_value='empty.sdf',
                          description='Gazebo world file'),
    DeclareLaunchArgument('headless', default_value='false',
                          choices=['true', 'false'],
                          description='Run Gazebo server-only (no GUI). Sensors still render.'),
]

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_description = get_package_share_directory('description')
    
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            str(Path(pkg_description).parent.resolve()),
            ':',
            os.path.join(pkg_description, 'models')
        ])

    gz_sim_launch = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    world_path = PathJoinSubstitution(
        [pkg_description, 'worlds', LaunchConfiguration('world')])

    # '-s' makes Gazebo run server-only (no GUI). Sensors still render so the
    # ZED/navsat plugins keep publishing — exactly what the headless harness needs.
    headless_flag = PythonExpression(
        ["' -s' if '", LaunchConfiguration('headless'), "' == 'true' else ''"])

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('gz_args', [world_path,
                        ' -r',
                        ' -v 4',        # Verbosity level, 0=errors only, 4=debug
                        headless_flag]
            )
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_resource_path)
    ld.add_action(gazebo_sim)
    return ld