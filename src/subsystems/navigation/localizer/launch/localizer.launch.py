import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    launch_file_dir = os.path.dirname(os.path.realpath(__file__))
    default_config = os.path.join(launch_file_dir, '..', 'config', 'localizer.yaml')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to the localizer configuration file'
        ),
        Node(
            package='localizer',
            executable='localizer_node',
            namespace=namespace,
            name='localizer_node',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time}
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
