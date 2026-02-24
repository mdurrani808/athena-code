import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_file_dir = os.path.dirname(os.path.realpath(__file__))
    config = os.path.join(launch_file_dir, '..', 'config', 'zed_tf_publisher.yaml')

    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace',
        ),
        Node(
            package='localizer',
            executable='zed_tf_publisher_node',
            namespace=namespace,
            name='zed_tf_publisher',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        ),
    ])
