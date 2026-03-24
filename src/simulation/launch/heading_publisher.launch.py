from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'noise_sigma',
            default_value='0.05',
            description='Gaussian noise sigma in radians added to heading'
        ),

        Node(
            package='simulation',
            executable='heading_publisher.py',
            name='heading_publisher',
            output='screen',
            parameters=[{
                'odom_topic': '/odom/ground_truth',
                'heading_topic': '/heading',
                'noise_sigma': LaunchConfiguration('noise_sigma'),
                'publish_rate': 10.0,
                'use_sim_time': True,
            }]
        ),
    ])
