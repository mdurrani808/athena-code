from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('mag_heading'), 'config', 'mag_heading_params.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='mag_heading',
            executable='mag_heading_node',
            name='mag_heading_node',
            parameters=[config, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
