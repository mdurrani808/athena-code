from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('mag_calib'), 'config', 'mag_calib_params.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='mag_calib',
            executable='mag_calib_node',
            name='mag_calib_node',
            parameters=[config, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
