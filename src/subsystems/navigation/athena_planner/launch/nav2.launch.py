from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    athena_map_share = get_package_share_directory('athena_map')
    dem_launch = os.path.join(athena_map_share, 'launch', 'dem_costmap.launch.py')

    athena_planner_share = get_package_share_directory('athena_planner')
    navigation_launch = os.path.join(athena_planner_share, 'launch', 'navigation_nodes.launch.py')

    default_params = PathJoinSubstitution([
        FindPackageShare('athena_planner'), 'config', 'nav2_params.yaml'
    ])
    params_file = LaunchConfiguration('params_file')

    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')

    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', map_frame, odom_frame],
        output='screen',
    )

    twist_stamper_node = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='cmd_vel_stamper',
        remappings=[
            ('cmd_vel_in',  '/cmd_vel_nav2'),                      
            ('cmd_vel_out', '/ackermann_steering_controller/reference'),
        ],
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Full path to the Nav2 params YAML'
        ),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),

        SetRemap(src='cmd_vel', dst='/cmd_vel_nav2'),

        static_map_to_odom,
        twist_stamper_node,

        IncludeLaunchDescription(PythonLaunchDescriptionSource(dem_launch)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': 'true',
                'autostart': 'true',
                'use_respawn': 'False',
                'log_level': 'info',
            }.items()
        ),
    ])
