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
    #dem_launch = os.path.join(athena_map_share, 'launch', 'dem_costmap.launch.py')

    athena_planner_share = get_package_share_directory('athena_planner')
    navigation_launch = os.path.join(athena_planner_share, 'launch', 'navigation_nodes.launch.py')


    localizer_share = get_package_share_directory('localizer')
    localizer_launch_file = os.path.join(localizer_share, 'launch', 'localizer.launch.py')
    default_params = PathJoinSubstitution([
        FindPackageShare('athena_planner'), 'config', 'nav2_params.yaml'
    ])
    ekf_params_file = PathJoinSubstitution([
        FindPackageShare('athena_planner'), 'config', 'ekf.yaml'
    ])

    params_file = LaunchConfiguration('params_file')
    ekf_config = LaunchConfiguration('ekf_config')

    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')

    twist_stamper_node = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='cmd_vel_stamper',
        remappings=[
            ('cmd_vel_in',  '/cmd_vel_nav2'),                      
            ('cmd_vel_out', '/ackermann_steering_controller/reference'),
        ],
    )
    
    localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localizer_launch_file),
        launch_arguments={
            'use_sim_time': 'True',
        }.items()
    )
    """
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/local')]
    )

    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/global')]
    )

    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
        remappings=[
            ('imu/data', '/imu'),
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', 'odometry/global')
        ]
    )"""
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Full path to the Nav2 params YAML'
        ),
        DeclareLaunchArgument(
            'ekf_config', default_value=ekf_params_file,
            description='Full path to the robot_localization EKF YAML'
        ),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),

        SetRemap(src='cmd_vel', dst='/cmd_vel_nav2'),

        #ekf_local,
        #ekf_global,
        #navsat_transform,
        twist_stamper_node,
        localizer_launch,
        #IncludeLaunchDescription(PythonLaunchDescriptionSource(dem_launch)),

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