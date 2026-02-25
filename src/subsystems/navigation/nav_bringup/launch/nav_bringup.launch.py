from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    sim               = LaunchConfiguration('sim')
    use_zed_localizer = LaunchConfiguration('use_zed_localizer')
    enable_gnss       = LaunchConfiguration('enable_gnss')
    params_file       = LaunchConfiguration('params_file')
    use_dem           = LaunchConfiguration('use_dem')
    use_respawn       = LaunchConfiguration('use_respawn')
    log_level         = LaunchConfiguration('log_level')

    use_localizer = PythonExpression(
        ["'false' if '", use_zed_localizer, "' == 'true' else 'true'"]
    )

    # Static TF: zed_camera_link -> base_footprint
    # The ZED always publishes odom -> zed_camera_link. This static transform
    # bridges to the robot's base frame using the inverse of the camera mount
    # offset (xyz=0.5, 0.1, 0.1 in athena_drive.urdf.xacro -> inverted here).
    #zed_to_base_tf = Node(
    #    package='tf2_ros',
    #    executable='static_transform_publisher',
    #    name='zed_to_base_footprint_tf',
    #    arguments=['-0.5', '-0.1', '-0.1', '0', '0', '0',
    #               'zed_camera_link', 'base_footprint'],
    #    condition=UnlessCondition(sim),
    #)

    drive_launch_file = os.path.join(
        get_package_share_directory('drive_bringup'), 'launch', 'athena_drive.launch.py'
    )

    drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(drive_launch_file),
        launch_arguments={
            'use_sim': sim,
        }.items(),
    )

    navigation_launch_file = os.path.join(
        get_package_share_directory('athena_planner'), 'launch', 'navigation.launch.py'
    )

    default_params = PathJoinSubstitution([
        FindPackageShare('athena_planner'), 'config', 'nav2_params.yaml'
    ])

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_file),
        launch_arguments={
            'sim':           sim,
            'params_file':   params_file,
            'use_dem':       use_dem,
            'use_respawn':   use_respawn,
            'log_level':     log_level,
            'use_localizer': use_localizer,
            'enable_gnss':   enable_gnss,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            choices=['true', 'false'],
            description='Set true when running in Gazebo simulation',
        ),
        DeclareLaunchArgument(
            'use_zed_localizer',
            default_value='false',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'enable_gnss',
            default_value='false',
            choices=['true', 'false'],
            description='Enable GNSS fusion inside the ZED camera',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the Nav2 params YAML',
        ),
        DeclareLaunchArgument(
            'use_dem',
            default_value='false',
            choices=['true', 'false'],
            description='Enable DEM costmap layer',
        ),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='false',
            description='Whether to respawn if a node crashes',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for nav2 nodes',
        ),

        #zed_to_base_tf,
        drive_launch,
        navigation_launch,
    ])
