#!/usr/bin/env python3
"""
Minimal Nav2 Launch File for Athena Rover
Starts localizer + nav2 navigation stack with Ackermann steering
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


def generate_launch_description():
    # ========== PACKAGE PATHS ==========
    localizer_share = get_package_share_directory('localizer')
    localizer_launch_file = os.path.join(localizer_share, 'launch', 'localizer.launch.py')

    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')

    # Default nav2 params file
    default_params_file = PathJoinSubstitution([
        FindPackageShare('athena_navigation'),
        'config',
        'nav2_params.yaml'
    ])

    # ========== LAUNCH ARGUMENTS ==========
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the Nav2 parameters YAML file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )

    # Initial pose arguments (nav2 compatible)
    initial_pose_x_arg = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='0.0',
        description='Initial pose X coordinate'
    )

    initial_pose_y_arg = DeclareLaunchArgument(
        'initial_pose_y',
        default_value='0.0',
        description='Initial pose Y coordinate'
    )

    initial_pose_z_arg = DeclareLaunchArgument(
        'initial_pose_z',
        default_value='0.0',
        description='Initial pose Z coordinate'
    )

    initial_pose_yaw_arg = DeclareLaunchArgument(
        'initial_pose_yaw',
        default_value='0.0',
        description='Initial pose yaw angle (radians)'
    )

    initial_pose_roll_arg = DeclareLaunchArgument(
        'initial_pose_roll',
        default_value='0.0',
        description='Initial pose roll angle (radians)'
    )

    initial_pose_pitch_arg = DeclareLaunchArgument(
        'initial_pose_pitch',
        default_value='0.0',
        description='Initial pose pitch angle (radians)'
    )

    set_initial_pose_arg = DeclareLaunchArgument(
        'set_initial_pose',
        default_value='false',
        description='Whether to set an initial pose'
    )

    # ========== LAUNCH CONFIGURATIONS ==========
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    namespace = LaunchConfiguration('namespace')
    set_initial_pose = LaunchConfiguration('set_initial_pose')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_z = LaunchConfiguration('initial_pose_z')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    initial_pose_roll = LaunchConfiguration('initial_pose_roll')
    initial_pose_pitch = LaunchConfiguration('initial_pose_pitch')

    # ========== LOCALIZER ==========
    # Start the localizer which provides:
    # - /localization/odom (fused IMU+GPS+wheel odometry)
    # - TF: map -> odom -> base_link
    localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localizer_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
        }.items()
    )

    # ========== NAV2 NAVIGATION STACK ==========
    # Launches: planner_server, controller_server, behavior_server, bt_navigator
    # Disable unwanted servers for minimal setup
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'namespace': namespace,
            'use_composition': 'False',
            'use_respawn': 'False',
            'use_smoother': 'False',
            'use_waypoint_follower': 'False',
            'use_velocity_smoother': 'False',
        }.items()
    )

    # ========== INITIAL POSE PUBLISHER ==========
    # Publishes initial pose to /initialpose topic for nav2 localization
    # Only runs if set_initial_pose is true
    initial_pose_publisher = Node(
        package='athena_navigation',
        executable='initial_pose_publisher.py',
        name='initial_pose_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y,
            'initial_pose_z': initial_pose_z,
            'initial_pose_yaw': initial_pose_yaw,
            'initial_pose_roll': initial_pose_roll,
            'initial_pose_pitch': initial_pose_pitch,
            'frame_id': 'map',
        }],
        output='screen',
        condition=IfCondition(set_initial_pose)
    )

    # ========== CMD_VEL CONVERSION ==========
    # Nav2 publishes geometry_msgs/Twist on cmd_vel
    # Ackermann controller expects geometry_msgs/TwistStamped on /ackermann_steering_controller/reference
    # Use twist_stamper to convert Twist -> TwistStamped
    cmd_vel_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='cmd_vel_stamper',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link'
        }],
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/ackermann_steering_controller/reference')
        ],
        output='screen'
    )

    # ========== LAUNCH DESCRIPTION ==========
    return LaunchDescription([
        # Arguments
        params_file_arg,
        use_sim_time_arg,
        autostart_arg,
        namespace_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_z_arg,
        initial_pose_yaw_arg,
        initial_pose_roll_arg,
        initial_pose_pitch_arg,
        set_initial_pose_arg,

        # Nodes & Includes
        localizer_launch,
        nav2_navigation_launch,
        initial_pose_publisher,
        cmd_vel_stamper,
    ])
