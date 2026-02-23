from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    sim = LaunchConfiguration('sim')
    camera_model = LaunchConfiguration('camera_model')
    publish_odom = LaunchConfiguration('publish_odom')
    publish_map = LaunchConfiguration("publish_map")
    enable_gnss = LaunchConfiguration('enable_gnss')

    declare_sim = DeclareLaunchArgument(
        'sim',
        default_value='false',
        choices=['true', 'false'],
        description='Use simulation bridges instead of real hardware'
    )
    declare_camera_model = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2i',
        description='ZED camera model'
    )
    declare_publish_odom = DeclareLaunchArgument(
        'publish_odom',
        default_value='false',
        description='Publish odometry tf'
    )
    declare_publish_map= DeclareLaunchArgument(
        'publish_map',
        default_value='false',
        description='Publish map tf'
    )
    declare_enable_gnss = DeclareLaunchArgument(
        'enable_gnss',
        default_value='false',
        choices=['true', 'false'],
        description='Enable GNSS fusion in the ZED camera'
    )

    athena_sensors_launch_dir = os.path.join(
        get_package_share_directory('athena_sensors'), 'launch'
    )
    athena_gps_launch_dir = os.path.join(
        get_package_share_directory('athena_gps'), 'launch'
    )

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(athena_sensors_launch_dir, 'zed.launch.py')
        ),
        launch_arguments={
            'sim':             sim,
            'camera_model':    camera_model,
            'publish_tf':      publish_odom, # this is really the odom tf, idk why its just called tf
            'publish_map_tf':  publish_map,
            'publish_urdf':   'false', # we take care of this elsewhere
            'enable_gnss':     enable_gnss,
        }.items(),
        condition=UnlessCondition(sim),
    )

    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(athena_gps_launch_dir, 'gps_launch.py')
        ),
        launch_arguments={
            'sim': sim,
        }.items(),
    )

    return LaunchDescription([
        declare_sim,
        declare_camera_model,
        declare_publish_odom,
        declare_publish_map,
        declare_enable_gnss,
        zed_launch,
        gps_launch,
    ])
