import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        choices=['true', 'false'],
        description='Use simulation bridges instead of real hardware'
    )

    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('athena_gps'),
                'launch',
                'gps_launch.py'
            )
        ),
        launch_arguments={'sim': LaunchConfiguration('sim')}.items()
    )

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            )
        ),
        launch_arguments={'camera_model': 'zed2i'}.items()
    )

    return LaunchDescription([
        sim_arg,
        gps_launch,
        zed_launch,
    ])
