from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    description_share = get_package_share_directory('description')
    urdf_file = os.path.join(description_share, 'urdf', 'athena_drive.urdf.xacro')

    robot_description_content = Command([
        'xacro ', urdf_file,
        ' use_mock_hardware:=false',
        ' sim_gazebo:=false',
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str),
            'use_sim_time': False,
        }],
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('athena_planner'),
                'launch', 'navigation.launch.py',
            )
        ),
    )

    return LaunchDescription([
        robot_state_publisher,
        navigation_launch,
    ])
