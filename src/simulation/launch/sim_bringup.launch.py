from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Gazebo world file to load'
    ),
    DeclareLaunchArgument(
        'publish_ground_truth_tf',
        default_value='true',
        choices=['true', 'false'],
        description='Publish ground truth odom -> base_footprint transform'
    ),
]


def generate_launch_description():
    pkg_sim = get_package_share_directory('simulation')

    gazebo_launch = PathJoinSubstitution([pkg_sim, 'launch', 'gz_sim.launch.py'])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )

    bridge_launch = PathJoinSubstitution([pkg_sim, 'launch', 'bridge.launch.py'])
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bridge_launch])
    )

    ground_truth_tf_launch = PathJoinSubstitution([pkg_sim, 'launch', 'ground_truth_tf.launch.py'])
    ground_truth_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ground_truth_tf_launch]),
        condition=IfCondition(LaunchConfiguration('publish_ground_truth_tf'))
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo)
    ld.add_action(bridge)
    ld.add_action(ground_truth_tf)
    return ld
