from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="description",
            description="Description package with robot URDF/xacro files.",
        ),
        DeclareLaunchArgument(
            "rviz_file",
            default_value="athena_drive.rviz",
            description="RViz config file.",
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            choices=["true", "false"],
            description="Use simulation mode (automatically sets use_sim_time).",
        ),
        DeclareLaunchArgument(
            "startup_delay",
            default_value="2.0",
            description="Delay in seconds before starting RViz.",
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    rviz_file = LaunchConfiguration("rviz_file")
    use_sim = LaunchConfiguration("use_sim")
    startup_delay = LaunchConfiguration("startup_delay")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", rviz_file]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim}],
    )

    # Delay RViz startup to allow other nodes to initialize
    delayed_rviz_launch = TimerAction(
        period=startup_delay,
        actions=[rviz_node],
    )

    return LaunchDescription(declared_arguments + [delayed_rviz_launch])
