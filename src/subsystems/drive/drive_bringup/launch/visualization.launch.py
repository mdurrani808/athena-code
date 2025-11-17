from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="description",
            description="Description package with robot URDF/xacro files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_file",
            default_value="athena_drive.rviz",
            description="RViz config file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            choices=["true", "false"],
            description="Use simulation mode (automatically sets use_sim_time).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "startup_delay",
            default_value="2.0",
            description="Delay in seconds before starting RViz.",
        )
    )

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
    delayed_rviz = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=rviz_node,
            on_start=[
                TimerAction(
                    period=startup_delay,
                    actions=[rviz_node],
                )
            ],
        )
    )

    return LaunchDescription(declared_arguments + [rviz_node])
