from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
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
        DeclareLaunchArgument(
            "rqt",
            default_value="false",
            choices=["true", "false"],
            description="Start rqt_image_view for camera topics.",
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/depth_camera",
            description="Image topic to view in rqt_image_view.",
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    rviz_file = LaunchConfiguration("rviz_file")
    use_sim = LaunchConfiguration("use_sim")
    startup_delay = LaunchConfiguration("startup_delay")
    rqt = LaunchConfiguration("rqt")
    image_topic = LaunchConfiguration("image_topic")

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

    rqt_image_view_node = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="rqt_image_view",
        arguments=[image_topic],
        parameters=[{"use_sim_time": use_sim}],
        condition=IfCondition(rqt),
    )

    return LaunchDescription(declared_arguments + [delayed_rviz_launch, rqt_image_view_node])
