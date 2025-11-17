from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="drive_bringup",
            description='Package with the configuration in "config" folder.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "joystick_config",
            default_value="joystick.yaml",
            description="YAML file with the joystick configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "teleop_twist_config",
            default_value="teleop_twist.yaml",
            description="YAML file with the teleop_twist_node configuration.",
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

    runtime_config_package = LaunchConfiguration("runtime_config_package")
    joystick_config = LaunchConfiguration("joystick_config")
    teleop_twist_config = LaunchConfiguration("teleop_twist_config")
    use_sim = LaunchConfiguration("use_sim")

    joystick_config_path = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", joystick_config]
    )
    teleop_twist_config_path = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", teleop_twist_config]
    )

    joystick_publisher = Node(
        package="teleop",
        executable="joystick",
        name="joystick",
        output="screen",
        parameters=[
            joystick_config_path,
            {"use_sim_time": use_sim}
        ],
        remappings=[
            ("controller_input", "joy"),
            ("/controller_input", "/joy"),
        ],
    )

    teleop_twist_joy = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy",
        output="screen",
        parameters=[
            teleop_twist_config_path,
            {"use_sim_time": use_sim}
        ],
    )

    return LaunchDescription(
        declared_arguments + [joystick_publisher, teleop_twist_joy]
    )
