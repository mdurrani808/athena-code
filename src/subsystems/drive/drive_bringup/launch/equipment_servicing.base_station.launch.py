from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # -- Declare arguments --
    declared_arguments = []
    
    # Drive arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "drive_runtime_config_package",
            default_value="drive_bringup",
            description='Package with the drive controller\'s configuration in "config" folder.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "drive_joystick_config",
            default_value="joystick.yaml",
            description="YAML file with the drive joystick configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "teleop_twist_config",
            default_value="teleop_twist.yaml",
            description="YAML file with the teleop_twist_node configuration.",
        )
    )
    
    # Arm arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_runtime_config_package",
            default_value="arm_bringup",
            description='Package with the arm controller\'s configuration in "config" folder.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_joystick_config",
            default_value="joystick.yaml",
            description="YAML file with the arm joystick configuration.",
        )
    )

    # -- Initialize Arguments --
    drive_runtime_config_package = LaunchConfiguration("drive_runtime_config_package")
    drive_joystick_config = LaunchConfiguration("drive_joystick_config")
    teleop_twist_config = LaunchConfiguration("teleop_twist_config")
    arm_runtime_config_package = LaunchConfiguration("arm_runtime_config_package")
    arm_joystick_config = LaunchConfiguration("arm_joystick_config")

    # -- Building Path Files --
    drive_joystick_config_path = PathJoinSubstitution(
        [FindPackageShare(drive_runtime_config_package), "config", drive_joystick_config]
    )
    teleop_twist_config_path = PathJoinSubstitution(
        [FindPackageShare(drive_runtime_config_package), "config", teleop_twist_config]
    )
    arm_joystick_config_path = PathJoinSubstitution(
        [FindPackageShare(arm_runtime_config_package), "config", arm_joystick_config]
    )

    # -- Drive Nodes --
    drive_joystick_publisher = Node(
        package='teleop',
        executable='joystick',
        name='drive_joystick',
        output='screen',
        parameters=[drive_joystick_config_path],
        remappings=[
            ('controller_input', 'joy'),
            ('/controller_input', '/joy'),
        ],
    )

    teleop_twist_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[teleop_twist_config_path],
        remappings=[
            ('/cmd_vel', '/rear_ackermann_controller/reference'),
        ],
    )

    # -- Arm Nodes --
    arm_joystick_publisher = Node(
        package='teleop',
        executable='joystick',
        name='arm_joystick',
        output='screen',
        parameters=[arm_joystick_config_path],
    )

    return LaunchDescription(
        declared_arguments + 
        [
            drive_joystick_publisher,
            teleop_twist_joy,
            arm_joystick_publisher,
        ]
    )
