"""
Hardware-Specific Launch File
Handles hardware interface nodes:
- Simulation: Robot spawning in Gazebo
- Real: CAN bus communication node

Uses use_sim flag to determine which nodes to launch.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def launch_setup(context, *args, **kwargs):
    """Generate launch description based on use_sim parameter."""

    # Get launch configurations
    use_sim = LaunchConfiguration("use_sim")
    namespace = LaunchConfiguration("namespace")
    robot_name = LaunchConfiguration("robot_name")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    # Simulation-specific: Robot spawner
    spawn_robot_node = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-name", robot_name,
                    "-x", spawn_x,
                    "-y", spawn_y,
                    "-z", spawn_z,
                    "-Y", spawn_yaw,
                    "-topic", "robot_description"
                ],
                output="screen",
                condition=IfCondition(use_sim),
            ),
        ]
    )

    # Real hardware-specific: CAN node
    umdloop_can_node = Node(
        package="umdloop_can",
        executable="can_node",
        name="can_node",
        output="log",
        arguments=["--ros-args", "--log-level", "fatal"],
        condition=UnlessCondition(use_sim),
    )

    # Joint state publisher for simulation (publishes zero states initially)
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        condition=IfCondition(use_sim),
    )

    return [
        spawn_robot_node,
        umdloop_can_node,
        joint_state_publisher,
    ]


def generate_launch_description():
    # -- Declare arguments --
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            choices=["true", "false"],
            description="Use simulation (spawns robot in Gazebo) or real hardware (launches CAN node).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Robot namespace for spawning in Gazebo.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="rover",
            description="Name of the robot in Gazebo.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_x",
            default_value="0.0",
            description="Spawn position X coordinate.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_y",
            default_value="0.0",
            description="Spawn position Y coordinate.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_z",
            default_value="3.0",
            description="Spawn position Z coordinate.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_yaw",
            default_value="0.0",
            description="Spawn yaw orientation.",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
