from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def spawn_robot_logic(context, *args, **kwargs):
    use_sim = LaunchConfiguration("use_sim").perform(context)
    if use_sim != "true":
        return []

    robot_name = LaunchConfiguration("robot_name").perform(context)
    world_name = LaunchConfiguration("world_name").perform(context)
    spawn_x = LaunchConfiguration("spawn_x").perform(context)
    spawn_y = LaunchConfiguration("spawn_y").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    spawn_yaw = LaunchConfiguration("spawn_yaw").perform(context)

    spawn_args = [
        "-name", robot_name,
        "-x", spawn_x,
        "-y", spawn_y,
        "-z", spawn_z,
        "-Y", spawn_yaw,
        "-topic", "robot_description",
    ]
    if world_name:
        spawn_args = ["-name", robot_name, "-world", world_name,
                      "-x", spawn_x, "-y", spawn_y, "-z", spawn_z,
                      "-Y", spawn_yaw, "-topic", "robot_description"]

    return [Node(
        package="ros_gz_sim",
        executable="create",
        arguments=spawn_args,
        output="screen",
    )]


def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        choices=["true", "false"],
        description="Use simulation (spawns robot in Gazebo) or real hardware (launches CAN node).",
    )

    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="rover",
        description="Name of the robot in Gazebo.",
    )

    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="",
        description="Name of the world inside Gazebo (optional; omitted from create args if empty).",
    )

    spawn_args = [
        DeclareLaunchArgument("spawn_x", default_value="0.0", description="Spawn X"),
        DeclareLaunchArgument("spawn_y", default_value="0.0", description="Spawn Y"),
        DeclareLaunchArgument("spawn_z", default_value="3.0", description="Spawn Z"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0", description="Spawn Yaw"),
    ]

    return LaunchDescription(
        [
            use_sim_arg,
            robot_name_arg,
            world_name_arg,
            *spawn_args,
            OpaqueFunction(function=spawn_robot_logic),
        ]
    )
