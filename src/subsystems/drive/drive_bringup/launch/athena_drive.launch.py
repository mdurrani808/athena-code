"""
Athena Drive System Launch File
Unified launch file for drive subsystem supporting both simulation and real hardware.

Usage:
  Simulation: ros2 launch drive_bringup athena_drive.launch.py use_sim:=true
  Real Hardware: ros2 launch drive_bringup athena_drive.launch.py use_sim:=false
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Generate launch description based on use_sim parameter."""

    # -- Get Launch Configurations --
    use_sim = LaunchConfiguration("use_sim")
    use_sim_value = use_sim.perform(context)

    runtime_config_package = LaunchConfiguration("runtime_config_package")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    robot_controller = LaunchConfiguration("robot_controller")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_file = LaunchConfiguration("rviz_file")

    # -- Conditional Parameters based on use_sim --
    if use_sim_value == "true":
        use_mock_hardware = "true"
        sim_gazebo = "true"
        use_sim_time = "true"
        controllers_file = "athena_drive_sim_controllers.yaml"
        simulation_controllers_path = PathJoinSubstitution(
            [FindPackageShare(description_package), "config", controllers_file]
        ).perform(context)
    else:
        use_mock_hardware = "false"
        sim_gazebo = "false"
        use_sim_time = "false"
        controllers_file = "athena_drive_controllers.yaml"
        simulation_controllers_path = ""

    # -- Build File Paths --
    pkg_drive_bringup = FindPackageShare(runtime_config_package).perform(context)
    pkg_description = FindPackageShare(description_package).perform(context)

    robot_controllers_path = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", rviz_file]
    )

    # -- Include Modular Launch Files --

    # 1. Robot Description (robot_state_publisher)
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_drive_bringup, "launch", "robot_description.launch.py"])
        ]),
        launch_arguments={
            "description_package": description_package,
            "description_file": description_file,
            "prefix": prefix,
            "use_mock_hardware": use_mock_hardware,
            "mock_sensor_commands": "false",
            "sim_gazebo": sim_gazebo,
            "simulation_controllers": simulation_controllers_path,
            "use_sim_time": use_sim_time,
        }.items()
    )

    # 2. Teleoperation (joystick + teleop_twist)
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_drive_bringup, "launch", "teleop.launch.py"])
        ]),
        launch_arguments={
            "runtime_config_package": runtime_config_package,
            "joystick_config": "joystick.yaml",
            "teleop_twist_config": "teleop_twist.yaml",
        }.items()
    )

    # 3. Hardware-specific nodes (spawner for sim, CAN for real)
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_drive_bringup, "launch", "hardware.launch.py"])
        ]),
        launch_arguments={
            "use_sim": use_sim,
            "namespace": "",
            "robot_name": "rover",
            "spawn_x": "0.0",
            "spawn_y": "0.0",
            "spawn_z": "3.0",
            "spawn_yaw": "0.0",
        }.items()
    )

    # 4. Controllers (spawned after ros2_control_node)
    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_drive_bringup, "launch", "controllers.launch.py"])
        ]),
        launch_arguments={
            "robot_controller": robot_controller,
            "start_rviz": start_rviz,
            "rviz_config": rviz_config_file,
            "use_sim_time": use_sim_time,
        }.items()
    )

    # -- ros2_control Node --
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_controllers_path.perform(context), {"use_sim_time": use_sim_time == "true"}],
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/single_ackermann_controller/reference", "/joy"),
            ("/ackermann_steering_controller/reference", "/cmd_vel"),
        ],
    )

    # -- Delay controllers after ros2_control_node starts --
    delay_controllers_after_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=5.0,  # Ensure hardware interfaces are fully initialized
                    actions=[controllers_launch],
                ),
            ],
        )
    )

    return [
        robot_description_launch,
        teleop_launch,
        hardware_launch,
        control_node,
        delay_controllers_after_control_node,
    ]


def generate_launch_description():
    # -- Declare Arguments --
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            choices=["true", "false"],
            description="Use simulation mode (true) or real hardware mode (false).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="drive_bringup",
            description='Package with the controller configuration in "config" folder.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="description",
            description="Description package with robot URDF/xacro files.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="athena_drive.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
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
            "prefix",
            default_value='""',
            description="Prefix of the joint names for multi-robot setup.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="single_ackermann_controller",
            choices=["single_ackermann_controller", "ackermann_steering_controller"],
            description="Robot controller to start.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="false",
            choices=["true", "false"],
            description="Start RViz2 for visualization.",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
