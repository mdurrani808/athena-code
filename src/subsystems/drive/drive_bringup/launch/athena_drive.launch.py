from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    use_sim = LaunchConfiguration("use_sim")
    use_sim_value = use_sim.perform(context)

    runtime_config_package = LaunchConfiguration("runtime_config_package")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    robot_controller = LaunchConfiguration("robot_controller")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_file = LaunchConfiguration("rviz_file")
    control_node_startup_delay = LaunchConfiguration("control_node_startup_delay")
    world = LaunchConfiguration("world")

    pkg_drive_bringup = FindPackageShare(runtime_config_package).perform(context)
    pkg_simulation = FindPackageShare("simulation").find("simulation")

    robot_controllers_path = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", "athena_drive_controllers.yaml"]
    )

    if use_sim_value == "true":
        simulation_controllers_path = robot_controllers_path.perform(context)
    else:
        simulation_controllers_path = ""

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", rviz_file]
    )

    # Launch simulation infrastructure if in simulation mode
    sim_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_simulation, "launch", "sim_bringup.launch.py"])
        ]),
        launch_arguments={
            "world": world,
            "use_sim_time": "true",
        }.items(),
        condition=IfCondition(use_sim)
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_drive_bringup, "launch", "robot_description.launch.py"])
        ]),
        launch_arguments={
            "description_package": description_package,
            "description_file": description_file,
            "prefix": prefix,
            "use_sim": use_sim,
            "simulation_controllers": simulation_controllers_path,
        }.items()
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_drive_bringup, "launch", "teleop.launch.py"])
        ]),
        launch_arguments={
            "runtime_config_package": runtime_config_package,
            "joystick_config": "joystick.yaml",
            "teleop_twist_config": "teleop_twist.yaml",
            "use_sim": use_sim,
        }.items()
    )

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

    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_drive_bringup, "launch", "controllers.launch.py"])
        ]),
        launch_arguments={
            "robot_controller": robot_controller,
            "use_sim": use_sim,
        }.items()
    )

    visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_drive_bringup, "launch", "visualization.launch.py"])
        ]),
        launch_arguments={
            "description_package": description_package,
            "rviz_file": rviz_file,
            "use_sim": use_sim,
        }.items(),
        condition=IfCondition(start_rviz)
    )

    # Only launch ros2_control_node for real hardware
    # For simulation, Gazebo's GazeboSimROS2ControlPlugin manages control
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            robot_controllers_path.perform(context),
            {"use_sim_time": use_sim}
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/single_ackermann_controller/reference", "/joy"),
            ("/ackermann_steering_controller/reference", "/cmd_vel"),
        ],
        condition=UnlessCondition(use_sim),
    )

    delay_controllers_after_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=float(control_node_startup_delay.perform(context)),
                    actions=[controllers_launch],
                ),
            ],
        ),
        condition=UnlessCondition(use_sim),
    )

    sim_topic_remapping_nodes = []
    if use_sim_value == "true":
        # Remap /cmd_vel to /ackermann_steering_controller/reference for simulation
        sim_topic_remapping_nodes = [
            Node(
                package="topic_tools",
                executable="relay",
                name="cmd_vel_to_ackermann_relay",
                arguments=["/cmd_vel", "/ackermann_steering_controller/reference"],
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="topic_tools",
                executable="relay",
                name="joy_to_single_ackermann_relay",
                arguments=["/joy", "/single_ackermann_controller/reference"],
                parameters=[{"use_sim_time": True}],
            ),
        ]


    # For simulation, delay controllers after hardware spawn (when Gazebo plugin loads)
    nodes_to_launch = []

    # Add sim_bringup first if in simulation mode
    if use_sim_value == "true":
        nodes_to_launch.append(sim_bringup_launch)

    nodes_to_launch.extend([
        robot_description_launch,
        teleop_launch,
        hardware_launch,
    ])

    # Add control node and delayed controllers only for real hardware
    if use_sim.perform(context) == "false":
        nodes_to_launch.extend([
            control_node,
            delay_controllers_after_control_node,
        ])
    else:
        # In simulation, launch controllers after a delay to let Gazebo initialize
        nodes_to_launch.append(
            TimerAction(
                period=float(control_node_startup_delay.perform(context)),
                actions=[controllers_launch],
            )
        )

    # Conditionally add visualization if start_rviz is true
    if start_rviz.perform(context) == "true":
        nodes_to_launch.append(visualization_launch)

    return nodes_to_launch


def generate_launch_description():
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
            default_value="ackermann_steering_controller",
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

    declared_arguments.append(
        DeclareLaunchArgument(
            "control_node_startup_delay",
            default_value="5.0",
            description="Delay in seconds before starting controllers after ros2_control_node.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value="empty.sdf",
            description="Gazebo world file to load (only used when use_sim:=true).",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
