from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        choices=["true", "false"],
        description="Use simulation mode (true) or real hardware mode (false)",
    )

    runtime_config_package_arg = DeclareLaunchArgument(
        "runtime_config_package",
        default_value="drive_bringup",
        description="Package with the controller configuration in 'config' folder",
    )

    description_package_arg = DeclareLaunchArgument(
        "description_package",
        default_value="description",
        description="Description package with robot URDF/xacro files",
    )

    description_file_arg = DeclareLaunchArgument(
        "description_file",
        default_value="athena_drive.urdf.xacro",
        description="URDF/XACRO description file with the robot",
    )

    rviz_file_arg = DeclareLaunchArgument(
        "rviz_file",
        default_value="athena_drive.rviz",
        description="RViz config file",
    )

    prefix_arg = DeclareLaunchArgument(
        "prefix",
        default_value='""',
        description="Prefix of the joint names for multi-robot setup",
    )

    robot_controller_arg = DeclareLaunchArgument(
        "robot_controller",
        default_value="ackermann_steering_controller",
        choices=["single_ackermann_controller", "ackermann_steering_controller"],
        description="Robot controller to start",
    )

    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="false",
        choices=["true", "false"],
        description="Start RViz2 for visualization",
    )

    control_node_startup_delay_arg = DeclareLaunchArgument(
        "control_node_startup_delay",
        default_value="5.0",
        description="Delay in seconds before starting controllers",
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="empty.sdf",
        description="Gazebo world file to load (only used when use_sim:=true)",
    )

    use_sim = LaunchConfiguration("use_sim")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    robot_controller = LaunchConfiguration("robot_controller")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_file = LaunchConfiguration("rviz_file")
    control_node_startup_delay = LaunchConfiguration("control_node_startup_delay")
    world = LaunchConfiguration("world")

    robot_controllers_path = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", "athena_drive_controllers.yaml"]
    )

    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("simulation"),
                "launch",
                "sim_bringup.launch.py"
            ])
        ]),
        launch_arguments={
            "world": world,
            "use_sim_time": "true",
        }.items(),
        condition=IfCondition(use_sim),
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(runtime_config_package),
                "launch",
                "robot_description.launch.py"
            ])
        ]),
        launch_arguments={
            "description_package": description_package,
            "description_file": description_file,
            "prefix": prefix,
            "use_sim": use_sim,
            "simulation_controllers": robot_controllers_path,
        }.items(),
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(runtime_config_package),
                "launch",
                "teleop.launch.py"
            ])
        ]),
        launch_arguments={
            "runtime_config_package": runtime_config_package,
            "joystick_config": "joystick.yaml",
            "teleop_twist_config": "teleop_twist.yaml",
            "use_sim": use_sim,
        }.items(),
    )

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(runtime_config_package),
                "launch",
                "hardware.launch.py"
            ])
        ]),
        launch_arguments={
            "use_sim": use_sim,
            "namespace": "",
            "robot_name": "rover",
            "spawn_x": "0.0",
            "spawn_y": "0.0",
            "spawn_z": "3.0",
            "spawn_yaw": "0.0",
        }.items(),
    )

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(runtime_config_package),
                "launch",
                "controllers.launch.py"
            ])
        ]),
        launch_arguments={
            "robot_controller": robot_controller,
            "use_sim": use_sim,
        }.items(),
    )

    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(runtime_config_package),
                "launch",
                "visualization.launch.py"
            ])
        ]),
        launch_arguments={
            "description_package": description_package,
            "rviz_file": rviz_file,
            "use_sim": use_sim,
        }.items(),
        condition=IfCondition(start_rviz),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            robot_controllers_path,
            {"use_sim_time": use_sim}
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/single_ackermann_controller/reference", "/joy"),
            ("/ackermann_steering_controller/reference", "/cmd_vel"),
        ],
        condition=UnlessCondition(use_sim),
    )

    delayed_controllers_real = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=control_node_startup_delay,
                    actions=[controllers],
                ),
            ],
        ),
        condition=UnlessCondition(use_sim),
    )

    delayed_controllers_sim = TimerAction(
        period=control_node_startup_delay,
        actions=[controllers],
        condition=IfCondition(use_sim),
    )

    cmd_vel_relay = Node(
        package="topic_tools",
        executable="relay",
        name="cmd_vel_to_ackermann_relay",
        arguments=[
            "/cmd_vel",
            "/ackermann_steering_controller/reference",
            "--ros-args",
            "--log-level",
            "error"
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_sim),
    )

    joy_relay = Node(
        package="topic_tools",
        executable="relay",
        name="joy_to_single_ackermann_relay",
        arguments=[
            "/joy",
            "/single_ackermann_controller/reference",
            "--ros-args",
            "--log-level",
            "error"
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_sim),
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_arg,
        runtime_config_package_arg,
        description_package_arg,
        description_file_arg,
        rviz_file_arg,
        prefix_arg,
        robot_controller_arg,
        start_rviz_arg,
        control_node_startup_delay_arg,
        world_arg,
        # Launch files and nodes
        sim_bringup,
        robot_description,
        teleop,
        hardware,
        control_node,
        delayed_controllers_real,
        delayed_controllers_sim,
        cmd_vel_relay,
        joy_relay,
        visualization,
    ])