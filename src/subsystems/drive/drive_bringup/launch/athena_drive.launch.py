# Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Unified launch file for Athena drive system (hardware and simulation).
Use sim:=true for Gazebo simulation, sim:=false for hardware.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # -- Declare arguments --
    declared_arguments = [
        DeclareLaunchArgument(
            "sim",
            default_value="false",
            choices=["true", "false"],
            description="Launch in simulation mode (Gazebo) vs hardware mode.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="empty.sdf",
            description="Gazebo world file (simulation only).",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Robot namespace.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            choices=["true", "false"],
            description="Start RViz for visualization.",
        ),
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="drive_bringup",
            description='Package with controller and teleop configs.',
        ),
        DeclareLaunchArgument(
            "joystick_config",
            default_value="joystick.yaml",
            description="YAML file with joystick configuration.",
        ),
        DeclareLaunchArgument(
            "teleop_twist_config",
            default_value="teleop_twist.yaml",
            description="YAML file with teleop_twist_node configuration.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="athena_drive_controllers.yaml",
            description="YAML file with controllers configuration.",
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="description",
            description="Description package with robot URDF/xacro files.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="athena_drive.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "rviz_file",
            default_value="athena_drive.rviz",
            description="RViz config file.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of joint names for multi-robot setups.",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            choices=["true", "false"],
            description="Use mock hardware (hardware mode only).",
        ),
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            choices=["true", "false"],
            description="Enable mock sensor commands (if use_mock_hardware=true).",
        ),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="",  # Will be set conditionally based on sim
            description="Robot controller to start. Defaults: sim=ackermann_steering, hw=single_ackermann.",
        ),
        DeclareLaunchArgument(
            "enable_teleop",
            default_value="",  # Will be set conditionally based on sim
            description="Enable joystick/teleop. Defaults: sim=false, hw=true.",
        ),
        DeclareLaunchArgument(
            "start_controller_switcher",
            default_value="true",
            choices=["true", "false"],
            description="Start controller switcher service for runtime switching.",
        ),
    ]

    # -- Initialize Arguments --
    sim = LaunchConfiguration("sim")
    world = LaunchConfiguration("world")
    namespace = LaunchConfiguration("namespace")
    rviz_arg = LaunchConfiguration("rviz")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    joystick_config = LaunchConfiguration("joystick_config")
    teleop_twist_config = LaunchConfiguration("teleop_twist_config")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    rviz_file = LaunchConfiguration("rviz_file")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    robot_controller_arg = LaunchConfiguration("robot_controller")
    enable_teleop_arg = LaunchConfiguration("enable_teleop")
    start_controller_switcher = LaunchConfiguration("start_controller_switcher")

    # Conditional defaults based on sim mode
    robot_controller = PythonExpression([
        "'", robot_controller_arg, "' if '", robot_controller_arg, "' != '' else ",
        "('ackermann_steering_controller' if ", sim, " == 'true' else 'single_ackermann_controller')"
    ])

    enable_teleop = PythonExpression([
        "'", enable_teleop_arg, "' if '", enable_teleop_arg, "' != '' else ",
        "('false' if ", sim, " == 'true' else 'true')"
    ])

    use_sim_time = sim

    # -- Build paths --
    pkg_description = FindPackageShare(description_package)
    pkg_drive_bringup = FindPackageShare(runtime_config_package)
    pkg_simulation = FindPackageShare('simulation')

    urdf_file = PathJoinSubstitution([pkg_description, 'urdf', description_file])
    controllers_config = PathJoinSubstitution([pkg_drive_bringup, 'config', controllers_file])
    rviz_config_file = PathJoinSubstitution([pkg_description, 'rviz', rviz_file])

    # -- Robot Description --
    # Simulation uses different xacro args than hardware
    robot_description_content = PythonExpression([
        "'xacro ", urdf_file, " use_mock_hardware:=true sim_gazebo:=true simulation_controllers:=",
        controllers_config, "' if '", sim, "' == 'true' else ",
        "'xacro ", urdf_file, " prefix:=", prefix, " use_mock_hardware:=", use_mock_hardware,
        " mock_sensor_commands:=", mock_sensor_commands, "'"
    ])

    robot_description_command = Command(robot_description_content)
    robot_description = {"robot_description": robot_description_command}

    # ========== SIMULATION MODE ==========

    # Launch Gazebo (sim only)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_simulation, 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world
        }.items(),
        condition=IfCondition(sim)
    )

    # ROS-Gazebo bridge (sim only)
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_simulation, 'launch', 'bridge.launch.py'])
        ]),
        condition=IfCondition(sim)
    )

    # Robot spawning in Gazebo (sim only)
    spawn_robot = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_command, value_type=str),
            'use_sim_time': use_sim_time
        }],
        namespace=namespace,
        condition=IfCondition(sim)
    )

    create_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rover',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '3.0',
            '-Y', '0.0',
            '-topic', 'robot_description'
        ],
        output='screen',
        namespace=namespace,
        condition=IfCondition(sim)
    )

    # ========== HARDWARE MODE ==========

    # ros2_control_node (hardware only)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[controllers_config],
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/single_ackermann_controller/reference", "/joy"),
            ("/ackermann_steering_controller/reference", "/cmd_vel"),
        ],
        condition=UnlessCondition(sim)
    )

    # robot_state_publisher (hardware only)
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        condition=UnlessCondition(sim)
    )

    # joint_state_publisher (hardware only, for testing without real hardware)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(sim)
    )

    # ========== COMMON COMPONENTS ==========

    # RViz (both modes)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz_arg),
    )

    # Controllers (both modes)
    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_drive_bringup, 'launch', 'controllers.launch.py'])
        ]),
        launch_arguments={
            'robot_controller': robot_controller,
            'start_controller_switcher': start_controller_switcher,
        }.items()
    )

    # Delay controller spawning until control infrastructure is ready
    # In sim: controllers are spawned immediately (Gazebo handles ros2_control)
    # In hw: delay after control_node starts
    delay_controllers_after_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[controllers_launch],
                ),
            ],
        ),
        condition=UnlessCondition(sim)
    )

    # Teleop (both modes, optional)
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_drive_bringup, 'launch', 'teleop.launch.py'])
        ]),
        launch_arguments={
            'runtime_config_package': runtime_config_package,
            'joystick_config': joystick_config,
            'teleop_twist_config': teleop_twist_config,
        }.items(),
        condition=IfCondition(enable_teleop),
    )

    # CAN node for hardware (currently disabled)
    # Uncomment when needed for actual hardware
    # umdloop_can_node = Node(
    #     package='umdloop_can',
    #     executable='can_node',
    #     name='can_node',
    #     output='log',
    #     arguments=['--ros-args', '--log-level', 'fatal'],
    #     condition=UnlessCondition(sim)
    # )

    return LaunchDescription(
        declared_arguments +
        [
            # Simulation infrastructure
            gazebo_launch,
            bridge_launch,
            spawn_robot,
            create_robot,
            # Hardware infrastructure
            control_node,
            robot_state_pub_node,
            joint_state_publisher,
            # Common components
            rviz_node,
            controllers_launch,  # Direct in sim
            delay_controllers_after_control_node,  # Delayed in hardware
            teleop_launch,
        ]
    )
