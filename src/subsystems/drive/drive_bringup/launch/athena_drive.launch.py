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

#
# Source of this file are templates in
# [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
#
# Author: Dr. Denis
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def create_controller_spawner(controller_name, inactive=False):
    """Create a controller spawner node."""
    args = [controller_name, "-c", "/controller_manager"]
    if inactive:
        args.append("--inactive")
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=args,
        output="screen"
    )


def create_sequential_spawners(controller_names, start_after, inactive=False):
    """Create event handlers to spawn controllers sequentially."""
    spawners = [create_controller_spawner(name, inactive) for name in controller_names]
    handlers = []

    for i, spawner in enumerate(spawners):
        target = start_after if i == 0 else spawners[i - 1]
        handlers.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=target,
                    on_exit=[spawner],
                )
            )
        )

    return handlers, spawners


def generate_launch_description():
    # -- Declare arguments --
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="drive_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
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
            "controllers_file",
            default_value="athena_drive_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
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
            description="Rviz config file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations. \
            Used only if 'use_mock_hardware' parameter is true.",
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

    # -- Initialize Arguments --
    use_sim = LaunchConfiguration("use_sim")
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
    robot_controller = LaunchConfiguration("robot_controller")

     # -- Building Path Files --
    robot_description_path = PathJoinSubstitution(
        [FindPackageShare(description_package), "urdf", description_file]
    )
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )
    joystick_config = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", joystick_config]
    )
    teleop_twist_config = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", teleop_twist_config]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", rviz_file]
    )

    # -- Additional Configuration Setup --
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_description_path,
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # -- Node Definitions -- 
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/single_ackermann_controller/reference", "/joy"),
            ("/ackermann_steering_controller/reference", "/cmd_vel"),
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_sim),
    )

    joint_state_broadcaster_spawner = create_controller_spawner("joint_state_broadcaster")

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Active controllers (spawned first)
    active_controller_names = [robot_controller]

    # Inactive controllers (loaded but not started, for runtime switching)
    inactive_controller_names = ["ackermann_steering_controller", "drive_velocity_controller", "drive_position_controller"]

    # Spawn active controllers sequentially after joint_state_broadcaster
    active_controller_handlers, active_spawners = create_sequential_spawners(
        active_controller_names,
        start_after=joint_state_broadcaster_spawner
    )

    # Spawn inactive controllers sequentially after active controllers
    inactive_controller_handlers, inactive_spawners = create_sequential_spawners(
        inactive_controller_names,
        start_after=active_spawners[-1] if active_spawners else joint_state_broadcaster_spawner,
        inactive=True
    )

    # Start controller switcher service after all controllers are loaded
    controller_switcher_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=inactive_spawners[-1],
            on_exit=[TimerAction(
                period=3.0,
                actions=[Node(
                    package="drive_bringup",
                    executable="controller_switcher.py",
                    name="controller_switcher",
                    output="screen"
                )]
            )],
        )
    )

    # Delay joint_state_broadcaster after control node starts
    delay_joint_state_broadcaster_spawner_after_ros2_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=5.0,  # Increased delay to ensure hardware interfaces are fully initialized
                    actions=[joint_state_broadcaster_spawner],
                ),
            ],
        )
    )

    # Start RViz after joint_state_broadcaster
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    umdloop_can_node = Node(
        package='umdloop_can',
        executable='can_node',
        name='can_node',
        output='log',
        arguments=['--ros-args', '--log-level', 'fatal']
    )

    delay_can_node_after_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=1.0,  # Small delay to let control node initialize
                    actions=[umdloop_can_node],
                ),
            ],
        )
    )

    joystick_publisher = Node(
        package='teleop',
        executable='joystick',
        name='joystick',
        output='screen',
        parameters = [joystick_config],
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
        parameters = [teleop_twist_config],
    )


    return LaunchDescription(
        declared_arguments +
        [
            control_node,
            robot_state_pub_node,
            joystick_publisher,
            teleop_twist_joy,
            joint_state_publisher,
            # delay_can_node_after_control_node,
            delay_joint_state_broadcaster_spawner_after_ros2_control_node,
            delay_rviz_after_joint_state_broadcaster_spawner,
            controller_switcher_node,
        ]
        + active_controller_handlers
        + inactive_controller_handlers
    )