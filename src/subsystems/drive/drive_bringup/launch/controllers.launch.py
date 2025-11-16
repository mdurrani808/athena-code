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
Launch file for spawning drive controllers.
Handles sequential controller spawning with proper event handlers.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    # Arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_controller",
            default_value="single_ackermann_controller",
            choices=["single_ackermann_controller", "ackermann_steering_controller"],
            description="Robot controller to start.",
        ),
        DeclareLaunchArgument(
            "start_controller_switcher",
            default_value="true",
            choices=["true", "false"],
            description="Start the controller switcher service for runtime switching.",
        ),
    ]

    robot_controller = LaunchConfiguration("robot_controller")
    start_controller_switcher = LaunchConfiguration("start_controller_switcher")

    # Joint state broadcaster (must spawn first)
    joint_state_broadcaster_spawner = create_controller_spawner("joint_state_broadcaster")

    # Active controllers (spawned and started)
    active_controller_names = [robot_controller]

    # Inactive controllers (loaded but not started, for runtime switching)
    inactive_controller_names = [
        "ackermann_steering_controller",
        "drive_velocity_controller",
        "drive_position_controller"
    ]

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

    return LaunchDescription(
        declared_arguments +
        [joint_state_broadcaster_spawner]
        + active_controller_handlers
        + inactive_controller_handlers
        + [controller_switcher_node]
    )
