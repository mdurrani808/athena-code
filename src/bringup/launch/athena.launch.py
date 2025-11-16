# Copyright (c) 2024
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
Top-level launch file for the complete Athena rover.
Orchestrates all subsystems (drive, arm, etc.) in simulation or hardware mode.

Usage:
  Simulation: ros2 launch bringup athena.launch.py sim:=true
  Hardware:   ros2 launch bringup athena.launch.py sim:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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
            "rviz",
            default_value="true",
            choices=["true", "false"],
            description="Start RViz for visualization.",
        ),
        DeclareLaunchArgument(
            "enable_drive",
            default_value="true",
            choices=["true", "false"],
            description="Enable drive subsystem.",
        ),
        DeclareLaunchArgument(
            "enable_arm",
            default_value="false",  # TODO: Enable when arm is ready
            choices=["true", "false"],
            description="Enable arm subsystem.",
        ),
        DeclareLaunchArgument(
            "enable_teleop",
            default_value="",  # Let subsystems decide defaults
            description="Enable teleoperation. If empty, subsystems use their defaults.",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Robot namespace for multi-robot scenarios.",
        ),
    ]

    # -- Initialize Arguments --
    sim = LaunchConfiguration("sim")
    world = LaunchConfiguration("world")
    rviz = LaunchConfiguration("rviz")
    enable_drive = LaunchConfiguration("enable_drive")
    enable_arm = LaunchConfiguration("enable_arm")
    enable_teleop = LaunchConfiguration("enable_teleop")
    namespace = LaunchConfiguration("namespace")

    # -- Build paths --
    pkg_drive_bringup = FindPackageShare('drive_bringup')
    # pkg_arm_bringup = FindPackageShare('arm_bringup')  # TODO: When arm is ready

    # ========== SUBSYSTEM LAUNCHES ==========

    # Drive subsystem
    drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_drive_bringup, 'launch', 'athena_drive.launch.py'])
        ]),
        launch_arguments={
            'sim': sim,
            'world': world,
            'rviz': rviz,
            'namespace': namespace,
            'enable_teleop': enable_teleop,
        }.items(),
        condition=IfCondition(enable_drive)
    )

    # Arm subsystem (TODO: Implement when ready)
    # arm_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([pkg_arm_bringup, 'launch', 'athena_arm.launch.py'])
    #     ]),
    #     launch_arguments={
    #         'sim': sim,
    #         'namespace': namespace,
    #     }.items(),
    #     condition=IfCondition(enable_arm)
    # )

    return LaunchDescription(
        declared_arguments +
        [
            drive_launch,
            # arm_launch,  # TODO: Uncomment when ready
        ]
    )
