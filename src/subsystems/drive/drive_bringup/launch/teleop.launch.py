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
Launch file for teleoperation nodes (joystick and teleop_twist_joy).
Optional subsystem for manual control of the rover.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="drive_bringup",
            description='Package with the teleop configuration in "config" folder.',
        ),
        DeclareLaunchArgument(
            "joystick_config",
            default_value="joystick.yaml",
            description="YAML file with the joystick configuration.",
        ),
        DeclareLaunchArgument(
            "teleop_twist_config",
            default_value="teleop_twist.yaml",
            description="YAML file with the teleop_twist_node configuration.",
        ),
    ]

    runtime_config_package = LaunchConfiguration("runtime_config_package")
    joystick_config = LaunchConfiguration("joystick_config")
    teleop_twist_config = LaunchConfiguration("teleop_twist_config")

    # Build config paths
    joystick_config_path = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", joystick_config]
    )
    teleop_twist_config_path = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", teleop_twist_config]
    )

    # Joystick publisher node
    joystick_publisher = Node(
        package='teleop',
        executable='joystick',
        name='joystick',
        output='screen',
        parameters=[joystick_config_path],
        remappings=[
            ('controller_input', 'joy'),
            ('/controller_input', '/joy'),
        ],
    )

    # Teleop twist joy node
    teleop_twist_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[teleop_twist_config_path],
    )

    return LaunchDescription(
        declared_arguments +
        [
            joystick_publisher,
            teleop_twist_joy,
        ]
    )
