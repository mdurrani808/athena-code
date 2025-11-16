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
Common utilities for generating robot descriptions from URDF/xacro files.
"""

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration


def build_robot_description_command(robot_description_path, prefix="",
                                    use_mock_hardware=False, mock_sensor_commands=False,
                                    sim_gazebo=False, simulation_controllers="",
                                    additional_args=None):
    """
    Build a Command substitution for generating robot description from xacro.

    Args:
        robot_description_path: PathJoinSubstitution to the xacro file
        prefix: Robot prefix for multi-robot setups
        use_mock_hardware: Enable mock hardware plugin
        sim_gazebo: Enable Gazebo simulation
        simulation_controllers: Path to simulation controllers file (for Gazebo)
        additional_args: Dictionary of additional xacro arguments

    Returns:
        Command substitution that generates the robot description
    """
    cmd_parts = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        robot_description_path,
    ]

    # Add standard arguments
    if prefix:
        cmd_parts.extend([" ", "prefix:=", prefix])

    if use_mock_hardware:
        cmd_parts.extend([" ", "use_mock_hardware:=", use_mock_hardware])

    if mock_sensor_commands:
        cmd_parts.extend([" ", "mock_sensor_commands:=", mock_sensor_commands])

    if sim_gazebo:
        cmd_parts.extend([" ", "sim_gazebo:=", sim_gazebo])

    if simulation_controllers:
        cmd_parts.extend([" ", "simulation_controllers:=", simulation_controllers])

    # Add any additional arguments
    if additional_args:
        for key, value in additional_args.items():
            cmd_parts.extend([" ", f"{key}:=", value])

    return Command(cmd_parts)


def build_robot_description_dict(robot_description_content):
    """
    Build a dictionary containing the robot description for use in node parameters.

    Args:
        robot_description_content: The robot description content (typically from Command)

    Returns:
        Dictionary with 'robot_description' key
    """
    return {"robot_description": robot_description_content}
