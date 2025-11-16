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
Common utilities for creating standard ROS2 nodes used in drive system.
"""

from launch_ros.actions import Node
from launch.conditions import IfCondition


def create_robot_state_publisher(robot_description, use_sim_time=False):
    """
    Create a robot_state_publisher node.

    Args:
        robot_description: Dictionary containing robot description
        use_sim_time: LaunchConfiguration or bool for sim time

    Returns:
        Node action for robot_state_publisher
    """
    params = [robot_description]
    if use_sim_time:
        params.append({"use_sim_time": use_sim_time})

    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=params,
    )


def create_control_node(robot_controllers_path, remappings=None):
    """
    Create a ros2_control_node for the controller manager.

    Args:
        robot_controllers_path: PathJoinSubstitution to controllers config file
        remappings: Optional list of topic remappings

    Returns:
        Node action for ros2_control_node
    """
    if remappings is None:
        remappings = [("~/robot_description", "/robot_description")]

    return Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_controllers_path],
        remappings=remappings,
    )


def create_rviz_node(rviz_config_path, condition=None):
    """
    Create an RViz2 node.

    Args:
        rviz_config_path: PathJoinSubstitution to RViz config file
        condition: Optional condition for launching (e.g., IfCondition)

    Returns:
        Node action for RViz2
    """
    node_args = {
        "package": "rviz2",
        "executable": "rviz2",
        "name": "rviz2",
        "output": "log",
        "arguments": ["-d", rviz_config_path],
    }

    if condition:
        node_args["condition"] = condition

    return Node(**node_args)


def create_joint_state_publisher(use_gui=False):
    """
    Create a joint_state_publisher node.

    Args:
        use_gui: If True, creates joint_state_publisher_gui instead

    Returns:
        Node action for joint_state_publisher
    """
    if use_gui:
        return Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        )
    else:
        return Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        )
