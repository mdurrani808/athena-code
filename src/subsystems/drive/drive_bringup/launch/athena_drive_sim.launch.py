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
Launch file for Athena drive simulation in Gazebo.
Orchestrates Gazebo, robot spawning, controllers, and optional teleop.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # -- Declare arguments --
    declared_arguments = [
        DeclareLaunchArgument(
            'world',
            default_value='empty.sdf',
            description='Gazebo world file to load'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            choices=['true', 'false'],
            description='Start RViz for visualization'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Use simulation time from Gazebo'
        ),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="ackermann_steering_controller",
            choices=["single_ackermann_controller", "ackermann_steering_controller"],
            description="Robot controller to start in simulation.",
        ),
        DeclareLaunchArgument(
            "enable_teleop",
            default_value="false",
            choices=["true", "false"],
            description="Enable joystick and teleop_twist nodes for manual control.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="athena_drive_controllers.yaml",
            description="YAML file with the controllers configuration.",
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
    ]

    # -- Initialize Arguments --
    world = LaunchConfiguration('world')
    namespace = LaunchConfiguration('namespace')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_controller = LaunchConfiguration('robot_controller')
    enable_teleop = LaunchConfiguration('enable_teleop')
    controllers_file = LaunchConfiguration('controllers_file')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    rviz_file = LaunchConfiguration('rviz_file')

    # -- Build paths --
    pkg_simulation = FindPackageShare('simulation')
    pkg_description = FindPackageShare(description_package)
    pkg_drive_bringup = FindPackageShare('drive_bringup')

    urdf_file = PathJoinSubstitution([pkg_description, 'urdf', description_file])
    controllers_config = PathJoinSubstitution([pkg_drive_bringup, 'config', controllers_file])
    rviz_config_file = PathJoinSubstitution([pkg_description, 'rviz', rviz_file])

    # -- Robot Description for Gazebo --
    robot_description_content = Command([
        'xacro ', urdf_file,
        ' use_mock_hardware:=true',
        ' sim_gazebo:=true',
        f' simulation_controllers:=', controllers_config
    ])

    # -- Launch Gazebo --
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_simulation, 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world
        }.items()
    )

    # -- Spawn Robot in Gazebo --
    robot_name = 'rover'
    spawn_robot = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                robot_description_content,
                value_type=str
            ),
            'use_sim_time': use_sim_time
        }],
        namespace=namespace
    )

    create_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', robot_name,
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '3.0',
                   '-Y', '0.0',
                   '-topic', 'robot_description'],
        output='screen',
        namespace=namespace
    )

    # -- ROS-Gazebo Bridge --
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_simulation, 'launch', 'bridge.launch.py'])
        ])
    )

    # -- Controllers --
    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_drive_bringup, 'launch', 'controllers.launch.py'])
        ]),
        launch_arguments={
            'robot_controller': robot_controller,
            'start_controller_switcher': 'false',  # Usually don't need switcher in sim
        }.items()
    )

    # -- Teleop (optional) --
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_drive_bringup, 'launch', 'teleop.launch.py'])
        ]),
        condition=IfCondition(enable_teleop),
    )

    # -- RViz --
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(rviz)
    )

    return LaunchDescription(
        declared_arguments +
        [
            gazebo_launch,
            bridge_launch,
            spawn_robot,
            create_robot,
            controllers_launch,
            teleop_launch,
            rviz_node,
        ]
    )
