from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_controller = LaunchConfiguration("robot_controller")
    use_sim = LaunchConfiguration("use_sim")
    controller_switcher_delay = LaunchConfiguration("controller_switcher_delay")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_names = [robot_controller]
    robot_controller_spawners = []
    for controller in robot_controller_names:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
                remappings=[
                    ("/single_ackermann_controller/reference", "/joy"),
                    ("/ackermann_steering_controller/reference", "/cmd_vel"),
                ],
            )
        ]

    inactive_robot_controller_names = [
        "ackermann_steering_controller",
        "drive_velocity_controller",
        "drive_position_controller"
    ]
    inactive_robot_controller_spawners = []
    for controller in inactive_robot_controller_names:
        inactive_robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager", "--inactive"],
            )
        ]

    controller_switcher_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=inactive_robot_controller_spawners[-1],
            on_exit=[
                TimerAction(
                    period=float(controller_switcher_delay.perform(context)),
                    actions=[
                        Node(
                            package="drive_bringup",
                            executable="controller_switcher.py",
                            name="controller_switcher",
                            output="screen",
                            parameters=[{"use_sim_time": use_sim}],
                        )
                    ],
                )
            ],
        )
    )

    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for i, controller in enumerate(robot_controller_spawners):
        delay_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=(
                        robot_controller_spawners[i - 1]
                        if i > 0
                        else joint_state_broadcaster_spawner
                    ),
                    on_exit=[controller],
                )
            )
        ]

    delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for i, controller in enumerate(inactive_robot_controller_spawners):
        delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=(
                        inactive_robot_controller_spawners[i - 1]
                        if i > 0
                        else robot_controller_spawners[-1]
                    ),
                    on_exit=[controller],
                )
            )
        ]

    return [
        joint_state_broadcaster_spawner,
        controller_switcher_node,
    ] + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner \
      + delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="single_ackermann_controller",
            choices=["single_ackermann_controller", "ackermann_steering_controller"],
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            choices=["true", "false"],
            description="Use simulation mode (automatically sets use_sim_time).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_switcher_delay",
            default_value="3.0",
            description="Delay in seconds before starting controller switcher after controllers are loaded.",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
