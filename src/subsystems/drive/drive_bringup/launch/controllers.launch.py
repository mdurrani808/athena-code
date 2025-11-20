from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

AVAILABLE_CONTROLLERS = [
    "ackermann_steering_controller",
    "single_ackermann_controller",
    "drive_velocity_controller",
    "drive_position_controller",
]


def controller_spawning_logic(context, *args, **kwargs):
    robot_controller = LaunchConfiguration("robot_controller").perform(context)
    use_sim = LaunchConfiguration("use_sim")
    switcher_delay = float(LaunchConfiguration("controller_switcher_delay").perform(context))

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    controllers_to_spawn = []

    controllers_to_spawn.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[robot_controller, "-c", "/controller_manager"],
        )
    )

    for controller in AVAILABLE_CONTROLLERS:
        if controller != robot_controller:
            controllers_to_spawn.append(
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[controller, "-c", "/controller_manager", "--inactive"],
                )
            )

    spawn_actions = [joint_state_broadcaster_spawner]
    previous_action = joint_state_broadcaster_spawner

    for spawner_node in controllers_to_spawn:
        spawn_actions.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=previous_action,
                    on_exit=[spawner_node],
                )
            )
        )
        previous_action = spawner_node

    controller_switcher_node = Node(
        package="drive_bringup",
        executable="controller_switcher.py",
        name="controller_switcher",
        output="screen",
        parameters=[{"use_sim_time": use_sim}],
    )

    spawn_actions.append(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=previous_action,
                on_exit=[
                    TimerAction(
                        period=switcher_delay,
                        actions=[controller_switcher_node],
                    )
                ],
            )
        )
    )

    return spawn_actions


def generate_launch_description():
    robot_controller_arg = DeclareLaunchArgument(
        "robot_controller",
        default_value="single_ackermann_controller",
        choices=["single_ackermann_controller", "ackermann_steering_controller"],
        description="The primary robot controller to start as active",
    )

    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        choices=["true", "false"],
        description="Use simulation mode (automatically sets use_sim_time)",
    )

    controller_switcher_delay_arg = DeclareLaunchArgument(
        "controller_switcher_delay",
        default_value="3.0",
        description="Delay in seconds before starting controller switcher after controllers are loaded",
    )

    robot_controller = LaunchConfiguration("robot_controller")
    use_sim = LaunchConfiguration("use_sim")
    controller_switcher_delay = LaunchConfiguration("controller_switcher_delay")

    controller_manager_setup = OpaqueFunction(function=controller_spawning_logic)

    return LaunchDescription([
        robot_controller_arg,
        use_sim_arg,
        controller_switcher_delay_arg,
        controller_manager_setup,
    ])