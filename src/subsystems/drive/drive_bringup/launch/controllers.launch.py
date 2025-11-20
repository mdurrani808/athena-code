from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def controller_spawning_logic(context, *args, **kwargs):
    robot_controller = LaunchConfiguration("robot_controller").perform(context)
    use_sim = LaunchConfiguration("use_sim")
    switcher_delay = float(LaunchConfiguration("controller_switcher_delay").perform(context))
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    control_node_startup_delay = LaunchConfiguration("control_node_startup_delay")

    robot_controllers_path = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", "athena_drive_controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            robot_controllers_path,
            {"use_sim_time": use_sim}
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        condition=UnlessCondition(use_sim),
    )

    # Topic relays for simulation mode
    # In sim mode, Gazebo's ros2_control doesn't support remappings like hardware mode
    # These relays handle the topic routing instead
    cmd_vel_relay = Node(
        package="topic_tools",
        executable="relay",
        name="cmd_vel_to_ackermann_relay",
        arguments=[
            "/cmd_vel",
            "/ackermann_steering_controller/reference",
            "--ros-args",
            "--log-level",
            "error"
        ],
        parameters=[{"use_sim_time": use_sim}],
        condition=IfCondition(use_sim),
    )

    joy_relay = Node(
        package="topic_tools",
        executable="relay",
        name="joy_to_single_ackermann_relay",
        arguments=[
            "/joy",
            "/single_ackermann_controller/reference",
            "--ros-args",
            "--log-level",
            "error"
        ],
        parameters=[{"use_sim_time": use_sim}],
        condition=IfCondition(use_sim),
    )

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

    for controller in [
        "ackermann_steering_controller",
        "single_ackermann_controller",
        "drive_velocity_controller",
        "drive_position_controller",
    ]:
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

    delayed_controllers_real = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=control_node_startup_delay,
                    actions=spawn_actions,
                ),
            ],
        ),
        condition=UnlessCondition(use_sim),
    )

    delayed_controllers_sim = TimerAction(
        period=control_node_startup_delay,
        actions=spawn_actions,
        condition=IfCondition(use_sim),
    )

    return [control_node, delayed_controllers_real, delayed_controllers_sim, cmd_vel_relay, joy_relay]


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

    runtime_config_package_arg = DeclareLaunchArgument(
        "runtime_config_package",
        default_value="drive_bringup",
        description="Package with the controller configuration in 'config' folder",
    )

    control_node_startup_delay_arg = DeclareLaunchArgument(
        "control_node_startup_delay",
        default_value="5.0",
        description="Delay in seconds before starting controllers",
    )

    controller_manager_setup = OpaqueFunction(function=controller_spawning_logic)

    return LaunchDescription([
        robot_controller_arg,
        use_sim_arg,
        controller_switcher_delay_arg,
        runtime_config_package_arg,
        control_node_startup_delay_arg,
        controller_manager_setup,
    ])