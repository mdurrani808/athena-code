from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Inject use_sim=true directly onto the controller_manager node so it is
    # picked up when the controller loads — no value needed in the YAML config.
    set_use_sim_param = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/controller_manager',
             'single_ackermann_controller.use_sim', 'true'],
        output='screen'
    )

    single_ackermann_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['single_ackermann_controller'],
        output='screen',
        remappings=[
            ("/single_ackermann_controller/reference", "/cmd_vel"),
            ("/single_ackermann_controller/tf_odometry", "/tf"),
        ]
    )

    # After joint_state_broadcaster is up, push use_sim param …
    delayed_set_param = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[set_use_sim_param],
        )
    )

    # … then spawn the controller once the param is confirmed set.
    delayed_spawn_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=set_use_sim_param,
            on_exit=[single_ackermann_controller_spawner],
        )
    )

    ld = LaunchDescription()
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(delayed_set_param)
    ld.add_action(delayed_spawn_controller)
    return ld
