from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def create_controller_spawner(controller_name):
    """Create a controller spawner node."""
    return Node(
        package='controller_manager',
        executable='spawner',
        arguments=[controller_name],
        output='screen'
    )


def generate_launch_description():
    joint_state_broadcaster_spawner = create_controller_spawner('joint_state_broadcaster')
    ackermann_controller_spawner = create_controller_spawner('ackermann_steering_controller')

    # Start ackermann controller after joint_state_broadcaster finishes
    delayed_ackermann_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[ackermann_controller_spawner],
        )
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        delayed_ackermann_controller_spawner,
    ])