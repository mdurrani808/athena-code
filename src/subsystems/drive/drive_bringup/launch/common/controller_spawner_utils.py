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
Common utilities for spawning controllers with proper sequencing and delays.
"""

from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node


def create_controller_spawner(controller_name, controller_manager="/controller_manager",
                               namespace=None, inactive=False):
    """
    Create a controller spawner node.

    Args:
        controller_name: Name of the controller to spawn
        controller_manager: Name of the controller manager (default: /controller_manager)
        namespace: Optional namespace for the controller
        inactive: If True, spawn controller in inactive state

    Returns:
        Node action for spawning the controller
    """
    args = [controller_name, "-c", controller_manager]
    if inactive:
        args.append("--inactive")

    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=args,
        output="screen"
    )


def create_sequential_controller_spawners(controller_names, previous_action,
                                          controller_manager="/controller_manager",
                                          inactive=False):
    """
    Create a list of event handlers that spawn controllers sequentially.

    Args:
        controller_names: List of controller names to spawn
        previous_action: The action to wait for before starting the sequence
        controller_manager: Name of the controller manager
        inactive: If True, spawn controllers in inactive state

    Returns:
        List of RegisterEventHandler actions for sequential spawning
    """
    spawners = []
    event_handlers = []

    # Create all spawner nodes
    for controller_name in controller_names:
        spawners.append(
            create_controller_spawner(controller_name, controller_manager, inactive=inactive)
        )

    # Create event handlers for sequential execution
    for i, spawner in enumerate(spawners):
        target = previous_action if i == 0 else spawners[i - 1]
        event_handlers.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=target,
                    on_exit=[spawner],
                )
            )
        )

    return event_handlers, spawners


def create_delayed_action_after_start(target_action, delayed_action, delay_seconds):
    """
    Create an event handler that triggers an action after a delay following the start of a target.

    Args:
        target_action: The action whose start triggers the timer
        delayed_action: The action to execute after the delay
        delay_seconds: Delay in seconds

    Returns:
        RegisterEventHandler action
    """
    return RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=target_action,
            on_start=[
                TimerAction(
                    period=delay_seconds,
                    actions=[delayed_action],
                ),
            ],
        )
    )


def create_delayed_action_after_exit(target_action, delayed_action, delay_seconds=0.0):
    """
    Create an event handler that triggers an action after a delay following the exit of a target.

    Args:
        target_action: The action whose exit triggers the timer
        delayed_action: The action to execute after the delay
        delay_seconds: Delay in seconds (default: 0.0 for immediate execution)

    Returns:
        RegisterEventHandler action
    """
    if delay_seconds > 0.0:
        action = TimerAction(period=delay_seconds, actions=[delayed_action])
    else:
        action = delayed_action

    return RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=target_action,
            on_exit=[action],
        )
    )
