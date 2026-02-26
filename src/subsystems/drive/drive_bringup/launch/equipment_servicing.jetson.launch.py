from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # -- Declare arguments --
    declared_arguments = []
    
    # Drive arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "drive_runtime_config_package",
            default_value="drive_bringup",
            description='Package with the drive controller\'s configuration in "config" folder.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "drive_controllers_file",
            default_value="athena_drive_controllers.yaml",
            description="YAML file with the drive controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "drive_description_file",
            default_value="athena_drive.urdf.xacro",
            description="URDF/XACRO description file for drive.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_file",
            default_value="athena_drive.rviz",
            description="Rviz config file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "drive_robot_controller",
            default_value="rear_ackermann_controller",
            choices=["front_ackermann_controller", "ackermann_steering_controller", "rear_ackermann_controller"],
            description="Drive robot controller to start.",
        )
    )
    
    # Arm arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_runtime_config_package",
            default_value="arm_bringup",
            description='Package with the arm controller\'s configuration in "config" folder.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_controllers_file",
            default_value="athena_arm_controllers.yaml",
            description="YAML file with the arm controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_description_file",
            default_value="athena_arm.urdf.xacro",
            description="URDF/XACRO description file for arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_robot_controller",
            default_value="manual_arm_joint_by_joint_controller",
            choices=["manual_arm_joint_by_joint_controller"],
            description="Arm robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names.",
        )
    )

    # -- Initialize Arguments --
    use_sim = LaunchConfiguration("use_sim")
    drive_runtime_config_package = LaunchConfiguration("drive_runtime_config_package")
    drive_controllers_file = LaunchConfiguration("drive_controllers_file")
    drive_description_file = LaunchConfiguration("drive_description_file")
    rviz_file = LaunchConfiguration("rviz_file")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    drive_robot_controller = LaunchConfiguration("drive_robot_controller")
    arm_runtime_config_package = LaunchConfiguration("arm_runtime_config_package")
    arm_controllers_file = LaunchConfiguration("arm_controllers_file")
    arm_description_file = LaunchConfiguration("arm_description_file")
    arm_robot_controller = LaunchConfiguration("arm_robot_controller")
    prefix = LaunchConfiguration("prefix")

    # -- Building Path Files --
    drive_robot_description_path = PathJoinSubstitution(
        [FindPackageShare("description"), "urdf", drive_description_file]
    )
    drive_robot_controllers = PathJoinSubstitution(
        [FindPackageShare(drive_runtime_config_package), "config", drive_controllers_file]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("description"), "rviz", rviz_file]
    )
    arm_robot_description_path = PathJoinSubstitution(
        [FindPackageShare("description"), "urdf", arm_description_file]
    )
    arm_robot_controllers = PathJoinSubstitution(
        [FindPackageShare(arm_runtime_config_package), "config", arm_controllers_file]
    )

    # -- Robot Descriptions --
    drive_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            drive_robot_description_path,
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
        ]
    )
    drive_robot_description = {"robot_description": drive_robot_description_content}

    arm_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            arm_robot_description_path,
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
        ]
    )
    arm_robot_description = {"robot_description": arm_robot_description_content}

    # -- Drive Nodes --
    drive_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="drive_controller_manager",
        output="both",
        parameters=[drive_robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/front_ackermann_controller/tf_odometry", "/tf"),
            ("/ackermann_steering_controller/reference", "/cmd_vel"),
        ],
    )

    drive_robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="drive_robot_state_publisher",
        output="both",
        parameters=[drive_robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_sim),
    )

    drive_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/drive_controller_manager"],
    )

    drive_motor_status_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["motor_status_broadcaster", "-c", "/drive_controller_manager"],
    )

    drive_robot_controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[drive_robot_controller, "-c", "/drive_controller_manager"],
        )
    ]

    drive_gpio_controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "-c", "/drive_controller_manager"],
        )
        for controller in ["led_gpio_controller", "killswitch_gpio_controller"]
    ]

    drive_inactive_robot_controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "-c", "/drive_controller_manager", "--inactive"],
        )
        for controller in ["ackermann_steering_controller", "drive_velocity_controller", "drive_position_controller"]
    ]

    # -- Arm Nodes --
    arm_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="arm_controller_manager",
        output="both",
        parameters=[arm_robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    arm_robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="arm_robot_state_publisher",
        output="both",
        parameters=[arm_robot_description],
    )

    arm_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/arm_controller_manager"],
    )

    arm_robot_controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[arm_robot_controller, "-c", "/arm_controller_manager"],
        )
    ]

    arm_inactive_robot_controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "-c", "/arm_controller_manager", "--inactive"],
        )
        for controller in ["manual_arm_cylindrical_controller", "joint_trajectory_controller", "arm_velocity_controller"]
    ]

    # -- Event Handlers: Drive --
    delay_drive_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=drive_control_node,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[drive_joint_state_broadcaster_spawner],
                ),
            ],
        )
    )

    delay_drive_motor_status_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=drive_joint_state_broadcaster_spawner,
            on_exit=[drive_motor_status_broadcaster_spawner],
        )
    )

    delay_drive_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=drive_joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_drive_robot_controllers = [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=drive_robot_controller_spawners[i - 1] if i > 0 else drive_joint_state_broadcaster_spawner,
                on_exit=[controller],
            )
        )
        for i, controller in enumerate(drive_robot_controller_spawners)
    ]

    delay_drive_inactive_controllers = [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=drive_inactive_robot_controller_spawners[i - 1] if i > 0 else drive_robot_controller_spawners[-1],
                on_exit=[controller],
            )
        )
        for i, controller in enumerate(drive_inactive_robot_controller_spawners)
    ]

    delay_drive_gpio_controllers = [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=drive_gpio_controller_spawners[i - 1] if i > 0 else drive_joint_state_broadcaster_spawner,
                on_exit=[controller],
            )
        )
        for i, controller in enumerate(drive_gpio_controller_spawners)
    ]

    drive_controller_switcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=drive_inactive_robot_controller_spawners[-1],
            on_exit=[TimerAction(
                period=3.0,
                actions=[Node(
                    package="drive_bringup",
                    executable="controller_switcher.py",
                    name="drive_controller_switcher",
                    output="screen"
                )]
            )],
        )
    )

    # -- Event Handlers: Arm --
    delay_arm_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=arm_control_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[arm_joint_state_broadcaster_spawner],
                ),
            ],
        )
    )

    delay_arm_robot_controllers = [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=arm_robot_controller_spawners[i - 1] if i > 0 else arm_joint_state_broadcaster_spawner,
                on_exit=[controller],
            )
        )
        for i, controller in enumerate(arm_robot_controller_spawners)
    ]

    delay_arm_inactive_controllers = [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=arm_inactive_robot_controller_spawners[i - 1] if i > 0 else arm_robot_controller_spawners[-1],
                on_exit=[controller],
            )
        )
        for i, controller in enumerate(arm_inactive_robot_controller_spawners)
    ]

    arm_controller_switcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_inactive_robot_controller_spawners[-1],
            on_exit=[TimerAction(
                period=3.0,
                actions=[Node(
                    package="arm_bringup",
                    executable="controller_switcher.py",
                    name="arm_controller_switcher",
                    output="screen"
                )]
            )],
        )
    )

    return LaunchDescription(
        declared_arguments + 
        [
            drive_control_node,
            drive_robot_state_pub_node,
            arm_control_node,
            arm_robot_state_pub_node,
            delay_drive_joint_state_broadcaster,
            delay_drive_motor_status_broadcaster,
            delay_drive_rviz,
            drive_controller_switcher,
            delay_arm_joint_state_broadcaster,
            arm_controller_switcher,
        ]
        + delay_drive_robot_controllers
        + delay_drive_inactive_controllers
        + delay_drive_gpio_controllers
        + delay_arm_robot_controllers
        + delay_arm_inactive_controllers
    )
