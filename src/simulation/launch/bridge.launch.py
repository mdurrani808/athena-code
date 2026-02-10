from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # All bridge nodes use simulation time since they only run in simulation
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='imu_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            ],
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gps_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '/gps/fix@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
            ],
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='depth_camera_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
                '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            ],
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ground_truth_odom_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '/odom/ground_truth@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            ]
        ),
    ])
