from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='zed_point_cloud_bridge',
            output='screen',
            arguments=[
                '/zed/zed_node/point_cloud/cloud_registered@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            ],
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            output='screen',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ground_truth_odom_bridge',
            output='screen',
            arguments=[
                '/odom/ground_truth@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            ]
        ),
    ])
