#!/usr/bin/env python3
"""
Publishes an initial pose to /initialpose topic for nav2 localization.
This allows setting the robot's initial position in the map frame.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.spatial.transform import Rotation
import math


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        # Declare parameters
        self.declare_parameter('initial_pose_x', 0.0)
        self.declare_parameter('initial_pose_y', 0.0)
        self.declare_parameter('initial_pose_z', 0.0)
        self.declare_parameter('initial_pose_yaw', 0.0)
        self.declare_parameter('initial_pose_roll', 0.0)
        self.declare_parameter('initial_pose_pitch', 0.0)
        self.declare_parameter('frame_id', 'map')

        # Get parameters
        x = self.get_parameter('initial_pose_x').value
        y = self.get_parameter('initial_pose_y').value
        z = self.get_parameter('initial_pose_z').value
        yaw = self.get_parameter('initial_pose_yaw').value
        roll = self.get_parameter('initial_pose_roll').value
        pitch = self.get_parameter('initial_pose_pitch').value
        frame_id = self.get_parameter('frame_id').value

        # Create publisher
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # Create and publish initial pose message
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        # Set position
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z

        # Convert Euler angles to quaternion using scipy
        # scipy uses 'xyz' convention for extrinsic rotations
        rotation = Rotation.from_euler('xyz', [roll, pitch, yaw])
        q = rotation.as_quat()  # Returns [x, y, z, w]
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # Set covariance (diagonal matrix with small values for certainty)
        # Covariance is a 6x6 matrix flattened to 36 elements
        # [x, y, z, rot_x, rot_y, rot_z]
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.25   # x variance
        msg.pose.covariance[7] = 0.25   # y variance
        msg.pose.covariance[14] = 0.0   # z variance (not used in 2D)
        msg.pose.covariance[21] = 0.0   # rot_x variance (not used in 2D)
        msg.pose.covariance[28] = 0.0   # rot_y variance (not used in 2D)
        msg.pose.covariance[35] = 0.06854  # rot_z (yaw) variance

        # Create a timer to publish the pose multiple times
        # This ensures the localization system receives it
        self.msg = msg
        self.publish_count = 0
        self.max_publishes = 5
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info(f'Publishing initial pose: x={x}, y={y}, z={z}, '
                               f'roll={roll}, pitch={pitch}, yaw={yaw}')

    def timer_callback(self):
        if self.publish_count < self.max_publishes:
            self.msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(self.msg)
            self.publish_count += 1
            self.get_logger().info(f'Initial pose published ({self.publish_count}/{self.max_publishes})')
        else:
            self.get_logger().info('Finished publishing initial pose. Shutting down.')
            self.timer.cancel()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
