#!/usr/bin/env python3

import math
import random

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from msgs.msg import Heading


def quaternion_to_yaw(q):
    """Extract ENU yaw (radians) from a quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class HeadingPublisher(Node):
    def __init__(self):
        super().__init__('heading_publisher')

        self.declare_parameter('odom_topic', '/odom/ground_truth')
        self.declare_parameter('heading_topic', '/heading')
        self.declare_parameter('noise_sigma', 0.0)
        self.declare_parameter('publish_rate', 10.0)

        odom_topic = self.get_parameter('odom_topic').value
        heading_topic = self.get_parameter('heading_topic').value
        self.noise_sigma = self.get_parameter('noise_sigma').value
        publish_rate = self.get_parameter('publish_rate').value

        self.latest_odom = None

        self.sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)
        self.pub = self.create_publisher(Heading, heading_topic, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_heading)

        self.get_logger().info(
            f'Heading publisher: {odom_topic} -> {heading_topic} '
            f'(noise_sigma={self.noise_sigma:.4f} rad)')

    def odom_callback(self, msg):
        self.latest_odom = msg

    def publish_heading(self):
        if self.latest_odom is None:
            return

        q = self.latest_odom.pose.pose.orientation
        yaw = quaternion_to_yaw(q)

        noise = random.gauss(0.0, self.noise_sigma) if self.noise_sigma > 0.0 else 0.0

        heading_msg = Heading()
        heading_msg.header = self.latest_odom.header
        heading_msg.heading = yaw + noise
        heading_msg.heading_acc = self.noise_sigma
        heading_msg.compass_bearing = 0.0

        self.pub.publish(heading_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HeadingPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
