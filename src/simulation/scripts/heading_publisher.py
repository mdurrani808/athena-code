#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import SensorDataQoS
from sensor_msgs.msg import MagneticField
from msgs.msg import Heading


class HeadingPublisher(Node):
    def __init__(self):
        super().__init__('heading_publisher')

        self.declare_parameter('mag_topic', '/mag')
        self.declare_parameter('heading_topic', '/gps/heading')
        self.declare_parameter('heading_acc', 5.0)

        mag_topic = self.get_parameter('mag_topic').get_parameter_value().string_value
        heading_topic = self.get_parameter('heading_topic').get_parameter_value().string_value
        self.heading_acc_ = self.get_parameter('heading_acc').get_parameter_value().double_value

        self.pub_ = self.create_publisher(Heading, heading_topic, 10)

        self.sub_ = self.create_subscription(
            MagneticField,
            mag_topic,
            self.on_mag,
            SensorDataQoS(),
        )

        self.get_logger().info(
            f'Heading publisher: {mag_topic} -> {heading_topic}'
        )

    def on_mag(self, msg: MagneticField):
        bx = msg.magnetic_field.x
        by = msg.magnetic_field.y

        # Gazebo magnetometer reports field in robot body frame (x=forward, y=left).
        # Earth's horizontal field points North, so:
        #   bx = H*sin(yaw_enu),  by = H*cos(yaw_enu)
        # atan2(bx, by) = atan2(sin θ, cos θ) = θ = ENU heading directly.
        heading_enu_deg = math.degrees(math.atan2(bx, by))

        # Compass bearing: degrees clockwise from North
        compass_bearing = (90.0 - heading_enu_deg + 360.0) % 360.0

        out = Heading()
        out.header = msg.header
        out.heading = heading_enu_deg   # degrees — gps_pose_publisher applies * pi/180
        out.heading_acc = self.heading_acc_
        out.compass_bearing = compass_bearing
        self.pub_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = HeadingPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
