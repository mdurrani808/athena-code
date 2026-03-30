#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
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
        
        # Always cache the first heading to start at 0 degrees
        self.initial_heading_enu = None

        self.sub_ = self.create_subscription(
            MagneticField,
            mag_topic,
            self.on_mag,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        self.get_logger().info(
            f'Heading publisher: {mag_topic} -> {heading_topic} (Force Initial Offset: Enabled)'
        )

    def on_mag(self, msg: MagneticField):
        bx = msg.magnetic_field.x
        by = msg.magnetic_field.y

        # In Gazebo's magnetometer the field is reported in the robot body frame
        # with the simulated Earth field pointing North (+Y world).  For a robot
        # with x=forward, y=left this gives:
        #   bx = H * sin(yaw_enu)   (North projected onto forward axis)
        #   by = H * cos(yaw_enu)   (North projected onto left axis)
        # So atan2(bx, by) = atan2(sin θ, cos θ) = θ, the correct CCW-from-East
        # ENU heading.  Using atan2(by, bx) would give a CW compass bearing
        # instead, inverting the sign of every turn.
        heading_enu_raw = math.degrees(math.atan2(bx, by))

        if self.initial_heading_enu is None:
            self.initial_heading_enu = heading_enu_raw
            self.get_logger().info(f'Initial raw heading cached: {self.initial_heading_enu:.2f} deg. Starting at 0.00 deg.')
        
        # Apply offset so we start at 0 (East)
        # ENU heading: degrees CCW from East
        heading_enu = (heading_enu_raw - self.initial_heading_enu + 360.0) % 360.0

        # Compass bearing: degrees clockwise from North
        # North is 90 deg ENU.
        compass_bearing = (90.0 - heading_enu + 360.0) % 360.0

        out = Heading()
        out.header = msg.header
        out.heading = heading_enu
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
