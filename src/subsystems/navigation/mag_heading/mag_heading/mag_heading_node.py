import os
import json

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from msgs.msg import Heading
from ament_index_python.packages import get_package_share_directory


def _sanitize_topic(topic: str) -> str:
    return topic.strip('/').replace('/', '_')


class MagHeadingNode(Node):

    def __init__(self):
        super().__init__('mag_heading_node')

        self.declare_parameter('mag_topic', 'imu/mag')
        self.declare_parameter('heading_topic', 'heading')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('declination_deg', 0.0)

        mag_topic = self.get_parameter('mag_topic').value
        heading_topic = self.get_parameter('heading_topic').value
        calib_file = self.get_parameter('calibration_file').value
        self.declination_rad = np.deg2rad(
            self.get_parameter('declination_deg').value)

        if not calib_file:
            share_dir = get_package_share_directory('mag_calib')
            calib_file = os.path.join(
                share_dir, f'{_sanitize_topic(mag_topic)}_calibration.json')

        self.hard_iron = np.zeros(3)
        self.soft_iron = np.eye(3)

        if os.path.isfile(calib_file):
            with open(calib_file, 'r') as f:
                calib = json.load(f)
            self.hard_iron = np.array(calib['hard_iron_offset_ut'])
            self.soft_iron = np.array(calib['soft_iron_matrix'])
            self.get_logger().info(f'Loaded calibration from {calib_file}')
        else:
            self.get_logger().warn(
                f'No calibration file at {calib_file}. '
                f'Publishing uncalibrated headings.')

        self.pub = self.create_publisher(Heading, heading_topic, 10)
        self.sub = self.create_subscription(
            MagneticField, mag_topic, self._mag_callback, 10)

        self.get_logger().info(
            f'Publishing heading on "{heading_topic}" '
            f'from magnetometer on "{mag_topic}"')

    def _mag_callback(self, msg: MagneticField):
        raw = np.array([
            msg.magnetic_field.x * 1e6,
            msg.magnetic_field.y * 1e6,
            msg.magnetic_field.z * 1e6,
        ])
        corrected = (raw - self.hard_iron) @ self.soft_iron

        # Assumes sensor X = North, Y = East.
        mag_bearing_rad = np.arctan2(corrected[1], corrected[0])
        true_bearing_rad = mag_bearing_rad + self.declination_rad
        true_bearing_deg = np.degrees(true_bearing_rad) % 360.0

        # ROS convention: 0 = East, CCW positive, [-π, π].
        heading_ros = np.arctan2(
            np.sin(np.pi / 2.0 - true_bearing_rad),
            np.cos(np.pi / 2.0 - true_bearing_rad))

        heading_acc = 0.0
        cov = msg.magnetic_field_covariance
        if cov[0] > 0.0:
            field_mag_xy = np.hypot(corrected[0], corrected[1])
            if field_mag_xy > 0.0:
                avg_var_ut2 = (cov[0] + cov[4]) / 2.0 * 1e12
                heading_acc = np.sqrt(avg_var_ut2) / field_mag_xy

        out = Heading()
        out.header = msg.header
        out.heading = heading_ros
        out.heading_acc = heading_acc
        out.compass_bearing = true_bearing_deg
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MagHeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
