import math
import datetime
import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import MagneticField, NavSatFix
from msgs.msg import Heading

try:
    from pygeomag import GeoMag
    _HAS_GEOMAG = True
except ImportError:
    _HAS_GEOMAG = False


class MagHeadingNode(Node):
    def __init__(self):
        super().__init__('mag_heading_node')

        self.declare_parameter('mag_topic', '/zed/zed_node/imu/mag')
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('hard_iron', [0.0, 0.0, 0.0])
        self.declare_parameter('soft_iron', [1.0, 0.0, 0.0,
                                             0.0, 1.0, 0.0,
                                             0.0, 0.0, 1.0])

        mag_topic = self.get_parameter('mag_topic').get_parameter_value().string_value
        gps_topic = self.get_parameter('gps_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        hi = self.get_parameter('hard_iron').get_parameter_value().double_array_value
        si = self.get_parameter('soft_iron').get_parameter_value().double_array_value
        self.hard_iron = np.array(hi, dtype=float)
        self.soft_iron = np.array(si, dtype=float).reshape(3, 3)

        # Declination in radians, None until first valid GPS fix
        self.declination_rad = None

        if not _HAS_GEOMAG:
            self.get_logger().warn(
                'pygeomag not installed — magnetic declination will not be applied. '
                'Install with: pip install pygeomag'
            )

        self.pub = self.create_publisher(Heading, 'heading', 10)
        self.create_subscription(MagneticField, mag_topic, self._mag_cb, 10)
        self.create_subscription(NavSatFix, gps_topic, self._gps_cb, 10)
        self.get_logger().info(f'Subscribed to {mag_topic} and {gps_topic}')

    def _gps_cb(self, msg: NavSatFix):
        # Only compute declination once — it changes negligibly over a competition day
        if self.declination_rad is not None:
            return
        if not _HAS_GEOMAG:
            return
        # Require a valid fix (STATUS_FIX or better)
        if msg.status.status < 0:
            return
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        today = datetime.date.today()
        decimal_year = today.year + (today.timetuple().tm_yday - 1) / 365.25

        try:
            result = GeoMag().calculate(
                glat=msg.latitude,
                glon=msg.longitude,
                alt=max(msg.altitude, 0.0) / 1000.0,  # metres → km
                time=decimal_year,
            )
            self.declination_rad = math.radians(result.d)
            self.get_logger().info(
                f'Magnetic declination set to {result.d:.3f}° '
                f'at ({msg.latitude:.5f}, {msg.longitude:.5f})'
            )
        except Exception as e:
            self.get_logger().error(f'Declination calculation failed: {e}')

    def _mag_cb(self, msg: MagneticField):
        raw = np.array([msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])
        corrected = self.soft_iron @ (raw - self.hard_iron)

        # compass_bearing: magnetic north, 0=North CW degrees [0, 360)
        mag_heading_rad = math.atan2(corrected[1], corrected[0])
        compass_bearing_deg = (90.0 - math.degrees(mag_heading_rad)) % 360.0

        # heading: true north if declination known, magnetic otherwise
        # ROS convention: 0=East, CCW, radians
        if self.declination_rad is not None:
            heading_rad = mag_heading_rad + self.declination_rad
        else:
            heading_rad = mag_heading_rad

        # Propagate covariance through atan2: sigma_theta ≈ sigma_B / |B_xy|
        cov = msg.magnetic_field_covariance
        b_xy_mag = math.hypot(corrected[0], corrected[1])
        if b_xy_mag > 1e-9 and cov[0] > 0.0:
            heading_acc = math.sqrt((cov[0] + cov[4]) / 2.0) / b_xy_mag
        else:
            heading_acc = 0.0

        out = Heading()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.heading = heading_rad
        out.heading_acc = heading_acc
        out.compass_bearing = compass_bearing_deg
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
        rclpy.shutdown()
