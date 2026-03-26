import os
import json
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from ament_index_python.packages import get_package_share_directory


def _sanitize_topic(topic: str) -> str:
    return topic.strip('/').replace('/', '_')


class MagCalibNode(Node):
    """Collect raw magnetometer samples and calibrate on shutdown."""

    def __init__(self):
        super().__init__('mag_calib_node')

        self.declare_parameter('mag_topic', 'imu/mag')
        self.declare_parameter('output_dir', '')

        mag_topic = self.get_parameter('mag_topic').value
        output_dir = self.get_parameter('output_dir').value

        self.output_dir = output_dir or get_package_share_directory('mag_calib')
        self.calib_filename = f'{_sanitize_topic(mag_topic)}_calibration.json'

        self.samples: list[list[float]] = []

        self.sub = self.create_subscription(
            MagneticField, mag_topic, self._mag_callback, 10)

        self.get_logger().info(
            f'Collecting magnetometer data on "{mag_topic}". '
            f'Rotate the sensor through all orientations, then Ctrl-C to calibrate.')

    def _mag_callback(self, msg: MagneticField):
        x = msg.magnetic_field.x * 1e6
        y = msg.magnetic_field.y * 1e6
        z = msg.magnetic_field.z * 1e6
        self.samples.append([x, y, z])

        if len(self.samples) % 500 == 0:
            self.get_logger().info(f'Collected {len(self.samples)} samples')

    def calibrate_and_save(self):
        if len(self.samples) < 100:
            self.get_logger().error(
                f'Only {len(self.samples)} samples collected. '
                f'Need at least 100 for calibration.')
            return

        data = np.array(self.samples)
        self.get_logger().info(f'Running calibration on {len(data)} samples...')

        hard_iron, soft_iron = self._fit_ellipsoid(data)
        corrected = (data - hard_iron) @ soft_iron

        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        calib_path = os.path.join(self.output_dir, self.calib_filename)
        with open(calib_path, 'w') as f:
            json.dump({
                'timestamp': timestamp,
                'num_samples': len(data),
                'hard_iron_offset_ut': hard_iron.tolist(),
                'soft_iron_matrix': soft_iron.tolist(),
            }, f, indent=2)
        self.get_logger().info(f'Calibration saved to {calib_path}')

        raw_path = os.path.join(self.output_dir, f'mag_raw_{timestamp}.csv')
        np.savetxt(raw_path, data, delimiter=',', header='x_ut,y_ut,z_ut', comments='')
        self.get_logger().info(f'Raw data saved to {raw_path}')

        self._save_plots(data, corrected, timestamp)

        self.get_logger().info(f'Hard iron offset (μT): {hard_iron}')
        self.get_logger().info(f'Soft iron matrix:\n{soft_iron}')

    def _fit_ellipsoid(self, data: np.ndarray):
        """Algebraic least-squares ellipsoid fit.

        Solves Ax²+By²+Cz²+2Dxy+2Exz+2Fyz+2Gx+2Hy+2Iz = 1 for every sample,
        then returns the ellipsoid center (hard iron offset) and a correction
        matrix (soft iron) that maps the ellipsoid to a sphere.
        """
        x, y, z = data[:, 0], data[:, 1], data[:, 2]
        D = np.column_stack([
            x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z, 2*x, 2*y, 2*z
        ])
        v, _, _, _ = np.linalg.lstsq(D, np.ones(len(data)), rcond=None)

        A, B, C, Dv, E, F, G, H, I = v
        Q = np.array([[A, Dv, E], [Dv, B, F], [E, F, C]])
        u = np.array([G, H, I])

        center = np.linalg.solve(Q, -u)
        eigvals, eigvecs = np.linalg.eigh(Q)
        radii = np.sqrt(1.0 / np.abs(eigvals))
        soft_iron = eigvecs @ np.diag(radii.mean() / radii) @ eigvecs.T
        return center, soft_iron

    def _save_plots(self, raw: np.ndarray, corrected: np.ndarray, timestamp: str):
        fig = plt.figure(figsize=(14, 6))

        ax1 = fig.add_subplot(121, projection='3d')
        ax1.scatter(raw[:, 0], raw[:, 1], raw[:, 2], s=1, alpha=0.3)
        ax1.set_xlabel('X (μT)')
        ax1.set_ylabel('Y (μT)')
        ax1.set_zlabel('Z (μT)')
        ax1.set_title('Raw Magnetometer Data')

        ax2 = fig.add_subplot(122, projection='3d')
        ax2.scatter(corrected[:, 0], corrected[:, 1], corrected[:, 2],
                    s=1, alpha=0.3, color='green')
        ax2.set_xlabel('X (μT)')
        ax2.set_ylabel('Y (μT)')
        ax2.set_zlabel('Z (μT)')
        ax2.set_title('Calibrated Magnetometer Data')

        fig.tight_layout()
        plot3d_path = os.path.join(self.output_dir, f'mag_calib_3d_{timestamp}.png')
        fig.savefig(plot3d_path, dpi=150)
        plt.close(fig)
        self.get_logger().info(f'3D plot saved to {plot3d_path}')

        fig, axes = plt.subplots(2, 3, figsize=(15, 9))
        pairs = [(0, 1, 'XY'), (0, 2, 'XZ'), (1, 2, 'YZ')]
        labels = ['X (μT)', 'Y (μT)', 'Z (μT)']

        for col, (i, j, name) in enumerate(pairs):
            axes[0, col].scatter(raw[:, i], raw[:, j], s=1, alpha=0.3)
            axes[0, col].set_xlabel(labels[i])
            axes[0, col].set_ylabel(labels[j])
            axes[0, col].set_title(f'Raw {name}')
            axes[0, col].set_aspect('equal')

            axes[1, col].scatter(corrected[:, i], corrected[:, j],
                                 s=1, alpha=0.3, color='green')
            axes[1, col].set_xlabel(labels[i])
            axes[1, col].set_ylabel(labels[j])
            axes[1, col].set_title(f'Calibrated {name}')
            axes[1, col].set_aspect('equal')

        fig.tight_layout()
        plot2d_path = os.path.join(self.output_dir, f'mag_calib_2d_{timestamp}.png')
        fig.savefig(plot2d_path, dpi=150)
        plt.close(fig)
        self.get_logger().info(f'2D plot saved to {plot2d_path}')


def main(args=None):
    rclpy.init(args=args)
    node = MagCalibNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.calibrate_and_save()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
