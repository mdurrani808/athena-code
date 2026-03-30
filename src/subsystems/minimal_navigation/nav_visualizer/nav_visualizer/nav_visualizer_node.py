import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from msgs.msg import NavStatus
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import threading
import math

def quat_to_yaw(q):
    # q is geometry_msgs/Quaternion
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class NavVisualizer(Node):
    def __init__(self):
        super().__init__('nav_visualizer')
        
        # Data storage
        self.robot_pose = None
        self.global_path = None
        self.mppi_path = None
        self.goal_pose = None
        self.nav_status = None
        
        # Subscriptions
        self.create_subscription(PoseStamped, '/robot_pose', self.pose_cb, 10)
        self.create_subscription(Path, '/global_path', self.global_path_cb, 10)
        self.create_subscription(Path, '/optimal_path', self.mppi_path_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(NavStatus, '/nav_status', self.status_cb, 10)
        
        self.get_logger().info("Nav Visualizer Node Started")
        
        # Lock for thread safety between ROS and Matplotlib
        self.lock = threading.Lock()

    def pose_cb(self, msg):
        with self.lock:
            self.robot_pose = msg

    def global_path_cb(self, msg):
        with self.lock:
            self.global_path = msg

    def mppi_path_cb(self, msg):
        with self.lock:
            self.mppi_path = msg

    def goal_cb(self, msg):
        with self.lock:
            self.goal_pose = msg

    def status_cb(self, msg):
        with self.lock:
            self.nav_status = msg

class VisualizerPlot:
    def __init__(self, node):
        self.node = node
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        
        # Plot elements
        self.robot_marker, = self.ax.plot([], [], 'ro', label='Robot', markersize=10)
        self.heading_line, = self.ax.plot([], [], 'r-', linewidth=2)
        self.global_path_line, = self.ax.plot([], [], 'b--', label='Global Path')
        self.mppi_path_line, = self.ax.plot([], [], 'g-', label='MPPI Path', linewidth=2)
        self.goal_marker, = self.ax.plot([], [], 'kx', label='Goal', markersize=12, markeredgewidth=2)
        
        self.text_status = self.ax.text(0.02, 0.95, '', transform=self.ax.transAxes, 
                                        bbox=dict(facecolor='white', alpha=0.7))
        
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.ax.legend(loc='upper right')
        self.ax.set_xlabel('East (m)')
        self.ax.set_ylabel('North (m)')
        self.ax.set_title('Athena 2D Navigation Visualizer')

    def init_plot(self):
        return self.robot_marker, self.heading_line, self.global_path_line, self.mppi_path_line, self.goal_marker, self.text_status

    def update(self, frame):
        with self.node.lock:
            # Update Robot
            if self.node.robot_pose:
                x = self.node.robot_pose.pose.position.x
                y = self.node.robot_pose.pose.position.y
                yaw = quat_to_yaw(self.node.robot_pose.pose.orientation)
                self.robot_marker.set_data([x], [y])
                
                # Heading arrow (0.5m long)
                arrow_len = 1.0
                self.heading_line.set_data([x, x + arrow_len * math.cos(yaw)], 
                                           [y, y + arrow_len * math.sin(yaw)])
            
            # Update Goal
            if self.node.goal_pose:
                self.goal_marker.set_data([self.node.goal_pose.pose.position.x], 
                                          [self.node.goal_pose.pose.position.y])
            
            # Update Global Path
            if self.node.global_path:
                px = [p.pose.position.x for p in self.node.global_path.poses]
                py = [p.pose.position.y for p in self.node.global_path.poses]
                self.global_path_line.set_data(px, py)
                
            # Update MPPI Path
            if self.node.mppi_path:
                mx = [p.pose.position.x for p in self.node.mppi_path.poses]
                my = [p.pose.position.y for p in self.node.mppi_path.poses]
                self.mppi_path_line.set_data(mx, my)

            # Update Status Text
            if self.node.nav_status:
                status_text = (f"State: {self.node.nav_status.state}\n"
                               f"Target: {self.node.nav_status.active_target_id}\n"
                               f"Dist to Goal: {self.node.nav_status.distance_to_goal_m:.2f}m\n"
                               f"X-Track Error: {self.node.nav_status.cross_track_error_m:.2f}m\n"
                               f"Speed: {self.node.nav_status.robot_speed_mps:.2f} m/s")
                self.text_status.set_text(status_text)

            # Auto-center view on robot
            if self.node.robot_pose:
                cx = self.node.robot_pose.pose.position.x
                cy = self.node.robot_pose.pose.position.y
                self.ax.set_xlim(cx - 15, cx + 15)
                self.ax.set_ylim(cy - 15, cy + 15)

        return self.robot_marker, self.heading_line, self.global_path_line, self.mppi_path_line, self.goal_marker, self.text_status

def main(args=None):
    rclpy.init(args=args)
    node = NavVisualizer()
    
    # Run ROS spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    # Setup Matplotlib
    viz = VisualizerPlot(node)
    ani = FuncAnimation(viz.fig, viz.update, frames=None, init_func=viz.init_plot, 
                        blit=True, interval=100, cache_frame_data=False)
    
    plt.show()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
