import rclpy
from rclpy.node import Node
from interfaces.msg import BB

class DummyBBNode(Node):
    def __init__(self):
        super().__init__('dummy_bb_node')

        self.bb_pub = self.create_publisher(BB, 'aruco_loc', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.width = 100  # Keep bounding box width constant
        self.center_shift = 0  # Tracks shifting
        self.direction = 1  # 1 for right, -1 for left

    def timer_callback(self):
        msg = BB()
        msg.img_width = 640
        msg.img_height = 320
        
        # Calculate bounding box positions
        msg.bb_top_left_x = 320 + self.center_shift - self.width // 2
        msg.bb_top_left_y = 160
        msg.bb_bottom_right_x = 320 + self.center_shift + self.width // 2
        msg.bb_bottom_right_y = 160

        # Update shift for oscillation
        self.center_shift += self.direction * 10  # Change 10 pixels per step

        # Reverse direction when reaching boundaries
        if abs(self.center_shift) >= 100:  # Max shift range
            self.direction *= -1

        self.bb_pub.publish(msg)



    
def main(args=None):
    rclpy.init(args=args)
    node = DummyBBNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()