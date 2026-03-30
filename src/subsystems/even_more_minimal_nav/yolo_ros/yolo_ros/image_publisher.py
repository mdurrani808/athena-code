import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
import cv2
import pyzed.sl as sl

class ZedPublisher(Node):
    def __init__(self):
        super().__init__('zed_publisher')
        self.publisher = self.create_publisher(RosImage, 'cam_in', 10)
        self.bridge = CvBridge()
        
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 30
        init_params.depth_mode = sl.DEPTH_MODE.NONE
        
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"Failed to open ZED camera: {err}")
            return

        self.image = sl.Mat()
        self.timer = self.create_timer(1 / 30.0, self.publish_frame)

    def publish_frame(self):
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            frame = self.image.get_data()
            if frame.ndim == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            elif frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(ros_image)
            cv2.imshow("ZED Camera feed", frame)
            if (cv2.waitKey(1) and 0xFF == ord('q')):
                self.get_logger().info("Exiting...")
                cv2.destroyAllWindows()
        else:
            self.get_logger().warn("Failed to grab ZED frame")

        def __del__(self):
            self.zed.close()

def main():
    rclpy.init()
    zed_publisher = ZedPublisher()
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(zed_publisher)
    executor.spin()
    
    zed_publisher.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()