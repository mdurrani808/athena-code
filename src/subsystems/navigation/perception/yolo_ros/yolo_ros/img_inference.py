import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
import cv2
import threading
from ultralytics import YOLO
import numpy as np
import time

class BoundingBox:
    def __init__(self, img_width, img_height, x1, y1, x2, y2):
        self.img_width = img_width
        self.img_height = img_height
        self.bb_top_left_x = x1
        self.bb_top_left_y = y1
        self.bb_bottom_right_x = x2
        self.bb_bottom_right_y = y2

    def __repr__(self):
        return (f"BoundingBox(img_width={self.img_width}, img_height={self.img_height}, "
                f"x1={self.bb_top_left_x}, y1={self.bb_top_left_y}, "
                f"x2={self.bb_bottom_right_x}, y2={self.bb_bottom_right_y})")

    def to_msg_format(self):
        return (f"img_width: {self.img_width}, img_height: {self.img_height}, "
                f"bb_top_left_x: {self.bb_top_left_x}, bb_top_left_y: {self.bb_top_left_y}, "
                f"bb_bottom_right_x: {self.bb_bottom_right_x}, bb_bottom_right_y: {self.bb_bottom_right_y}\n")

class InferenceNode(Node):
    def __init__(self):
        super().__init__('yolo_inference_node')
        self.subscription = self.create_subscription(
            RosImage,
            '/zed/zed_node/left_gray/image_rect_gray',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.frame_queue = []
        self.frame_lock = threading.Lock()
        self.model = YOLO("best.pt")
        self.detection_file = "detections.txt"
        self.last_inference_time = time.time()
        self.fps = 0
        
        open(self.detection_file, "w").close()
        
        self.inference_thread = threading.Thread(target=self.inference_loop, daemon=True) # use ros executers and executer groups
        self.inference_thread.start()

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.frame_lock:
            self.frame_queue.append((frame, msg.width, msg.height))
            if len(self.frame_queue) > 1:
                self.frame_queue.pop(0)

    def inference_loop(self):
        while rclpy.ok():
            if not self.frame_queue:
                time.sleep(0.001)
                continue
                
            with self.frame_lock:
                if not self.frame_queue:
                    continue
                frame, width, height = self.frame_queue.pop(0)

            results = self.model(frame)
            
            current_time = time.time()
            self.fps = 1.0 / (current_time - self.last_inference_time)
            self.last_inference_time = current_time
            
            self.process_results(frame, results, width, height)

    def process_results(self, frame, results, img_width, img_height):
        cv2.putText(frame, f"FPS: {self.fps:.1f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        with open(self.detection_file, "a") as file:
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    label = f"{self.model.names[cls]}: {conf:.2f}"

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, label, (x1, y1 - 10), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    bb = BoundingBox(img_width, img_height, x1, y1, x2, y2)
                    file.write(bb.to_msg_format())

        cv2.imshow("YOLO Output", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    yolo_node = InferenceNode()
    
    try:
        rclpy.spin(yolo_node)
    except KeyboardInterrupt:
        pass
    finally:
        yolo_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()