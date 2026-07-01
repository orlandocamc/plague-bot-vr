#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.subscription = self.create_subscription(
            Image,
            '/plaguebot/camera/color/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.frame_count = 0
        self.start_time = time.time()
        self.get_logger().info('Camera node started, waiting for images...')
        # Placeholder for YOLOv8s initialization
        # from ultralytics import YOLO
        # self.model = YOLO('yolov8s.pt')

    def image_callback(self, msg):
        self.frame_count += 1
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # Calculate FPS over 1 second window
        if elapsed_time >= 1.0:
            fps = self.frame_count / elapsed_time
            width = msg.width
            height = msg.height
            self.get_logger().info(f'Received frame: {width}x{height} | Estimated FPS: {fps:.2f}')
            
            # Reset counters
            self.start_time = current_time
            self.frame_count = 0
            
        # Placeholder for YOLOv8s inference
        # cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # results = self.model(cv_image)
        # self.get_logger().info('YOLOv8s inference placeholder')

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
