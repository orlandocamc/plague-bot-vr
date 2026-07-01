#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import cv2
import time

class RealSenseCameraNode(Node):
    def __init__(self):
        super().__init__('camera_realsense')

        # Publishers
        self.color_pub = self.create_publisher(Image, '/plaguebot/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/plaguebot/camera/depth/image_rect_raw', 10)
        self.color_comp_pub = self.create_publisher(CompressedImage, '/plaguebot/camera/color/image_raw/compressed', 10)
        self.depth_comp_pub = self.create_publisher(CompressedImage, '/plaguebot/camera/depth/image_rect_raw/compressed', 10)

        self.bridge = CvBridge()

        # Configure RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Enable streams for Intel RealSense D435i
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Start pipeline with retries to handle USB enumeration delays
        max_retries = 10
        retry_delay = 3.0
        for attempt in range(1, max_retries + 1):
            try:
                self.pipeline.start(self.config)
                self.get_logger().info("RealSense pipeline started successfully.")
                break
            except Exception as e:
                if attempt < max_retries:
                    self.get_logger().warn(
                        f"RealSense not ready (attempt {attempt}/{max_retries}): {e}. "
                        f"Retrying in {retry_delay:.0f}s..."
                    )
                    time.sleep(retry_delay)
                else:
                    self.get_logger().error(f"Failed to start RealSense pipeline after {max_retries} attempts: {e}")
                    raise e
            
        # Align depth to color
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        
        # Timer to read and publish frames
        timer_period = 1.0 / 30.0  # 30 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        try:
            # Wait for a coherent pair of frames: depth and color
            frames = self.pipeline.wait_for_frames()
            
            # Align the depth frame to color frame
            aligned_frames = self.align.process(frames)
            
            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not aligned_depth_frame or not color_frame:
                return
                
            # Convert images to numpy arrays
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            # Get timestamp for ROS message
            now = self.get_clock().now().to_msg()
            
            # Convert and publish color image
            color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            color_msg.header.stamp = now
            color_msg.header.frame_id = "camera_color_optical_frame"
            self.color_pub.publish(color_msg)
            
            # Compress and publish color image
            success, encoded_color = cv2.imencode('.jpg', color_image)
            if success:
                comp_color_msg = CompressedImage()
                comp_color_msg.header.stamp = now
                comp_color_msg.header.frame_id = "camera_color_optical_frame"
                comp_color_msg.format = "jpeg"
                comp_color_msg.data = encoded_color.tobytes()
                self.color_comp_pub.publish(comp_color_msg)
            
            # Convert and publish depth image
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
            depth_msg.header.stamp = now
            depth_msg.header.frame_id = "camera_depth_optical_frame"
            self.depth_pub.publish(depth_msg)
            
            # Compress and publish depth image
            success, encoded_depth = cv2.imencode('.png', depth_image)
            if success:
                comp_depth_msg = CompressedImage()
                comp_depth_msg.header.stamp = now
                comp_depth_msg.header.frame_id = "camera_depth_optical_frame"
                comp_depth_msg.format = "16UC1; compressedDepth"
                comp_depth_msg.data = encoded_depth.tobytes()
                self.depth_comp_pub.publish(comp_depth_msg)
            
        except Exception as e:
            self.get_logger().warn(f"Error grabbing frame: {e}")

    def __del__(self):
        try:
            self.pipeline.stop()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()