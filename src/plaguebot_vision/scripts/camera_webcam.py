#!/usr/bin/env python3
"""Publicador de webcam genérica (Steren COM-126 en /dev/video0).

Publica color a /plaguebot/camera/color/image_raw, el MISMO tópico que el nodo
camera_realsense, para poder intercambiar webcam <-> RealSense sin tocar el
nodo de inferencia. Usado en Fase 2 mientras la RealSense D435 no está conectada.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class WebcamNode(Node):
    def __init__(self):
        super().__init__('camera_webcam')

        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('topic', '/plaguebot/camera/color/image_raw')

        device = self.get_parameter('device').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        topic = self.get_parameter('topic').value

        self.pub = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f'No pude abrir la cámara {device}')
            raise RuntimeError(f'cannot open {device}')
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self.timer = self.create_timer(1.0 / float(fps), self.tick)
        self.get_logger().info(f'Webcam {device} -> {topic} @ {fps}fps ({width}x{height})')

    def tick(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            self.cap.release()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
