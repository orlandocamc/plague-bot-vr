#!/usr/bin/env python3
"""Nodo de la segunda cámara (Razer Kiyo) para el demo de doble cámara.

Abre la Kiyo por una ruta estable /dev/v4l/by-id/ (parámetro `device`, no por
/dev/videoN que cambia de número al conectar varias USB) y publica el frame ya
comprimido en JPEG:
  - sensor_msgs/CompressedImage en /plaguebot/vision/kiyo/compressed
    (la consume vision_dashboard como segundo MJPEG, en crudo y sin detección).

La compresión JPEG se hace aquí —en el nodo que ya tiene el frame en mano— para
que el dashboard siga siendo un relay tonto de tópicos ya comprimidos.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

import cv2


class KiyoCameraNode(Node):
    def __init__(self):
        super().__init__('camera_kiyo')

        # Ruta estable por dispositivo físico. Ajustar el default con la ruta
        # real una vez conectada la Kiyo: ls -l /dev/v4l/by-id/
        self.declare_parameter(
            'device',
            '/dev/v4l/by-id/'
            'usb-Alpha_Imaging_Tech._Corp._Razer_Kiyo-video-index0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('jpeg_quality', 80)

        self.device = self.get_parameter('device').value
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        fps = int(self.get_parameter('fps').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        self.pub = self.create_publisher(
            CompressedImage, '/plaguebot/vision/kiyo/compressed', 10)

        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(
                f'No se pudo abrir la Razer Kiyo en {self.device}')
            return

        # La Razer Kiyo necesita MJPEG; sin esto cap.read() se bloquea en V4L2/Pi.
        self.cap.set(cv2.CAP_PROP_FOURCC,
                     cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        self.timer = self.create_timer(1.0 / max(1, fps), self.timer_callback)
        self.get_logger().info(
            f'camera_kiyo publicando JPEG en /plaguebot/vision/kiyo/compressed '
            f'(device {self.device})')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('No se pudo leer frame de la Razer Kiyo')
            return
        ok, jpg = cv2.imencode('.jpg', frame,
                               [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_kiyo_optical_frame'
        msg.format = 'jpeg'
        msg.data = jpg.tobytes()
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            if hasattr(self, 'cap') and self.cap.isOpened():
                self.cap.release()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KiyoCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
