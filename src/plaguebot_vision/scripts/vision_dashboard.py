#!/usr/bin/env python3
"""Nodo de presentación (Fase 2).

Suscribe dos corrientes ya en JPEG y las sirve como MJPEG over HTTP
(multipart/x-mixed-replace) en una sola página de dashboard:
  - /stream  : RealSense con detección IA (/plaguebot/vision/annotated/compressed)
  - /stream2 : Razer Kiyo en crudo (/plaguebot/vision/kiyo/compressed)
Reemplaza al monolito webrtc_server.py: HTTP puro funciona sobre LAN / Tailscale
sin el dolor de ICE/STUN de WebRTC.

Cada corriente tiene su propio buffer, así que si una cámara se cae su panel
queda en blanco pero el otro sigue vivo (no se tumba la página).

Visible en LAN en http://<ip>:8080/.
"""
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

_latest_main = None
_latest_kiyo = None
_lock = threading.Lock()
_stream_fps = 15


class DashboardNode(Node):
    def __init__(self):
        super().__init__('vision_dashboard')
        self.declare_parameter('annotated_topic',
                               '/plaguebot/vision/annotated/compressed')
        self.declare_parameter('kiyo_topic',
                               '/plaguebot/vision/kiyo/compressed')
        self.declare_parameter('port', 8080)
        self.declare_parameter('stream_fps', 15)

        main_topic = self.get_parameter('annotated_topic').value
        kiyo_topic = self.get_parameter('kiyo_topic').value
        self.port = int(self.get_parameter('port').value)
        global _stream_fps
        _stream_fps = int(self.get_parameter('stream_fps').value)

        self.sub_main = self.create_subscription(
            CompressedImage, main_topic, self.on_main, 10)
        self.sub_kiyo = self.create_subscription(
            CompressedImage, kiyo_topic, self.on_kiyo, 10)

        self.httpd = ThreadingHTTPServer(('0.0.0.0', self.port), Handler)
        self.http_thread = threading.Thread(
            target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(
            f'vision_dashboard sirviendo MJPEG en http://0.0.0.0:{self.port}/ '
            f'(RealSense {main_topic} | Kiyo {kiyo_topic})')

    def on_main(self, msg):
        global _latest_main
        with _lock:
            _latest_main = bytes(msg.data)

    def on_kiyo(self, msg):
        global _latest_kiyo
        with _lock:
            _latest_kiyo = bytes(msg.data)

    def destroy_node(self):
        try:
            self.httpd.shutdown()
        except Exception:
            pass
        super().destroy_node()


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            html = (
                "<html><head><title>Plague-Bot Vision</title>"
                "<meta charset='utf-8'></head>"
                "<body style='margin:0;background:#111;color:#eee;"
                "font-family:sans-serif'>"
                "<div style='padding:8px 12px;font-size:18px'>"
                "PLAGUE-BOT VISION</div>"
                "<div style='display:flex;gap:8px;padding:0 8px 8px;"
                "align-items:flex-start'>"
                "<div style='flex:3'>"
                "<div style='font-size:13px;padding:2px'>RealSense + detección IA</div>"
                "<img src='/stream' style='width:100%;height:auto;display:block'></div>"
                "<div style='flex:1'>"
                "<div style='font-size:13px;padding:2px'>Kiyo (crudo)</div>"
                "<img src='/stream2' style='width:100%;height:auto;display:block'></div>"
                "</div></body></html>").encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', str(len(html)))
            self.end_headers()
            self.wfile.write(html)
        elif self.path == '/stream':
            self._mjpeg(lambda: _latest_main)
        elif self.path == '/stream2':
            self._mjpeg(lambda: _latest_kiyo)
        else:
            self.send_error(404)

    def _mjpeg(self, get_frame):
        import time
        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        try:
            while True:
                with _lock:
                    jpg = get_frame()
                if jpg is not None:
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(
                        f'Content-Length: {len(jpg)}\r\n\r\n'.encode())
                    self.wfile.write(jpg)
                    self.wfile.write(b'\r\n')
                time.sleep(1.0 / max(1, _stream_fps))
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, *args):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
