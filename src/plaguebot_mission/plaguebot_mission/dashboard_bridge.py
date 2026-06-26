"""Dashboard bridge (Phase 6A — robot -> dashboard).

Forwards each mission's detections to Mario's robot-cultivos backend
(`POST /robot/datos`, see that repo's docs/INTEGRACION_ROBOT.md). It subscribes
to /mission/detections (published by mission_node after DETECT), maps our model
classes to the backend's schema, transforms each detection point into the map
frame for a world (x, y), and POSTs with the x-api-key header.

This is deliberately a separate node so the dashboard/HTTP concern stays out of
the mission state machine; swapping mock/torch/hailo or the backend URL needs no
change to mission_node. The same node runs unchanged on the real robot.
"""

import json
import threading
from datetime import datetime, timezone

import requests
import rclpy
from rclpy.node import Node

import tf2_ros
from rclpy.time import Time
from rclpy.duration import Duration as RclDuration
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transforms)

from plaguebot_msgs.msg import DetectionArray

# Default class -> backend "tipo". The backend's enum is plaga|enfermedad|
# tomate_maduro; TomatoNotReady has no exact fit (kept as tomate_maduro for now —
# tune via the class_map param with Mario's team).
DEFAULT_CLASS_MAP = {
    'EnfermedadCalor': 'enfermedad',
    'TomatoReady': 'tomate_maduro',
    'TomatoNotReady': 'tomate_maduro',
    'pest': 'plaga',
}


class DashboardBridge(Node):

    def __init__(self):
        super().__init__('dashboard_bridge')

        self.declare_parameter('enabled', True)
        self.declare_parameter('backend_url', 'http://localhost:8000')
        self.declare_parameter('api_key', 'changeme')
        self.declare_parameter('origen', 'plaguebot-sim')
        self.declare_parameter('zona', 'invernadero')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('default_tipo', 'plaga')
        self.declare_parameter('http_timeout', 5.0)
        self.declare_parameter('class_map_json', json.dumps(DEFAULT_CLASS_MAP))

        self._enabled = self.get_parameter('enabled').value
        self._url = self.get_parameter('backend_url').value.rstrip('/')
        self._api_key = self.get_parameter('api_key').value
        self._origen = self.get_parameter('origen').value
        self._zona = self.get_parameter('zona').value
        self._map_frame = self.get_parameter('map_frame').value
        self._default_tipo = self.get_parameter('default_tipo').value
        self._timeout = float(self.get_parameter('http_timeout').value)
        try:
            self._class_map = json.loads(
                self.get_parameter('class_map_json').value)
        except (ValueError, TypeError):
            self.get_logger().warn('class_map_json invalid; using defaults')
            self._class_map = dict(DEFAULT_CLASS_MAP)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(
            DetectionArray, '/mission/detections', self._on_detections, 10)

        self.get_logger().info(
            f"dashboard_bridge up (enabled={self._enabled}); "
            f"POST {self._url}/robot/datos as origen='{self._origen}'")

    def _on_detections(self, msg):
        if not self._enabled:
            return
        dets = []
        for d in msg.detections:
            tipo = self._class_map.get(d.class_name, self._default_tipo)
            entry = {
                'tipo': tipo,
                'etiqueta': d.class_name,
                'confianza': round(float(d.confidence), 3),
            }
            pos = self._to_map_xy(d.position)
            if pos is not None:
                entry['posicion'] = pos
            dets.append(entry)
        if not dets:
            return
        payload = {
            'origen': self._origen,
            'timestamp': datetime.now(timezone.utc).isoformat(),
            'zona': self._zona,
            'detecciones': dets,
        }
        # POST off the executor thread so a slow/hanging backend never stalls ROS.
        threading.Thread(target=self._post, args=(payload,), daemon=True).start()

    def _to_map_xy(self, point_stamped):
        """Transform a detection point into the map frame -> {x, y}, or None."""
        try:
            src = PointStamped()
            src.header.frame_id = point_stamped.header.frame_id
            src.header.stamp = Time().to_msg()
            src.point = point_stamped.point
            out = self._tf_buffer.transform(
                src, self._map_frame, timeout=RclDuration(seconds=2.0))
            return {'x': round(float(out.point.x), 3),
                    'y': round(float(out.point.y), 3)}
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'tf to {self._map_frame} failed: {exc}')
            return None

    def _post(self, payload):
        try:
            r = requests.post(
                f'{self._url}/robot/datos',
                json=payload,
                headers={'x-api-key': self._api_key},
                timeout=self._timeout,
            )
            if r.status_code == 200:
                self.get_logger().info(
                    f"posted {len(payload['detecciones'])} detection(s) -> "
                    f"dashboard: {r.json()}")
            else:
                self.get_logger().warn(
                    f'dashboard POST {r.status_code}: {r.text[:200]}')
        except requests.RequestException as exc:
            self.get_logger().warn(f'dashboard POST failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = DashboardBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
