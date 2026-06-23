"""Detection node (SPEC §6.3, ADR-0003).

Subscribes to the D435 RGB image and depth point cloud, runs a pluggable
inference backend on demand via the ``/perception/detect`` service, reprojects
each bounding-box centroid into the organized point cloud for a 3D position, and
publishes the results as a MarkerArray for RViz.
"""

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

from plaguebot_msgs.srv import Detect
from plaguebot_msgs.msg import Detection

from plaguebot_perception.backends import make_backend


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')

        self.declare_parameter('backend', 'mock')
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('image_topic', '/d435/image_raw')
        self.declare_parameter('cloud_topic', '/d435/depth/points')

        backend_name = self.get_parameter('backend').value
        model_path = self.get_parameter('model_path').value
        conf = self.get_parameter('confidence_threshold').value
        image_topic = self.get_parameter('image_topic').value
        cloud_topic = self.get_parameter('cloud_topic').value

        self._bridge = CvBridge()
        self._latest_image = None
        self._latest_cloud = None

        self._backend = make_backend(
            backend_name, model_path, conf, logger=self.get_logger())

        self.create_subscription(Image, image_topic, self._on_image, 10)
        self.create_subscription(PointCloud2, cloud_topic, self._on_cloud, 10)

        self._markers_pub = self.create_publisher(
            MarkerArray, '/perception/detections', 10)
        self._srv = self.create_service(
            Detect, '/perception/detect', self._on_detect)

        self.get_logger().info(
            f"perception_node up (backend={backend_name}); "
            f"image={image_topic}, cloud={cloud_topic}")

    def _on_image(self, msg):
        self._latest_image = msg

    def _on_cloud(self, msg):
        self._latest_cloud = msg

    def _reproject(self, u, v):
        """Return the 3D point at organized-cloud pixel (u, v), or None."""
        cloud = self._latest_cloud
        if cloud is None or cloud.height <= 1:
            return None
        u = max(0, min(u, cloud.width - 1))
        v = max(0, min(v, cloud.height - 1))
        pts = list(point_cloud2.read_points(
            cloud, field_names=('x', 'y', 'z'), skip_nans=False, uvs=[(u, v)]))
        if not pts:
            return None
        x, y, z = float(pts[0][0]), float(pts[0][1]), float(pts[0][2])
        if any(math.isnan(c) or math.isinf(c) for c in (x, y, z)):
            return None
        return (x, y, z)

    def _on_detect(self, request, response):
        if self._latest_image is None:
            response.success = False
            response.message = 'no image received yet'
            return response

        try:
            image_bgr = self._bridge.imgmsg_to_cv2(
                self._latest_image, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f'cv_bridge failed: {exc}'
            return response

        cloud_frame = (self._latest_cloud.header.frame_id
                       if self._latest_cloud else
                       self._latest_image.header.frame_id)

        raw_dets = self._backend.infer(image_bgr)
        detections = []
        for raw in raw_dets:
            u, v = raw.center
            xyz = self._reproject(u, v)
            if xyz is None:
                self.get_logger().warn(
                    f'no depth at pixel ({u},{v}) for {raw.class_name}; skipping')
                continue
            det = Detection()
            det.class_name = raw.class_name
            det.confidence = float(raw.confidence)
            det.bbox = [int(c) for c in raw.bbox]
            ps = PointStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = cloud_frame
            ps.point.x, ps.point.y, ps.point.z = xyz
            det.position = ps
            detections.append(det)

        self._publish_markers(detections, cloud_frame)

        response.success = True
        response.message = f'{len(detections)} detection(s)'
        response.detections = detections
        self.get_logger().info(response.message)
        return response

    def _publish_markers(self, detections, frame_id):
        markers = MarkerArray()
        # Clear previous markers so stale detections don't linger in RViz.
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for i, det in enumerate(detections):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'detections'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = det.position.point
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.08
            m.color.r = 1.0
            m.color.g = 0.1
            m.color.b = 0.1
            m.color.a = 0.9
            markers.markers.append(m)
        self._markers_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
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
