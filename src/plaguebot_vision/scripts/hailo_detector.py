#!/usr/bin/env python3
"""Nodo de inferencia Hailo-8 (Fase 2).

Suscribe color -> corre best_320.hef en el chip Hailo-8 -> decode YOLOv8 en host
(DFL + NMS por clase) -> publica:
  - vision_msgs/Detection2DArray en /plaguebot/vision/detections (para brazo/chasis)
  - sensor_msgs/CompressedImage anotada en /plaguebot/vision/annotated/compressed
    (la consume el nodo de presentación vision_dashboard).

La inferencia corre 100% en el acelerador; el host solo decodifica y dibuja,
que es lo que mantiene fría la Pi (antes ultralytics quemaba la CPU al 100%).
"""
from contextlib import ExitStack

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge

import cv2
import numpy as np

from hailo_platform import (
    VDevice, HEF, ConfigureParams, HailoStreamInterface,
    InputVStreamParams, OutputVStreamParams, FormatType, InferVStreams,
)


def make_anchors(input_size, strides):
    """Centros de ancla YOLOv8 anchor-free (orden stride 8,16,32; row-major y,x)."""
    points, stride_t = [], []
    for s in strides:
        n = input_size // s
        sx = np.arange(n, dtype=np.float32) + 0.5
        sy = np.arange(n, dtype=np.float32) + 0.5
        gy, gx = np.meshgrid(sy, sx, indexing="ij")
        points.append(np.stack([gx.ravel(), gy.ravel()], axis=-1))
        stride_t.append(np.full((n * n,), s, dtype=np.float32))
    return np.concatenate(points, 0), np.concatenate(stride_t, 0)


def softmax(x, axis=-1):
    x = x - x.max(axis=axis, keepdims=True)
    e = np.exp(x)
    return e / e.sum(axis=axis, keepdims=True)


def decode(boxes_dist, anchor_points, stride_t):
    """DFL: (N,64) -> xyxy en pixeles del input 320."""
    proj = np.arange(16, dtype=np.float32)
    dist = (softmax(boxes_dist.reshape(-1, 4, 16), axis=-1) * proj).sum(-1)
    ax, ay, st = anchor_points[:, 0], anchor_points[:, 1], stride_t
    x1 = (ax - dist[:, 0]) * st
    y1 = (ay - dist[:, 1]) * st
    x2 = (ax + dist[:, 2]) * st
    y2 = (ay + dist[:, 3]) * st
    return np.stack([x1, y1, x2, y2], axis=-1)


def postprocess(scores, boxes_dist, anchor_points, stride_t,
                scale_x, scale_y, conf_thres, iou_thres):
    if scores.max() > 1.0 + 1e-3 or scores.min() < -1e-3:
        scores = 1.0 / (1.0 + np.exp(-scores))

    cls_id = scores.argmax(axis=1)
    cls_sc = scores.max(axis=1)
    keep = cls_sc >= conf_thres
    if not keep.any():
        return []

    boxes = decode(boxes_dist[keep], anchor_points[keep], stride_t[keep])
    cls_id, cls_sc = cls_id[keep], cls_sc[keep]

    dets = []
    for c in np.unique(cls_id):
        m = cls_id == c
        b = boxes[m]
        s = cls_sc[m]
        rects = [[float(x1), float(y1), float(x2 - x1), float(y2 - y1)]
                 for x1, y1, x2, y2 in b]
        idxs = cv2.dnn.NMSBoxes(rects, s.tolist(), conf_thres, iou_thres)
        for i in np.array(idxs).flatten():
            x1, y1, x2, y2 = b[i]
            dets.append((int(x1 * scale_x), int(y1 * scale_y),
                         int(x2 * scale_x), int(y2 * scale_y),
                         int(c), float(s[i])))
    return dets


class HailoDetector(Node):
    def __init__(self):
        super().__init__('hailo_detector')

        self.declare_parameter('hef_path',
            '/home/jada/plaguebot_vr_ws/src/plaguebot_vision/scripts/IA/best_320.hef')
        self.declare_parameter('input_topic', '/plaguebot/camera/color/image_raw')
        self.declare_parameter('input_size', 320)
        self.declare_parameter('conf_thres', 0.45)
        self.declare_parameter('iou_thres', 0.45)
        self.declare_parameter('jpeg_quality', 80)
        # Orden autoritativo del modelo (best.pt): 0=EnfermedadCalor, 1=TomatoNotReady, 2=TomatoReady
        self.declare_parameter('class_names',
            ['EnfermedadCalor', 'TomatoNotReady', 'TomatoReady'])

        self.hef_path = self.get_parameter('hef_path').value
        self.input_size = int(self.get_parameter('input_size').value)
        self.conf_thres = float(self.get_parameter('conf_thres').value)
        self.iou_thres = float(self.get_parameter('iou_thres').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.class_names = list(self.get_parameter('class_names').value)
        input_topic = self.get_parameter('input_topic').value

        # BGR: calor=rojo, notready=oscuro, ready=verde
        self.class_colors = {0: (0, 0, 255), 1: (139, 0, 0), 2: (0, 255, 0)}

        self.bridge = CvBridge()
        self.anchor_points, self.stride_t = make_anchors(
            self.input_size, [8, 16, 32])

        self._setup_hailo()

        self.det_pub = self.create_publisher(
            Detection2DArray, '/plaguebot/vision/detections', 10)
        self.img_pub = self.create_publisher(
            CompressedImage, '/plaguebot/vision/annotated/compressed', 10)
        self.sub = self.create_subscription(
            Image, input_topic, self.on_image, 10)

        self._infer_ms = 0.0
        self.get_logger().info(
            f'hailo_detector listo. Suscrito a {input_topic}, '
            f'modelo {self.hef_path}')

    def _setup_hailo(self):
        """Abre VDevice + pipeline VStreams y los mantiene vivos con un ExitStack."""
        self._stack = ExitStack()
        self.hef = HEF(self.hef_path)
        self.in_info = self.hef.get_input_vstream_infos()[0]

        target = self._stack.enter_context(VDevice())
        cfg = ConfigureParams.create_from_hef(
            self.hef, interface=HailoStreamInterface.PCIe)
        self.ng = target.configure(self.hef, cfg)[0]
        self.ng_params = self.ng.create_params()
        in_params = InputVStreamParams.make(self.ng, format_type=FormatType.UINT8)
        out_params = OutputVStreamParams.make(self.ng, format_type=FormatType.FLOAT32)

        self.pipeline = self._stack.enter_context(
            InferVStreams(self.ng, in_params, out_params))
        self._stack.enter_context(self.ng.activate(self.ng_params))
        self.get_logger().info('[Hailo] dispositivo configurado y activado.')

    def on_image(self, msg):
        t0 = self.get_clock().now()
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        H, W = frame.shape[:2]

        rgb = cv2.cvtColor(
            cv2.resize(frame, (self.input_size, self.input_size)),
            cv2.COLOR_BGR2RGB)
        inp = np.expand_dims(rgb, 0).astype(np.uint8)

        results = self.pipeline.infer({self.in_info.name: inp})
        scores = boxes_dist = None
        for arr in results.values():
            a = arr.reshape(-1, arr.shape[-1])
            if a.shape[-1] == len(self.class_names):
                scores = a
            elif a.shape[-1] == 64:
                boxes_dist = a

        dets = postprocess(scores, boxes_dist, self.anchor_points, self.stride_t,
                           W / self.input_size, H / self.input_size,
                           self.conf_thres, self.iou_thres)

        self._infer_ms = (self.get_clock().now() - t0).nanoseconds / 1e6
        self.publish_detections(dets, msg.header)
        self.publish_annotated(frame, dets, msg.header)

    def publish_detections(self, dets, header):
        arr = Detection2DArray()
        arr.header = header
        for x1, y1, x2, y2, c, sc in dets:
            d = Detection2D()
            d.header = header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = (self.class_names[c]
                                       if c < len(self.class_names) else str(c))
            hyp.hypothesis.score = float(sc)
            d.results.append(hyp)
            d.bbox.center.position.x = float((x1 + x2) / 2.0)
            d.bbox.center.position.y = float((y1 + y2) / 2.0)
            d.bbox.size_x = float(x2 - x1)
            d.bbox.size_y = float(y2 - y1)
            arr.detections.append(d)
        self.det_pub.publish(arr)

    def publish_annotated(self, frame, dets, header):
        for x1, y1, x2, y2, c, sc in dets:
            color = self.class_colors.get(c, (255, 255, 255))
            name = self.class_names[c] if c < len(self.class_names) else str(c)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            label = f'{name} {sc:.2f}'
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (x1, y1 - th - 5), (x1 + tw + 3, y1), color, -1)
            cv2.putText(frame, label, (x1 + 2, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame,
                    f'HAILO-8 | infer {self._infer_ms:.0f}ms | {len(dets)} det',
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2,
                    cv2.LINE_AA)

        ok, jpg = cv2.imencode('.jpg', frame,
                               [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            return
        out = CompressedImage()
        out.header = header
        out.format = 'jpeg'
        out.data = jpg.tobytes()
        self.img_pub.publish(out)

    def destroy_node(self):
        try:
            self._stack.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HailoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
