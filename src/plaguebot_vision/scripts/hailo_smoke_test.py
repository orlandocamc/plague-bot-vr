#!/usr/bin/env python3
"""Fase 1 smoke test (standalone, sin ROS):
webcam -> Hailo-8 (best_320.hef) -> decode YOLOv8 (DFL+NMS) en host -> MJPEG por HTTP.

Objetivo: probar que la inferencia corre en el chip (CPU ~0, Pi fria) y verla
desde cualquier dispositivo en http://<ip-de-la-pi>:8080
"""
import time
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

from hailo_platform import (
    VDevice, HEF, ConfigureParams, HailoStreamInterface,
    InputVStreamParams, OutputVStreamParams, FormatType, InferVStreams,
)

# ── Config ────────────────────────────────────────────────────────────────
HEF_PATH    = "/home/jada/plaguebot_vr_ws/src/plaguebot_vision/scripts/IA/best_320.hef"
CAM_DEVICE  = "/dev/video0"          # webcam Steren COM-126
INPUT_SIZE  = 320
# Orden autoritativo del modelo (best.pt): 0=EnfermedadCalor, 1=TomatoNotReady, 2=TomatoReady
CLASS_NAMES = ["EnfermedadCalor", "TomatoNotReady", "TomatoReady"]
CLASS_COLORS = {0: (0, 0, 255), 1: (139, 0, 0), 2: (0, 255, 0)}  # BGR: calor=rojo, notready=oscuro, ready=verde
CONF_THRES  = 0.45
IOU_THRES   = 0.45
TARGET_FPS  = 15
STRIDES     = [8, 16, 32]
PORT        = 8080

# ── Estado compartido con el servidor MJPEG ───────────────────────────────
_latest_jpeg = None
_lock = threading.Lock()
_stats = {"infer_ms": 0.0, "fps": 0.0, "ndet": 0}


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
    """DFL: (2100,64) -> xyxy en pixeles del input 320."""
    proj = np.arange(16, dtype=np.float32)
    dist = (softmax(boxes_dist.reshape(-1, 4, 16), axis=-1) * proj).sum(-1)  # (2100,4)
    ax, ay, st = anchor_points[:, 0], anchor_points[:, 1], stride_t
    x1 = (ax - dist[:, 0]) * st
    y1 = (ay - dist[:, 1]) * st
    x2 = (ax + dist[:, 2]) * st
    y2 = (ay + dist[:, 3]) * st
    return np.stack([x1, y1, x2, y2], axis=-1)


def postprocess(scores, boxes_dist, anchor_points, stride_t, scale_x, scale_y):
    # scores ya deberian venir post-sigmoid (capa 'activation1'); aplica sigmoid solo si hace falta
    if scores.max() > 1.0 + 1e-3 or scores.min() < -1e-3:
        scores = 1.0 / (1.0 + np.exp(-scores))

    cls_id = scores.argmax(axis=1)
    cls_sc = scores.max(axis=1)
    keep = cls_sc >= CONF_THRES
    if not keep.any():
        return []

    boxes = decode(boxes_dist[keep], anchor_points[keep], stride_t[keep])
    cls_id, cls_sc = cls_id[keep], cls_sc[keep]

    dets = []
    for c in np.unique(cls_id):
        m = cls_id == c
        b = boxes[m]
        s = cls_sc[m]
        rects = [[float(x1), float(y1), float(x2 - x1), float(y2 - y1)] for x1, y1, x2, y2 in b]
        idxs = cv2.dnn.NMSBoxes(rects, s.tolist(), CONF_THRES, IOU_THRES)
        for i in np.array(idxs).flatten():
            x1, y1, x2, y2 = b[i]
            dets.append((int(x1 * scale_x), int(y1 * scale_y),
                         int(x2 * scale_x), int(y2 * scale_y),
                         int(c), float(s[i])))
    return dets


def draw(frame, dets):
    for x1, y1, x2, y2, c, sc in dets:
        color = CLASS_COLORS.get(c, (255, 255, 255))
        name = CLASS_NAMES[c] if c < len(CLASS_NAMES) else str(c)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        label = f"{name} {sc:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (x1, y1 - th - 5), (x1 + tw + 3, y1), color, -1)
        cv2.putText(frame, label, (x1 + 2, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, f"HAILO-8 | infer {_stats['infer_ms']:.0f}ms | {_stats['fps']:.1f} fps | {_stats['ndet']} det",
                (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA)


def inference_loop():
    global _latest_jpeg
    anchor_points, stride_t = make_anchors(INPUT_SIZE, STRIDES)

    cap = cv2.VideoCapture(CAM_DEVICE, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"[ERROR] no pude abrir la camara {CAM_DEVICE}")
        return
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    hef = HEF(HEF_PATH)
    in_info = hef.get_input_vstream_infos()[0]
    print("[Hailo] configurando dispositivo...")

    with VDevice() as target:
        cfg = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
        ng = target.configure(hef, cfg)[0]
        ng_params = ng.create_params()
        in_params = InputVStreamParams.make(ng, format_type=FormatType.UINT8)
        out_params = OutputVStreamParams.make(ng, format_type=FormatType.FLOAT32)  # dequant auto

        print("[Hailo] listo. Sirviendo MJPEG en http://0.0.0.0:%d" % PORT)
        first = True
        with InferVStreams(ng, in_params, out_params) as pipeline, ng.activate(ng_params):
            period = 1.0 / TARGET_FPS
            while True:
                t0 = time.time()
                ret, frame = cap.read()
                if not ret:
                    time.sleep(0.05)
                    continue
                H, W = frame.shape[:2]
                rgb = cv2.cvtColor(cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE)), cv2.COLOR_BGR2RGB)
                inp = np.expand_dims(rgb, 0).astype(np.uint8)

                results = pipeline.infer({in_info.name: inp})
                scores = boxes_dist = None
                for arr in results.values():
                    a = arr.reshape(-1, arr.shape[-1])
                    if a.shape[-1] == len(CLASS_NAMES):
                        scores = a
                    elif a.shape[-1] == 64:
                        boxes_dist = a

                if first:
                    print(f"[debug] scores rango [{scores.min():.3f},{scores.max():.3f}] "
                          f"boxes rango [{boxes_dist.min():.3f},{boxes_dist.max():.3f}]")
                    first = False

                dets = postprocess(scores, boxes_dist, anchor_points, stride_t,
                                   W / INPUT_SIZE, H / INPUT_SIZE)
                _stats["infer_ms"] = (time.time() - t0) * 1000
                _stats["ndet"] = len(dets)
                draw(frame, dets)

                ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    with _lock:
                        _latest_jpeg = jpg.tobytes()

                dt = time.time() - t0
                _stats["fps"] = 1.0 / dt if dt > 0 else 0.0
                if dt < period:
                    time.sleep(period - dt)


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            html = (b"<html><head><title>Hailo smoke test</title></head>"
                    b"<body style='margin:0;background:#111;text-align:center'>"
                    b"<img src='/stream' style='max-width:100%;height:auto'></body></html>")
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(html)))
            self.end_headers()
            self.wfile.write(html)
        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                while True:
                    with _lock:
                        jpg = _latest_jpeg
                    if jpg is not None:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                    time.sleep(1.0 / TARGET_FPS)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404)

    def log_message(self, *args):
        pass


def main():
    t = threading.Thread(target=inference_loop, daemon=True)
    t.start()
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()


if __name__ == "__main__":
    main()
