#!/usr/bin/env python3

import asyncio
import json
import os
import cv2
import numpy as np
import psutil
import subprocess
import threading
import math
import queue
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
import av
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

# ── Snapshots globales ───────────────────────────────────────────────────────
_latest_realsense_jpeg = None
_latest_kiyo_jpeg      = None
_latest_lidar_jpeg     = None


# ── Track de video WebRTC ────────────────────────────────────────────────────
class ROSVideoStreamTrack(VideoStreamTrack):
    def __init__(self, node, track_name="Unknown"):
        super().__init__()
        self.node       = node
        self.queue      = asyncio.Queue(maxsize=2)
        self.track_name = track_name

    async def recv(self):
        try:
            cv_image = await asyncio.wait_for(self.queue.get(), timeout=2.0)
        except asyncio.TimeoutError:
            print(f"[{self.track_name}] Timeout — frame negro")
            pts, time_base = await self.next_timestamp()
            blank = av.VideoFrame(width=640, height=480, format='yuv420p')
            blank.pts       = pts
            blank.time_base = time_base
            return blank
        try:
            new_frame = av.VideoFrame.from_ndarray(cv_image, format="bgr24")
            pts, time_base  = await self.next_timestamp()
            new_frame.pts       = pts
            new_frame.time_base = time_base
            return new_frame
        except Exception as e:
            print(f"[{self.track_name}] ERROR: {e}")
            raise


# ── Thread de inferencia YOLO ────────────────────────────────────────────────
class YOLOInferenceThread(threading.Thread):
    def __init__(self, model_path):
        super().__init__(daemon=True, name="YOLOThread")

        self.model = YOLO(model_path)
        self.nombres_clases = self.model.names
        self.colores = {
            'tomatoready':     (0, 255, 0),
            'tomatonotready':  (139, 0, 0),
            'enfermedadcalor': (0, 0, 255),
            'default':         (255, 255, 255)
        }

        self.input_queue       = queue.Queue(maxsize=1)
        self._latest_annotated = None
        self._lock             = threading.Lock()
        self._frame_counter    = 0

        # Warm up — evita que el primer frame tarde 3x más
        print("[YOLO] Calentando modelo...")
        dummy = np.zeros((240, 320, 3), dtype=np.uint8)
        self.model.predict(source=dummy, conf=0.5, verbose=False)
        print("[YOLO] Modelo listo")

    def run(self):
        print("[YOLO] Thread corriendo — esperando frames...")
        while True:
            try:
                cv_image  = self.input_queue.get(timeout=1.0)
                annotated = self._run_inference(cv_image)
                with self._lock:
                    self._latest_annotated = annotated
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[YOLO] Error en thread: {e}")

    def _run_inference(self, cv_image):
        try:
            t0 = time.time()

            # Inferencia en resolución reducida — 2x más rápido
            img_small  = cv2.resize(cv_image, (320, 240))
            resultados = self.model.predict(
                source=img_small,
                conf=0.45,
                verbose=False,
                half=True       # FP16 — más rápido en CPU moderno
            )[0]

            elapsed = (time.time() - t0) * 1000
            print(f"[YOLO] Inference: {elapsed:.0f}ms")

            # Factores de escala: 320x240 → 640x480
            sx, sy = 2.0, 2.0

            for box in resultados.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                x1, x2 = int(x1 * sx), int(x2 * sx)
                y1, y2 = int(y1 * sy), int(y2 * sy)

                confianza    = float(box.conf[0])
                id_clase     = int(box.cls[0])
                nombre_clase = self.nombres_clases[id_clase]
                clase_norm   = str(nombre_clase).lower().strip()
                color        = self.colores.get(clase_norm,
                                                self.colores['default'])

                cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)
                etiqueta = f"{nombre_clase} {confianza:.2f}"
                t_size   = cv2.getTextSize(
                    etiqueta, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                cv2.rectangle(cv_image,
                              (x1, y1),
                              (x1 + t_size[0] + 3, y1 - t_size[1] - 5),
                              color, -1)
                cv2.putText(cv_image, etiqueta, (x1 + 3, y1 - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                           (255, 255, 255), 1, cv2.LINE_AA)

        except Exception as e:
            print(f"[YOLO] Inference error: {e}")

        return cv_image

    def submit_frame(self, cv_image):
        # Procesa 1 de cada 4 frames — reduce carga a la mitad
        self._frame_counter += 1
        if self._frame_counter % 4 != 0:
            return
        try:
            while not self.input_queue.empty():
                try:
                    self.input_queue.get_nowait()
                except queue.Empty:
                    break
            self.input_queue.put_nowait(cv_image.copy())
        except queue.Full:
            pass

    def get_latest_annotated(self, fallback):
        with self._lock:
            if self._latest_annotated is not None:
                return self._latest_annotated.copy()
        return fallback


# ── Nodo ROS 2 ───────────────────────────────────────────────────────────────
class WebRTCServerNode(Node):
    def __init__(self):
        super().__init__('webrtc_server')

        model_path = ("/home/jada/plaguebot_vr_ws/src/"
                      "plaguebot_vision/scripts/IA/best.onnx")

        print("[Server] Cargando modelo YOLO...")
        self.yolo = YOLOInferenceThread(model_path)
        self.yolo.start()
        print("[Server] YOLO thread iniciado")

        self.realsense_sub = self.create_subscription(
            Image, '/plaguebot/camera/color/image_raw',
            self.realsense_callback, 10)
        self.kiyo_sub = self.create_subscription(
            Image, '/plaguebot/camera/kiyo/image_raw',
            self.kiyo_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan',
            self.lidar_callback, 10)

        self.realsense_tracks = set()
        self.kiyo_tracks      = set()
        self.pcs              = set()
        self.bridge           = CvBridge()

    def create_tracks(self):
        rs_track   = ROSVideoStreamTrack(self, "RealSense")
        kiyo_track = ROSVideoStreamTrack(self, "Kiyo")
        self.realsense_tracks.add(rs_track)
        self.kiyo_tracks.add(kiyo_track)
        return rs_track, kiyo_track

    def _feed_track_set(self, tracks, cv_image):
        for track in list(tracks):
            while not track.queue.empty():
                try:
                    track.queue.get_nowait()
                except asyncio.QueueEmpty:
                    break
            try:
                track.queue.put_nowait(cv_image)
            except asyncio.QueueFull:
                pass

    def realsense_callback(self, msg):
        global _latest_realsense_jpeg
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = cv2.resize(cv_image, (640, 480))

            # Envía a YOLO en background — no bloquea
            self.yolo.submit_frame(cv_image)

            # Usa el último frame anotado disponible
            # Si YOLO no terminó, usa el frame crudo (sin lag)
            frame_to_send = self.yolo.get_latest_annotated(cv_image)

            # Envía a WebRTC y snapshot
            self._feed_track_set(self.realsense_tracks, frame_to_send)
            _, jpeg = cv2.imencode('.jpg', frame_to_send,
                                   [cv2.IMWRITE_JPEG_QUALITY, 75])
            _latest_realsense_jpeg = jpeg.tobytes()

        except Exception as e:
            print(f"[RealSense] Error: {e}")

    def kiyo_callback(self, msg):
        global _latest_kiyo_jpeg
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = cv2.resize(cv_image, (640, 480))
            self._feed_track_set(self.kiyo_tracks, cv_image)
            _, jpeg = cv2.imencode('.jpg', cv_image,
                                   [cv2.IMWRITE_JPEG_QUALITY, 80])
            _latest_kiyo_jpeg = jpeg.tobytes()
        except Exception as e:
            print(f"[Kiyo] Error: {e}")

    def lidar_callback(self, msg):
        global _latest_lidar_jpeg
        try:
            img_size  = 400
            center    = img_size // 2
            max_range = 4.0
            img = np.zeros((img_size, img_size, 3), dtype=np.uint8)

            for r_m in [1.0, 2.0, 3.0, 4.0]:
                r_px = int(r_m / max_range * center)
                cv2.circle(img, (center, center), r_px, (40, 40, 40), 1)

            cv2.line(img, (center, 0),
                     (center, img_size), (40, 40, 40), 1)
            cv2.line(img, (0, center),
                     (img_size, center), (40, 40, 40), 1)

            for r_m in [1, 2, 3]:
                r_px = int(r_m / max_range * center)
                cv2.putText(img, f"{r_m}m",
                           (center + r_px + 2, center - 4),
                           cv2.FONT_HERSHEY_SIMPLEX,
                           0.3, (60, 60, 60), 1)

            angle = msg.angle_min
            for distance in msg.ranges:
                if msg.range_min < distance < max_range:
                    x  = distance * math.cos(angle)
                    y  = distance * math.sin(angle)
                    px = int(center + x / max_range * center)
                    py = int(center - y / max_range * center)
                    if 0 <= px < img_size and 0 <= py < img_size:
                        norm = distance / max_range
                        if norm < 0.25:
                            color = (0, 0, 220)
                        elif norm < 0.5:
                            color = (0, 200, 220)
                        else:
                            color = (0, 220, 80)
                        cv2.circle(img, (px, py), 2, color, -1)
                angle += msg.angle_increment

            cv2.circle(img, (center, center), 6, (220, 80, 80), -1)
            cv2.circle(img, (center, center), 8, (255, 255, 255), 1)
            cv2.putText(img, "LIDAR · TOP VIEW",
                       (8, 16), cv2.FONT_HERSHEY_SIMPLEX,
                       0.4, (150, 150, 150), 1)

            _, jpeg = cv2.imencode('.jpg', img,
                                   [cv2.IMWRITE_JPEG_QUALITY, 85])
            _latest_lidar_jpeg = jpeg.tobytes()

        except Exception as e:
            print(f"[LIDAR] Error: {e}")


# ── Endpoints HTTP ────────────────────────────────────────────────────────────

async def offer(request):
    params     = await request.json()
    offer_desc = RTCSessionDescription(
        sdp=params["sdp"], type=params["type"])
    pc   = RTCPeerConnection()
    node = request.app["node"]
    node.pcs.add(pc)
    rs_track, kiyo_track = node.create_tracks()

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        if pc.connectionState in ("failed", "closed"):
            await pc.close()
            node.pcs.discard(pc)
            node.realsense_tracks.discard(rs_track)
            node.kiyo_tracks.discard(kiyo_track)

    await pc.setRemoteDescription(offer_desc)
    video_transceivers = [t for t in pc.getTransceivers()
                          if t.kind == 'video']
    for transceiver, track in zip(video_transceivers,
                                  [rs_track, kiyo_track]):
        transceiver.sender.replaceTrack(track)
        transceiver.direction = 'sendrecv'
    if len(video_transceivers) < 2:
        node.kiyo_tracks.discard(kiyo_track)

    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    return web.Response(
        content_type="application/json",
        text=json.dumps({
            "sdp":  pc.localDescription.sdp,
            "type": pc.localDescription.type
        }),
    )

async def snapshot_realsense(request):
    if _latest_realsense_jpeg is None:
        return web.Response(status=503, text="No frame yet")
    return web.Response(body=_latest_realsense_jpeg,
                        content_type='image/jpeg',
                        headers={'Access-Control-Allow-Origin': '*'})

async def snapshot_kiyo(request):
    if _latest_kiyo_jpeg is None:
        return web.Response(status=503, text="No frame yet")
    return web.Response(body=_latest_kiyo_jpeg,
                        content_type='image/jpeg',
                        headers={'Access-Control-Allow-Origin': '*'})

async def snapshot_lidar(request):
    if _latest_lidar_jpeg is None:
        return web.Response(status=503, text="No LIDAR data yet")
    return web.Response(body=_latest_lidar_jpeg,
                        content_type='image/jpeg',
                        headers={'Access-Control-Allow-Origin': '*'})

async def metrics(request):
    cpu  = psutil.cpu_percent(interval=0.1)
    ram  = psutil.virtual_memory()
    temp = 0.0
    try:
        temps = psutil.sensors_temperatures()
        if 'cpu_thermal' in temps:
            temp = temps['cpu_thermal'][0].current
        elif 'coretemp' in temps:
            temp = temps['coretemp'][0].current
    except Exception:
        try:
            result = subprocess.run(
                ['vcgencmd', 'measure_temp'],
                capture_output=True, text=True)
            temp = float(result.stdout.strip()
                        .replace("temp=", "").replace("'C", ""))
        except Exception:
            temp = 0.0

    return web.Response(
        content_type="application/json",
        text=json.dumps({
            "cpu_percent":  round(cpu, 1),
            "ram_percent":  round(ram.percent, 1),
            "ram_used_gb":  round(ram.used  / (1024**3), 2),
            "ram_total_gb": round(ram.total / (1024**3), 2),
            "temp_celsius": round(temp, 1)
        }),
        headers={"Access-Control-Allow-Origin": "*"}
    )

async def on_shutdown(app):
    coros = [pc.close() for pc in app["node"].pcs]
    await asyncio.gather(*coros)
    app["node"].pcs.clear()

async def start_server(app):
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 8080)
    await site.start()
    return runner

def ros_spin(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

async def main_async(args=None):
    rclpy.init(args=args)
    node = WebRTCServerNode()

    ros_thread = threading.Thread(
        target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        share_dir = get_package_share_directory('plaguebot_vision')
        web_root  = os.path.join(share_dir, 'web')
    except Exception:
        web_root = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), '..', 'web')

    app = web.Application()
    app["node"] = node
    app.on_shutdown.append(on_shutdown)

    app.router.add_post("/offer",             offer)
    app.router.add_get("/snapshot/realsense", snapshot_realsense)
    app.router.add_get("/snapshot/kiyo",      snapshot_kiyo)
    app.router.add_get("/snapshot/lidar",     snapshot_lidar)
    app.router.add_get("/metrics",            metrics)

    async def index(request):
        return web.FileResponse(os.path.join(web_root, 'index.html'))
    app.router.add_get("/", index)
    app.router.add_static("/", web_root, follow_symlinks=True)

    runner = await start_server(app)
    print("[Server] Puerto 8080 listo")
    print("[Server] Endpoints: /snapshot/realsense  /snapshot/kiyo"
          "  /snapshot/lidar  /metrics")

    try:
        while rclpy.ok():
            await asyncio.sleep(0.1)
    finally:
        await runner.cleanup()
        node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()