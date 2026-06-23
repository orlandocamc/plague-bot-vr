"""Pluggable inference backends for the Detection node (see ADR-0003).

Each backend implements ``infer(image_bgr) -> list[RawDetection]``. The heavy
dependencies (torch/ultralytics, ncnn, hailo) are imported lazily inside each
backend's constructor, so selecting ``mock`` never pulls them in. This is what
lets the Mission state machine run end-to-end in simulation with no model.
"""

from dataclasses import dataclass


@dataclass
class RawDetection:
    """A 2D detection before depth reprojection."""
    class_name: str
    confidence: float
    bbox: tuple  # (x_min, y_min, x_max, y_max) in image pixels

    @property
    def center(self):
        x1, y1, x2, y2 = self.bbox
        return (int((x1 + x2) / 2), int((y1 + y2) / 2))


class Backend:
    """Inference backend interface."""

    def infer(self, image_bgr):
        raise NotImplementedError


class MockBackend(Backend):
    """Returns one synthetic Detection centered in the frame.

    Used in simulation, where the greenhouse plants are plain foliage boxes with
    no disease texture for a real model to find. The 3D position is filled in by
    the node from the depth cloud at the bbox center, so the Mission's IK step
    gets a real point to aim at.
    """

    def __init__(self, class_name='pest', confidence=0.99, logger=None):
        self._class_name = class_name
        self._confidence = confidence
        if logger:
            logger.warn('perception backend=mock: emitting synthetic detections')

    def infer(self, image_bgr):
        h, w = (image_bgr.shape[0], image_bgr.shape[1]) if image_bgr is not None \
            else (480, 640)
        cx, cy = w // 2, h // 2
        half = min(w, h) // 8
        bbox = (cx - half, cy - half, cx + half, cy + half)
        return [RawDetection(self._class_name, self._confidence, bbox)]


class _UltralyticsBackend(Backend):
    """Shared base for the PyTorch and NCNN Ultralytics paths.

    ``best.pt`` and the NCNN export are both loaded through the same Ultralytics
    ``YOLO`` API; only the model path differs (a ``.pt`` file vs a ``_ncnn_model``
    directory). Ultralytics dispatches to the right runtime by extension.
    """

    def __init__(self, model_path, confidence, logger=None):
        from ultralytics import YOLO  # lazy: only when this backend is chosen
        self._model = YOLO(model_path)
        self._conf = confidence
        if logger:
            logger.info(f'perception backend loaded model: {model_path}')

    def infer(self, image_bgr):
        if image_bgr is None:
            return []
        results = self._model.predict(image_bgr, conf=self._conf, verbose=False)
        dets = []
        for r in results:
            names = r.names
            for box in r.boxes:
                cls_id = int(box.cls[0])
                xyxy = box.xyxy[0].tolist()
                dets.append(RawDetection(
                    class_name=names.get(cls_id, str(cls_id)),
                    confidence=float(box.conf[0]),
                    bbox=tuple(int(v) for v in xyxy),
                ))
        return dets


class TorchBackend(_UltralyticsBackend):
    """Ultralytics PyTorch inference on best.pt (dev/sim CPU)."""


class NcnnBackend(_UltralyticsBackend):
    """Ultralytics NCNN inference on best_ncnn_model (dev/sim CPU)."""


class HailoBackend(Backend):
    """HailoRT inference on the compiled best.hef (real robot Raspberry Pi).

    Stubbed until run on the Pi: the HailoRT API and the model's exact
    pre/post-processing (input shape, NMS) are filled in against the real .hef.
    """

    def __init__(self, model_path, confidence, logger=None):
        self._conf = confidence
        self._model_path = model_path
        if logger:
            logger.warn('perception backend=hailo is a stub; implement on the Pi '
                        f'against {model_path}')
        # from hailo_platform import (HEF, VDevice, ...)  # implement on the Pi

    def infer(self, image_bgr):
        raise NotImplementedError(
            'HailoBackend.infer must be implemented on the Raspberry Pi against '
            f'the compiled model {self._model_path}')


def make_backend(name, model_path='', confidence=0.5, logger=None):
    """Factory: build a backend by name (see ADR-0003)."""
    name = (name or 'mock').lower()
    if name == 'mock':
        return MockBackend(logger=logger)
    if name == 'torch':
        return TorchBackend(model_path, confidence, logger=logger)
    if name == 'ncnn':
        return NcnnBackend(model_path, confidence, logger=logger)
    if name == 'hailo':
        return HailoBackend(model_path, confidence, logger=logger)
    raise ValueError(f"unknown perception backend '{name}' "
                     "(expected: mock | torch | ncnn | hailo)")
