#!/usr/bin/env python3
# ============================================================================
#  detector.py — perception backends, lifted from the ROS detector_node so sim
#  and real share the SAME detection maths. rospy/cv_bridge stripped; the
#  HsvDetector / CnnDetector classes are otherwise byte-for-byte the sim logic.
#
#  detect(bgr) -> [(cls, x, y, w, h, score), ...] for both backends.
#  Class indices are fixed by config order [green_ball, cone].
# ============================================================================
from __future__ import annotations

import cv2
import numpy as np

CLASS_GREEN_BALL = 0
CLASS_CONE = 1

NAME_TO_CLASS = {
    "ball": CLASS_GREEN_BALL,
    "green_ball": CLASS_GREEN_BALL,
    "cone": CLASS_CONE,
}
LABELS = {CLASS_GREEN_BALL: "green_ball", CLASS_CONE: "cone"}
COLOURS = {CLASS_GREEN_BALL: (0, 255, 0), CLASS_CONE: (0, 140, 255)}


# ---------------------------------------------------------------------------
#  HSV colour thresholding (the v0 baseline, two colours) — verbatim from the
#  ROS HsvDetector; see detector_node.py for the close-before-open rationale.
# ---------------------------------------------------------------------------
class HsvDetector:
    def __init__(self, cfg):
        self.cfg = cfg

    @staticmethod
    def _ksize(spec, default):
        if isinstance(spec, (list, tuple)) and len(spec) == 2:
            kh, kw = int(spec[0]), int(spec[1])
        else:
            kh = kw = int(spec if spec is not None else default)
        return max(1, kh), max(1, kw)

    def _detect_colour(self, hsv, sub):
        lower = np.array(sub["lower"], dtype=np.uint8)
        upper = np.array(sub["upper"], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        ch, cw = self._ksize(sub.get("close_ksize"), 5)
        oh, ow = self._ksize(sub.get("open_ksize"), 5)
        if ch > 1 or cw > 1:
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_CLOSE, np.ones((ch, cw), np.uint8))
        if oh > 1 or ow > 1:
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN, np.ones((oh, ow), np.uint8))
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < sub["min_area"]:
                continue
            x, y, w, h = cv2.boundingRect(c)
            score = float(min(1.0, area / float(w * h + 1e-6)))
            boxes.append((x, y, w, h, score))
        return boxes, mask

    def detect(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        dets = []
        gb, gmask = self._detect_colour(hsv, self.cfg["green_ball"])
        for (x, y, w, h, s) in gb:
            dets.append((CLASS_GREEN_BALL, x, y, w, h, s))
        cn, cmask = self._detect_colour(hsv, self.cfg["cone"])
        for (x, y, w, h, s) in cn:
            dets.append((CLASS_CONE, x, y, w, h, s))
        return dets, cv2.bitwise_or(gmask, cmask)


# ---------------------------------------------------------------------------
#  CNN backend (YOLOv8-nano via onnxruntime / ultralytics). Optional — if the
#  runtime or weights are missing it returns empty detections, never crashes.
# ---------------------------------------------------------------------------
class CnnDetector:
    def __init__(self, cfg):
        self.cfg = cfg
        self.runtime = cfg.get("runtime", "onnxruntime")
        self.conf = float(cfg.get("conf_threshold", 0.45))
        self.iou = float(cfg.get("iou_threshold", 0.50))
        self.imgsz = int(cfg.get("input_size", 416))
        self.weights = cfg.get("weights", "")
        self.n_classes = len(cfg.get("class_names", ["green_ball", "cone"]))
        self._impl = None
        self._load()

    def _load(self):
        try:
            if not self.weights:
                raise RuntimeError("no weights configured")
            if self.runtime in ("ultralytics", "ncnn"):
                from ultralytics import YOLO
                self._impl = ("ultralytics", YOLO(self.weights))
            else:
                import onnxruntime as ort
                self._impl = ("onnx", ort.InferenceSession(
                    self.weights, providers=["CPUExecutionProvider"]))
            print(f"[detector] CNN ready ({self.runtime}): {self.weights}")
        except Exception as exc:  # noqa: BLE001
            print(f"[detector] CNN unavailable ({self.runtime}): {exc} "
                  "-> empty detections")
            self._impl = None

    def _decode_onnx(self, output, scale, padw, padh):
        preds = np.squeeze(output[0]).T
        boxes, scores, classes = [], [], []
        for row in preds:
            cls_scores = row[4:4 + self.n_classes]
            cid = int(np.argmax(cls_scores))
            conf = float(cls_scores[cid])
            if conf < self.conf:
                continue
            cx, cy, w, h = row[0], row[1], row[2], row[3]
            x = (cx - w / 2 - padw) / scale
            y = (cy - h / 2 - padh) / scale
            boxes.append([int(x), int(y), int(w / scale), int(h / scale)])
            scores.append(conf)
            classes.append(cid)
        dets = []
        if boxes:
            idxs = cv2.dnn.NMSBoxes(boxes, scores, self.conf, self.iou)
            for i in np.array(idxs).flatten():
                x, y, w, h = boxes[i]
                dets.append((classes[i], x, y, w, h, scores[i]))
        return dets

    def _letterbox(self, bgr):
        h0, w0 = bgr.shape[:2]
        scale = min(self.imgsz / w0, self.imgsz / h0)
        nw, nh = int(round(w0 * scale)), int(round(h0 * scale))
        resized = cv2.resize(bgr, (nw, nh))
        canvas = np.full((self.imgsz, self.imgsz, 3), 114, dtype=np.uint8)
        padw, padh = (self.imgsz - nw) // 2, (self.imgsz - nh) // 2
        canvas[padh:padh + nh, padw:padw + nw] = resized
        return canvas, scale, padw, padh

    def detect(self, bgr):
        if self._impl is None:
            return [], None
        kind, model = self._impl
        if kind == "ultralytics":
            res = model.predict(bgr, imgsz=self.imgsz, conf=self.conf,
                                iou=self.iou, verbose=False)[0]
            dets = []
            for b in res.boxes:
                x1, y1, x2, y2 = b.xyxy[0].tolist()
                dets.append((int(b.cls[0]), int(x1), int(y1),
                             int(x2 - x1), int(y2 - y1), float(b.conf[0])))
            return dets, None
        canvas, scale, padw, padh = self._letterbox(bgr)
        blob = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        out = model.run(None, {model.get_inputs()[0].name: blob})
        return self._decode_onnx(out, scale, padw, padh), None


def make_detector(cfg):
    """cfg = the `detector` config block. Returns (backend_name, detector)."""
    backend = str(cfg.get("backend", "hsv"))
    if backend == "cnn":
        return "cnn", CnnDetector(cfg.get("cnn", {}))
    return "hsv", HsvDetector(cfg.get("hsv", {}))
