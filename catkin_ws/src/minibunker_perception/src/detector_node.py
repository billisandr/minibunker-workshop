#!/usr/bin/env python3
# ============================================================================
#  detector_node.py  —  the perception heart of the MiniBunker station
#
#  Subscribes : /camera/image_raw            (sensor_msgs/Image)
#  Publishes  : /minibunker/detections       (vision_msgs/Detection2DArray)
#               /minibunker/debug_image       (sensor_msgs/Image, annotated)
#               /minibunker/perception_state  (std_msgs/Float32MultiArray)
#               /minibunker/hsv_mask          (sensor_msgs/Image, HSV mode only)
#
#  Two interchangeable backends behind ONE topic contract (plan.md §4.3):
#     detector/backend: cnn   -> YOLOv8-nano via onnxruntime | ultralytics | ncnn
#     detector/backend: hsv   -> the v0 colour-threshold loop, two colours
#  behavior_node never knows which one ran. The backend is re-read every frame
#  so the Streamlit UI can flip it live.
#
#  perception_state Float32MultiArray layout (shared with behavior_node):
#     [0] target_seen      (0/1)   green ball present
#     [1] target_cx_norm   (-1..1) ball centre x, 0 = image centre, + = right
#     [2] target_cy_norm   (-1..1) ball centre y, + = down
#     [3] target_h_frac    (0..1)  ball bbox height / image height  (distance proxy)
#     [4] cone_seen        (0/1)
#     [5] cone_danger      (0/1)   a cone is big + low-centre (in the danger zone)
#     [6] cone_cx_norm     (-1..1) nearest cone centre x
# ============================================================================
import threading

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

# Class indices are fixed by the config order [green_ball, cone].
CLASS_GREEN_BALL = 0
CLASS_CONE = 1


# ---------------------------------------------------------------------------
#  Backend: classic HSV colour thresholding (the v0 baseline, two colours)
# ---------------------------------------------------------------------------
class HsvDetector:
    """Recreates Exercise_MiniBunker2_v0's cv2.inRange loop for two colours."""

    def __init__(self, cfg):
        self.cfg = cfg

    def _detect_colour(self, hsv, sub):
        lower = np.array(sub["lower"], dtype=np.uint8)
        upper = np.array(sub["upper"], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < sub["min_area"]:
                continue
            x, y, w, h = cv2.boundingRect(c)
            # score ~ how filled the bbox is (a crude confidence proxy)
            score = float(min(1.0, area / float(w * h + 1e-6)))
            boxes.append((x, y, w, h, score))
        return boxes, mask

    def detect(self, bgr):
        """Returns (detections, mask) where detections = [(cls, x, y, w, h, score)]."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        dets = []
        gb, gmask = self._detect_colour(hsv, self.cfg["green_ball"])
        for (x, y, w, h, s) in gb:
            dets.append((CLASS_GREEN_BALL, x, y, w, h, s))
        cn, cmask = self._detect_colour(hsv, self.cfg["cone"])
        for (x, y, w, h, s) in cn:
            dets.append((CLASS_CONE, x, y, w, h, s))
        mask = cv2.bitwise_or(gmask, cmask)
        return dets, mask


# ---------------------------------------------------------------------------
#  Backend: CNN (YOLOv8-nano). Runtime is selectable; all paths return the same
#  (cls, x, y, w, h, score) tuples so the ROS wrapper is runtime-agnostic.
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
            if self.runtime == "ultralytics":
                from ultralytics import YOLO
                self._impl = ("ultralytics", YOLO(self.weights))
            elif self.runtime == "ncnn":
                # ncnn is loaded through ultralytics' exported NCNN folder
                from ultralytics import YOLO
                self._impl = ("ultralytics", YOLO(self.weights))
            else:  # onnxruntime (default)
                import onnxruntime as ort
                sess = ort.InferenceSession(
                    self.weights, providers=["CPUExecutionProvider"]
                )
                self._impl = ("onnx", sess)
            rospy.loginfo("[detector] CNN backend ready (%s): %s",
                          self.runtime, self.weights)
        except Exception as exc:  # noqa: BLE001 — never crash the stack
            rospy.logwarn("[detector] CNN backend unavailable (%s). "
                          "Falling back to empty detections until a model is "
                          "present. Reason: %s", self.runtime, exc)
            self._impl = None

    # -- YOLOv8 ONNX decode (output [1, 4+nc, N]) with cv2 NMS ----------------
    def _decode_onnx(self, output, scale, padw, padh):
        preds = np.squeeze(output[0]).T  # -> (N, 4+nc)
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
        # onnx
        canvas, scale, padw, padh = self._letterbox(bgr)
        blob = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        out = model.run(None, {model.get_inputs()[0].name: blob})
        return self._decode_onnx(out, scale, padw, padh), None


# ---------------------------------------------------------------------------
#  ROS wrapper
# ---------------------------------------------------------------------------
class DetectorNode:
    COLOURS = {CLASS_GREEN_BALL: (0, 255, 0), CLASS_CONE: (0, 140, 255)}
    LABELS = {CLASS_GREEN_BALL: "green_ball", CLASS_CONE: "cone"}

    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.backend_name = None
        self.hsv = None
        self.cnn = None

        # Danger zone: a cone counts as "danger" if its bbox height fraction is
        # >= cone_danger_frac AND its centre sits in the lower-centre of frame.
        self.cone_danger_frac = float(
            rospy.get_param("behavior/avoid/cone_danger_frac", 0.35))

        self.det_pub = rospy.Publisher(
            "/minibunker/detections", Detection2DArray, queue_size=1)
        self.dbg_pub = rospy.Publisher(
            "/minibunker/debug_image", Image, queue_size=1)
        # Compressed twin: cheap for the Streamlit UI to pull over rosbridge.
        self.dbg_comp_pub = rospy.Publisher(
            "/minibunker/debug_image/compressed", CompressedImage, queue_size=1)
        self.state_pub = rospy.Publisher(
            "/minibunker/perception_state", Float32MultiArray, queue_size=1)
        self.mask_pub = rospy.Publisher(
            "/minibunker/hsv_mask", Image, queue_size=1)

        self.flip = bool(rospy.get_param("camera/flip_horizontal", False))
        self.sub = rospy.Subscriber(
            "/camera/image_raw", Image, self.on_image, queue_size=1,
            buff_size=2 ** 24)
        rospy.loginfo("[detector] up; waiting for /camera/image_raw")

    # -- lazy / live backend selection ---------------------------------------
    def _ensure_backend(self):
        want = rospy.get_param("detector/backend", "hsv")
        if want == self.backend_name:
            return want
        if want == "cnn":
            self.cnn = CnnDetector(rospy.get_param("detector/cnn", {}))
        else:
            self.hsv = HsvDetector(rospy.get_param("detector/hsv", {}))
        self.backend_name = want
        rospy.loginfo("[detector] backend = %s", want)
        return want

    def on_image(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn_throttle(5.0, "[detector] cv_bridge: %s", exc)
            return
        if self.flip:
            bgr = cv2.flip(bgr, 1)

        backend = self._ensure_backend()
        # Re-read the active backend's "soft" knobs each frame so the Streamlit
        # UI can tune them live without a relaunch (the model itself is never
        # reloaded here — only thresholds / colour ranges).
        self.cone_danger_frac = float(
            rospy.get_param("behavior/avoid/cone_danger_frac", self.cone_danger_frac))
        mask = None
        if backend == "cnn":
            self.cnn.conf = float(
                rospy.get_param("detector/cnn/conf_threshold", self.cnn.conf))
            self.cnn.iou = float(
                rospy.get_param("detector/cnn/iou_threshold", self.cnn.iou))
            dets, _ = self.cnn.detect(bgr)
        else:
            self.hsv.cfg = rospy.get_param("detector/hsv", self.hsv.cfg)
            dets, mask = self.hsv.detect(bgr)

        self._publish(msg.header, bgr, dets, mask)

    def _publish(self, header, bgr, dets, mask):
        h, w = bgr.shape[:2]
        arr = Detection2DArray(header=header)
        annotated = bgr.copy()

        best_ball = None       # largest green ball
        best_cone = None       # largest cone
        for (cls, x, y, bw, bh, score) in dets:
            d = Detection2D(header=header)
            hyp = ObjectHypothesisWithPose()
            hyp.id = int(cls)
            hyp.score = float(score)
            d.results.append(hyp)
            d.bbox.center.x = x + bw / 2.0
            d.bbox.center.y = y + bh / 2.0
            d.bbox.size_x = float(bw)
            d.bbox.size_y = float(bh)
            arr.detections.append(d)

            colour = self.COLOURS.get(cls, (255, 255, 255))
            cv2.rectangle(annotated, (x, y), (x + bw, y + bh), colour, 2)
            cv2.putText(annotated, "%s %.2f" % (self.LABELS.get(cls, "?"), score),
                        (x, max(0, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        colour, 2)
            if cls == CLASS_GREEN_BALL and (best_ball is None or bh > best_ball[3]):
                best_ball = (x, y, bw, bh)
            if cls == CLASS_CONE and (best_cone is None or bh > best_cone[3]):
                best_cone = (x, y, bw, bh)

        # ---- perception_state ----
        st = [0.0] * 7
        if best_ball is not None:
            x, y, bw, bh = best_ball
            st[0] = 1.0
            st[1] = ((x + bw / 2.0) / w) * 2.0 - 1.0
            st[2] = ((y + bh / 2.0) / h) * 2.0 - 1.0
            st[3] = bh / float(h)
        if best_cone is not None:
            x, y, bw, bh = best_cone
            st[4] = 1.0
            cx_norm = ((x + bw / 2.0) / w) * 2.0 - 1.0
            cy_norm = ((y + bh / 2.0) / h)
            big = (bh / float(h)) >= self.cone_danger_frac
            low_centre = cy_norm > 0.45 and abs(cx_norm) < 0.6
            st[5] = 1.0 if (big and low_centre) else 0.0
            st[6] = cx_norm

        self._draw_hud(annotated, st)
        self.det_pub.publish(arr)
        self.state_pub.publish(Float32MultiArray(data=st))
        try:
            self.dbg_pub.publish(
                self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8"))
            ok, buf = cv2.imencode(".jpg", annotated,
                                   [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                cmsg = CompressedImage(header=header)
                cmsg.format = "jpeg"
                cmsg.data = buf.tobytes()
                self.dbg_comp_pub.publish(cmsg)
            if mask is not None:
                self.mask_pub.publish(
                    self.bridge.cv2_to_imgmsg(mask, encoding="mono8"))
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn_throttle(5.0, "[detector] publish debug: %s", exc)

    def _draw_hud(self, img, st):
        txt = "backend:%s  ball:%d cone:%d danger:%d" % (
            self.backend_name, int(st[0]), int(st[4]), int(st[5]))
        cv2.putText(img, txt, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 2)


def main():
    rospy.init_node("minibunker_detector")
    DetectorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
