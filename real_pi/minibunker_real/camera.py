#!/usr/bin/env python3
# ============================================================================
#  camera.py — native (no-ROS) frame source for the Pi stack.
#
#  Mirrors pi_camera_node.py's capture logic without rospy/cv_bridge. Sources
#  (config camera/source):
#     picamera   -> picamera2 (libcamera); falls back to V4L2 if unavailable
#     webcam     -> V4L2 cv2.VideoCapture on camera/v4l2_device
#     video:/p   -> loop a video file (hardware-free bring-up / demo clip)
#     synthetic  -> a generated frame with a green ball + orange cone (no camera)
#
#  All sources yield BGR uint8 frames via read() -> (ok, frame), so the detector
#  is identical across them.
# ============================================================================
from __future__ import annotations

import cv2
import numpy as np


class Camera:
    def __init__(self, cfg):
        self.w = int(cfg.get("camera/width", 640))
        self.h = int(cfg.get("camera/height", 480))
        self.flip = bool(cfg.get("camera/flip_horizontal", False))
        self.source = str(cfg.get("camera/source", "picamera"))
        self.device = cfg.get("camera/v4l2_device", "/dev/video0")
        # picamera2 "RGB888" already returns a BGR-ordered array (libcamera fourcc
        # names are byte-reversed), which is what OpenCV/our HSV want — so by
        # DEFAULT we do NOT swap. Set camera/picam_swap_rb: true only if your
        # build hands back true RGB (reds and blues look swapped otherwise).
        self.picam_swap_rb = bool(cfg.get("camera/picam_swap_rb", False))
        # white-balance: leave AWB on (default) for auto colour; or disable it and
        # pin manual red/blue gains [r, b] for a stable colour under arena light.
        self.awb_enable = bool(cfg.get("camera/awb_enable", True))
        self.colour_gains = cfg.get("camera/colour_gains", None)  # [r, b] or None
        self._picam = None
        self._cap = None
        self._t = 0
        self._open()

    # -- open the chosen source ---------------------------------------------
    def _open(self):
        if self.source == "synthetic":
            print("[camera] synthetic source (no hardware)")
            return
        if self.source.startswith("video:"):
            path = self.source.split(":", 1)[1]
            self._cap = cv2.VideoCapture(path)
            if not self._cap.isOpened():
                raise RuntimeError(f"[camera] cannot open video file: {path}")
            print(f"[camera] video file: {path}")
            return
        if self.source == "picamera":
            self._picam = self._try_picamera2()
            if self._picam is not None:
                return
            print("[camera] picamera2 unavailable; falling back to V4L2")
        # webcam, or picamera->V4L2 fallback
        self._cap = cv2.VideoCapture(self.device)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        if not self._cap.isOpened():
            raise RuntimeError(f"[camera] cannot open V4L2 device: {self.device}")
        print(f"[camera] V4L2 device: {self.device}")

    def _try_picamera2(self):
        try:
            from picamera2 import Picamera2
            cam = Picamera2()
            cam.configure(cam.create_video_configuration(
                main={"size": (self.w, self.h), "format": "RGB888"}))
            cam.start()
            # white-balance controls (applied live after start)
            try:
                ctrls = {"AwbEnable": self.awb_enable}
                if not self.awb_enable and self.colour_gains:
                    ctrls["ColourGains"] = (float(self.colour_gains[0]),
                                            float(self.colour_gains[1]))
                cam.set_controls(ctrls)
            except Exception as exc:  # noqa: BLE001 — controls are best-effort
                print(f"[camera] AWB controls not applied: {exc}")
            print(f"[camera] picamera2 (libcamera); swap_rb={self.picam_swap_rb} "
                  f"awb={self.awb_enable}")
            return cam
        except Exception as exc:  # noqa: BLE001
            print(f"[camera] picamera2 init failed: {exc}")
            return None

    # -- read one BGR frame --------------------------------------------------
    def read(self):
        if self._picam is not None:
            # "RGB888" already arrives BGR-ordered -> use as-is by default
            frame = self._picam.capture_array()
            if self.picam_swap_rb:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif self._cap is not None:
            ok, frame = self._cap.read()
            if not ok:
                # loop a finite video file so the demo source never ends
                if self.source.startswith("video:"):
                    self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ok, frame = self._cap.read()
                if not ok:
                    return False, None
        else:
            frame = self._synthetic()
        if self.flip:
            frame = cv2.flip(frame, 1)
        return True, frame

    def _synthetic(self):
        """A drifting green ball + a static orange cone, so the whole pipeline
        (detect -> FSM -> CAN) runs with no camera at all."""
        img = np.full((self.h, self.w, 3), (40, 40, 50), np.uint8)
        self._t += 1
        cx = int(self.w * (0.5 + 0.25 * np.sin(self._t / 30.0)))
        cy = int(self.h * 0.55)
        cv2.circle(img, (cx, cy), 40, (40, 220, 40), -1)        # green ball (BGR)
        cone_x = int(self.w * 0.78)
        pts = np.array([[cone_x, 180], [cone_x - 35, 300], [cone_x + 35, 300]])
        cv2.fillPoly(img, [pts], (40, 130, 240))                # orange cone (BGR)
        return img

    def close(self):
        if self._picam is not None:
            try:
                self._picam.stop()
            except Exception:  # noqa: BLE001
                pass
        if self._cap is not None:
            self._cap.release()
