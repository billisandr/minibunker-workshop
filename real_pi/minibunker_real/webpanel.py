#!/usr/bin/env python3
# ============================================================================
#  webpanel.py — tiny Flask control panel for the native stack (optional).
#
#  Baked into run.py (run with --web): a daemon-thread Flask server that shares
#  the SAME Controls object as the keyboard, so the panel is just another input
#  to the one control loop — there is never a second motion owner on CAN.
#
#  Endpoints (POSTs use query params so they stay CORS "simple" — no preflight,
#  which lets the page be served from the LAPTOP and still hit the Pi's API):
#     GET  /                -> the panel page (real_pi/panel/index.html)
#     GET  /api/state       -> JSON telemetry snapshot
#     GET  /stream.mjpg     -> MJPEG of the annotated frames
#     POST /api/arm?armed=1     -> ARM / DISARM
#     POST /api/teleop?key=w    -> w/s/j/l/x nudge (auto-expires via watchdog)
#     POST /api/mission?follow=ball  -> none|ball|cone live override
#
#  Two deployment modes (same page, both supported):
#     • ON THE PI:  run.py --web ; browse http://raspberrypi2.local:8080
#     • ON A LAPTOP: serve real_pi/panel/ from the laptop and point the page's
#       "API base" field at http://raspberrypi2.local:8080 (CORS is open).
#
#  Flask is an OPTIONAL dependency — only imported when --web is used.
# ============================================================================
from __future__ import annotations

import os
import threading
import time

PANEL_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "panel"))


class Telemetry:
    """Thread-safe handoff of the latest snapshot dict + annotated JPEG."""

    def __init__(self, jpeg_quality=70):
        self._lock = threading.Lock()
        self._snap = {"state": "BOOT", "armed": False, "can": False}
        self._jpeg = None
        self._mask = None
        self._q = int(jpeg_quality)

    @staticmethod
    def _encode(img, q):
        import cv2
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, q])
        return buf.tobytes() if ok else None

    def update(self, snap, annotated_bgr):
        jpeg = self._encode(annotated_bgr, self._q) if annotated_bgr is not None else None
        with self._lock:
            self._snap = snap
            if jpeg is not None:
                self._jpeg = jpeg

    def update_mask(self, mask):
        if mask is None:
            return
        jpeg = self._encode(mask, 60)
        if jpeg is not None:
            with self._lock:
                self._mask = jpeg

    def snapshot(self):
        with self._lock:
            return dict(self._snap)

    def jpeg(self):
        with self._lock:
            return self._jpeg

    def mask(self):
        with self._lock:
            return self._mask


def _hsv_ranges(hsv_detector):
    """{green_ball:{lower,upper,min_area}, cone:{...}} from the live detector cfg."""
    out = {}
    cfg = getattr(hsv_detector, "cfg", {}) or {}
    for c in ("green_ball", "cone"):
        sub = cfg.get(c, {})
        out[c] = {"lower": list(sub.get("lower", [0, 0, 0])),
                  "upper": list(sub.get("upper", [179, 255, 255])),
                  "min_area": int(sub.get("min_area", 500))}
    return out


def create_app(controls, telemetry, hsv_detector=None, dist_est=None):
    from flask import Flask, Response, jsonify, request, send_from_directory

    app = Flask(__name__, static_folder=None)
    has_hsv = hsv_detector is not None and "green_ball" in getattr(
        hsv_detector, "cfg", {})

    @app.after_request
    def _cors(resp):
        # open CORS so a laptop-served page can call the Pi's API
        resp.headers["Access-Control-Allow-Origin"] = "*"
        resp.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
        resp.headers["Access-Control-Allow-Headers"] = "Content-Type"
        return resp

    @app.route("/")
    def index():
        return send_from_directory(PANEL_DIR, "index.html")

    @app.route("/api/state")
    def state():
        return jsonify(telemetry.snapshot())

    @app.route("/api/arm", methods=["POST", "OPTIONS"])
    def arm():
        if request.method == "OPTIONS":
            return ("", 204)
        armed = request.args.get("armed", "0") in ("1", "true", "True")
        controls.arm() if armed else controls.disarm()
        return jsonify(ok=True, armed=controls.armed)

    @app.route("/api/teleop", methods=["POST", "OPTIONS"])
    def teleop():
        if request.method == "OPTIONS":
            return ("", 204)
        controls.nudge(request.args.get("key", "x"))
        return jsonify(ok=True)

    @app.route("/api/mission", methods=["POST", "OPTIONS"])
    def mission():
        if request.method == "OPTIONS":
            return ("", 204)
        follow = request.args.get("follow", "none")
        controls.set_follow(follow if follow in ("none", "ball", "cone") else "none")
        return jsonify(ok=True, follow=controls.follow_override)

    def _mjpeg(getter):
        def gen():
            while True:
                jpg = getter()
                if jpg is not None:
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\n"
                           b"Content-Length: " + str(len(jpg)).encode() +
                           b"\r\n\r\n" + jpg + b"\r\n")
                time.sleep(1.0 / 15.0)
        return Response(gen(),
                        mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.route("/stream.mjpg")
    def stream():
        return _mjpeg(telemetry.jpeg)

    @app.route("/mask.mjpg")
    def mask_stream():
        return _mjpeg(telemetry.mask)

    # -- HSV calibration (only meaningful with the HSV backend) -------------
    @app.route("/api/maskclass", methods=["POST", "OPTIONS"])
    def maskclass():
        if request.method == "OPTIONS":
            return ("", 204)
        cls = request.args.get("cls", "green_ball")
        controls.set_mask_class(cls if cls in ("green_ball", "cone") else "green_ball")
        return jsonify(ok=True, mask_class=controls.mask_class)

    @app.route("/api/hsv", methods=["GET", "POST", "OPTIONS"])
    def hsv():
        if request.method == "OPTIONS":
            return ("", 204)
        if not has_hsv:
            return jsonify(ok=False, error="HSV backend not active"), 400
        if request.method == "GET":
            return jsonify(ranges=_hsv_ranges(hsv_detector),
                           mask_class=controls.mask_class)
        cls = request.args.get("cls", "green_ball")
        if cls not in ("green_ball", "cone"):
            return jsonify(ok=False, error="bad cls"), 400
        sub = hsv_detector.cfg.setdefault(cls, {})
        cur_l = list(sub.get("lower", [0, 0, 0]))
        cur_u = list(sub.get("upper", [179, 255, 255]))

        def gi(key, default):
            v = request.args.get(key)
            return int(v) if v is not None else default
        sub["lower"] = [gi("hl", cur_l[0]), gi("sl", cur_l[1]), gi("vl", cur_l[2])]
        sub["upper"] = [gi("hu", cur_u[0]), gi("su", cur_u[1]), gi("vu", cur_u[2])]
        sub["min_area"] = gi("min_area", sub.get("min_area", 500))
        controls.set_mask_class(cls)
        return jsonify(ok=True, ranges=_hsv_ranges(hsv_detector))

    # -- pixel distance calibration ----------------------------------------
    @app.route("/api/distance", methods=["GET", "POST", "OPTIONS"])
    def distance():
        if request.method == "OPTIONS":
            return ("", 204)
        if dist_est is None:
            return jsonify(ok=False, error="distance estimator not active"), 400
        if request.method == "GET":
            return jsonify(refs=dist_est.refs())
        cls = request.args.get("cls", "green_ball")
        if cls not in ("green_ball", "cone"):
            return jsonify(ok=False, error="bad cls"), 400
        try:
            dist_m = float(request.args.get("distance_m", "1.0"))
            h_px = int(request.args.get("height_px", "0"))
        except ValueError:
            return jsonify(ok=False, error="bad params"), 400
        if h_px <= 0:
            return jsonify(ok=False, error="no detection (height_px=0) — get the "
                           "object in view first"), 400
        dist_est.calibrate(cls, dist_m, h_px)
        return jsonify(ok=True, refs=dist_est.refs())

    return app


def start_web(controls, telemetry, host="0.0.0.0", port=8080, hsv_detector=None,
              dist_est=None):
    """Launch Flask in a daemon thread. Returns immediately."""
    try:
        app = create_app(controls, telemetry, hsv_detector=hsv_detector,
                         dist_est=dist_est)
    except ImportError as exc:  # pragma: no cover
        print(f"[webpanel] Flask not installed ({exc}). `pip install flask` to "
              "enable --web. Continuing without the panel.")
        return None

    def _serve():
        # threaded so the MJPEG stream doesn't block the API; no reloader (thread)
        app.run(host=host, port=port, threaded=True, debug=False,
                use_reloader=False)

    t = threading.Thread(target=_serve, daemon=True)
    t.start()
    return t
