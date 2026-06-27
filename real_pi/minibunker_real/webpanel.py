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
        self._q = int(jpeg_quality)

    def update(self, snap, annotated_bgr):
        jpeg = None
        if annotated_bgr is not None:
            import cv2
            ok, buf = cv2.imencode(".jpg", annotated_bgr,
                                   [cv2.IMWRITE_JPEG_QUALITY, self._q])
            if ok:
                jpeg = buf.tobytes()
        with self._lock:
            self._snap = snap
            if jpeg is not None:
                self._jpeg = jpeg

    def snapshot(self):
        with self._lock:
            return dict(self._snap)

    def jpeg(self):
        with self._lock:
            return self._jpeg


def create_app(controls, telemetry):
    from flask import Flask, Response, jsonify, request, send_from_directory

    app = Flask(__name__, static_folder=None)

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

    @app.route("/stream.mjpg")
    def stream():
        def gen():
            boundary = b"--frame"
            while True:
                jpg = telemetry.jpeg()
                if jpg is not None:
                    yield (boundary + b"\r\nContent-Type: image/jpeg\r\n"
                           b"Content-Length: " + str(len(jpg)).encode() +
                           b"\r\n\r\n" + jpg + b"\r\n")
                time.sleep(1.0 / 15.0)
        return Response(gen(),
                        mimetype="multipart/x-mixed-replace; boundary=frame")

    return app


def start_web(controls, telemetry, host="0.0.0.0", port=8080):
    """Launch Flask in a daemon thread. Returns immediately."""
    try:
        app = create_app(controls, telemetry)
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
